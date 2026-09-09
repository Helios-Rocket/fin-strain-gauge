use core::sync::atomic::Ordering;
use defmt::{error, info, println};
use hal::access_global;
use hal::instant::Instant;
use hal::pac::TIM1;

use crate::adc::{self, ADC};
use crate::flash::{self, WinbondFlash};
use crate::statemachine::FinState;
use crate::{COMMAND_READY, PULSE_READY, RX_BUF, RX_LEN, UART};
use hal::{
    flash::{Bank, Flash},
    gpio::{Edge, Pin, PinMode},
    timer::Timer,
    usart::UsartInterrupt,
};
use shared::fin_commands::FinCommands;

pub struct StateHandler {
    adc: ADC,
    winbond_flash: WinbondFlash,
    internal_flash: Flash,
    timer: Timer<TIM1>,
    timer_start: Instant,
    recording_start: Option<Instant>,
    pb10: Pin,
    pb11: Pin,
    record_buf: [u32; 512],
    record_idx: usize,
    page: u32,
}

impl StateHandler {
    pub fn new(
        adc: ADC,
        winbond_flash: WinbondFlash,
        internal_flash: Flash,
        mut timer: Timer<TIM1>,
        timer_start: Instant,
        pb10: Pin,
        pb11: Pin,
        flight_flag: bool,
    ) -> Self {
        let page = if flight_flag {
            let mut meta = [0u8; 4];
            internal_flash.read(Bank::B1, 31, 1, &mut meta);
            u32::from_ne_bytes(meta[0..4].try_into().unwrap())
        } else {
            0
        };
        let recording_start = if flight_flag { Some(timer.now()) } else { None };

        Self {
            adc,
            winbond_flash,
            internal_flash,
            timer,
            timer_start,
            recording_start,
            pb10,
            pb11,
            record_buf: [0u32; 512],
            record_idx: 0,
            page,
        }
    }

    fn persist_status(&mut self, recording: bool) {
        let mut flash_data = [0u8; 5];
        flash_data[0] = recording as u8;
        flash_data[1..5].copy_from_slice(&self.page.to_ne_bytes());

        let _ = self.internal_flash.unlock();
        let _ = self
            .internal_flash
            .erase_write_page(Bank::B1, 31, &flash_data)
            .inspect_err(|e| error!("Failed to write to internal flash: {}", e));
        let _ = self.internal_flash.lock();
    }

    pub fn handle_wait_for_command(&mut self) -> Option<FinState> {
        println!("Entered Wait for Command Handler");

        // Make sure reception is armed and we're not carrying over a stale partial
        // count from a previous state (e.g. WaitForPulse disables these).
        critical_section::with(|cs| {
            access_global!(UART, uart, cs);
            let _ = uart.enable_interrupt(UsartInterrupt::Idle);
            let _ = uart.enable_interrupt(UsartInterrupt::ReadNotEmpty);
            RX_LEN.borrow(cs).set(0);
        });

        while !COMMAND_READY.swap(false, Ordering::AcqRel) {
            cortex_m::asm::wfi();
        }

        // Bytes were already captured one-at-a-time by the USART3 ISR; just read
        // out the completed frame it assembled.
        let command = critical_section::with(|cs| RX_BUF.borrow(cs).get());

        println!("Received Command: {}", command);
        //println!("Received Command: {}", command[3]);

        if &command[0..3] == b"FIN" {
            match FinCommands::try_from(command[3]).unwrap() {
                FinCommands::RecordData => Some(FinState::WaitForRecordPulse),
                FinCommands::EraseFlash => Some(FinState::EraseFlash),
                _ => None,
            }
        } else {
            None
        }
    }

    pub fn handle_wait_for_pulse(&mut self) -> Option<FinState> {
        println!("Entered wait for pulse handler");

        critical_section::with(|cs| {
            access_global!(UART, uart, cs);
            uart.disable_interrupt(UsartInterrupt::Idle);
            uart.disable_interrupt(UsartInterrupt::ReadNotEmpty);
        });
        self.pb10.mode(PinMode::Output);
        self.pb11.mode(PinMode::Input);
        self.pb11.enable_interrupt(Edge::Falling);

        while !PULSE_READY.swap(false, Ordering::AcqRel) {
            cortex_m::asm::wfi();
        }

        self.pb11.clear_interrupt();
        self.pb11.enable_interrupt(Edge::Rising);

        self.recording_start = Some(self.timer.now());

        Some(FinState::RecordData)
    }

    pub fn handle_record_data(&mut self) -> Option<FinState> {
        //println!("Entered Record Data Handler");

        if PULSE_READY.swap(false, Ordering::AcqRel) {
            return Some(FinState::StopRecord);
        }

        match self.adc.read_adc_data() {
            Ok(samples) => {
                // Record sample time

                self.record_buf[self.record_idx] =
                    self.timer.elapsed(self.timer_start).as_millis() as u32;
                self.record_idx += 1;

                //println!("ADC Reading: {}", samples);

                // Record Sample
                for channel in samples {
                    // Record time as well?
                    self.record_buf[self.record_idx] = (channel as f32).to_bits();
                    self.record_idx += 1;
                    // println!("Idx: {}", self.record_idx);
                    // println!("Buff: {}", self.record_buf);

                    if self.record_idx == self.record_buf.len() {
                        println!("Writing to Flash");
                        self.winbond_flash.write_page(self.record_buf); // Have this return an error
                        self.record_idx = 0;
                        self.page += 1;
                        // critical_section::with(|cs| {
                        //     access_global!(UART, uart, cs);
                        //     uart.write(&[1]).ok();
                        // });
                        println!("Wrote to Flash");
                        self.persist_status(true);
                    }
                }
                None
            }

            Err(adc::Error::CRC { computed }) => {
                // TODO: Don't go to error state
                error!("Got CRC Error {}", computed);
                Some(FinState::Error)
            }
        }
    }

    pub fn handle_stop_recording(&mut self) -> Option<FinState> {
        println!("Stop recording handler entered");
        // Finish recording final buffer
        self.winbond_flash.write_page(self.record_buf);
        self.page += 1;

        // Set flight flag to off
        self.persist_status(false);

        Some(FinState::WaitForCommand)
    }

    pub fn handle_erase_flash(&mut self) -> Option<FinState> {
        println!("Erase flash handler entered");
        match self.winbond_flash.erase_chip() {
            Ok(()) => {
                critical_section::with(|cs| {
                    access_global!(UART, uart, cs);
                    info!("Success in erasing");
                    let _ = uart.write(b"FIN");
                    let _ = uart.write(&[FinCommands::Success as u8, 0xff]);
                });
                Some(FinState::WaitForCommand)
            }
            Err(e) => {
                critical_section::with(|cs| {
                    access_global!(UART, uart, cs);
                    error!("Failed to erase");

                    let err_str = match e {
                        flash::Error::FailToErase => b"Erase failed",
                        flash::Error::FailToWrite => b"Write failed",
                    };

                    let _ = uart.write(b"FIN");
                    let _ = uart.write(&[FinCommands::Failure as u8]);
                    let _ = uart.write(err_str);
                    uart.write_one(0xff); // Idle signals end of frame
                });

                Some(FinState::WaitForCommand)
            }
        }
    }

    pub fn handle_status_cmd(&mut self) -> Option<FinState> {
        critical_section::with(|cs| {
            access_global!(UART, uart, cs);
            uart.write(b"A").unwrap();
            println!("Sending UART Msg");
        });
        Some(FinState::WaitForCommand)
    }

    pub fn handle_error(&mut self) -> Option<FinState> {
        // Send error signal over uart
        println!("Error State Entered");
        None
    }
}

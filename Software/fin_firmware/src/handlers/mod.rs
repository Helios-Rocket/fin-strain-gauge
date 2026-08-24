use core::sync::atomic::Ordering;
use core::time::Duration;
use core::{mem, u8};
use cortex_m::{Peripherals, delay, peripheral};
use defmt::{error, info, println};
use hal::instant::Instant;
use hal::pac::TIM1;
use hal::{access_global, timer};

use crate::adc::{self, ADC};
use crate::flash::{self, WinbondFlash};
use crate::statemachine::{Event, FinStateMachine};
use crate::{COMMAND_READY, PULSE_READY, UART};
use hal::{
    delay_ms,
    flash::{Bank, Flash},
    gpio::{Edge, Pin, PinMode},
    pac::{USART3, interrupt},
    timer::Timer,
    usart::{Usart, UsartInterrupt},
};
use shared::fin_commands::FinCommands;
use shared::winbond_flash;

pub struct StateHandler {
    adc: ADC,
    winbond_flash: WinbondFlash,
    internal_flash: Flash,
    timer: Timer<TIM1>,
    timer_start: Instant,
    pb10: Pin,
    pb11: Pin,
    record_buf: [u32; 512],
    record_idx: usize,
    page: u32,
    start_time: Duration,
}

impl StateHandler {
    pub fn new(
        adc: ADC,
        winbond_flash: WinbondFlash,
        internal_flash: Flash,
        timer: Timer<TIM1>,
        timer_start: Instant,
        pb10: Pin,
        pb11: Pin,
        flight_flag: bool,
    ) -> Self {
        let (page, start_time) = if flight_flag {
            let mut meta = [0u8; 9]; // [flag:1][page:4][start_time:4]
            internal_flash.read(Bank::B1, 31, 0, &mut meta);
            (
                u32::from_ne_bytes(meta[1..5].try_into().unwrap()),
                u32::from_ne_bytes(meta[5..9].try_into().unwrap()),
            )
        } else {
            (0, 0)
        };

        println!("Page and start time recorded {}, {}", page, start_time);

        Self {
            adc,
            winbond_flash,
            internal_flash,
            timer,
            timer_start,
            pb10,
            pb11,
            record_buf: [0u32; 512],
            record_idx: 0,
            page,
            start_time: Duration::from_millis(start_time as u64),
        }
    }

    fn persist_status(&mut self, recording: bool) {
        let mut flash_data = [0u8; 21];
        flash_data[0] = recording as u8;
        flash_data[1..5].copy_from_slice(&self.page.to_ne_bytes());
        println!("here");
        flash_data[5..21].copy_from_slice(&self.start_time.as_millis().to_ne_bytes());

        let mut buf: [u8; 9] = [0u8; 9];
        self.internal_flash
            .read(hal::flash::Bank::B1, 31, 0, &mut buf);
        println!("Internal flash flag: {}", buf[0]);
        println!("Internal flash page: {}", buf[1..5]);
        println!("Internal flash time: {}", buf[5..9]);

        self.internal_flash.unlock();
        self.internal_flash
            .erase_write_page(Bank::B1, 31, &flash_data)
            .ok();
        self.internal_flash.lock();
    }

    pub fn handle_wait_for_command(&mut self) -> Event {
        println!("Entered Wait for Command Handler");

        while !COMMAND_READY.load(Ordering::Acquire) {
            cortex_m::asm::wfi();
        }
        COMMAND_READY.store(false, Ordering::Release);

        critical_section::with(|cs| {
            access_global!(UART, uart, cs);
            uart.clear_interrupt(UsartInterrupt::ReadNotEmpty);
        });
        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::USART3);
        }

        let mut command = [0u8; 4];
        critical_section::with(|cs| {
            access_global!(UART, uart, cs);
            uart.read(&mut command);
        });

        println!("Received Command: {}", command[0..3]);
        println!("Received Command: {}", command[3]); 

        if &command[0..3] == b"FIN" {
            match FinCommands::try_from(command[3]).unwrap() {
                FinCommands::RecordData => Event::RecordCommand,
                FinCommands::EraseFlash => Event::EraseCommand,
                _ => Event::Wait,
            }
        } else {
            Event::Wait
        }
    }

    pub fn handle_wait_for_pulse(&mut self) -> Event {
        println!("Entered wait for pulse handler");

        critical_section::with(|cs| {
            access_global!(UART, uart, cs);
            uart.disable_interrupt(UsartInterrupt::ReadNotEmpty);
        });
        self.pb10.mode(PinMode::Output);
        self.pb11.mode(PinMode::Input);
        self.pb11.enable_interrupt(Edge::Falling);
        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::EXTI15_10);
        }

        while !PULSE_READY.load(Ordering::Acquire) {
            cortex_m::asm::wfi();
        }
        PULSE_READY.store(false, Ordering::Release);

        self.pb11.clear_interrupt();
        self.pb11.enable_interrupt(Edge::Rising);

        // let start = self.timer.now();

        // while self.timer.elapsed(start).as_nanos() < 2000000000{
        //     //println!("time {} ", self.timer.elapsed(start).as_nanos());
        // }

        self.start_time = self.timer.elapsed(self.timer_start);
        println!("Recording Started at Time {}", self.start_time.as_millis());

        Event::RecordPulseReceived
    }

    pub fn handle_record_data(&mut self) -> Event {
        //println!("Entered Record Data Handler");

        if PULSE_READY.swap(false, Ordering::Acquire) {
            unsafe {
                cortex_m::peripheral::NVIC::unmask(interrupt::EXTI15_10);
            }
            return Event::StopCommand;
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
                        self.winbond_flash.write_page(self.record_buf);
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
                Event::Wait
            }

            Err(adc::Error::CRC { computed }) => {
                error!("Got CRC Error {}", computed);
                Event::Fail
            }
        }
    }

    pub fn handle_stop_recording(&mut self) -> Event {
        println!("Stop recording handler entered");
        // Finish recording final buffer
        self.winbond_flash.write_page(self.record_buf);
        self.page += 1;

        // Set flight flag to off
        self.persist_status(false);

        Event::Success
    }

    pub fn handle_erase_flash(&mut self) -> Event {
        println!("Erase flash handler entered");
        match self.winbond_flash.erase_chip() {
            Ok(()) => {
                critical_section::with(|cs| {
                    access_global!(UART, uart, cs);
                    uart.write(b"FIN"); 
                    uart.write(&[FinCommands::Success as u8]); 
                });
                Event::Success
            }
            Err(flash::Error::FailToErase) => return Event::Fail,
            Err(flash::Error::FailToWrite) => return Event::Wait, //figure out best way to error handle
        }
    }

    pub fn handle_error(&mut self) -> Event {
        // Send error signal over uart
        Event::Success
    }
}

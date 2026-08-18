use core::sync::atomic::Ordering;
use cortex_m::{Peripherals, peripheral};
use defmt::{error, println};

use crate::adc::{self, ADC};
use crate::flash::WinbondFlash;
use crate::statemachine::{Event, FinStateMachine};
use crate::{COMMAND_READY, PULSE_READY};
use hal::{
    flash::{Bank, Flash},
    gpio::{Edge, Pin, PinMode},
    pac::{USART3, interrupt},
    usart::{Usart, UsartInterrupt},
};
use shared::winbond_flash;

pub struct StateHandler {
    adc: ADC,
    winbond_flash: WinbondFlash,
    internal_flash: Flash,
    usart: Usart<USART3>,
    pb10: Pin,
    pb11: Pin,
    record_buf: [u32; 512],
    record_idx: usize,
    page: u32,
    start_time: u32,
}

impl StateHandler {
    pub fn new(
        adc: ADC,
        winbond_flash: WinbondFlash,
        internal_flash: Flash,
        usart: Usart<USART3>,
        pb10: Pin,
        pb11: Pin,
        flight_flag: bool,
    ) -> Self {
    
        let (page, start_time) = if flight_flag {
            let mut meta = [0u8; 9]; // [flag:1][page:4][start_time:4]
            internal_flash.read(Bank::B1, 0, 0, &mut meta);
            (
                u32::from_le_bytes(meta[1..5].try_into().unwrap()),
                u32::from_le_bytes(meta[5..9].try_into().unwrap()),
            )
        } else {
            (0, 0)
        };

        Self {
            adc,
            winbond_flash,
            internal_flash,
            usart,
            pb10,
            pb11,
            record_buf: [0u32; 512],
            record_idx: 0,
            page,
            start_time,
        }
    }

    fn persist_status(&mut self, recording: bool) {
        let mut flash_data = [0u8; 9];
        flash_data[0] = recording as u8;
        flash_data[1..5].copy_from_slice(&self.page.to_le_bytes());
        flash_data[5..9].copy_from_slice(&self.start_time.to_le_bytes());

        self.internal_flash.unlock();
        self.internal_flash
            .erase_write_page(Bank::B1, 0, &flash_data)
            .ok();
        self.internal_flash.lock();
    }

    pub fn handle_wait_for_command(&mut self) -> Event {
        println!("Entered Wait for Command Handler");

        while !COMMAND_READY.load(Ordering::Acquire) {
            cortex_m::asm::wfi();
        }
        COMMAND_READY.store(false, Ordering::Release);

        self.usart.clear_interrupt(UsartInterrupt::Idle);
        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::USART3);
        }

        // TODO: decode the real command protocol
        let command = self.usart.read_one();

        match command {
            _ => Event::RecordCommand,
        }
    }

    pub fn handle_wait_for_pulse(&mut self) -> Event {
        println!("Entered wait for pulse handler");

        self.usart.disable_interrupt(UsartInterrupt::Idle);
        self.pb10.mode(PinMode::Input);
        self.pb11.mode(PinMode::Input);
        self.pb11.enable_interrupt(Edge::Rising);
        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::EXTI15_10);
        }

        while !PULSE_READY.load(Ordering::Acquire) {
            cortex_m::asm::wfi();
        }
        PULSE_READY.store(false, Ordering::Release);

        self.pb11.clear_interrupt();
        self.pb10.mode(PinMode::Alt(7));
        self.pb11.mode(PinMode::Alt(7));
        self.usart.enable_interrupt(UsartInterrupt::Idle);
        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::USART3);
        }

        // TODO: no monotonic clock wired up yet
        self.start_time = 0;

        Event::RecordPulseReceived
    }

    pub fn handle_record_data(&mut self, heartbeat: bool) -> Event {
        println!("Entered Record Data Handler");

        if COMMAND_READY.swap(false, Ordering::Acquire) {
            self.usart.clear_interrupt(UsartInterrupt::Idle);
            unsafe {
                cortex_m::peripheral::NVIC::unmask(interrupt::USART3);
            }

            // TODO: decode against the real protocol once it exists.
            let _command = self.usart.read_one();

            if command = STOP_COMMAND{ //Fix this with real commands  
                  return Event::StopCommand;
            }
          
        }

        if heartbeat {
            self.usart.write(&[1]).ok();
        }

        match self.adc.read_adc_data() {
            Ok(samples) => {
            
                for channel in samples {
                    self.record_buf[self.record_idx] = (channel as f32).to_bits();
                    self.record_idx += 1;

                    if self.record_idx == self.record_buf.len() {
                        self.winbond_flash.write_page(self.record_buf);
                        self.record_idx = 0;
                        self.page += 1;
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
        // stop recording, set flight flag to off, send status, total data, etc.
        self.persist_status(false);
        Event::Success
    }

    pub fn handle_erase_flash(&mut self) -> Event {
        // erase flash, send success or failure flag over UART
    }

    pub fn handle_error(&mut self) -> Event {
        // Send error signal over uart
    }
}

use defmt::{error, println};
mod statemachine; 
mod adc;
use hal::flash::Flash;
use shared::winbond_flash;
use statemachine::Event;
use adc::ADC; 


use crate::flash::WinbondFlash; 

pub struct StateHandler{
    adc:ADC, 
    winbond_flash:WinbondFlash, 
    internal_flash:Flash,
    // TODO: Finish importing peripherals needed
    // uart, communication, pwm 
}

impl StateHandler{
    pub fn new(adc: ADC, winbond_flash:WinbondFlash, internal_flash:Flash) -> Self{
        let mut state_handler = Self{
            adc, 
            winbond_flash,
            internal_flash,
        }; 

        state_handler
    }

    pub fn handle_wait_for_command(self) -> Event{
        // Wait for Uart command, return command event
    }

    pub fn handle_wait_for_pulse(self) -> Event{
        // Wait for pwm pulse
    }

    pub fn handle_record_data(self, heartbeat:bool) -> Event{
        // start recording, iterate page, set flight flag to on, save time, send status 
        // Go into recording state Every x iteration, send heartbeat pwm
        // Check for command/make stop recording command an interrupt 

        match self.adc.read_adc_data() {
            Ok(data) => {
                println!("{}", data)
            }
            Err(adc::Error::CRC { computed }) => {
                error!("Got CRC Error {}", computed)
            }
        }

    }

    pub fn handle_stop_recording(self) -> Event{
         // stop recording, change to UART, set flight flag to off, send status, total data, etc. 
    }

    pub fn handle_erase_flash(self) -> Event{
        // erase flash, send success or failure flag over UART
    } 

    pub fn handle_error(self, error:Event) -> Event{
        // Send error signal over uart
    }
}
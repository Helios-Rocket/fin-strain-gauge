mod statemachine; 
use statemachine::Event; 

pub struct StateHandler{
    adc:ADC, 
    winbond_flash:WinbondFlash, 
    // TODO: Finish importing peripherals needed
}

impl StateHandler{
    pub fn new() -> Self(){}
    pub fn handle_wait(self) -> Event{
        // Wait for Uart command, return command event
    }
    pub fn handle_record_data(self, heartbeat:bool) -> Event{
         //wait for pulse, start recording, iterate page, set flight flag to on,save time, send status 
        //Maybe pulse should be it's own state...
                // Go into recording state Every x iteration, send heartbeat pwm
                // Check for command/make stop recording command an interrupt 
    }
    pub fn handle_stop_recording(self) -> Event{
         // stop recording, change to UART, set flight flag to off, send status, total data, etc. 
    }
    pub fn handle_erase_flash(self) -> Event{
        // erase flash, send success or failure flag over UART
    } 
    pub fn handle_error(self, error:FinError) -> Event{
        // Send error signal over uart
    }
}
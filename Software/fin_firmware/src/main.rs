#![no_std]
#![no_main]

use adc::ADC;
use defmt::{error, println};
use flash::WinbondFlash;
use statemachine::FinStateMachine;
use statemachine::Event; 
use handlers::StateHandler;  
use hal::{
    clocks::Clocks,
    delay_ms,
    gpio::{Pin, PinMode, Port},
    pac::{self},
};

use defmt_rtt as _;
use panic_probe as _;

use shared::winbond_flash::WinbondStatusReg;
// use shared::communication::CommunicationProtocol

mod statemachine;
mod adc;
mod flash;
mod handlers;

#[cortex_m_rt::entry]
unsafe fn main() -> ! {

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();


    // Setup Clock configuration 
    let clock_cfg = Clocks {
        hsi48_on: true,
        ..Default::default()
    };

    clock_cfg.setup().unwrap();

    let ahb_freq = clock_cfg.apb1();
    println!("{}", ahb_freq);

    // Setup Peripherals 
    let mut led_pin = Pin::new(Port::B, 5, PinMode::Output);
    let mut adc = ADC::new(dp.TIM2, dp.SPI2, &clock_cfg);
    // TODO: Set up better write methods for the windbond
    let mut flash = WinbondFlash::new(&mut dp.RCC, dp.QUADSPI, dp.FLASH, &clock_cfg);
    // TODO: Set up uart/pwm communication pins 
    // TODO: Move stm flash stuff here from out of Winbond Flash


    // Check if block bad 
    for i in 0..512 {
        if flash.is_block_bad(i) {
            println!("Block {} is bad!", i);
        } else {
            println!("Block {} is good!", i);
        }
    }
    // flash.is_block_bad(0);
    println!("Done checking bad blocks");
    // let data = [0xAAAAAAAAu32; 512];
    // flash.write_page(data);
    // println!("Done writing first page");#[repr(
    loop {
        led_pin.toggle();
        delay_ms(1000, ahb_freq);

        println!(
            "Flash Status Regs {:08b} {:08b} {:08b}",
            flash.read_status_register(WinbondStatusReg::One),
            flash.read_status_register(WinbondStatusReg::Two),
            flash.read_status_register(WinbondStatusReg::Three)
        );



        // match adc.read_adc_data() {
        //     Ok(data) => {
        //         println!("{}", data)
        //     }
        //     Err(adc::Error::CRC { computed }) => {
        //         error!("Got CRC Error {}", computed)
        //     }
        // }

        // Flight routine

        // TODO: Finish Setup stuff 

        let mut flight_flag = false; // TODO: Pull this from the stm flash rather than setting it 

        let state = FinStateMachine::new(flight_flag); 
        let handler = StateHandler::new(); 

        loop{

            let event = match state{
                // TODO: Set up so wait does not loop internally and returns a None (no command event)
                FinStateMachine::WaitForCommand => handler.handle_wait(), 
                FinStateMachine::RecordData => handler.handle_record_data(heartbeat), 
                FinStateMachine::StopRecord =>  handler.handle_stop_recording(), 
                FinStateMachine::EraseFlash =>  handler.handle_erase_flash(), 
                FinStateMachine::Error =>  handler.handle_error(error)
            }; 

            state = state.next(event); 

        }
    }
}

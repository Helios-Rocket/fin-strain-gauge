#![no_std]
#![no_main]

use adc::ADC;
use defmt::{error, println};
use flash::WinbondFlash;
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

mod adc;
mod flash;

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


    // Central Procedure

    // Check flight mode flag, if on, start recording data state 

    // Else go into wait for command state

    // If erase command received, erase flash, send success flag over UART
        // Go into wait for command state

    // If start recording received- start recording, iterate page, set flight flag to on,save time, send status 
        // Go into recording state Every x iteration, send heartbeat pwm
        // Check for command/make stop recording command an interrupt 

    // If error, go into error state and send error signal 

    // If stop recording command received, stop recording, change to UART, set flight flag to off, send status, total data, etc. 
        // Go into wait for command state 




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
    }
}

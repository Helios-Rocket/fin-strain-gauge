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
//TODO: add stm flash stuff
use defmt_rtt as _;
use panic_probe as _;

use crate::flash::WinbondStatusReg;
mod adc;
mod flash;

#[cortex_m_rt::entry]
unsafe fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks {
        hsi48_on: true,
        ..Default::default()
    };

    clock_cfg.setup().unwrap();

    let ahb_freq = clock_cfg.apb1();
    println!("{}", ahb_freq);

    let mut led_pin = Pin::new(Port::B, 5, PinMode::Output);
    let mut adc = ADC::new(dp.TIM2, dp.SPI2, &clock_cfg);
    let mut flash = WinbondFlash::new(&mut dp.RCC, dp.QUADSPI, dp.FLASH, &clock_cfg);

    for i in 0..512 {
        if flash.is_block_bad(i) {
            println!("Block {} is bad!", i);
        } else {
            println!("Block {} is good!", i);
        }
    }
    println!("Done checking bad blocks");
    loop {
        led_pin.toggle();
        delay_ms(1000, ahb_freq);

        // println!(
        //     "Flash Status Reg {:08b}",
        //     flash.read_status_register(WinbondStatusReg::One)
        // );

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

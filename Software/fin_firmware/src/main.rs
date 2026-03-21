#![no_std]
#![no_main]

use adc::ADC;
use defmt::{error, println};
use hal::{
    clocks::Clocks,
    delay_ms,
    gpio::{Pin, PinMode, Port},
    pac,
};

use defmt_rtt as _;
use panic_probe as _;
mod adc;

#[cortex_m_rt::entry]
unsafe fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks {
        hsi48_on: true,
        ..Default::default()
    };

    clock_cfg.setup().unwrap();

    let ahb_freq = clock_cfg.apb1();
    println!("{}", ahb_freq);

    let mut led_pin = Pin::new(Port::B, 5, PinMode::Output);
    let mut adc = ADC::new(dp.TIM2, dp.SPI2, &clock_cfg);

    loop {
        println!("I'm working :D!");
        led_pin.toggle();
        delay_ms(1000, ahb_freq);

        match adc.read_adc_data() {
            Ok(data) => {
                println!("{}", data)
            }
            Err(adc::Error::CRC { computed }) => {
                error!("Got CRC Error {}", computed)
            }
        }
    }
}

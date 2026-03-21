#![no_std]
#![no_main]

use hal::{
    clocks::Clocks,
    delay_ms,
    gpio::{Pin, PinMode, Port},
    pac,
};

#[cortex_m_rt::entry]
unsafe fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks {
        hsi48_on: true,
        ..Default::default()
    };

    clock_cfg.setup().unwrap();

    let ahb_freq = clock_cfg.ahb1();

    let mut led_pin = Pin::new(Port::B, 5, PinMode::Output);

    loop {
        led_pin.toggle();
        delay_ms(200, ahb_freq);
    }
}

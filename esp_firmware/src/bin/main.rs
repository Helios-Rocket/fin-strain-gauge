#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::mem;

use defmt::{error, info, println};
use esp_firmware::fin::{self, Fins, GpioPins, PwmPins, SpiPins};
use esp_firmware::lsm::Lsm;
use esp_firmware::sd::{pins::PinsBuilder as SdPinsBuilder, SdHost};
use esp_hal::clock::CpuClock;
use esp_hal::spi::{
    master::{Config, Spi},
    Mode,
};
use esp_hal::time::{Duration, Instant, Rate};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{dma_buffers, main, mcpwm};
use panic_rtt_target as _;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    let spi = Spi::new(
        p.SPI2,
        Config::default()
            .with_frequency(Rate::from_mhz(1))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(p.GPIO18)
    .with_mosi(p.GPIO37)
    .with_miso(p.GPIO36)
    .with_cs(p.GPIO8);

    let mut lsm = Lsm::new(spi);
    let mut sd = SdHost::new(
        &p.SYSTEM,
        p.SDHOST,
        SdPinsBuilder::default()
            .with_clk(p.GPIO13)
            .with_cmd(p.GPIO14)
            .with_data(0, p.GPIO12)
            .with_data(1, p.GPIO11)
            .with_data(2, p.GPIO47)
            .with_data(3, p.GPIO21)
            .build(),
    );

    let mut fins = Fins::new(
        GpioPins::new(p.GPIO4, p.GPIO5, p.GPIO6, p.GPIO7, p.GPIO16),
        SpiPins {
            sck: p.GPIO39.into(),
            miso: p.GPIO40.into(),
            mosi: p.GPIO41.into(),
        },
        p.SPI3.into(),
        PwmPins {
            clk: p.GPIO15.into(),
        },
        p.MCPWM0,
    );

    // let (rx_buf, rx_desc, tx_buf, tx_desc) = dma_buffers!(10);
    // rx_buf[0] = 5;

    // let start = Instant::now();

    // mem::swap(rx_buf, tx_buf);

    // info!("Took {}", Instant::now() - start);

    // declare_aligned_dma_buffer!(BUFFER, 10);
    // let buf = as_mut_byte_array!(BUFFER, 10);

    let timg0 = TimerGroup::new(p.TIMG0);
    let _init = esp_wifi::init(timg0.timer0, esp_hal::rng::Rng::new(p.RNG)).unwrap();

    // sd.init().unwrap();

    loop {
        for (i, data) in fins.read_all_data().into_iter().enumerate() {
            println!("---Fin {}---", i);
            match data {
                Ok(data) => {
                    println!("temp: {}", esp_firmware::fin::convert_volts2temp(data[0]));
                    println!("volts left: {}", data[1]);
                    println!("volts right: {}", data[2]);
                }
                Err(fin::Error::CRC { computed }) => {
                    error!("!!! CRC Failed, got remainder {}", computed)
                }
            }
        }
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(100) {}
    }
}

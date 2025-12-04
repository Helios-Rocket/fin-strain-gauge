#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use alloc::format;
use defmt::{debug, error, info, println, warn};
use esp_firmware::fin::{self, Fins, GpioPins, PwmPins, SpiPins};
use esp_firmware::lsm::Lsm;
use esp_firmware::sd::{pins::PinsBuilder as SdPinsBuilder, SdHost};
use esp_firmware::wifi::Wifi;
use esp_hal::analog::adc::{Adc, AdcConfig};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::spi::{
    master::{Config, Spi},
    Mode,
};
use esp_hal::time::{Duration, Instant, Rate};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{dma_buffers, main, mcpwm};
use fatfs::{FileSystem, FsOptions, Read, Seek, Write};
use mbr_nostd::{MasterBootRecord, PartitionTable};
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
    .with_miso(p.GPIO36);

    let mut lsm = Lsm::new(spi, p.GPIO8);
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
        GpioPins::new(p.GPIO41, p.GPIO42, p.GPIO15, p.GPIO4, p.GPIO6),
        SpiPins {
            sck: p.GPIO5.into(),
            miso: p.GPIO40.into(),
            mosi: p.GPIO39.into(),
        },
        p.SPI3.into(),
        PwmPins {
            clk: p.GPIO7.into(),
        },
        p.MCPWM0,
    );

    let mut adc1_config = AdcConfig::new();
    let mut adc1_pin1 = adc1_config.enable_pin(p.GPIO1, esp_hal::analog::adc::Attenuation::_0dB);
    let mut adc1_pin2 = adc1_config.enable_pin(p.GPIO2, esp_hal::analog::adc::Attenuation::_0dB);
    let mut adc1 = Adc::new(p.ADC1, adc1_config);
    let mut remote_start1_en = Output::new(p.GPIO9, Level::High, OutputConfig::default());
    let mut remote_start2_en = Output::new(p.GPIO10, Level::High, OutputConfig::default());

    // let has_sd = match sd.init() {
    //     Ok(_) => true,
    //     Err(e) => {
    //         warn!("No SD card found!");
    //         false
    //     }
    // };

    // let mut file = Option::None;
    // let mut fs = Option::None;

    // if has_sd {
    //     let fs = fs.insert(
    //         FileSystem::new(sd, FsOptions::new().update_accessed_date(false)).expect("filesystem"),
    //     );

    //     let root_dir = fs.root_dir();

    //     println!("num items {}", root_dir.iter().count());

    //     file = match root_dir.open_file("FOO.TXT") {
    //         Ok(f) => Some(f),
    //         Err(e) => {
    //             warn!("File FOO.TXT not found");
    //             None
    //         }
    //     };

    //     if let Some(foo) = &mut file {
    //         let mut buf = [0_u8; 1024];
    //         let n = foo.read(&mut buf).expect("read");
    //         info!(
    //             "File contents: {}",
    //             str::from_utf8(&buf[0..n]).expect("convert to utf8")
    //         );
    //     }
    // }

    // let timg0 = TimerGroup::new(p.TIMG0);
    // esp_rtos::start(timg0.timer0);
    // let esp_radio_ctrl = esp_radio::init().unwrap();

    // let mut wifi = Wifi::new(p.WIFI, &esp_radio_ctrl);
    // let mut next_send_time = Instant::now() + Duration::from_secs(5);

    let mut start = Instant::now();
    let mut start2 = Instant::now();
    let mut no_crc_err = 0;
    let mut crc_err = 0;
    loop {
        // wifi.receive_data();
        // if start.elapsed() >= Duration::from_millis(250) {
        //     start = Instant::now();
        //     info!(
        //         "Remote Start 1: Pin Level: {}, Current: {}",
        //         remote_start1_en.output_level(),
        //         1100.0 / 4096.0 * (((adc1.read_blocking(&mut adc1_pin1) << 4) as i16) >> 4) as f32
        //             / 20.0
        //     );
        //     info!(
        //         "Remote Start 2: Pin Level: {}, Current: {}",
        //         remote_start2_en.output_level(),
        //         1100.0 / 4096.0 * (((adc1.read_blocking(&mut adc1_pin2) << 4) as i16) >> 4) as f32
        //             / 20.0
        //     );
        //     // println!("Lsm data: {}", lsm.read_lsm().1);

        //     // if let Some(foo) = &mut file {
        //     //     foo.write(format!("{:?}\n", lsm.read_lsm().1).as_bytes())
        //     //         .unwrap();
        //     //     foo.flush().unwrap();
        //     // }
        // }

        // if start2.elapsed() >= Duration::from_secs(1) {
        //     start2 = Instant::now();
        //     // remote_start1_en.toggle();
        //     // remote_start2_en.toggle();
        // }
        if start2.elapsed() >= Duration::from_millis(500) {
            start2 = Instant::now();
            info!(
                "CRC Errors: {} ({}%)",
                crc_err,
                (crc_err as f64 / (crc_err + no_crc_err) as f64 * 100.0)
            );
            crc_err = 0;
            no_crc_err = 0;
        }
        if start.elapsed() >= Duration::from_micros(20000) {
            // debug!("elapsed: {}", start.elapsed());
            start = Instant::now();
            for (i, data) in fins.read_all_data().into_iter().enumerate().take(4) {
                info!("---Fin {}---", i);
                match data {
                    Ok(data) => {
                        println!("temp: {}", esp_firmware::fin::convert_volts2temp(data[0]));
                        println!("volts left: {}", data[1]);
                        println!("volts right: {}", data[2]);
                        no_crc_err += 1;
                    }
                    Err(fin::Error::CRC { computed }) => {
                        crc_err += 1;
                        // error!("!!! CRC Failed, got remainder {}", computed)
                    }
                }
            }
        }
    }
}

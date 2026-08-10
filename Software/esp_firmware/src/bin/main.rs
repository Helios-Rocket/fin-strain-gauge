#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use alloc::format;
use defmt::{info, println, warn};
use esp_firmware::logging::Logger;
use esp_firmware::lsm::Lsm;
use esp_firmware::now_ms;
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
use esp_hal::{main, ram};
use fatfs::{FileSystem, FsOptions, Read, Write};
use panic_rtt_target as _;
use static_cell::StaticCell;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static FS: StaticCell<Option<FileSystem<SdHost<'static>>>> = StaticCell::new();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
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

    let mut adc1_config = AdcConfig::new();
    let mut adc1_pin1 = adc1_config.enable_pin(p.GPIO1, esp_hal::analog::adc::Attenuation::_0dB);
    let mut adc1_pin2 = adc1_config.enable_pin(p.GPIO2, esp_hal::analog::adc::Attenuation::_0dB);
    let mut adc1 = Adc::new(p.ADC1, adc1_config);
    let mut remote_start1_en = Output::new(p.GPIO9, Level::High, OutputConfig::default());
    let mut remote_start2_en = Output::new(p.GPIO10, Level::High, OutputConfig::default());

    let has_sd = match sd.init() {
        Ok(_) => true,
        Err(_e) => {
            warn!("No SD card found!");
            false
        }
    };

    let fs = FS.init(None);
    let log_file;
    let mut lsm_log_file = None;
    let mut logger = Logger::default();

    if has_sd {
        *fs = FileSystem::new(sd, FsOptions::new().update_accessed_date(false)).ok();
    }
    if let Some(fs) = fs {
        let root_dir = fs.root_dir();

        log_file = root_dir
            .create_file("log.txt")
            .inspect_err(|_e| logger.warn(format_args!("Unable to create or open log.txt")))
            .ok();

        if let Some(mut f) = log_file {
            // Don't really care if this errors
            #[allow(unused_must_use)]
            f.truncate();

            logger.with_file(f);
        }

        lsm_log_file = root_dir
            .create_file("lsm_data.csv")
            .inspect_err(|_e| logger.warn(format_args!("Unable to create or open lsm_data.csv")))
            .ok();
    }

    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_int = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let mut wifi = Wifi::new(p.WIFI);

    let mut lsm_last_read = Instant::now();
    // let mut wifi_last_send = Instant::now();

    loop {
        // wifi.receive_data();
        // if wifi_last_send.elapsed() >= Duration::from_secs(1) {
        //     wifi_last_send = Instant::now();
        //     wifi.send_data();
        // }
        if lsm_last_read.elapsed() >= Duration::from_millis(250) {
            logger.info(format_args!("log lsm"));
            lsm_last_read = Instant::now();
            if let Some(f) = lsm_log_file.as_mut() {
                #[allow(unused_must_use)]
                f.write(format!("{},{:?}\n", now_ms!(), lsm.read_lsm().1).as_bytes());

                #[allow(unused_must_use)]
                f.flush();
            }
        }

        // info!(
        //     "Remote Start 1: Pin Level: {}, Current: {}",
        //     remote_start1_en.output_level(),
        //     1100.0 / 4096.0 * (((adc1.read_blocking(&mut adc1_pin1) << 4) as i16) >> 4) as f32
        //         / 20.0
        // );
        // info!(
        //     "Remote Start 2: Pin Level: {}, Current: {}",
        //     remote_start2_en.output_level(),
        //     1100.0 / 4096.0 * (((adc1.read_blocking(&mut adc1_pin2) << 4) as i16) >> 4) as f32
        //         / 20.0
        // );
    }
}

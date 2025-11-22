#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use alloc::format;
use defmt::{error, info, println};
use esp_firmware::fin::{self, Fins, GpioPins, PwmPins, SpiPins};
use esp_firmware::lsm::Lsm;
use esp_firmware::sd::{pins::PinsBuilder as SdPinsBuilder, SdHost};
use esp_firmware::wifi::Wifi;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::spi::{
    master::{Config, Spi},
    Mode,
};
use esp_hal::time::{Duration, Instant, Rate};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{dma_buffers, main, mcpwm};
use fatfs::{FileSystem, FsOptions, Read, Write};
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


    info!("START OF SD CARD STUFF"); 

    sd.init().expect("sd init");
    let fs = FileSystem::new(sd, FsOptions::new().update_accessed_date(false)).expect("filesystem");
    {
        let root_dir = fs.root_dir();

        println!("num items {}", root_dir.iter().count());

        let mut foo = root_dir.open_file("FOO.TXT").expect("open");
        let mut buf = [0_u8; 1024];
        let n = foo.read(&mut buf).expect("read");

        error!(
            "File contents: {}",
            str::from_utf8(&buf[0..n]).expect("convert to utf8")
        );

        let start = Instant::now();
        while Instant::now() < start + Duration::from_secs(5) {}

        let mut count = 0;

        foo.write(format!("{count}").as_bytes())
            .expect("buf not writing :(");
        count += 1;
        foo.flush().unwrap();
    }
    fs.unmount();

    let timg0 = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timg0.timer0);
    let esp_radio_ctrl = esp_radio::init().unwrap();

    let mut wifi = Wifi::new(p.WIFI, &esp_radio_ctrl);
    let mut next_send_time = Instant::now() + Duration::from_secs(5);

    loop {
        wifi.receive_data();

        if Instant::now() >= next_send_time {
            wifi.send_data();
            next_send_time = Instant::now() + Duration::from_secs(5);
            // println!(
            //     "Length of count bytes thing {}",
            //     format!("{count}").as_bytes().len()
            // );
        }

        // for (i, data) in fins.read_all_data().into_iter().enumerate().take(1) {
        //     println!("---Fin {}---", i);
        //     match data {
        //         Ok(data) => {
        //             println!("temp: {}", esp_firmware::fin::convert_volts2temp(data[0]));
        //             println!("volts left: {}", data[1]);
        //             println!("volts right: {}", data[2]);
        //         }
        //         Err(fin::Error::CRC { computed }) => {
        //             error!("!!! CRC Failed, got remainder {}", computed)
        //         }
        //     }
        // }
    }
}

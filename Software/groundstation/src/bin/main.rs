#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::fmt::Write;

use alloc::format;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use esp_radio::esp_now::BROADCAST_ADDRESS;
use esp_radio::wifi::{ControllerConfig, CountryInfo};
use shared::remote_commands;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.3.0
    // generator parameters: --chip esp32 -o esp32-wroom-32e -o unstable-hal -o alloc -o wifi -o log -o esp-backtrace

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    // The following pins are used to bootstrap the chip. They are available
    // for use, but check the datasheet of the module for more information on them.
    // - GPIO0
    // - GPIO2
    // - GPIO5
    // - GPIO12
    // - GPIO15
    // These GPIO pins are in use by some feature of the module and should not be used.
    let _ = p.GPIO6;
    let _ = p.GPIO7;
    let _ = p.GPIO8;
    let _ = p.GPIO9;
    let _ = p.GPIO10;
    let _ = p.GPIO11;
    let _ = p.GPIO16;
    let _ = p.GPIO20;

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_interrupt = esp_hal::interrupt::software::SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);
    let (mut _wifi_controller, interfaces) = esp_radio::wifi::new(
        p.WIFI,
        ControllerConfig::default().with_country_info(CountryInfo::from(*b"US")),
    )
    .expect("Failed to initialize Wi-Fi controller");

    let mut usb = UsbSerialJtag::new(p.USB_DEVICE);

    let mut esp_now = interfaces.esp_now;
    esp_now.set_channel(11).unwrap();

    // let mut uart = Uart::new(p.UART0, uart::Config::default().with_baudrate(115200))
    //     .expect("UART0")
    //     .with_tx(p.GPIO21)
    //     .with_rx(p.GPIO20);

    let mut uart_buf = [0_u8; 4096];
    let mut idx = 0;

    // let delay = Delay::new();

    loop {
        while let Ok(d) = usb.read_byte() {
            uart_buf[idx] = d;
            idx += 1;
            if idx != 0 && uart_buf[idx - 1] == b'\n' {
                if let Some(cmd) = match str::from_utf8(&uart_buf[..idx - 1]).unwrap() {
                    "fin erase" => Some(remote_commands::Command::EraseFinFlashes),
                    "fin arm" => Some(remote_commands::Command::ArmFins),
                    "fin disarm" => Some(remote_commands::Command::DisarmFins),
                    "altimeter arm" => Some(remote_commands::Command::ArmAltimeter),
                    "altimeter disarm" => Some(remote_commands::Command::DisarmAltimeter),
                    "video start" => Some(remote_commands::Command::StartLiveVideo),
                    "video stop" => Some(remote_commands::Command::StopLiveVideo),
                    cmd => {
                        usb.write_str(&format!("Invalid command: {:?}\n", cmd))
                            .expect("write");
                        None
                    }
                } {
                    esp_now
                        .send(&BROADCAST_ADDRESS, &[cmd as u8])
                        .unwrap()
                        .wait()
                        .unwrap();
                }
                idx = 0;
            }
        }
    }
}

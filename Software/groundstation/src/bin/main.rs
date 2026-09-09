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
use esp_radio::esp_now::{BROADCAST_ADDRESS, PeerInfo};
use esp_radio::wifi::{ControllerConfig, CountryInfo};
use shared::remote_commands::{COMMANDS, REMOTE_HEADER};

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

    let mut peer_addr = BROADCAST_ADDRESS;

    let mut uart_buf = [0_u8; 4096];
    let mut idx = 0;

    loop {
        while let Ok(d) = usb.read_byte() {
            uart_buf[idx] = d;
            idx += 1;
            if idx != 0 && uart_buf[idx - 1] == b'\n' {
                if let Some(cmd) = COMMANDS
                    .iter()
                    .find(|c| c.0 == str::from_utf8(&uart_buf[..idx - 1]).unwrap())
                    .map(|c| c.1)
                {
                    esp_now
                        .send(
                            &peer_addr,
                            [REMOTE_HEADER, &[cmd as u8]].concat().as_slice(),
                        )
                        .unwrap()
                        .wait()
                        .unwrap();

                    loop {
                        let r = esp_now.receive();
                        if let Some(r) = r {
                            if !esp_now.peer_exists(&r.info.src_address) {
                                esp_now
                                    .add_peer(PeerInfo {
                                        interface: esp_radio::esp_now::EspNowWifiInterface::Station,
                                        peer_address: r.info.src_address,
                                        lmk: None,
                                        channel: None,
                                        encrypt: false,
                                    })
                                    .unwrap();
                            }
                            peer_addr = r.info.src_address;

                            usb.write(r.data()).expect("write");

                            break;
                        }
                    }
                }
                idx = 0;
            }
        }
    }
}

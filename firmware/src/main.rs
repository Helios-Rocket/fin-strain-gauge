#![no_std]
#![no_main]

mod fin;
mod sd;
use teensy4_panic as _;

// Not sure what the dispatcher is doing here... potentially nothing yet
// It's an interrupt that can be used for task preemption, but afaict, there isn't any of that going on
// other than maybe with the poll usb task, but that's bound to its own interrupt
#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use bsp::board;

    use teensy4_bsp::{
        self as bsp,
        hal::{
            flexpwm,
            gpio::Input,
            iomuxc,
            usbd::{self, gpt::Mode},
        },
        pins::{self, Config},
    };

    use arrform::{arrform, ArrForm};

    // use imxrt_log as logging;

    use rtic_monotonics::{
        systick::{Systick, *},
        Monotonic,
    };
    use usb_device::{
        bus::UsbBusAllocator,
        device::{UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
    };
    use usbd_serial::SerialPort;

    use crate::{
        fin::{self, Fins},
        sd::SdCard,
    };

    const VID_PID: UsbVidPid = UsbVidPid(0x5824, 0x27dd);
    const PRODUCT: &str = "strain-gauge-logger";

    #[shared]
    struct Shared {
        serial: SerialPort<'static, usbd::BusAdapter>,
    }

    /// These resources are local to individual tasks.
    #[local]
    struct Local {
        /// A poller to control USB logging.
        // poller: logging::Poller,
        start_pin: Input<pins::t41::P22>,
        fins: Fins<
            fin::Pins<
                pins::t41::P12,
                pins::t41::P11,
                pins::t41::P13,
                pins::t41::P16,
                pins::t41::P2,
                pins::t41::P33,
            >,
            4, // lpspi4
            4, // flexpwm4
            2, // flexpwm4 submodule2
        >,
        usb_device: UsbDevice<'static, usbd::BusAdapter>,
        sd: SdCard,
    }

    #[init(local = [
        ep_mem: usbd::EndpointMemory<2048> = usbd::EndpointMemory::new(),
        ep_state: usbd::EndpointState = usbd::EndpointState::new(),
        bus_alloc: Option<UsbBusAllocator<usbd::BusAdapter>> = None,
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio1,
            mut gpio2,
            mut gpio4,
            mut pins,
            lpspi4,
            usb,
            ccm,
            flexpwm4,
            ..
        } = board::t41(cx.device);

        let sd = unsafe { SdCard::new(&mut pins, &ccm, Systick::now) };

        // let poller = logging::log::usbd(usb, logging::Interrupts::Enabled).unwrap();

        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        iomuxc::configure(
            &mut pins.p23,
            Config::zero().set_pull_keeper(Some(iomuxc::PullKeeper::Pulldown100k)),
        );
        let start_pin = gpio1.input(pins.p22);

        let bus_adapter = usbd::BusAdapter::with_speed(
            usb,
            cx.local.ep_mem,
            cx.local.ep_state,
            usbd::Speed::LowFull,
        );
        bus_adapter.set_interrupts(true);
        bus_adapter.gpt_mut(usbd::gpt::Instance::Gpt0, |gpt| {
            gpt.stop();
            gpt.clear_elapsed();
            gpt.set_interrupt_enabled(true);
            gpt.set_mode(Mode::Repeat);
            gpt.set_load(10_000); // microseconds.
            gpt.reset();
            gpt.run();
        });
        let bus_alloc = cx.local.bus_alloc.insert(UsbBusAllocator::new(bus_adapter));
        let mut serial = SerialPort::new(bus_alloc);
        let mut usb_device = UsbDeviceBuilder::new(bus_alloc, VID_PID)
            .product(PRODUCT)
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        loop {
            if usb_device.poll(&mut [&mut serial]) {
                if usb_device.state() == UsbDeviceState::Configured {
                    break;
                }
            }
        }

        usb_device.bus().configure();

        let mut fins = Fins::new(
            fin::Pins {
                miso: pins.p12,
                mosi: pins.p11,
                sck: pins.p13,
                cs1: gpio1.output(pins.p16),
                rst: gpio4.output(pins.p33),
                clk: flexpwm::Output::new_a(pins.p2),
            },
            lpspi4,
            flexpwm4.0,
            flexpwm4.1 .2,
        );

        // fins.disable_adc_channels();
        // // let reg = fins.read_register(fin::registers::mode::ADDR);
        // // fins.write_register(
        // //     fin::registers::mode::ADDR,
        // //     (reg & !fin::registers::mode::wlength::MASK)
        // //         | fin::registers::mode::wlength::LENGTH_32BITS,
        // // );
        // fins.write_register(0xe, 0b10);
        // fins.write_register(0x13, 0b11);
        // fins.enable_adc_channels();

        // blink::spawn().unwrap();
        output::spawn().unwrap();
        read_fin_spi::spawn().unwrap();
        (
            Shared { serial },
            Local {
                // poller,
                sd,
                fins,
                usb_device,
                start_pin,
            },
        )
    }

    #[task(local = [start_pin, sd])]
    async fn output(cx: output::Context) {
        while !cx.local.start_pin.is_set() {
            Systick::delay(5.millis()).await;
        }

        let resp = cx.local.sd.init();
        if let Err(e) = resp {
            // log::error!("{:?}", e);
        }
    }

    #[task(binds = USB_OTG1, local = [usb_device], shared = [serial])]
    fn log_over_usb(mut cx: log_over_usb::Context) {
        cx.local
            .usb_device
            .bus()
            .gpt_mut(usbd::gpt::Instance::Gpt0, |gpt| {
                while gpt.is_elapsed() {
                    gpt.clear_elapsed();
                }
            });

        cx.shared.serial.lock(|serial| {
            cx.local.usb_device.poll(&mut [serial]);
        });
    }

    #[task(shared = [serial], local = [fins])]
    // Make this a function within fins
    async fn read_fin_spi(mut cx: read_fin_spi::Context) {
        loop {
            match cx.local.fins.read_adc_data() {
                Ok(data) => {
                    cx.shared.serial.lock(|serial| {
                        if let Ok(_) = serial.write(
                            arrform!(
                                128,
                                "{}\r\n",
                                fin::convert_volts2temp(fin::convert_adc2volts(data[0])),
                            )
                            .as_bytes(),
                        ) {
                        } else {
                        }
                    });
                    cx.shared.serial.lock(|serial| {
                        if let Ok(_) = serial.write(
                            arrform!(128, "{} V\r\n", fin::convert_adc2volts(data[1]),).as_bytes(),
                        ) {
                        } else {
                        }
                    });
                    let volts = fin::convert_adc2volts(data[2]);
                    //if volts.abs() < 0.025 {
                        cx.shared.serial.lock(|serial| {
                            if let Ok(_) =
                                serial.write(arrform!(128, ">volts:{}\r\n", volts,).as_bytes())
                            {
                            } else {
                            }
                        });
                    //}
                }
                Err(fin::Error::CRC { computed }) => cx.shared.serial.lock(|serial| {
                    if let Ok(_) = serial.write(
                        arrform!(128, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! CRC Failed! Got remainder {}\r\n", computed).as_bytes(),
                    ) {
                    } else {
                    }
                }),
            }
            Systick::delay(20_u32.millis()).await;
        }
    }
}

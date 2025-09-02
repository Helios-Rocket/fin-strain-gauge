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
        hal::{flexpwm, gpio::Input, iomuxc},
        pins::{self, Config},
    };

    use imxrt_log as logging;

    use rtic_monotonics::{
        systick::{Systick, *},
        Monotonic,
    };

    use crate::{
        fin::{self, Fins},
        sd::SdCard,
    };

    /// There are no resources shared across tasks.
    #[shared]
    struct Shared {}

    /// These resources are local to individual tasks.
    #[local]
    struct Local {
        /// A poller to control USB logging.
        poller: logging::Poller,
        start_pin: Input<pins::t41::P23>,
        fins: Fins<
            fin::Pins<
                pins::t41::P12,
                pins::t41::P11,
                pins::t41::P13,
                pins::t41::P15,
                pins::t41::P2,
                pins::t41::P10,
            >,
            4, // lpspi4
            4, // flexpwm4
            2, // flexpwm4 submodule2
        >,
        sd: SdCard,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio1,
            mut gpio2,
            mut pins,
            lpspi4,
            usb,
            ccm,
            flexpwm4,
            ..
        } = board::t41(cx.device);

        let sd = unsafe { SdCard::new(&mut pins, &ccm, Systick::now) };

        let mut fins = Fins::new(
            fin::Pins {
                miso: pins.p12,
                mosi: pins.p11,
                sck: pins.p13,
                cs1: gpio1.output(pins.p15),
                rst: gpio2.output(pins.p10),
                clk: flexpwm::Output::new_a(pins.p2),
            },
            lpspi4,
            flexpwm4.0,
            flexpwm4.1 .2,
        );

        let poller = logging::log::usbd(usb, logging::Interrupts::Enabled).unwrap();

        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        iomuxc::configure(
            &mut pins.p23,
            Config::zero().set_pull_keeper(Some(iomuxc::PullKeeper::Pulldown100k)),
        );
        let start_pin = gpio1.input(pins.p23);

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
            Shared {},
            Local {
                poller,
                sd,
                fins,
                start_pin,
            },
        )
    }

    #[task(local = [start_pin, sd])]
    async fn output(cx: output::Context) {
        while !cx.local.start_pin.is_set() {
            Systick::delay(5.millis()).await;
        }
        log::info!("hi");

        let resp = cx.local.sd.init();
        if let Err(e) = resp {
            log::error!("{:?}", e);
        }
    }

    #[task(binds = USB_OTG1, local = [poller])]
    fn log_over_usb(cx: log_over_usb::Context) {
        cx.local.poller.poll();
    }

    #[task(local = [fins])]
    // Make this a function within fins
    async fn read_fin_spi(cx: read_fin_spi::Context) {
        loop {
            cx.local.fins.read_adc_data().into_iter().for_each(|t| {
                log::info!(
                    "{} (volts: {}) [raw: {}]",
                    fin::convert_volts2temp(fin::convert_adc2volts(t)),
                    fin::convert_adc2volts(t),
                    t,
                )
            });
            // let data = cx.local.fins.read_register(0);
            // log::info!("{:016b}", data);
            Systick::delay(500_u32.millis()).await;
        }
    }
}

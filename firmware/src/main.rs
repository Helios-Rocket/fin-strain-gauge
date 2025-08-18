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
        hal::{self, gpio::Input, iomuxc, lpspi::Lpspi},
        pins::{self, Config, PullKeeper},
        ral::{self, modify_reg, write_reg},
    };

    use imxrt_log as logging;

    use rtic_monotonics::{
        systick::{Systick, *},
        Monotonic,
    };

    use crate::{/*fin::Fins,*/ sd::SdCard};

    /// There are no resources shared across tasks.
    #[shared]
    struct Shared {}

    /// These resources are local to individual tasks.
    #[local]
    struct Local {
        /// The LED on pin 13.
        // led: board::Led,
        /// A poller to control USB logging.
        poller: logging::Poller,
        start_pin: Input<pins::t41::P23>,
        // fins: Fins<pins::t41::P15, pins::t41::P14, U4, U1>,
        spi: Lpspi<(), 4>,
        tmr: ral::tmr::Instance<3>,

        sd: SdCard,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio1,
            mut pins,
            lpspi4,
            usb,
            ccm,
            ..
        } = board::t41(cx.device);

        let sd = unsafe { SdCard::new(&mut pins, &ccm, Systick::now) };

        // let led = board::led(&mut gpio2, pins.p13);

        // let fins = Fins::new(&mut pins, lpspi4, &mut gpio1);
        iomuxc::lpspi::prepare(&mut pins.p11);
        iomuxc::lpspi::prepare(&mut pins.p12);
        iomuxc::lpspi::prepare(&mut pins.p13);
        let spi = hal::lpspi::Lpspi::without_pins(lpspi4);
        let cs = gpio1.output(pins.p15);

        iomuxc::alternate(&mut pins.p14, 1);
        iomuxc::set_sion(&mut pins.p14);
        iomuxc::configure(
            &mut pins.p14,
            Config::zero().set_pull_keeper(Some(PullKeeper::Keeper)),
        );
        modify_reg!(ral::ccm, ccm, CCGR6, CG15: 0b11);
        let tmr = unsafe { ral::tmr::Instance::<3>::instance() };

        tmr.CNTR2.write(0);
        modify_reg!(ral::tmr, tmr, SCTRL2, FORCE: 1, OEN: 1);
        tmr.COMP12.write(1500);
        // tmr.COMP12.write(18);
        // tmr.LOAD2.write(0xffff - 18);
        write_reg!(ral::tmr, tmr, CTRL2, CM: 0b001, PCS: 0b1000, ONCE: 0, LENGTH: 1, OUTMODE: 0b011);
        tmr.ENBL.write(0b0100);

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

        // blink::spawn().unwrap();
        output::spawn().unwrap();
        (
            Shared {},
            Local {
                // led,
                poller,
                sd,
                spi,
                // cs,
                tmr,
                start_pin,
            },
        )
    }

    // #[task(local = [led])]
    // async fn blink(cx: blink::Context) {
    //     loop {
    //         Systick::delay(1.secs()).await;
    //         cx.local.led.toggle();
    //     }
    // }

    #[task(local = [spi, tmr])]
    async fn read_fins(cx: read_fins::Context) {}

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

    // #[task(binds = USDHC1, shared = [sd])]
    // fn handle_sd_ints(cx: handle_sd_ints::Context) {}
}

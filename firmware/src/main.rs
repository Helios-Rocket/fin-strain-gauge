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

    use cortex_m::prelude::{_embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write};
    use teensy4_bsp::{
        self as bsp,
        hal::{self, ccm::{analog::pll2, clock_gate, lpspi_clk}, flexpwm, gpio::{self, Input}, iomuxc::{self, flexpwm::Output}, lpspi::{Lpspi, SamplePoint, MODE_1}},
        pins::{self, Config, PullKeeper},
        ral::{self, modify_reg, read_reg, write_reg},
    };

    use imxrt_log as logging;

    use rtic_monotonics::{
        systick::{Systick, *},
        Monotonic,
    };

    use embedded_hal::spi::{SpiDevice}; 

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
        //tmr: ral::tmr::Instance<3>,

        sd: SdCard,
        cs: gpio::Output<pins::t41::P15>,
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

        // let led = board::led(&mut gpio2, pins.p13);

        // let fins = Fins::new(&mut pins, lpspi4, &mut gpio1);
        iomuxc::lpspi::prepare(&mut pins.p11); // DIN (MOSI)
        iomuxc::lpspi::prepare(&mut pins.p12); // DOUT (MISO)
        iomuxc::lpspi::prepare(&mut pins.p13); // SCK
        let mut spi = hal::lpspi::Lpspi::without_pins(lpspi4);

        const LPSPI_CLK_DIVIDER: u32 = 8;
        const LPSPI_CLK_HZ: u32 = pll2::FREQUENCY / LPSPI_CLK_DIVIDER;

        let mut ccm = unsafe { ral::ccm::CCM::instance() };
        clock_gate::lpspi::<2>().set(&mut ccm, clock_gate::OFF);
        lpspi_clk::set_selection(&mut ccm, lpspi_clk::Selection::Pll2);
        lpspi_clk::set_divider(&mut ccm, LPSPI_CLK_DIVIDER);

        clock_gate::lpspi::<2>().set(&mut ccm, clock_gate::ON);
        
        spi.disabled(|spi|{
            spi.set_clock_hz(LPSPI_CLK_HZ, 1_000_000);
            spi.set_sample_point(SamplePoint::Edge);
        }); 
        spi.set_mode(MODE_1);

        let cs = gpio1.output(pins.p15); // CS 
        let rst = gpio2.output(pins.p10); //RST
        rst.set();

        // FLEX PWM IMPLEMENTATION
        let mut sm2 = flexpwm4.1.2; 
        let mut pwm = flexpwm4.0; 

        sm2.set_debug_enable(true);
        sm2.set_wait_enable(true);
        sm2.set_clock_select(flexpwm::ClockSelect::Ipg);
        sm2.set_prescaler(flexpwm::Prescaler::Prescaler1);
        sm2.set_pair_operation(flexpwm::PairOperation::Independent);
        sm2.set_load_mode(flexpwm::LoadMode::reload_full());
        sm2.set_load_frequency(1);

        flexpwm::Output::new_a(pins.p2); // CLK

        sm2.set_initial_count(&pwm, 0);
        sm2.set_value(flexpwm::FULL_RELOAD_VALUE_REGISTER, 38);
        sm2.set_turn_on(flexpwm::Channel::A, 19);
        sm2.set_turn_off(flexpwm::Channel::A, 38);
        sm2.set_output_enable(&mut pwm, flexpwm::Channel::A, true);
        // Load the values into the PWM registers.
        sm2.set_load_ok(&mut pwm);

        sm2.set_output_enable(&mut pwm, flexpwm::Channel::A, true);
        sm2.set_running(&mut pwm, true);
        
        // QUAD TIMER IMPLEMENTATION 
        // iomuxc::alternate(&mut pins.p14, 1);
        // iomuxc::clear_sion(&mut pins.p14);
        // iomuxc::configure(
        //     &mut pins.p14,
        //     Config::zero().set_pull_keeper(Some(PullKeeper::Pullup100k)),
        // );

        // let tmr = unsafe { ral::tmr::Instance::<3>::instance() };

        // modify_reg!(ral::tmr, tmr, CSCTRL2, TCF1EN:1); 
        

        // //tmr.ENBL.write(0b1101);
        // tmr.CNTR2.write(0);
        // modify_reg!(ral::tmr, tmr, SCTRL2, OPS: 0, OEN: 1);
        // tmr.COMP12.write(1500);
        // // tmr.COMP12.write(18);
        // // tmr.LOAD2.write(0xffff - 18);
        // write_reg!(ral::tmr, tmr, CTRL2, CM: 0b001, PCS: 0b1000, ONCE: 0, LENGTH: 1, OUTMODE: 0b010);

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
        read_spi::spawn().unwrap(); 
        (
            Shared {},
            Local {
                // led,
                poller,
                sd,
                spi,
                cs,
                //tmr,
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

    // #[task(local = [spi, tmr])]
    // async fn read_fins(cx: read_fins::Context) {
    //     loop{
    //         log::info!("{}", cx.local.tmr.CNTR2.read()); 
    //         Systick::delay(1700.millis()).await;
    //     }

    // }

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

    // #[task(binds = TMR3, local = [tmr])]
    // fn tmr_interrupt(cx: tmr_interrupt::Context){
    //     if read_reg!(ral::tmr, cx.local.tmr, CSCTRL2, TCF1==1){
    //         log::info!("{}", cx.local.tmr.CNTR2.read()); 
    //         modify_reg!(ral::tmr, cx.local.tmr, CSCTRL2, TCF1:0); 
    //     }
       
    // }

    // #[task(binds = USDHC1, shared = [sd])]
    // fn handle_sd_ints(cx: handle_sd_ints::Context) {}
    #[task(local = [spi, cs])]
    async fn read_spi(mut cx: read_spi::Context){
        loop{
            cx.local.cs.clear(); 
            log::info!("{:?}", convert_adc2temp(read_registers(&mut cx.local.spi, 0)[0])); 
            cx.local.cs.set(); 
            Systick::delay(1_u32.secs()).await; 
        }
    }


    fn read_registers(spi:&mut Lpspi<(), 4>, start_addr: u8)->[u32; 3]{
        let start_addr: u32 = (start_addr as u32) & 0x3F; 
        let word: u32 = 0;
        log::info!("{:#018b}", word);  
        spi.transfer(&mut word.to_be_bytes()[0..=2]).unwrap(); 
        let mut dummy = [0_u8;12];
        let output = spi.transfer(&mut dummy).unwrap(); 
        [
            (output[3] as u32) << 16 | (output[4] as u32) << 8 | (output[5] as u32), 
            (output[6] as u32) << 16 | (output[7] as u32) << 8 | (output[8] as u32), 
            (output[9] as u32) << 16 | (output[10] as u32) << 8 | (output[11] as u32), 

        ]
    
    }

    fn convert_adc2temp(input:u32)->f32{
        let ref_voltage = (1.2/(((1_u32<<24) as f32) - 1.0)) * input as f32; 
        (ref_voltage - 500.0)*100.0 
    }

    fn write_registers(){}

}



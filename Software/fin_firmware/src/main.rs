#![no_std]
#![no_main]

use adc::ADC;
use core::slice;
use core::sync::atomic::{AtomicBool, Ordering};
use core::time::Duration;
use cortex_m::delay;
use defmt::{error, info, println};
use embedded_hal::spi::{MODE_0, MODE_1, Mode};
use embedded_io::Write;
use flash::WinbondFlash;
use hal::flash::Bank;
use hal::pac::USART3;
use hal::pac::i2c1::cr2::HEAD10R;
use hal::timer::TimerConfig;
use hal::usart::UsartConfig;
use hal::usart::UsartInterrupt;
use hal::{
    BaudPeriph, access_global, gpio, init_globals, make_globals, make_simple_globals, setup_nvic,
};
use hal::{
    clocks::Clocks,
    delay_ms,
    flash::Flash,
    gpio::{Pin, PinMode, Port},
    instant::Instant,
    pac::{self, SPI1, SPI2, TIM2, interrupt},
    spi::Spi,
    timer::{self, Timer},
    usart::Usart,
};
use handlers::StateHandler;
use shared::errors::{AdcError, WinbondFlashError};
use shared::winbond_flash;
use statemachine::FinState;

use defmt_rtt as _;
use panic_probe as _;

use shared::winbond_flash::WinbondStatusReg;

use crate::adxl::ADXL;
// use shared::communication::CommunicationProtocol

mod adc;
mod adxl;
mod flash;
mod handlers;
mod statemachine;

pub static COMMAND_READY: AtomicBool = AtomicBool::new(false);
pub static PULSE_READY: AtomicBool = AtomicBool::new(false);

// Filled in one byte at a time by the USART3 RXNE interrupt. `RX_LEN` is how many
// bytes have landed since the last Idle-line event; the Idle interrupt snapshots
// and resets it, so a frame is only considered valid (and COMMAND_READY set) if
// exactly 4 bytes showed up between idle periods.

make_simple_globals!(
    (RX_BUF, [u8; 4], [0u8; 4]),
    (RX_LEN, usize, 0),
    (LAST_ADC_ERROR, Option<AdcError>, None),
    (ADC_ERROR_COUNT, u16, 0),
    (LAST_WINBOND_ERROR, Option<WinbondFlashError>, None),
    (WINBOND_ERROR_COUNT, u16, 0),
);

#[cfg(feature = "prototype")]
type SensorSpi = SPI2;
#[cfg(not(feature = "prototype"))]
type SensorSpi = SPI1;

make_globals!((UART, Usart<USART3>), (SPI, Spi<SensorSpi>));

fn set_spi_mode(new_mode: Mode) {
    critical_section::with(|cs| {
        access_global!(SPI, spi, cs);
        spi.disable().unwrap();
        spi.regs.cr1().modify(|_, w| {
            w.cpol()
                .bit(new_mode.polarity as u8 != 0)
                .cpha()
                .bit(new_mode.phase as u8 != 0)
                .spe()
                .bit(true)
        });
    });
}

#[cortex_m_rt::entry]
unsafe fn main() -> ! {
    println!("hello world");
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

    // Setup Clock configuration
    let clock_cfg = Clocks {
        hsi48_on: true,
        ..Default::default()
    };

    clock_cfg.setup().unwrap();

    // FIXME: temporary and only for debugging
    hal::debug_workaround();

    let ahb_freq = clock_cfg.apb1();
    // println!("{}", ahb_freq);
    // cfg_if::cfg_if! {
    //     if #[cfg(feature = "prototype")] {
    //         let mut led = Pin::new(Port::B, 5, PinMode::Output);
    //     } else {
    //         let mut led = Pin::new(Port::A, 8, PinMode::Output);
    //     }
    // }
    // led.set_high();

    // loop {
    //     delay_ms(500, ahb_freq);
    //     info!("Toggle");
    //     led.toggle();
    // }

    // Setup SPI
    // CS for ADC, figure out how to handle
    // let mut cs_pin = Pin::new(Port::B, 12, PinMode::Output);
    // cs_pin.set_high();
    #[cfg(feature = "prototype")]
    {
        let _sck = Pin::new(Port::B, 13, PinMode::Alt(5));
        let _miso = Pin::new(Port::B, 14, PinMode::Alt(5));
        let _mosi = Pin::new(Port::B, 15, PinMode::Alt(5));
    }

    #[cfg(not(feature = "prototype"))]
    {
        let _sck = Pin::new(Port::A, 5, PinMode::Alt(5));
        let _miso = Pin::new(Port::A, 11, PinMode::Alt(5));
        let _mosi = Pin::new(Port::A, 12, PinMode::Alt(5));
    }

    let spi_config = hal::spi::SpiConfig {
        mode: MODE_1,
        ..Default::default()
    };

    // Setup SPI bus with clock speed of 8 MHz
    cfg_if::cfg_if! {
        if #[cfg(feature = "prototype")] {
            let spi_periph = dp.SPI2;
        } else {
            let spi_periph = dp.SPI1;
        }
    }
    let spi = Spi::new(spi_periph, spi_config, hal::spi::BaudRate::Div32);

    // Comm pins to Avbay
    let mut pb10 = Pin::new(Port::B, 10, PinMode::Alt(7));
    let mut pb11 = Pin::new(Port::B, 11, PinMode::Alt(7));
    let usart3 = Usart::new(dp.USART3, 1800, UsartConfig::default(), &clock_cfg)
        .expect("Failed to initialize");

    init_globals!((UART, usart3), (SPI, spi));

    // Setup Peripherals
    cfg_if::cfg_if! {
        if #[cfg(not(feature = "prototype"))] {
            let mut adxl1_cs = Pin::new(Port::B, 6, PinMode::Output);
            adxl1_cs.set_high();
            let mut adxl2_cs = Pin::new(Port::B, 7, PinMode::Output);
            adxl2_cs.set_high();
        }
    }
    let mut adc = ADC::new(dp.TIM2, &clock_cfg);
    let mut flash = WinbondFlash::new(&mut dp.RCC, dp.QUADSPI, &clock_cfg);
    cfg_if::cfg_if! {
        if #[cfg(not(feature = "prototype"))] {
            let mut adxl1 = ADXL::new(&clock_cfg, adxl1_cs);
            let mut adxl2 = ADXL::new(&clock_cfg, adxl2_cs);
        }
    }

    setup_nvic!([(USART3, 2), (EXTI15_10, 1), (TIM16, 0)], cp);
    // Enable interrupt
    critical_section::with(|cs| {
        access_global!(UART, uart, cs);
        uart.enable_interrupt(UsartInterrupt::Idle)
            .expect("enable uart idle interrupt");
        uart.enable_interrupt(UsartInterrupt::ReadNotEmpty)
            .expect("enable uart read not empty interrupt");
    });

    let mut internal_flash = Flash::new(dp.FLASH);

    let timer_config = TimerConfig {
        update_request_source: hal::timer::UpdateReqSrc::OverUnderFlow,
        ..Default::default()
    };

    let mut timer = Timer::new_tim1(dp.TIM1, 1000 as f32, timer_config, &clock_cfg);
    timer.enable_interrupt(hal::timer::TimerInterrupt::Update);
    timer.enable();
    println!("Enabled?: {}", timer.is_enabled());
    let timer_start = timer.now();
    println!("Timer Start: {}", timer_start.as_nanos());

    // let (mut handler, mut state) =
    // StateHandler::new(adc, flash, internal_flash, timer, timer_start, pb10, pb11);

    //========= Old Flash stuff =========================
    // Check if block bad
    // // flash.is_block_bad(0);
    // println!("Done checking bad blocks");
    // // let data = [0xAAAAAAAAu32; 512];
    // // flash.write_page(data);
    // // println!("Done writing first page");#[repr(
    //===================================================

    //println!("Starting Flight routine");
    // info!("Erasing the Flash");
    // handler.handle_erase_flash();

    cfg_if::cfg_if! {
        if #[cfg(feature = "prototype")] {
            let mut led = Pin::new(Port::B, 5, PinMode::Output);
        } else {
            let mut led = Pin::new(Port::A, 8, PinMode::Output);
        }
    }
    led.set_high();

    let mut write_data = [0u8; 2048];
    for i in 0..write_data.len() {
        write_data[i] = i as u8;
    }

    // let last_page = 131070u32;
    let last_page = 127u32;

    // for i in 0..2048 {
    //     if flash.is_block_bad(i) {
    //         error!("Bad block: {}", i);
    //     }
    // }

    info!("Done checking bad blocks");

    match flash.erase_block(last_page) {
        Ok(_) => (),
        Err(_) => error!("Failed to erase flash"),
    }
    match flash.write_page(write_data, last_page) {
        Ok(_) => (),
        Err(_) => error!("Failed to write to flash"),
    }

    loop {
        info!("ADC data: {:?}", adc.read_adc_data().unwrap());

        set_spi_mode(MODE_0);

        #[cfg(not(feature = "prototype"))]
        {
            info!("ADXL 1: 0x{:x}", adxl1.read_register(0x0));
            info!("ADXL 2: 0x{:x}", adxl2.read_register(0x0));
        }

        set_spi_mode(MODE_1);

        let flash_id = flash.get_id();
        info!("Flash ID: {:x}, {:x}", flash_id[0], flash_id[1]);
        info!(
            "Flash status: {:08b} {:08b} {:08b}",
            flash.read_status_register(WinbondStatusReg::One),
            flash.read_status_register(WinbondStatusReg::Two),
            flash.read_status_register(WinbondStatusReg::Three)
        );

        let read_data = flash.read_page(last_page);
        if read_data != write_data {
            error!("Bad data in page: {:x}", read_data);
        } else {
            info!("Flash read successful");
        }

        led.toggle();
        delay_ms(500, ahb_freq);

        // while !uart_regs.isr().read().rxne().bit_is_set(){}

        // println!("UART isr reading: {:b}", uart_regs.isr().read().bits());

        // println!("UART Reading: {:x}", uart_regs.rdr().read().rdr().bits());

        // println!("UART isr reading: {:b}", uart_regs.isr().read().bits());

        // critical_section::with(|cs| {
        //     access_global!(UART, uart, cs);
        //     uart.write(b"A").unwrap();
        //     println!("Sending UART Msg");
        // });
        // delay_ms(2000, ahb_freq);

        // led_pin.toggle();
        // delay_ms(1000, ahb_freq);

        // println!(
        //     "Flash Status Regs {:08b} {:08b} {:08b}",
        //     flash.read_status_register(WinbondStatusReg::One),
        //     flash.read_status_register(WinbondStatusReg::Two),
        //     flash.read_status_register(WinbondStatusReg::Three)
        // );

        //===============================================================

        // Flight routine

        //     let next_state = match state {
        //         FinState::WaitForCommand => handler.handle_wait_for_command(),
        //         FinState::WaitForRecordPulse => handler.handle_wait_for_pulse(),
        //         FinState::RecordData => handler.handle_record_data(),
        //         FinState::StopRecord => handler.handle_stop_recording(),
        //         FinState::EraseFlash => handler.handle_erase_flash(),
        //         FinState::SendStatus => handler.handle_status_cmd(),
        //     };
        //     if let Some(next_state) = next_state {
        //         state = next_state;
        //     }
    }
}

#[interrupt]
fn EXTI15_10() {
    gpio::clear_exti_interrupt(11);
    PULSE_READY.store(true, Ordering::Release);
}

#[interrupt]
fn USART3() {
    critical_section::with(|cs| {
        access_global!(UART, uart, cs);
        let status = uart.regs.isr().read();

        // An overrun blocks further reception until cleared; clear it unconditionally
        // so a missed byte doesn't wedge the line permanently.
        if status.ore().is_overrun() {
            uart.clear_interrupt(UsartInterrupt::Overrun);
        }

        // Pull in a byte as soon as it lands, independent of Idle. This is what lets
        // a command survive being sent with gaps between bytes: each byte is captured
        // the moment it arrives instead of being blocking-read after the fact.
        if status.rxne().is_data_ready() {
            let byte = uart.read_one();
            let len = RX_LEN.borrow(cs).get();
            if len < 4 {
                let mut buf = RX_BUF.borrow(cs).get();
                buf[len] = byte;
                RX_BUF.borrow(cs).set(buf);
                RX_LEN.borrow(cs).set(len + 1);
            }
        }

        // Idle marks the end of a frame: snapshot how many bytes arrived since the
        // last Idle and reset the counter. Only signal a command if we actually got
        // a full 4-byte frame; a short/garbage frame (e.g. a spurious early Idle) is
        // silently dropped instead of being handed to the handler as-is.
        if status.idle().is_idle() {
            uart.clear_interrupt(UsartInterrupt::Idle);
            let len = RX_LEN.borrow(cs).get();
            RX_LEN.borrow(cs).set(0);
            if len == 4 {
                COMMAND_READY.store(true, Ordering::Release);
            }
        }
    });
    info!("wah");
}

// This is the TIM16 and TIM1 interrupt (we are using TIM1), the pac just calls it TIM16 for some reason
#[interrupt]
fn TIM16() {
    unsafe {
        let regs = &(*pac::TIM1::ptr());
        regs.sr().write(|w| w.bits(0xffff_ffff).uif().clear_bit());
    }

    timer::TICK_OVERFLOW_COUNT.fetch_add(1, Ordering::Relaxed);
}

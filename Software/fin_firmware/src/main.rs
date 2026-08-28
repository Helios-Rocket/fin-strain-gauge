#![no_std]
#![no_main]

use adc::ADC;
use cortex_m::delay;
use core::sync::atomic::{AtomicBool, Ordering};
use core::time::Duration;
use defmt::{error, info, println};
use flash::WinbondFlash;
use hal::pac::USART3;
use hal::pac::i2c1::cr2::HEAD10R;
use hal::timer::TimerConfig;
use hal::usart::UsartConfig;
use hal::usart::UsartInterrupt;
use hal::{access_global, gpio, init_globals, make_globals, make_simple_globals, setup_nvic, BaudPeriph};
use hal::{
    clocks::Clocks,
    delay_ms,
    flash::Flash,
    gpio::{Pin, PinMode, Port},
    instant::Instant,
    pac::{self, interrupt},
    timer::{self, Timer},
    usart::Usart,
};
use handlers::StateHandler;
use shared::winbond_flash;
use statemachine::Event;
use statemachine::FinStateMachine;

use defmt_rtt as _;
use panic_probe as _;

use shared::winbond_flash::WinbondStatusReg;
// use shared::communication::CommunicationProtocol

mod adc;
mod flash;
mod handlers;
mod statemachine;

pub static COMMAND_READY: AtomicBool = AtomicBool::new(false);
pub static PULSE_READY: AtomicBool = AtomicBool::new(false);

// Filled in one byte at a time by the USART3 RXNE interrupt. `RX_LEN` is how many
// bytes have landed since the last Idle-line event; the Idle interrupt snapshots
// and resets it, so a frame is only considered valid (and COMMAND_READY set) if
// exactly 4 bytes showed up between idle periods. This decouples byte reception
// from the (slow, blocking) command handler, so a gap between bytes on the wire
// no longer desyncs or corrupts the buffer.
make_simple_globals!((RX_BUF, [u8; 4], [0u8; 4]), (RX_LEN, usize, 0));

make_globals!((UART, Usart<USART3>),);

#[cortex_m_rt::entry]
unsafe fn main() -> ! {
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
    println!("{}", ahb_freq);

    // Setup Peripherals
    let mut led_pin = Pin::new(Port::B, 5, PinMode::Output);
    let mut adc = ADC::new(dp.TIM2, dp.SPI2, &clock_cfg);
    let mut flash = WinbondFlash::new(&mut dp.RCC, dp.QUADSPI, &clock_cfg);

    // Comm pins to Avbay
    let mut pb10 = Pin::new(Port::B, 10, PinMode::Alt(7));
    let mut pb11 = Pin::new(Port::B, 11, PinMode::Alt(7));
    let usart3 = Usart::new(dp.USART3, 1800, UsartConfig::default(), &clock_cfg)
        .expect("Failed to initialize");
    let uart_regs = unsafe{USART3::steal()}; 
    println!("fclk/baud: {}", (USART3::baud(&clock_cfg)/1200) as u16); 
    println!("Baud: {}", uart_regs.brr().read().bits()); 

    init_globals!((UART, usart3));

    setup_nvic!([(USART3, 2), (EXTI15_10, 1), (TIM16, 0)], cp);
    // Enable interrupt
    critical_section::with(|cs| {
        access_global!(UART, uart, cs);
        uart.enable_interrupt(UsartInterrupt::Idle);
        uart.enable_interrupt(UsartInterrupt::ReadNotEmpty);
    });

    let internal_flash = Flash::new(dp.FLASH);
    let mut buf: [u8; 1] = [0u8; 1];
    internal_flash.read(hal::flash::Bank::B1, 31, 0, &mut buf);
    println!("Internal flash buffer: {}", buf);
    let flight_flag_byte = buf[0];
    let flight_flag = false; //flight_flag_byte == 1;

    println!("Flight Flag detected as: {}", flight_flag);

    let timer_config = TimerConfig{update_request_source: hal::timer::UpdateReqSrc::OverUnderFlow, ..Default::default()}; 

    let mut timer = Timer::new_tim1(dp.TIM1, 1000 as f32, timer_config, &clock_cfg);
    timer.enable_interrupt(hal::timer::TimerInterrupt::Update);
    timer.enable();
    println!("Enabled?: {}", timer.is_enabled()); 
    let timer_start = timer.now();
    println!("Timer Start: {}", timer_start.as_nanos()); 

    let mut state = FinStateMachine::new(flight_flag);
    let mut handler = StateHandler::new(
        adc,
        flash,
        internal_flash,
        timer,
        timer_start,
        pb10,
        pb11,
        flight_flag,
    );

    //========= Old Flash stuff =========================
    // Check if block bad
    // // flash.is_block_bad(0);
    // println!("Done checking bad blocks");
    // // let data = [0xAAAAAAAAu32; 512];
    // // flash.write_page(data);
    // // println!("Done writing first page");#[repr(
    //===================================================

    println!("Starting Flight routine");

    loop {

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

        // Flight routine

        info!("about to handle event");
        let event = match state {
            FinStateMachine::WaitForCommand => handler.handle_wait_for_command(),
            FinStateMachine::WaitForRecordPulse => handler.handle_wait_for_pulse(),
            FinStateMachine::RecordData => handler.handle_record_data(),
            FinStateMachine::StopRecord => handler.handle_stop_recording(),
            FinStateMachine::EraseFlash => handler.handle_erase_flash(),
            FinStateMachine::Error => handler.handle_error(),
        };

        state = state.next(event);


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

        // An overrun blocks further reception until cleared; clear it unconditionally
        // so a missed byte doesn't wedge the line permanently.
        if uart.check_status_flag(UsartInterrupt::Overrun) {
            uart.clear_interrupt(UsartInterrupt::Overrun);
        }

        // Pull in a byte as soon as it lands, independent of Idle. This is what lets
        // a command survive being sent with gaps between bytes: each byte is captured
        // the moment it arrives instead of being blocking-read after the fact.
        if uart.check_status_flag(UsartInterrupt::ReadNotEmpty) {
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
        if uart.check_status_flag(UsartInterrupt::Idle) {
            uart.clear_interrupt(UsartInterrupt::Idle);
            let len = RX_LEN.borrow(cs).get();
            RX_LEN.borrow(cs).set(0);
            if len == 4 {
                COMMAND_READY.store(true, Ordering::Release);
            }
        }
    });
    println!("wah");
}

#[interrupt]
fn TIM16() {  // use whatever name matched above
    unsafe {
        let regs = &(*pac::TIM1::ptr());
        regs.sr().write(|w| w.bits(0xffff_ffff).uif().clear_bit());
    }
    
    timer::TICK_OVERFLOW_COUNT.fetch_add(1, Ordering::Relaxed);
}
//Stop recording rising, start recording falling

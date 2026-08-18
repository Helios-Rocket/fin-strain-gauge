#![no_std]
#![no_main]

use adc::ADC;
use hal::pac::i2c1::cr2::HEAD10R;
use hal::timer::TimerConfig;
use core::sync::atomic::{AtomicBool, Ordering};
use core::time::Duration;
use defmt::{error, println};
use flash::WinbondFlash;
use hal::usart::UsartConfig;
use hal::usart::UsartInterrupt;
use shared::winbond_flash;
use statemachine::FinStateMachine;
use statemachine::Event;
use handlers::StateHandler;
use hal::{
    clocks::Clocks,
    delay_ms,
    flash::Flash,
    usart::Usart,
    instant::Instant,
    gpio::{Pin, PinMode, Port},
    pac::{self, interrupt},
    timer::Timer,
};

use defmt_rtt as _;
use panic_probe as _;

use shared::winbond_flash::WinbondStatusReg;
// use shared::communication::CommunicationProtocol

mod statemachine;
mod adc;
mod flash;
mod handlers;

pub static COMMAND_READY: AtomicBool = AtomicBool::new(false);
pub static PULSE_READY: AtomicBool = AtomicBool::new(false);

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

    let ahb_freq = clock_cfg.apb1();
    println!("{}", ahb_freq);

    // Setup Peripherals 
    let mut led_pin = Pin::new(Port::B, 5, PinMode::Output);
    let mut adc = ADC::new(dp.TIM2, dp.SPI2, &clock_cfg);
    // TODO: Set up better write methods for the windbond
    let mut flash = WinbondFlash::new(&mut dp.RCC, dp.QUADSPI, &clock_cfg);
    
    // Comm pins to Avbay
    let mut pb10 = Pin::new(Port::B, 10, PinMode::Alt(7)); 
    let mut pb11 = Pin::new(Port::B, 11, PinMode::Alt(7)); 
    let mut usart3 = Usart::new(dp.USART3, 9600, UsartConfig::default(), &clock_cfg).expect("Failed to initialize"); 

    // Enable interrupt
    usart3.enable_interrupt(UsartInterrupt::Idle); 
    unsafe{cortex_m::peripheral::NVIC::unmask(pac::interrupt::USART3);}

    // TODO: Finish Setup stuff 
    let internal_flash = Flash::new(dp.FLASH); 
    let mut buf: [u8; 32] = [0; 32];
    internal_flash.read(hal::flash::Bank::B1, 0, 0, &mut buf);
    let flight_flag_byte = buf[0];
    let flight_flag = flight_flag_byte != 0;

    let mut timer = Timer::new_tim1(dp.TIM1, ahb_freq as f32, TimerConfig::default(), &clock_cfg); 
    let mut timer_start = timer.now(); 

    let mut state = FinStateMachine::new(flight_flag);
    let mut handler = StateHandler::new(adc, flash, internal_flash, timer, timer_start,usart3, pb10, pb11, flight_flag);


    //========= Old Flash stuff =========================
    // // Check if block bad 
    // for i in 0..512 {
    //     if flash.is_block_bad(i) {
    //         println!("Block {} is bad!", i);
    //     }
    //     else {
    //         println!("Block {} is good!", i);
    //     }
    // }
    // // flash.is_block_bad(0);
    // println!("Done checking bad blocks");
    // // let data = [0xAAAAAAAAu32; 512];
    // // flash.write_page(data);
    // // println!("Done writing first page");#[repr(
    //===================================================


    loop {
        led_pin.toggle();
        delay_ms(1000, ahb_freq);

        // println!(
        //     "Flash Status Regs {:08b} {:08b} {:08b}",
        //     flash.read_status_register(WinbondStatusReg::One),
        //     flash.read_status_register(WinbondStatusReg::Two),
        //     flash.read_status_register(WinbondStatusReg::Three)
        // );

        // Flight routine

        let event = match state{
            FinStateMachine::WaitForCommand => handler.handle_wait_for_command(), 
            FinStateMachine::WaitForRecordPulse => handler.handle_wait_for_pulse(),
            FinStateMachine::RecordData => handler.handle_record_data(), 
            FinStateMachine::StopRecord =>  handler.handle_stop_recording(), 
            FinStateMachine::EraseFlash =>  handler.handle_erase_flash(), 
            FinStateMachine::Error =>  handler.handle_error(),
        }; 

        state = state.next(event);


    }
}

#[interrupt]
fn USART3() {
    cortex_m::peripheral::NVIC::mask(interrupt::USART3);
    COMMAND_READY.store(true, Ordering::Release);
}

#[interrupt]
fn EXTI15_10() {
    cortex_m::peripheral::NVIC::mask(interrupt::EXTI15_10);
    PULSE_READY.store(true, Ordering::Release);
}


//Stop recording rising, start recording falling 
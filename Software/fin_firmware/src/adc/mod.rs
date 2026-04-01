// use core::arch::asm;

use crc_any::CRC;
mod adc_consts;

use adc_consts::*;

pub use adc_consts::{AdcChannel, registers::gain::Gain};
use defmt::{info, println};
use hal::{
    clocks::Clocks,
    delay_ms, delay_us,
    gpio::{Pin, PinMode, Port},
    pac::{self, SPI2, TIM2},
    spi::{Spi, SpiMode},
    timer::{Timer, TimerConfig},
};

use crate::adc::adc_consts::registers::clock::osr::OSR;

#[derive(Copy, Clone)]
pub enum Error {
    CRC { computed: u16 },
}
pub struct ADC {
    spi: Spi<SPI2>,
    pwm: Timer<TIM2>,
    crc16: CRC,
    gains: [Gain; 3],
    cs_pin: Pin,
    rst_pin: Pin,
    clk_freq: u32,
}

impl ADC {
    pub fn new(tim2_reg: TIM2, spi2_reg: SPI2, clk_config: &Clocks) -> Self {
        println!("New ADC"); 
        let mut rst_pin = Pin::new(Port::B, 9, PinMode::Output);
        rst_pin.set_low(); // Make sure ADC is stopped during setup

        let mut cs_pin = Pin::new(Port::B, 12, PinMode::Output);
        cs_pin.set_high();

        let _sck = Pin::new(Port::B, 13, PinMode::Alt(5));
        let _miso = Pin::new(Port::B, 14, PinMode::Alt(5));
        let _mosi = Pin::new(Port::B, 15, PinMode::Alt(5));

        let spi_config = hal::spi::SpiConfig {
            mode: SpiMode::mode1(),
            ..Default::default()
        };

        // Setup SPI bus with clock speed of 8 MHz
        let spi = Spi::new(spi2_reg, spi_config, hal::spi::BaudRate::Div8);

        // Configure PWM for the ADC external clock- 8MHz
        let mut tim2 = Timer::new_tim2(tim2_reg, 8_000_000.0, TimerConfig::default(), clk_config);
        let _pwm_pin = Pin::new(Port::A, 0, PinMode::Alt(1));
        tim2.enable_pwm_output(
            hal::timer::TimChannel::C1,
            hal::timer::OutputCompare::Pwm1,
            0.5,
        );
        tim2.enable();
        
        let clk_freq = clk_config.apb1();
        delay_ms(5, clk_freq);

        rst_pin.set_high(); // Set the (active low) RST pin for the ADC to pull it out of reset and start it running

        delay_ms(5, clk_freq);

        let mut adc = Self {
            spi,
            pwm: tim2,
            crc16: CRC::create_crc(0x1021, 16, 0xffff, 0, false),
            gains: [Gain::Times1; 3],
            cs_pin,
            rst_pin,
            clk_freq,
        };

        let reg = adc.read_register(registers::gain::ADDR);
        adc.gains[AdcChannel::CH0 as usize] = registers::gain::ch0::get(reg);
        adc.gains[AdcChannel::CH1 as usize] = registers::gain::ch1::get(reg);
        adc.gains[AdcChannel::CH2 as usize] = registers::gain::ch2::get(reg);

        adc.set_osr(OSR::Times512);

        adc.set_gain(AdcChannel::CH1, Gain::Times128);
        adc.set_gain(AdcChannel::CH2, Gain::Times128);
        adc.enable_adc_channels();

        // fin.write_register(0x8, 0b0100);
        // fin.write_register(0x13, 0b100);

        adc
    }

    fn set_gain(&mut self, channel: AdcChannel, gain: Gain) {
        use registers::*;

        let reg = self.read_register(gain::ADDR);
        self.write_register(
            gain::ADDR,
            match channel {
                AdcChannel::CH0 => gain::ch0::set(reg, gain),
                AdcChannel::CH1 => gain::ch1::set(reg, gain),
                AdcChannel::CH2 => gain::ch2::set(reg, gain),
            },
        );

        self.gains[channel as usize] = gain;
    }

    fn set_osr(&mut self, osr: OSR) {
        use registers::*;

        let reg = self.read_register(clock::ADDR);
        self.write_register(clock::ADDR, clock::osr::set(reg, osr));
    }

    fn disable_adc_channels(&mut self) {
        use registers::*;
        let reg = self.read_register(clock::ADDR);
        self.write_register(clock::ADDR, reg & clock::DISABLE_ALL_CHANNEL_MASK);
    }

    fn enable_adc_channels(&mut self) {
        use registers::*;
        let reg = self.read_register(clock::ADDR);
        self.write_register(clock::ADDR, reg | clock::ENABLE_ALL_CHANNEL_MASK);
    }

    fn read_register(&mut self, addr: u8) -> u16 {
        let addr: u32 = (addr as u32) & 0x3F;
        let cmd: u32 = (0b101_u32 << 13 | addr << 7) << 8;

        self.assert_cs();

        self.spi
            .write(&cmd.to_be_bytes()[1..])
            .expect("Spi Write Command Failed!");
        for _ in 0..4 {
            self.spi
                .write(&[0_u8, 0, 0])
                .expect("Spi Write Zeros Failed!");
        }

        self.unassert_cs();

        self.assert_cs();

        let mut buf = [0_u8; 15];
        self.spi.transfer(&mut buf).expect("Spi Read Reg Failed!");

        self.unassert_cs();

        let reg_val = u16::from_be_bytes([buf[0], buf[1]]);

        reg_val
    }

    // Get all adc channel values in volts for a single fin
    pub fn read_adc_data(&mut self) -> Result<[f64; 3], Error> {
        println!("Before"); 
        self.assert_cs();
        

        let mut buf = [0_u8; 15];
        // let mut buf = [0b10100010, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        self.spi.transfer(&mut buf).expect("Spi Transfer Failed!");
        println!("Buffer {}", buf); 

        self.unassert_cs();

        self.crc16.reset();
        self.crc16.digest(&buf);
        println!("After"); 

        // info!("{:02x}{:02x}{:02x}", buf[0], buf[1], buf[2]);
        // info!("{:02x}{:02x}{:02x}", buf[3], buf[4], buf[5]);
        // info!("{:02x}{:02x}{:02x}", buf[6], buf[7], buf[8]);
        // info!("{:02x}{:02x}{:02x}", buf[9], buf[10], buf[11]);
        // info!("{:02x}{:02x}{:02x}", buf[12], buf[13], buf[14]);

        let rem = self.crc16.get_crc() as u16;
        println!("CRC Error{}", rem); 
        if rem != 0 {
            Err(Error::CRC { computed: rem })
        } else {
            Ok([
                convert_adc2volts(
                    ((buf[3] as i32) << 24 | (buf[4] as i32) << 16 | (buf[5] as i32) << 8) >> 8,
                    self.gains[AdcChannel::CH0 as usize],
                ),
                convert_adc2volts(
                    ((buf[6] as i32) << 24 | (buf[7] as i32) << 16 | (buf[8] as i32) << 8) >> 8,
                    self.gains[AdcChannel::CH1 as usize],
                ),
                convert_adc2volts(
                    ((buf[9] as i32) << 24 | (buf[10] as i32) << 16 | (buf[11] as i32) << 8) >> 8,
                    self.gains[AdcChannel::CH2 as usize],
                ),
            ])
        }
    }

    fn write_register(&mut self, start_addr: u8, data: u16) {
        let start_addr: u32 = (start_addr as u32) & 0x3f;
        let cmd = (0b011_u32 << 13 | start_addr << 7) << 8;

        self.assert_cs();

        self.spi
            .write(&cmd.to_be_bytes()[1..])
            .expect("Spi Write Command Failed!");
        self.spi
            .write(&((data as u32) << 8).to_be_bytes()[1..])
            .expect("Spi Write Command Data Failed!");
        for _ in 0..3 {
            self.spi
                .write(&[0_u8, 0, 0])
                .expect("Spi Write Zeros Failed!");
        }

        self.unassert_cs();
    }

    fn assert_cs(&mut self) {
        self.cs_pin.set_low();

        delay_us(1, self.clk_freq);
    }

    fn unassert_cs(&mut self) {
        delay_us(1, self.clk_freq);

        self.cs_pin.set_high();
    }
}

pub fn convert_volts2temp(input: f64) -> f64 {
    // let ref_voltage = (1.2 / (((1_u32 << 24) as f64) - 1.0)) * input as f32;
    (input - 0.5) * 100.0
}

pub fn convert_adc2volts(code: i32, gain: Gain) -> f64 {
    (code as f64) * ((2.4 / gain.to_multiplier() as f64) / (1_u32 << 24) as f64)
}

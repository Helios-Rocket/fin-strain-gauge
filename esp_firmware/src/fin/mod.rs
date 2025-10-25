use crc_any::CRC;
mod adc_consts;

use adc_consts::*;

pub use adc_consts::{registers::gain::Gain, AdcChannel};
use esp_hal::{
    gpio::{
        interconnect::{self, PeripheralOutput},
        AnyPin, DriveStrength, Level, Output, OutputConfig, OutputPin,
    },
    mcpwm::{self, operator::PwmPinConfig, timer::PwmWorkingMode, McPwm, PwmPeripheral},
    spi::master::{AnySpi, Config, Spi},
    time::Rate,
    Blocking,
};

#[derive(Copy, Clone)]
pub enum Error {
    CRC { computed: u16 },
}

pub struct GpioPins<'d> {
    cs1: Output<'d>,
    cs2: Output<'d>,
    cs3: Output<'d>,
    cs4: Output<'d>,
    rst: Output<'d>,
}

impl<'d> GpioPins<'d> {
    pub fn new(
        cs1: impl OutputPin + 'd,
        cs2: impl OutputPin + 'd,
        cs3: impl OutputPin + 'd,
        cs4: impl OutputPin + 'd,
        rst: impl OutputPin + 'd,
    ) -> Self {
        Self {
            cs1: Output::new(
                cs1,
                Level::High,
                OutputConfig::default().with_drive_strength(DriveStrength::_20mA),
            ),
            cs2: Output::new(
                cs2,
                Level::High,
                OutputConfig::default().with_drive_strength(DriveStrength::_20mA),
            ),
            cs3: Output::new(
                cs3,
                Level::High,
                OutputConfig::default().with_drive_strength(DriveStrength::_20mA),
            ),
            cs4: Output::new(
                cs4,
                Level::High,
                OutputConfig::default().with_drive_strength(DriveStrength::_20mA),
            ),
            rst: Output::new(
                rst,
                Level::Low,
                OutputConfig::default().with_drive_strength(DriveStrength::_20mA),
            ),
        }
    }
}

pub struct SpiPins<'d> {
    pub miso: interconnect::InputSignal<'d>,
    pub mosi: interconnect::OutputSignal<'d>,
    pub sck: interconnect::OutputSignal<'d>,
}

pub struct PwmPins<'d> {
    pub clk: interconnect::OutputSignal<'d>,
}

pub struct Fins<'d, PWM: PwmPeripheral> {
    spi: Spi<'d, Blocking>,
    pwm: mcpwm::timer::Timer<0, PWM>,
    pins: GpioPins<'d>,
    crc16: CRC,
    gains: [[Gain; 3]; 4],
}

impl<'d, PWM: PwmPeripheral> Fins<'d, PWM> {
    pub fn new(
        mut gpio_pins: GpioPins<'d>,
        spi_pins: SpiPins<'d>,
        spi: AnySpi<'d>,
        pwm_pins: PwmPins<'d>,
        pwm_periph: PWM,
    ) -> Self {
        gpio_pins.rst.set_low(); // Make sure ADC is stopped during setup

        // Setup SPI bus with clock speed of 8 MHz
        let spi = Spi::new(
            spi,
            Config::default()
                .with_frequency(Rate::from_mhz(8))
                .with_mode(esp_hal::spi::Mode::_1),
        )
        .unwrap()
        .with_sck(spi_pins.sck)
        .with_mosi(spi_pins.mosi)
        .with_miso(spi_pins.miso);

        // Configure PWM for the ADC external clock
        let clock_cfg = mcpwm::PeripheralClockConfig::with_frequency(Rate::from_mhz(160)).unwrap();
        let mut mcpwm = McPwm::new(pwm_periph, clock_cfg);
        mcpwm.operator0.set_timer(&mcpwm.timer0);
        let mut pwm_pin = mcpwm
            .operator0
            .with_pin_a(pwm_pins.clk, PwmPinConfig::UP_ACTIVE_HIGH);

        let timer_clock_config = clock_cfg
            .timer_clock_with_frequency(9, PwmWorkingMode::Increase, Rate::from_mhz(4))
            .unwrap();

        mcpwm.timer0.start(timer_clock_config);
        pwm_pin.set_timestamp(5);

        gpio_pins.rst.set_high(); // Set the (active low) RST pin for the ADC to pull it out of reset and start it running

        let mut fin = Self {
            spi,
            pins: gpio_pins,
            pwm: mcpwm.timer0,
            crc16: CRC::create_crc(0x1021, 16, 0xffff, 0, false),
            gains: [[Gain::Times1; 3]; 4],
        };

        for i in 0..4 {
            let reg = fin.read_register(i, registers::gain::ADDR);
            fin.gains[i][AdcChannel::CH0 as usize] = registers::gain::ch0::get(reg);
            fin.gains[i][AdcChannel::CH1 as usize] = registers::gain::ch1::get(reg);
            fin.gains[i][AdcChannel::CH2 as usize] = registers::gain::ch2::get(reg);

            fin.set_gain(i, AdcChannel::CH1, Gain::Times4);
            fin.set_gain(i, AdcChannel::CH2, Gain::Times4);
        }

        // fin.write_register(0x8, 0b0100);
        // fin.write_register(0x13, 0b100);

        fin
    }

    /// Read all adc data from each fin
    pub fn read_all_data(&mut self) -> [Result<[f64; 3], Error>; 4] {
        let mut ret = [Ok([0_f64; 3]); 4];
        for i in 0..4 {
            ret[i] = self.read_adc_data(i);
        }

        ret
    }

    fn set_gain(self: &mut Self, fin_idx: usize, channel: AdcChannel, gain: Gain) {
        use registers::*;

        let reg = self.read_register(fin_idx, gain::ADDR);
        self.write_register(
            fin_idx,
            gain::ADDR,
            reg | match channel {
                AdcChannel::CH0 => gain::ch0::set(gain),
                AdcChannel::CH1 => gain::ch1::set(gain),
                AdcChannel::CH2 => gain::ch2::set(gain),
            },
        );

        self.gains[fin_idx][channel as usize] = gain;
    }

    fn disable_adc_channels(self: &mut Self, fin_idx: usize) {
        use registers::*;
        let reg = self.read_register(fin_idx, clock::ADDR);
        self.write_register(fin_idx, clock::ADDR, reg & clock::DISABLE_ALL_CHANNEL_MASK);
    }

    fn enable_adc_channels(self: &mut Self, fin_idx: usize) {
        use registers::*;
        let reg = self.read_register(fin_idx, clock::ADDR);
        self.write_register(fin_idx, clock::ADDR, reg | clock::ENABLE_ALL_CHANNEL_MASK);
    }

    fn read_register(self: &mut Self, fin_idx: usize, addr: u8) -> u16 {
        let addr: u32 = (addr as u32) & 0x3F;
        let cmd: u32 = (0b101_u32 << 13 | addr << 7) << 8;

        self.assert_cs(fin_idx);

        self.spi
            .write(&cmd.to_be_bytes()[1..])
            .expect("Spi Write Command Failed!");
        for _ in 0..4 {
            self.spi
                .write(&[0_u8, 0, 0])
                .expect("Spi Write Zeros Failed!");
        }

        self.unassert_cs(fin_idx);

        self.assert_cs(fin_idx);

        let mut buf = [0_u8; 15];
        self.spi.transfer(&mut buf).expect("Spi Read Reg Failed!");

        self.unassert_cs(fin_idx);

        let reg_val = u16::from_be_bytes([buf[1], buf[2]]);

        reg_val
    }

    // Get all adc channel values in volts for a single fin
    fn read_adc_data(self: &mut Self, fin_idx: usize) -> Result<[f64; 3], Error> {
        self.assert_cs(fin_idx);

        let mut buf = [0_u8; 15];
        self.spi.transfer(&mut buf).expect("Spi Transfer Failed!");

        self.unassert_cs(fin_idx);

        self.crc16.reset();
        self.crc16.digest(&buf);

        let rem = self.crc16.get_crc() as u16;
        if rem != 0 {
            Err(Error::CRC { computed: rem })
        } else {
            Ok([
                convert_adc2volts(
                    ((buf[3] as i32) << 24 | (buf[4] as i32) << 16 | (buf[5] as i32) << 8) >> 8,
                    self.gains[fin_idx][AdcChannel::CH0 as usize],
                ),
                convert_adc2volts(
                    ((buf[6] as i32) << 24 | (buf[7] as i32) << 16 | (buf[8] as i32) << 8) >> 8,
                    self.gains[fin_idx][AdcChannel::CH1 as usize],
                ),
                convert_adc2volts(
                    ((buf[9] as i32) << 24 | (buf[10] as i32) << 16 | (buf[11] as i32) << 8) >> 8,
                    self.gains[fin_idx][AdcChannel::CH2 as usize],
                ),
            ])
        }
    }

    fn write_register(self: &mut Self, fin_idx: usize, start_addr: u8, data: u16) {
        let start_addr: u32 = (start_addr as u32) & 0x3f;
        let cmd = (0b011_u32 << 13 | start_addr << 7) << 8;

        self.assert_cs(fin_idx);

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

        self.unassert_cs(fin_idx);
    }

    fn assert_cs(&mut self, fin_idx: usize) {
        match fin_idx {
            0 => self.pins.cs1.set_low(),
            1 => self.pins.cs2.set_low(),
            2 => self.pins.cs3.set_low(),
            3 => self.pins.cs4.set_low(),
            _ => panic!("Invalid fin idx {}", fin_idx),
        }
    }

    fn unassert_cs(&mut self, fin_idx: usize) {
        match fin_idx {
            0 => self.pins.cs1.set_high(),
            1 => self.pins.cs2.set_high(),
            2 => self.pins.cs3.set_high(),
            3 => self.pins.cs4.set_high(),
            _ => panic!("Invalid fin idx {}", fin_idx),
        }
    }
}

pub fn convert_volts2temp(input: f64) -> f64 {
    // let ref_voltage = (1.2 / (((1_u32 << 24) as f64) - 1.0)) * input as f32;
    (input - 0.5) * 100.0
}

pub fn convert_adc2volts(code: i32, gain: Gain) -> f64 {
    (code as f64) * ((2.4 / gain.to_multiplier() as f64) / (1_u32 << 24) as f64)
}

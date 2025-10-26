use crc_any::CRC;
use teensy4_bsp::{
    board,
    hal::{
        self, flexpwm, gpio,
        iomuxc::{
            self,
            consts::Const,
            lpspi::{Sck, Sdi, Sdo},
        },
        lpspi::{Lpspi, SamplePoint, Transaction, MODE_1},
    },
    ral,
};
mod adc_consts;

use adc_consts::*;

pub use adc_consts::{registers::gain::Gain, AdcChannel};

#[derive(Copy, Clone)]
pub enum Error {
    CRC { computed: u16 },
}

pub struct Pins<SpiMiso, SpiMosi, SpiSck, SpiCs1, SpiCs2, SpiCs3, SpiCs4, Clk, Rst> {
    pub miso: SpiMiso,
    pub mosi: SpiMosi,
    pub sck: SpiSck,
    pub cs1: gpio::Output<SpiCs1>,
    pub cs2: gpio::Output<SpiCs2>,
    pub cs3: gpio::Output<SpiCs3>,
    pub cs4: gpio::Output<SpiCs4>,
    pub rst: gpio::Output<Rst>,
    pub clk: flexpwm::Output<Clk>,
}

pub struct Fins<P, const SPI_N: u8, const PWM_N: u8, const PWM_M: u8> {
    spi: Lpspi<(), SPI_N>,
    pwm: flexpwm::Pwm<PWM_N>,
    pwm_submod: flexpwm::Submodule<PWM_N, PWM_M>,
    pins: P,
    crc16: CRC,
    gains: [[Gain; 3]; 4],
}

impl<
        SpiMiso,
        SpiMosi,
        SpiSck,
        SpiCs1,
        SpiCs2,
        SpiCs3,
        SpiCs4,
        Clk,
        Rst,
        const SPI_N: u8,
        const PWM_N: u8,
        const PWM_M: u8,
    >
    Fins<
        Pins<SpiMiso, SpiMosi, SpiSck, SpiCs1, SpiCs2, SpiCs3, SpiCs4, Clk, Rst>,
        SPI_N,
        PWM_N,
        PWM_M,
    >
where
    SpiMiso: iomuxc::lpspi::Pin<Signal = Sdi, Module = Const<SPI_N>>,
    SpiMosi: iomuxc::lpspi::Pin<Signal = Sdo, Module = Const<SPI_N>>,
    SpiSck: iomuxc::lpspi::Pin<Signal = Sck, Module = Const<SPI_N>>,
    Clk: iomuxc::flexpwm::Pin<Module = Const<PWM_N>, Submodule = Const<PWM_M>>,
    ral::lpspi::Instance<SPI_N>: ral::Valid,
{
    pub fn new(
        mut pins: Pins<SpiMiso, SpiMosi, SpiSck, SpiCs1, SpiCs2, SpiCs3, SpiCs4, Clk, Rst>,
        lpspi: ral::lpspi::Instance<SPI_N>,
        mut pwm: flexpwm::Pwm<PWM_N>,
        mut pwm_submod: flexpwm::Submodule<PWM_N, PWM_M>,
    ) -> Self {
        pins.rst.clear(); // Make sure ADC is stopped during setup

        // Setup SPI bus
        iomuxc::lpspi::prepare(&mut pins.miso);
        iomuxc::lpspi::prepare(&mut pins.mosi);
        iomuxc::lpspi::prepare(&mut pins.sck);
        let mut spi = hal::lpspi::Lpspi::without_pins(lpspi);

        // Configure SPI clock speed to be 1 MHz
        spi.disabled(|spi| {
            spi.set_clock_hz(board::LPSPI_FREQUENCY, 8_000_000);
            spi.set_sample_point(SamplePoint::Edge);
        });
        spi.set_mode(MODE_1);

        // Configure PWM for the ADC external clock
        pwm_submod.set_debug_enable(true);
        pwm_submod.set_wait_enable(true);
        // PWM clock = IPG clock (150 MHz) / 1 = 150 MHz
        pwm_submod.set_clock_select(flexpwm::ClockSelect::Ipg);
        pwm_submod.set_prescaler(flexpwm::Prescaler::Prescaler1);
        // Just use a single PWM output
        pwm_submod.set_pair_operation(flexpwm::PairOperation::Independent);
        pwm_submod.set_load_mode(flexpwm::LoadMode::reload_full());
        // Load register changes to the peripheral as fast as possible
        pwm_submod.set_load_frequency(1);

        pwm_submod.set_initial_count(&pwm, 0);
        // Set the period to 38 counts = 38 * (1 / 150 MHz) = 253.33 ns
        pwm_submod.set_value(flexpwm::FULL_RELOAD_VALUE_REGISTER, 36);
        // Set the duty cycle to 50% by turning on at 38 / 2 = 19 counts
        pins.clk.set_turn_on(&pwm_submod, 18);
        // Turn off on wrap around
        pins.clk.set_turn_off(&pwm_submod, 36);
        pins.clk.set_output_enable(&mut pwm, true);
        // Load the values into the PWM registers.
        pwm_submod.set_load_ok(&mut pwm);

        pwm_submod.set_running(&mut pwm, true);

        pins.rst.set(); // Set the (active low) RST pin for the ADC to pull it out of reset and start it running

        let mut fin = Self {
            spi,
            pwm,
            pwm_submod,
            pins,
            crc16: CRC::create_crc(0x1021, 16, 0xffff, 0, false),
            gains: [[Gain::Times1; 3]; 4],
        };

<<<<<<< HEAD
        fin.set_gain(AdcChannel::CH2, Gain::Times4);
=======
        for i in 0..4 {
            let reg = fin.read_register(i, registers::gain::ADDR);
            fin.gains[i][AdcChannel::CH0 as usize] = registers::gain::ch0::get(reg);
            fin.gains[i][AdcChannel::CH1 as usize] = registers::gain::ch1::get(reg);
            fin.gains[i][AdcChannel::CH2 as usize] = registers::gain::ch2::get(reg);

            fin.set_gain(i, AdcChannel::CH1, Gain::Times4);
            fin.set_gain(i, AdcChannel::CH2, Gain::Times4);
        }
>>>>>>> 718144128ca5d7f674a413a9bf8eef56e4676822

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
        let transaction = Transaction::new(24).unwrap();

        self.assert_cs(fin_idx);

        self.spi.enqueue_transaction(&transaction);
        self.spi.enqueue_data(cmd);
        for _ in 0..4 {
            self.spi.enqueue_transaction(&transaction);
            self.spi.enqueue_data(0);
        }
        self.spi.flush().unwrap();

        self.unassert_cs(fin_idx);

        self.spi.clear_fifo(hal::lpspi::Direction::Rx);

        self.assert_cs(fin_idx);

        for _ in 0..5 {
            self.spi.enqueue_transaction(&transaction);
            self.spi.enqueue_data(0);
        }
        self.spi.flush().unwrap();

        self.unassert_cs(fin_idx);

        let reg_val = (self.spi.read_data().unwrap() >> 8) as u16;

        self.spi.clear_fifo(hal::lpspi::Direction::Rx);

        reg_val
    }

    // Get all adc channel values in volts for a single fin
    fn read_adc_data(self: &mut Self, fin_idx: usize) -> Result<[f64; 3], Error> {
        let transaction = Transaction::new(24).unwrap();

        self.assert_cs(fin_idx);

        for _ in 0..5 {
            self.spi.enqueue_transaction(&transaction);
            self.spi.enqueue_data(0);
        }

        self.spi.flush().unwrap();

        self.unassert_cs(fin_idx);

        let output = [
            self.spi.read_data().unwrap().to_be_bytes(),
            self.spi.read_data().unwrap().to_be_bytes(),
            self.spi.read_data().unwrap().to_be_bytes(),
            self.spi.read_data().unwrap().to_be_bytes(),
            self.spi.read_data().unwrap().to_be_bytes(),
        ];

        self.crc16.reset();
        for word in output {
            for &byte in word[1..].iter() {
                self.crc16.digest(&[byte]);
            }
        }

        let rem = self.crc16.get_crc() as u16;
        if rem != 0 {
            Err(Error::CRC { computed: rem })
        } else {
            Ok([
                convert_adc2volts(
                    ((output[1][1] as i32) << 24
                        | (output[1][2] as i32) << 16
                        | (output[1][3] as i32) << 8)
                        >> 8,
                    self.gains[fin_idx][AdcChannel::CH0 as usize],
                ),
                convert_adc2volts(
                    ((output[2][1] as i32) << 24
                        | (output[2][2] as i32) << 16
                        | (output[2][3] as i32) << 8)
                        >> 8,
                    self.gains[fin_idx][AdcChannel::CH1 as usize],
                ),
                convert_adc2volts(
                    ((output[3][1] as i32) << 24
                        | (output[3][2] as i32) << 16
                        | (output[3][3] as i32) << 8)
                        >> 8,
                    self.gains[fin_idx][AdcChannel::CH2 as usize],
                ),
            ])
        }
    }

    fn write_register(self: &mut Self, fin_idx: usize, start_addr: u8, data: u16) {
        let start_addr: u32 = (start_addr as u32) & 0x3f;
        let cmd = (0b011_u32 << 13 | start_addr << 7) << 8;
        let transaction = Transaction::new(24).unwrap();

        self.assert_cs(fin_idx);

        self.spi.enqueue_transaction(&transaction);
        self.spi.enqueue_data(cmd);
        self.spi.enqueue_transaction(&transaction);
        self.spi.enqueue_data((data as u32) << 8);
        for _ in 0..3 {
            self.spi.enqueue_transaction(&transaction);
            self.spi.enqueue_data(0);
        }
        self.spi.flush().unwrap();

        self.unassert_cs(fin_idx);

        self.spi.clear_fifo(hal::lpspi::Direction::Rx);
    }

    fn assert_cs(&self, fin_idx: usize) {
        match fin_idx {
            0 => self.pins.cs1.clear(),
            1 => self.pins.cs2.clear(),
            2 => self.pins.cs3.clear(),
            3 => self.pins.cs4.clear(),
            _ => panic!("Invalid fin idx {}", fin_idx),
        }
    }

    fn unassert_cs(&self, fin_idx: usize) {
        match fin_idx {
            0 => self.pins.cs1.set(),
            1 => self.pins.cs2.set(),
            2 => self.pins.cs3.set(),
            3 => self.pins.cs4.set(),
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

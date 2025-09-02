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

pub(crate) use adc_consts::*;

pub struct Pins<SpiMiso, SpiMosi, SpiSck, SpiCs1, Clk, Rst> {
    pub miso: SpiMiso,
    pub mosi: SpiMosi,
    pub sck: SpiSck,
    pub cs1: gpio::Output<SpiCs1>,
    pub rst: gpio::Output<Rst>,
    pub clk: flexpwm::Output<Clk>,
}

pub struct Fins<P, const SPI_N: u8, const PWM_N: u8, const PWM_M: u8> {
    spi: Lpspi<(), SPI_N>,
    pwm: flexpwm::Pwm<PWM_N>,
    pwm_submod: flexpwm::Submodule<PWM_N, PWM_M>,
    pins: P,
}

impl<
        SpiMiso,
        SpiMosi,
        SpiSck,
        SpiCs1,
        Clk,
        Rst,
        const SPI_N: u8,
        const PWM_N: u8,
        const PWM_M: u8,
    > Fins<Pins<SpiMiso, SpiMosi, SpiSck, SpiCs1, Clk, Rst>, SPI_N, PWM_N, PWM_M>
where
    SpiMiso: iomuxc::lpspi::Pin<Signal = Sdi, Module = Const<SPI_N>>,
    SpiMosi: iomuxc::lpspi::Pin<Signal = Sdo, Module = Const<SPI_N>>,
    SpiSck: iomuxc::lpspi::Pin<Signal = Sck, Module = Const<SPI_N>>,
    Clk: iomuxc::flexpwm::Pin<Module = Const<PWM_N>, Submodule = Const<PWM_M>>,
    ral::lpspi::Instance<SPI_N>: ral::Valid,
{
    pub fn new(
        mut pins: Pins<SpiMiso, SpiMosi, SpiSck, SpiCs1, Clk, Rst>,
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
            spi.set_clock_hz(board::LPSPI_FREQUENCY, 1_000_000);
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

        Self {
            spi,
            pwm,
            pwm_submod,
            pins,
        }
    }

    pub fn disable_adc_channels(self: &mut Self) {
        use adc_consts::registers::*;
        let reg = self.read_register(clock::ADDR);
        self.write_register(clock::ADDR, reg & clock::DISABLE_ALL_CHANNEL_MASK);
    }

    pub fn enable_adc_channels(self: &mut Self) {
        use adc_consts::registers::*;
        let reg = self.read_register(clock::ADDR);
        self.write_register(clock::ADDR, reg | clock::ENABLE_ALL_CHANNEL_MASK);
    }

    pub fn read_register(self: &mut Self, addr: u8) -> u16 {
        let addr: u32 = (addr as u32) & 0x3F;
        let cmd: u32 = (0b101_u32 << 13 | addr << 7) << 8;
        let transaction = Transaction::new(24).unwrap();
        self.pins.cs1.clear();
        self.spi.enqueue_transaction(&transaction);
        self.spi.enqueue_data(cmd);
        for _ in 0..4 {
            self.spi.enqueue_transaction(&transaction);
            self.spi.enqueue_data(0);
        }
        self.spi.flush().unwrap();
        self.pins.cs1.set();

        self.spi.clear_fifo(hal::lpspi::Direction::Rx);

        self.pins.cs1.clear();
        for _ in 0..5 {
            self.spi.enqueue_transaction(&transaction);
            self.spi.enqueue_data(0);
        }
        self.spi.flush().unwrap();
        self.pins.cs1.set();

        let reg_val = (self.spi.read_data().unwrap() >> 8) as u16;

        self.spi.clear_fifo(hal::lpspi::Direction::Rx);

        reg_val
    }

    pub fn read_adc_data(self: &mut Self) -> [i32; 3] {
        let transaction = Transaction::new(24).unwrap();
        self.pins.cs1.clear();
        for _ in 0..5 {
            self.spi.enqueue_transaction(&transaction);
            self.spi.enqueue_data(0);
        }

        self.spi.flush().unwrap();
        self.pins.cs1.set();

        let output = [
            self.spi.read_data().unwrap().to_be_bytes(),
            self.spi.read_data().unwrap().to_be_bytes(),
            self.spi.read_data().unwrap().to_be_bytes(),
            self.spi.read_data().unwrap().to_be_bytes(),
            self.spi.read_data().unwrap().to_be_bytes(),
        ];

        log::info!(
            "Status: {:#018b}",
            (output[0][1] as u32) << 8 | (output[0][2] as u32)
        );

        [
            (((output[1][1] as i32) << 24
                | (output[1][2] as i32) << 16
                | (output[1][3] as i32) << 8)
                >> 8),
            (((output[2][1] as i32) << 24
                | (output[2][2] as i32) << 16
                | (output[2][3] as i32) << 8)
                >> 8),
            (((output[3][1] as i32) << 24
                | (output[3][2] as i32) << 16
                | (output[3][3] as i32) << 8)
                >> 8),
        ]
    }

    pub fn write_register(self: &mut Self, start_addr: u8, data: u16) {
        let start_addr: u32 = (start_addr as u32) & 0x3f;
        let cmd = (0b011_u32 << 13 | start_addr << 7) << 8;
        let transaction = Transaction::new(24).unwrap();

        self.pins.cs1.clear();
        self.spi.enqueue_transaction(&transaction);
        self.spi.enqueue_data(cmd);
        self.spi.enqueue_transaction(&transaction);
        self.spi.enqueue_data((data as u32) << 8);
        for _ in 0..3 {
            self.spi.enqueue_transaction(&transaction);
            self.spi.enqueue_data(0);
        }
        self.spi.flush().unwrap();
        self.pins.cs1.set();

        self.spi.clear_fifo(hal::lpspi::Direction::Rx);
    }
}

pub fn convert_volts2temp(input: f64) -> f64 {
    // let ref_voltage = (1.2 / (((1_u32 << 24) as f64) - 1.0)) * input as f32;
    (input - 0.5) * 100.0
}

pub fn convert_adc2volts(code: i32) -> f64 {
    (code as f64) * (2.4 / (1_u32 << 24) as f64) * 2.0
}

use core::marker::PhantomData;

use cortex_m::prelude::_embedded_hal_blocking_spi_Transfer;
use teensy4_bsp::{
    board,
    hal::{
        self, flexpwm, gpio,
        iomuxc::{
            self,
            consts::Const,
            lpspi::{Sck, Sdi, Sdo},
        },
        lpspi::{Lpspi, SamplePoint, MODE_1},
    },
    ral,
};

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
    _pins: PhantomData<P>,
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
        pins: &mut Pins<SpiMiso, SpiMosi, SpiSck, SpiCs1, Clk, Rst>,
        lpspi: ral::lpspi::Instance<SPI_N>,
        mut pwm: flexpwm::Pwm<PWM_N>,
        mut pwm_submod: flexpwm::Submodule<PWM_N, PWM_M>,
    ) -> Self {
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
        pwm_submod.set_value(flexpwm::FULL_RELOAD_VALUE_REGISTER, 38);
        // Set the duty cycle to 50% by turning on at 38 / 2 = 19 counts
        pins.clk.set_turn_on(&pwm_submod, 19);
        // Turn off on wrap around
        pins.clk.set_turn_off(&pwm_submod, 38);
        pins.clk.set_output_enable(&mut pwm, true);
        // Load the values into the PWM registers.
        pwm_submod.set_load_ok(&mut pwm);

        pwm_submod.set_running(&mut pwm, true);

        pins.rst.set(); // Set the (active low) RST pin for the ADC to pull it out of reset and start it running

        Self {
            spi,
            pwm,
            pwm_submod,
            _pins: PhantomData,
        }
    }

   
    // fn write_registers() {}

}

pub fn convert_adc2temp(input: u32) -> f32 {
    let ref_voltage = (1.2 / (((1_u32 << 24) as f32) - 1.0)) * input as f32;
    (ref_voltage - 500.0) * 100.0
}

pub fn read_registers(spi: &mut Lpspi<(), 4>, start_addr: u8) -> [u32; 3] {
    let start_addr: u32 = (start_addr as u32) & 0x3F;
    let word: u32 = 0;
    log::info!("{:#018b}", word);
    spi.transfer(&mut word.to_be_bytes()[0..=2]).unwrap();
    let mut dummy = [0_u8; 12];
    let output = spi.transfer(&mut dummy).unwrap();
    [
        (output[3] as u32) << 16 | (output[4] as u32) << 8 | (output[5] as u32),
        (output[6] as u32) << 16 | (output[7] as u32) << 8 | (output[8] as u32),
        (output[9] as u32) << 16 | (output[10] as u32) << 8 | (output[11] as u32),
    ]
}

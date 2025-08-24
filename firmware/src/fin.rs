use core::marker::PhantomData;

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
}

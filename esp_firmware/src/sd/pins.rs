use esp_hal::gpio::{
    interconnect::{self, PeripheralOutput, PeripheralSignal},
    DriveMode, InputConfig, InputSignal, OutputConfig, OutputSignal, Pull,
};

pub struct Pins<'d> {
    clk: interconnect::OutputSignal<'d>,
    cmd: interconnect::OutputSignal<'d>,
    data0: interconnect::OutputSignal<'d>,
    data1: interconnect::OutputSignal<'d>,
    data2: interconnect::OutputSignal<'d>,
    data3: interconnect::OutputSignal<'d>,
}

impl<'d> Pins<'d> {
    pub(crate) fn set_cmd_push_pull(&mut self) {
        self.cmd.apply_output_config(&OutputConfig::default());
    }
}

#[derive(Default)]
pub struct PinsBuilder<'d> {
    clk: Option<interconnect::OutputSignal<'d>>,
    cmd: Option<interconnect::OutputSignal<'d>>,
    data0: Option<interconnect::OutputSignal<'d>>,
    data1: Option<interconnect::OutputSignal<'d>>,
    data2: Option<interconnect::OutputSignal<'d>>,
    data3: Option<interconnect::OutputSignal<'d>>,
}

impl<'d> PinsBuilder<'d> {
    pub fn build(self) -> Pins<'d> {
        Pins {
            clk: self.clk.expect("Must set clk pin using with_clk!"),
            cmd: self.cmd.expect("Must set cmd pin using with_cmd!"),
            data0: self.data0.expect("Must set data0 pin using with_data!"),
            data1: self.data1.expect("Must set data1 pin using with_data!"),
            data2: self.data2.expect("Must set data2 pin using with_data!"),
            data3: self.data3.expect("Must set data3 pin using with_data!"),
        }
    }

    pub fn with_clk(mut self, clk: impl PeripheralOutput<'d>) -> Self {
        let clk = self.clk.insert(clk.into());

        let signal = OutputSignal::SDHOST_CCLK_OUT_1;

        clk.apply_output_config(&OutputConfig::default());

        clk.set_output_enable(true);

        signal.connect_to(clk);
        clk.connect_peripheral_to_output(signal);

        self
    }

    pub fn with_cmd(mut self, cmd: impl PeripheralOutput<'d>) -> Self {
        let cmd = self.cmd.insert(cmd.into());

        let out_signal = OutputSignal::SDHOST_CCMD_OUT_1;
        let in_signal = InputSignal::SDHOST_CCMD_IN_1;

        cmd.apply_output_config(
            &OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::Up),
        );

        cmd.set_output_enable(false);
        cmd.set_input_enable(true);

        in_signal.connect_to(cmd);
        cmd.connect_input_to_peripheral(in_signal);

        out_signal.connect_to(cmd);
        cmd.connect_peripheral_to_output(out_signal);

        self
    }

    pub fn with_data(mut self, n: usize, data: impl PeripheralOutput<'d>) -> Self {
        let (pin, in_signal, out_signal) = match n {
            0 => (
                self.data0.insert(data.into()),
                InputSignal::SDHOST_CDATA_IN_10,
                OutputSignal::SDHOST_CDATA_OUT_10,
            ),
            1 => (
                self.data1.insert(data.into()),
                InputSignal::SDHOST_CDATA_IN_11,
                OutputSignal::SDHOST_CDATA_OUT_11,
            ),
            2 => (
                self.data2.insert(data.into()),
                InputSignal::SDHOST_CDATA_IN_12,
                OutputSignal::SDHOST_CDATA_OUT_12,
            ),
            3 => (
                self.data3.insert(data.into()),
                InputSignal::SDHOST_CDATA_IN_13,
                OutputSignal::SDHOST_CDATA_OUT_13,
            ),
            _ => panic!("Invalid data pin (data{})", n),
        };

        pin.apply_input_config(&InputConfig::default().with_pull(Pull::Up));
        pin.apply_output_config(&OutputConfig::default().with_pull(Pull::Up));

        pin.set_output_enable(false);
        pin.set_input_enable(true);

        in_signal.connect_to(pin);
        pin.connect_input_to_peripheral(in_signal);

        out_signal.connect_to(pin);
        pin.connect_peripheral_to_output(out_signal);

        self
    }
}

use esp_hal::{
    gpio::{
        dedicated::{DedicatedGpio, DedicatedGpioOutput, DedicatedGpioOutputChannel},
        AnyPin, Level, NoPin, Output, OutputConfig,
    },
    peripherals::GPIO_DEDICATED,
    time::{Duration, Instant},
    uart::{self, Uart},
    Blocking,
};

pub struct FinPins<'d> {
    rx: AnyPin<'d>,
    tx: AnyPin<'d>,
}

pub struct Fins<'d> {
    raw_pins: [FinPins<'d>; 4],

    uart: Option<Uart<'d, Blocking>>,

    _dgpio_chans: DedicatedGpio<'d>,
    synced_output: Option<DedicatedGpioOutput<'d>>,
}

impl<'d> Fins<'d> {
    pub fn new(
        uart: impl uart::Instance + 'd,
        dgpio: GPIO_DEDICATED<'d>,
        fin0_rx: AnyPin<'d>,
        fin0_tx: AnyPin<'d>,
        fin1_rx: AnyPin<'d>,
        fin1_tx: AnyPin<'d>,
        fin2_rx: AnyPin<'d>,
        fin2_tx: AnyPin<'d>,
        fin3_rx: AnyPin<'d>,
        fin3_tx: AnyPin<'d>,
    ) -> Self {
        Self {
            raw_pins: [
                FinPins {
                    rx: fin0_rx,
                    tx: fin0_tx,
                },
                FinPins {
                    rx: fin1_rx,
                    tx: fin1_tx,
                },
                FinPins {
                    rx: fin2_rx,
                    tx: fin2_tx,
                },
                FinPins {
                    rx: fin3_rx,
                    tx: fin3_tx,
                },
            ],

            uart: Uart::new(uart, uart::Config::default().with_baudrate(1200)).ok(),

            _dgpio_chans: DedicatedGpio::new(dgpio),
            synced_output: None,
        }
    }

    // SAFETY NOTE: the following functions use `clone_unchecked` in a bunch of places because the compiler cannot know that there is only one copy of each peripheral in use, due to the dynamic nature of this code.
    // We ensure safety by always dropping all other structs that could be holding a peripheral before using it. `Option`s are used to do this dropping since you can't actually `drop` a struct field without moving the whole struct.

    fn select_fin(&mut self, idx: usize) {
        // NOTE: this disconnects the pins from the dedicated gpio peripheral and disables the corresponding output drivers
        self.synced_output = None;

        let u = self
            .uart
            .take()
            .unwrap()
            // Drops the previous pins used for uart (if any).
            .with_rx(unsafe { self.raw_pins[idx].rx.clone_unchecked() })
            .with_tx(unsafe { self.raw_pins[idx].tx.clone_unchecked() });
        self.uart = Some(u);
    }

    fn enable_synced(&mut self) {
        // Drops any pin peripherals used for uart.
        let u = self.uart.take().unwrap().with_rx(NoPin).with_tx(NoPin);
        self.uart = Some(u);
        // NOTE: this disconnects the pins from the dedicated gpio peripheral and disables the corresponding output drivers
        self.synced_output = None;

        let [p0, p1, p2, p3] = self.raw_pins.each_mut().map(|p| {
            Output::new(
                unsafe { p.tx.clone_unchecked() }, // SAFETY: The top of this function drops the two things that could be using the pin peripherals
                Level::High,
                OutputConfig::default().with_drive_strength(esp_hal::gpio::DriveStrength::_40mA),
            )
        });

        // SAFETY: the DedicatedGpio peripheral has been setup, and we own it, so nothing else should be using this (or any) channel
        let mut dedicated_output =
            DedicatedGpioOutput::new(unsafe { DedicatedGpioOutputChannel::<0>::steal() });
        dedicated_output.set_level(Level::High); // Since UART is idle High, we will use a falling edge to trigger the synced start, and thus we should ensure the pins stay high when attached to dedicated gpio
        self.synced_output = Some(
            dedicated_output
                .with_pin(p0)
                .with_pin(p1)
                .with_pin(p2)
                .with_pin(p3),
        );
    }

    pub fn start_fins(&mut self) {
        for i in 0..4 {
            self.select_fin(i);

            let uart = self.uart.as_mut().unwrap();
            uart.write(match i {
                0 => b"abc",
                1 => b"def",
                2 => b"hij",
                3 => b"klm",
                _ => unreachable!(),
            })
            .unwrap();
            uart.flush().unwrap();
        }

        self.enable_synced();

        let t = Instant::now();
        self.synced_output.as_mut().unwrap().set_level(Level::Low);
        while t.elapsed() < Duration::from_millis(10) {}
        self.synced_output.as_mut().unwrap().set_level(Level::High);
        // while t.elapsed() < Duration::from_millis(10) {}
    }
}

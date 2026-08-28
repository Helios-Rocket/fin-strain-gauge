use defmt::{error, info};
use esp_hal::{
    delay::Delay,
    gpio::{
        dedicated::{DedicatedGpio, DedicatedGpioOutput, DedicatedGpioOutputChannel},
        AnyPin, Level, NoPin, Output, OutputConfig,
    },
    peripherals::GPIO_DEDICATED,
    time::{Duration, Instant},
    uart::{self, RxError, TxError, Uart},
    Blocking,
};
use num_enum::TryFromPrimitiveError;
use shared::fin_commands::FinCommands;

pub struct FinsIdle(());
pub struct FinsStarted(());

#[derive(Clone, Copy, PartialEq, defmt::Format, Debug)]
pub enum FinCommsError {
    NoUart,
    UartTx(TxError),
    UartRx(RxError),
    InvalidCommand(u8),
    BadCommandHeader([u8; 3]),
    Timeout,
}

impl From<TxError> for FinCommsError {
    fn from(value: TxError) -> Self {
        Self::UartTx(value)
    }
}

impl From<RxError> for FinCommsError {
    fn from(value: RxError) -> Self {
        Self::UartRx(value)
    }
}

impl From<TryFromPrimitiveError<FinCommands>> for FinCommsError {
    fn from(value: TryFromPrimitiveError<FinCommands>) -> Self {
        Self::InvalidCommand(value.number)
    }
}

pub struct FinPins<'d> {
    rx: AnyPin<'d>,
    tx: AnyPin<'d>,
}

pub struct Fins<'d> {
    raw_pins: [FinPins<'d>; 4],

    uart: Option<Uart<'d, Blocking>>,

    delay: Delay,

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
    ) -> Result<(Self, FinsIdle), FinCommsError> {
        let mut this = Self {
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

            uart: Some(
                Uart::new(
                    uart,
                    uart::Config::default()
                        .with_baudrate(1800)
                        .with_stop_bits(uart::StopBits::_1),
                )
                .ok()
                .ok_or(FinCommsError::NoUart)?,
            ),

            delay: Delay::new(),

            _dgpio_chans: DedicatedGpio::new(dgpio),
            synced_output: None,
        };

        this.enable_synced();

        Ok((this, FinsIdle(())))
    }

    // SAFETY NOTE: the following functions use `clone_unchecked` in a bunch of places because the compiler cannot know that there is only one copy of each peripheral in use, due to the dynamic nature of this code.
    // We ensure safety by always dropping all other structs that could be holding a peripheral before using it. `Option`s are used to do this dropping since you can't actually `drop` a struct field without moving the whole struct.

    fn select_fin(&mut self, idx: usize) {
        // Drops any pin peripherals used for uart.
        // IMPORTANT: even though directly putting in the new rx and tx pins would also drop the old ones,
        // we must do it this way in case the old and new pins are actually the same, in which case
        // the drop will undo the effects of adding them, since the drop runs last.
        let u = self.uart.take().unwrap().with_rx(NoPin).with_tx(NoPin);
        self.uart = Some(u);
        // NOTE: this disconnects the pins from the dedicated gpio peripheral and disables the corresponding output drivers
        self.synced_output = None;

        let u = self
            .uart
            .take()
            .unwrap()
            .with_rx(unsafe { self.raw_pins[idx].rx.clone_unchecked() })
            .with_tx(unsafe { self.raw_pins[idx].tx.clone_unchecked() });
        self.uart = Some(u);
    }

    fn enable_synced(&mut self) -> FinsStarted {
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

        FinsStarted(())
    }

    fn send_fin_cmd(&mut self, cmd: FinCommands) -> Result<(), FinCommsError> {
        let mut resp = [0u8; 64];
        let uart = self.uart.as_mut().unwrap();
        uart.write(b"FIN")?;
        uart.write(&[cmd as u8])?;
        uart.flush()?;

        let send_time = Instant::now();

        let mut i = 0;
        loop {
            if send_time.elapsed() >= Duration::from_millis(1000) {
                return Err(FinCommsError::Timeout);
            }

            if let Ok(n) = uart.read_buffered(&mut resp[i..]) {
                i += n;
                if i >= 4 {
                    break;
                }
            }
        }
        if &resp[0..3] == b"FIN" {
            if FinCommands::try_from(resp[3])? == FinCommands::Failure {
                loop {
                    if let Ok(n) = uart.read_buffered(&mut resp) {
                        i += n;
                        if resp[i - 1] == b'.' {
                            break;
                        }
                    }
                }

                error!("{=str}", str::from_utf8(&resp[4..i]).unwrap());
            }
        } else {
            error!(
                "Did not find 'FIN' header in response from fin (got {})",
                resp[0..3]
            );
            return Err(FinCommsError::BadCommandHeader(
                resp[0..3].try_into().unwrap(),
            ));
        }

        Ok(())
    }

    pub fn start_fins(&mut self, _: FinsIdle) -> (FinsStarted, [Result<(), FinCommsError>; 4]) {
        info!("Starting fins");

        let mut errs = [Ok(()); 4];
        for i in 0..4 {
            info!("Fin {}", i);
            self.select_fin(i);

            errs[i] = self.send_fin_cmd(FinCommands::RecordData);
        }

        let started = self.enable_synced();

        self.delay.delay_millis(10);

        self.synced_output.as_mut().unwrap().set_level(Level::Low);

        (started, errs)
    }

    pub fn erase_fin_flashes(&mut self, _: &FinsIdle) -> [Result<(), FinCommsError>; 4] {
        info!("Erasing fin flashes");

        let mut errs = [Ok(()); 4];
        for i in 0..4 {
            info!("Fin {}", i);
            self.select_fin(i);
            errs[i] = self.send_fin_cmd(FinCommands::EraseFlash);
        }

        errs
    }

    pub fn stop_fins(&mut self, _: FinsStarted) -> FinsIdle {
        self.synced_output.as_mut().unwrap().set_level(Level::High);
        FinsIdle(())
    }
}

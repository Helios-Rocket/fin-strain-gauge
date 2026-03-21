use embedded_error::{mci, ImplError};
use rtic_monotonics::systick::{
    fugit::{Duration, Instant},
    ExtU32,
};
use teensy4_bsp::{
    hal::iomuxc,
    pins::t41::Pins,
    ral::{self, modify_reg, read_reg, write_reg},
};

const TIMEOUT: Duration<u32, 1, 1000> = Duration::<u32, _, _>::millis(50_u32);

pub struct SdCard {
    instance: ral::usdhc::Instance<1>,
    prev_resp_conf: ResponseConfig,
    card_addr: Option<u32>,
    now: fn() -> Instant<u32, 1, 1000>,
}

impl SdCard {
    /// Safety: assumes there is only one card on the bus, and takes ownership of the uSDHC1 peripheral.
    /// Do not construct a second `SdCard` or get an instance of the uSDHC1 peripheral anywhere else.
    /// Does not initialize the card, call `init` for that
    pub unsafe fn new(
        pins: &mut Pins,
        ccm: &ral::ccm::CCM,
        now: fn() -> Instant<u32, 1, 1000>,
    ) -> Self {
        // Configure uSDHC1 pins (teensy pins 42-47)
        iomuxc::usdhc::prepare(&mut pins.p42);
        iomuxc::usdhc::prepare(&mut pins.p43);
        iomuxc::usdhc::prepare(&mut pins.p44);
        iomuxc::usdhc::prepare(&mut pins.p45);
        iomuxc::usdhc::prepare(&mut pins.p46);
        iomuxc::usdhc::prepare(&mut pins.p47);

        // Enable the uSDHC1 clocks
        modify_reg!(ral::ccm, ccm, CCGR6, CG1: 0b11);

        Self {
            instance: ral::usdhc::USDHC1::instance(),
            prev_resp_conf: ResponseConfig::RZ,
            card_addr: None,
            now,
        }
    }

    pub fn init(&mut self) -> Result<(), mci::MciError> {
        modify_reg!(ral::usdhc, self.instance, SYS_CTRL, RSTA: 1);

        {
            let start = (self.now)();
            while read_reg!(ral::usdhc, self.instance, SYS_CTRL, RSTA == 1) {
                if (self.now)() >= start + TIMEOUT {
                    return Err(mci::MciError::Impl(ImplError::TimedOut));
                }
            }
        }

        // Base clock frequency is 150MHz, and the target frequency is 400kHz.
        // We choose a prescaler of 32 and a divisor of 12, which gives us an actual frequency of 390.625kHz.
        modify_reg!(ral::usdhc, self.instance, SYS_CTRL, SDCLKFS: 0x10, DVS: 12);

        modify_reg!(ral::usdhc, self.instance, INT_STATUS_EN, CCSEN: 1, CTOESEN: 1, CCESEN: 1, CEBESEN: 1);

        self.instance
            .SYS_CTRL
            .write(ral::usdhc::SYS_CTRL::INITA::mask);
        modify_reg!(ral::usdhc, self.instance, SYS_CTRL, INITA: 1);

        {
            let start = (self.now)();
            while read_reg!(ral::usdhc, self.instance, SYS_CTRL, INITA == 1) {
                if (self.now)() >= start + TIMEOUT {
                    return Err(mci::MciError::Impl(ImplError::TimedOut));
                }
            }
        }

        self.send_cmd(0, 0, ResponseConfig::RZ)?;
        let pat = 0xaa;
        // 0b0001 => Supplied voltage range 2.7-3.6V
        self.send_cmd(8, 0b0001 << 8 | pat, ResponseConfig::R7)?;
        let resp = self.read_response()?;

        // TODO: errors
        if pat != resp[0] & 0xff {
            return Err(mci::MciError::UnusableCard);
        }

        {
            let start = (self.now)();
            loop {
                // TODO: wtf are these numbers?
                self.send_acmd(41, 0x80100000 | 0x40000000, ResponseConfig::R3)?;
                let resp = self.read_response()?;

                if resp[0] >> 31 == 1 {
                    break;
                }

                if (self.now)() >= start + TIMEOUT {
                    return Err(mci::MciError::NoCard);
                }
            }
        }

        self.send_cmd(2, 0, ResponseConfig::R2)?;
        let _ = self.read_response()?;

        self.send_cmd(3, 0, ResponseConfig::R6)?;
        let resp = self.read_response()?;

        // This is actually the card address shifted into the first two MSB, since it is always used there as a command arg
        self.card_addr = Some(resp[0] & (0xffff << 16));
        // log::info!("{:#018b}", self.card_addr.unwrap());

        self.send_acmd(13, 0, ResponseConfig::R1)?;
        let resp = self.read_response()?;
        // log::info!("{:#034b}", resp[0]);

        Ok(())
    }

    fn send_cmd(
        &mut self,
        cmd_idx: u8,
        arg: u32,
        resp_conf: ResponseConfig,
    ) -> Result<(), mci::MciError> {
        // Wait for previous command
        {
            let start = (self.now)();
            while read_reg!(ral::usdhc, self.instance, PRES_STATE, CIHB == 1) {
                if (self.now)() >= start + TIMEOUT {
                    return Err(mci::MciError::CommandInhibited);
                }
            }
        }

        self.instance.CMD_ARG.write(arg);
        write_reg!(
            ral::usdhc,
            self.instance,
            CMD_XFR_TYP,
            CMDINX: cmd_idx as u32,
            CICEN: if resp_conf.check_index { 1 } else { 0 },
            CCCEN: if resp_conf.check_crc { 1 } else { 0 },
            RSPTYP: match resp_conf.length {
                ResponseLength::None => 0b00,
                ResponseLength::Long => 0b01,
                ResponseLength::Short { check_busy: true } => 0b11,
                ResponseLength::Short { check_busy: false } => 0b10,
            }
        );
        self.prev_resp_conf = resp_conf;

        Ok(())
    }

    /// Sends CMD55 followed by ACMD<`cmd_idx`>
    /// If there is already an RCA (relative card address) set, use that as the argument to CMD55
    /// Otherwise the arg is 0, which is used when setting up a card for the first time
    fn send_acmd(
        &mut self,
        cmd_idx: u8,
        acmd_arg: u32,
        resp_conf: ResponseConfig,
    ) -> Result<(), mci::MciError> {
        let cmd_arg = match self.card_addr {
            Some(rca) => rca,
            None => 0,
        };
        self.send_cmd(55, cmd_arg, ResponseConfig::R1)?;
        let _ = self.read_response()?;

        self.send_cmd(cmd_idx, acmd_arg, resp_conf)?;

        Ok(())
    }

    fn read_response(&self) -> Result<[u32; 4], mci::MciError> {
        {
            let start = (self.now)();
            while read_reg!(ral::usdhc, self.instance, INT_STATUS) == 0 {
                if (self.now)() >= start + TIMEOUT {
                    return Err(mci::MciError::Impl(ImplError::TimedOut));
                }
            }
        }

        let (complete, timeout_err, end_bit_err, crc_err) =
            read_reg!(ral::usdhc, self.instance, INT_STATUS, CC, CTOE, CEBE, CCE);

        // Clear all status bits (writing 1 to a bit clears it)
        self.instance
            .INT_STATUS
            .write(self.instance.INT_STATUS.read());

        // We seem to need to spin here for a little before the CMD_RSP<N> registers are filled
        {
            let start = (self.now)();
            while (self.now)() < start + 1_u32.millis() {}
        }

        if timeout_err == 1 {
            return Err(mci::MciError::CommandError(
                mci::CommandOrDataError::Timeout,
            ));
        }

        if end_bit_err == 1 {
            return Err(mci::MciError::CommandError(mci::CommandOrDataError::EndBit));
        }

        if crc_err == 1 {
            return Err(mci::MciError::CommandError(mci::CommandOrDataError::Crc));
        }

        // I don't think this is possible
        if complete != 1 {
            return Err(mci::MciError::Impl(ImplError::Internal));
        }

        Ok(match self.prev_resp_conf.length {
            ResponseLength::None => [0, 0, 0, 0],
            ResponseLength::Short { .. } => [self.instance.CMD_RSP0.read(), 0, 0, 0],
            ResponseLength::Long => [
                self.instance.CMD_RSP0.read(),
                self.instance.CMD_RSP1.read(),
                self.instance.CMD_RSP2.read(),
                self.instance.CMD_RSP3.read(),
            ],
        })
    }
}

#[derive(Copy, Clone)]
enum ResponseLength {
    None,
    Long,
    Short { check_busy: bool },
}

#[derive(Copy, Clone)]
struct ResponseConfig {
    check_index: bool,
    check_crc: bool,
    length: ResponseLength,
}

#[allow(unused)]
impl ResponseConfig {
    /// No response
    const RZ: Self = Self {
        check_index: false,
        check_crc: false,
        length: ResponseLength::None,
    };

    /// Default response with cmd index and crc.
    const R1: Self = Self {
        check_index: true,
        check_crc: true,
        length: ResponseLength::Short { check_busy: false },
    };

    /// Same as R1 but with busy signal.
    #[allow(non_upper_case_globals)]
    const R1b: Self = Self {
        check_index: true,
        check_crc: true,
        length: ResponseLength::Short { check_busy: true },
    };

    /// Long response for CID, CSD. No cmd index or crc.
    const R2: Self = Self {
        check_index: false,
        check_crc: false,
        length: ResponseLength::Long,
    };

    /// OCR Register
    const R3: Self = Self {
        check_index: false,
        check_crc: false,
        length: ResponseLength::Short { check_busy: false },
    };

    /// RCA response
    const R6: Self = Self {
        check_index: true,
        check_crc: true,
        length: ResponseLength::Short { check_busy: false },
    };

    /// Card interface condition
    const R7: Self = Self {
        check_index: true,
        check_crc: true,
        length: ResponseLength::Short { check_busy: false },
    };
}

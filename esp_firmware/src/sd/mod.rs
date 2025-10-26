use defmt::{error, info, warn};
use embedded_error::{mci, ImplError};
use esp_hal::{
    peripherals,
    time::{Duration, Instant},
};

pub mod dma;
pub mod pins;
pub mod response;

use pins::Pins;
use response::{ResponseConfig, ResponseLength};

use crate::sd::response::response_status::{self, DEFAULT_EVENTS};

const TIMEOUT: Duration = Duration::from_millis(100);

pub struct SdHost<'d> {
    instance: peripherals::SDHOST<'d>,
    pins: Pins<'d>,
    prev_resp_conf: ResponseConfig,
    card_addr: Option<u32>,
}

impl<'d> SdHost<'d> {
    pub fn new(sys: &peripherals::SYSTEM, sdhost: peripherals::SDHOST<'d>, pins: Pins<'d>) -> Self {
        let sys_regs = sys.register_block();

        // Enable the SDHOST clock gate
        sys_regs
            .perip_clk_en1()
            .modify(|_, w| w.sdio_host_clk_en().set_bit());
        // Pull the SDHOST peripheral out of reset
        sys_regs
            .perip_rst_en1()
            .modify(|_, w| w.sdio_host_rst().clear_bit());

        Self {
            instance: sdhost,
            pins,
            prev_resp_conf: ResponseConfig::RZ,
            card_addr: None,
        }
    }

    pub fn init(&mut self) -> Result<(), mci::MciError> {
        // Set the first card to 4 bit mode
        self.instance
            .register_block()
            .ctype()
            .modify(|_, w| unsafe { w.card_width4().bits(1) });

        // Clock configuration: we want a cclk_out of 400kHz initially
        // We will use the 160M PLL clock which always runs at 160MHz, thus we need a total of 400 times division
        // We can control a prescaler (clk_divider0), counter value when positive edge is generated (ccllkin_edge_h), counter value when negative edge is generated (ccllkin_edge_l), and counter rollover (ccllkin_edge_n) which should be the same as ccllkin_edge_l.
        self.instance
            .register_block()
            .clkdiv()
            // Prescaler of 2 * 25 = 50
            .modify(|_, w| unsafe { w.clk_divider0().bits(25) });
        self.instance
            .register_block()
            .clksrc()
            .modify(|_, w| unsafe { w.clksrc().bits(0) });
        self.instance
            .register_block()
            .clk_edge_sel()
            .modify(|_, w| unsafe {
                w.ccllkin_edge_n()
                    .bits(7) // Rollover at 8
                    .ccllkin_edge_l()
                    .bits(7) // Negative edge at 8
                    .ccllkin_edge_h()
                    .bits(3) // Positive edge at 4
                    // This register appears to be misnamed. The data sheet says it is the clock source (1 = PLL160M, 0 = XTAL)
                    .cclk_en()
                    .set_bit()
            });
        self.instance
            .register_block()
            .tmout()
            .modify(|_, w| unsafe { w.response_timeout().bits(255) });

        self.update_clocks()?;

        // Enable the clock for the first card
        self.instance
            .register_block()
            .clkena()
            .modify(|_, w| unsafe { w.cclk_enable().bits(0b01).lp_enable().bits(0b01) });

        self.update_clocks()?;

        self.instance
            .register_block()
            .rintsts()
            .write(|w| unsafe { w.bits(0xffffffff) });
        self.instance
            .register_block()
            .intmask()
            .write(|w| unsafe { w.int_mask().bits(DEFAULT_EVENTS).sdio_int_mask().bits(0b01) });
        self.instance
            .register_block()
            .ctrl()
            .modify(|_, w| w.int_enable().set_bit());

        // Send the first command along with an init sequence of 80 clock cycles
        self.instance
            .register_block()
            .cmd()
            .modify(|_, w| w.send_initialization().set_bit());
        self.send_cmd(0, 0, ResponseConfig::RZ)?;
        self.instance
            .register_block()
            .cmd()
            .modify(|_, w| w.send_initialization().clear_bit());

        let pat = 0xaa;
        // 0b0001 => Supplied voltage range 2.7-3.6V
        self.send_cmd(8, 0b0001 << 8 | pat, ResponseConfig::R7)?;
        let resp = self.read_response()?;

        // TODO: errors
        if pat != resp[0] & 0xff {
            return Err(mci::MciError::UnusableCard);
        }

        {
            let start = Instant::now();
            loop {
                // TODO: wtf are these numbers?
                self.send_acmd(41, 0x80100000 | 0x40000000, ResponseConfig::R3)?;
                let resp = self.read_response()?;

                if resp[0] >> 31 == 1 {
                    break;
                }

                if Instant::now() - start >= TIMEOUT {
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
        info!("addr: {:#018b}", self.card_addr.unwrap());

        self.send_cmd(9, self.card_addr.unwrap(), ResponseConfig::R2)?;
        let csd = self.read_response()?;

        let temp1 = (csd[1] & 0xFFFF0000) >> 16;
        let temp2 = (csd[2] & 0x3F) << 16;
        let card_capacity: u64 = (((temp2 | temp1) + 1) as u64) * 512 * 1024;

        info!("capacity: {} bytes", card_capacity);

        self.send_cmd(7, self.card_addr.unwrap(), ResponseConfig::R1b)?;

        self.send_acmd(6, 0b10, ResponseConfig::R1)?;
        let _ = self.read_response()?;

        // Switch to a higher clock speed
        self.instance
            .register_block()
            .clkena()
            .modify(|_, w| unsafe { w.cclk_enable().bits(0).lp_enable().bits(0) });

        self.update_clocks()?;

        self.instance
            .register_block()
            .clkdiv()
            .modify(|_, w| unsafe { w.clk_divider0().bits(0) });

        self.update_clocks()?;

        self.pins.set_cmd_push_pull();

        self.instance
            .register_block()
            .clkena()
            .modify(|_, w| unsafe { w.cclk_enable().bits(0b01).lp_enable().bits(0b01) });

        self.update_clocks()?;

        // self.send_acmd(13, 0, ResponseConfig::R1)?;
        // let resp = self.read_response()?;
        // info!("{:#034b}", resp[0]);

        Ok(())
    }

    fn update_clocks(&mut self) -> Result<(), mci::MciError> {
        let regs = self.instance.register_block();

        let start = Instant::now();
        while regs.cmd().read().start_cmd().bit() {
            if Instant::now() - start >= TIMEOUT {
                error!("Waiting for previous command");
                return Err(mci::MciError::Impl(ImplError::TimedOut));
            }
        }

        regs.cmdarg().write(|w| unsafe { w.bits(0) });
        regs.cmd().modify(|_, w| unsafe {
            w.use_hole()
                .set_bit()
                .start_cmd()
                .set_bit()
                .card_number()
                .bits(0)
                .wait_prvdata_complete()
                .set_bit()
                .update_clock_registers_only()
                .set_bit()
        });

        Ok(())
    }

    fn send_cmd(
        &mut self,
        cmd_idx: u8,
        arg: u32,
        resp_conf: ResponseConfig,
    ) -> Result<(), mci::MciError> {
        let regs = self.instance.register_block();

        // Wait for previous command
        let start = Instant::now();
        while regs.cmd().read().start_cmd().bit() {
            if Instant::now() - start >= TIMEOUT {
                error!("Waiting for previous command");
                return Err(mci::MciError::Impl(ImplError::TimedOut));
            }
        }

        regs.cmdarg().write(|w| unsafe { w.bits(arg) });

        regs.cmd().modify(|_, w| unsafe {
            w.index()
                .bits(cmd_idx)
                .use_hole()
                .set_bit()
                .check_response_crc()
                .bit(resp_conf.check_crc)
                .card_number()
                .bits(0)
                .wait_prvdata_complete()
                .set_bit()
                .response_length()
                .bit(resp_conf.length == ResponseLength::Long)
                .response_expect()
                .bit(resp_conf.length != ResponseLength::None)
                .update_clock_registers_only()
                .clear_bit()
                .start_cmd()
                .set_bit()
        });

        self.prev_resp_conf = resp_conf;

        let start = Instant::now();
        while regs.cmd().read().start_cmd().bit() {
            if Instant::now() - start >= TIMEOUT {
                error!("Waiting for command to take");
                return Err(mci::MciError::Impl(ImplError::TimedOut));
            }
        }

        Ok(())
    }

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

    fn read_response(&mut self) -> Result<[u32; 4], mci::MciError> {
        let regs = self.instance.register_block();

        let start = Instant::now();
        loop {
            let status_reg = regs.rintsts().read();
            if status_reg.int_status_raw().bits() != 0 {
                let int_status = status_reg.int_status_raw().bits();
                let status = regs.status().read();

                if int_status & response_status::CMD_MASK != 0
                    && status.command_fsm_states().bits() == 0
                {
                    break;
                } else if int_status & response_status::RE_MASK != 0 {
                    return Err(mci::MciError::ReadError);
                } else if int_status & response_status::EBE_MASK != 0 {
                    return Err(mci::MciError::CommandError(mci::CommandOrDataError::EndBit));
                } else if int_status & response_status::RTO_MASK != 0 {
                    return Err(mci::MciError::CommandError(
                        mci::CommandOrDataError::Timeout,
                    ));
                } else if int_status & response_status::RCRC_MASK != 0 {
                    return Err(mci::MciError::CommandError(mci::CommandOrDataError::Crc));
                } else if int_status & response_status::HLE_MASK != 0 {
                    return Err(mci::MciError::Impl(ImplError::Internal));
                }
            }
            if Instant::now() - start >= TIMEOUT {
                error!("Waiting for command response");
                return Err(mci::MciError::Impl(ImplError::TimedOut));
            }
        }

        // Clear status bits (writing 1 to a bit clears it)
        regs.rintsts().modify(|r, w| unsafe {
            w.sdio_interrupt_raw()
                .bits(r.sdio_interrupt_raw().bits())
                .int_status_raw()
                .bits(r.int_status_raw().bits())
        });

        // let start = Instant::now();
        // while Instant::now() - start <= Duration::from_millis(1) {}
        // let status = regs.status().read();
        // info!(
        //     "State machine: (sm busy: {}, data busy: {}, state: {})",
        //     status.data_state_mc_busy().bit(),
        //     status.data_busy().bit(),
        //     status.command_fsm_states().bits()
        // );

        Ok(match self.prev_resp_conf.length {
            ResponseLength::None => [0, 0, 0, 0],
            ResponseLength::Short { .. } => [regs.resp0().read().bits(), 0, 0, 0],
            ResponseLength::Long => [
                regs.resp0().read().bits(),
                regs.resp1().read().bits(),
                regs.resp2().read().bits(),
                regs.resp3().read().bits(),
            ],
        })
    }
}

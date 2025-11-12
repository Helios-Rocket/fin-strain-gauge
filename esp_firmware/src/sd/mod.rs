use defmt::{error, info};
use embedded_error::{mci, ImplError};
use esp32s3::sdhost::RegisterBlock;
use esp_hal::{
    delay::Delay,
    peripherals,
    time::{Duration, Instant},
};

pub mod dma;
pub mod pins;
pub mod response;

use fatfs::{IoBase, IoError, Read, Seek, SeekFrom, Write};
use pins::Pins;
use response::{ResponseConfig, ResponseLength};

use crate::sd::response::response_status::{self, DEFAULT_EVENTS};

const TIMEOUT: Duration = Duration::from_millis(500);
const BLOCK_SIZE: usize = 512;

#[derive(Debug, Clone)]
pub struct SdError(pub mci::MciError);

impl IoError for SdError {
    fn is_interrupted(&self) -> bool {
        false
    }

    fn new_unexpected_eof_error() -> Self {
        Self(mci::MciError::Impl(ImplError::OutOfMemory))
    }

    fn new_write_zero_error() -> Self {
        Self(mci::MciError::WriteError)
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub enum CmdConfig {
    #[default]
    Default,
    ClockOnly,
    Data {
        write: bool,
    },
    InitSequence,
}

pub struct SdHost<'d> {
    instance: peripherals::SDHOST<'d>,
    pins: Pins<'d>,
    prev_resp_conf: ResponseConfig,
    card_addr: Option<u32>,
    capacity: u64,
    block_addr: usize,
    byte_addr: usize,
    offset: u64,
    end: u64,
    tx_buf: [u8; BLOCK_SIZE],
    rx_buf: [u8; BLOCK_SIZE],
    rx_valid: bool,
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
            capacity: 0,
            block_addr: 0,
            byte_addr: 0,
            offset: 0,
            end: 0,
            tx_buf: [0; BLOCK_SIZE],
            rx_buf: [0; BLOCK_SIZE],
            rx_valid: false,
        }
    }

    pub fn init(&mut self) -> Result<(), SdError> {
        // Reset the first card
        self.regs().rst_n().write(|w| unsafe { w.bits(0) });
        self.regs().rst_n().write(|w| unsafe { w.bits(0b01) });
        // let i = 10_u16 + 5_u8;
        // Set the first card to 4 bit mode
        self.regs()
            .ctype()
            .modify(|_, w| unsafe { w.card_width4().bits(1) });

        // Clock configuration: we want a cclk_out of 400kHz initially
        // We will use the 160M PLL clock which always runs at 160MHz, thus we need a total of 400 times division
        // We can control a prescaler (clk_divider0), counter value when positive edge is generated (ccllkin_edge_h), counter value when negative edge is generated (ccllkin_edge_l), and counter rollover (ccllkin_edge_n) which should be the same as ccllkin_edge_l.
        self.regs()
            .clkdiv()
            // Prescaler of 2 * 25 = 50
            .modify(|_, w| unsafe { w.clk_divider0().bits(25) });
        self.regs()
            .clksrc()
            .modify(|_, w| unsafe { w.clksrc().bits(0) });
        self.regs().clk_edge_sel().modify(|_, w| unsafe {
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
        self.regs()
            .tmout()
            .modify(|_, w| unsafe { w.response_timeout().bits(255) });

        self.update_clocks()?;

        // Enable the clock for the first card
        self.regs()
            .clkena()
            .modify(|_, w| unsafe { w.cclk_enable().bits(0b01).lp_enable().bits(0b01) });

        self.update_clocks()?;

        self.regs()
            .rintsts()
            .write(|w| unsafe { w.bits(0xffffffff) });
        self.regs()
            .intmask()
            .write(|w| unsafe { w.int_mask().bits(DEFAULT_EVENTS).sdio_int_mask().bits(0b01) });
        self.regs().ctrl().modify(|_, w| w.int_enable().set_bit());

        self.regs().ctrl().modify(|_, w| w.fifo_reset().set_bit());
        while self.regs().ctrl().read().fifo_reset().bit() {}

        // Send the first command along with an init sequence of 80 clock cycles
        self.send_cmd(0, 0, ResponseConfig::RZ, CmdConfig::InitSequence)?;

        let pat = 0xaa;
        // 0b0001 => Supplied voltage range 2.7-3.6V
        self.send_cmd(8, 0b0001 << 8 | pat, ResponseConfig::R7, CmdConfig::Default)?;
        let resp = self.read_response()?;

        // TODO: errors
        if pat != resp[0] & 0xff {
            return Err(SdError(mci::MciError::UnusableCard));
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

                if start.elapsed() >= TIMEOUT {
                    return Err(SdError(mci::MciError::NoCard));
                }
            }
        }

        self.send_cmd(2, 0, ResponseConfig::R2, CmdConfig::Default)?;
        let _ = self.read_response()?;

        self.send_cmd(3, 0, ResponseConfig::R6, CmdConfig::Default)?;
        let resp = self.read_response()?;

        // This is actually the card address shifted into the first two MSB, since it is always used there as a command arg
        self.card_addr = Some(resp[0] & (0xffff << 16));
        info!("addr: {:#018b}", self.card_addr.unwrap());

        self.send_cmd(
            9,
            self.card_addr.unwrap(),
            ResponseConfig::R2,
            CmdConfig::Default,
        )?;
        let csd = self.read_response()?;

        let temp1 = (csd[1] & 0xFFFF0000) >> 16;
        let temp2 = (csd[2] & 0x3F) << 16;
        let card_capacity: u64 = (((temp2 | temp1) + 1) as u64) * 512 * 1024;
        self.capacity = card_capacity;

        info!("capacity: {} bytes", card_capacity);

        self.send_cmd(
            7,
            self.card_addr.unwrap(),
            ResponseConfig::R1b,
            CmdConfig::Default,
        )?;

        self.send_acmd(6, 0b10, ResponseConfig::R1)?;
        let _ = self.read_response()?;

        // Switch to a higher clock speed
        self.regs()
            .clkena()
            .modify(|_, w| unsafe { w.cclk_enable().bits(0).lp_enable().bits(0) });

        self.update_clocks()?;

        // No prescaler
        self.regs()
            .clkdiv()
            .modify(|_, w| unsafe { w.clk_divider0().bits(1) });

        // 160MHz / 16 = 40MHz
        // self.regs().clk_edge_sel().modify(|_, w| unsafe {
        //     w.ccllkin_edge_n()
        //         .bits(3) // Rollover at 4
        //         .ccllkin_edge_l()
        //         .bits(3) // Negative edge at 4
        //         .ccllkin_edge_h()
        //         .bits(1) // Positive edge at 2
        // });

        self.update_clocks()?;

        self.pins.set_cmd_push_pull();

        self.regs()
            .clkena()
            .modify(|_, w| unsafe { w.cclk_enable().bits(0b01).lp_enable().bits(0b01) });

        self.update_clocks()?;

        self.send_acmd(42, 0, ResponseConfig::R1)?;
        let _ = self.read_response()?;

        self.regs().blksiz().write(|w| unsafe { w.bits(512) });
        self.regs().bytcnt().write(|w| unsafe { w.bits(512) });

        Ok(())
    }

    pub fn set_offset(&mut self, offset: u32, length: u32) -> Result<(), SdError> {
        self.offset = offset as u64 * 512;
        self.end = self.offset + length as u64 * 512;
        self.seek(SeekFrom::Start(0))?;

        Ok(())
    }

    fn update_clocks(&mut self) -> Result<(), SdError> {
        let start = Instant::now();
        while self.regs().cmd().read().start_cmd().bit() {
            if start.elapsed() >= TIMEOUT {
                error!("Waiting for previous command");
                return Err(SdError(mci::MciError::Impl(ImplError::TimedOut)));
            }
        }

        self.send_cmd(0, 0, ResponseConfig::RZ, CmdConfig::ClockOnly)

        // regs.cmdarg().write(|w| unsafe { w.bits(0) });
        // regs.cmd().modify(|_, w| unsafe {
        //     w.use_hole()
        //         .set_bit()
        //         .start_cmd()
        //         .set_bit()
        //         .card_number()
        //         .bits(0)
        //         .wait_prvdata_complete()
        //         .set_bit()
        //         .update_clock_registers_only()
        //         .set_bit()
        // });
    }

    fn clear_status(&mut self) {
        // Clear status bits (writing 1 to a bit clears it)
        self.regs().rintsts().modify(|r, w| unsafe {
            w.sdio_interrupt_raw()
                .bits(r.sdio_interrupt_raw().bits())
                .int_status_raw()
                .bits(r.int_status_raw().bits())
        });
    }

    fn send_cmd(
        &mut self,
        cmd_idx: u8,
        arg: u32,
        resp_conf: ResponseConfig,
        cmd_config: CmdConfig,
    ) -> Result<(), SdError> {
        // Wait for previous command
        let start = Instant::now();
        while self.regs().cmd().read().start_cmd().bit() {
            if start.elapsed() >= TIMEOUT {
                error!("Waiting for previous command");
                return Err(SdError(mci::MciError::Impl(ImplError::TimedOut)));
            }
        }

        self.clear_status();

        let start = Instant::now();
        while start.elapsed() <= Duration::from_micros(1) {}

        self.regs().cmdarg().write(|w| unsafe { w.bits(arg) });

        self.regs().cmd().write(|w| unsafe {
            w.index()
                .bits(cmd_idx)
                .use_hole()
                .set_bit()
                .card_number()
                .bits(0)
                .send_initialization()
                .bit(cmd_config == CmdConfig::InitSequence)
                .data_expected()
                .bit(match cmd_config {
                    CmdConfig::Data { write: _ } => true,
                    _ => false,
                })
                .read_write()
                .bit(match cmd_config {
                    CmdConfig::Data { write } => write,
                    _ => false,
                })
                .send_auto_stop()
                .set_bit()
                .check_response_crc()
                .bit(resp_conf.check_crc)
                .wait_prvdata_complete()
                .set_bit()
                .response_length()
                .bit(resp_conf.length == ResponseLength::Long)
                .response_expect()
                .bit(resp_conf.length != ResponseLength::None)
                .update_clock_registers_only()
                .bit(cmd_config == CmdConfig::ClockOnly)
                .start_cmd()
                .set_bit()
        });

        self.prev_resp_conf = resp_conf;

        let start = Instant::now();
        while self.regs().cmd().read().start_cmd().bit() {
            if start.elapsed() >= TIMEOUT {
                error!("Waiting for command to take");
                return Err(SdError(mci::MciError::Impl(ImplError::TimedOut)));
            }
        }

        Ok(())
    }

    fn send_acmd(
        &mut self,
        cmd_idx: u8,
        acmd_arg: u32,
        resp_conf: ResponseConfig,
    ) -> Result<(), SdError> {
        let cmd_arg = match self.card_addr {
            Some(rca) => rca,
            None => 0,
        };
        self.send_cmd(55, cmd_arg, ResponseConfig::R1, CmdConfig::Default)?;
        let _ = self.read_response()?;

        self.send_cmd(cmd_idx, acmd_arg, resp_conf, CmdConfig::Default)?;

        Ok(())
    }

    fn read_response(&mut self) -> Result<[u32; 4], SdError> {
        let start = Instant::now();
        loop {
            let status_reg = self.regs().rintsts().read();
            if status_reg.int_status_raw().bits() != 0 {
                let int_status = status_reg.int_status_raw().bits();

                if int_status & response_status::CMD_MASK != 0 {
                    break;
                } else if int_status & response_status::RE_MASK != 0 {
                    error!("Read error");
                    return Err(SdError(mci::MciError::ReadError));
                } else if int_status & response_status::EBE_MASK != 0 {
                    error!("End bit");
                    return Err(SdError(mci::MciError::CommandError(
                        mci::CommandOrDataError::EndBit,
                    )));
                } else if int_status & response_status::RTO_MASK != 0 {
                    error!("Response Timeout");
                    return Err(SdError(mci::MciError::CommandError(
                        mci::CommandOrDataError::Timeout,
                    )));
                } else if int_status & response_status::RCRC_MASK != 0 {
                    error!("Crc error");
                    return Err(SdError(mci::MciError::CommandError(
                        mci::CommandOrDataError::Crc,
                    )));
                } else if int_status & response_status::HLE_MASK != 0 {
                    return Err(SdError(mci::MciError::Impl(ImplError::Internal)));
                }
            }
            if start.elapsed() >= TIMEOUT {
                error!("Waiting for command response");
                return Err(SdError(mci::MciError::Impl(ImplError::TimedOut)));
            }
        }

        self.clear_status();

        let start = Instant::now();
        while self.regs().status().read().command_fsm_states() != 0 {
            if start.elapsed() >= TIMEOUT {
                error!("Waiting for cmd state machine to go idle");
                return Err(SdError(mci::MciError::Impl(ImplError::TimedOut)));
            }
        }

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
            ResponseLength::Short { .. } => [self.regs().resp0().read().bits(), 0, 0, 0],
            ResponseLength::Long => [
                self.regs().resp0().read().bits(),
                self.regs().resp1().read().bits(),
                self.regs().resp2().read().bits(),
                self.regs().resp3().read().bits(),
            ],
        })
    }

    fn regs(&self) -> &RegisterBlock {
        self.instance.register_block()
    }

    fn get_byte_position(&self) -> u64 {
        self.block_addr as u64 * 512 + self.byte_addr as u64
    }

    fn read_block(&mut self) -> Result<(), SdError> {
        let start = Instant::now();
        while self.regs().status().read().data_busy().bit() {
            if start.elapsed() >= TIMEOUT {
                return Err(SdError(mci::MciError::DataError(
                    mci::CommandOrDataError::Timeout,
                )));
            }
        }

        self.send_cmd(
            17,
            self.block_addr as u32,
            ResponseConfig::R1,
            CmdConfig::Data { write: false },
        )?;
        _ = self.read_response()?;

        let start = Instant::now();
        while self.regs().status().read().data_state_mc_busy().bit()
            || self.regs().status().read().data_busy().bit()
        {
            if start.elapsed() >= TIMEOUT {
                return Err(SdError(mci::MciError::DataError(
                    mci::CommandOrDataError::Timeout,
                )));
            }
        }
        let start = Instant::now();
        loop {
            let status_reg = self.regs().rintsts().read();
            if status_reg.int_status_raw().bits() != 0 {
                let int_status = status_reg.int_status_raw().bits();
                let status = self.regs().status().read();

                if int_status & response_status::DTO_MASK != 0
                    && status.command_fsm_states().bits() == 0
                {
                    break;
                }
            }
            if start.elapsed() >= TIMEOUT {
                error!("Waiting for data response");
                return Err(SdError(mci::MciError::Impl(ImplError::TimedOut)));
            }
        }

        // Clear status bits (writing 1 to a bit clears it)
        self.regs().rintsts().modify(|r, w| unsafe {
            w.sdio_interrupt_raw()
                .bits(r.sdio_interrupt_raw().bits())
                .int_status_raw()
                .bits(r.int_status_raw().bits())
        });

        for i in 0..(512 / 4) {
            let word = self.regs().buffifo().read().bits();
            self.rx_buf[4 * i..4 * (i + 1)].copy_from_slice(&word.to_ne_bytes());
        }
        // let fifo_addr = self.regs().buffifo().read().bits();
        // let fifo_ptr = fifo_addr as *const u8;

        // info!("addr: {}", fifo_addr);
        // info!("Probably about to explode");
        // unsafe {
        //     self.rx_buf[0..1].copy_from_slice(core::slice::from_raw_parts(fifo_ptr, 1));
        // }
        // info!("Didn't explode???");

        self.rx_valid = true;

        Ok(())
    }

    fn write_block(&mut self) -> Result<(), SdError> {
        let start = Instant::now();
        while self.regs().status().read().data_busy().bit() {
            if start.elapsed() >= TIMEOUT {
                return Err(SdError(mci::MciError::DataError(
                    mci::CommandOrDataError::Timeout,
                )));
            }
        }

        self.send_cmd(
            24,
            self.block_addr as u32,
            ResponseConfig::R1,
            CmdConfig::Data { write: true },
        )?;
        Ok(())
    }
}

impl<'d> IoBase for SdHost<'d> {
    type Error = SdError;
}

impl<'d> Read for SdHost<'d> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut n = 0;
        let len = buf.len();
        while n < len {
            if !self.rx_valid {
                self.read_block()?;
            }

            if len - n < self.rx_buf.len() - self.byte_addr {
                buf[n..].copy_from_slice(&self.rx_buf[self.byte_addr..self.byte_addr + len - n]);
                self.byte_addr += len - n;
                n = len;
            } else {
                buf[n..n + self.rx_buf.len() - self.byte_addr]
                    .copy_from_slice(&self.rx_buf[self.byte_addr..]);
                n += self.rx_buf.len() - self.byte_addr;
                self.byte_addr = 0;
                self.block_addr += 1;
                self.rx_valid = false;
            }
        }

        Ok(n)
    }
}

impl<'d> Seek for SdHost<'d> {
    fn seek(&mut self, pos: SeekFrom) -> Result<u64, Self::Error> {
        let abs_pos = match pos {
            SeekFrom::Start(pos) => pos as i64 + self.offset as i64,
            SeekFrom::End(pos) => self.end as i64 + pos,
            SeekFrom::Current(pos) => self.get_byte_position() as i64 + pos,
        };

        if abs_pos < 0 || abs_pos > self.end as i64 {
            return Err(SdError(mci::MciError::Impl(ImplError::OutOfMemory)));
        }

        let new_block_addr: usize = (abs_pos / 512) as usize;
        self.byte_addr = (abs_pos - (new_block_addr as i64) * 512) as usize;
        if new_block_addr != self.block_addr {
            self.rx_valid = false;
            self.block_addr = new_block_addr;
        }

        Ok(abs_pos as u64)
    }
}

impl<'d> Write for SdHost<'d> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        todo!()
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

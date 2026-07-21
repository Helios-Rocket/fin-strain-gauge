use core::ptr;

use cortex_m::asm::nop;
use defmt::{info, println};
use hal::{
    clocks::Clocks,
    delay_ms, delay_us,
    flash::Flash,
    gpio::{Pin, PinMode, Port},
    pac::{FLASH, QUADSPI, RCC},
    qspi::Qspi,
};

pub const BYTES_PER_PAGE: u32 = 2048;
pub const PAGES_PER_BLOCK: u32 = 64;
pub const FLASH_SIZE_BYTES: u32 = 512 * PAGES_PER_BLOCK * BYTES_PER_PAGE;

#[allow(unused)]
#[repr(u8)]
pub enum WinbondStatusReg {
    One = 0xA0,
    Two = 0xB0,
    Three = 0xC0,
}

#[allow(unused)]
#[repr(u8)]
pub enum WinbondInstruction {
    /// This can be used on its own and is a sort of "soft reset"
    DeviceReset = 0xFF,

    /// The following two commands must be used together and are a sort of "hard reset," including resetting protection and configuration registers
    EnableReset = 0x66,
    /// See `EnableReset` (which must be used first)
    ResetDevice = 0x99,

    ReadJedecId = 0x9F,

    ReadStatusRegister = 0x0F,
    WriteStatusRegister = 0x1F,

    WriteEnable = 0x06,
    /// NOTE: this is often not necessary, as WEL is reset after most commands that require it to be set
    WriteDisable = 0x04,

    /// Requires `WriteEnable` first
    BadBlockMgmnt = 0xA1,
    ReadBbmLut = 0xA5,

    LastEccFailPageAddr = 0xA9,

    /// Requires `WriteEnable` first
    BlockErase = 0xD8,

    /// Requires `WriteEnable` first
    LoadProgramData = 0x02,
    /// Requires `WriteEnable` first
    RandomLoadProgramData = 0x84,

    /// Requires `WriteEnable` first
    QuadLoadProgramData = 0x32,
    /// Requires `WriteEnable` first
    QuadRandomLoadProgramData = 0x34,

    /// Requires `WriteEnable` and `*LoadProgramData` first
    ProgramExecute = 0x10,

    PageDataRead = 0x13,

    ReadData = 0x03,
    FastRead = 0x0B,

    /// single line address, dual line data
    FastReadDualOutput = 0x3B,
    /// single line address, quad line data
    FastReadQuadOutput = 0x6B,

    /// dual line address, dual line data
    FastReadDualIo = 0xBB,
    /// quad line address, quad line data
    FastReadQuadIo = 0xEB,

    DeepPowerDown = 0xB9,
    ReleasePowerDown = 0xAB,

    /// Requires `WriteEnable` first
    ChipErase = 0xC7,
}

pub struct WinbondFlash {
    // TODO: Finish this
    clk_freq: u32,
    internal_flash: Flash,
    page_count: u16,
    regs: QUADSPI,
}

impl WinbondFlash {
    pub fn new(rcc: &mut RCC, regs: QUADSPI, stm_flash: FLASH, clk_config: &Clocks) -> Self {
        println!("New Flash!");

        let clk_freq = clk_config.apb1();

        // Read page count currently stored in stm flash
        // let page_count = stm_flash.read(bank, page, offset, buf);
        // if(page_count != )

        // // Write initial page count to the stm flash
        // let page_count = 0;
        // stm_flash.unlock();
        // stm_flash.erase_write_page(0, 0, 0); //Write 0 to bank 0, page 0 change to be correct size
        // stm_flash.lock();

        let _sck = Pin::new(Port::A, 3, PinMode::Alt(10));
        let _cs = Pin::new(Port::A, 2, PinMode::Alt(10));
        let _io0 = Pin::new(Port::B, 1, PinMode::Alt(10));
        let _io1 = Pin::new(Port::B, 0, PinMode::Alt(10));
        let _io2 = Pin::new(Port::A, 7, PinMode::Alt(10));
        let _io3 = Pin::new(Port::A, 6, PinMode::Alt(10));

        // Enable QSPI clocks
        rcc.ahb3enr().modify(|_, w| w.qspien().set_bit());
        rcc.ahb3rstr().modify(|_, w| w.qspirst().set_bit());
        rcc.ahb3rstr().modify(|_, w| w.qspirst().clear_bit());

        // Disable the QUADSPI peripheral before configuring it
        regs.cr().write(|w| w.en().disabled());

        // Wait for BUSY to clear
        while regs.sr().read().busy().is_busy() {}

        // We don't setup the ccr register here, since it might have different configurations on each use

        // Set the flash size, which is 512Mbits
        // The actual size is given by 2^(FSIZE + 1)
        let fsize = FLASH_SIZE_BYTES.ilog2() - 1;
        regs.dcr().modify(|_, w| w.fsize().set(fsize as u8));

        // Set the CLK speed
        regs.cr().modify(|_, w| w.prescaler().set(79));

        // Enable the QUADSPI peripheral
        regs.cr().modify(|_, w| w.en().enabled());

        let flash = Self {
            clk_freq,
            internal_flash: Flash::new(stm_flash),
            page_count: 0,
            regs,
        };

        flash
    }

    pub fn is_block_bad(&mut self, addr: u16) -> bool {
        self.regs.fcr().write(|w| w.ctcf().clear());

        self.regs.ccr().write(|w| {
            w.fmode()
                .indirect_write()
                .imode()
                .single_line()
                .instruction()
                .set(WinbondInstruction::PageDataRead as u8)
                .admode()
                .single_line()
                .adsize()
                .bit24() /* NOTE: the address is only 16 bits, but we need 8 dummy cycles between the instruction and address,
                   so the 8 MSB of the address (which will all be 0) are used for that purpose.
                   Also note that we cannot use the data section to send the address, since the STM32L412 has errata relating to using dummy cycles
                   along with data which causes the first nibble of the data to be lost if any dummy cycles are used
                */
                .dmode()
                .no_data()
        });

        self.regs
            .ar()
            .write(|w| w.address().set((addr * 16) as u32));

        while self.regs.sr().read().tcf().is_not_complete() {}
        self.regs.fcr().write(|w| w.ctcf().clear());

        // NOTE: the winbond datasheet says a 5us delays is needed before issuing another command after Page Data Read
        delay_us(5, self.clk_freq);

        while self.read_status_register(WinbondStatusReg::Three) & 1 != 0 {}

        // We only need the first byte of data returned, though the flash chip would continue outputting up to 2048 bytes if we wanted
        self.regs.dlr().write(|w| w.dl().set(0));
        self.regs.ccr().write(|w| {
            w.fmode()
                .indirect_read()
                .imode()
                .single_line()
                .instruction()
                .set(WinbondInstruction::FastReadQuadOutput as u8)
                .admode()
                .single_line()
                .adsize()
                .bit16()
                .dcyc()
                .set(8)
                .dmode()
                .four_lines()
        });

        self.regs.ar().write(|w| w.address().set(0));

        while self.regs.sr().read().tcf().is_not_complete() {}

        let word = self.regs.dr8().read().bits();

        word != 0xff
    }

    pub fn read_status_register(&mut self, r: WinbondStatusReg) -> u8 {
        // Protection (SR-1): Axh
        // Configuration (SR-2): Bxh
        // Status (SR-3):  Cxh
        // "accessed by Read Status Register and Write Status Register commands combined with 1-Byte Register Address respectively"

        self.regs.fcr().write(|w| w.ctcf().clear()); // Flag clear register, clear the transfer complete flag to 0 

        self.regs.dlr().write(|w| w.dl().set(0));

        self.regs.ccr().write(|w| {
            w.fmode()
                .indirect_read()
                .imode()
                .single_line()
                .instruction()
                .set(WinbondInstruction::ReadStatusRegister as u8)
                .admode()
                .single_line()
                .adsize()
                .bit8()
                .dmode()
                .single_line()
        });

        self.regs.ar().write(|w| w.address().set((r as u8) as u32));

        while self.regs.sr().read().tcf().is_not_complete() {}
        self.regs.fcr().write(|w| w.ctcf().clear());

        let word = self.regs.dr8().read().bits(); // data register 8 bit access, read the bits 

        // NOTE: This seems to be required to stop the code from either hanging or reporting wrong values on a following read... no clue why.
        for _ in 0..16 {
            nop();
        }

        word
    }

    pub fn write_page(&mut self, data: [u32; 512]) {
        // Check designated page is erased (FFh), if not, erase, if yes, continue

        // Write Enable command
        self.regs.fcr().write(|w| w.ctcf().clear());
        self.regs.ccr().write(|w| {
            w.fmode()
                .indirect_write()
                .imode()
                .single_line()
                .instruction()
                .set(WinbondInstruction::WriteEnable as u8) //instruction sent to the external device
        });

        while self.regs.sr().read().tcf().is_not_complete() {} // While the transfer complete flag is on, wait 
        delay_us(1, self.clk_freq);

        // Quad load program data command- loads data onto 2,112 byte buffer
        self.regs.fcr().write(|w| w.ctcf().clear());
        self.regs.dlr().write(|w| w.dl().set(2047));
        self.regs.ccr().write(|w| {
            w.fmode()
                .indirect_write() // QUADSPI functional mode of operation
                .imode()
                .single_line() // instruction phase mode of operation
                .instruction()
                .set(WinbondInstruction::QuadLoadProgramData as u8) //instruction sent to the external device
                .admode()
                .single_line()
                .adsize()
                .bit16()
                .dmode()
                .four_lines() // data phase mode of operation
        });

        self.regs.ar().write(|w| w.address().set(0));
        for word in data {
            self.regs.dr().write(|w| w.data().set(word));
        }

        while self.regs.sr().read().tcf().is_not_complete() {} // While the transfer complete flag is on, wait 
        delay_us(1, self.clk_freq);

        // Program execute- transfers the data from the buffer to the designated page
        self.regs.fcr().write(|w| w.ctcf().clear());
        self.regs.ccr().write(|w| {
            w.fmode()
                .indirect_write()
                .imode()
                .single_line()
                .instruction()
                .set(WinbondInstruction::ProgramExecute as u8) //instruction sent to the external device
                .admode()
                .single_line()
                .adsize()
                .bit24()
        });

        self.regs
            .ar()
            .write(|w| w.address().set(self.page_count as u32));
        while self.regs.sr().read().tcf().is_not_complete() {} // While the transfer complete flag is on, wait 
        delay_us(1, self.clk_freq);

        //TODO: implement this in not pseudo code

        // if (success){
        //     page_count += 1;
        //     stm_flash.unlock();
        //     stm_flash.erase_write_page(0, 0, page_count);
        //     stm_flash.lock();
        // }

        self.page_count += 1;
    }
}

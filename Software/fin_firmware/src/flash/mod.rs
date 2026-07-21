use core::ptr;

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

#[repr(u8)]
pub enum WinbondStatusReg {
    One = 0xA0,
    Two = 0xB0,
    Three = 0xC0,
}

pub struct WinbondFlash {
    // TODO: Finish this
    clk_freq: u32,
    flash: Flash,
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
        regs.cr().write(|w| w.en().clear_bit());

        // Wait for BUSY to clear
        while regs.sr().read().busy().is_busy() {}

        // We don't setup the ccr register here, since it might have different configurations on each use

        // Set the flash size, which is 512Mbits
        // The actual size is given by 2^(FSIZE + 1)
        let fsize = FLASH_SIZE_BYTES.ilog2() - 1;
        regs.dcr()
            .modify(|_, w| unsafe { w.fsize().bits(fsize as u8) });

        // Set the CLK speed
        regs.cr().modify(|_, w| unsafe { w.prescaler().bits(79) });

        // Enable the QUADSPI peripheral
        regs.cr().modify(|_, w| w.en().set_bit());

        let flash = Self {
            clk_freq,
            flash: Flash::new(stm_flash),
            page_count: 0,
            regs,
        };

        flash
    }

    pub fn is_block_bad(&mut self, addr: u16) -> bool {
        self.regs.fcr().write(|w| w.ctcf().clear());
        self.regs.dlr().write(|w| unsafe { w.dl().bits(1) });
        self.regs.ccr().write(|w| unsafe {
            w.admode()
                .bits(0b00)
                .imode()
                .bits(0b01)
                .fmode()
                .bits(0b00)
                .instruction()
                .bits(0x13)
                .dmode()
                .bits(0b01)
                .dcyc()
                .bits(8)
        });

        self.regs
            .dr16()
            .write(|w| unsafe { w.data().bits(addr * 16) });

        delay_us(50, self.clk_freq);

        while self.read_status_register(WinbondStatusReg::Three) & 1 != 0 {}

        self.regs.dlr().write(|w| unsafe { w.bits(0) });
        self.regs.ccr().write(|w| unsafe {
            w.admode()
                .bits(0b01)
                .imode()
                .bits(0b01)
                .adsize()
                .bits(0b01)
                .fmode()
                .bits(0b01)
                .instruction()
                .bits(0x6b)
                .dmode()
                .bits(0b11)
                .dcyc()
                .bits(8)
        });

        self.regs.ar().write(|w| unsafe { w.bits(0) });

        while self.regs.sr().read().tcf().is_not_complete() {}

        let word = self.regs.dr8().read().bits();

        word != 0xff
    }

    pub fn read_status_register(&mut self, r: WinbondStatusReg) -> u8 {
        // Protection (SR-1): Axh
        // Configuration (SR-2): Bxh
        // Status (SR-3):  Cxh
        // "accessed by Read Status Register and Write Status Register commands combined with 1-Byte Register Address respectively"
        let op_code = 0x0F;

        self.regs.fcr().write(|w| w.ctcf().clear()); // Flag clear register, clear the transfer complete flag to 0 

        self.regs.dlr().write(|w| unsafe { w.bits(0) }); // Data length register, set the data length to num + 1 

        // Communication configuration register
        // This configuration can be found in the Winbond data sheet
        self.regs.ccr().write(
            |w| {
                w.admode()
                    .single_line()
                    .imode()
                    .single_line() // instruction phase mode of operation: 01 instruction on a single line
                    .instruction()
                    .set(op_code) //instruction sent to the external device
                    .dmode()
                    .single_line() // data phase mode of operation: 01 is data on a single line
                    .fmode()
                    .indirect_read()
            }, // QUADSPI functional mode of operation: 01 is indirect read mode
        );

        self.regs
            .ar()
            .write(|w| unsafe { w.bits((r as u8) as u32) }); // Address register- write status register 

        while self.regs.sr().read().tcf().is_not_complete() {} // While the transfer complete flag is on, wait 

        let word = self.regs.dr8().read().bits(); // data register 8 bit access, read the bits 

        delay_us(1, self.clk_freq);

        word
    }

    pub fn write_page(&mut self, page: u16, data: [u32; 512]) {
        let we_op_code = 0x06;
        let lpd_op_code = 0x32;
        let pe_op_code = 0x10;

        // Check designated page is erased (FFh), if not, erase, if yes, continue
        

        // Write Enable command
        self.regs.fcr().write(|w| w.ctcf().clear());
        self.regs.ccr().write(|w| {
            w.imode()
                .single_line()
                .instruction()
                .set(we_op_code) //instruction sent to the external device
                .fmode()
                .indirect_write()
        });

        while self.regs.sr().read().tcf().is_not_complete() {} // While the transfer complete flag is on, wait 
        delay_us(1, self.clk_freq);

        // Quad load program data command- loads data onto 2,112 byte buffer
        self.regs.fcr().write(|w| w.ctcf().clear());
        self.regs.dlr().write(|w| w.dl().set(2047));
        self.regs.ccr().write(|w| {
            w.admode()
                .single_line()
                .imode()
                .single_line() // instruction phase mode of operation
                .instruction()
                .set(lpd_op_code) //instruction sent to the external device
                .dmode()
                .four_lines() // data phase mode of operation
                .fmode()
                .indirect_write() // QUADSPI functional mode of operation
        });

        self.regs.ar().write(|w| w.address().set(0)); 
        for i in data{
            self.regs.dr().write(|w| w.data().set(i)); 
        }
        
        while self.regs.sr().read().tcf().is_not_complete() {} // While the transfer complete flag is on, wait 
        delay_us(1, self.clk_freq);

        // Program execute- transfers the data from the buffer to the designated page
        self.regs.fcr().write(|w| w.ctcf().clear());
        self.regs.ccr().write(|w| {
            w.admode()
                .single_line()
                .imode()
                .single_line()
                .instruction()
                .set(pe_op_code) //instruction sent to the external device
                .fmode()
                .indirect_write()
        });

        self.regs.ar().write(|w| w.address().set(0b000)); 
        while self.regs.sr().read().tcf().is_not_complete() {} // While the transfer complete flag is on, wait 
        delay_us(1, self.clk_freq);

        //TODO: implement this in not pseudo code

        // if (success){
        //     page_count += 1;
        //     stm_flash.unlock();
        //     stm_flash.erase_write_page(0, 0, page_count);
        //     stm_flash.lock();
        // }
    }
}

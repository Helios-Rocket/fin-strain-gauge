use core::ptr;

use defmt::{info, println};
use hal::{
    clocks::Clocks,
    delay_ms, delay_us,
    gpio::{Pin, PinMode, Port},
    pac::{QUADSPI, RCC},
};

pub const BYTES_PER_PAGE: u32 = 2048;
pub const PAGES_PER_BLOCK: u32 = 64;
pub const FLASH_SIZE_BYTES: u32 = 512 * PAGES_PER_BLOCK * BYTES_PER_PAGE;

pub struct WinbondFlash {
    // TODO: Finish this
    clk_freq: u32,
    regs: QUADSPI,
}

impl WinbondFlash {
    pub fn new(rcc: &mut RCC, regs: QUADSPI, clk_config: &Clocks) -> Self {
        println!("New Flash!");

        let clk_freq = clk_config.apb1();

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

        // Enable the QUADSPI peripheral
        regs.cr().write(|w| w.en().set_bit());

        let flash = Self { clk_freq, regs };

        flash
    }

    pub fn is_block_bad(&mut self, addr: u16) -> bool {
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

        while self.read_status_registers() & 1 != 0 {}

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

    pub fn read_status_registers(&mut self) -> u8 {
        // Protection (SR-1): Axh
        // Configuration (SR-2): Bxh
        // Status (SR-3):  Cxh
        // "accessed by Read Status Register and Write Status Register commands combined with 1-Byte Register Address respectively"
        let op_code = 0x0F;
        let sr_addr = 0xC0;

        self.regs.dlr().write(|w| unsafe { w.bits(0) });

        self.regs.ccr().write(|w| unsafe {
            w.admode()
                .bits(0b01)
                .imode()
                .bits(0b01)
                .instruction()
                .bits(op_code)
                .dmode()
                .bits(0b01)
                .fmode()
                .bits(0b01)
        });

        self.regs.ar().write(|w| unsafe { w.bits(sr_addr) });

        let word = self.regs.dr8().read().bits();

        word
    }
}

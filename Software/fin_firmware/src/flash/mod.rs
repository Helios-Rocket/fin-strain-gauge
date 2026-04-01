use core::ptr;

use defmt::{info, println};
use hal::{
    clocks::Clocks,
    delay_ms, delay_us,
    gpio::{Pin, PinMode, Port},
    pac::QUADSPI,
    qspi::{Qspi, QspiConfig},
};

// #[derive(Copy, Clone)]

pub struct Flash {
    // TODO: Finish this
    qspi: Qspi,
    cs_pin: Pin,
    clk_freq: u32,
    regs: QUADSPI,
}

impl Flash {
    pub fn new(regs: QUADSPI, clk_config: &Clocks) -> Self {
        println!("New Flash!");
        // FIXME: add in pin config if needed, any other configuration

        let mut cs_pin = Pin::new(Port::A, 2, PinMode::Output);
        let clk_freq = clk_config.apb1();

        let _sck = Pin::new(Port::A, 3, PinMode::Alt(10));
        let _io0 = Pin::new(Port::B, 1, PinMode::Alt(10));
        let _io1 = Pin::new(Port::B, 0, PinMode::Alt(10));
        let _io2 = Pin::new(Port::A, 7, PinMode::Alt(10));
        let _io3 = Pin::new(Port::A, 6, PinMode::Alt(10));

        let qspi_config = QspiConfig {
            //TODO: Check configuration values
            protocol_mode: hal::qspi::ProtocolMode::Single,
            ..Default::default()
        };

        let qspi = Qspi::new(regs, qspi_config, clk_config).unwrap();
        cs_pin.set_high(); // unassert cs

        let mut flash = Self {
            qspi,
            cs_pin,
            clk_freq,
            regs: unsafe { QUADSPI::steal() },
        };

        flash
    }

    fn assert_cs(&mut self) {
        self.cs_pin.set_low();

        delay_us(1, self.clk_freq);
    }

    fn unassert_cs(&mut self) {
        delay_us(1, self.clk_freq);

        self.cs_pin.set_high();
    }

    // fn read_register(){
    //     // cs active low
    //     //Writes on rising edge of clk and reads on falling edge
    // }

    pub fn read_status_registers(&mut self) -> u8 {
        // Protection (SR-1): Axh
        // Configuration (SR-2): Bxh
        // Status (SR-3):  Cxh
        // "accessed by Read Status Register and Write Status Register commands combined with 1-Byte Register Address respectively"
        self.assert_cs();

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

        let word: u8 = unsafe { ptr::read_volatile(self.regs.dr().as_ptr() as *mut u8) };

        self.unassert_cs();

        word
    }
}

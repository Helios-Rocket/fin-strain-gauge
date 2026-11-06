// TODO: Move adc spi into a global variable, change functions to call global var

//powers up in standby, will need to be switched to a measurement mode(HP) and wait 2 ms
// put in 16g mode

// Has a self test mode, ignore for now but good to know
// consider filters- may be useful for noise given the application, consider status and error handling

use defmt::println;
use embedded_hal::spi::{self, Mode};
use hal::{access_global, clocks::Clocks, delay_ms, delay_us, gpio::Pin};

use crate::SPI;

pub struct ADXL {
    cs_pin: Pin,
    clk_freq: u32,
    // Will prob need const values for config
}

impl ADXL {
    pub fn new(clk_config: &Clocks, mut cs_pin: Pin) -> Self {
        println!("New ADXL!");

        // Needs to be passed in since there are two accelerometers
        cs_pin.set_high(); //FIXME: Figure out where to move this, maybe make adc and adxl pins globals? 

        let clk_freq = clk_config.apb1();
        delay_ms(2, clk_freq); // Put between standby and measure op mode

        let mut adxl = Self { cs_pin, clk_freq };

        adxl
    }

    fn assert_cs(&mut self) {
        self.cs_pin.set_low();

        delay_us(1, self.clk_freq);
    }

    fn unassert_cs(&mut self) {
        self.cs_pin.set_high();

        delay_us(1, self.clk_freq);
    }

    pub fn read_status_register(&mut self) {
        // Read register 0x00 and expect result 0xAD
        let reg = self.read_register(0x00);
        println!("ADXL Status Reg Val {:x}", reg);
    }

    fn read_register(&mut self, addr: u8) -> u8 {
        let cmd: u8 = (addr << 1) | 0b1;

        let mut buf = [cmd, 0];

        self.assert_cs();

        //FIXME: Do cs assertions need to be in the critical section, idk if it'll mess with things if both periphs assert at the same time...
        critical_section::with(|cs| {
            access_global!(SPI, spi, cs);
            spi.disable().unwrap();
            spi.regs
                .cr1()
                .modify(|_, w| w.cpha().clear_bit().spe().set_bit());
            spi.transfer(&mut buf)
                .expect("Spi Write Command Failed!");
        });

        self.unassert_cs();

        buf[1]
    }

    pub fn write_register(&mut self, addr: u8, data: u8) {
        let buf = [addr << 1, data];

        self.assert_cs();

        critical_section::with(|cs| {
            access_global!(SPI, spi, cs);
            spi.write(&buf)
                .expect("Spi Write Command Failed!");
        });

        self.unassert_cs();
    }

    pub fn set_op_mode(&mut self) {
        // 1. Write 0xF0 to Register 0x27 (DIG_EN).
        self.write_register(0x27, 0xF0); // Enables x, y, z, and temperature channels.
        // 2. Write 0x0C to Register 0x26 (OP_MODE).
        self.write_register(0x26, 0x0C); // Enables HP mode.
    }

    pub fn read_accel_registers(&mut self) -> [u16; 3] {
        // X reading
        let x_high = self.read_register(0x15);
        let x_low = self.read_register(0x16);
        let x: u16 = (x_high as u16) << 8 | x_low as u16;

        // Y reading
        let y_high = self.read_register(0x17);
        let y_low = self.read_register(0x18);
        let y: u16 = (y_high as u16) << 8 | y_low as u16;

        // Z reading
        let z_high = self.read_register(0x19);
        let z_low = self.read_register(0x1A);
        let z: u16 = (z_high as u16) << 8 | z_low as u16;

        [x, y, z]
    }
}

use defmt::{info, Format};
use esp_hal::{
    gpio::{Level, Output, OutputConfig, OutputPin},
    spi::master::Spi,
    Blocking,
};

const CTRL1_XL_ADDR: u8 = 0x10;
const CTRL2_G_ADDR: u8 = 0x11;
const OUT_REG_START_ADDR: u8 = 0x22;
const STATUS_REG_ADDR: u8 = 0x1E;

const XL_FS_32G: u8 = 0b01;
const XL_ODR_166kHZ: u8 = 0b1000;
const G_FS_32G: u8 = 0b11;
const G_ODR_166kHZ: u8 = 0b1000;

pub struct Lsm<'a> {
    spi: Spi<'a, Blocking>,
    cs: Output<'a>,
}

#[derive(Format)]
pub struct Data {
    accel_x: u16,
    accel_y: u16,
    accel_z: u16,
    gyro_x: u16,
    gyro_y: u16,
    gyro_z: u16,
}

impl<'a> Lsm<'a> {
    pub fn new(spi: Spi<'a, Blocking>, cs_pin: impl OutputPin + 'a) -> Self {
        // Write the accel and gyro control registers
        // Set max g's
        // Enable the axis + set ODR
        let mut this = Self {
            spi,
            cs: Output::new(cs_pin, Level::High, OutputConfig::default()),
        };

        this.write_register(CTRL1_XL_ADDR, XL_ODR_166kHZ << 4 | XL_FS_32G << 2);
        this.write_register(CTRL2_G_ADDR, G_ODR_166kHZ << 4 | G_FS_32G << 2);

        this
    }

    fn read_register(&mut self, addr: u8, buf: &mut [u8]) {
        let command: u8 = 1 << 7 | addr;
        self.cs.set_low();

        self.spi.write(&mut [command]).unwrap();
        self.spi.read(buf).unwrap();

        self.cs.set_high();
    }

    fn write_register(&mut self, addr: u8, data: u8) {
        let command: u16 = (addr as u16) << 8 | (data as u16);
        self.cs.set_low();

        self.spi.write(&command.to_be_bytes()).unwrap();

        self.cs.set_high();
    }

    pub fn read_lsm(&mut self) -> Data {
        let mut buf = [0_u8; 12];
        self.read_register(OUT_REG_START_ADDR, &mut buf);

        Data {
            gyro_x: (buf[0] as u16) | (buf[1] as u16) << 8,
            gyro_y: (buf[2] as u16) | (buf[3] as u16) << 8,
            gyro_z: (buf[4] as u16) | (buf[5] as u16) << 8,
            accel_x: (buf[6] as u16) | (buf[7] as u16) << 8,
            accel_y: (buf[8] as u16) | (buf[9] as u16) << 8,
            accel_z: (buf[10] as u16) | (buf[11] as u16) << 8,
        }
    }
}

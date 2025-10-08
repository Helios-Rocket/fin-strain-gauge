use esp_hal::{spi::master::Spi, Blocking};

pub struct Lsm<'a> {
    spi: Spi<'a, Blocking>,
}

#[derive(Debug)]
pub struct Data{
    accel_x: u16,
    accel_y: u16, 
    accel_z: u16, 
    gyro_x: u16, 
    gyro_y: u16, 
    gyro_z: u16, 
}

impl<'a> Lsm<'a> {
    pub fn new(spi: Spi<'a, Blocking>) -> Self {

        // Write the accel and gyro control registers 
        let accel_command: u8 = 1_u8 << 7;  
        write_register(self, 0x10h, )
        Self { spi }
    }

    fn read_register(self: &mut Self, addr: u8, buf: &mut[u8]){
        let command: u8 = 1_u16 << 7 | addr;
        self.spi.write(&[command]);
        self.spi.read(buf).unwrap();
    }

    fn write_register(self: &mut Self, addr: u8, data: u8) {
        let command: u16 = (addr as u16) << 8 | (data as u16);
        self.spi.write(&command.to_be_bytes()).unwrap();
    }

    pub fn read_lsm(self: &mut Self) -> Data{
        let mut buf = [0_u8; 12]; 
        self.read_register(0x22, &mut buf); 

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


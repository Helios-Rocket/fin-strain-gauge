use esp_hal::spi::{
    master::Spi,
};

pub struct Lsm{
    spi: Spi, 
}

impl Lsm{
    pub fn new(spi: Spi) -> Self{
        Self{spi}
    }

    pub fn read_register(self: &mut Self, addr: u8) -> u8{
        let command: u16 = 1_u16 << 15 | (addr as u16) << 8;
        let mut buf = command.to_be_bytes(); 
        self.spi.transfer(&mut buf).unwrap(); 
        buf[1]  
    }

    pub fn write_register(self: &mut Self, addr: u8, data: u8){
        let command: u16 = (addr as u16) << 8 | (data as u16);  
        self.spi.write(command.to_be_bytes()); 
    }


}

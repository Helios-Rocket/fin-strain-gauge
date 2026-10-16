use esp_hal::{i2c::master::I2c, Blocking};

pub struct Mpl<'a> {
    i2c: I2c<'a, Blocking>,
}

impl<'a> Mpl<'a> {
    pub fn new(i2c: I2c<'a, Blocking>) -> Self {
        Self { i2c }
    }
}

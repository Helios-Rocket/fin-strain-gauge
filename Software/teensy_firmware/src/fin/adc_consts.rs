#![allow(unused)]

#[derive(Copy, Clone, Debug)]
pub enum AdcChannel {
    CH0,
    CH1,
    CH2,
}

pub mod registers {
    pub mod mode {
        pub const ADDR: u8 = 0x2;

        pub mod wlength {
            pub const SHIFT: u16 = 8;
            pub const MASK: u16 = 0b11 << SHIFT;

            #[repr(u8)]
            #[derive(Copy, Clone, Debug)]
            pub enum Length {
                Bits16 = 0b00,
                Bits24 = 0b01,
                Bits32ZeroPad = 0b10,
                Bits32SignExtend = 0b11,
            }

            impl Length {
                fn shift(self: Self) -> u16 {
                    (self as u16) << SHIFT
                }
            }
        }
    }

    pub mod clock {
        pub const ADDR: u8 = 0x3;

        pub const DISABLE_ALL_CHANNEL_MASK: u16 = !(0b111 << 8);
        pub const ENABLE_ALL_CHANNEL_MASK: u16 = 0b111 << 8;
    }

    pub mod gain {
        pub const ADDR: u8 = 0x4;

        #[repr(u8)]
        #[derive(Copy, Clone, Debug)]
        pub enum Gain {
            Times1 = 0b000,
            Times2 = 0b001,
            Times4 = 0b010,
            Times8 = 0b011,
            Times16 = 0b100,
            Times32 = 0b101,
            Times64 = 0b110,
            Times128 = 0b111,
        }

        impl Gain {
            pub fn to_multiplier(self) -> u32 {
                2_u32.pow(self as _)
            }
        }

        impl TryFrom<u8> for Gain {
            type Error = u8;

            fn try_from(value: u8) -> Result<Self, Self::Error> {
                if value <= 0b111 {
                    Ok(unsafe { core::mem::transmute(value) })
                } else {
                    Err(value)
                }
            }
        }

        pub mod ch0 {
            use super::Gain;

            pub const SHIFT: u16 = 0;
            pub const MASK: u16 = 0b111 << SHIFT;

            pub fn set(g: Gain) -> u16 {
                (g as u16) << SHIFT
            }

            pub fn get(reg: u16) -> Gain {
                (((reg & MASK) >> SHIFT) as u8).try_into().unwrap()
            }
        }

        pub mod ch1 {
            use super::Gain;

            pub const SHIFT: u16 = 4;
            pub const MASK: u16 = 0b111 << SHIFT;

            pub fn set(g: Gain) -> u16 {
                (g as u16) << SHIFT
            }

            pub fn get(reg: u16) -> Gain {
                (((reg & MASK) >> SHIFT) as u8).try_into().unwrap()
            }
        }

        pub mod ch2 {
            use super::Gain;

            pub const SHIFT: u16 = 8;
            pub const MASK: u16 = 0b111 << SHIFT;

            pub fn set(g: Gain) -> u16 {
                (g as u16) << SHIFT
            }

            pub fn get(reg: u16) -> Gain {
                (((reg & MASK) >> SHIFT) as u8).try_into().unwrap()
            }
        }
    }
}

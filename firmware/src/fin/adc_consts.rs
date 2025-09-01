pub mod registers {
    pub mod mode {
        pub const ADDR: u8 = 0x2;

        pub mod wlength {
            pub const SHIFT: u16 = 8;
            pub const MASK: u16 = 0b11 << SHIFT;

            pub const LENGTH_32BITS: u16 = 0b10 << SHIFT;
            pub enum Length {}
        }
    }

    pub mod clock {
        pub const ADDR: u8 = 0x3;

        pub const DISABLE_ALL_CHANNEL_MASK: u16 = !(0b111 << 8);
        pub const ENABLE_ALL_CHANNEL_MASK: u16 = 0b111 << 8;
    }
}

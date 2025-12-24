use bitfield::bitfield;

#[repr(u8)]
pub enum SdDmaOwner {
    Host = 0,
    DmaController = 1,
}

impl From<u8> for SdDmaOwner {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Host,
            1 => Self::DmaController,
            _ => panic!("Invalid owner"),
        }
    }
}

impl Into<u8> for SdDmaOwner {
    fn into(self) -> u8 {
        self as _
    }
}

bitfield! {
    pub struct SdDmaDescriptorFlags(u32);
    u32;
    disable_int, set_disable_int: 1;
    last, set_last: 2;
    first, set_first: 3;
    second_addr_chained, set_second_addr_chained: 4;
    end_of_ring, set_end_of_ring: 5;
    card_error, _: 30;
    u8, from into SdDmaOwner, owner, set_owner: 31, 31;
}

pub struct SdDmaDescriptor {
    flags: SdDmaDescriptorFlags,
    buf_size: u32,
    buf_ptr: *mut u8,
    next_descriptor: *mut SdDmaDescriptor,
}

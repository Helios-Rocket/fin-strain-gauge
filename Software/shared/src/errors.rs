use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, Serialize, Deserialize, MaxSize)]
#[repr(u8)]
pub enum WinbondFlashError {
    FailToErase,
    FailToWrite,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize, MaxSize)]
#[repr(u8)]
pub enum AdcError {
    CRC {
        #[serde(skip)]
        computed: u16,
    },
    TransferError,
}

#[derive(MaxSize)]
pub struct FinStatus {
    pub adc_count: u16,
    pub last_adc: Option<AdcError>,
    pub winbond_count: u16,
    pub last_winbond: Option<WinbondFlashError>,
}

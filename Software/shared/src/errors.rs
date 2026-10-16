#[derive(Debug, Copy, Clone)]
#[repr(u8)]
pub enum WinbondFlashError {
    FailToErase,
    FailToWrite,
}

#[derive(Debug, Copy, Clone)]
#[repr(u8)]
pub enum AdcError {
    CRC,
}

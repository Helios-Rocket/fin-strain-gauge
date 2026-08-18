#[repr(u8)]

pub enum FinCommands{
    RecordData,
    StopRecord,
    EraseFlash,
}
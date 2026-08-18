#[repr(u8)]

pub enum FinCommands{
    Success, 
    Failure,
    RecordData,
    StopRecord,
    EraseFlash,
}
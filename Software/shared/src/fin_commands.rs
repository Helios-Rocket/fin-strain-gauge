use num_enum::TryFromPrimitive;

#[derive(Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum FinCommands {
    Success,
    Failure,
    RecordData,
    StopRecord,
    EraseFlash,
}

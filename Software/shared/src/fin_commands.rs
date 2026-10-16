use num_enum::TryFromPrimitive;

#[derive(Copy, Clone, Eq, PartialEq, TryFromPrimitive, Debug)]
#[repr(u8)]
pub enum FinCommands {
    Success,
    Failure,
    RecordData,
    StopRecord,
    EraseFlash,
    SendStatus,
}

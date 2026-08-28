use num_enum::TryFromPrimitive;

#[derive(Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum Command {
    EraseFinFlashes,
    ArmAltimeter,
    DisarmAltimeter,
    StartLiveVideo,
    StopLiveVideo,
    ArmFins,
    DisarmFins,
}

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

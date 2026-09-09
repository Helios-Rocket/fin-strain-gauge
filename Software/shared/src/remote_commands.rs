use num_enum::TryFromPrimitive;

#[derive(Copy, Clone, Eq, PartialEq, TryFromPrimitive, Debug)]
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

pub static REMOTE_HEADER: &[u8] = b"HELIOS_REMOTE_CMD";

pub static COMMANDS: [(&str, Command); 7] = [
    ("fin erase", Command::EraseFinFlashes),
    ("fin arm", Command::ArmFins),
    ("fin disarm", Command::DisarmFins),
    ("altimeter arm", Command::ArmAltimeter),
    ("altimeter disarm", Command::DisarmAltimeter),
    ("video start", Command::StartLiveVideo),
    ("video stop", Command::StopLiveVideo),
];

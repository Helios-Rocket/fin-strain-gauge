#[derive(defmt::Format, Debug, Copy, Clone)]
pub enum FinState {
    WaitForCommand,
    WaitForRecordPulse,
    SendStatus,
    RecordData,
    StopRecord,
    EraseFlash,
}


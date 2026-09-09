#[derive(defmt::Format, Debug, Copy, Clone)]
pub enum FinState {
    WaitForCommand,
    WaitForRecordPulse,
    SendStatus,
    RecordData,
    StopRecord,
    EraseFlash,
    Error,
}

impl FinState {
    pub fn new(flight_flag: bool) -> Self {
        if flight_flag == true {
            FinState::RecordData
        } else {
            FinState::WaitForCommand
        }
    }
}

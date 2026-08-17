
pub enum FinStateMachine {
    WaitForCommand,
    WaitForRecordPulse,
    RecordData,
    StopRecord,
    EraseFlash,
    Error,
}

pub enum Event {
    EraseCommand,
    RecordCommand,
    RecordPulseReceived,
    StopCommand,
    Success,
    Fail,
    // Differentiate between adc crc failure and flash failure stuff 
}

impl FinStateMachine {
    pub fn new(flight_flag: bool) -> Self {
        if flight_flag {
            FinStateMachine::RecordData
        } else {
            FinStateMachine::WaitForCommand
        }
    }

    pub fn next(self, event: Event) -> Self {
        match (self, event) {
            (Self::WaitForCommand, Event::EraseCommand) => Self::EraseFlash,
            (Self::WaitForCommand, Event::RecordCommand) => Self::WaitForRecordPulse,
            (Self::WaitForRecordPulse, Event::RecordPulseReceived) => Self::RecordData,
            (Self::EraseFlash, Event::Success) => Self::WaitForCommand,
            (Self::EraseFlash, Event::Fail) => Self::Error,
            (Self::RecordData, Event::Fail) => Self::Error,
            (Self::RecordData, Event::StopCommand) => Self::StopRecord,
            (Self::StopRecord, Event::Success) => Self::WaitForCommand,
            (Self::StopRecord, Event::Fail) => Self::Error,
            (state, _) => state,
        }
    }
}

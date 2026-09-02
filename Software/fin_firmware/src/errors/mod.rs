pub enum FinError{
    AdcCrc, 
    AdcSPIFailToRead, 
    ADCSPIFailToWrite,
    
}

impl From<AdcError> for FinError {
    fn from(value: AdcError) -> Self {
        Self::Adc(value)
    }
}

impl From<WinbondFlashError> for FinError {
    fn from(value: WinbondFlashError) -> Self {
        Self::WinbondFlash(value)
    }
}

impl From<StmFlashError> for FinError {
    fn from(value: StmFlashError) -> Self {
        Self::StmFlash(value)
    }
}
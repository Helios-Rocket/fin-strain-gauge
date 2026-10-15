#[derive(Debug, Copy, Clone)]
#[derive(serde::Serialize)]
#[repr(u8)]
pub enum WinbondFlashError {
    FailToErase,
    FailToWrite,
}

#[derive(Debug, Copy, Clone)]
#[derive(Serialize, Deserialize)]
#[repr(u8)]
pub enum AdcError {
    CRC {#[serde(skip)] computed: u16 },
    TransferError,
}

pub struct FinStatus{
    adc_count: i64, 
    last_adc: Option<AdcError>, 
    winbond_count: i64, 
    last_winbond: Option<WinbondFlashError>,
}

impl FinStatus{
    pub fn new(adc_count:i64, last_adc: Option<AdcError>, winbond_count: i64, last_winbond: Option<WinbondFlashError>) -> Self{
        let fin_status = Self{
            adc_count, 
            last_adc, 
            winbond_count, 
            last_winbond,
        }; 
        Self
    }
}
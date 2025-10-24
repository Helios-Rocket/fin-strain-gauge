pub mod response_status {
    // See https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf#Regfloat.34.17
    pub const CD_MASK: u16 = 1 << 0;
    pub const RE_MASK: u16 = 1 << 1;
    pub const CMD_MASK: u16 = 1 << 2;
    pub const DTO_MASK: u16 = 1 << 3;
    pub const TXDR_MASK: u16 = 1 << 4;
    pub const RXDR_MASK: u16 = 1 << 5;
    pub const RCRC_MASK: u16 = 1 << 6;
    pub const DCRC_MASK: u16 = 1 << 7;
    pub const RTO_MASK: u16 = 1 << 8;
    pub const DTRO_MASK: u16 = 1 << 9;
    pub const HTO_MASK: u16 = 1 << 10;
    pub const FRUN_MASK: u16 = 1 << 11;
    pub const HLE_MASK: u16 = 1 << 12;
    pub const SBE_MASK: u16 = 1 << 13;
    pub const ACD_MASK: u16 = 1 << 14;
    pub const EBE_MASK: u16 = 1 << 15;

    pub const DEFAULT_EVENTS: u16 = CD_MASK
        | RE_MASK
        | CMD_MASK
        | DTO_MASK
        | RCRC_MASK
        | DCRC_MASK
        | RTO_MASK
        | DTRO_MASK
        | HTO_MASK
        | HLE_MASK
        | SBE_MASK
        | EBE_MASK;
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub(crate) enum ResponseLength {
    None,
    Long,
    Short { check_busy: bool },
}

#[derive(Copy, Clone)]
pub(crate) struct ResponseConfig {
    pub(crate) check_index: bool,
    pub(crate) check_crc: bool,
    pub(crate) length: ResponseLength,
}

#[allow(unused)]
impl ResponseConfig {
    /// No response
    pub(crate) const RZ: Self = Self {
        check_index: false,
        check_crc: false,
        length: ResponseLength::None,
    };

    /// Default response with cmd index and crc.
    pub(crate) const R1: Self = Self {
        check_index: true,
        check_crc: true,
        length: ResponseLength::Short { check_busy: false },
    };

    /// Same as R1 but with busy signal.
    #[allow(non_upper_case_globals)]
    pub(crate) const R1b: Self = Self {
        check_index: true,
        check_crc: true,
        length: ResponseLength::Short { check_busy: true },
    };

    /// Long response for CID, CSD. No cmd index or crc.
    pub(crate) const R2: Self = Self {
        check_index: false,
        check_crc: false,
        length: ResponseLength::Long,
    };

    /// OCR Register
    pub(crate) const R3: Self = Self {
        check_index: false,
        check_crc: false,
        length: ResponseLength::Short { check_busy: false },
    };

    /// RCA response
    pub(crate) const R6: Self = Self {
        check_index: true,
        check_crc: true,
        length: ResponseLength::Short { check_busy: false },
    };

    /// Card interface condition
    pub(crate) const R7: Self = Self {
        check_index: true,
        check_crc: true,
        length: ResponseLength::Short { check_busy: false },
    };
}

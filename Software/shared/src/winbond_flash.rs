#[repr(u8)]
pub enum WinbondStatusReg {
    One = 0xA0,
    Two = 0xB0,
    Three = 0xC0,
}

#[repr(u8)]
pub enum WinbondInstruction {
    /// This can be used on its own and is a sort of "soft reset"
    DeviceReset = 0xFF,

    /// The following two commands must be used together and are a sort of "hard reset," including resetting protection and configuration registers
    EnableReset = 0x66,
    /// See `EnableReset` (which must be used first)
    ResetDevice = 0x99,

    ReadJedecId = 0x9F,

    ReadStatusRegister = 0x0F,
    WriteStatusRegister = 0x1F,

    WriteEnable = 0x06,
    /// NOTE: this is often not necessary, as WEL is reset after most commands that require it to be set
    WriteDisable = 0x04,

    /// Requires `WriteEnable` first
    BadBlockMgmnt = 0xA1,
    ReadBbmLut = 0xA5,

    LastEccFailPageAddr = 0xA9,

    /// Requires `WriteEnable` first
    BlockErase = 0xD8,

    /// Requires `WriteEnable` first
    LoadProgramData = 0x02,
    /// Requires `WriteEnable` first
    RandomLoadProgramData = 0x84,

    /// Requires `WriteEnable` first
    QuadLoadProgramData = 0x32,
    /// Requires `WriteEnable` first
    QuadRandomLoadProgramData = 0x34,

    /// Requires `WriteEnable` and `*LoadProgramData` first
    ProgramExecute = 0x10,

    PageDataRead = 0x13,

    ReadData = 0x03,
    FastRead = 0x0B,

    /// single line address, dual line data
    FastReadDualOutput = 0x3B,
    /// single line address, quad line data
    FastReadQuadOutput = 0x6B,

    /// dual line address, dual line data
    FastReadDualIo = 0xBB,
    /// quad line address, quad line data
    FastReadQuadIo = 0xEB,

    DeepPowerDown = 0xB9,
    ReleasePowerDown = 0xAB,

    /// Requires `WriteEnable` first
    ChipErase = 0xC7,
}

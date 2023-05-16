// Device settings
pub(crate) const ID_ADDR: u8 = 0x0;
pub(crate) const STATUS_ADDR: u8 = 0x1;

// Global across channels
pub(crate) const MODE_ADDR: u8 = 0x2;
pub(crate) const CLOCK_ADDR: u8 = 0x3;
pub(crate) const GAIN_ADDR: u8 = 0x4;
pub(crate) const CFG_ADDR: u8 = 0x6;
pub(crate) const THRSHLD_MSB_ADDR: u8 = 0x7;
pub(crate) const THRSHLD_LSB_ADDR: u8 = 0x8;

// Channel 0
pub(crate) const CH0_CFG_ADDR: u8 = 0x9;
pub(crate) const CH0_OCAL_MSB_ADDR: u8 = 0xA;
pub(crate) const CH0_OCAL_LSB_ADDR: u8 = 0xB;
pub(crate) const CH0_GCAL_MSB_ADDR: u8 = 0xC;
pub(crate) const CH0_GCAL_LSB_ADDR: u8 = 0xD;

// Channel 1
pub(crate) const CH1_CFG_ADDR: u8 = 0xE;
pub(crate) const CH1_OCAL_MSB_ADDR: u8 = 0xF;
pub(crate) const CH1_OCAL_LSB_ADDR: u8 = 0x10;
pub(crate) const CH1_GCAL_MSB_ADDR: u8 = 0x11;
pub(crate) const CH1_GCAL_LSB_ADDR: u8 = 0x12;

// Channel 2
pub(crate) const CH2_CFG_ADDR: u8 = 0x13;
pub(crate) const CH2_OCAL_MSB_ADDR: u8 = 0x14;
pub(crate) const CH2_OCAL_LSB_ADDR: u8 = 0x15;
pub(crate) const CH2_GCAL_MSB_ADDR: u8 = 0x16;
pub(crate) const CH2_GCAL_LSB_ADDR: u8 = 0x17;

// Channel 3
pub(crate) const CH3_CFG_ADDR: u8 = 0x18;
pub(crate) const CH3_OCAL_MSB_ADDR: u8 = 0x19;
pub(crate) const CH3_OCAL_LSB_ADDR: u8 = 0x1A;
pub(crate) const CH3_GCAL_MSB_ADDR: u8 = 0x1B;
pub(crate) const CH3_GCAL_LSB_ADDR: u8 = 0x1C;

// Channel 4
pub(crate) const CH4_CFG_ADDR: u8 = 0x1D;
pub(crate) const CH4_OCAL_MSB_ADDR: u8 = 0x1E;
pub(crate) const CH4_OCAL_LSB_ADDR: u8 = 0x1F;
pub(crate) const CH4_GCAL_MSB_ADDR: u8 = 0x20;
pub(crate) const CH4_GCAL_LSB_ADDR: u8 = 0x21;

// Channel 5
pub(crate) const CH5_CFG_ADDR: u8 = 0x22;
pub(crate) const CH5_OCAL_MSB_ADDR: u8 = 0x23;
pub(crate) const CH5_OCAL_LSB_ADDR: u8 = 0x24;
pub(crate) const CH5_GCAL_MSB_ADDR: u8 = 0x25;
pub(crate) const CH5_GCAL_LSB_ADDR: u8 = 0x26;

// Channel 6
pub(crate) const CH6_CFG_ADDR: u8 = 0x27;
pub(crate) const CH6_OCAL_MSB_ADDR: u8 = 0x28;
pub(crate) const CH6_OCAL_LSB_ADDR: u8 = 0x29;
pub(crate) const CH6_GCAL_MSB_ADDR: u8 = 0x2A;
pub(crate) const CH6_GCAL_LSB_ADDR: u8 = 0x2B;

// Channel 7
pub(crate) const CH7_CFG_ADDR: u8 = 0x2C;
pub(crate) const CH7_OCAL_MSB_ADDR: u8 = 0x2D;
pub(crate) const CH7_OCAL_LSB_ADDR: u8 = 0x2E;
pub(crate) const CH7_GCAL_MSB_ADDR: u8 = 0x2F;
pub(crate) const CH7_GCAL_LSB_ADDR: u8 = 0x30;

// Register map CRC
pub(crate) const REGMAP_CRC_ADDR: u8 = 0x3E;

pub(crate) const CHANNEL_CONFIG_ADDRS: &[u8] = &[
    CH0_CFG_ADDR,
    CH1_CFG_ADDR,
    CH2_CFG_ADDR,
    CH3_CFG_ADDR,
    CH4_CFG_ADDR,
    CH5_CFG_ADDR,
    CH6_CFG_ADDR,
    CH7_CFG_ADDR,
];

pub(crate) const CHANNEL_OCAL_MSB_ADDRS: &[u8] = &[
    CH0_OCAL_MSB_ADDR,
    CH1_OCAL_MSB_ADDR,
    CH2_OCAL_MSB_ADDR,
    CH3_OCAL_MSB_ADDR,
    CH4_OCAL_MSB_ADDR,
    CH5_OCAL_MSB_ADDR,
    CH6_OCAL_MSB_ADDR,
    CH7_OCAL_MSB_ADDR,
];

pub(crate) const CHANNEL_OCAL_LSB_ADDRS: &[u8] = &[
    CH0_OCAL_LSB_ADDR,
    CH1_OCAL_LSB_ADDR,
    CH2_OCAL_LSB_ADDR,
    CH3_OCAL_LSB_ADDR,
    CH4_OCAL_LSB_ADDR,
    CH5_OCAL_LSB_ADDR,
    CH6_OCAL_LSB_ADDR,
    CH7_OCAL_LSB_ADDR,
];

pub(crate) const CHANNEL_GCAL_MSB_ADDRS: &[u8] = &[
    CH0_GCAL_MSB_ADDR,
    CH1_GCAL_MSB_ADDR,
    CH2_GCAL_MSB_ADDR,
    CH3_GCAL_MSB_ADDR,
    CH4_GCAL_MSB_ADDR,
    CH5_GCAL_MSB_ADDR,
    CH6_GCAL_MSB_ADDR,
    CH7_GCAL_MSB_ADDR,
];

pub(crate) const CHANNEL_GCAL_LSB_ADDRS: &[u8] = &[
    CH0_GCAL_LSB_ADDR,
    CH1_GCAL_LSB_ADDR,
    CH2_GCAL_LSB_ADDR,
    CH3_GCAL_LSB_ADDR,
    CH4_GCAL_LSB_ADDR,
    CH5_GCAL_LSB_ADDR,
    CH6_GCAL_LSB_ADDR,
    CH7_GCAL_LSB_ADDR,
];

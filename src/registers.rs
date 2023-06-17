// Device settings
pub const ID_ADDR: u8 = 0x0;
pub const STATUS_ADDR: u8 = 0x1;

// Global across channels
pub const MODE_ADDR: u8 = 0x2;
pub const CLOCK_ADDR: u8 = 0x3;
pub const GAIN_ADDR: u8 = 0x4;
pub const CFG_ADDR: u8 = 0x6;
pub const THRSHLD_MSB_ADDR: u8 = 0x7;
pub const THRSHLD_LSB_ADDR: u8 = 0x8;

// Channel 0
const CH0_CFG_ADDR: u8 = 0x9;
const CH0_OCAL_MSB_ADDR: u8 = 0xA;
const CH0_OCAL_LSB_ADDR: u8 = 0xB;
const CH0_GCAL_MSB_ADDR: u8 = 0xC;
const CH0_GCAL_LSB_ADDR: u8 = 0xD;

// Channel 1
const CH1_CFG_ADDR: u8 = 0xE;
const CH1_OCAL_MSB_ADDR: u8 = 0xF;
const CH1_OCAL_LSB_ADDR: u8 = 0x10;
const CH1_GCAL_MSB_ADDR: u8 = 0x11;
const CH1_GCAL_LSB_ADDR: u8 = 0x12;

// Channel 2
const CH2_CFG_ADDR: u8 = 0x13;
const CH2_OCAL_MSB_ADDR: u8 = 0x14;
const CH2_OCAL_LSB_ADDR: u8 = 0x15;
const CH2_GCAL_MSB_ADDR: u8 = 0x16;
const CH2_GCAL_LSB_ADDR: u8 = 0x17;

// Channel 3
const CH3_CFG_ADDR: u8 = 0x18;
const CH3_OCAL_MSB_ADDR: u8 = 0x19;
const CH3_OCAL_LSB_ADDR: u8 = 0x1A;
const CH3_GCAL_MSB_ADDR: u8 = 0x1B;
const CH3_GCAL_LSB_ADDR: u8 = 0x1C;

// Channel 4
const CH4_CFG_ADDR: u8 = 0x1D;
const CH4_OCAL_MSB_ADDR: u8 = 0x1E;
const CH4_OCAL_LSB_ADDR: u8 = 0x1F;
const CH4_GCAL_MSB_ADDR: u8 = 0x20;
const CH4_GCAL_LSB_ADDR: u8 = 0x21;

// Channel 5
const CH5_CFG_ADDR: u8 = 0x22;
const CH5_OCAL_MSB_ADDR: u8 = 0x23;
const CH5_OCAL_LSB_ADDR: u8 = 0x24;
const CH5_GCAL_MSB_ADDR: u8 = 0x25;
const CH5_GCAL_LSB_ADDR: u8 = 0x26;

// Channel 6
const CH6_CFG_ADDR: u8 = 0x27;
const CH6_OCAL_MSB_ADDR: u8 = 0x28;
const CH6_OCAL_LSB_ADDR: u8 = 0x29;
const CH6_GCAL_MSB_ADDR: u8 = 0x2A;
const CH6_GCAL_LSB_ADDR: u8 = 0x2B;

// Channel 7
const CH7_CFG_ADDR: u8 = 0x2C;
const CH7_OCAL_MSB_ADDR: u8 = 0x2D;
const CH7_OCAL_LSB_ADDR: u8 = 0x2E;
const CH7_GCAL_MSB_ADDR: u8 = 0x2F;
const CH7_GCAL_LSB_ADDR: u8 = 0x30;

// Register map CRC
pub const REGMAP_CRC_ADDR: u8 = 0x3E;

pub const CHANNEL_CONFIG_ADDRS: &[u8] = &[
    CH0_CFG_ADDR,
    CH1_CFG_ADDR,
    CH2_CFG_ADDR,
    CH3_CFG_ADDR,
    CH4_CFG_ADDR,
    CH5_CFG_ADDR,
    CH6_CFG_ADDR,
    CH7_CFG_ADDR,
];

pub const CHANNEL_OCAL_MSB_ADDRS: &[u8] = &[
    CH0_OCAL_MSB_ADDR,
    CH1_OCAL_MSB_ADDR,
    CH2_OCAL_MSB_ADDR,
    CH3_OCAL_MSB_ADDR,
    CH4_OCAL_MSB_ADDR,
    CH5_OCAL_MSB_ADDR,
    CH6_OCAL_MSB_ADDR,
    CH7_OCAL_MSB_ADDR,
];

pub const CHANNEL_OCAL_LSB_ADDRS: &[u8] = &[
    CH0_OCAL_LSB_ADDR,
    CH1_OCAL_LSB_ADDR,
    CH2_OCAL_LSB_ADDR,
    CH3_OCAL_LSB_ADDR,
    CH4_OCAL_LSB_ADDR,
    CH5_OCAL_LSB_ADDR,
    CH6_OCAL_LSB_ADDR,
    CH7_OCAL_LSB_ADDR,
];

pub const CHANNEL_GCAL_MSB_ADDRS: &[u8] = &[
    CH0_GCAL_MSB_ADDR,
    CH1_GCAL_MSB_ADDR,
    CH2_GCAL_MSB_ADDR,
    CH3_GCAL_MSB_ADDR,
    CH4_GCAL_MSB_ADDR,
    CH5_GCAL_MSB_ADDR,
    CH6_GCAL_MSB_ADDR,
    CH7_GCAL_MSB_ADDR,
];

pub const CHANNEL_GCAL_LSB_ADDRS: &[u8] = &[
    CH0_GCAL_LSB_ADDR,
    CH1_GCAL_LSB_ADDR,
    CH2_GCAL_LSB_ADDR,
    CH3_GCAL_LSB_ADDR,
    CH4_GCAL_LSB_ADDR,
    CH5_GCAL_LSB_ADDR,
    CH6_GCAL_LSB_ADDR,
    CH7_GCAL_LSB_ADDR,
];

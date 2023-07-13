//! Types for configuring an ADS131M registers

use enum_iterator::{self, Sequence};
use num_enum::{IntoPrimitive, TryFromPrimitive};
use ux::{u10, u24, u4};

macro_rules! is_bit_set {
    ($word:expr, $bit:literal) => {
        ($word & (1 << $bit)) != 0
    };
}

/// An ADC channel
#[derive(Debug, PartialEq, Eq, Clone, Copy, IntoPrimitive)]
#[repr(u8)]
pub enum Channel {
    /// ADC channel 0
    ///
    /// This channel is available on all ADS131M models
    Zero,
    /// ADC channel 1
    ///
    /// This channel is available on all ADS131M models
    One,
    /// ADC channel 2
    ///
    /// This channel is available on the following ADS131M models:
    /// - ADS131M03
    /// - ADS131M04
    /// - ADS131M06
    /// - ADS131M08
    Two,
    /// ADC channel 3
    ///
    /// This channel is available on the following ADS131M models:
    /// - ADS131M04
    /// - ADS131M06
    /// - ADS131M08
    Three,
    /// ADC channel 4
    ///
    /// This channel is available on the following ADS131M models:
    /// - ADS131M06
    /// - ADS131M08
    Four,
    /// ADC channel 5
    ///
    /// This channel is available on the following ADS131M models:
    /// - ADS131M06
    /// - ADS131M08
    Five,
    /// ADC channel 6
    ///
    /// This channel is available on the following ADS131M models:
    /// - ADS131M08
    Six,
    /// ADC channel 7
    ///
    /// This channel is available on the following ADS131M models:
    /// - ADS131M08
    Seven,
}

/// An address for an ADC register
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Address {
    /// The device `ID` register
    Id,
    /// The device `STATUS` register
    Status,
    /// The device `MODE` register
    Mode,
    /// The device `CLOCK` register
    Clock,
    /// The device `GAIN` register
    Gain,
    /// The device `CFG` register
    Config,
    /// The device `THRSHLD_MSB` register
    ThresholdMsb,
    /// The device `THRSHLD_LSB` register
    ThresholdLsb,
    /// The device `CH*_CFG` register
    ChannelConfig(Channel),
    /// The device `CH*_OCAL_MSB` register
    ChannelOffsetCalMsb(Channel),
    /// The device `CH*_OCAL_LSB` register
    ChannelOffsetCalLsb(Channel),
    /// The device `CH*_GCAL_MSB` register
    ChannelGainCalMsb(Channel),
    /// The device `CH*_GCAL_LSB` register
    ChannelGainCalLsb(Channel),
    /// The device `REGMAP_CRC` register
    RegisterMapCrc,
}

impl Address {
    /// Get the address value for this `RegisterAddress`, if the channel is valid
    pub(crate) const fn address(self) -> u8 {
        match self {
            Self::Id => 0x0,
            Self::Status => 0x1,
            Self::Mode => 0x2,
            Self::Clock => 0x3,
            Self::Gain => 0x4,
            Self::Config => 0x6,
            Self::ThresholdMsb => 0x7,
            Self::ThresholdLsb => 0x8,
            Self::ChannelConfig(Channel::Zero) => 0x9,
            Self::ChannelOffsetCalMsb(Channel::Zero) => 0xA,
            Self::ChannelOffsetCalLsb(Channel::Zero) => 0xB,
            Self::ChannelGainCalMsb(Channel::Zero) => 0xC,
            Self::ChannelGainCalLsb(Channel::Zero) => 0xD,
            Self::ChannelConfig(Channel::One) => 0xE,
            Self::ChannelOffsetCalMsb(Channel::One) => 0xF,
            Self::ChannelOffsetCalLsb(Channel::One) => 0x10,
            Self::ChannelGainCalMsb(Channel::One) => 0x11,
            Self::ChannelGainCalLsb(Channel::One) => 0x12,
            Self::ChannelConfig(Channel::Two) => 0x13,
            Self::ChannelOffsetCalMsb(Channel::Two) => 0x14,
            Self::ChannelOffsetCalLsb(Channel::Two) => 0x15,
            Self::ChannelGainCalMsb(Channel::Two) => 0x16,
            Self::ChannelGainCalLsb(Channel::Two) => 0x17,
            Self::ChannelConfig(Channel::Three) => 0x18,
            Self::ChannelOffsetCalMsb(Channel::Three) => 0x19,
            Self::ChannelOffsetCalLsb(Channel::Three) => 0x1A,
            Self::ChannelGainCalMsb(Channel::Three) => 0x1B,
            Self::ChannelGainCalLsb(Channel::Three) => 0x1C,
            Self::ChannelConfig(Channel::Four) => 0x1D,
            Self::ChannelOffsetCalMsb(Channel::Four) => 0x1E,
            Self::ChannelOffsetCalLsb(Channel::Four) => 0x1F,
            Self::ChannelGainCalMsb(Channel::Four) => 0x20,
            Self::ChannelGainCalLsb(Channel::Four) => 0x21,
            Self::ChannelConfig(Channel::Five) => 0x22,
            Self::ChannelOffsetCalMsb(Channel::Five) => 0x23,
            Self::ChannelOffsetCalLsb(Channel::Five) => 0x24,
            Self::ChannelGainCalMsb(Channel::Five) => 0x25,
            Self::ChannelGainCalLsb(Channel::Five) => 0x26,
            Self::ChannelConfig(Channel::Six) => 0x27,
            Self::ChannelOffsetCalMsb(Channel::Six) => 0x28,
            Self::ChannelOffsetCalLsb(Channel::Six) => 0x29,
            Self::ChannelGainCalMsb(Channel::Six) => 0x2A,
            Self::ChannelGainCalLsb(Channel::Six) => 0x2B,
            Self::ChannelConfig(Channel::Seven) => 0x2C,
            Self::ChannelOffsetCalMsb(Channel::Seven) => 0x2D,
            Self::ChannelOffsetCalLsb(Channel::Seven) => 0x2E,
            Self::ChannelGainCalMsb(Channel::Seven) => 0x2F,
            Self::ChannelGainCalLsb(Channel::Seven) => 0x30,
            Self::RegisterMapCrc => 0x3E,
        }
    }

    /// Get the channel for the register, if applicable
    pub(crate) const fn channel(self) -> Option<Channel> {
        match self {
            Self::ChannelConfig(n)
            | Self::ChannelOffsetCalMsb(n)
            | Self::ChannelOffsetCalLsb(n)
            | Self::ChannelGainCalMsb(n)
            | Self::ChannelGainCalLsb(n) => Some(n),
            _ => None,
        }
    }
}

/// A global device register
pub trait Global
where
    Self: Clone + Copy + PartialEq + Eq,
{
    /// The memory address of the register
    const ADDRESS: Address;

    /// Unpack a register value from it's byte representation
    ///
    /// Words must be MSB first
    #[must_use]
    fn from_be_bytes(bytes: [u8; 2]) -> Self;

    /// Pack a register value to it's byte representation
    ///
    /// Words are MSB first
    #[must_use]
    fn to_be_bytes(self) -> [u8; 2];
}

/// A channel-specific device register
pub trait ChannelSpecific
where
    Self: Clone + Copy + PartialEq + Eq,
{
    /// The memory address of the register for a given chanel
    #[must_use]
    fn address_for_channel(channel: Channel) -> Address;

    /// Unpack a register value from it's byte representation
    ///
    /// Words must be MSB first
    #[must_use]
    fn from_be_bytes(bytes: [u8; 2]) -> Self;

    /// Pack a register value to it's byte representation
    ///
    /// Words are MSB first
    #[must_use]
    fn to_be_bytes(self) -> [u8; 2];
}

/// SPI Word size configuration
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum WordLength {
    /// 16-bit words
    Bits16 = 0,

    /// 24-bit words
    ///
    /// This is the default word length
    #[default]
    Bits24 = 1,

    /// 32-bit words, with zero padding
    Bits32Zero = 2,

    /// 32-bit words with the sign extension for 24-bit ADC data
    Bits32Signed = 3,
}

impl WordLength {
    pub(crate) const fn byte_count(self) -> usize {
        match self {
            Self::Bits16 => 2,
            Self::Bits24 => 3,
            _ => 4,
        }
    }
}

/// CRC implementation used for device communication
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum CrcType {
    /// 16 bit CCITT.
    ///
    /// This is the default CRC mode
    #[default]
    Ccitt = 0,

    /// 16 bit ANSI
    Ansi = 1,
}

/// DRDY pin source selection
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum DrdySource {
    /// Most lagging enabled channel
    ///
    /// This is the default DRDY source
    #[default]
    MostLagging = 0,

    /// Logic OR of all the enabled channels
    LogicOr = 1,

    /// Most leading enabled channel
    #[num_enum(alternatives = [3])]
    MostLeading = 2,
}

/// DRDY state when conversion data is not available
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum DrdyNotReadyState {
    /// Logic high
    ///
    /// This is the default DRDY not ready state
    #[default]
    LogicHigh = 0,

    /// High impedance
    HighImpedance = 1,
}

/// DRDY state when conversion data is available
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum DrdyReadyState {
    /// Logic low
    ///
    /// This is the default DRDY not state
    #[default]
    LogicLow = 0,

    /// Low pulse with a fixed duration
    LowPulse = 1,
}

/// Oversampling mode configuration
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum OversamplingRatio {
    /// Oversampling ratio of 64
    #[num_enum(alternatives = [9..16])]
    Osr64 = 8,

    /// Oversampling ratio of 128
    Osr128 = 0,

    /// Oversampling ratio of 256
    Osr256 = 1,

    /// Oversampling ratio of 512
    Osr512 = 2,

    /// Oversampling ratio of 1024
    ///
    /// This is the default oversampling ratio
    #[default]
    Osr1024 = 3,

    /// Oversampling ratio of 2048
    Osr2048 = 4,

    /// Oversampling ratio of 4096
    Osr4096 = 5,

    /// Oversampling ratio of 8192
    Osr8192 = 6,

    /// Oversampling ratio of 16256
    Osr16256 = 7,
}

/// Power mode setting
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum PowerMode {
    /// Very low power mode
    VeryLowPower = 0,

    /// Low power mode
    LowPower = 1,

    /// High resolution mode
    ///
    /// This is the default power mode
    #[default]
    #[num_enum(alternatives = [3])]
    HighResolution = 2,
}

/// PGA gain setting
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum PgaGain {
    /// 1x gain
    ///
    /// This is the default gain setting
    #[default]
    Gain1 = 0,

    /// 2x gain
    Gain2 = 1,

    /// 4x gain
    Gain4 = 2,

    /// 8x gain
    Gain8 = 3,

    /// 16x gain
    Gain16 = 4,

    /// 32x gain
    Gain32 = 5,

    /// 64x gain
    Gain64 = 6,

    /// 128x gain
    Gain128 = 7,
}

/// Global chop delay selection
///
/// Delay in modulator clock periods before measurement begins
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum GlobalChopDelay {
    /// 2 modulator clock delay
    Delay2 = 0,

    /// 4 modulator clock delay
    Delay4 = 1,

    /// 8 modulator clock delay
    Delay8 = 2,

    /// 16 modulator clock delay
    ///
    /// This is the default global-chop delay
    #[default]
    Delay16 = 3,

    /// 32 modulator clock delay
    Delay32 = 4,

    /// 64 modulator clock delay
    Delay64 = 5,

    /// 128 modulator clock delay
    Delay128 = 6,

    /// 256 modulator clock delay
    Delay256 = 7,

    /// 512 modulator clock delay
    Delay512 = 8,

    /// 1024 modulator clock delay
    Delay1024 = 9,

    /// 2048 modulator clock delay
    Delay2048 = 10,

    /// 4096 modulator clock delay
    Delay4096 = 11,

    /// 8192 modulator clock delay
    Delay8192 = 12,

    /// 16384 modulator clock delay
    Delay16384 = 13,

    /// 32768 modulator clock delay
    Delay32768 = 14,

    /// 65536 modulator clock delay
    Delay65536 = 15,
}

/// Current-detect channel selection
/// Channels required to trigger current-detect
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum CurrentDetectChannels {
    /// Any channel
    ///
    /// This is the default channel selection
    #[default]
    AnyChannel = 0,

    /// All channels
    AllChannels = 1,
}

/// Number of current-detect exceeded thresholds to trigger a detection
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum CurrentDetectCount {
    /// 1 detection
    ///
    /// This is the default current-detect length
    #[default]
    Count1 = 0,

    /// 2 detections
    Count2 = 1,

    /// 4 detections
    Count4 = 2,

    /// 8 detections
    Count8 = 3,

    /// 16 detections
    Count16 = 4,

    /// 32 detections
    Count32 = 5,

    /// 64 detections
    Count64 = 6,

    /// 128 detections
    Count128 = 7,
}

/// Current-detect measurement length in conversion periods
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum CurrentDetectLength {
    /// 128 conversion periods
    ///
    /// This is the default current-detect length
    #[default]
    Len128 = 0,

    /// 256 conversion periods
    Len256 = 1,

    /// 512 conversion periods
    Len512 = 2,

    /// 768 conversion periods
    Len768 = 3,

    /// 1280 conversion periods
    Len1280 = 4,

    /// 1792 conversion periods
    Len1792 = 5,

    /// 2560 conversion periods
    Len2560 = 6,

    /// 3584 conversion periods
    Len3584 = 7,
}

/// DC block filter setting
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum DcBlock {
    /// DC block filter disabled
    #[default]
    Disabled = 0,

    /// 1/4
    OneOver4 = 1,

    /// 1/8
    OneOver8 = 2,

    /// 1/16
    OneOver16 = 3,

    /// 1/32
    OneOver32 = 4,

    /// 1/64
    OneOver64 = 5,

    /// 1/128
    OneOver128 = 6,

    /// 1/256
    OneOver256 = 7,

    /// 1/512
    OneOver512 = 8,

    /// 1/1024
    OneOver1024 = 9,

    /// 1/2048
    OneOver2048 = 10,

    /// 1/4096
    OneOver4096 = 11,

    /// 1/8192
    OneOver8192 = 12,

    /// 1/16384
    OneOver16384 = 13,

    /// 1/32768
    OneOver32768 = 14,

    /// 1/65536
    OneOver65536 = 15,
}

/// Channel input selection
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive, Sequence)]
#[repr(u8)]
pub enum ChannelMux {
    /// AINxP and AINxN
    ///
    /// This is the default channel mux setting
    #[default]
    AnalogIn = 0,

    /// ADC inputs shorted
    Shorted = 1,

    /// Positive DC test signal
    PositiveTest = 2,

    /// Negative DC test signal
    NegativeTest = 3,
}

/// Device `ID` register
///
/// This register is read only. Writing to it has no effect
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Id {
    /// Channel count
    pub channel_count: u4,
}

impl Global for Id {
    const ADDRESS: Address = Address::Id;

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self {
            channel_count: u4::new(bytes[0] & 0b1111),
        }
    }

    fn to_be_bytes(self) -> [u8; 2] {
        [u8::from(self.channel_count), 0]
    }
}

/// Device `STATUS` register
///
/// This register is read only. Writing to it has no effect
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)]
pub struct Status {
    /// Whether the SPI interface is locked
    pub lock: bool,

    /// ADC resynchronization indicator.
    /// This bit is set each time the ADC resynchronizes.
    pub resync: bool,

    /// Whether the register map had a CRC error
    pub reg_map_crc_err: bool,

    /// Whether a SPI input had a CRC error
    pub spi_crc_err: bool,

    /// CRC type
    pub crc_type: CrcType,

    /// Whether a reset occurred
    pub reset: bool,

    /// Data word length
    pub word_length: WordLength,

    /// Channel 0 ADC data available indicator
    pub drdy0: bool,

    /// Channel 1 ADC data available indicator
    pub drdy1: bool,

    /// Channel 2 ADC data available indicator
    pub drdy2: bool,

    /// Channel 3 ADC data available indicator
    pub drdy3: bool,
}

impl Global for Status {
    const ADDRESS: Address = Address::Status;

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self {
            lock: is_bit_set!(bytes[0], 7),
            resync: is_bit_set!(bytes[0], 6),
            reg_map_crc_err: is_bit_set!(bytes[0], 5),
            spi_crc_err: is_bit_set!(bytes[0], 4),
            crc_type: CrcType::try_from((bytes[0] >> 3) & 0b1).unwrap(),
            reset: is_bit_set!(bytes[0], 2),
            word_length: WordLength::try_from(bytes[0] & 0b11).unwrap(),
            drdy3: is_bit_set!(bytes[1], 3),
            drdy2: is_bit_set!(bytes[1], 2),
            drdy1: is_bit_set!(bytes[1], 1),
            drdy0: is_bit_set!(bytes[1], 0),
        }
    }

    fn to_be_bytes(self) -> [u8; 2] {
        [
            u8::from(self.lock) << 7
                | u8::from(self.resync) << 6
                | u8::from(self.reg_map_crc_err) << 5
                | u8::from(self.spi_crc_err) << 4
                | u8::from(self.crc_type) << 3
                | u8::from(self.reset) << 2
                | u8::from(self.word_length),
            u8::from(self.drdy3) << 3
                | u8::from(self.drdy2) << 2
                | u8::from(self.drdy1) << 1
                | u8::from(self.drdy0),
        ]
    }
}

impl Default for Status {
    fn default() -> Self {
        Self {
            lock: false,
            resync: false,
            reg_map_crc_err: false,
            spi_crc_err: false,
            crc_type: CrcType::Ccitt,
            reset: true,
            word_length: WordLength::Bits24,
            drdy0: false,
            drdy1: false,
            drdy2: false,
            drdy3: false,
        }
    }
}

/// Device `MODE` register
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)]
pub struct Mode {
    /// Whether register map CRC checking is enabled
    pub reg_crc_enable: bool,

    /// Whether SPI CRC checking is enabled
    pub spi_crc_enable: bool,

    /// SPI and register map CRC type
    pub crc_type: CrcType,

    /// Reset
    /// Write false to clear this bit in the status register
    pub reset: bool,

    /// Data word length
    pub word_length: WordLength,

    /// SPI timeout enable
    pub spi_timeout_enable: bool,

    /// DRDY pin signal source selection
    pub drdy_source: DrdySource,

    /// DRDY state when conversion data is not available
    pub drdy_not_ready_state: DrdyNotReadyState,

    /// DRDY state when conversion data is available
    pub drdy_ready_state: DrdyReadyState,
}

impl Global for Mode {
    const ADDRESS: Address = Address::Mode;

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self {
            reg_crc_enable: is_bit_set!(bytes[0], 5),
            spi_crc_enable: is_bit_set!(bytes[0], 4),
            crc_type: CrcType::try_from((bytes[0] >> 3) & 0b1).unwrap(),
            reset: is_bit_set!(bytes[0], 2),
            word_length: WordLength::try_from(bytes[0] & 0b11).unwrap(),
            spi_timeout_enable: is_bit_set!(bytes[1], 4),
            drdy_source: DrdySource::try_from((bytes[1] >> 2) & 0b11).unwrap(),
            drdy_not_ready_state: DrdyNotReadyState::try_from((bytes[1] >> 1) & 0b1).unwrap(),
            drdy_ready_state: DrdyReadyState::try_from(bytes[1] & 0b1).unwrap(),
        }
    }

    fn to_be_bytes(self) -> [u8; 2] {
        [
            u8::from(self.reg_crc_enable) << 5
                | u8::from(self.spi_crc_enable) << 4
                | u8::from(self.crc_type) << 3
                | u8::from(self.reset) << 2
                | u8::from(self.word_length),
            u8::from(self.spi_timeout_enable) << 4
                | u8::from(self.drdy_source) << 2
                | u8::from(self.drdy_not_ready_state) << 1
                | u8::from(self.drdy_ready_state),
        ]
    }
}

impl Default for Mode {
    fn default() -> Self {
        Self {
            reg_crc_enable: false,
            spi_crc_enable: false,
            crc_type: CrcType::default(),
            reset: true,
            word_length: WordLength::default(),
            spi_timeout_enable: true,
            drdy_source: DrdySource::default(),
            drdy_not_ready_state: DrdyNotReadyState::default(),
            drdy_ready_state: DrdyReadyState::default(),
        }
    }
}

/// Device `CLOCK` register
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)]
pub struct Clock {
    /// Channel 0 ADC enable
    pub channel0_en: bool,

    /// Channel 1 ADC enable
    pub channel1_en: bool,

    /// Channel 2 ADC enable
    pub channel2_en: bool,

    /// Channel 3 ADC enable
    pub channel3_en: bool,

    /// Modulator oversampling ratio
    pub oversampling_ratio: OversamplingRatio,

    /// Power mode
    pub power_mode: PowerMode,
}

impl Global for Clock {
    const ADDRESS: Address = Address::Clock;

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self {
            channel0_en: is_bit_set!(bytes[0], 0),
            channel1_en: is_bit_set!(bytes[0], 1),
            channel2_en: is_bit_set!(bytes[0], 2),
            channel3_en: is_bit_set!(bytes[0], 3),
            oversampling_ratio: OversamplingRatio::try_from((bytes[1] >> 2) & 0b1111).unwrap(),
            power_mode: PowerMode::try_from(bytes[1] & 0b11).unwrap(),
        }
    }

    fn to_be_bytes(self) -> [u8; 2] {
        [
            u8::from(self.channel3_en) << 3
                | u8::from(self.channel2_en) << 2
                | u8::from(self.channel1_en) << 1
                | u8::from(self.channel0_en),
            u8::from(self.oversampling_ratio) << 2 | u8::from(self.power_mode),
        ]
    }
}

impl Default for Clock {
    fn default() -> Self {
        Self {
            channel0_en: true,
            channel1_en: true,
            channel2_en: true,
            channel3_en: true,
            oversampling_ratio: OversamplingRatio::default(),
            power_mode: PowerMode::default(),
        }
    }
}

/// Device `GAIN1` register
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub struct Gain {
    /// PGA gain selection for channel 0
    pub pga_gain0: PgaGain,

    /// PGA gain selection for channel 1
    pub pga_gain1: PgaGain,

    /// PGA gain selection for channel 2
    pub pga_gain2: PgaGain,

    /// PGA gain selection for channel 3
    pub pga_gain3: PgaGain,
}

impl Global for Gain {
    const ADDRESS: Address = Address::Gain;

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self {
            pga_gain2: PgaGain::try_from(bytes[0] & 0b111).unwrap(),
            pga_gain3: PgaGain::try_from((bytes[0] >> 4) & 0b111).unwrap(),
            pga_gain0: PgaGain::try_from(bytes[1] & 0b111).unwrap(),
            pga_gain1: PgaGain::try_from((bytes[1] >> 4) & 0b111).unwrap(),
        }
    }

    fn to_be_bytes(self) -> [u8; 2] {
        [
            u8::from(self.pga_gain2) | u8::from(self.pga_gain3) << 4,
            u8::from(self.pga_gain0) | u8::from(self.pga_gain1) << 4,
        ]
    }
}

/// Device `CFG` register
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Global-chop delay
    pub global_chop_delay: GlobalChopDelay,

    /// Global-chop mode enable
    pub global_chop_enable: bool,

    /// Current-detect channel selection
    /// Channels required to trigger current-detect
    pub current_detect_channels: CurrentDetectChannels,

    /// Number of current-detect exceeded thresholds to trigger a detection
    pub current_detect_count: CurrentDetectCount,

    /// Current-detect measurement length
    pub current_detect_length: CurrentDetectLength,

    /// Current-detect mode enable
    pub current_detect_enable: bool,
}

impl Global for Config {
    const ADDRESS: Address = Address::Config;

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self {
            global_chop_delay: GlobalChopDelay::try_from((bytes[0] >> 1) & 0b1111).unwrap(),
            global_chop_enable: is_bit_set!(bytes[0], 0),
            current_detect_channels: CurrentDetectChannels::try_from((bytes[1] >> 7) & 0b1)
                .unwrap(),
            current_detect_count: CurrentDetectCount::try_from((bytes[1] >> 4) & 0b111).unwrap(),
            current_detect_length: CurrentDetectLength::try_from((bytes[1] >> 1) & 0b111).unwrap(),
            current_detect_enable: is_bit_set!(bytes[1], 0),
        }
    }

    fn to_be_bytes(self) -> [u8; 2] {
        [
            u8::from(self.global_chop_delay) << 1 | u8::from(self.global_chop_enable),
            u8::from(self.current_detect_channels) << 7
                | u8::from(self.current_detect_count) << 4
                | u8::from(self.current_detect_length) << 1
                | u8::from(self.current_detect_enable),
        ]
    }
}

/// Device `THRSHLD_MSB` and `THRSHLD_LSB` registers
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub struct Threshold {
    /// Current-detect mode threshold
    pub current_detect_threshold: u24,

    /// DC block filter setting
    pub dc_block: DcBlock,
}

impl Threshold {
    /// Split this into the two [`Global`] halves that make up this `Threshold`
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn into_parts(self) -> (ThresholdMsb, ThresholdLsb) {
        let threshold = u32::from(self.current_detect_threshold);

        (
            ThresholdMsb {
                bytes: [(threshold >> 16) as u8, (threshold >> 8) as u8],
            },
            ThresholdLsb {
                bytes: [threshold as u8, u8::from(self.dc_block)],
            },
        )
    }

    /// Build a `Threshold` from it's two [`Global`] halves
    #[must_use]
    #[allow(
        clippy::missing_panics_doc,
        clippy::needless_pass_by_value,
        clippy::similar_names
    )]
    pub fn from_parts(msb: ThresholdMsb, lsb: ThresholdLsb) -> Self {
        Self {
            current_detect_threshold: u24::from(msb.bytes[0]) << 16
                | u24::from(msb.bytes[1]) << 8
                | u24::from(lsb.bytes[0]),
            dc_block: DcBlock::try_from(lsb.bytes[1] & 0b1111).unwrap(),
        }
    }
}

/// Device `THRSHLD_MSB` register
///
/// This is only one half of a [`Threshold`], use [`Threshold::from_parts`] to combine the two halves
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct ThresholdMsb {
    bytes: [u8; 2],
}

impl Global for ThresholdMsb {
    const ADDRESS: Address = Address::ThresholdMsb;

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self { bytes }
    }

    fn to_be_bytes(self) -> [u8; 2] {
        self.bytes
    }
}

/// Device `THRSHLD_LSB` register
///
/// This is only one half of a [`Threshold`], use [`Threshold::from_parts`] to combine the two halves
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct ThresholdLsb {
    bytes: [u8; 2],
}

impl Global for ThresholdLsb {
    const ADDRESS: Address = Address::ThresholdLsb;

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self { bytes }
    }

    fn to_be_bytes(self) -> [u8; 2] {
        self.bytes
    }
}

/// Device `CHx_CFG` register
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelConfig {
    /// Channel 0 phase delay in modulator clock cycles
    pub phase: u10,

    /// DC block filter for channel 0 disable
    pub dc_block_disable: bool,

    /// Channel 0 input selection
    pub mux: ChannelMux,
}

impl ChannelSpecific for ChannelConfig {
    fn address_for_channel(channel: Channel) -> Address {
        Address::ChannelConfig(channel)
    }

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self {
            phase: u10::from(bytes[0]) << 2 | u10::from(bytes[1] >> 6),
            dc_block_disable: is_bit_set!(bytes[1], 2),
            mux: ChannelMux::try_from(bytes[1] & 0b11).unwrap(),
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn to_be_bytes(self) -> [u8; 2] {
        let phase = u16::from(self.phase);
        [
            (phase >> 2) as u8,
            (phase << 6) as u8 | u8::from(self.dc_block_disable) << 2 | u8::from(self.mux),
        ]
    }
}

/// Device `CHx_OCAL_MSB` and `CHx_OCAL_LSB` registers
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelOffsetCal {
    /// Channel offset calibration
    pub offset: u24,
}

impl ChannelOffsetCal {
    /// Split this into the two [`ChannelSpecific`] halves that make up this `ChannelOffsetCal`
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn into_parts(self) -> (ChannelOffsetCalMsb, ChannelOffsetCalLsb) {
        let offset = u32::from(self.offset);

        (
            ChannelOffsetCalMsb {
                bytes: [(offset >> 16) as u8, (offset >> 8) as u8],
            },
            ChannelOffsetCalLsb {
                bytes: [offset as u8, 0],
            },
        )
    }

    /// Build a `ChannelOffsetCal` from it's two [`ChannelSpecific`] halves
    #[must_use]
    #[allow(
        clippy::missing_panics_doc,
        clippy::needless_pass_by_value,
        clippy::similar_names
    )]
    pub fn from_parts(msb: ChannelOffsetCalMsb, lsb: ChannelOffsetCalLsb) -> Self {
        Self {
            offset: u24::from(msb.bytes[0]) << 16
                | u24::from(msb.bytes[1]) << 8
                | u24::from(lsb.bytes[0]),
        }
    }
}

/// Device `CHx_OCAL_MSB` register
///
/// This is only one half of a [`ChannelOffsetCal`], use [`ChannelOffsetCal::from_parts`] to combine the two halves
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct ChannelOffsetCalMsb {
    bytes: [u8; 2],
}

impl ChannelSpecific for ChannelOffsetCalMsb {
    fn address_for_channel(channel: Channel) -> Address {
        Address::ChannelOffsetCalMsb(channel)
    }

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self { bytes }
    }

    fn to_be_bytes(self) -> [u8; 2] {
        self.bytes
    }
}

/// Device `CHx_OCAL_LSB` register
///
/// This is only one half of a [`ChannelOffsetCal`], use [`ChannelOffsetCal::from_parts`] to combine the two halves
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct ChannelOffsetCalLsb {
    bytes: [u8; 2],
}

impl ChannelSpecific for ChannelOffsetCalLsb {
    fn address_for_channel(channel: Channel) -> Address {
        Address::ChannelOffsetCalLsb(channel)
    }

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self { bytes }
    }

    fn to_be_bytes(self) -> [u8; 2] {
        self.bytes
    }
}

/// Device `CHx_GCAL_MSB` and `CHx_GCAL_LSB` registers
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelGainCal {
    /// Channel gain calibration
    pub gain: u24,
}

impl ChannelGainCal {
    /// Split this into the two [`ChannelSpecific`] halves that make up this `ChannelGainCal`
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn into_parts(self) -> (ChannelGainCalMsb, ChannelGainCalLsb) {
        let gain = u32::from(self.gain);

        (
            ChannelGainCalMsb {
                bytes: [(gain >> 16) as u8, (gain >> 8) as u8],
            },
            ChannelGainCalLsb {
                bytes: [gain as u8, 0],
            },
        )
    }

    /// Build a `ChannelGainCal` from it's two [`ChannelSpecific`] halves
    #[must_use]
    #[allow(
        clippy::missing_panics_doc,
        clippy::needless_pass_by_value,
        clippy::similar_names
    )]
    pub fn from_parts(msb: ChannelGainCalMsb, lsb: ChannelGainCalLsb) -> Self {
        Self {
            gain: u24::from(msb.bytes[0]) << 16
                | u24::from(msb.bytes[1]) << 8
                | u24::from(lsb.bytes[0]),
        }
    }
}

impl Default for ChannelGainCal {
    fn default() -> Self {
        Self::from_parts(ChannelGainCalMsb::default(), ChannelGainCalLsb::default())
    }
}

/// Device `CHx_GCAL_MSB` register
///
/// This is only one half of a [`ChannelGainCal`], use [`ChannelGainCal::from_parts`] to combine the two halves
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelGainCalMsb {
    bytes: [u8; 2],
}

impl ChannelSpecific for ChannelGainCalMsb {
    fn address_for_channel(channel: Channel) -> Address {
        Address::ChannelGainCalMsb(channel)
    }

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self { bytes }
    }

    fn to_be_bytes(self) -> [u8; 2] {
        self.bytes
    }
}

impl Default for ChannelGainCalMsb {
    fn default() -> Self {
        Self {
            bytes: [0x80, 0x00],
        }
    }
}

/// Device `CHx_GCAL_LSB` register
///
/// This is only one half of a [`ChannelGainCal`], use [`ChannelGainCal::from_parts`] to combine the two halves
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct ChannelGainCalLsb {
    bytes: [u8; 2],
}

impl ChannelSpecific for ChannelGainCalLsb {
    fn address_for_channel(channel: Channel) -> Address {
        Address::ChannelGainCalLsb(channel)
    }

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self { bytes }
    }

    fn to_be_bytes(self) -> [u8; 2] {
        self.bytes
    }
}

/// Device `REGMAP_CRC` register
///
/// This register is read only. Writing to it has no effect
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct RegistryMapCrc {
    /// Register map CRC
    pub crc: u16,
}

impl Global for RegistryMapCrc {
    const ADDRESS: Address = Address::RegisterMapCrc;

    fn from_be_bytes(bytes: [u8; 2]) -> Self {
        Self {
            crc: u16::from_be_bytes(bytes),
        }
    }

    fn to_be_bytes(self) -> [u8; 2] {
        self.crc.to_be_bytes()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn id_decode() {
        for (word, count) in [
            ([0x22, 0x00], 2),
            ([0x23, 0x00], 3),
            ([0x24, 0x00], 4),
            ([0x26, 0x00], 6),
            ([0x28, 0x00], 8),
        ] {
            assert_eq!(
                Id::from_be_bytes(word),
                Id {
                    channel_count: u4::new(count)
                }
            );
        }
    }

    #[test]
    fn status_default() {
        assert_eq!(Status::from_be_bytes([0x05, 0x00]), Status::default());
    }

    #[test]
    fn mode_default() {
        assert_eq!(Mode::default().to_be_bytes(), [0x05, 0x10]);
        assert_eq!(Mode::from_be_bytes([0x05, 0x10]), Mode::default());
    }

    #[test]
    fn mode_round_trip() {
        for bools in [false, true] {
            for crc_type in enum_iterator::all::<CrcType>() {
                for word_length in enum_iterator::all::<WordLength>() {
                    for drdy_source in enum_iterator::all::<DrdySource>() {
                        for drdy_not_ready_state in enum_iterator::all::<DrdyNotReadyState>() {
                            for drdy_ready_state in enum_iterator::all::<DrdyReadyState>() {
                                let mode = Mode {
                                    reg_crc_enable: bools,
                                    spi_crc_enable: bools,
                                    crc_type,
                                    reset: bools,
                                    word_length,
                                    spi_timeout_enable: bools,
                                    drdy_source,
                                    drdy_not_ready_state,
                                    drdy_ready_state,
                                };

                                assert_eq!(mode, Mode::from_be_bytes(mode.to_be_bytes()));
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn clock_default() {
        assert_eq!(Clock::default().to_be_bytes(), [0x0F, 0x0E]);
        assert_eq!(Clock::from_be_bytes([0x0F, 0x0E]), Clock::default());
    }

    #[test]
    fn clock_round_trip() {
        for bools in [false, true] {
            for oversampling_ratio in enum_iterator::all::<OversamplingRatio>() {
                for power_mode in enum_iterator::all::<PowerMode>() {
                    let clock = Clock {
                        channel0_en: bools,
                        channel1_en: bools,
                        channel2_en: bools,
                        channel3_en: bools,
                        oversampling_ratio,
                        power_mode,
                    };

                    assert_eq!(clock, Clock::from_be_bytes(clock.to_be_bytes()));
                }
            }
        }
    }

    #[test]
    fn gain_default() {
        assert_eq!(Gain::default().to_be_bytes(), [0x00, 0x00]);
        assert_eq!(Gain::from_be_bytes([0x00, 0x00]), Gain::default());
    }

    #[test]
    fn gain_round_trip() {
        for gains in enum_iterator::all::<PgaGain>() {
            let gain = Gain {
                pga_gain0: gains,
                pga_gain1: gains,
                pga_gain2: gains,
                pga_gain3: gains,
            };

            assert_eq!(gain, Gain::from_be_bytes(gain.to_be_bytes()));
        }
    }

    #[test]
    fn config_default() {
        assert_eq!(Config::default().to_be_bytes(), [0x06, 0x00]);
        assert_eq!(Config::from_be_bytes([0x06, 0x00]), Config::default());
    }

    #[test]
    fn config_round_trip() {
        for bools in [false, true] {
            for global_chop_delay in enum_iterator::all::<GlobalChopDelay>() {
                for current_detect_channels in enum_iterator::all::<CurrentDetectChannels>() {
                    for current_detect_count in enum_iterator::all::<CurrentDetectCount>() {
                        for current_detect_length in enum_iterator::all::<CurrentDetectLength>() {
                            let config = Config {
                                global_chop_delay,
                                global_chop_enable: bools,
                                current_detect_channels,
                                current_detect_count,
                                current_detect_length,
                                current_detect_enable: bools,
                            };

                            assert_eq!(config, Config::from_be_bytes(config.to_be_bytes()));
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn threshold_default() {
        assert_eq!(
            Threshold::default().into_parts(),
            (
                ThresholdMsb {
                    bytes: [0x00, 0x00]
                },
                ThresholdLsb {
                    bytes: [0x00, 0x00]
                }
            )
        );
        assert_eq!(
            Threshold::from_parts(
                ThresholdMsb {
                    bytes: [0x00, 0x00]
                },
                ThresholdLsb {
                    bytes: [0x00, 0x00]
                }
            ),
            Threshold::default()
        );
    }

    #[test]
    #[allow(clippy::similar_names)]
    fn threshold_round_trip() {
        for thresholds in [
            0, 2_097_152, 4_194_304, 6_291_456, 8_388_608, 10_485_760, 12_582_912, 14_680_064,
        ] {
            for dc_block in enum_iterator::all::<DcBlock>() {
                let threshold = Threshold {
                    current_detect_threshold: u24::new(thresholds),
                    dc_block,
                };

                let (msb, lsb) = threshold.into_parts();
                assert_eq!(threshold, Threshold::from_parts(msb, lsb));
            }
        }
    }

    #[test]
    fn channel_config_default() {
        assert_eq!(ChannelConfig::default().to_be_bytes(), [0x00, 0x00]);
        assert_eq!(
            ChannelConfig::from_be_bytes([0x00, 0x00]),
            ChannelConfig::default()
        );
    }

    #[test]
    fn channel_config_round_trip() {
        for phase in [0, 128, 256, 384, 512, 640, 768, 896] {
            for dc_block_disable in [false, true] {
                for mux in enum_iterator::all::<ChannelMux>() {
                    let config = ChannelConfig {
                        phase: u10::new(phase),
                        dc_block_disable,
                        mux,
                    };

                    assert_eq!(config, ChannelConfig::from_be_bytes(config.to_be_bytes()));
                }
            }
        }
    }

    #[test]
    fn channel_offset_cal_default() {
        assert_eq!(
            ChannelOffsetCal::default().into_parts(),
            (
                ChannelOffsetCalMsb {
                    bytes: [0x00, 0x00]
                },
                ChannelOffsetCalLsb {
                    bytes: [0x00, 0x00]
                }
            )
        );
        assert_eq!(
            ChannelOffsetCal::from_parts(
                ChannelOffsetCalMsb {
                    bytes: [0x00, 0x00]
                },
                ChannelOffsetCalLsb {
                    bytes: [0x00, 0x00]
                }
            ),
            ChannelOffsetCal::default()
        );
    }

    #[test]
    #[allow(clippy::similar_names)]
    fn channel_offset_cal_round_trip() {
        for offset in [
            0, 2_097_152, 4_194_304, 6_291_456, 8_388_608, 10_485_760, 12_582_912, 14_680_064,
        ] {
            let offset_cal = ChannelOffsetCal {
                offset: u24::new(offset),
            };

            let (msb, lsb) = offset_cal.into_parts();
            assert_eq!(offset_cal, ChannelOffsetCal::from_parts(msb, lsb));
        }
    }

    #[test]
    fn channel_gain_cal_default() {
        assert_eq!(
            ChannelGainCal::default().into_parts(),
            (
                ChannelGainCalMsb { bytes: [0x80, 00] },
                ChannelGainCalLsb {
                    bytes: [0x00, 0x00]
                }
            )
        );
        assert_eq!(
            ChannelGainCal::from_parts(
                ChannelGainCalMsb { bytes: [0x80, 00] },
                ChannelGainCalLsb {
                    bytes: [0x00, 0x00]
                }
            ),
            ChannelGainCal::default()
        );
    }

    #[test]
    #[allow(clippy::similar_names)]
    fn channel_gain_cal_round_trip() {
        for gain in [
            0, 2_097_152, 4_194_304, 6_291_456, 8_388_608, 10_485_760, 12_582_912, 14_680_064,
        ] {
            let gain_cal = ChannelGainCal {
                gain: u24::new(gain),
            };

            let (msb, lsb) = gain_cal.into_parts();
            assert_eq!(gain_cal, ChannelGainCal::from_parts(msb, lsb));
        }
    }
}

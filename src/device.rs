//! Device Driver

use core::marker::PhantomData;

use crate::interface::Interface;
use crate::types::{
    ChannelConfig, Clock, Config, Gain, GainCal, Id, Mode, OffsetCal, Status, Threshold,
};
use crate::Error;

const ID_ADDR: u8 = 0x0;
const STATUS_ADDR: u8 = 0x1;
const MODE_ADDR: u8 = 0x2;
const CLOCK_ADDR: u8 = 0x3;
const GAIN_ADDR: u8 = 0x4;
const CFG_ADDR: u8 = 0x6;
const THRSHLD_MSB_ADDR: u8 = 0x7;
const THRSHLD_LSB_ADDR: u8 = 0x8;

const CH0_CFG_ADDR: u8 = 0x9;
const CH0_OCAL_MSB_ADDR: u8 = 0xA;
const CH0_OCAL_LSB_ADDR: u8 = 0xB;
const CH0_GCAL_MSB_ADDR: u8 = 0xC;
const CH0_GCAL_LSB_ADDR: u8 = 0xD;

const CH1_CFG_ADDR: u8 = 0xE;
const CH1_OCAL_MSB_ADDR: u8 = 0xF;
const CH1_OCAL_LSB_ADDR: u8 = 0x10;
const CH1_GCAL_MSB_ADDR: u8 = 0x11;
const CH1_GCAL_LSB_ADDR: u8 = 0x12;

const CH2_CFG_ADDR: u8 = 0x13;
const CH2_OCAL_MSB_ADDR: u8 = 0x14;
const CH2_OCAL_LSB_ADDR: u8 = 0x15;
const CH2_GCAL_MSB_ADDR: u8 = 0x16;
const CH2_GCAL_LSB_ADDR: u8 = 0x17;

const CH3_CFG_ADDR: u8 = 0x18;
const CH3_OCAL_MSB_ADDR: u8 = 0x19;
const CH3_OCAL_LSB_ADDR: u8 = 0x1A;
const CH3_GCAL_MSB_ADDR: u8 = 0x1B;
const CH3_GCAL_LSB_ADDR: u8 = 0x1C;

const REGMAP_CRC_ADDR: u8 = 0x3E;

/// ADS131M ADC driver
#[derive(Debug)]
pub struct Ads131m<W, I> {
    intf: I,
    register_config: bool,
    w: PhantomData<W>,
}

impl<W, I> Ads131m<W, I>
where
    I: Interface<W>,
    W: From<u8> + Copy,
{
    /// Initialize an ADS131M driver from an [`embedded-hal`] SPI interface
    ///
    /// The SPI interface must be configured for SPI mode 1
    ///
    /// This command assumes the device is in it's default state
    ///
    /// [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
    pub fn open(intf: I) -> Result<Ads131m<W, I>, Error<I::Error>> {
        Self::open_with_config(intf, bool::default())
    }

    /// Initialize an ADS131M driver from an [`embedded-hal`] SPI interface with a custom configuration
    ///
    /// The SPI interface must be configured for SPI mode 1
    ///
    /// [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
    pub fn open_with_config(intf: I, config: bool) -> Result<Ads131m<W, I>, Error<I::Error>> {
        Ok(Ads131m {
            intf,
            register_config: config,
            w: PhantomData,
        })
    }

    /// Get the current cache of the device configuration
    pub fn get_current_config(&self) -> &bool {
        &self.register_config
    }

    /// Read the ID register
    pub fn get_id(&mut self) -> Result<Id, Error<I::Error>> {
        let word = self.read_reg(ID_ADDR)?;
        Ok(Id::from_word(word))
    }

    /// Read the STATUS register
    pub fn get_status(&mut self) -> Result<Status, Error<I::Error>> {
        let word = self.read_reg(STATUS_ADDR)?;
        Ok(Status::from_word(word))
    }

    /// Read the MODE register
    pub fn get_mode(&mut self) -> Result<Mode, Error<I::Error>> {
        let word = self.read_reg(MODE_ADDR)?;
        Ok(Mode::from_word(word))
    }

    /// Write to the MODE register
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<I::Error>> {
        self.write_reg(MODE_ADDR, mode.to_word())
    }

    /// Read the CLOCK register
    pub fn get_clock(&mut self) -> Result<Clock, Error<I::Error>> {
        let word = self.read_reg(CLOCK_ADDR)?;
        Ok(Clock::from_word(word))
    }

    /// Write to the CLOCK register
    pub fn set_clock(&mut self, clock: Clock) -> Result<(), Error<I::Error>> {
        self.write_reg(CLOCK_ADDR, clock.to_word())
    }

    /// Read the GAIN1 register
    pub fn get_gain(&mut self) -> Result<Gain, Error<I::Error>> {
        let word = self.read_reg(GAIN_ADDR)?;
        Ok(Gain::from_word(word))
    }

    /// Write to the GAIN1 register
    pub fn set_gain(&mut self, gain: Gain) -> Result<(), Error<I::Error>> {
        self.write_reg(GAIN_ADDR, gain.to_word())
    }

    /// Read the CFG register
    pub fn get_config(&mut self) -> Result<Config, Error<I::Error>> {
        let word = self.read_reg(CFG_ADDR)?;
        Ok(Config::from_word(word))
    }

    /// Write to the CFG register
    pub fn set_config(&mut self, gain: Config) -> Result<(), Error<I::Error>> {
        self.write_reg(CFG_ADDR, gain.to_word())
    }

    /// Read the THRSHLD_MSB and THRSHLD_LSB registers
    pub fn get_threshold(&mut self) -> Result<Threshold, Error<I::Error>> {
        let msb = self.read_reg(THRSHLD_MSB_ADDR)?;
        let lsb = self.read_reg(THRSHLD_LSB_ADDR)?;
        Ok(Threshold::from_words([msb, lsb]))
    }

    /// Write to the THRSHLD_MSB and THRSHLD_LSB registers
    pub fn set_threshold(&mut self, threshold: Threshold) -> Result<(), Error<I::Error>> {
        let [msb, lsb] = threshold.to_words();
        self.write_reg(THRSHLD_MSB_ADDR, msb)?;
        self.write_reg(THRSHLD_LSB_ADDR, lsb)
    }

    /// Read the CH0_CFG register
    ///
    /// TODO: Figure out how to handle multiple channels
    pub fn get_channel_0_config(&mut self) -> Result<ChannelConfig, Error<I::Error>> {
        let word = self.read_reg(CH0_CFG_ADDR)?;
        Ok(ChannelConfig::from_word(word))
    }

    /// Write to the CH0_CFG register
    ///
    /// TODO: Figure out how to handle multiple channels
    pub fn set_channel_0_config(&mut self, config: ChannelConfig) -> Result<(), Error<I::Error>> {
        self.write_reg(CH0_CFG_ADDR, config.to_word())
    }

    /// Read the CH0_OCAL_MSB and CH0_OCAL_LSB registers
    ///
    /// TODO: Figure out how to handle multiple channels
    pub fn get_channel_0_offset_cal(&mut self) -> Result<OffsetCal, Error<I::Error>> {
        let msb = self.read_reg(CH0_OCAL_MSB_ADDR)?;
        let lsb = self.read_reg(CH0_OCAL_LSB_ADDR)?;
        Ok(OffsetCal::from_words([msb, lsb]))
    }

    /// Write to the CH0_OCAL_MSB and CH0_OCAL_LSB registers
    ///
    /// TODO: Figure out how to handle multiple channels
    pub fn set_channel_0_offset_cal(
        &mut self,
        offset_cal: OffsetCal,
    ) -> Result<(), Error<I::Error>> {
        let [msb, lsb] = offset_cal.to_words();
        self.write_reg(CH0_OCAL_MSB_ADDR, msb)?;
        self.write_reg(CH0_OCAL_LSB_ADDR, lsb)
    }

    /// Read the CH0_GCAL_MSB and CH0_GCAL_LSB registers
    ///
    /// TODO: Figure out how to handle multiple channels
    pub fn get_channel_0_gain_cal(&mut self) -> Result<GainCal, Error<I::Error>> {
        let msb = self.read_reg(CH0_GCAL_MSB_ADDR)?;
        let lsb = self.read_reg(CH0_GCAL_LSB_ADDR)?;
        Ok(GainCal::from_words([msb, lsb]))
    }

    /// Write to the CH0_GCAL_MSB and CH0_GCAL_LSB registers
    ///
    /// TODO: Figure out how to handle multiple channels
    pub fn set_channel_0_gain_cal(&mut self, gain_cal: GainCal) -> Result<(), Error<I::Error>> {
        let [msb, lsb] = gain_cal.to_words();
        self.write_reg(CH0_GCAL_MSB_ADDR, msb)?;
        self.write_reg(CH0_GCAL_LSB_ADDR, lsb)
    }

    fn read_reg(&mut self, _address: u8) -> Result<u16, Error<I::Error>> {
        unimplemented!()
    }

    fn write_reg(&mut self, _address: u8, _word: u16) -> Result<(), Error<I::Error>> {
        unimplemented!()
    }
}

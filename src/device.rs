//! Device Driver

use core::marker::PhantomData;

use crate::interface::Interface;
use crate::registers;
use crate::types::{
    ChannelConfig, Clock, Config, Gain, GainCal, Id, Mode, OffsetCal, Status, Threshold,
};
use crate::Error;
use concat_idents::concat_idents;

// Maybe some fancy const stuff can be done instead of this at some point.
// Just having a channel_idx input would not be checked at compile time
#[rustfmt::skip]
macro_rules! impl_channel {
    ($trait_name:ident, $channel_name:ident, $channel_num:literal) => {
        #[doc=concat!("Methods for configuring ADC channel ", $channel_num)]
        pub trait $trait_name<E>
        where
            Self: AdcCommon<E>,
        {
            concat_idents!(get_channel_config = get_, $channel_name, _config, {
                #[doc=concat!(
                    "Read the `CH",
                    $channel_num,
                    "_CFG` register"
                )]
                fn get_channel_config(&mut self) -> Result<ChannelConfig, E> {
                    let word = self.read_reg(registers::CHANNEL_CONFIG_ADDRS[$channel_num])?;
                    Ok(ChannelConfig::from_word(word))
                }
            });

            concat_idents!(set_channel_config = set_, $channel_name, _config, {
                #[doc=concat!(
                    "Write to the `CH",
                    $channel_num,
                    "_CFG` register"
                )]
                fn set_channel_config(&mut self, config: ChannelConfig) -> Result<(), E> {
                    self.write_reg(registers::CHANNEL_CONFIG_ADDRS[$channel_num], config.to_word())
                }
            });

            concat_idents!(get_channel_offset_cal = get_, $channel_name, _offset_cal, {
                #[doc=concat!(
                    "Read the `CH",
                    $channel_num,
                    "_OCAL_MSB` and `CH",
                    $channel_num,
                    "_OCAL_LSB` registers"
                )]
                fn get_channel_offset_cal(&mut self) -> Result<OffsetCal, E> {
                    let msb = self.read_reg(registers::CHANNEL_OCAL_MSB_ADDRS[$channel_num])?;
                    let lsb = self.read_reg(registers::CHANNEL_OCAL_LSB_ADDRS[$channel_num])?;
                    Ok(OffsetCal::from_words([msb, lsb]))
                }
            });

            concat_idents!(set_channel_offset_cal = set_, $channel_name, _offset_cal, {
                #[doc=concat!(
                    "Write to the `CH",
                    $channel_num,
                    "_OCAL_MSB` and `CH",
                    $channel_num,
                    "_OCAL_LSB` registers"
                )]
                fn set_channel_offset_cal(&mut self, offset_cal: OffsetCal) -> Result<(), E> {
                    let [msb, lsb] = offset_cal.to_words();
                    self.write_reg(registers::CHANNEL_OCAL_MSB_ADDRS[$channel_num], msb)?;
                    self.write_reg(registers::CHANNEL_OCAL_LSB_ADDRS[$channel_num], lsb)
                }
            });

            concat_idents!(get_channel_gain_cal = get_, $channel_name, _gain_cal, {
                #[doc=concat!(
                    "Read the `CH",
                    $channel_num,
                    "_GCAL_MSB` and `CH",
                    $channel_num,
                    "_GCAL_LSB` registers"
                )]
                fn get_channel_gain_cal(&mut self) -> Result<GainCal, E> {
                    let msb = self.read_reg(registers::CHANNEL_GCAL_MSB_ADDRS[$channel_num])?;
                    let lsb = self.read_reg(registers::CHANNEL_GCAL_LSB_ADDRS[$channel_num])?;
                    Ok(GainCal::from_words([msb, lsb]))
                }
            });

            concat_idents!(set_channel_gain_cal = set_, $channel_name, _gain_cal, {
                #[doc=concat!(
                    "Write to the `CH",
                    $channel_num,
                    "_GCAL_MSB` and `CH",
                    $channel_num,
                    "_GCAL_LSB` registers"
                )]
                fn set_channel_gain_cal(&mut self, gain_cal: GainCal) -> Result<(), E> {
                    let [msb, lsb] = gain_cal.to_words();
                    self.write_reg(registers::CHANNEL_GCAL_MSB_ADDRS[$channel_num], msb)?;
                    self.write_reg(registers::CHANNEL_GCAL_LSB_ADDRS[$channel_num], lsb)
                }
            });
        }
    };
}

macro_rules! impl_model {
    ($model:ident, $channel_count_str:literal, [$($channel:ident),+]) => {
        #[doc=concat!( "Driver for a TI ADS131M0", $channel_count_str, " ", $channel_count_str, "-channel 24-bit ADC\n\n[Datasheet](https://www.ti.com/lit/ds/symlink/ads131m0", $channel_count_str, ".pdf)")]
        pub struct $model<E> {
            inner: AdcInner<E>,
        }

        impl<E> AdcBase<E> for $model<E> {
            fn transfer(&mut self, send: bool) -> bool {
                self.inner.transfer(send)
            }
            fn get_mode_mut(&mut self) -> &mut Mode {
                &mut self.inner.mode
            }
        }

        impl<E> AdcCommon<E> for $model<E> {
            fn open_with_mode(intf: bool, mode: Mode) -> Result<Self, E> {
                Ok(Self {
                    inner: AdcInner::new(intf, mode)?
                })
            }
        }

        $(impl<E> $channel<E> for $model<E> {})+
    };
}

struct AdcInner<E> {
    intf: bool,
    mode: Mode,
    e: PhantomData<E>,
}

impl<E> AdcInner<E> {
    fn new(intf: bool, mode: Mode) -> Result<Self, E> {
        unimplemented!()
    }

    // TODO: Figure out transfer signature
    fn transfer(&mut self, send: bool) -> bool {
        unimplemented!()
    }
}

#[doc(hidden)]
pub trait AdcBase<E> {
    // TODO: Figure out transfer signature
    fn transfer(&mut self, send: bool) -> bool;
    fn get_mode_mut(&mut self) -> &mut Mode;
}

/// Main ADC methods
pub trait AdcCommon<E>
where
    Self: AdcBase<E> + Sized,
{
    /// Initialize an ADC driver from an [`embedded-hal`] SPI interface with a custom configuration
    ///
    /// The SPI interface must be configured for SPI mode 1
    ///
    /// [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
    fn open_with_mode(intf: bool, mode: Mode) -> Result<Self, E>;

    /// Initialize an ADC driver from an [`embedded-hal`] SPI interface
    ///
    /// The SPI interface must be configured for SPI mode 1
    ///
    /// This command assumes the device is in it's default state
    ///
    /// [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
    fn open(intf: bool) -> Result<Self, E> {
        Self::open_with_mode(intf, Mode::default())
    }

    /// Read the ID register
    fn get_id(&mut self) -> Result<Id, E> {
        let word = self.read_reg(registers::ID_ADDR)?;
        Ok(Id::from_word(word))
    }

    /// Read the STATUS register
    fn get_status(&mut self) -> Result<Status, E> {
        let word = self.read_reg(registers::STATUS_ADDR)?;
        Ok(Status::from_word(word))
    }

    /// Read the MODE register
    fn get_mode(&mut self) -> Result<Mode, E> {
        let word = self.read_reg(registers::MODE_ADDR)?;
        Ok(Mode::from_word(word))
    }

    /// Write to the MODE register
    fn set_mode(&mut self, mode: Mode) -> Result<(), E> {
        self.write_reg(registers::MODE_ADDR, mode.to_word())
    }

    /// Read the CLOCK register
    fn get_clock(&mut self) -> Result<Clock, E> {
        let word = self.read_reg(registers::CLOCK_ADDR)?;
        Ok(Clock::from_word(word))
    }

    /// Write to the CLOCK register
    fn set_clock(&mut self, clock: Clock) -> Result<(), E> {
        self.write_reg(registers::CLOCK_ADDR, clock.to_word())
    }

    /// Read the GAIN1 register
    fn get_gain(&mut self) -> Result<Gain, E> {
        let word = self.read_reg(registers::GAIN_ADDR)?;
        Ok(Gain::from_word(word))
    }

    /// Write to the GAIN1 register
    fn set_gain(&mut self, gain: Gain) -> Result<(), E> {
        self.write_reg(registers::GAIN_ADDR, gain.to_word())
    }

    /// Read the CFG register
    fn get_config(&mut self) -> Result<Config, E> {
        let word = self.read_reg(registers::CFG_ADDR)?;
        Ok(Config::from_word(word))
    }

    /// Write to the CFG register
    fn set_config(&mut self, gain: Config) -> Result<(), E> {
        self.write_reg(registers::CFG_ADDR, gain.to_word())
    }

    /// Read the `THRSHLD_MSB` and `THRSHLD_LSB` registers
    fn get_threshold(&mut self) -> Result<Threshold, E> {
        let msb = self.read_reg(registers::THRSHLD_MSB_ADDR)?;
        let lsb = self.read_reg(registers::THRSHLD_LSB_ADDR)?;
        Ok(Threshold::from_words([msb, lsb]))
    }

    /// Write to the `THRSHLD_MSB` and `THRSHLD_LSB` registers
    fn set_threshold(&mut self, threshold: Threshold) -> Result<(), E> {
        let [msb, lsb] = threshold.to_words();
        self.write_reg(registers::THRSHLD_MSB_ADDR, msb)?;
        self.write_reg(registers::THRSHLD_LSB_ADDR, lsb)
    }

    #[doc(hidden)]
    fn read_reg(&mut self, address: u8) -> Result<u16, E> {
        unimplemented!()
    }

    #[doc(hidden)]
    fn write_reg(&mut self, address: u8, word: u16) -> Result<(), E> {
        let _ = self;
        unimplemented!()
    }
}

impl_channel!(AdcChannel0, channel_0, 0);
impl_channel!(AdcChannel1, channel_1, 1);
impl_channel!(AdcChannel2, channel_2, 2);
impl_channel!(AdcChannel3, channel_3, 3);
impl_channel!(AdcChannel4, channel_4, 4);
impl_channel!(AdcChannel5, channel_5, 5);
impl_channel!(AdcChannel6, channel_6, 6);
impl_channel!(AdcChannel7, channel_7, 7);

impl_model!(Ads131m02, "2", [AdcChannel0, AdcChannel1]);
impl_model!(Ads131m03, "3", [AdcChannel0, AdcChannel1, AdcChannel2]);
impl_model!(
    Ads131m04,
    "4",
    [AdcChannel0, AdcChannel1, AdcChannel2, AdcChannel3]
);
impl_model!(
    Ads131m06,
    "6",
    [
        AdcChannel0,
        AdcChannel1,
        AdcChannel2,
        AdcChannel3,
        AdcChannel4,
        AdcChannel5
    ]
);
impl_model!(
    Ads131m08,
    "8",
    [
        AdcChannel0,
        AdcChannel1,
        AdcChannel2,
        AdcChannel3,
        AdcChannel4,
        AdcChannel5,
        AdcChannel6,
        AdcChannel7
    ]
);

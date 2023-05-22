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
        pub trait $trait_name<I, E>
        where
            Self: AdcCommon<I, E>,
        {
            concat_idents!(get_channel_config = get_, $channel_name, _config, {
                #[doc=concat!(
                    "Read the `CH",
                    $channel_num,
                    "_CFG` register"
                )]
                fn get_channel_config(&mut self) -> Result<ChannelConfig, Error<E>> {
                    let bytes = self.read_reg(registers::CHANNEL_CONFIG_ADDRS[$channel_num])?;
                    Ok(ChannelConfig::from_be_bytes(bytes))
                }
            });

            concat_idents!(set_channel_config = set_, $channel_name, _config, {
                #[doc=concat!(
                    "Write to the `CH",
                    $channel_num,
                    "_CFG` register"
                )]
                fn set_channel_config(&mut self, config: ChannelConfig) -> Result<(), Error<E>> {
                    self.write_reg(registers::CHANNEL_CONFIG_ADDRS[$channel_num], config.to_be_bytes())
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
                fn get_channel_offset_cal(&mut self) -> Result<OffsetCal, Error<E>> {
                    let [b0, b1] = self.read_reg(registers::CHANNEL_OCAL_MSB_ADDRS[$channel_num])?;
                    let [b2, b3] = self.read_reg(registers::CHANNEL_OCAL_LSB_ADDRS[$channel_num])?;
                    Ok(OffsetCal::from_be_bytes([b0, b1, b2, b3]))
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
                fn set_channel_offset_cal(&mut self, offset_cal: OffsetCal) -> Result<(), Error<E>> {
                    let [b0, b1, b2, b3] = offset_cal.to_be_bytes();
                    self.write_reg(registers::CHANNEL_OCAL_MSB_ADDRS[$channel_num], [b0, b1])?;
                    self.write_reg(registers::CHANNEL_OCAL_LSB_ADDRS[$channel_num], [b2, b3])
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
                fn get_channel_gain_cal(&mut self) -> Result<GainCal, Error<E>> {
                    let [b0, b1] = self.read_reg(registers::CHANNEL_GCAL_MSB_ADDRS[$channel_num])?;
                    let [b2, b3] = self.read_reg(registers::CHANNEL_GCAL_LSB_ADDRS[$channel_num])?;
                    Ok(GainCal::from_be_bytes([b0, b1, b2, b3]))
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
                fn set_channel_gain_cal(&mut self, gain_cal: GainCal) -> Result<(), Error<E>> {
                    let [b0, b1, b2, b3] = gain_cal.to_be_bytes();
                    self.write_reg(registers::CHANNEL_GCAL_MSB_ADDRS[$channel_num], [b0, b1])?;
                    self.write_reg(registers::CHANNEL_GCAL_LSB_ADDRS[$channel_num], [b2, b3])
                }
            });
        }
    };
}

#[rustfmt::skip]
macro_rules! impl_model {
    ($model:ident, $channel_count_str:literal, [$($channel:ident),+]) => {
        #[doc=concat!(
            "Driver for a TI ADS131M0",
            $channel_count_str,
            " ",
            $channel_count_str,
            "-channel 24-bit ADC\n\n",
            "[Datasheet](https://www.ti.com/lit/ds/symlink/ads131m0",
            $channel_count_str,
            ".pdf)",
        )]
        pub struct $model<I: Interface<E, W>, E, W: Copy> {
            inner: AdcInner<I, E, W>,
        }

        impl<I: Interface<E, W>, E, W: Copy> $model<I, E, W> {
            /// Initialize an ADC driver from an [`embedded-hal`] SPI interface
            ///
            /// The SPI interface must be configured for SPI mode 1
            ///
            /// This command assumes the device is in it's default (reset) state
            ///
            /// [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
            pub fn open(intf: I) -> Result<Self, Error<E>> {
                Self::open_with_mode(intf, Mode::default())
            }

            /// Initialize an ADC driver from an [`embedded-hal`] SPI interface with a custom configuration
            ///
            /// The SPI interface must be configured for SPI mode 1
            ///
            /// [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
            pub fn open_with_mode(intf: I, mode: Mode) -> Result<Self, Error<E>> {
                Ok(Self {
                    inner: AdcInner::new(intf, mode)?
                })
            }
        }

        impl<I: Interface<E, W>, E, W: Copy> AdcBase<E> for $model<I, E, W> {
            fn transfer(&mut self, send: bool) -> Result<bool, Error<E>> {
                self.inner.transfer(send)
            }
            fn get_mode_mut(&mut self) -> &mut Mode {
                &mut self.inner.mode
            }
        }

        impl<I: Interface<E, W>, E, W: Copy> AdcCommon<I, E> for $model<I, E, W> {}

        $(impl<I: Interface<E, W>, E, W: Copy> $channel<I, E> for $model<I, E, W> {})+
    };
}

struct AdcInner<I: Interface<E, W>, E, W: Copy> {
    intf: I,
    mode: Mode,
    e: PhantomData<E>,
    w: PhantomData<W>,
}

impl<I, E, W> AdcInner<I, E, W>
where
    I: Interface<E, W>,
    W: Copy,
{
    fn new(intf: I, mode: Mode) -> Result<Self, Error<E>> {
        unimplemented!()
    }

    // TODO: Figure out transfer signature
    fn transfer(&mut self, send: bool) -> Result<bool, Error<E>> {
        unimplemented!()
    }
}

#[doc(hidden)]
pub trait AdcBase<E> {
    // TODO: Figure out transfer signature
    fn transfer(&mut self, send: bool) -> Result<bool, Error<E>>;
    fn get_mode_mut(&mut self) -> &mut Mode;
}

/// Main ADC methods
pub trait AdcCommon<I, E>
where
    Self: AdcBase<E> + Sized,
{
    /// Read the ID register
    fn get_id(&mut self) -> Result<Id, Error<E>> {
        let bytes = self.read_reg(registers::ID_ADDR)?;
        Ok(Id::from_be_bytes(bytes))
    }

    /// Read the STATUS register
    fn get_status(&mut self) -> Result<Status, Error<E>> {
        let bytes = self.read_reg(registers::STATUS_ADDR)?;
        Ok(Status::from_be_bytes(bytes))
    }

    /// Read the MODE register
    fn get_mode(&mut self) -> Result<Mode, Error<E>> {
        let bytes = self.read_reg(registers::MODE_ADDR)?;
        Ok(Mode::from_be_bytes(bytes))
    }

    /// Write to the MODE register
    fn set_mode(&mut self, mode: Mode) -> Result<(), Error<E>> {
        self.write_reg(registers::MODE_ADDR, mode.to_be_bytes())
    }

    /// Read the CLOCK register
    fn get_clock(&mut self) -> Result<Clock, Error<E>> {
        let bytes = self.read_reg(registers::CLOCK_ADDR)?;
        Ok(Clock::from_be_bytes(bytes))
    }

    /// Write to the CLOCK register
    fn set_clock(&mut self, clock: Clock) -> Result<(), Error<E>> {
        self.write_reg(registers::CLOCK_ADDR, clock.to_be_bytes())
    }

    /// Read the GAIN1 register
    fn get_gain(&mut self) -> Result<Gain, Error<E>> {
        let bytes = self.read_reg(registers::GAIN_ADDR)?;
        Ok(Gain::from_be_bytes(bytes))
    }

    /// Write to the GAIN1 register
    fn set_gain(&mut self, gain: Gain) -> Result<(), Error<E>> {
        self.write_reg(registers::GAIN_ADDR, gain.to_be_bytes())
    }

    /// Read the CFG register
    fn get_config(&mut self) -> Result<Config, Error<E>> {
        let bytes = self.read_reg(registers::CFG_ADDR)?;
        Ok(Config::from_be_bytes(bytes))
    }

    /// Write to the CFG register
    fn set_config(&mut self, gain: Config) -> Result<(), Error<E>> {
        self.write_reg(registers::CFG_ADDR, gain.to_be_bytes())
    }

    /// Read the `THRSHLD_MSB` and `THRSHLD_LSB` registers
    fn get_threshold(&mut self) -> Result<Threshold, Error<E>> {
        let [b0, b1] = self.read_reg(registers::THRSHLD_MSB_ADDR)?;
        let [b2, b3] = self.read_reg(registers::THRSHLD_LSB_ADDR)?;
        Ok(Threshold::from_be_bytes([b0, b1, b2, b3]))
    }

    /// Write to the `THRSHLD_MSB` and `THRSHLD_LSB` registers
    fn set_threshold(&mut self, threshold: Threshold) -> Result<(), Error<E>> {
        let [b0, b1, b2, b3] = threshold.to_be_bytes();
        self.write_reg(registers::THRSHLD_MSB_ADDR, [b0, b1])?;
        self.write_reg(registers::THRSHLD_LSB_ADDR, [b2, b3])
    }

    #[doc(hidden)]
    fn read_reg(&mut self, address: u8) -> Result<[u8; 2], Error<E>> {
        unimplemented!()
    }

    #[doc(hidden)]
    fn write_reg(&mut self, address: u8, bytes: [u8; 2]) -> Result<(), Error<E>> {
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

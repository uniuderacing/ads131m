//! Device Driver

use core::marker::PhantomData;

use crate::interface::Interface;
use crate::registers;
use crate::types::{
    ChannelConfig, Clock, Command, Config, CrcType, Gain, GainCal, Id, Mode, OffsetCal, Response,
    Status, Threshold, WordLength,
};
use crate::Error;
use concat_idents::concat_idents;
use crc::{Crc, CRC_16_CMS, CRC_16_IBM_3740};
use ux::u6;

// Since no const-generic math, we have to use the max possible buffer size
// 32-bit words (4 bytes) * ( 1 response word + 8 channel data + 1 CRC word)
const BUF_SIZE: usize = 4 * (1 + 8 + 1);

// Maybe some fancy const stuff can be done instead of this at some point.
// Just having a channel_idx input would not be checked at compile time
#[rustfmt::skip]
macro_rules! impl_channel {
    ($trait_name:ident, $channel_name:ident, $channel_num:literal) => {
        #[doc=concat!("Methods for configuring ADC channel ", $channel_num)]
        pub trait $trait_name<I, E>
        where
            Self: RegisterAccess<E>,
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

macro_rules! impl_model {
    ($model:ident, $channel_count:literal, [$($channel:ident),+]) => {
        impl<I, E, W> Ads131m<I, E, W, $channel_count>
        where
            I: Interface<E, W>,
            W: Copy,
        {
            concat_idents!(open = open_, $model, {
                #[doc=concat!(
                    "Initialize a TI [`ADS131M0",
                    $channel_count,
                    "`] ADC driver from an [`embedded-hal`] SPI interface\n\n",
                    "The SPI interface must be configured for SPI mode 1\n\n",
                    "The device must have it's `MODE` register in the default (reset) state\n\n",
                    "[`ADS131M0",
                    $channel_count,
                    "`]: https://www.ti.com/lit/ds/symlink/ads131m0",
                    $channel_count,
                    "\n",
                    "[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal"
                )]
                pub fn open(intf: I) -> Result<Self, Error<E>> {
                    Self::new(intf, Mode::default())
                }
            });
            concat_idents!(open_with_mode = open_, $model, _with_mode, {
                #[doc=concat!(
                    "Initialize a TI [`ADS131M0",
                    $channel_count,
                    "`] ADC driver from an [`embedded-hal`] SPI interface with a custom configuration\n\n",
                    "The SPI interface must be configured for SPI mode 1\n\n",
                    "The current state of the device's `MODE` register must match the `mode` argument\n\n",
                    "[`ADS131M0",
                    $channel_count,
                    "`]: https://www.ti.com/lit/ds/symlink/ads131m0",
                    $channel_count,
                    "\n",
                    "[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal"
                )]
                pub fn open_with_mode(intf: I, mode: Mode) -> Result<Self, Error<E>> {
                    Self::new(intf, mode)
                }
            });
        }

        $(impl<I: Interface<E, W>, E, W: Copy> $channel<I, E> for Ads131m<I, E, W, $channel_count> {})+
    };
}

/// Driver
///
/// TODO: Description
///
/// TODO: Examples
pub struct Ads131m<I: Interface<E, W>, E, W: Copy, const C: usize> {
    // Comms
    intf: I,
    read_buf: [u8; BUF_SIZE],
    write_buf: [u8; BUF_SIZE],
    // Mode
    crc_table: Crc<u16>,
    word_len: usize,
    word_packing: WordLength,
    spi_crc_enable: bool,
    //Type stuff
    e: PhantomData<E>,
    w: PhantomData<W>,
}

impl<Intf, IntfError, IntfWord, const CHANNELS: usize> Ads131m<Intf, IntfError, IntfWord, CHANNELS>
where
    Intf: Interface<IntfError, IntfWord>,
    IntfWord: Copy,
{
    /// Reset the device to it's default state.
    ///
    /// The device must not be used again for 5us while the registers stabilize
    ///
    /// This also updates the internal cache of the device mode, which can change how the driver communicates
    pub fn reset(&mut self) -> Result<(), Error<IntfError>> {
        // TODO: Do something with these samples
        let (_, samples) = self.send_simple_command(Command::Reset)?;
        self.process_new_mode(Mode::default());
        Ok(())
    }

    /// Place the device in a low power standby mode
    /// This disables all device channels and powers down non essential circuitry
    /// The sample clock should be disabled after this command to maximize power savings
    pub fn standby(&mut self) -> Result<(), Error<IntfError>> {
        // TODO: Do something with these samples
        let (_, samples_1) = self.send_simple_command(Command::Standby)?;
        let (resp, samples_2) = self.send_simple_command(Command::Null)?;
        match Response::try_from_be_bytes(resp) {
            Some(Response::Standby) => Ok(()),
            e => Err(Error::UnexpectedResponse(e)),
        }
    }

    /// Wake the device from standby mode
    /// The sample clock should be enables after this command
    pub fn wakeup(&mut self) -> Result<(), Error<IntfError>> {
        // TODO: Do something with these samples
        let (_, samples_1) = self.send_simple_command(Command::Wakeup)?;
        let (resp, samples_2) = self.send_simple_command(Command::Null)?;
        match Response::try_from_be_bytes(resp) {
            Some(Response::Wakeup) => Ok(()),
            e => Err(Error::UnexpectedResponse(e)),
        }
    }

    /// Lock the device to only respond to the Null, Read Register, and Unlock commands
    pub fn lock(&mut self) -> Result<(), Error<IntfError>> {
        // TODO: Do something with these samples
        let (_, samples_1) = self.send_simple_command(Command::Lock)?;
        let (resp, samples_2) = self.send_simple_command(Command::Null)?;
        match Response::try_from_be_bytes(resp) {
            Some(Response::Lock) => Ok(()),
            e => Err(Error::UnexpectedResponse(e)),
        }
    }

    /// Unlock the device if it was locked
    pub fn unlock(&mut self) -> Result<(), Error<IntfError>> {
        // TODO: Do something with these samples
        let (_, samples_1) = self.send_simple_command(Command::Unlock)?;
        let (resp, samples_2) = self.send_simple_command(Command::Null)?;
        match Response::try_from_be_bytes(resp) {
            Some(Response::Unlock) => Ok(()),
            e => Err(Error::UnexpectedResponse(e)),
        }
    }

    /// Read the `ID` register
    pub fn get_id(&mut self) -> Result<Id, Error<IntfError>> {
        let bytes = self.read_reg(registers::ID_ADDR)?;
        Ok(Id::from_be_bytes(bytes))
    }

    /// Read the `STATUS` register
    pub fn get_status(&mut self) -> Result<Status, Error<IntfError>> {
        let bytes = self.read_reg(registers::STATUS_ADDR)?;
        Ok(Status::from_be_bytes(bytes))
    }

    /// Read the `MODE` register
    ///
    /// This also updates the internal cache of the device mode, which can change how the driver communicates
    pub fn get_mode(&mut self) -> Result<Mode, Error<IntfError>> {
        let bytes = self.read_reg(registers::MODE_ADDR)?;
        let mode = Mode::from_be_bytes(bytes);

        self.process_new_mode(mode.clone());
        Ok(mode)
    }

    /// Write to the `MODE` register
    ///
    /// This also updates the internal cache of the device mode, which can change how the driver communicates
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<IntfError>> {
        self.write_reg(registers::MODE_ADDR, mode.to_be_bytes())?;

        self.process_new_mode(mode);
        Ok(())
    }

    /// Read the `CLOCK` register
    pub fn get_clock(&mut self) -> Result<Clock, Error<IntfError>> {
        let bytes = self.read_reg(registers::CLOCK_ADDR)?;
        Ok(Clock::from_be_bytes(bytes))
    }

    /// Write to the `CLOCK` register
    pub fn set_clock(&mut self, clock: Clock) -> Result<(), Error<IntfError>> {
        self.write_reg(registers::CLOCK_ADDR, clock.to_be_bytes())
    }

    /// Read the GAIN1 register
    pub fn get_gain(&mut self) -> Result<Gain, Error<IntfError>> {
        let bytes = self.read_reg(registers::GAIN_ADDR)?;
        Ok(Gain::from_be_bytes(bytes))
    }

    /// Write to the `GAIN1` register
    pub fn set_gain(&mut self, gain: Gain) -> Result<(), Error<IntfError>> {
        self.write_reg(registers::GAIN_ADDR, gain.to_be_bytes())
    }

    /// Read the `CFG` register
    pub fn get_config(&mut self) -> Result<Config, Error<IntfError>> {
        let bytes = self.read_reg(registers::CFG_ADDR)?;
        Ok(Config::from_be_bytes(bytes))
    }

    /// Write to the `CFG` register
    pub fn set_config(&mut self, gain: Config) -> Result<(), Error<IntfError>> {
        self.write_reg(registers::CFG_ADDR, gain.to_be_bytes())
    }

    /// Read the `THRSHLD_MSB` and `THRSHLD_LSB` registers
    pub fn get_threshold(&mut self) -> Result<Threshold, Error<IntfError>> {
        let [b0, b1] = self.read_reg(registers::THRSHLD_MSB_ADDR)?;
        let [b2, b3] = self.read_reg(registers::THRSHLD_LSB_ADDR)?;
        Ok(Threshold::from_be_bytes([b0, b1, b2, b3]))
    }

    /// Write to the `THRSHLD_MSB` and `THRSHLD_LSB` registers
    pub fn set_threshold(&mut self, threshold: Threshold) -> Result<(), Error<IntfError>> {
        let [b0, b1, b2, b3] = threshold.to_be_bytes();
        self.write_reg(registers::THRSHLD_MSB_ADDR, [b0, b1])?;
        self.write_reg(registers::THRSHLD_LSB_ADDR, [b2, b3])
    }

    fn new(intf: Intf, mode: Mode) -> Result<Self, Error<IntfError>> {
        let mut driver = Self {
            intf,
            read_buf: [0; BUF_SIZE],
            write_buf: [0; BUF_SIZE],
            crc_table: Crc::<u16>::new(&CRC_16_CMS),
            word_len: 2,
            word_packing: WordLength::Bits16,
            spi_crc_enable: false,
            e: PhantomData,
            w: PhantomData,
        };
        driver.process_new_mode(mode);

        Ok(driver)
    }

    fn process_new_mode(&mut self, mode: Mode) {
        let crc_alg = match mode.crc_type {
            CrcType::Ccitt => &CRC_16_CMS,
            CrcType::Ansi => &CRC_16_IBM_3740,
        };

        let word_len = match mode.word_length {
            WordLength::Bits16 => 2,
            WordLength::Bits24 => 3,
            _ => 4,
        };

        self.crc_table = Crc::<u16>::new(crc_alg);
        self.word_len = word_len;
        self.word_packing = mode.word_length;
        self.spi_crc_enable = mode.spi_crc_enable;
    }

    fn send_simple_command(
        &mut self,
        command: Command,
    ) -> Result<([u8; 2], [[u8; 3]; CHANNELS]), Error<IntfError>> {
        let mut bytes_written = 0;

        // Commands are always 2 bytes
        self.write_buf[0..2].copy_from_slice(&command.to_be_bytes());
        bytes_written += 2;

        while bytes_written < self.word_len {
            self.write_buf[bytes_written] = 0;
            bytes_written += 1;
        }

        let read_len = (1 + CHANNELS) * self.word_len;
        self.transfer_frame(bytes_written, read_len, command != Command::Reset)?;

        // Responses are always 2 bytes
        let response: [u8; 2] = self.read_buf[..2].try_into().unwrap();
        let sample_data =
            self.decode_samples(&self.read_buf[self.word_len..self.word_len * (CHANNELS + 1)]);

        Ok((response, sample_data))
    }

    /// Exchange a single SPI frame with the ADC
    /// Panics if `command_len` or `resp_len` is greater than `BUF_SIZE` - `self.word_len`
    fn transfer_frame(
        &mut self,
        command_len: usize,
        read_len: usize,
        check_resp_crc: bool,
    ) -> Result<(), Error<IntfError>> {
        let mut crc_len = 0;

        if self.spi_crc_enable {
            // CRCs are always 2 bytes
            let write_crc = self.crc_table.checksum(&self.write_buf[..command_len]);
            // Only write 2 bytes of CRC, transfer will pad remaining bytes out to word length
            self.write_buf[command_len..command_len + 2].copy_from_slice(&write_crc.to_be_bytes());
            crc_len += 2;

            while crc_len < self.word_len {
                self.write_buf[command_len + crc_len] = 0;
                crc_len += 1;
            }
        }

        // This will only happen for ADS131M03 with 24bit word len
        // This will avoid a panic, but I have no idea how that device will handle a padded transaction
        // TODO: Test this
        let real_read_len = if read_len % 2 == 1 {
            read_len + 1
        } else {
            read_len
        };

        self.intf.transfer(
            &self.write_buf[..command_len + crc_len],
            &mut self.read_buf[..real_read_len + self.word_len],
        )?;

        if check_resp_crc {
            // CRCs are always 2 bytes
            let resp_crc =
                u16::from_be_bytes(self.read_buf[read_len..read_len + 2].try_into().unwrap());
            let computed_crc = self.crc_table.checksum(&self.read_buf[..read_len]);
            if resp_crc != computed_crc {
                return Err(Error::ReceiveCrc {
                    computed: computed_crc,
                    received: resp_crc,
                });
            }
        }

        Ok(())
    }

    /// TODO: This should probably return a more rich type
    /// Panics if `buf` is not `self.word_len` * `CHANNELS` in length
    fn decode_samples(&self, buf: &[u8]) -> [[u8; 3]; CHANNELS] {
        let mut sample_data = [[0; 3]; CHANNELS];
        for channel in 0..CHANNELS {
            let word_idx = channel * self.word_len;
            match self.word_packing {
                WordLength::Bits16 => sample_data[channel] = [buf[word_idx], buf[word_idx + 1], 0],
                WordLength::Bits24 | WordLength::Bits32Zero => {
                    sample_data[channel].copy_from_slice(&buf[word_idx..word_idx + 3])
                }
                WordLength::Bits32Signed => {
                    sample_data[channel].copy_from_slice(&buf[word_idx + 1..word_idx + 4])
                }
            };
        }

        sample_data
    }
}

#[doc(hidden)]
pub trait RegisterAccess<E> {
    fn read_reg(&mut self, address: u6) -> Result<[u8; 2], Error<E>>;
    fn write_reg(&mut self, address: u6, bytes: [u8; 2]) -> Result<(), Error<E>>;
}

impl<I, E, W, const C: usize> RegisterAccess<E> for Ads131m<I, E, W, C>
where
    I: Interface<E, W>,
    W: Copy,
{
    fn read_reg(&mut self, address: u6) -> Result<[u8; 2], Error<E>> {
        unimplemented!()
    }

    fn write_reg(&mut self, address: u6, bytes: [u8; 2]) -> Result<(), Error<E>> {
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

impl_model!(ads131m02, 2, [AdcChannel0, AdcChannel1]);
impl_model!(ads131m03, 3, [AdcChannel0, AdcChannel1, AdcChannel2]);
impl_model!(
    ads131m04,
    4,
    [AdcChannel0, AdcChannel1, AdcChannel2, AdcChannel3]
);
impl_model!(
    ads131m06,
    6,
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
    ads131m08,
    8,
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

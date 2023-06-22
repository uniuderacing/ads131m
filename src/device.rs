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
use ringbuffer::{ConstGenericRingBuffer, RingBuffer, RingBufferRead, RingBufferWrite};
use static_assertions::const_assert_eq;
use ux::i24;

// Since no const-generic math, we have to use the max possible buffer size
// 32-bit words (4 bytes) * ( 1 response word + 8 channel data + 1 CRC word)
const BUF_SIZE: usize = 4 * (1 + 8 + 1);

// Maybe some fancy const stuff can be done instead of this at some point.
// Just having a channel_idx input would not be checked at compile time
#[rustfmt::skip]
macro_rules! impl_channel {
    ($trait_name:ident, $channel_name:ident, $channel_num:literal) => {
        #[doc=concat!("Methods for configuring ADC channel ", $channel_num)]
        pub trait $trait_name<I>
        where
            Self: RegisterAccess,
        {
            concat_idents!(get_channel_config = get_, $channel_name, _config, {
                #[doc=concat!(
                    "Read the `CH",
                    $channel_num,
                    "_CFG` register",
                    "\n\n",
                    "# Errors\n",
                    "Returns `Err` if there was an error during SPI communication"
                )]
                fn get_channel_config(&mut self) -> Result<ChannelConfig, Error> {
                    let mut bytes = [0u8; 2];
                    self.read_regs(registers::CHANNEL_CONFIG_ADDRS[$channel_num], &mut bytes)?;
                    Ok(ChannelConfig::from_be_bytes(bytes))
                }
            });

            concat_idents!(set_channel_config = set_, $channel_name, _config, {
                #[doc=concat!(
                    "Write to the `CH",
                    $channel_num,
                    "_CFG` register",
                    "\n\n",
                    "# Errors\n",
                    "Returns `Err` if there was an error during SPI communication"
                )]
                fn set_channel_config(&mut self, config: ChannelConfig) -> Result<(), Error> {
                    self.write_all_regs(registers::CHANNEL_CONFIG_ADDRS[$channel_num], &config.to_be_bytes())
                }
            });

            concat_idents!(get_channel_offset_cal = get_, $channel_name, _offset_cal, {
                #[doc=concat!(
                    "Read the `CH",
                    $channel_num,
                    "_OCAL_MSB` and `CH",
                    $channel_num,
                    "_OCAL_LSB` registers",
                    "\n\n",
                    "# Errors\n",
                    "Returns `Err` if there was an error during SPI communication"
                )]
                fn get_channel_offset_cal(&mut self) -> Result<OffsetCal, Error> {
                    // Ensure registers are contiguous
                    const_assert_eq!(
                        registers::CHANNEL_OCAL_MSB_ADDRS[$channel_num] + 1,
                        registers::CHANNEL_OCAL_LSB_ADDRS[$channel_num]
                    );
                    // This macro doesn't "use" these variables, so we need to fake it
                    let _ = registers::CHANNEL_OCAL_LSB_ADDRS[$channel_num];

                    let mut bytes = [0u8; 4];
                    self.read_regs(registers::CHANNEL_OCAL_MSB_ADDRS[$channel_num], &mut bytes)?;
                    Ok(OffsetCal::from_be_bytes(bytes))
                }
            });

            concat_idents!(set_channel_offset_cal = set_, $channel_name, _offset_cal, {
                #[doc=concat!(
                    "Write to the `CH",
                    $channel_num,
                    "_OCAL_MSB` and `CH",
                    $channel_num,
                    "_OCAL_LSB` registers",
                    "\n\n",
                    "# Errors\n",
                    "Returns `Err` if there was an error during SPI communication"
                )]
                fn set_channel_offset_cal(&mut self, offset_cal: OffsetCal) -> Result<(), Error> {
                    // Ensure registers are contiguous
                    const_assert_eq!(
                        registers::CHANNEL_OCAL_MSB_ADDRS[$channel_num] + 1,
                        registers::CHANNEL_OCAL_LSB_ADDRS[$channel_num]
                    );
                    // This macro doesn't "use" these variables, so we need to fake it
                    let _ = registers::CHANNEL_OCAL_LSB_ADDRS[$channel_num];

                    self.write_all_regs(registers::CHANNEL_OCAL_MSB_ADDRS[$channel_num], &offset_cal.to_be_bytes())
                }
            });

            concat_idents!(get_channel_gain_cal = get_, $channel_name, _gain_cal, {
                #[doc=concat!(
                    "Read the `CH",
                    $channel_num,
                    "_GCAL_MSB` and `CH",
                    $channel_num,
                    "_GCAL_LSB` registers",
                    "\n\n",
                    "# Errors\n",
                    "Returns `Err` if there was an error during SPI communication"
                )]
                fn get_channel_gain_cal(&mut self) -> Result<GainCal, Error> {
                    // Ensure registers are contiguous
                    const_assert_eq!(
                        registers::CHANNEL_GCAL_MSB_ADDRS[$channel_num] + 1,
                        registers::CHANNEL_GCAL_LSB_ADDRS[$channel_num]
                    );
                    // This macro doesn't "use" these variables, so we need to fake it
                    let _ = registers::CHANNEL_GCAL_LSB_ADDRS[$channel_num];

                    let mut bytes = [0u8; 4];
                    self.read_regs(registers::CHANNEL_GCAL_MSB_ADDRS[$channel_num], &mut bytes)?;
                    Ok(GainCal::from_be_bytes(bytes))
                }
            });

            concat_idents!(set_channel_gain_cal = set_, $channel_name, _gain_cal, {
                #[doc=concat!(
                    "Write to the `CH",
                    $channel_num,
                    "_GCAL_MSB` and `CH",
                    $channel_num,
                    "_GCAL_LSB` registers",
                    "\n\n",
                    "# Errors\n",
                    "Returns `Err` if there was an error during SPI communication"
                )]
                fn set_channel_gain_cal(&mut self, gain_cal: GainCal) -> Result<(), Error> {
                    // Ensure registers are contiguous
                    const_assert_eq!(
                        registers::CHANNEL_GCAL_MSB_ADDRS[$channel_num] + 1,
                        registers::CHANNEL_GCAL_LSB_ADDRS[$channel_num]
                    );
                    // This macro doesn't "use" these variables, so we need to fake it
                    let _ = registers::CHANNEL_GCAL_LSB_ADDRS[$channel_num];

                    self.write_all_regs(registers::CHANNEL_GCAL_MSB_ADDRS[$channel_num], &gain_cal.to_be_bytes())
                }
            });
        }
    };
}

macro_rules! impl_model {
    ($model:ident, $channel_count:literal, [$($channel:ident),+]) => {
        impl<I, W, const BUFSIZE: usize> Ads131m<I, W, BUFSIZE, $channel_count>
        where
            I: Interface<W>,
            W: Copy,
        {
            concat_idents!(open = open_, $model, {
                #[doc=concat!(
                    "Initialize a TI [`ADS131M0",
                    $channel_count,
                    "`] ADC driver from an [`embedded-hal`] SPI interface\n\n",
                    "# Notes\n",
                    "The SPI interface must be configured for SPI mode 1\n\n",
                    "The device must have it's `MODE` register in the default (reset) state\n\n",
                    "`BUFSIZE` should be a power of two, or performance could be significantly slower.\n\n",
                    "[`ADS131M0",
                    $channel_count,
                    "`]: https://www.ti.com/lit/ds/symlink/ads131m0",
                    $channel_count,
                    "\n",
                    "[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal"
                )]
                pub fn open(intf: I) -> Self {
                    Self::new(intf, Mode::default())
                }
            });
            concat_idents!(open_with_mode = open_, $model, _with_mode, {
                #[doc=concat!(
                    "Initialize a TI [`ADS131M0",
                    $channel_count,
                    "`] ADC driver from an [`embedded-hal`] SPI interface with a custom configuration\n\n",
                    "# Notes\n",
                    "The SPI interface must be configured for SPI mode 1\n\n",
                    "The current state of the device's `MODE` register must match the `mode` argument\n\n",
                    "`BUFSIZE` should be a power of two, or performance could be significantly slower.\n\n",
                    "[`ADS131M0",
                    $channel_count,
                    "`]: https://www.ti.com/lit/ds/symlink/ads131m0",
                    $channel_count,
                    "\n",
                    "[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal"
                )]
                pub fn open_with_mode(intf: I, mode: Mode) -> Self {
                    Self::new(intf, mode)
                }
            });
        }

        $(impl<I: Interface<W>, W: Copy, const BUFSIZE: usize> $channel<I> for Ads131m<I, W, BUFSIZE, $channel_count> {})+
    };
}

/// A single ADC sample grab for all channels
pub struct SampleGrab<const CHANNELS: usize> {
    data: [[u8; 3]; CHANNELS],
}

impl<const CHANNELS: usize> SampleGrab<CHANNELS> {
    /// Get a reference to the underlying sample bytes
    #[must_use]
    pub const fn as_bytes(&self) -> &[[u8; 3]; CHANNELS] {
        &self.data
    }

    /// Extract the underlying sample bytes
    #[must_use]
    pub const fn to_bytes(self) -> [[u8; 3]; CHANNELS] {
        self.data
    }

    /// Convert the sample data into signed integers between
    #[must_use]
    pub fn into_ints(self) -> [i24; CHANNELS] {
        let mut values = [i24::new(0); CHANNELS];
        for (idx, value) in values.iter_mut().enumerate() {
            let bytes = self.data[idx];
            if bytes[0] >= 0x80 {
                // Negative
                *value = i24::new(i32::from_be_bytes([0xFF, bytes[0], bytes[1], bytes[2]]));
            } else {
                // Positive
                *value = i24::new(i32::from_be_bytes([0, bytes[0], bytes[1], bytes[2]]));
            }
        }

        values
    }

    /// Convert the sample data into floating point numbers between -1 and 1
    #[must_use]
    pub fn into_floats(self) -> [f64; CHANNELS] {
        let mut values = [0.0; CHANNELS];
        for (idx, value) in values.iter_mut().enumerate() {
            let bytes = self.data[idx];
            if bytes[0] >= 0x80 {
                // Negative
                *value = f64::from(i32::from_be_bytes([0xFF, bytes[0], bytes[1], bytes[2]]))
                    / f64::from(i32::from(i24::MIN))
                    * -1.0;
            } else {
                // Positive
                *value = f64::from(i32::from_be_bytes([0, bytes[0], bytes[1], bytes[2]]))
                    / f64::from(i32::from(i24::MAX));
            }
        }

        values
    }
}

/// Driver
///
/// TODO: Description
///
/// TODO: Examples
pub struct Ads131m<I: Interface<W>, W: Copy, const BUFSIZE: usize, const CHANNELS: usize> {
    // Comms
    intf: I,
    read_buf: [u8; BUF_SIZE],
    write_buf: [u8; BUF_SIZE],
    // Samples
    sample_buf: ConstGenericRingBuffer<SampleGrab<CHANNELS>, BUFSIZE>,
    sample_drop_count: usize,
    // Mode
    crc_table: Crc<u16>,
    word_len: usize,
    word_packing: WordLength,
    spi_crc_enable: bool,
    //Type stuff
    w: PhantomData<W>,
}

impl<I, W, const BUFSIZE: usize, const CHANNELS: usize> Ads131m<I, W, BUFSIZE, CHANNELS>
where
    I: Interface<W>,
    W: Copy,
{
    /// Get the number of sample grabs available
    pub fn sample_grab_count(&self) -> usize {
        self.sample_buf.len()
    }

    /// Get the number of sample grabs that overflowed the sample buffer
    pub const fn sample_grab_overflow_count(&self) -> usize {
        self.sample_drop_count
    }

    /// Reset the count of sample grabs that overflowed the sample buffer
    pub fn reset_grab_overflow_count(&mut self) {
        self.sample_drop_count = 0;
    }

    /// Get a sample grab from the buffer
    pub fn dequeue_sample_grab(&mut self) -> Option<SampleGrab<CHANNELS>> {
        self.sample_buf.dequeue()
    }

    /// Reset the device to it's default state.
    ///
    /// The device must not be used again for 5us while the registers stabilize
    ///
    /// This also updates the internal cache of the device mode, which can change how the driver communicates
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn reset(&mut self) -> Result<(), Error> {
        let _ = self.send_simple_command(Command::Reset)?;
        self.process_new_mode(Mode::default());
        Ok(())
    }

    /// Place the device in a low power standby mode
    /// This disables all device channels and powers down non essential circuitry
    /// The sample clock should be disabled after this command to maximize power savings
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn standby(&mut self) -> Result<(), Error> {
        let _ = self.send_simple_command(Command::Standby)?;
        let resp = self.send_simple_command(Command::Null)?;
        match Response::try_from_be_bytes(resp) {
            Some(Response::Standby) => Ok(()),
            _ => Err(Error::UnexpectedResponse(Some(resp))),
        }
    }

    /// Wake the device from standby mode
    /// The sample clock should be enables after this command
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn wakeup(&mut self) -> Result<(), Error> {
        let _ = self.send_simple_command(Command::Wakeup)?;
        let resp = self.send_simple_command(Command::Null)?;
        match Response::try_from_be_bytes(resp) {
            Some(Response::Wakeup) => Ok(()),
            _ => Err(Error::UnexpectedResponse(Some(resp))),
        }
    }

    /// Lock the device to only respond to the Null, Read Register, and Unlock commands
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn lock(&mut self) -> Result<(), Error> {
        let _ = self.send_simple_command(Command::Lock)?;
        let resp = self.send_simple_command(Command::Null)?;
        match Response::try_from_be_bytes(resp) {
            Some(Response::Lock) => Ok(()),
            _ => Err(Error::UnexpectedResponse(Some(resp))),
        }
    }

    /// Unlock the device if it was locked
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn unlock(&mut self) -> Result<(), Error> {
        let _ = self.send_simple_command(Command::Unlock)?;
        let resp = self.send_simple_command(Command::Null)?;
        match Response::try_from_be_bytes(resp) {
            Some(Response::Unlock) => Ok(()),
            _ => Err(Error::UnexpectedResponse(Some(resp))),
        }
    }

    /// Read the `ID` register
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn get_id(&mut self) -> Result<Id, Error> {
        let mut bytes = [0u8; 2];
        self.read_regs(registers::ID_ADDR, &mut bytes)?;
        Ok(Id::from_be_bytes(bytes))
    }

    /// Read the `STATUS` register
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn get_status(&mut self) -> Result<Status, Error> {
        let mut bytes = [0u8; 2];
        self.read_regs(registers::STATUS_ADDR, &mut bytes)?;
        Ok(Status::from_be_bytes(bytes))
    }

    /// Read the `MODE` register
    ///
    /// This also updates the internal cache of the device mode, which can change how the driver communicates
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn get_mode(&mut self) -> Result<Mode, Error> {
        let mut bytes = [0u8; 2];
        self.read_regs(registers::MODE_ADDR, &mut bytes)?;
        let mode = Mode::from_be_bytes(bytes);

        self.process_new_mode(mode);
        Ok(mode)
    }

    /// Write to the `MODE` register
    ///
    /// This also updates the internal cache of the device mode, which can change how the driver communicates
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error> {
        self.write_all_regs(registers::MODE_ADDR, &mode.to_be_bytes())?;

        self.process_new_mode(mode);
        Ok(())
    }

    /// Read the `CLOCK` register
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn get_clock(&mut self) -> Result<Clock, Error> {
        let mut bytes = [0u8; 2];
        self.read_regs(registers::CLOCK_ADDR, &mut bytes)?;
        Ok(Clock::from_be_bytes(bytes))
    }

    /// Write to the `CLOCK` register
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn set_clock(&mut self, clock: Clock) -> Result<(), Error> {
        self.write_all_regs(registers::CLOCK_ADDR, &clock.to_be_bytes())
    }

    /// Read the GAIN1 register
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn get_gain(&mut self) -> Result<Gain, Error> {
        let mut bytes = [0u8; 2];
        self.read_regs(registers::GAIN_ADDR, &mut bytes)?;
        Ok(Gain::from_be_bytes(bytes))
    }

    /// Write to the `GAIN1` register
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn set_gain(&mut self, gain: Gain) -> Result<(), Error> {
        self.write_all_regs(registers::GAIN_ADDR, &gain.to_be_bytes())
    }

    /// Read the `CFG` register
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn get_config(&mut self) -> Result<Config, Error> {
        let mut bytes = [0u8; 2];
        self.read_regs(registers::CFG_ADDR, &mut bytes)?;
        Ok(Config::from_be_bytes(bytes))
    }

    /// Write to the `CFG` register
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn set_config(&mut self, gain: Config) -> Result<(), Error> {
        self.write_all_regs(registers::CFG_ADDR, &gain.to_be_bytes())
    }

    /// Read the `THRSHLD_MSB` and `THRSHLD_LSB` registers
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn get_threshold(&mut self) -> Result<Threshold, Error> {
        // Ensure registers are contiguous
        const_assert_eq!(registers::THRSHLD_MSB_ADDR + 1, registers::THRSHLD_LSB_ADDR);
        // This macro doesn't "use" these variables, so we need to fake it
        let _ = registers::THRSHLD_LSB_ADDR;

        let mut bytes = [0u8; 4];
        self.read_regs(registers::THRSHLD_MSB_ADDR, &mut bytes)?;
        Ok(Threshold::from_be_bytes(bytes))
    }

    /// Write to the `THRSHLD_MSB` and `THRSHLD_LSB` registers
    ///
    /// # Errors
    /// Returns `Err` if there was an error during SPI communication
    pub fn set_threshold(&mut self, threshold: Threshold) -> Result<(), Error> {
        // Ensure registers are contiguous
        const_assert_eq!(registers::THRSHLD_MSB_ADDR + 1, registers::THRSHLD_LSB_ADDR);
        // This macro doesn't "use" these variables, so we need to fake it
        let _ = registers::THRSHLD_LSB_ADDR;

        self.write_all_regs(registers::THRSHLD_MSB_ADDR, &threshold.to_be_bytes())
    }

    fn new(intf: I, mode: Mode) -> Self {
        let mut driver = Self {
            intf,
            read_buf: [0; BUF_SIZE],
            write_buf: [0; BUF_SIZE],
            sample_buf: ConstGenericRingBuffer::new(),
            sample_drop_count: 0,
            crc_table: Crc::<u16>::new(&CRC_16_CMS),
            word_len: 2,
            word_packing: WordLength::Bits16,
            spi_crc_enable: false,
            w: PhantomData,
        };
        driver.process_new_mode(mode);

        driver
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

    #[allow(clippy::identity_op)]
    fn send_simple_command(&mut self, command: Command) -> Result<[u8; 2], Error> {
        let mut bytes_written = 0;

        // Commands are always 2 bytes
        self.write_buf[0..2].copy_from_slice(&command.to_be_bytes());
        bytes_written += 2;

        while bytes_written < self.word_len {
            self.write_buf[bytes_written] = 0;
            bytes_written += 1;
        }

        let read_len = self.word_len * (1 + CHANNELS);
        self.transfer_frame(bytes_written, read_len, command != Command::Reset)?;

        // Responses are always 2 bytes
        let response: [u8; 2] = self.read_buf[..2].try_into().unwrap();
        let samples =
            self.decode_samples(&self.read_buf[self.word_len * 1..self.word_len * (CHANNELS + 1)]);
        self.enqueue_sample_grab(samples);

        Ok(response)
    }

    /// Exchange a single SPI frame with the ADC
    /// Panics if `command_len` or `resp_len` is greater than `BUF_SIZE` - `self.word_len`
    fn transfer_frame(
        &mut self,
        command_len: usize,
        read_len: usize,
        check_resp_crc: bool,
    ) -> Result<(), Error> {
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

    /// Panics if `buf` is not `self.word_len` * `CHANNELS` in length
    fn decode_samples(&self, buf: &[u8]) -> SampleGrab<CHANNELS> {
        let mut data = [[0; 3]; CHANNELS];
        for (channel, sample) in data.iter_mut().enumerate() {
            let word_idx = channel * self.word_len;
            match self.word_packing {
                WordLength::Bits16 => {
                    sample.copy_from_slice(&[buf[word_idx], buf[word_idx + 1], 0]);
                }
                WordLength::Bits24 | WordLength::Bits32Zero => {
                    sample.copy_from_slice(&buf[word_idx..word_idx + 3]);
                }
                WordLength::Bits32Signed => {
                    sample.copy_from_slice(&buf[word_idx + 1..word_idx + 4]);
                }
            };
        }

        SampleGrab { data }
    }

    fn enqueue_sample_grab(&mut self, sample: SampleGrab<CHANNELS>) {
        if self.sample_buf.is_full() {
            self.sample_drop_count = self.sample_drop_count.saturating_add(1);
        }
        self.sample_buf.enqueue(sample);
    }
}

#[doc(hidden)]
pub trait RegisterAccess {
    /// Read some device registers
    /// Currently only reading 1 or 2 registers is supported
    /// Panics if `buf` is not 2 or 4 in length
    fn read_regs(&mut self, address: u8, buf: &mut [u8]) -> Result<(), Error>;
    /// Write some device registers
    /// Currently only writing 1 or 2 registers is supported
    /// Returns the number of registers actually written
    /// Panics if `buf` is not 2 or 4 in length
    fn write_regs(&mut self, address: u8, buf: &[u8]) -> Result<usize, Error>;
    /// Write the entire buf to device registers
    /// Currently only writing 1 or 2 registers is supported
    /// Panics if `buf` is not 2 or 4 in length
    fn write_all_regs(&mut self, address: u8, buf: &[u8]) -> Result<(), Error>;
}

impl<I, W, const BUFSIZE: usize, const CHANNELS: usize> RegisterAccess
    for Ads131m<I, W, BUFSIZE, CHANNELS>
where
    I: Interface<W>,
    W: Copy,
{
    #[allow(clippy::cast_possible_truncation, clippy::identity_op)]
    fn read_regs(&mut self, address: u8, buf: &mut [u8]) -> Result<(), Error> {
        let reg_count = buf.len() / 2;
        assert!(reg_count > 0 && reg_count < 3);

        let mut bytes_written;

        // Send command
        let request_bytes = [
            0xA0 | address >> 1,
            (address & 0b1) << 7 | ((reg_count - 1) & 0x7F) as u8,
        ];
        self.write_buf[0..2].copy_from_slice(&request_bytes);
        bytes_written = 2;

        while bytes_written < self.word_len {
            self.write_buf[bytes_written] = 0;
            bytes_written += 1;
        }

        let read_len = self.word_len * (1 + CHANNELS);
        self.transfer_frame(bytes_written, read_len, true)?;

        // Ignore response
        let samples =
            self.decode_samples(&self.read_buf[self.word_len * 1..self.word_len * (CHANNELS + 1)]);
        self.enqueue_sample_grab(samples);

        // Fetch data
        self.write_buf[0..2].copy_from_slice(&Command::Null.to_be_bytes());
        bytes_written = 2;

        while bytes_written < self.word_len {
            self.write_buf[bytes_written] = 0;
            bytes_written += 1;
        }

        let read_len = match reg_count {
            1 => self.word_len * (1 + CHANNELS),
            n => self.word_len * (n / 2),
        };

        self.transfer_frame(bytes_written, read_len, true)?;

        match reg_count {
            1 => {
                buf[0..2].copy_from_slice(&self.read_buf[0..2]);
                let samples = self.decode_samples(
                    &self.read_buf[self.word_len * 1..self.word_len * (CHANNELS + 1)],
                );
                self.enqueue_sample_grab(samples);
            }
            n => {
                if self.read_buf[0] != request_bytes[0] | 0x40
                    || self.read_buf[1] != request_bytes[1]
                {
                    return Err(Error::UnexpectedResponse(Some([
                        self.read_buf[0],
                        self.read_buf[1],
                    ])));
                }

                for i in 0..n {
                    buf[i * 2..(i + 1) * 2].copy_from_slice(
                        &self.read_buf[self.word_len * i..self.word_len * (i + 1)],
                    );
                    // No samples returned
                }
            }
        }

        Ok(())
    }

    // Returns the number of registers actually written
    #[allow(clippy::cast_possible_truncation, clippy::identity_op)]
    fn write_regs(&mut self, address: u8, buf: &[u8]) -> Result<usize, Error> {
        let reg_count = buf.len() / 2;
        assert!(reg_count > 0 && reg_count < 3);

        let mut bytes_written;

        // Send command
        let request_bytes = [
            0x60 | address >> 1,
            (address & 0b1) << 7 | ((reg_count - 1) & 0x7F) as u8,
        ];
        self.write_buf[0..2].copy_from_slice(&request_bytes);
        bytes_written = 2;

        while bytes_written < self.word_len {
            self.write_buf[bytes_written] = 0;
            bytes_written += 1;
        }

        for i in 0..reg_count {
            self.write_buf[bytes_written..bytes_written + 2]
                .copy_from_slice(&buf[i * 2..(i + 1) * 2]);
            bytes_written += 2;

            while bytes_written < (self.word_len * (1 + i + 1)) {
                self.write_buf[bytes_written] = 0;
                bytes_written += 1;
            }
        }

        let read_len = self.word_len * (1 + CHANNELS);
        self.transfer_frame(bytes_written, read_len, true)?;

        // Ignore response
        let samples =
            self.decode_samples(&self.read_buf[self.word_len * 1..self.word_len * (CHANNELS + 1)]);
        self.enqueue_sample_grab(samples);

        // Fetch data
        self.write_buf[0..2].copy_from_slice(&Command::Null.to_be_bytes());
        bytes_written = 2;

        while bytes_written < self.word_len {
            self.write_buf[bytes_written] = 0;
            bytes_written += 1;
        }

        let read_len = (1 + CHANNELS) * self.word_len;
        self.transfer_frame(bytes_written, read_len, true)?;

        let samples =
            self.decode_samples(&self.read_buf[self.word_len..self.word_len * (CHANNELS + 1)]);
        self.enqueue_sample_grab(samples);
        let resp = &self.read_buf[0..2];

        if resp[0] != 0x40 | address >> 1 || resp[1] & 0x80 != address << 7 {
            return Err(Error::UnexpectedResponse(Some(resp.try_into().unwrap())));
        }

        Ok(usize::from(resp[1] & 0x7F))
    }

    fn write_all_regs(&mut self, address: u8, buf: &[u8]) -> Result<(), Error> {
        let reg_count = buf.len() / 2;
        assert!(reg_count > 0 && reg_count < 3);

        let mut regs_written = 0;
        while regs_written < reg_count {
            regs_written += self.write_regs(address, &buf[regs_written * 2..])?;
        }

        Ok(())
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

#[cfg(test)]
mod tests {
    use super::*;
    use float_cmp::assert_approx_eq;

    #[test]
    fn sample_to_ints() {
        assert_eq!(
            SampleGrab {
                data: [[0x7F, 0xFF, 0xFF]]
            }
            .into_ints(),
            [i24::new(8_388_607)]
        );
        assert_eq!(
            SampleGrab {
                data: [[0x00, 0x00, 0x01]]
            }
            .into_ints(),
            [i24::new(1)]
        );
        assert_eq!(
            SampleGrab {
                data: [[0x00, 0x00, 0x00]]
            }
            .into_ints(),
            [i24::new(0)]
        );
        assert_eq!(
            SampleGrab {
                data: [[0xFF, 0xFF, 0xFF]]
            }
            .into_ints(),
            [i24::new(-1)]
        );
        assert_eq!(
            SampleGrab {
                data: [[0x80, 0x00, 0x00]]
            }
            .into_ints(),
            [i24::new(-8_388_608)]
        );
    }

    #[test]
    fn sample_to_floats() {
        assert_approx_eq!(
            f64,
            SampleGrab {
                data: [[0x7F, 0xFF, 0xFF]]
            }
            .into_floats()[0],
            1.0,
            epsilon = 1e-8
        );
        assert_approx_eq!(
            f64,
            SampleGrab {
                data: [[0x00, 0x00, 0x01]]
            }
            .into_floats()[0],
            0.000_000_119_209,
            epsilon = 1e-8
        );
        assert_approx_eq!(
            f64,
            SampleGrab {
                data: [[0x00, 0x00, 0x00]]
            }
            .into_floats()[0],
            0.0,
            epsilon = 1e-8
        );
        assert_approx_eq!(
            f64,
            SampleGrab {
                data: [[0xFF, 0xFF, 0xFF]]
            }
            .into_floats()[0],
            -0.000_000_119_209,
            epsilon = 1e-8
        );
        assert_approx_eq!(
            f64,
            SampleGrab {
                data: [[0x80, 0x00, 0x00]]
            }
            .into_floats()[0],
            -1.0,
            epsilon = 1e-8
        );
    }
}

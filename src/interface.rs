//! Low level message interface for communicating with the device

use core::marker::PhantomData;

use crate::register::{
    Address, Channel, ChannelSpecific, CrcType, Global, Mode, Status, WordLength,
};
use crate::spi::Transfer;
use crate::Error;

use crc::{Crc, CRC_16_CMS, CRC_16_IBM_3740};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub use crate::sample_grab::SampleGrab;

// Since no const-generic math, we have to use the max possible buffer sizes
// Max word len * (command + 1 register writes + CRC)
const MAX_WRITE_LEN: usize = 4 * (1 + 1 + 1);

// The max read len is 2 words so that won't be longer than a normal command
// Max word len * (command + 8 channels + CRC)
const MAX_READ_LEN: usize = 4 * (1 + 8 + 1);

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
enum CommandKind {
    Null,
    Reset,
    Standby,
    Wakeup,
    Lock,
    Unlock,
    WriteRegister { addr: Address, data: [u8; 2] },
    ReadRegister { addr: Address },
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum ResponseKind {
    Null,
    Reset,
    Standby,
    Wakeup,
    Lock,
    Unlock,
    WriteRegister { addr: u8, count: u8 },
    ReadRegister { addr: Address },
}

impl ResponseKind {
    /// Try to decode a response from some bytes, returning `None` if the bytes do not match any response type
    ///
    /// # Note
    ///
    /// This will never return `ReadRegister` or `WriteRegister`
    /// as those cannot always be distinguished from other values
    fn try_decode_simple_response(bytes: [u8; 2]) -> Result<Self, Option<Status>> {
        match bytes {
            [0xFF, 0x24] => Ok(Self::Reset),
            [0x00, 0x22] => Ok(Self::Standby),
            [0x00, 0x33] => Ok(Self::Wakeup),
            [0x05, 0x55] => Ok(Self::Lock),
            [0x06, 0x55] => Ok(Self::Unlock),
            _ => Err(Self::try_decode_null_response(bytes)),
        }
    }

    /// Try to decode a status from some bytes, returning `None` if the bytes do not match a status message
    #[must_use]
    fn try_decode_null_response(bytes: [u8; 2]) -> Option<Status> {
        if (bytes[1] & 0xF0) == 0 {
            Some(Status::from_be_bytes(bytes))
        } else {
            None
        }
    }

    /// Try to decode a `WriteRegister`response from some bytes,
    /// returning `None` if the bytes do not represent a valid `WriteRegister`
    fn try_decode_write_register(bytes: [u8; 2]) -> Result<Self, Option<Status>> {
        if bytes[0] & 0xE0 != 0x40 {
            return Err(Self::try_decode_null_response(bytes));
        }

        Ok(Self::WriteRegister {
            addr: (bytes[0] & 0x1F) << 1 | bytes[1] >> 7,
            count: (bytes[1] & 0x7F) + 1,
        })
    }
}

struct ModeCache {
    crc_table: Crc<u16>,
    word_len: usize,
    word_packing: WordLength,
    spi_crc_enable: bool,
}

impl ModeCache {
    const fn new(mode: Mode) -> Self {
        let crc_alg = match mode.crc_type {
            CrcType::Ccitt => &CRC_16_IBM_3740,
            CrcType::Ansi => &CRC_16_CMS,
        };

        Self {
            crc_table: Crc::<u16>::new(crc_alg),
            word_len: mode.word_length.byte_count(),
            word_packing: mode.word_length,
            spi_crc_enable: mode.spi_crc_enable,
        }
    }

    fn update_word_length(&mut self, word_length: WordLength) {
        self.word_len = word_length.byte_count();
        self.word_packing = word_length;
    }
}

/// A command to the device
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Command {
    inner: CommandKind,
}

impl Command {
    /// No operation
    ///
    /// Send this command if you just want to receive a sample grab
    #[must_use]
    pub const fn new_null() -> Self {
        Self {
            inner: CommandKind::Null,
        }
    }

    /// Reset the device
    ///
    /// TODO: After this message is sent, this driver will not perform another transaction
    /// for 5us while the registers stabilize
    ///
    /// After this command is sent the internal mode cache will be updated,
    /// which can change how the driver communicates
    #[must_use]
    pub const fn new_reset() -> Self {
        Self {
            inner: CommandKind::Reset,
        }
    }

    /// Place the device in a low power standby mode
    /// This disables all device channels and powers down non essential circuitry
    /// The sample clock should be disabled after this command to maximize power savings
    #[must_use]
    pub const fn new_standby() -> Self {
        Self {
            inner: CommandKind::Standby,
        }
    }

    /// Wake the device from standby mode
    /// The sample clock should be enabled after this command
    #[must_use]
    pub const fn new_wakeup() -> Self {
        Self {
            inner: CommandKind::Wakeup,
        }
    }

    /// Lock lock the device to only respond to the `null`, `read_register`, and `unlock` commands
    #[must_use]
    pub const fn new_lock() -> Self {
        Self {
            inner: CommandKind::Lock,
        }
    }

    /// Unlock the device after it has been locked
    #[must_use]
    pub const fn new_unlock() -> Self {
        Self {
            inner: CommandKind::Unlock,
        }
    }

    /// Write to a global device register
    ///
    /// If the device's [`Mode`] register is written, the internal mode cache will be updated,
    /// which can change how the driver communicates
    #[must_use]
    #[allow(clippy::needless_pass_by_value)]
    pub fn new_write_global_register<R: Global>(register: R) -> Self {
        Self {
            inner: CommandKind::WriteRegister {
                addr: R::ADDRESS,
                data: register.to_be_bytes(),
            },
        }
    }

    /// Write to a channel-specific device register
    #[must_use]
    #[allow(clippy::needless_pass_by_value)]
    pub fn new_write_channel_register<R: ChannelSpecific>(register: R, channel: Channel) -> Self {
        Self {
            inner: CommandKind::WriteRegister {
                addr: R::address_for_channel(channel),
                data: register.to_be_bytes(),
            },
        }
    }

    /// Read from a device register
    #[must_use]
    pub const fn new_read_register(address: Address) -> Self {
        Self {
            inner: CommandKind::ReadRegister { addr: address },
        }
    }

    /// Return the channel for this command, if applicable
    const fn channel(self) -> Option<Channel> {
        match self.inner {
            CommandKind::ReadRegister { addr } | CommandKind::WriteRegister { addr, .. } => {
                addr.channel()
            }
            _ => None,
        }
    }

    /// Encode this command to `buf` and return the number of bytes written
    ///
    /// Words are MSB first
    ///
    /// This will pad the buffer with zeros to ensure the data written is a multiple of `word_len` in length
    ///
    /// # Panics
    ///
    /// Will panic if `buf` does not have room for the command.
    /// The maximum command length is 2 * `word_len`
    #[must_use]
    fn encode_words(self, buf: &mut [u8], word_len: usize) -> usize {
        let bytes = match self.inner {
            CommandKind::Null => [0x00, 0x00],
            CommandKind::Reset => [0x00, 0x11],
            CommandKind::Standby => [0x00, 0x22],
            CommandKind::Wakeup => [0x00, 0x33],
            CommandKind::Lock => [0x05, 0x55],
            CommandKind::Unlock => [0x06, 0x55],
            CommandKind::WriteRegister { addr, .. } => {
                let a = addr.address();
                [0x60 | a >> 1, (a & 0b1) << 7]
            }
            CommandKind::ReadRegister { addr } => {
                let a = addr.address();
                [0xA0 | a >> 1, (a & 0b1) << 7]
            }
        };

        buf[..2].copy_from_slice(&bytes);
        let mut len = 2;

        while len < word_len {
            buf[len] = 0;
            len += 1;
        }

        if let CommandKind::WriteRegister { data, .. } = self.inner {
            buf[len..len + 2].copy_from_slice(&data);
            len += 2;

            while len < word_len * 2 {
                buf[len] = 0;
                len += 1;
            }
        }

        len
    }

    /// Get the expected response to the command
    #[must_use]
    const fn expected_response(self) -> ResponseKind {
        match self.inner {
            CommandKind::Null => ResponseKind::Null,
            CommandKind::Reset => ResponseKind::Reset,
            CommandKind::Standby => ResponseKind::Standby,
            CommandKind::Wakeup => ResponseKind::Wakeup,
            CommandKind::Lock => ResponseKind::Lock,
            CommandKind::Unlock => ResponseKind::Unlock,
            CommandKind::WriteRegister { addr, .. } => ResponseKind::WriteRegister {
                addr: addr.address(),
                count: 1,
            },
            CommandKind::ReadRegister { addr } => ResponseKind::ReadRegister { addr },
        }
    }
}

/// The result of a register read
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RegisterData {
    /// The address that was read from
    pub address: Address,
    /// The data that was read
    pub data: [u8; 2],
}

/// A response from the ADC, usually containing a sample grab, a register read result or a a device status message
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[must_use = "this `Response` may contain a `SampleGrab` and a `RegisterData`, which should be used"]
pub struct Response<const CHANNELS: usize> {
    /// The returned sample grab, if any
    pub sample_grab: Option<SampleGrab<CHANNELS>>,
    /// The data read from a register, if a read was requested
    pub register_read: Option<RegisterData>,
    /// The current ADC status, if it was returned
    pub status: Option<Status>,
}

/// Low level device message interface
///
/// This handles encoding and decoding SPI frames based on the current device state,
/// but has no ability to determine when a a SPI transaction should occur.
///
///
/// TODO: Description
///
/// TODO: Examples
pub struct Ads131m<S: Transfer<W>, W: Copy, const CHANNELS: usize> {
    intf: S,
    _word: PhantomData<W>,
    read_buf: [u8; MAX_READ_LEN],
    write_buf: [u8; MAX_WRITE_LEN],
    expected_response: ResponseKind,
    mode_cache: ModeCache,
}

impl<S, W, const CHANNELS: usize> Ads131m<S, W, CHANNELS>
where
    S: Transfer<W>,
    W: Copy,
{
    /// Communicate with the device
    ///
    /// Commands must be frequently exchanged with the ADC to ensure sample grabs do not get dropped
    ///
    /// # Note
    /// Responses to commands are given on the following successful exchange,
    /// ***not*** in the exchange in which they were sent
    ///
    /// # Errors
    ///
    /// Will return `Err` if communication with the device failed
    pub fn communicate(&mut self, command: Command) -> Result<Response<CHANNELS>, Error> {
        let mut write_len = self.encode_command(command)?;

        // Get expected response for upcoming response, and store the new one
        let expected_response =
            core::mem::replace(&mut self.expected_response, command.expected_response());

        let read_len = match expected_response {
            ResponseKind::ReadRegister { .. } => 1 + 1,
            _ => 1 + CHANNELS + 1,
        } * self.mode_cache.word_len;

        // This should be fine as long as writes are shorter than reads
        if write_len % 2 == 1 {
            self.write_buf[write_len] = 0;
            write_len += 1;
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
            &self.write_buf[..write_len],
            &mut self.read_buf[..real_read_len],
        )?;

        let resp = self.decode_response(
            read_len,
            expected_response,
            command.inner == CommandKind::Reset,
        )?;

        match command.inner {
            CommandKind::WriteRegister {
                addr: Address::Mode,
                data,
            } => self.mode_cache = ModeCache::new(Mode::from_be_bytes(data)),
            CommandKind::Reset => self.mode_cache = ModeCache::new(Mode::default()),
            _ => {}
        }

        Ok(resp)
    }

    const fn new(intf: S, mode: Mode) -> Self {
        Self {
            intf,
            _word: PhantomData,
            read_buf: [0; MAX_READ_LEN],
            write_buf: [0; MAX_WRITE_LEN],
            expected_response: ResponseKind::Reset,
            mode_cache: ModeCache::new(mode),
        }
    }

    fn encode_command(&mut self, command: Command) -> Result<usize, Error> {
        if let Some(channel) = command.channel() {
            if usize::from(u8::from(channel)) > CHANNELS - 1 {
                return Err(Error::UnsupportedChannel);
            }
        }

        let cmd_len: usize = command.encode_words(&mut self.write_buf, self.mode_cache.word_len);
        let mut write_len = cmd_len;

        if self.mode_cache.spi_crc_enable {
            let write_crc = self
                .mode_cache
                .crc_table
                .checksum(&self.write_buf[..cmd_len]);

            self.write_buf[write_len..write_len + 2].copy_from_slice(&write_crc.to_be_bytes());
            write_len += 2;
        }

        while write_len < (cmd_len + self.mode_cache.word_len) {
            self.write_buf[write_len] = 0;
            write_len += 1;
        }

        Ok(write_len)
    }

    fn decode_response(
        &mut self,
        read_len: usize,
        kind: ResponseKind,
        reset_frame: bool,
    ) -> Result<Response<CHANNELS>, Error> {
        if !reset_frame {
            let crc_idx = read_len - self.mode_cache.word_len;

            // CRCs are always 2 bytes
            let resp_crc =
                u16::from_be_bytes(self.read_buf[crc_idx..crc_idx + 2].try_into().unwrap());
            let computed_crc = self
                .mode_cache
                .crc_table
                .checksum(&self.read_buf[..crc_idx]);
            if resp_crc != computed_crc {
                return Err(Error::ReceiveCrc {
                    computed: computed_crc,
                    received: resp_crc,
                });
            }
        }

        // TODO: Examine response byte spacing to detect a changing word_len

        let mut status = None;
        let mut register_read = None;

        let resp_bytes = self.read_buf[..2].try_into().unwrap();
        let resp = match kind {
            ResponseKind::ReadRegister { addr } => {
                register_read = Some(RegisterData {
                    address: addr,
                    data: resp_bytes,
                });
                // Nothing can be checked here
                Ok(ResponseKind::ReadRegister { addr })
            }
            ResponseKind::WriteRegister { .. } => {
                ResponseKind::try_decode_write_register(resp_bytes)
            }
            _ => ResponseKind::try_decode_simple_response(resp_bytes),
        };

        match resp {
            Ok(r) if r != kind => {
                return Err(Error::UnexpectedResponse);
            }
            Ok(_) => {}
            Err(Some(s)) => {
                if s.word_length != self.mode_cache.word_packing {
                    // Reset word length
                    self.mode_cache.update_word_length(s.word_length);
                    return Err(Error::WordLengthChanged);
                } else if s.spi_crc_err {
                    return Err(Error::SendCrc);
                } else if kind != ResponseKind::Null {
                    return Err(Error::UnexpectedResponse);
                }

                status = Some(s);
            }
            Err(None) => {
                return Err(Error::UnexpectedResponse);
            }
        }

        let sample_grab = if reset_frame {
            None
        } else {
            Some(self.decode_samples(
                &self.read_buf[self.mode_cache.word_len..self.mode_cache.word_len * (CHANNELS + 1)],
            ))
        };

        Ok(Response {
            sample_grab,
            register_read,
            status,
        })
    }

    /// Panics if `buf` is not `self.word_len` * `CHANNELS` in length
    fn decode_samples(&self, buf: &[u8]) -> SampleGrab<CHANNELS> {
        debug_assert!(buf.len() == self.mode_cache.word_len * CHANNELS);

        let mut data = [[0; 3]; CHANNELS];
        for (channel, sample) in data.iter_mut().enumerate() {
            let word_idx = channel * self.mode_cache.word_len;
            match self.mode_cache.word_packing {
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
}

impl<S, W> Ads131m<S, W, 2>
where
    S: Transfer<W>,
    W: Copy,
{
    /// Initialize an `ADS131M02` ADC driver
    ///
    /// The SPI interface must be configured for SPI mode 1 and the device must
    /// have it's `MODE` register in the default (reset) state in order for
    /// communications to work properly
    pub fn open_ads131m02(intf: S) -> Self {
        Self::new(intf, Mode::default())
    }

    /// Initialize an `ADS131M02` ADC driver with a custom initial state
    ///
    /// This does not configure the `MODE` register, instead this allows
    /// communication with a device already in a non-default communication configuration
    ///
    /// The SPI interface must be configured for SPI mode 1 in order for
    /// communications to work properly
    pub const fn open_ads131m02_with_mode(intf: S, mode: Mode) -> Self {
        Self::new(intf, mode)
    }
}

impl<S, W> Ads131m<S, W, 3>
where
    S: Transfer<W>,
    W: Copy,
{
    /// Initialize an `ADS131M03` ADC driver
    ///
    /// The SPI interface must be configured for SPI mode 1 and the device must
    /// have it's `MODE` register in the default (reset) state in order for
    /// communications to work properly
    pub fn open_ads131m03(intf: S) -> Self {
        Self::new(intf, Mode::default())
    }

    /// Initialize an `ADS131M03` ADC driver with a custom initial state
    ///
    /// This does not configure the `MODE` register, instead this allows
    /// communication with a device already in a non-default communication configuration
    ///
    /// The SPI interface must be configured for SPI mode 1 in order for
    /// communications to work properly
    pub const fn open_ads131m03_with_mode(intf: S, mode: Mode) -> Self {
        Self::new(intf, mode)
    }
}

impl<S, W> Ads131m<S, W, 4>
where
    S: Transfer<W>,
    W: Copy,
{
    /// Initialize an `ADS131M04` ADC driver
    ///
    /// The SPI interface must be configured for SPI mode 1 and the device must
    /// have it's `MODE` register in the default (reset) state in order for
    /// communications to work properly
    pub fn open_ads131m04(intf: S) -> Self {
        Self::new(intf, Mode::default())
    }

    /// Initialize an `ADS131M04` ADC driver with a custom initial state
    ///
    /// This does not configure the `MODE` register, instead this allows
    /// communication with a device already in a non-default communication configuration
    ///
    /// The SPI interface must be configured for SPI mode 1 in order for
    /// communications to work properly
    pub const fn open_ads131m04_with_mode(intf: S, mode: Mode) -> Self {
        Self::new(intf, mode)
    }
}

impl<S, W> Ads131m<S, W, 6>
where
    S: Transfer<W>,
    W: Copy,
{
    /// Initialize an `ADS131M06` ADC driver
    ///
    /// The SPI interface must be configured for SPI mode 1 and the device must
    /// have it's `MODE` register in the default (reset) state in order for
    /// communications to work properly
    pub fn open_ads131m06(intf: S) -> Self {
        Self::new(intf, Mode::default())
    }

    /// Initialize an `ADS131M06` ADC driver with a custom initial state
    ///
    /// This does not configure the `MODE` register, instead this allows
    /// communication with a device already in a non-default communication configuration
    ///
    /// The SPI interface must be configured for SPI mode 1 in order for
    /// communications to work properly
    pub const fn open_ads131m06_with_mode(intf: S, mode: Mode) -> Self {
        Self::new(intf, mode)
    }
}

impl<S, W> Ads131m<S, W, 8>
where
    S: Transfer<W>,
    W: Copy,
{
    /// Initialize an `ADS131M08` ADC driver
    ///
    /// The SPI interface must be configured for SPI mode 1 and the device must
    /// have it's `MODE` register in the default (reset) state in order for
    /// communications to work properly
    pub fn open_ads131m08(intf: S) -> Self {
        Self::new(intf, Mode::default())
    }

    /// Initialize an `ADS131M08` ADC driver with a custom initial state
    ///
    /// This does not configure the `MODE` register, instead this allows
    /// communication with a device already in a non-default communication configuration
    ///
    /// The SPI interface must be configured for SPI mode 1 in order for
    /// communications to work properly
    pub const fn open_ads131m08_with_mode(intf: S, mode: Mode) -> Self {
        Self::new(intf, mode)
    }
}

#[cfg(test)]
#[allow(clippy::too_many_lines)]
mod tests {
    use super::*;

    #[test]
    fn message_encode() {
        for (message, word_len, bytes, expected_len) in [
            (
                Command::new_null(),
                2,
                [0b0000_0000, 0b0000_0000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                2,
            ),
            (
                Command::new_reset(),
                2,
                [0b0000_0000, 0b0001_0001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                2,
            ),
            (
                Command::new_standby(),
                2,
                [0b0000_0000, 0b0010_0010, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                2,
            ),
            (
                Command::new_wakeup(),
                2,
                [0b0000_0000, 0b0011_0011, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                2,
            ),
            (
                Command::new_lock(),
                2,
                [0b0000_0101, 0b0101_0101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                2,
            ),
            (
                Command::new_unlock(),
                2,
                [0b0000_0110, 0b0101_0101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                2,
            ),
            (
                Command::new_unlock(),
                3,
                [0b0000_0110, 0b0101_0101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                3,
            ),
            (
                Command::new_unlock(),
                4,
                [0b0000_0110, 0b0101_0101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                4,
            ),
        ] {
            let mut buf = [0u8; 12];
            let len = message.encode_words(&mut buf, word_len);
            assert_eq!(len, expected_len);
            assert_eq!(buf, bytes);
        }
    }

    #[test]
    fn response_decode() {
        assert_eq!(
            ResponseKind::try_decode_simple_response([0b1111_1111, 0b0010_0100]),
            Ok(ResponseKind::Reset)
        );
        assert_eq!(
            ResponseKind::try_decode_simple_response([0b0000_0000, 0b0010_0010]),
            Ok(ResponseKind::Standby)
        );
        assert_eq!(
            ResponseKind::try_decode_simple_response([0b0000_0000, 0b0011_0011]),
            Ok(ResponseKind::Wakeup)
        );
        assert_eq!(
            ResponseKind::try_decode_simple_response([0b0000_0101, 0b0101_0101]),
            Ok(ResponseKind::Lock)
        );
        assert_eq!(
            ResponseKind::try_decode_simple_response([0b0000_0110, 0b0101_0101]),
            Ok(ResponseKind::Unlock)
        );
        assert_eq!(
            ResponseKind::try_decode_simple_response([0b0000_0101, 0b0000_0000]),
            Err(Some(Status {
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
                drdy3: false
            }))
        );
        assert_eq!(
            ResponseKind::try_decode_simple_response([0b1111_1111, 0b0000_1111]),
            Err(Some(Status {
                lock: true,
                resync: true,
                reg_map_crc_err: true,
                spi_crc_err: true,
                crc_type: CrcType::Ansi,
                reset: true,
                word_length: WordLength::Bits32Signed,
                drdy0: true,
                drdy1: true,
                drdy2: true,
                drdy3: true
            }))
        );
        assert_eq!(
            ResponseKind::try_decode_write_register([0b0100_0000, 0b0000_0000]),
            Ok(ResponseKind::WriteRegister { addr: 0, count: 1 })
        );
        assert_eq!(
            ResponseKind::try_decode_write_register([0b0101_0101, 0b0010_1010]),
            Ok(ResponseKind::WriteRegister {
                addr: 0x2A,
                count: 43
            })
        );
        assert_eq!(
            ResponseKind::try_decode_write_register([0b1111_1111, 0b0000_1111]),
            Err(Some(Status {
                lock: true,
                resync: true,
                reg_map_crc_err: true,
                spi_crc_err: true,
                crc_type: CrcType::Ansi,
                reset: true,
                word_length: WordLength::Bits32Signed,
                drdy0: true,
                drdy1: true,
                drdy2: true,
                drdy3: true
            }))
        );
        assert_eq!(
            ResponseKind::try_decode_simple_response([0b1111_1111, 0b0001_1111]),
            Err(None)
        );
        assert_eq!(
            ResponseKind::try_decode_simple_response([0b0000_0000, 0b0010_0000]),
            Err(None)
        );
    }
}

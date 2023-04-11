//! Device Driver

use core::marker::PhantomData;

use embedded_hal::spi::FullDuplex;
use nb::block;

use crate::config::Config;
use crate::Error;

/// Helper trait for SPI interfaces with different word lengths
pub trait Interface<W>
where
    Self: FullDuplex<W>,
{
    /// Number of bytes in a word
    const WORD_LEN: usize;

    /// Read and write some bytes on the SPI interface.
    ///
    /// The transaction will be the length of input or output, whichever is longer
    /// If input is shorter than output, the remaining bytes will be sent as zeros
    /// If output is shorter than input, the extra received bytes will be dropped
    ///
    /// Panics if either buffer is not a multiple of WORD_LEN in length
    fn transfer(&mut self, input: &[u8], output: &mut [u8]) -> Result<(), Error<Self::Error>> {
        assert!(input.len() % Self::WORD_LEN == 0);
        assert!(output.len() % Self::WORD_LEN == 0);

        let transfer_len = core::cmp::max(input.len(), output.len());
        let mut bytes_transferred = 0;

        while bytes_transferred < transfer_len {
            if bytes_transferred < input.len() {
                block!(self.send(Self::pack_word(
                    &input[transfer_len..(transfer_len + Self::WORD_LEN)]
                )))?;
            }

            if bytes_transferred < output.len() {
                Self::unpack_word(
                    block!(self.read())?,
                    &mut output[transfer_len..(transfer_len + Self::WORD_LEN)],
                );
            }

            bytes_transferred += Self::WORD_LEN;
        }

        Ok(())
    }

    /// Unpack a word into a buffer
    ///
    /// Panics if buf if not WORD_LEN in length
    fn unpack_word(word: W, buf: &mut [u8]);

    /// Pack a word from some bytes
    ///
    /// Panics if buf if not WORD_LEN in length
    fn pack_word(buf: &[u8]) -> W;
}

impl<T: FullDuplex<u16>> Interface<u16> for T {
    const WORD_LEN: usize = 2;

    #[inline]
    fn unpack_word(word: u16, buf: &mut [u8]) {
        buf.copy_from_slice(&word.to_be_bytes());
    }

    #[inline]
    fn pack_word(buf: &[u8]) -> u16 {
        u16::from_be_bytes(buf.try_into().unwrap())
    }
}

impl<T: FullDuplex<u8>> Interface<u8> for T {
    const WORD_LEN: usize = 1;

    #[inline]
    fn unpack_word(word: u8, buf: &mut [u8]) {
        buf[0] = word;
    }

    #[inline]
    fn pack_word(buf: &[u8]) -> u8 {
        buf[0]
    }
}

/// ADS131M ADC driver
#[derive(Debug)]
pub struct Ads131m<W, I> {
    intf: I,
    register_config: Config,
    w: PhantomData<W>,
}

impl<W, I> Ads131m<W, I>
where
    I: Interface<W>,
{
    /// Initialize an ADS131M driver from an [`embedded-hal`] SPI interface
    ///
    /// The SPI interface must be configured for SPI mode 1
    ///
    /// This command assumes the device is in it's default state
    ///
    /// [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
    pub fn open_default(intf: I) -> Result<Ads131m<W, I>, Error<I::Error>> {
        Self::open_with_config(intf, Config::default())
    }

    /// Initialize an ADS131M driver from an [`embedded-hal`] SPI interface with a custom configuration
    ///
    /// [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
    pub fn open_with_config(intf: I, config: Config) -> Result<Ads131m<W, I>, Error<I::Error>> {
        Ok(Ads131m {
            intf,
            register_config: config,
            w: PhantomData,
        })
    }
}

//! Helper trait for SPI interfaces with different word lengths

use embedded_hal::spi::FullDuplex;
use nb;

use crate::Error;

/// Trait for interacting with SPI interfaces with different word sizes
pub trait Interface<E, W: Copy> {
    /// Length of a word in bytes
    const WORD_LEN: usize;

    /// Read and write some bytes on the SPI interface
    ///
    /// The transaction will be the length of input or output, whichever is longer.
    /// If input is shorter than output, the remaining bytes will be sent as zeros.
    /// If output is shorter than input, the extra received bytes will be dropped.
    ///
    /// Both buffer lengths MUST be a multiple of two.
    ///
    /// Panics if either buffer length is not a multiple of two.
    fn transfer(&mut self, send: &[u8], receive: &mut [u8]) -> Result<(), Error<E>>;
}

/// Trait for interacting with an [`embedded-hal`] SPI interface
///
/// [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
pub trait WordTransfer<E, W: Copy> {
    /// Length of a word in bytes
    const WORD_LEN: usize;

    /// Attempt to read and decode a word to a buffer
    ///
    /// Note: A word must be written before calling this
    ///
    /// Panics if buf length is not `Self::WORD_LEN`
    fn read_word(&mut self, buf: &mut [u8]) -> nb::Result<(), E>;

    /// Encode a word from a buffer
    ///
    /// Panics if buf length is not `Self::WORD_LEN`
    fn encode_word(buf: &[u8]) -> W;

    /// Attempt to write a word
    fn write_word(&mut self, word: W) -> nb::Result<(), E>;
}

impl<T: WordTransfer<E, W>, E, W: Copy> Interface<E, W> for T {
    const WORD_LEN: usize = Self::WORD_LEN;

    fn transfer(&mut self, send: &[u8], receive: &mut [u8]) -> Result<(), Error<E>> {
        debug_assert!(send.len() % 2 == 0);
        debug_assert!(receive.len() % 2 == 0);

        let transfer_len = core::cmp::max(send.len(), receive.len());
        let mut bytes_read = 0;
        let mut bytes_written = 0;
        let mut word_cache: Option<W> = None;

        // Try to keep write buffer full
        while bytes_read < transfer_len || bytes_written < transfer_len {
            // Write first
            if bytes_written < transfer_len {
                // Cache the word that bytes_written currently points to
                let word = word_cache.take().unwrap_or_else(|| {
                    Self::encode_word(&send[bytes_written..(bytes_written + Self::WORD_LEN)])
                });

                match self.write_word(word) {
                    Ok(_) => bytes_written += Self::WORD_LEN,
                    Err(nb::Error::WouldBlock) => word_cache = Some(word),
                    Err(nb::Error::Other(e)) => return Err(Error::SpiError(e)),
                }
            }

            // Read only if more has been written
            if bytes_written > bytes_read && bytes_read < transfer_len {
                match self.read_word(&mut receive[bytes_read..(bytes_read + Self::WORD_LEN)]) {
                    Ok(_) => bytes_read += Self::WORD_LEN,
                    Err(nb::Error::WouldBlock) => {}
                    Err(nb::Error::Other(e)) => return Err(Error::SpiError(e)),
                }
            }
        }

        Ok(())
    }
}

impl<I: FullDuplex<u16, Error = E>, E> WordTransfer<E, u16> for I {
    const WORD_LEN: usize = 2;

    fn read_word(&mut self, buf: &mut [u8]) -> nb::Result<(), E> {
        let word = self.read()?;
        buf.copy_from_slice(&word.to_be_bytes());
        Ok(())
    }

    fn encode_word(buf: &[u8]) -> u16 {
        u16::from_be_bytes(buf.try_into().unwrap())
    }

    fn write_word(&mut self, word: u16) -> nb::Result<(), E> {
        self.send(word)
    }
}

impl<I: FullDuplex<u8, Error = E>, E> WordTransfer<E, u8> for I {
    const WORD_LEN: usize = 1;

    fn read_word(&mut self, buf: &mut [u8]) -> nb::Result<(), E> {
        buf[0] = self.read()?;
        Ok(())
    }

    fn encode_word(buf: &[u8]) -> u8 {
        buf[0]
    }

    fn write_word(&mut self, word: u8) -> nb::Result<(), E> {
        self.send(word)
    }
}

// TODO: DMA Interfaces

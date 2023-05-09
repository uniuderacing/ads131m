//! Helper trait for SPI interfaces with different word lengths

use embedded_hal::spi::FullDuplex;
use nb;

use crate::Error;

/// Helper trait for SPI interfaces with different word lengths
pub trait Interface<W>
where
    Self: FullDuplex<W>,
    W: From<u8> + Copy,
{
    /// Number of bytes in a word
    const WORD_LEN: usize;

    /// Read and write some bytes on the SPI interface
    ///
    /// The transaction will be the length of input or output, whichever is longer.
    /// If input is shorter than output, the remaining bytes will be sent as zeros.
    /// If output is shorter than input, the extra received bytes will be dropped.
    ///
    /// Panics if either buffer is not a multiple of `WORD_LEN` in length.
    fn transfer(&mut self, send: &[u8], receive: &mut [u8]) -> Result<(), Error<Self::Error>> {
        debug_assert!(send.len() % Self::WORD_LEN == 0);
        debug_assert!(receive.len() % Self::WORD_LEN == 0);

        let transfer_len = core::cmp::max(send.len(), receive.len());
        let mut bytes_read = 0;
        let mut bytes_written = 0;
        let mut word_cache = None;

        // Try to keep write buffer full
        while bytes_read < transfer_len || bytes_written < transfer_len {
            if bytes_written < transfer_len {
                if word_cache.is_none() {
                    if bytes_written < send.len() {
                        word_cache = Some(Self::pack_word(
                            &send[transfer_len..(transfer_len + Self::WORD_LEN)],
                        ));
                    } else {
                        word_cache = Some(0u8.into());
                    }
                }

                if let Some(w) = word_cache {
                    match self.send(w) {
                        Ok(_) => {
                            word_cache = None;
                            bytes_written += Self::WORD_LEN;
                        }
                        Err(nb::Error::WouldBlock) => {}
                        Err(nb::Error::Other(e)) => return Err(Error::SpiError(e)),
                    }
                }
            }

            if bytes_written > bytes_read && bytes_read < transfer_len {
                match self.read() {
                    Ok(w) => {
                        if bytes_read < receive.len() {
                            Self::unpack_word(
                                w,
                                &mut receive[transfer_len..(transfer_len + Self::WORD_LEN)],
                            );
                        }
                        bytes_read += Self::WORD_LEN;
                    }
                    Err(nb::Error::WouldBlock) => {}
                    Err(nb::Error::Other(e)) => return Err(Error::SpiError(e)),
                }
            }
        }

        Ok(())
    }

    /// Unpack a word into a buffer
    ///
    /// Panics if buf if not `WORD_LEN` in length
    fn unpack_word(word: W, buf: &mut [u8]);

    /// Pack a word from some bytes
    ///
    /// Panics if buf if not `WORD_LEN` in length
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

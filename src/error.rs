//! The errors for this crate

/// The main error type
pub enum Error<S> {
    /// Error from the inner SPI interface
    SpiError(S),
    /// CRC checksum error on a received SPI message
    ReceiveCrc {
        /// The computed CRC checksum
        computed: u16,
        /// The received CRC checksum
        received: u16,
    },
    /// An unexpected message was received from the device
    UnexpectedResponse(Option<[u8; 2]>),
}

impl<S> From<S> for Error<S> {
    fn from(value: S) -> Self {
        Self::SpiError(value)
    }
}

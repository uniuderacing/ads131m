//! The errors for this crate

/// The main error type
pub enum Error {
    /// Error from the SPI interface
    ///
    /// No samples are enqueued if this error occurs
    SpiIOError,
    /// Error from the DRDY pin
    ///
    /// No samples are enqueued if this error occurs
    DrdyIOError,
    /// CRC checksum error on a received SPI message
    ///
    /// No samples are enqueued if this error occurs
    ReceiveCrc {
        /// The computed CRC checksum
        computed: u16,
        /// The received CRC checksum
        received: u16,
    },
    /// An unexpected message was received from the device
    ///
    /// No samples are enqueued if this error occurs
    UnexpectedResponse,
    /// The SPI word length changed unexpectedly
    ///
    /// This can happen if changing the `MODE` register failed, or the device reset unexpectedly
    ///
    /// No samples are enqueued if this error occurs, but sample integrity may be questionable if the device did reset
    WordLengthChanged,
    /// An unsupported channel was specified in a command
    UnsupportedChannel,
}

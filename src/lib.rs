//! This is an [`embedded_hal`] driver for the Texas Instruments [`ADS131M`] series of simultaneously sampling
//! 24-bit delta-sigma analog-to-digital converters.
//!
//!
//! This driver allows you to:
//! - Nothing Yet
//!
//! ## The Devices
//!
//! Here are the different models of ADS131M ADCs:
//!
//! | Device    | Resolution | Max Sample Rate | Channels | Datasheet                                               |
//! |-----------|------------|-----------------|----------|---------------------------------------------------------|
//! | ADS131M02 | 24-bit     | 64 kSPS         | 2        | [Link](https://www.ti.com/lit/ds/symlink/ads131m02.pdf) |
//! | ADS131M03 | 24-bit     | 64 kSPS         | 3        | [Link](https://www.ti.com/lit/ds/symlink/ads131m03.pdf) |
//! | ADS131M04 | 24-bit     | 64 kSPS         | 4        | [Link](https://www.ti.com/lit/ds/symlink/ads131m04.pdf) |
//! | ADS131M06 | 24-bit     | 32 kSPS         | 6        | [Link](https://www.ti.com/lit/ds/symlink/ads131m06.pdf) |
//! | ADS131M08 | 24-bit     | 32 kSPS         | 8        | [Link](https://www.ti.com/lit/ds/symlink/ads131m08.pdf) |
//!
//! All of the models should work, however only the ADS131M04 has been tested
//!
//! ## Usage Examples
//!
//! TODO
//!
//! ```no_run
//! unimplemented!();
//! ```
//!
//!
//! [`ADS131M`]: https://www.ti.com/sitesearch/en-us/docs/universalsearch.tsp?searchTerm=ADS131M

#![deny(unsafe_code)]
#![warn(
    clippy::all,
    clippy::pedantic,
    clippy::cargo,
    clippy::nursery,
    missing_docs
)]
#![allow(clippy::multiple_crate_versions)] // TODO: Remove this once embedded-hal 1.0 drops
#![no_std]

pub mod interface;
pub mod register;
pub mod spi;

/// The main error type
pub enum Error {
    /// Error from the SPI interface
    SpiIOError,
    /// CRC checksum error on a received SPI message
    ReceiveCrc {
        /// The computed CRC checksum
        computed: u16,
        /// The received CRC checksum
        received: u16,
    },
    /// The device reported a CRC checksum error on a sent SPI message
    SendCrc,
    /// An unexpected message was received from the device
    UnexpectedResponse,
    /// The SPI word length changed unexpectedly
    ///
    /// This can happen if changing the `MODE` register failed, or the device reset unexpectedly
    WordLengthChanged,
    /// An unsupported channel was specified in a command
    UnsupportedChannel,
}

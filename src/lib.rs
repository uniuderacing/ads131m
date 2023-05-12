//! This is an [`embedded-hal`] Rust driver for the Texas Instruments [`ADS131M`] series of simultaneously sampling
//! 24-bit delta-sigma analog-to-digital converters.
//!
//! However, currently only the ADS131M04 is supported.
//!
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//! [`ADS131M`]: https://www.ti.com/sitesearch/en-us/docs/universalsearch.tsp?searchTerm=ADS131M
//!
//!
//! This driver allows you to:
//! - Nothing Yet
//!
//! ## The Devices
//!
//! TODO: Description
//!
//! Here are the different models of ADS131M ADCs:
//!
//! | Device    | Resolution | Max Sample Rate | Channels |
//! |-----------|------------|-----------------|----------|
//! | ADS131M02 | 24-bit     | 64 kSPS         | 2        |
//! | ADS131M03 | 24-bit     | 64 kSPS         | 3        |
//! | ADS131M04 | 24-bit     | 64 kSPS         | 4        |
//! | ADS131M06 | 24-bit     | 32 kSPS         | 6        |
//! | ADS131M08 | 24-bit     | 32 kSPS         | 8        |
//!
//! Datasheets:
//! - [ADS131M02](https://www.ti.com/lit/ds/symlink/ads131m02.pdf)
//! - [ADS131M03](https://www.ti.com/lit/ds/symlink/ads131m03.pdf)
//! - [ADS131M04](https://www.ti.com/lit/ds/symlink/ads131m04.pdf)
//! - [ADS131M06](https://www.ti.com/lit/ds/symlink/ads131m06.pdf)
//! - [ADS131M08](https://www.ti.com/lit/ds/symlink/ads131m08.pdf)
//!
//! ## Usage Examples
//!
//! TODO
//!
//! ```no_run
//! unimplemented!();
//! ```
//!

#![warn(
    clippy::all,
    clippy::pedantic,
    clippy::cargo,
    clippy::nursery,
    missing_docs
)]
#![allow(clippy::missing_errors_doc, clippy::similar_names)]
#![deny(unsafe_code)]
#![no_std]

pub mod device;
pub mod error;
pub mod interface;
pub mod types;

#[doc(hidden)]
pub mod ic;

pub use device::Ads131m;
pub use error::Error;
pub use interface::Interface;

//! The errors for this crate

/// The main error type
pub enum Error<S> {
    /// Error originating in the SPI interface
    SpiError(S),
}

impl<S> From<S> for Error<S> {
    fn from(value: S) -> Self {
        Self::SpiError(value)
    }
}

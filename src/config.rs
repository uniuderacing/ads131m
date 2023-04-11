//! Configuration parameters for a device config
//!
//! A config object represents the current settings of certain ADC registers

/// SPI Word size configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WordSize {
    /// 16-bit words
    Bits16,
    /// 24-bit words
    Bits24,
    /// 32-bit words
    Bits32,
}

/// Device configuration
///
/// This represents the current state of the registers
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// SPI word size configuration
    pub word_size: WordSize,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            word_size: WordSize::Bits24,
        }
    }
}

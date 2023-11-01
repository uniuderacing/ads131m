//! Basic integer types used by the ADC
//! TODO: Better doc

use core::fmt::{Debug, Formatter};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A signed 10-bit integer
///
/// This can be a value in the range `[-512, 511]`
#[derive(Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize), serde(transparent))]
#[allow(non_camel_case_types)]
pub struct i10(i16);

impl i10 {
    /// Minimum value
    pub const MIN: i16 = -(1 << (10 - 1));
    /// Maximum value
    pub const MAX: i16 = (1 << (10 - 1)) - 1;

    /// Try to create a new `i10`
    ///
    /// `Some` is returned if `value` is within `[-512, 511]`, otherwise `None` is returned
    #[must_use]
    pub const fn try_new(value: i16) -> Option<Self> {
        if value >= Self::MIN && value <= Self::MAX {
            Some(Self(value))
        } else {
            None
        }
    }

    /// Create a new `i10`, clamped to be within `[-512, 511]`
    #[must_use]
    pub const fn new_clamped(value: i16) -> Self {
        if value > Self::MAX {
            Self(Self::MAX)
        } else if value < Self::MIN {
            Self(Self::MIN)
        } else {
            Self(value)
        }
    }

    /// Create a new `i10` from the lower 10 bits of `value`
    ///
    /// Any value above the lower 10 bits is masked off
    #[must_use]
    #[allow(clippy::cast_possible_wrap, clippy::cast_sign_loss)]
    pub const fn new_masked(value: i16) -> Self {
        // Sign extend to chop off the top bits
        let value = (((value as u16) << (16 - 10)) as i16) >> (16 - 10);

        Self(value)
    }

    /// Get the value as an `i16`
    #[must_use]
    pub const fn get(&self) -> i16 {
        self.0
    }

    /// Construct a new `i10` from its representation as a byte array, ignoring the bits above 10 bits
    #[must_use]
    #[allow(clippy::cast_possible_wrap)]
    pub const fn from_be_bytes(bytes: [u8; 2]) -> Self {
        let unsigned = u16::from_be_bytes(bytes);
        let value = ((unsigned << (16 - 10)) as i16) >> (16 - 10);

        Self(value)
    }

    /// Return the representation of this `i10` as a byte array, leaving the bits above 10 bits as 0
    #[must_use]
    #[allow(clippy::missing_panics_doc)]
    pub const fn to_be_bytes(self) -> [u8; 2] {
        let mut bytes: [u8; 2] = self.0.to_be_bytes();
        bytes[0] &= 0x03;
        bytes
    }
}

impl Debug for i10 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        self.0.fmt(f)
    }
}

impl TryFrom<i16> for i10 {
    type Error = &'static str;

    fn try_from(value: i16) -> Result<Self, Self::Error> {
        Self::try_new(value).ok_or("value out of range")
    }
}

impl From<i10> for i16 {
    fn from(value: i10) -> Self {
        value.get()
    }
}

/// An unsigned 24-bit integer
///
/// This can be a value in the range 0 to 16,777,215
#[derive(Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize), serde(transparent))]
#[allow(non_camel_case_types)]
pub struct u24(u32);

impl u24 {
    /// Minimum value
    pub const MIN: u32 = 0;
    /// Maximum value
    pub const MAX: u32 = (1 << 24) - 1;

    /// Try to create a new `u24`
    ///
    /// `Some` is returned if `value` is within `[0, 16,777,215]`, otherwise `None` is returned
    #[must_use]
    pub const fn try_new(value: u32) -> Option<Self> {
        if value <= Self::MAX {
            Some(Self(value))
        } else {
            None
        }
    }

    /// Create a new `u24`, clamped to be within `[0, 16,777,215]`
    #[must_use]
    pub const fn new_clamped(value: u32) -> Self {
        if value <= Self::MAX {
            Self(value)
        } else {
            Self(Self::MAX)
        }
    }

    /// Create a new `u24` from the lower 24 bits of `value`
    ///
    /// Any value above the lower 24 bits is masked off
    #[must_use]
    pub const fn new_masked(value: u32) -> Self {
        Self(value & Self::MAX)
    }

    /// Get the value as a `u32`
    #[must_use]
    pub const fn get(&self) -> u32 {
        self.0
    }

    /// Construct a new `u24` from its representation as a byte array
    #[must_use]
    pub fn from_be_bytes(bytes: [u8; 3]) -> Self {
        let mut val_bytes = [0; 4];
        val_bytes[1..].copy_from_slice(&bytes);

        Self(u32::from_be_bytes(val_bytes))
    }

    /// Return the representation of this `u24` as a byte array
    #[must_use]
    #[allow(clippy::missing_panics_doc)]
    pub fn to_be_bytes(self) -> [u8; 3] {
        self.0.to_be_bytes()[1..].try_into().unwrap()
    }
}

impl Debug for u24 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        self.0.fmt(f)
    }
}

impl TryFrom<u32> for u24 {
    type Error = &'static str;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        Self::try_new(value).ok_or("value out of range")
    }
}

impl From<u24> for u32 {
    fn from(value: u24) -> Self {
        value.get()
    }
}

/// A signed 24-bit integer
///
/// This can be a value in the range -8,388,608 to 8,388,607
#[derive(Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize), serde(transparent))]
#[allow(non_camel_case_types)]
pub struct i24(i32);

impl i24 {
    /// Minimum value
    pub const MIN: i32 = -(1 << (24 - 1));
    /// Maximum value
    pub const MAX: i32 = (1 << (24 - 1)) - 1;

    /// Try to create a new `i24`
    ///
    /// `Some` is returned if `value` is within `[-8,388,608, 8,388,607]`, otherwise `None` is returned
    #[must_use]
    pub const fn try_new(value: i32) -> Option<Self> {
        if value >= Self::MIN && value <= Self::MAX {
            Some(Self(value))
        } else {
            None
        }
    }

    /// Create a new `i24`, clamped to be within `[-8,388,608, 8,388,607]`
    #[must_use]
    pub const fn new_clamped(value: i32) -> Self {
        if value > Self::MAX {
            Self(Self::MAX)
        } else if value < Self::MIN {
            Self(Self::MIN)
        } else {
            Self(value)
        }
    }

    /// Create a new `i24` from the lower 24 bits of `value`
    ///
    /// Any value above the lower 24 bits is masked off
    #[must_use]
    #[allow(clippy::cast_possible_wrap, clippy::cast_sign_loss)]
    pub const fn new_masked(value: i32) -> Self {
        // Sign extend to chop off the top bits
        let value = (((value as u32) << (32 - 24)) as i32) >> (32 - 24);

        Self(value)
    }

    /// Get the value as an `i32`
    #[must_use]
    pub const fn get(&self) -> i32 {
        self.0
    }

    /// Construct a new `i24` from its representation as a byte array
    #[must_use]
    #[allow(clippy::cast_possible_wrap)]
    pub fn from_be_bytes(bytes: [u8; 3]) -> Self {
        let mut value_bytes = [0; 4];
        value_bytes[1..].copy_from_slice(&bytes);

        let unsigned = u32::from_be_bytes(value_bytes);
        let value = ((unsigned << (32 - 24)) as i32) >> (32 - 24);

        Self(value)
    }

    /// Return the representation of this `i24` as a byte array
    #[must_use]
    #[allow(clippy::missing_panics_doc)]
    pub fn to_be_bytes(self) -> [u8; 3] {
        self.0.to_be_bytes()[1..].try_into().unwrap()
    }
}

impl Debug for i24 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        self.0.fmt(f)
    }
}

impl TryFrom<i32> for i24 {
    type Error = &'static str;

    fn try_from(value: i32) -> Result<Self, Self::Error> {
        Self::try_new(value).ok_or("value out of range")
    }
}

impl From<i24> for i32 {
    fn from(value: i24) -> Self {
        value.get()
    }
}

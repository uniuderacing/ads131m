use serde::de::{self, Visitor};
use serde::{Deserializer, Serializer};
use ux::{u10, u24, u4};

const EXPECTED_U4: &str = "an integer between -2^3 and 2^3";
const EXPECTED_U10: &str = "an integer between -2^9 and 2^9";
const EXPECTED_U24: &str = "an integer between -2^23 and 2^23";

struct U4Visitor;

impl<'de> Visitor<'de> for U4Visitor {
    type Value = u4;

    fn expecting(&self, formatter: &mut core::fmt::Formatter) -> core::fmt::Result {
        formatter.write_str(EXPECTED_U4)
    }

    fn visit_u8<E>(self, value: u8) -> Result<Self::Value, E>
    where
        E: de::Error,
    {
        u4::try_from(value).map_err(|_| {
            de::Error::invalid_value(de::Unexpected::Unsigned(value.into()), &EXPECTED_U4)
        })
    }
}

struct U10Visitor;

impl<'de> Visitor<'de> for U10Visitor {
    type Value = u10;

    fn expecting(&self, formatter: &mut core::fmt::Formatter) -> core::fmt::Result {
        formatter.write_str(EXPECTED_U10)
    }

    fn visit_u16<E>(self, value: u16) -> Result<Self::Value, E>
    where
        E: de::Error,
    {
        u10::try_from(value).map_err(|_| {
            de::Error::invalid_value(de::Unexpected::Unsigned(value.into()), &EXPECTED_U10)
        })
    }
}

struct U24Visitor;

impl<'de> Visitor<'de> for U24Visitor {
    type Value = u24;

    fn expecting(&self, formatter: &mut core::fmt::Formatter) -> core::fmt::Result {
        formatter.write_str(EXPECTED_U24)
    }

    fn visit_u32<E>(self, value: u32) -> Result<Self::Value, E>
    where
        E: de::Error,
    {
        u24::try_from(value).map_err(|_| {
            de::Error::invalid_value(de::Unexpected::Unsigned(value.into()), &EXPECTED_U24)
        })
    }
}

pub mod serde_u4 {
    use super::{u4, Deserializer, Serializer, U4Visitor};

    #[allow(clippy::trivially_copy_pass_by_ref)]
    pub fn serialize<S>(value: &u4, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_u8((*value).into())
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<u4, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_u8(U4Visitor)
    }
}

pub mod serde_u10 {
    use super::{u10, Deserializer, Serializer, U10Visitor};

    #[allow(clippy::trivially_copy_pass_by_ref)]
    pub fn serialize<S>(value: &u10, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_u16((*value).into())
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<u10, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_u16(U10Visitor)
    }
}

pub mod serde_u24 {
    use super::{u24, Deserializer, Serializer, U24Visitor};

    #[allow(clippy::trivially_copy_pass_by_ref)]
    pub fn serialize<S>(value: &u24, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_u32((*value).into())
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<u24, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_u32(U24Visitor)
    }
}

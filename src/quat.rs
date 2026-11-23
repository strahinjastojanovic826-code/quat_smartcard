use crate::error::QuatError;

/// Quaternary logic state represented in 2 bits:
/// - `ZeroZero` (`0b00` = 0)
/// - `ZeroOne`  (`0b01` = 1)
/// - `OneZero`  (`0b10` = 2)
/// - `OneOne`   (`0b11` = 3)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Quat {
    ZeroZero = 0b00,
    ZeroOne = 0b01,
    OneZero = 0b10,
    OneOne = 0b11,
}

impl Quat {
    /// Converts a raw `u8` (`0..=3`) into a validated `Quat`.
    #[inline]
    pub const fn from_u8(val: u8) -> Result<Self, QuatError> {
        match val {
            0b00 => Ok(Self::ZeroZero),
            0b01 => Ok(Self::ZeroOne),
            0b10 => Ok(Self::OneZero),
            0b11 => Ok(Self::OneOne),
            _ => Err(QuatError::InvalidQuatValue(val)),
        }
    }

    /// Converts a 2-bit pair `(msb, lsb)` where each bit is `0` or `1`.
    #[inline]
    pub const fn from_bit_pair(msb: u8, lsb: u8) -> Result<Self, QuatError> {
        if msb > 1 {
            return Err(QuatError::InvalidBitPair(msb, lsb));
        }
        if lsb > 1 {
            return Err(QuatError::InvalidBitPair(msb, lsb));
        }
        let combined = (msb << 1) | lsb;
        Self::from_u8(combined)
    }

    /// Returns the 2-bit pair `(msb, lsb)` as `(0 or 1, 0 or 1)`.
    #[inline]
    pub const fn to_bit_pair(self) -> (u8, u8) {
        let v = self as u8;
        ((v >> 1) & 1, v & 1)
    }

    /// Converts a 2-bit mask (`0..=3`) directly without error handling.
    #[inline]
    pub const fn from_bits_masked(bits: u8) -> Self {
        match bits & 0b11 {
            0b00 => Self::ZeroZero,
            0b01 => Self::ZeroOne,
            0b10 => Self::OneZero,
            _ => Self::OneOne,
        }
    }

    /// Parses an ASCII digit `'0'`, `'1'`, `'2'`, or `'3'`.
    #[inline]
    pub const fn from_ascii_digit(byte: u8) -> Result<Self, QuatError> {
        match byte {
            b'0' => Ok(Self::ZeroZero),
            b'1' => Ok(Self::ZeroOne),
            b'2' => Ok(Self::OneZero),
            b'3' => Ok(Self::OneOne),
            _ => Err(QuatError::InvalidAsciiDigit(byte)),
        }
    }

    /// Returns the 2-bit value (`0..=3`).
    #[inline]
    pub const fn bits(self) -> u8 {
        self as u8
    }

    /// Returns the ASCII digit representation (`b'0'..=b'3'`).
    #[inline]
    pub const fn to_ascii_digit(self) -> u8 {
        b'0' + (self as u8)
    }

    /// Computes exact EEPROM bytes required for `quat_count` symbols
    /// at 4 quaternary symbols (8 bits) per byte.
    #[inline]
    pub const fn packed_bytes_for_quats(quat_count: usize) -> usize {
        quat_count.div_ceil(4)
    }

    /// Computes memory saved (in bits) when storing `quat_count` symbols
    /// as 2-bit quats instead of 8-bit bytes (75% of 8 bits = 6 bits per symbol).
    #[inline]
    pub const fn bits_saved_vs_byte_per_symbol(quat_count: usize) -> usize {
        quat_count * 6
    }

    /// Computes memory saved (in bytes) when comparing 1 byte per symbol
    /// against 2 bits per symbol (75% reduction).
    #[inline]
    pub const fn bytes_saved_vs_byte_per_symbol(quat_count: usize) -> usize {
        quat_count - Self::packed_bytes_for_quats(quat_count)
    }
}
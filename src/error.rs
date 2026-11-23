#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuatError {
    /// Input value is outside the 2-bit quaternary domain (0..=3).
    InvalidQuatValue(u8),
    /// Bit pair contains values outside 0 or 1.
    InvalidBitPair(u8, u8),
    /// ASCII character is not one of '0', '1', '2', '3'.
    InvalidAsciiDigit(u8),
    /// Requested symbol index exceeds capacity.
    IndexOutOfBounds { index: usize, len: usize },
    /// Target EEPROM slice is too small to hold the packed quaternary stream.
    BufferTooSmall { required_bytes: usize, available_bytes: usize },
    /// Source or destination slice length does not match expected symbol count.
    InvalidLength { expected: usize, actual: usize },
    /// No active or writable wear-leveled slot is available.
    NoAvailableSlot,
    /// Stored CRC-8 checksum does not match computed CRC-8.
    CrcMismatch { expected: u8, actual: u8 },
    /// Neither dual-bank commit slot has a valid header and CRC-8.
    CorruptedBanks,
    /// Underlying EEPROM hardware driver returned an error.
    DeviceIoError,
    /// APDU command header or payload is malformed or unsupported.
    InvalidApdu,
}
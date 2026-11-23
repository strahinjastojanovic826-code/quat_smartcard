use crate::error::QuatError;
use crate::quat::Quat;

/// View over a mutable EEPROM/RAM byte slice storing packed quaternary symbols.
/// Bit layout inside each byte (MSB-first lane 0..=3):
/// - lane 0: bits 7..6 (`shift = 6`)
/// - lane 1: bits 5..4 (`shift = 4`)
/// - lane 2: bits 3..2 (`shift = 2`)
/// - lane 3: bits 1..0 (`shift = 0`)
pub struct PackedQuatSlice<'a> {
    storage: &'a mut [u8],
    len: usize,
}

impl<'a> PackedQuatSlice<'a> {
    /// Wraps `storage` for `len` quaternary symbols, verifying byte capacity.
    pub fn new(storage: &'a mut [u8], len: usize) -> Result<Self, QuatError> {
        let required_bytes = Quat::packed_bytes_for_quats(len);
        if storage.len() < required_bytes {
            return Err(QuatError::BufferTooSmall {
                required_bytes,
                available_bytes: storage.len(),
            });
        }
        Ok(Self { storage, len })
    }

    /// Number of addressable quaternary symbols.
    #[inline]
    pub const fn len(&self) -> usize {
        self.len
    }

    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Reads the quaternary symbol at `index`.
    pub fn get(&self, index: usize) -> Result<Quat, QuatError> {
        if index >= self.len {
            return Err(QuatError::IndexOutOfBounds {
                index,
                len: self.len,
            });
        }
        let byte_idx = index / 4;
        let lane = index % 4;
        let shift = (3 - lane) * 2;
        let bits = (self.storage[byte_idx] >> shift) & 0b11;
        Ok(Quat::from_bits_masked(bits))
    }

    /// Writes `quat` at `index` without disturbing neighboring 2-bit lanes.
    pub fn set(&mut self, index: usize, quat: Quat) -> Result<(), QuatError> {
        if index >= self.len {
            return Err(QuatError::IndexOutOfBounds {
                index,
                len: self.len,
            });
        }
        let byte_idx = index / 4;
        let lane = index % 4;
        let shift = (3 - lane) * 2;
        let mask = !(0b11u8 << shift);
        self.storage[byte_idx] = (self.storage[byte_idx] & mask) | (quat.bits() << shift);
        Ok(())
    }

    /// Fills all `len` quaternary symbols with `quat`.
    pub fn fill(&mut self, quat: Quat) -> Result<(), QuatError> {
        for i in 0..self.len {
            self.set(i, quat)?;
        }
        Ok(())
    }

    /// Writes a slice of `Quat` values into the packed buffer.
    pub fn encode_quats(&mut self, quats: &[Quat]) -> Result<(), QuatError> {
        if quats.len() != self.len {
            return Err(QuatError::InvalidLength {
                expected: self.len,
                actual: quats.len(),
            });
        }
        for (i, &q) in quats.iter().enumerate() {
            self.set(i, q)?;
        }
        Ok(())
    }

    /// Reads all packed `Quat` values into `out_quats`.
    pub fn decode_quats(&self, out_quats: &mut [Quat]) -> Result<(), QuatError> {
        if out_quats.len() != self.len {
            return Err(QuatError::InvalidLength {
                expected: self.len,
                actual: out_quats.len(),
            });
        }
        for (i, slot) in out_quats.iter_mut().enumerate() {
            *slot = self.get(i)?;
        }
        Ok(())
    }

    /// Encodes ASCII `'0'..'3'` bytes into packed quaternary storage,
    /// saving 75% of bits compared to 1 byte per symbol.
    pub fn encode_ascii(&mut self, ascii_digits: &[u8]) -> Result<(), QuatError> {
        if ascii_digits.len() != self.len {
            return Err(QuatError::InvalidLength {
                expected: self.len,
                actual: ascii_digits.len(),
            });
        }
        for (i, &ch) in ascii_digits.iter().enumerate() {
            let q = Quat::from_ascii_digit(ch)?;
            self.set(i, q)?;
        }
        Ok(())
    }

    /// Decodes packed quaternary symbols into an ASCII `'0'..'3'` output buffer.
    pub fn decode_ascii(&self, out_ascii: &mut [u8]) -> Result<(), QuatError> {
        if out_ascii.len() != self.len {
            return Err(QuatError::InvalidLength {
                expected: self.len,
                actual: out_ascii.len(),
            });
        }
        for (i, slot) in out_ascii.iter_mut().enumerate() {
            *slot = self.get(i)?.to_ascii_digit();
        }
        Ok(())
    }

    /// Returns the underlying packed EEPROM bytes.
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        let used = Quat::packed_bytes_for_quats(self.len);
        &self.storage[..used]
    }
}
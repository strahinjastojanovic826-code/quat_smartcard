use crate::error::QuatError;
use crate::packed_buf::PackedQuatSlice;
use crate::quat::Quat;

/// Target hardware profiles with tightly constrained non-volatile memory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceProfile {
    Esim,
    RfidTag,
    SimCard,
    SecurityToken,
}

/// Hardware abstraction for non-volatile EEPROM access.
pub trait EepromDevice {
    type Error;

    fn read_eeprom(&self, addr: usize, dst: &mut [u8]) -> Result<(), Self::Error>;
    fn write_eeprom(&mut self, addr: usize, src: &[u8]) -> Result<(), Self::Error>;
}

/// Fixed-capacity EEPROM slot descriptor for quaternary state/code tables.
pub struct EepromSlot<'a> {
    pub profile: DeviceProfile,
    pub slot_id: u16,
    pub quats: PackedQuatSlice<'a>,
}

impl<'a> EepromSlot<'a> {
    pub fn init(
        profile: DeviceProfile,
        slot_id: u16,
        eeprom_slice: &'a mut [u8],
        quat_len: usize,
        fill: Quat,
    ) -> Result<Self, QuatError> {
        let mut quats = PackedQuatSlice::new(eeprom_slice, quat_len)?;
        quats.fill(fill)?;
        Ok(Self {
            profile,
            slot_id,
            quats,
        })
    }

    /// Returns packed byte footprint in EEPROM.
    #[inline]
    pub fn eeprom_bytes_used(&self) -> usize {
        self.quats.as_bytes().len()
    }
}

/// Simple rotating wear-leveling cursor across `slot_count` identical slots
/// to distribute EEPROM erase/write cycles evenly on smartcards and RFID tags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WearLeveler {
    pub slot_count: u16,
    pub current_slot: u16,
    pub write_count: u32,
}

impl WearLeveler {
    pub const fn new(slot_count: u16) -> Result<Self, QuatError> {
        if slot_count == 0 {
            return Err(QuatError::NoAvailableSlot);
        }
        Ok(Self {
            slot_count,
            current_slot: 0,
            write_count: 0,
        })
    }

    /// Advances to the next slot in round-robin order and increments write count.
    pub fn advance(&mut self) -> Result<u16, QuatError> {
        if self.slot_count == 0 {
            return Err(QuatError::NoAvailableSlot);
        }
        self.current_slot = (self.current_slot + 1) % self.slot_count;
        self.write_count = self.write_count.saturating_add(1);
        Ok(self.current_slot)
    }
}
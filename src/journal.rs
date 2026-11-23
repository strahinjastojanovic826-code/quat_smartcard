use crate::crc::crc8_smbus;
use crate::error::QuatError;
use crate::quat::Quat;
use crate::smartcard::EepromDevice;

const MAGIC_BANK: u8 = 0x51; // 'Q'

/// Fixed 4-byte bank header stored right before packed quaternary bytes:
/// - `[0]`: `MAGIC_BANK` (`0x51`)
/// - `[1]`: monotonic sequence counter (`seq`)
/// - `[2]`: symbol length low/high byte (`quat_len as u8`, capped to `u8::MAX` in header)
/// - `[3]`: CRC-8 over packed payload bytes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct BankHeader {
    magic: u8,
    seq: u8,
    quat_len: u8,
    payload_crc: u8,
}

impl BankHeader {
    const SIZE: usize = 4;

    const fn to_bytes(self) -> [u8; Self::SIZE] {
        [self.magic, self.seq, self.quat_len, self.payload_crc]
    }

    const fn from_bytes(buf: [u8; Self::SIZE]) -> Self {
        Self {
            magic: buf[0],
            seq: buf[1],
            quat_len: buf[2],
            payload_crc: buf[3],
        }
    }
}

/// Dual-bank atomic commit manager for constrained EEPROMs.
/// Each bank occupies `4 + packed_bytes_for_quats(max_quats)` bytes.
pub struct DualBankCommit {
    pub bank_a_addr: usize,
    pub bank_b_addr: usize,
    pub max_quats: usize,
    pub last_seq: u8,
}

impl DualBankCommit {
    pub const fn bank_footprint_bytes(max_quats: usize) -> usize {
        BankHeader::SIZE + Quat::packed_bytes_for_quats(max_quats)
    }

    pub const fn new(bank_a_addr: usize, bank_b_addr: usize, max_quats: usize) -> Self {
        Self {
            bank_a_addr,
            bank_b_addr,
            max_quats,
            last_seq: 0,
        }
    }

    fn read_bank_if_valid<D: EepromDevice>(
        &self,
        dev: &D,
        addr: usize,
        out_packed: &mut [u8],
    ) -> Result<Option<(u8, usize)>, QuatError> {
        let packed_len = Quat::packed_bytes_for_quats(self.max_quats);
        if out_packed.len() < packed_len {
            return Err(QuatError::BufferTooSmall {
                required_bytes: packed_len,
                available_bytes: out_packed.len(),
            });
        }

        let mut hdr_bytes = [0u8; BankHeader::SIZE];
        dev.read_eeprom(addr, &mut hdr_bytes)
            .map_err(|_| QuatError::DeviceIoError)?;
        let hdr = BankHeader::from_bytes(hdr_bytes);
        if hdr.magic != MAGIC_BANK {
            return Ok(None);
        }

        let q_len = hdr.quat_len as usize;
        if q_len > self.max_quats {
            return Ok(None);
        }

        let used_bytes = Quat::packed_bytes_for_quats(q_len);
        dev.read_eeprom(addr + BankHeader::SIZE, &mut out_packed[..used_bytes])
            .map_err(|_| QuatError::DeviceIoError)?;

        let actual_crc = crc8_smbus(&out_packed[..used_bytes]);
        if actual_crc != hdr.payload_crc {
            return Ok(None);
        }

        Ok(Some((hdr.seq, q_len)))
    }

    /// Commits a pre-packed quaternary byte slice into the inactive EEPROM bank
    /// with a monotonic sequence number and CRC-8 integrity tag.
    pub fn commit_packed<D: EepromDevice>(
        &mut self,
        dev: &mut D,
        quat_len: usize,
        packed_src: &[u8],
    ) -> Result<u8, QuatError> {
        if quat_len > self.max_quats || quat_len > u8::MAX as usize {
            return Err(QuatError::InvalidLength {
                expected: self.max_quats.min(u8::MAX as usize),
                actual: quat_len,
            });
        }
        let used_bytes = Quat::packed_bytes_for_quats(quat_len);
        if packed_src.len() < used_bytes {
            return Err(QuatError::BufferTooSmall {
                required_bytes: used_bytes,
                available_bytes: packed_src.len(),
            });
        }

        let next_seq = self.last_seq.wrapping_add(1);
        let target_addr = if next_seq % 2 == 1 {
            self.bank_a_addr
        } else {
            self.bank_b_addr
        };
        let payload_crc = crc8_smbus(&packed_src[..used_bytes]);

        dev.write_eeprom(target_addr + BankHeader::SIZE, &packed_src[..used_bytes])
            .map_err(|_| QuatError::DeviceIoError)?;

        let hdr = BankHeader {
            magic: MAGIC_BANK,
            seq: next_seq,
            quat_len: quat_len as u8,
            payload_crc,
        };
        dev.write_eeprom(target_addr, &hdr.to_bytes())
            .map_err(|_| QuatError::DeviceIoError)?;

        self.last_seq = next_seq;
        Ok(next_seq)
    }

    /// Loads the newest valid bank into `out_packed` and returns `quat_len`.
    pub fn load_latest<D: EepromDevice>(
        &mut self,
        dev: &D,
        out_packed: &mut [u8],
    ) -> Result<usize, QuatError> {
        let packed_len = Quat::packed_bytes_for_quats(self.max_quats);
        if out_packed.len() < packed_len {
            return Err(QuatError::BufferTooSmall {
                required_bytes: packed_len,
                available_bytes: out_packed.len(),
            });
        }

        let mut buf_a = [0u8; 64];
        let mut buf_b = [0u8; 64];
        if packed_len > buf_a.len() {
            return Err(QuatError::BufferTooSmall {
                required_bytes: packed_len,
                available_bytes: buf_a.len(),
            });
        }

        let valid_a = self.read_bank_if_valid(dev, self.bank_a_addr, &mut buf_a[..packed_len])?;
        let valid_b = self.read_bank_if_valid(dev, self.bank_b_addr, &mut buf_b[..packed_len])?;

        match (valid_a, valid_b) {
            (Some((seq_a, len_a)), Some((seq_b, len_b))) => {
                // Wrapping comparison handles 8-bit sequence rollover safely.
                let a_is_newer = seq_a.wrapping_sub(seq_b) < 128;
                if a_is_newer {
                    let used = Quat::packed_bytes_for_quats(len_a);
                    out_packed[..used].copy_from_slice(&buf_a[..used]);
                    self.last_seq = seq_a;
                    Ok(len_a)
                } else {
                    let used = Quat::packed_bytes_for_quats(len_b);
                    out_packed[..used].copy_from_slice(&buf_b[..used]);
                    self.last_seq = seq_b;
                    Ok(len_b)
                }
            }
            (Some((seq_a, len_a)), None) => {
                let used = Quat::packed_bytes_for_quats(len_a);
                out_packed[..used].copy_from_slice(&buf_a[..used]);
                self.last_seq = seq_a;
                Ok(len_a)
            }
            (None, Some((seq_b, len_b))) => {
                let used = Quat::packed_bytes_for_quats(len_b);
                out_packed[..used].copy_from_slice(&buf_b[..used]);
                self.last_seq = seq_b;
                Ok(len_b)
            }
            (None, None) => Err(QuatError::CorruptedBanks),
        }
    }
}
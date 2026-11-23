#![no_std]

pub mod apdu;
pub mod crc;
pub mod error;
pub mod iter;
pub mod journal;
pub mod packed_buf;
pub mod quat;
pub mod smartcard;

pub use apdu::{handle_write_ascii_apdu, ApduHeader, QuatInstruction};
pub use crc::crc8_smbus;
pub use error::QuatError;
pub use iter::QuatIter;
pub use journal::DualBankCommit;
pub use packed_buf::PackedQuatSlice;
pub use quat::Quat;
pub use smartcard::{DeviceProfile, EepromDevice, EepromSlot, WearLeveler};

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::println;
    use std::time::Instant;

    struct RamEeprom<const N: usize> {
        mem: [u8; N],
        fail_writes: bool,
    }

    impl<const N: usize> RamEeprom<N> {
        const fn new() -> Self {
            Self {
                mem: [0xFF; N],
                fail_writes: false,
            }
        }
    }

    impl<const N: usize> EepromDevice for RamEeprom<N> {
        type Error = ();

        fn read_eeprom(&self, addr: usize, dst: &mut [u8]) -> Result<(), Self::Error> {
            let end = addr.checked_add(dst.len()).ok_or(())?;
            if end > self.mem.len() {
                return Err(());
            }
            dst.copy_from_slice(&self.mem[addr..end]);
            Ok(())
        }

        fn write_eeprom(&mut self, addr: usize, src: &[u8]) -> Result<(), Self::Error> {
            if self.fail_writes {
                return Err(());
            }
            let end = addr.checked_add(src.len()).ok_or(())?;
            if end > self.mem.len() {
                return Err(());
            }
            self.mem[addr..end].copy_from_slice(src);
            Ok(())
        }
    }

    fn report_bench(name: &str, iters: u128, elapsed_ns: u128) {
        let per_iter_ns = if iters > 0 { elapsed_ns / iters } else { 0 };
        println!(
            "[BENCH] {:<36} | iters={:<7} | total={} ns | avg={}.{} ns/iter",
            name,
            iters,
            elapsed_ns,
            per_iter_ns / 1000,
            per_iter_ns % 1000
        );
    }

    #[test]
    fn test_01_quat_four_states_and_bit_pairs() {
        let iters: u128 = 50_000;
        let start = Instant::now();
        for _ in 0..iters {
            let q0 = Quat::from_bit_pair(0, 0).unwrap();
            let q1 = Quat::from_bit_pair(0, 1).unwrap();
            let q2 = Quat::from_bit_pair(1, 0).unwrap();
            let q3 = Quat::from_bit_pair(1, 1).unwrap();

            assert_eq!(q0.bits(), 0b00);
            assert_eq!(q1.bits(), 0b01);
            assert_eq!(q2.bits(), 0b10);
            assert_eq!(q3.bits(), 0b11);
            assert_eq!(q2.to_bit_pair(), (1, 0));
            assert_eq!(q3.to_bit_pair(), (1, 1));
        }
        report_bench("01_quat_four_states_and_bit_pairs", iters, start.elapsed().as_nanos());
    }

    #[test]
    fn test_02_seventy_five_percent_savings_math() {
        let iters: u128 = 50_000;
        let start = Instant::now();
        for _ in 0..iters {
            let symbols = 16usize;
            // 16 symbols * 8 bits = 128 bits vs 16 symbols * 2 bits = 32 bits -> 96 bits saved (75%).
            let saved_bits = Quat::bits_saved_vs_byte_per_symbol(symbols);
            let packed_bytes = Quat::packed_bytes_for_quats(symbols);
            assert_eq!(saved_bits, 96);
            assert_eq!(saved_bits * 100 / (symbols * 8), 75);
            assert_eq!(packed_bytes, 4);
            assert_eq!(Quat::bytes_saved_vs_byte_per_symbol(symbols), 12);
        }
        report_bench("02_seventy_five_percent_savings_math", iters, start.elapsed().as_nanos());
    }

    #[test]
    fn test_03_single_byte_four_lane_packing() {
        let iters: u128 = 25_000;
        let start = Instant::now();
        for _ in 0..iters {
            let mut eeprom = [0u8; 1];
            let mut slice = PackedQuatSlice::new(&mut eeprom, 4).unwrap();
            slice.set(0, Quat::OneOne).unwrap();
            slice.set(1, Quat::ZeroOne).unwrap();
            slice.set(2, Quat::OneZero).unwrap();
            slice.set(3, Quat::ZeroZero).unwrap();

            assert_eq!(slice.as_bytes(), &[0b11_01_10_00]);
            assert_eq!(slice.get(0).unwrap(), Quat::OneOne);
            assert_eq!(slice.get(1).unwrap(), Quat::ZeroOne);
            assert_eq!(slice.get(2).unwrap(), Quat::OneZero);
            assert_eq!(slice.get(3).unwrap(), Quat::ZeroZero);
        }
        report_bench("03_single_byte_four_lane_packing", iters, start.elapsed().as_nanos());
    }

    #[test]
    fn test_04_neighbor_lane_isolation_on_overwrite() {
        let iters: u128 = 25_000;
        let start = Instant::now();
        for _ in 0..iters {
            let mut eeprom = [0u8; 2];
            let mut slice = PackedQuatSlice::new(&mut eeprom, 8).unwrap();
            slice.encode_ascii(b"01233210").unwrap();
            // Overwrite lane 2 only; lanes 0, 1, 3 and byte 1 must stay untouched.
            slice.set(2, Quat::ZeroZero).unwrap();
            assert_eq!(slice.get(1).unwrap(), Quat::ZeroOne);
            assert_eq!(slice.get(2).unwrap(), Quat::ZeroZero);
            assert_eq!(slice.get(3).unwrap(), Quat::OneOne);
            assert_eq!(slice.get(4).unwrap(), Quat::OneOne);
        }
        report_bench("04_neighbor_lane_isolation_on_overwrite", iters, start.elapsed().as_nanos());
    }

    #[test]
    fn test_05_ascii_encode_decode_roundtrip() {
        let iters: u128 = 10_000;
        let start = Instant::now();
        for _ in 0..iters {
            let input = b"0123123032100011";
            let mut eeprom = [0u8; 4];
            let mut decoded = [0u8; 16];

            let mut slice = PackedQuatSlice::new(&mut eeprom, input.len()).unwrap();
            slice.encode_ascii(input).unwrap();
            slice.decode_ascii(&mut decoded).unwrap();

            assert_eq!(&decoded, input);
            assert_eq!(slice.as_bytes().len(), 4);
        }
        report_bench("05_ascii_encode_decode_roundtrip", iters, start.elapsed().as_nanos());
    }

    #[test]
    fn test_06_validation_and_bounds_errors() {
        let iters: u128 = 25_000;
        let start = Instant::now();
        for _ in 0..iters {
            assert_eq!(Quat::from_u8(4), Err(QuatError::InvalidQuatValue(4)));
            assert_eq!(Quat::from_bit_pair(1, 2), Err(QuatError::InvalidBitPair(1, 2)));
            assert_eq!(Quat::from_ascii_digit(b'4'), Err(QuatError::InvalidAsciiDigit(b'4')));

            let mut tiny = [0u8; 1];
            assert!(matches!(
                PackedQuatSlice::new(&mut tiny, 5),
                Err(QuatError::BufferTooSmall {
                    required_bytes: 2,
                    available_bytes: 1
                })
            ));

            let mut ok_buf = [0u8; 1];
            let mut slice = PackedQuatSlice::new(&mut ok_buf, 4).unwrap();
            assert!(matches!(
                slice.get(4),
                Err(QuatError::IndexOutOfBounds { index: 4, len: 4 })
            ));
        }
        report_bench("06_validation_and_bounds_errors", iters, start.elapsed().as_nanos());
    }

    #[test]
    fn test_07_exact_size_quat_iterator() {
        let iters: u128 = 10_000;
        let start = Instant::now();
        for _ in 0..iters {
            let mut eeprom = [0u8; 3];
            let mut slice = PackedQuatSlice::new(&mut eeprom, 10).unwrap();
            let pattern = [
                Quat::ZeroZero,
                Quat::ZeroOne,
                Quat::OneZero,
                Quat::OneOne,
                Quat::OneOne,
                Quat::OneZero,
                Quat::ZeroOne,
                Quat::ZeroZero,
                Quat::ZeroOne,
                Quat::OneOne,
            ];
            slice.encode_quats(&pattern).unwrap();

            let mut iter = QuatIter::new(&slice);
            assert_eq!(iter.len(), 10);
            let mut collected = [Quat::ZeroZero; 10];
            for (idx, item) in iter.by_ref().enumerate() {
                collected[idx] = item;
            }
            assert_eq!(iter.len(), 0);
            assert_eq!(collected, pattern);
        }
        report_bench("07_exact_size_quat_iterator", iters, start.elapsed().as_nanos());
    }

    #[test]
    fn test_08_eeprom_slot_and_wear_leveler() {
        let iters: u128 = 10_000;
        let start = Instant::now();
        for _ in 0..iters {
            let mut eeprom = [0u8; 8];
            let slot = EepromSlot::init(DeviceProfile::Esim, 7, &mut eeprom, 24, Quat::OneZero).unwrap();
            assert_eq!(slot.profile, DeviceProfile::Esim);
            assert_eq!(slot.slot_id, 7);
            assert_eq!(slot.eeprom_bytes_used(), 6);

            let mut wl = WearLeveler::new(4).unwrap();
            assert_eq!(wl.current_slot, 0);
            assert_eq!(wl.advance().unwrap(), 1);
            assert_eq!(wl.advance().unwrap(), 2);
            assert_eq!(wl.advance().unwrap(), 3);
            assert_eq!(wl.advance().unwrap(), 0);
            assert_eq!(wl.write_count, 4);
        }
        report_bench("08_eeprom_slot_and_wear_leveler", iters, start.elapsed().as_nanos());
    }

    #[test]
    fn test_09_crc8_tamper_detection() {
        let iters: u128 = 10_000;
        let start = Instant::now();
        for _ in 0..iters {
            let payload = [0b11_10_01_00u8, 0b00_01_10_11u8, 0b10_10_00_11u8];
            let c1 = crc8_smbus(&payload);
            let mut tampered = payload;
            tampered[1] ^= 0b00_00_00_01;
            let c2 = crc8_smbus(&tampered);
            assert_ne!(c1, c2);
        }
        report_bench("09_crc8_tamper_detection", iters, start.elapsed().as_nanos());
    }

    #[test]
    fn test_10_apdu_parse_and_packed_write() {
        let iters: u128 = 10_000;
        let start = Instant::now();
        for _ in 0..iters {
            let raw_hdr = [0x00u8, 0xD6u8, 0x00u8, 0x00u8, 0x04u8];
            let hdr = ApduHeader::parse(&raw_hdr).unwrap();
            assert_eq!(hdr.ins, QuatInstruction::WriteAsciiPacked);

            let mut eeprom = [0u8; 2];
            let mut target = PackedQuatSlice::new(&mut eeprom, 8).unwrap();
            let sw = handle_write_ascii_apdu(hdr, b"3210", 2, &mut target).unwrap();
            assert_eq!(sw, (0x90, 0x00));
            assert_eq!(target.get(2).unwrap(), Quat::OneOne);
            assert_eq!(target.get(3).unwrap(), Quat::OneZero);
            assert_eq!(target.get(4).unwrap(), Quat::ZeroOne);
            assert_eq!(target.get(5).unwrap(), Quat::ZeroZero);
        }
        report_bench("10_apdu_parse_and_packed_write", iters, start.elapsed().as_nanos());
    }

    #[test]
    fn test_11_dual_bank_power_loss_recovery() {
        let iters: u128 = 1_000;
        let start = Instant::now();
        for _ in 0..iters {
            let max_quats = 32usize;
            let bank_size = DualBankCommit::bank_footprint_bytes(max_quats);
            let mut dev = RamEeprom::<256>::new();
            let mut journal = DualBankCommit::new(0, bank_size, max_quats);

            let mut p1 = [0u8; 8];
            let mut s1 = PackedQuatSlice::new(&mut p1, 32).unwrap();
            s1.encode_ascii(b"00001111222233330000111122223333").unwrap();
            journal.commit_packed(&mut dev, 32, s1.as_bytes()).unwrap();

            let mut p2 = [0u8; 8];
            let mut s2 = PackedQuatSlice::new(&mut p2, 32).unwrap();
            s2.encode_ascii(b"33332222111100003333222211110000").unwrap();
            journal.commit_packed(&mut dev, 32, s2.as_bytes()).unwrap();

            // Simulate power loss / bit corruption in Bank B payload.
            dev.mem[bank_size + 4 + 2] ^= 0xFF;

            let mut loaded = [0u8; 8];
            let loaded_len = journal.load_latest(&dev, &mut loaded).unwrap();
            assert_eq!(loaded_len, 32);
            assert_eq!(&loaded, s1.as_bytes());
        }
        report_bench("11_dual_bank_power_loss_recovery", iters, start.elapsed().as_nanos());
    }

    #[test]
    fn test_12_stress_high_volume_quats_and_commits() {
        let iters: u128 = 250;
        let start = Instant::now();
        for _ in 0..iters {
            // 256 quats = 64 bytes packed (vs 256 bytes unaligned 1-byte-per-symbol -> 75% saved).
            const QUAT_LEN: usize = 252;
const PACKED_LEN: usize = QUAT_LEN / 4;
let mut eeprom_buf = [0u8; PACKED_LEN];
let mut slice = PackedQuatSlice::new(&mut eeprom_buf, QUAT_LEN).unwrap();

for i in 0..QUAT_LEN {
    let q = Quat::from_bits_masked((i as u8).wrapping_mul(3));
    slice.set(i, q).unwrap();
}

let mut out_ascii = [0u8; QUAT_LEN];
slice.decode_ascii(&mut out_ascii).unwrap();

let mut roundtrip_buf = [0u8; PACKED_LEN];
let mut roundtrip = PackedQuatSlice::new(&mut roundtrip_buf, QUAT_LEN).unwrap();
roundtrip.encode_ascii(&out_ascii).unwrap();
assert_eq!(roundtrip.as_bytes(), slice.as_bytes());

let crc = crc8_smbus(roundtrip.as_bytes());
assert_eq!(crc, crc8_smbus(slice.as_bytes()));

let mut dev = RamEeprom::<512>::new();
let bank_span = DualBankCommit::bank_footprint_bytes(QUAT_LEN);
let mut journal = DualBankCommit::new(0, bank_span, QUAT_LEN);
for _step in 0..16 {
    journal.commit_packed(&mut dev, QUAT_LEN, roundtrip.as_bytes()).unwrap();
}
let mut recovered = [0u8; PACKED_LEN];
let len = journal.load_latest(&dev, &mut recovered).unwrap();
assert_eq!(len, QUAT_LEN);
assert_eq!(&recovered, slice.as_bytes());
        }
        report_bench("12_stress_high_volume_quats_and_commits", iters, start.elapsed().as_nanos());
    }
}
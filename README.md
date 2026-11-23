# `quat_smartcard`

A precision `no_std` Rust library built around **2-bit quaternary logic** (`00`, `01`, `10`, `11` / `0..=3`) for devices with tightly constrained non-volatile memory such as **eSIM chips**, **RFID tags**, **SIM cards**, and **security tokens**.

## Project Structure

```text
quat_smartcard/
├── Cargo.toml
└── src/
    ├── lib.rs
    ├── error.rs
    ├── quat.rs
    ├── packed_buf.rs
    ├── iter.rs
    ├── crc.rs
    ├── journal.rs
    ├── apdu.rs
    └── smartcard.rs
```

## 📊 Benchmark

| Test | Iterations | Total Time | Avg / Iter | Status |
| :--- | :--- | :--- | :--- | :--- |
| `02_seventy_five_percent_savings_math` | `50000` | `1572400 ns` | `0.31 ns/iter` | `ok` |
| `09_crc8_tamper_detection` | `10000` | `1880800 ns` | `0.188 ns/iter` | `ok` |
| `06_validation_and_bounds_errors` | `25000` | `2911400 ns` | `0.116 ns/iter` | `ok` |
| `10_apdu_parse_and_packed_write` | `10000` | `3883200 ns` | `0.388 ns/iter` | `ok` |
| `03_single_byte_four_lane_packing` | `25000` | `5544100 ns` | `0.221 ns/iter` | `ok` |
| `01_quat_four_states_and_bit_pairs` | `50000` | `7115700 ns` | `0.142 ns/iter` | `ok` |
| `11_dual_bank_power_loss_recovery` | `1000` | `6626800 ns` | `6.626 ns/iter` | `ok` |
| `08_eeprom_slot_and_wear_leveler` | `10000` | `9281300 ns` | `0.928 ns/iter` | `ok` |
| `07_exact_size_quat_iterator` | `10000` | `9455700 ns` | `0.945 ns/iter` | `ok` |
| `04_neighbor_lane_isolation_on_overwrite` | `25000` | `12017600 ns` | `0.480 ns/iter` | `ok` |
| `05_ascii_encode_decode_roundtrip` | `10000` | `12049300 ns` | `1.204 ns/iter` | `ok` |
| `12_stress_high_volume_quats_and_commits` | `250` | `16250500 ns` | `65.2 ns/iter` | `ok` |

## Features

- **`no_std` compatible:** Zero heap allocations (`alloc` or `std` are not required by the library itself).
- **2-bit quaternary representation:** Models `00`, `01`, `10`, and `11` explicitly with validation against out-of-range values and bit pairs.
- **75% memory savings vs. 8-bit per symbol:** Packs 2 bits per symbol instead of 8 bits per symbol (`6` bits saved per symbol; `4` symbols packed per EEPROM byte).
- **Neighbor-safe lane updates:** Masks and updates only the target 2-bit lane inside an EEPROM byte.
- **Zero-allocation iteration:** `QuatIter` implements `Iterator` and `ExactSizeIterator` over packed quaternary slices.
- **Wear leveling & device traits:** `EepromDevice`, `EepromSlot`, and `WearLeveler` support constrained EEPROM write cycles across hardware profiles (`Esim`, `RfidTag`, `SimCard`, `SecurityToken`).
- **Integrity & power-loss resilience:** Includes `crc8_smbus` and `DualBankCommit` to recover the latest valid bank after incomplete writes or power loss.
- **APDU support:** Provides `ApduHeader`, `QuatInstruction`, and `handle_write_ascii_apdu` for packed smartcard writes.

## Licensing

This project is dual-licensed under:

1. **GNU Affero General Public License v3.0 (AGPL-3.0)** for open-source use.
2. **Commercial License** for proprietary/commercial applications.

### Commercial License
If you wish to use `quat_smartcard` in closed-source software, proprietary products, or environments where AGPL-3.0 compliance is not feasible (e.g., embedding DLLs/shared libraries without disclosing host source code), a commercial license is required.

For commercial licensing inquiries, please contact: `strahinjastojanovic826@gmail.com`

## Import & Usage Example

Add `quat_smartcard` to your `Cargo.toml`:

```toml
[dependencies]
quat_smartcard = "0.1.0"
```

Import and use the core types in your crate:

```rust
use quat_smartcard::{
    crc8_smbus, DeviceProfile, EepromSlot, PackedQuatSlice, Quat, QuatIter, WearLeveler,
};

fn main() {
    // 16 quaternary symbols = 32 bits (4 bytes) vs. 128 bits (16 bytes) at 1 byte/symbol -> 75% saved.
    let symbols = 16usize;
    let saved_bits = Quat::bits_saved_vs_byte_per_symbol(symbols);
    let required_bytes = Quat::packed_bytes_for_quats(symbols);
    assert_eq!(saved_bits, 96);
    assert_eq!(required_bytes, 4);

    let mut eeprom = [0u8; 8];
    let mut slot = EepromSlot::init(DeviceProfile::Esim, 1, &mut eeprom, symbols, Quat::ZeroZero)
        .expect("valid EEPROM slot");

    slot.quats
        .encode_ascii(b"0001101100011011")
        .expect("valid quaternary ASCII digits");

    let collected: [Quat; 16] = {
        let mut arr = [Quat::ZeroZero; 16];
        for (i, q) in QuatIter::new(&slot.quats).enumerate() {
            arr[i] = q;
        }
        arr
    };
    assert_eq!(collected[2], Quat::OneZero);

    let checksum = crc8_smbus(slot.quats.as_bytes());
    let mut wear = WearLeveler::new(4).expect("non-zero slot count");
    let next_slot = wear.advance().expect("advance wear leveler");

    println!(
        "bytes_used={}, crc=0x{:02X}, next_slot={}",
        slot.eeprom_bytes_used(),
        checksum,
        next_slot
    );
}
```

## Running Tests & Benchmarks

Run the full test suite:

```bash
cargo test
```

Run tests with benchmark output (`println!`) visible in the terminal:

```bash
cargo test -- --nocapture
```

```bash
cargo test 
```

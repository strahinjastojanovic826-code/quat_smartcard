/// Deterministic CRC-8 (polynomial 0x07, init 0x00) for verifying
/// packed quaternary EEPROM payloads in `no_std` environments.
pub const fn crc8_smbus(data: &[u8]) -> u8 {
    let mut crc: u8 = 0x00;
    let mut i = 0;
    while i < data.len() {
        crc ^= data[i];
        let mut bit = 0;
        while bit < 8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
            bit += 1;
        }
        i += 1;
    }
    crc
}
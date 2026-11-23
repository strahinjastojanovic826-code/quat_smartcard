use crate::error::QuatError;
use crate::packed_buf::PackedQuatSlice;
use crate::quat::Quat;

/// Instruction codes for packed quaternary smartcard operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuatInstruction {
    /// Read packed quaternary bytes (`INS = 0xB0`).
    ReadPacked = 0xB0,
    /// Write ASCII `'0'..'3'` symbols packed at 2 bits per symbol (`INS = 0xD6`).
    WriteAsciiPacked = 0xD6,
}

impl QuatInstruction {
    pub const fn from_byte(ins: u8) -> Result<Self, QuatError> {
        match ins {
            0xB0 => Ok(Self::ReadPacked),
            0xD6 => Ok(Self::WriteAsciiPacked),
            _ => Err(QuatError::InvalidApdu),
        }
    }
}

/// Minimal ISO-7816-like command header (`CLA, INS, P1, P2, LC`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ApduHeader {
    pub cla: u8,
    pub ins: QuatInstruction,
    pub p1: u8,
    pub p2: u8,
    pub lc: u8,
}

impl ApduHeader {
    pub const fn parse(raw: &[u8]) -> Result<Self, QuatError> {
    if raw.len() < 5 {
        return Err(QuatError::InvalidApdu);
    }
    let ins = match QuatInstruction::from_byte(raw[1]) {
        Ok(instruction) => instruction,
        Err(err) => return Err(err),
    };
    Ok(Self {
        cla: raw[0],
        ins,
        p1: raw[2],
        p2: raw[3],
        lc: raw[4],
    })
}

}

/// Executes a `WriteAsciiPacked` APDU payload into `target` starting at symbol `start_quat`.
/// Returns status word `(SW1, SW2)` — `(0x90, 0x00)` on success.
pub fn handle_write_ascii_apdu(
    header: ApduHeader,
    data: &[u8],
    start_quat: usize,
    target: &mut PackedQuatSlice<'_>,
) -> Result<(u8, u8), QuatError> {
    if header.ins != QuatInstruction::WriteAsciiPacked {
        return Err(QuatError::InvalidApdu);
    }
    if data.len() != header.lc as usize {
        return Err(QuatError::InvalidLength {
            expected: header.lc as usize,
            actual: data.len(),
        });
    }
    for (offset, &ch) in data.iter().enumerate() {
        let q = Quat::from_ascii_digit(ch)?;
        target.set(start_quat + offset, q)?;
    }
    Ok((0x90, 0x00))
}
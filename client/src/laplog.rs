//! LAPLOG v1 serialization — CONTRACTS.md §3, little-endian throughout.
//!
//! ```text
//! offset      size  field
//! 0           4     magic "STLG"
//! 4           2     version = 1
//! 6           2     tick_rate = 400
//! 8           8     track_hash (FNV-1a-64 of exact track.bin bytes)
//! 16          4     tick_count N (1 ≤ N ≤ 72000)
//! 20          8·N   tick records { i16 steer, u16 throttle, u16 brake, u16 flags }
//! 20+8N       8     final_state_hash
//! 28+8N       4     claimed_lap_ticks
//! ```

pub const MAGIC: [u8; 4] = *b"STLG";
pub const VERSION: u16 = 1;
pub const TICK_RATE: u16 = 400;
pub const MAX_TICKS: u32 = 72_000;
pub const HEADER_LEN: usize = 20;
pub const TRAILER_LEN: usize = 12;
pub const TICK_RECORD_LEN: usize = 8;

/// FNV-1a 64-bit hash. Used for track hashes (CONTRACTS.md §3/§8).
pub fn fnv1a64(bytes: &[u8]) -> u64 {
    const OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;
    let mut h = OFFSET_BASIS;
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(PRIME);
    }
    h
}

/// One quantized 400 Hz input tick — CONTRACTS.md §2.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TickRecord {
    pub steer: i16,
    pub throttle: u16,
    pub brake: u16,
    pub flags: u16,
}

impl TickRecord {
    pub fn to_bytes(self) -> [u8; TICK_RECORD_LEN] {
        let mut b = [0u8; TICK_RECORD_LEN];
        b[0..2].copy_from_slice(&self.steer.to_le_bytes());
        b[2..4].copy_from_slice(&self.throttle.to_le_bytes());
        b[4..6].copy_from_slice(&self.brake.to_le_bytes());
        b[6..8].copy_from_slice(&self.flags.to_le_bytes());
        b
    }

    pub fn from_bytes(b: &[u8]) -> TickRecord {
        TickRecord {
            steer: i16::from_le_bytes([b[0], b[1]]),
            throttle: u16::from_le_bytes([b[2], b[3]]),
            brake: u16::from_le_bytes([b[4], b[5]]),
            flags: u16::from_le_bytes([b[6], b[7]]),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct LapLog {
    pub track_hash: u64,
    pub ticks: Vec<TickRecord>,
    pub final_state_hash: u64,
    pub claimed_lap_ticks: u32,
}

#[derive(Debug, PartialEq, Eq)]
pub enum LapLogError {
    TooShort,
    BadMagic,
    BadVersion(u16),
    BadTickRate(u16),
    BadTickCount(u32),
    LengthMismatch { expected: usize, actual: usize },
}

impl std::fmt::Display for LapLogError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LapLogError::TooShort => write!(f, "laplog too short for header"),
            LapLogError::BadMagic => write!(f, "bad magic (expected \"STLG\")"),
            LapLogError::BadVersion(v) => write!(f, "unsupported laplog version {v}"),
            LapLogError::BadTickRate(r) => write!(f, "unexpected tick rate {r} (expected 400)"),
            LapLogError::BadTickCount(n) => write!(f, "tick count {n} out of range 1..=72000"),
            LapLogError::LengthMismatch { expected, actual } => {
                write!(f, "laplog length mismatch: expected {expected} bytes, got {actual}")
            }
        }
    }
}

impl std::error::Error for LapLogError {}

impl LapLog {
    pub fn serialized_len(&self) -> usize {
        HEADER_LEN + TICK_RECORD_LEN * self.ticks.len() + TRAILER_LEN
    }

    /// Serialize to the exact §3 byte layout.
    pub fn serialize(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.serialized_len());
        out.extend_from_slice(&MAGIC);
        out.extend_from_slice(&VERSION.to_le_bytes());
        out.extend_from_slice(&TICK_RATE.to_le_bytes());
        out.extend_from_slice(&self.track_hash.to_le_bytes());
        out.extend_from_slice(&(self.ticks.len() as u32).to_le_bytes());
        for t in &self.ticks {
            out.extend_from_slice(&t.to_bytes());
        }
        out.extend_from_slice(&self.final_state_hash.to_le_bytes());
        out.extend_from_slice(&self.claimed_lap_ticks.to_le_bytes());
        out
    }

    /// Parse and validate a §3 blob. Rejects bad magic/version/rate/count and
    /// any length that does not exactly equal 20 + 8·N + 12.
    pub fn deserialize(bytes: &[u8]) -> Result<LapLog, LapLogError> {
        if bytes.len() < HEADER_LEN + TRAILER_LEN {
            return Err(LapLogError::TooShort);
        }
        if bytes[0..4] != MAGIC {
            return Err(LapLogError::BadMagic);
        }
        let version = u16::from_le_bytes([bytes[4], bytes[5]]);
        if version != VERSION {
            return Err(LapLogError::BadVersion(version));
        }
        let tick_rate = u16::from_le_bytes([bytes[6], bytes[7]]);
        if tick_rate != TICK_RATE {
            return Err(LapLogError::BadTickRate(tick_rate));
        }
        let track_hash = u64::from_le_bytes(bytes[8..16].try_into().unwrap());
        let n = u32::from_le_bytes(bytes[16..20].try_into().unwrap());
        if n == 0 || n > MAX_TICKS {
            return Err(LapLogError::BadTickCount(n));
        }
        let expected = HEADER_LEN + TICK_RECORD_LEN * n as usize + TRAILER_LEN;
        if bytes.len() != expected {
            return Err(LapLogError::LengthMismatch { expected, actual: bytes.len() });
        }
        let mut ticks = Vec::with_capacity(n as usize);
        for i in 0..n as usize {
            let off = HEADER_LEN + i * TICK_RECORD_LEN;
            ticks.push(TickRecord::from_bytes(&bytes[off..off + TICK_RECORD_LEN]));
        }
        let toff = HEADER_LEN + TICK_RECORD_LEN * n as usize;
        let final_state_hash = u64::from_le_bytes(bytes[toff..toff + 8].try_into().unwrap());
        let claimed_lap_ticks = u32::from_le_bytes(bytes[toff + 8..toff + 12].try_into().unwrap());
        Ok(LapLog { track_hash, ticks, final_state_hash, claimed_lap_ticks })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fnv1a64_known_vector() {
        // Well-known FNV-1a-64 test vector, also pinned by the task contract.
        assert_eq!(fnv1a64(b"hello"), 0xa430d84680aabd0b);
        // Empty input = offset basis.
        assert_eq!(fnv1a64(b""), 0xcbf29ce484222325);
    }

    fn three_tick_log() -> LapLog {
        LapLog {
            track_hash: 0x1122334455667788,
            ticks: vec![
                TickRecord { steer: -32767, throttle: 0, brake: 65535, flags: 1 },
                TickRecord { steer: 0, throttle: 32768, brake: 0, flags: 0 },
                TickRecord { steer: 32767, throttle: 65535, brake: 258, flags: 0 },
            ],
            final_state_hash: 0xAABBCCDDEEFF0011,
            claimed_lap_ticks: 12345,
        }
    }

    #[test]
    fn serialize_byte_exact_layout() {
        let log = three_tick_log();
        let b = log.serialize();
        assert_eq!(b.len(), 20 + 8 * 3 + 12);

        // magic "STLG" as bytes S,T,L,G
        assert_eq!(&b[0..4], b"STLG");
        assert_eq!(&b[0..4], &[0x53, 0x54, 0x4C, 0x47]);
        // version = 1 LE
        assert_eq!(&b[4..6], &[0x01, 0x00]);
        // tick_rate = 400 LE (0x0190)
        assert_eq!(&b[6..8], &[0x90, 0x01]);
        // track_hash LE
        assert_eq!(&b[8..16], &[0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11]);
        // tick_count = 3 LE
        assert_eq!(&b[16..20], &[0x03, 0x00, 0x00, 0x00]);

        // tick 0: steer -32767 = 0x8001 LE, throttle 0, brake 0xFFFF, flags 1
        assert_eq!(&b[20..28], &[0x01, 0x80, 0x00, 0x00, 0xFF, 0xFF, 0x01, 0x00]);
        // tick 1: steer 0, throttle 32768 = 0x8000 LE, brake 0, flags 0
        assert_eq!(&b[28..36], &[0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00]);
        // tick 2: steer 32767 = 0x7FFF, throttle 0xFFFF, brake 258 = 0x0102, flags 0
        assert_eq!(&b[36..44], &[0xFF, 0x7F, 0xFF, 0xFF, 0x02, 0x01, 0x00, 0x00]);

        // final_state_hash at 20 + 8*3 = 44, LE
        assert_eq!(&b[44..52], &[0x11, 0x00, 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA]);
        // claimed_lap_ticks at 52, 12345 = 0x3039 LE
        assert_eq!(&b[52..56], &[0x39, 0x30, 0x00, 0x00]);
    }

    #[test]
    fn roundtrip() {
        let log = three_tick_log();
        let parsed = LapLog::deserialize(&log.serialize()).unwrap();
        assert_eq!(parsed, log);
    }

    #[test]
    fn rejects_malformed() {
        let log = three_tick_log();
        let good = log.serialize();

        let mut bad_magic = good.clone();
        bad_magic[0] = b'X';
        assert_eq!(LapLog::deserialize(&bad_magic), Err(LapLogError::BadMagic));

        let mut bad_version = good.clone();
        bad_version[4] = 2;
        assert_eq!(LapLog::deserialize(&bad_version), Err(LapLogError::BadVersion(2)));

        let mut bad_rate = good.clone();
        bad_rate[6..8].copy_from_slice(&500u16.to_le_bytes());
        assert_eq!(LapLog::deserialize(&bad_rate), Err(LapLogError::BadTickRate(500)));

        // truncated: drop last byte
        let truncated = &good[..good.len() - 1];
        assert!(matches!(
            LapLog::deserialize(truncated),
            Err(LapLogError::LengthMismatch { .. })
        ));

        // zero ticks
        let mut zero = good.clone();
        zero[16..20].copy_from_slice(&0u32.to_le_bytes());
        assert_eq!(LapLog::deserialize(&zero), Err(LapLogError::BadTickCount(0)));

        assert_eq!(LapLog::deserialize(&[]), Err(LapLogError::TooShort));
    }
}

//! TRK1 track parser — CONTRACTS.md §8. Rendering use only: the kernel parses
//! the same blob itself inside sim.wasm; we never feed it parsed data.
//!
//! ```text
//! offset  size  field
//! 0       4     magic "STRK"
//! 4       2     version = 1
//! 6       2     checkpoint_count C
//! 8       8     seed (u64)
//! 16      4     centerline sample count S
//! 20      4     terrain vertex count V
//! 24      4     terrain triangle count T
//! 28      40·S  centerline: pos f32[3], up f32[3], tangent f32[3], width f32
//! 28+40S  4·C   checkpoint sample indices (u32, ascending)
//! …       12·V  terrain vertices f32[3]
//! …       12·T  triangle indices u32[3]
//! …       12    spawn pose: position index u32, yaw f32, reserved f32
//! ```

use crate::laplog::fnv1a64;

pub const MAGIC: [u8; 4] = *b"STRK";
pub const VERSION: u16 = 1;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CenterlineSample {
    pub pos: [f32; 3],
    pub up: [f32; 3],
    pub tangent: [f32; 3],
    pub width: f32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Track {
    pub seed: u64,
    pub centerline: Vec<CenterlineSample>,
    /// Indices into `centerline`, ascending.
    pub checkpoints: Vec<u32>,
    pub terrain_vertices: Vec<[f32; 3]>,
    pub terrain_triangles: Vec<[u32; 3]>,
    /// Index into `centerline` of the spawn position.
    pub spawn_pos_index: u32,
    pub spawn_yaw: f32,
    /// FNV-1a-64 of the exact track.bin bytes (LAPLOG track_hash).
    pub hash: u64,
}

#[derive(Debug, PartialEq, Eq)]
pub enum TrackError {
    TooShort,
    BadMagic,
    BadVersion(u16),
    Truncated { need: usize, have: usize },
    CheckpointOutOfRange { index: u32, samples: u32 },
    TriangleOutOfRange { index: u32, vertices: u32 },
    SpawnOutOfRange { index: u32, samples: u32 },
}

impl std::fmt::Display for TrackError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TrackError::TooShort => write!(f, "track blob too short for header"),
            TrackError::BadMagic => write!(f, "bad magic (expected \"STRK\")"),
            TrackError::BadVersion(v) => write!(f, "unsupported TRK version {v}"),
            TrackError::Truncated { need, have } => {
                write!(f, "track blob truncated: need {need} bytes, have {have}")
            }
            TrackError::CheckpointOutOfRange { index, samples } => {
                write!(f, "checkpoint index {index} out of range ({samples} samples)")
            }
            TrackError::TriangleOutOfRange { index, vertices } => {
                write!(f, "triangle vertex index {index} out of range ({vertices} vertices)")
            }
            TrackError::SpawnOutOfRange { index, samples } => {
                write!(f, "spawn index {index} out of range ({samples} samples)")
            }
        }
    }
}

impl std::error::Error for TrackError {}

/// Little-endian byte cursor.
struct Cursor<'a> {
    data: &'a [u8],
    off: usize,
}

impl<'a> Cursor<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], TrackError> {
        if self.off + n > self.data.len() {
            return Err(TrackError::Truncated { need: self.off + n, have: self.data.len() });
        }
        let s = &self.data[self.off..self.off + n];
        self.off += n;
        Ok(s)
    }
    fn u16(&mut self) -> Result<u16, TrackError> {
        Ok(u16::from_le_bytes(self.take(2)?.try_into().unwrap()))
    }
    fn u32(&mut self) -> Result<u32, TrackError> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    fn u64(&mut self) -> Result<u64, TrackError> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn f32(&mut self) -> Result<f32, TrackError> {
        Ok(f32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    fn f32x3(&mut self) -> Result<[f32; 3], TrackError> {
        Ok([self.f32()?, self.f32()?, self.f32()?])
    }
}

impl Track {
    pub fn parse(bytes: &[u8]) -> Result<Track, TrackError> {
        if bytes.len() < 28 {
            return Err(TrackError::TooShort);
        }
        let mut c = Cursor { data: bytes, off: 0 };
        if c.take(4)? != MAGIC {
            return Err(TrackError::BadMagic);
        }
        let version = c.u16()?;
        if version != VERSION {
            return Err(TrackError::BadVersion(version));
        }
        let checkpoint_count = c.u16()? as usize;
        let seed = c.u64()?;
        let sample_count = c.u32()?;
        let vertex_count = c.u32()?;
        let triangle_count = c.u32()?;

        let mut centerline = Vec::with_capacity(sample_count as usize);
        for _ in 0..sample_count {
            centerline.push(CenterlineSample {
                pos: c.f32x3()?,
                up: c.f32x3()?,
                tangent: c.f32x3()?,
                width: c.f32()?,
            });
        }

        let mut checkpoints = Vec::with_capacity(checkpoint_count);
        for _ in 0..checkpoint_count {
            let idx = c.u32()?;
            if idx >= sample_count {
                return Err(TrackError::CheckpointOutOfRange { index: idx, samples: sample_count });
            }
            checkpoints.push(idx);
        }

        let mut terrain_vertices = Vec::with_capacity(vertex_count as usize);
        for _ in 0..vertex_count {
            terrain_vertices.push(c.f32x3()?);
        }

        let mut terrain_triangles = Vec::with_capacity(triangle_count as usize);
        for _ in 0..triangle_count {
            let tri = [c.u32()?, c.u32()?, c.u32()?];
            for &i in &tri {
                if i >= vertex_count {
                    return Err(TrackError::TriangleOutOfRange { index: i, vertices: vertex_count });
                }
            }
            terrain_triangles.push(tri);
        }

        let spawn_pos_index = c.u32()?;
        let spawn_yaw = c.f32()?;
        let _reserved = c.f32()?;
        if spawn_pos_index >= sample_count {
            return Err(TrackError::SpawnOutOfRange { index: spawn_pos_index, samples: sample_count });
        }

        Ok(Track {
            seed,
            centerline,
            checkpoints,
            terrain_vertices,
            terrain_triangles,
            spawn_pos_index,
            spawn_yaw,
            hash: fnv1a64(bytes),
        })
    }
}

#[cfg(test)]
pub(crate) fn synthetic_track_bytes() -> Vec<u8> {
    // 3 centerline samples, 1 checkpoint, 3 terrain vertices, 1 triangle.
    let mut b = Vec::new();
    b.extend_from_slice(b"STRK");
    b.extend_from_slice(&1u16.to_le_bytes()); // version
    b.extend_from_slice(&1u16.to_le_bytes()); // checkpoint_count
    b.extend_from_slice(&0xDEADBEEFu64.to_le_bytes()); // seed
    b.extend_from_slice(&3u32.to_le_bytes()); // S
    b.extend_from_slice(&3u32.to_le_bytes()); // V
    b.extend_from_slice(&1u32.to_le_bytes()); // T
    for i in 0..3u32 {
        let x = i as f32 * 10.0;
        for v in [x, 0.0, 0.5 * x] {
            b.extend_from_slice(&v.to_le_bytes()); // pos
        }
        for v in [0.0f32, 1.0, 0.0] {
            b.extend_from_slice(&v.to_le_bytes()); // up
        }
        for v in [1.0f32, 0.0, 0.0] {
            b.extend_from_slice(&v.to_le_bytes()); // tangent
        }
        b.extend_from_slice(&8.0f32.to_le_bytes()); // width
    }
    b.extend_from_slice(&2u32.to_le_bytes()); // checkpoint at sample 2
    for v in [[0.0f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]] {
        for c in v {
            b.extend_from_slice(&c.to_le_bytes());
        }
    }
    for i in [0u32, 1, 2] {
        b.extend_from_slice(&i.to_le_bytes());
    }
    b.extend_from_slice(&1u32.to_le_bytes()); // spawn index
    b.extend_from_slice(&0.25f32.to_le_bytes()); // yaw
    b.extend_from_slice(&0.0f32.to_le_bytes()); // reserved
    b
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_synthetic_track() {
        let bytes = synthetic_track_bytes();
        let t = Track::parse(&bytes).unwrap();
        assert_eq!(t.seed, 0xDEADBEEF);
        assert_eq!(t.centerline.len(), 3);
        assert_eq!(t.centerline[1].pos, [10.0, 0.0, 5.0]);
        assert_eq!(t.centerline[0].up, [0.0, 1.0, 0.0]);
        assert_eq!(t.centerline[2].tangent, [1.0, 0.0, 0.0]);
        assert_eq!(t.centerline[0].width, 8.0);
        assert_eq!(t.checkpoints, vec![2]);
        assert_eq!(t.terrain_vertices.len(), 3);
        assert_eq!(t.terrain_triangles, vec![[0, 1, 2]]);
        assert_eq!(t.spawn_pos_index, 1);
        assert!((t.spawn_yaw - 0.25).abs() < 1e-6);
        assert_eq!(t.hash, fnv1a64(&bytes));
    }

    #[test]
    fn rejects_bad_magic_and_version() {
        let mut bytes = synthetic_track_bytes();
        bytes[0] = b'X';
        assert_eq!(Track::parse(&bytes), Err(TrackError::BadMagic));

        let mut bytes = synthetic_track_bytes();
        bytes[4] = 9;
        assert_eq!(Track::parse(&bytes), Err(TrackError::BadVersion(9)));
    }

    #[test]
    fn rejects_truncation_and_bad_indices() {
        let bytes = synthetic_track_bytes();
        assert!(matches!(
            Track::parse(&bytes[..bytes.len() - 4]),
            Err(TrackError::Truncated { .. })
        ));

        // checkpoint index out of range (offset 28 + 40*3 = 148)
        let mut bad = synthetic_track_bytes();
        bad[148..152].copy_from_slice(&99u32.to_le_bytes());
        assert_eq!(
            Track::parse(&bad),
            Err(TrackError::CheckpointOutOfRange { index: 99, samples: 3 })
        );
    }
}

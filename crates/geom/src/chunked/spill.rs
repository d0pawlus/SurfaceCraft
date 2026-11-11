//! Spill files of the build: triangles routed to their leaf and parked on disk until the leaf is
//! turned into a chunk. At most [`SPILL_BUFFER_TRIANGLES`] triangles are buffered at any time.
use super::StoreError;
use crate::Point3;
use std::fs::OpenOptions;
use std::io::Write;
use std::path::{Path, PathBuf};

/// Triangles buffered in memory before the largest buffers are appended to their spill files.
pub const SPILL_BUFFER_TRIANGLES: usize = 1 << 18;
/// Bytes of one spilled triangle: nine `f64`.
pub const SPILL_TRIANGLE_BYTES: usize = 9 * 8;

/// File name of the spill of leaf `leaf`.
#[must_use]
pub fn spill_name(leaf: usize) -> String {
    format!("spill-{leaf:07}.tmp")
}

/// Routes triangles into per-leaf spill files in a directory.
pub struct Spill {
    dir: PathBuf,
    buffers: Vec<Vec<f64>>,
    buffered: usize,
    /// Triangles routed to each leaf so far.
    pub routed: Vec<u32>,
    /// High-water mark of buffered triangles.
    pub peak_buffered: usize,
    /// Number of flushes.
    pub flushes: usize,
}

impl Spill {
    /// A spill for `leaves` leaves in `dir`.
    #[must_use]
    pub fn new(dir: &Path, leaves: usize) -> Self {
        Self {
            dir: dir.to_path_buf(),
            buffers: vec![Vec::new(); leaves],
            buffered: 0,
            routed: vec![0; leaves],
            peak_buffered: 0,
            flushes: 0,
        }
    }

    /// Buffers triangle `t` for `leaf`, flushing when the budget is reached.
    pub fn push(&mut self, leaf: usize, t: &[Point3; 3]) -> Result<(), StoreError> {
        let (Some(buf), Some(n)) = (self.buffers.get_mut(leaf), self.routed.get_mut(leaf)) else {
            return Err(StoreError::Input("a triangle was routed to a leaf that does not exist".into()));
        };
        for p in t {
            buf.extend_from_slice(&[p.x, p.y, p.z]);
        }
        *n += 1;
        self.buffered += 1;
        self.peak_buffered = self.peak_buffered.max(self.buffered);
        if self.buffered >= SPILL_BUFFER_TRIANGLES {
            self.flush_to(SPILL_BUFFER_TRIANGLES / 2)?;
        }
        Ok(())
    }

    /// Appends the largest buffers to their files until at most `target` triangles stay buffered.
    fn flush_to(&mut self, target: usize) -> Result<(), StoreError> {
        let mut order: Vec<usize> =
            (0..self.buffers.len()).filter(|&i| self.buffers.get(i).is_some_and(|b| !b.is_empty())).collect();
        order.sort_by_key(|&i| std::cmp::Reverse(self.buffers.get(i).map_or(0, Vec::len)));
        for leaf in order {
            if self.buffered <= target {
                break;
            }
            self.flush_leaf(leaf)?;
        }
        self.flushes += 1;
        Ok(())
    }

    fn flush_leaf(&mut self, leaf: usize) -> Result<(), StoreError> {
        let Some(buf) = self.buffers.get_mut(leaf) else { return Ok(()) };
        if buf.is_empty() {
            return Ok(());
        }
        let path = self.dir.join(spill_name(leaf));
        let mut bytes = Vec::with_capacity(buf.len() * 8);
        for v in buf.iter() {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        let mut f = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&path)
            .map_err(|e| StoreError::Io(format!("{}: {e}", path.display())))?;
        f.write_all(&bytes).map_err(|e| StoreError::Io(format!("{}: {e}", path.display())))?;
        self.buffered -= buf.len() / 9;
        *buf = Vec::new();
        Ok(())
    }

    /// Writes everything still buffered.
    pub fn finish(&mut self) -> Result<(), StoreError> {
        self.flush_to(0)
    }
}

/// Reads back the spill of `leaf`, which must hold exactly `count` triangles, and deletes it.
pub fn take_spill(dir: &Path, leaf: usize, count: usize) -> Result<Vec<[Point3; 3]>, StoreError> {
    let path = dir.join(spill_name(leaf));
    let bytes = std::fs::read(&path).map_err(|e| StoreError::Io(format!("{}: {e}", path.display())))?;
    if count.checked_mul(SPILL_TRIANGLE_BYTES) != Some(bytes.len()) {
        return Err(StoreError::Input("the input changed while it was being imported (spill size mismatch)".into()));
    }
    let mut vals = bytes.as_chunks::<8>().0.iter().map(|c| f64::from_le_bytes(*c));
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        let mut next = || vals.next().ok_or_else(|| StoreError::Input("spill ended early".into()));
        let mut corners = [Point3::origin(); 3];
        for c in &mut corners {
            *c = Point3::new(next()?, next()?, next()?);
        }
        out.push(corners);
    }
    std::fs::remove_file(&path).map_err(|e| StoreError::Io(format!("{}: {e}", path.display())))?;
    Ok(out)
}

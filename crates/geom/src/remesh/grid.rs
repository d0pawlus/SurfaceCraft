//! The voxel grid of a remesh and the arithmetic that sizes it without overflow.
use super::RemeshError;
use crate::{Aabb, Point3, Vec3};

/// Grid axes may have at most this many cells (the cell key has 21 bits per axis).
pub const MAX_AXIS: usize = (1 << 21) - 4;

/// `a · b` as a `u64`: the product of two grid axes cannot wrap, whatever the pointer width.
#[must_use]
pub fn cell_product(a: usize, b: usize) -> u64 {
    (a as u64).saturating_mul(b as u64)
}

/// The grid: voxel `(i, j, k)` has its centre at `origin + (i + ½, j + ½, k + ½) · h`.
#[derive(Clone, Copy, Debug)]
pub struct Grid {
    /// Lower corner.
    pub origin: Point3,
    /// Voxel edge (mm).
    pub h: f64,
    /// Cells along x.
    pub nx: usize,
    /// Cells along y.
    pub ny: usize,
    /// Cells along z.
    pub nz: usize,
}

impl Grid {
    /// A grid of voxels of `h` around `bbox` with `border` empty voxels on every side. The number of
    /// cells is checked as a float before it is cast, so a tiny voxel is a refusal, never an overflow.
    pub fn new(bbox: &Aabb, h: f64, border: usize) -> Result<Self, RemeshError> {
        let too_fine = |needed: f64| RemeshError::TooMuchWork {
            what: "cells along one axis".into(),
            needed: needed as usize,
            limit: MAX_AXIS,
            advice: "use a larger voxel".into(),
        };
        if !(h.is_finite() && h > 0.0) {
            return Err(RemeshError::Params("the voxel must be a positive number".into()));
        }
        let size = bbox.size();
        let extra = 2.0 * border as f64 + 1.0;
        let mut n = [0usize; 3];
        for (k, s) in [size.x, size.y, size.z].into_iter().enumerate() {
            let cells = (s / h).ceil() + extra;
            if !(cells.is_finite() && cells <= MAX_AXIS as f64) {
                return Err(too_fine(cells));
            }
            n[k] = cells as usize;
        }
        let origin = bbox.min - Vec3::new(1.0, 1.0, 1.0) * (border as f64 * h);
        Ok(Self { origin, h, nx: n[0], ny: n[1], nz: n[2] })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_tiny_voxel_is_a_refusal_not_an_overflow() {
        let b = Aabb { min: Point3::origin(), max: Point3::new(300.0, 300.0, 300.0) };
        for h in [1e-18, 1e-300, f64::MIN_POSITIVE, 1e-9] {
            assert!(matches!(Grid::new(&b, h, 3), Err(RemeshError::TooMuchWork { .. })), "{h}");
        }
        assert!(matches!(Grid::new(&b, f64::NAN, 3), Err(RemeshError::Params(_))));
        assert!(Grid::new(&b, 0.3, 3).is_ok());
    }

    #[test]
    fn the_column_product_does_not_wrap_at_wasm_sizes() {
        assert_eq!(cell_product(65_536, 65_536), 4_294_967_296);
        assert_eq!(cell_product(usize::MAX, 4), if usize::BITS == 64 { u64::MAX } else { 4 * u64::from(u32::MAX) });
    }
}

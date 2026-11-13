//! Canonical coordinates: the cloud is rounded to a power-of-two lattice relative to its size.
//!
//! Converting a cloud between units (a file in metres, inches, ...) moves coordinates by a few
//! last bits, and a cloud sampled from a regular structure (a grid, a CAD tessellation) is full
//! of exact ties: equal neighbour distances, cocircular and cospherical points, equal pivot
//! angles. A tie decides on the last bit, so a rescaled copy of the same cloud triangulated
//! differently. Rounding every coordinate to a lattice of about `2^-LATTICE_BITS` of the cloud's
//! extent (a power of two, so the rounding is exact) removes those bits. The lattice is far below
//! scanner accuracy (about 1e-9 of the extent, and never coarser than `2^MAX_STEP_EXPONENT` mm).
//!
//! The guarantee is this: the same cloud always gives the same mesh, on every platform (the
//! rounding uses exact operations only), and a copy that differs only by conversion noise gives
//! the same mesh except when some coordinate lies within that noise of a midpoint between two
//! lattice points and rounds the other way (a probability of about `|noise| / step` per
//! coordinate, around 2^-22 for a conversion through a decimal factor and back, larger for clouds
//! far from the origin). That is the best a rounding can do: exact equality of two clouds that
//! differ in their bits cannot be promised in general.
//!
//! The reconstructed mesh takes the rounded positions as its vertices: a vertex lies at most half a
//! lattice step (about 5e-10 of the extent, at most 2^-21 mm) from the point that was read, and
//! points closer than a step are one point.

use super::*;

/// The lattice spacing is between `2^-(LATTICE_BITS + 1)` and `2^-LATTICE_BITS` of the extent.
const LATTICE_BITS: i32 = 30;

/// The lattice is never coarser than `2^MAX_STEP_EXPONENT` mm (about a nanometre): a far outlier
/// makes the extent huge, but must not coarsen the rest of the cloud.
const MAX_STEP_EXPONENT: i32 = -20;

/// Mantissa bits of the extent kept before its exponent is read: noise in the extent of a part
/// whose extent is a power of two must not move the lattice by a factor of two.
const EXTENT_BITS: u32 = 20;

/// `2^k` for an exponent in the normal range of `f64`, else `None`.
fn pow2(k: i32) -> Option<f64> {
    let biased = u64::try_from(k.checked_add(1023)?).ok().filter(|b| (1..2047).contains(b))?;
    Some(f64::from_bits(biased << 52))
}

/// The lattice spacing of `pts`: a power of two of about `1e-9` of the largest extent (at most `2^MAX_STEP_EXPONENT`); `None` for
/// clouds without a usable extent (empty, a single point, or subnormal sizes).
pub(super) fn lattice_step(pts: &[Point3]) -> Option<f64> {
    let bb = Aabb::from_points(pts);
    if bb.is_empty() {
        return None;
    }
    let ext = (bb.max - bb.min).max();
    if !(ext.is_finite() && ext > 0.0) {
        return None;
    }
    // round to EXTENT_BITS of mantissa (the carry may reach the exponent: 63.99999999999997 -> 64)
    let drop = 52 - EXTENT_BITS;
    let rounded = (ext.to_bits() + (1u64 << (drop - 1))) >> drop << drop;
    let exponent = i32::try_from((rounded >> 52) & 0x7ff).ok()? - 1023;
    pow2((exponent - LATTICE_BITS).min(MAX_STEP_EXPONENT))
}

/// `pts` rounded to the lattice (unchanged when the cloud has no usable extent).
pub(super) fn snapped(pts: &[Point3]) -> Vec<Point3> {
    let Some(step) = lattice_step(pts) else { return pts.to_vec() };
    let inv = 1.0 / step; // a power of two: exact
    let round = |v: f64| {
        let r = (v * inv).round() * step;
        // a coordinate just below zero rounds to -0.0: the duplicate sort orders -0.0 before 0.0,
        // so the two zeros must be one value
        if !r.is_finite() {
            v
        } else if r == 0.0 {
            0.0
        } else {
            r
        }
    };
    pts.iter().map(|p| Point3::new(round(p.x), round(p.y), round(p.z))).collect()
}

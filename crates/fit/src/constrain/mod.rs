//! Relations between fitted primitives that fix a position or a size, not only a direction:
//! tangency, symmetry about a plane and equal distances, plus the coaxial and equal-radius
//! relations that must keep holding while those are imposed (design doc §6.2).
//!
//! The relations are found within tolerances and then imposed by the smallest change of the
//! parameters that satisfies all of them (the usual approach of constraint recovery in reverse
//! engineering). Once the directions are fixed, every relation here is linear in the changes of
//! the parameters (a signed distance along a fixed direction, a radius, a coordinate), so
//! [`apply`] solves one small linear system per group of coupled primitives: the least-squares
//! change `x = Aᵀ(AAᵀ + λI)⁻¹ r`, refined iteratively. Relations that conflict leave a residual
//! and are reported as a [`Failure`], never forced.
//!
//! Variables: five per primitive, `[dx, dy, dz, d(radius 0), d(radius 1)]` (a shift of the anchor
//! point and the changes of its radii: cylinder and sphere radius; torus major and minor radius).
use crate::primitive::Primitive;
use nalgebra::{Matrix3, SymmetricEigen};
use std::cmp::Ordering;
use std::collections::BTreeMap;
use surfacecraft_geom::{Point3, Vec3, any_perpendicular, normalize};

/// Tangency and equal distance need directions that agree to this many degrees: by then the
/// angular snap has made them exact (or the angular relation was rejected, and then these
/// relations have no meaning).
const EXACT_DEG: f64 = 1e-4;
/// Largest family of parallel primitives searched for equally spaced chains.
const MAX_FAMILY: usize = 200;
/// Longest chain of equally spaced primitives.
const MAX_CHAIN: usize = 64;
/// Most chains reported.
const MAX_CHAINS: usize = 400;
/// Most tangent and symmetric relations reported.
const MAX_RELATIONS: usize = 3000;
/// Most rows in one coupled group (dense Gram matrix of that size).
const MAX_COMPONENT_ROWS: usize = 1200;
/// Most rows in one system.
const MAX_ROWS: usize = 6000;
/// Mirror planes are only searched among at most this many primitives.
const MAX_MIRROR_PRIMS: usize = 300;
/// Most points used for the principal planes.
const MAX_SAMPLE: usize = 20_000;
/// A solution leaves at most this residual (mm) for the relations to count as consistent.
const CONSISTENT: f64 = 1e-7;
mod find;
mod geometry;
mod rows;
mod solve;
#[cfg(test)]
mod tests;
mod types;

pub use find::{detect_mirrors, find_relations};
pub use rows::measure;
pub use solve::apply;
pub use types::{Failure, Mirror, Rel, RelKind};

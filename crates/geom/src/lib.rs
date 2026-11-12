//! SurfaceCraft geometry (layer L0): f64 points and vectors, rigid transforms, axis-aligned
//! boxes, indexed triangle meshes, a k-d tree over points and a deterministic random source.
//!
//! Nothing here knows about B-rep, files or UI. Algorithms that walk a mesh take a
//! [`PointSource`] where they can, so a later out-of-core mesh (design doc §9) can stand in.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

pub mod aabb;
pub mod bvh;
pub mod cancel;
pub mod cap;
pub mod chunked;
pub mod cloud;
mod crease;
mod decimate;
pub mod edit;
mod holes;
pub mod kdtree;
pub mod lod;
pub mod mesh;
pub mod par;
pub mod quality;
pub mod recon;
pub mod reduce;
pub mod rng;
pub mod section;
pub mod synth;
pub mod transform;
pub mod tri;
mod weld;

pub use aabb::Aabb;
pub use bvh::{Bvh, SurfaceHit};
pub use cancel::Cancel;
pub use cloud::{CloudError, PointCloud};
pub use kdtree::KdTree;
pub use mesh::{Mesh, MeshError, PointSource, TriangleSource};
pub use recon::{ReconError, ReconParams, ReconReport, reconstruct};
pub use rng::Rng;
pub use transform::Transform;

/// A point in model space, millimetres.
pub type Point3 = nalgebra::Point3<f64>;
/// A vector in model space.
pub type Vec3 = nalgebra::Vector3<f64>;
/// A 3×3 matrix.
pub type Mat3 = nalgebra::Matrix3<f64>;
/// A 4×4 homogeneous matrix.
pub type Mat4 = nalgebra::Matrix4<f64>;

/// Lengths below this are treated as zero (mm).
pub const EPS_LEN: f64 = 1e-12;

/// A unit vector or `None` when `v` is (numerically) zero or not finite.
#[must_use]
pub fn normalize(v: Vec3) -> Option<Vec3> {
    let n = v.norm();
    if n.is_finite() && n > EPS_LEN { Some(v / n) } else { None }
}

/// Any unit vector perpendicular to the unit vector `n`.
#[must_use]
pub fn any_perpendicular(n: &Vec3) -> Vec3 {
    let a = if n.x.abs() < 0.9 { Vec3::x() } else { Vec3::y() };
    let p = n.cross(&a);
    normalize(p).unwrap_or_else(Vec3::z)
}

/// Is every coordinate finite?
#[must_use]
pub fn finite(p: &Point3) -> bool {
    p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
}

/// Is every coordinate of the vector finite?
#[must_use]
pub fn finite_vec(v: &Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn perpendicular_is_unit_and_orthogonal() {
        for n in [Vec3::x(), Vec3::y(), Vec3::z(), Vec3::new(1.0, 2.0, 3.0).normalize()] {
            let p = any_perpendicular(&n);
            assert!((p.norm() - 1.0).abs() < 1e-12);
            assert!(p.dot(&n).abs() < 1e-12);
        }
        assert!(normalize(Vec3::zeros()).is_none());
        assert!(normalize(Vec3::new(f64::NAN, 0.0, 0.0)).is_none());
    }
}

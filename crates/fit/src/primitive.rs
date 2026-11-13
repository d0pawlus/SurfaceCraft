//! Analytic primitives fitted to mesh points: plane, sphere, cylinder, cone, torus.
//!
//! Each fit starts from a closed-form estimate (PCA, algebraic sphere/circle, normal-based axis,
//! a scan over the torus tube radius) and is refined by Levenberg–Marquardt on the orthogonal
//! (geometric) distance. The results are the exact analytic surfaces the kernel record holds
//! (`Plane`, `Sphere`, `Cylinder`, `Cone`, `Torus`), so a fitted primitive can later become a B-rep
//! face without approximation.
use crate::lm;
use nalgebra::{DVector, Matrix3, Matrix4, Vector3, Vector4};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};
use surfacecraft_geom::{Mat3, Mesh, Point3, Vec3, any_perpendicular, normalize};

mod estimate;
mod model;
mod quadrics;
#[cfg(test)]
mod tests;
mod torus;

pub use estimate::PrimitiveFit;
pub use model::{Primitive, PrimitiveKind};
pub use quadrics::{fit_cone, fit_cylinder, fit_plane, fit_sphere};
pub use torus::fit_torus;

/// Fits a primitive of the given kind.
#[must_use]
pub fn fit(kind: PrimitiveKind, pts: &[Point3], normals: &[Vec3]) -> Option<PrimitiveFit> {
    let pts_ok: Vec<Point3> = pts.iter().copied().filter(surfacecraft_geom::finite).collect();
    if pts_ok.len() != pts.len() {
        return None;
    }
    match kind {
        PrimitiveKind::Plane => fit_plane(pts),
        PrimitiveKind::Sphere => fit_sphere(pts),
        PrimitiveKind::Cylinder => fit_cylinder(pts, normals),
        PrimitiveKind::Cone => fit_cone(pts, normals),
        PrimitiveKind::Torus => fit_torus(pts, normals),
    }
}

//! Automatic primitive type: fit every kind, keep the simplest whose residual is close to the
//! best one (design doc §6.2: "auto-detection by residual"). A more general model always fits
//! at least as well, so "close" is relative: within 20 % of the best RMS (plus a floor for
//! noise-free data). Degenerate results of the general models are dropped and explained: a cone
//! with a tiny half angle is a cylinder, a cylinder or sphere with a huge radius is a plane.
use crate::primitive::{Primitive, PrimitiveFit, PrimitiveKind, fit};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Aabb, Point3, Vec3};

/// Result of an automatic fit.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct AutoFit {
    /// The chosen primitive.
    pub best: PrimitiveFit,
    /// Every kind that could be fitted, with its RMS.
    pub candidates: Vec<(PrimitiveKind, f64)>,
    /// Why a more general kind was not chosen, when that is worth telling.
    pub note: Option<String>,
}

/// Relative closeness to the best RMS for a simpler model to win.
pub const RELATIVE: f64 = 1.2;

fn degenerate(f: &PrimitiveFit, extent: f64) -> Option<String> {
    match &f.primitive {
        Primitive::Cone { half_angle, .. } if half_angle.to_degrees() < 0.2 => Some(format!(
            "cone half-angle {:.3}° is too small: it is more suitable as a cylinder",
            half_angle.to_degrees()
        )),
        Primitive::Cone { half_angle, .. } if half_angle.to_degrees() > 88.0 => {
            Some("cone half-angle near 90°: it is more suitable as a plane".into())
        }
        Primitive::Cylinder { radius, .. } if *radius > 50.0 * extent => {
            Some("cylinder radius is far larger than the region: it is more suitable as a plane".into())
        }
        Primitive::Sphere { radius, .. } if *radius > 50.0 * extent => {
            Some("sphere radius is far larger than the region: it is more suitable as a plane".into())
        }
        Primitive::Torus { major_radius, minor_radius, .. } if *minor_radius >= major_radius.abs() => {
            Some("torus tube reaches the axis (spindle torus): not used".into())
        }
        Primitive::Torus { major_radius, .. } if *major_radius > 50.0 * extent => {
            Some("torus major radius is far larger than the region: it is more suitable as a cylinder".into())
        }
        _ => None,
    }
}

/// Fits every kind and picks one.
#[must_use]
pub fn fit_auto(pts: &[Point3], normals: &[Vec3]) -> Option<AutoFit> {
    let extent = Aabb::from_points(pts).diagonal().max(1e-9);
    let mut fits: Vec<PrimitiveFit> = Vec::new();
    let mut notes: Vec<String> = Vec::new();
    for k in PrimitiveKind::ALL {
        if let Some(f) = fit(k, pts, normals) {
            match degenerate(&f, extent) {
                Some(n) => notes.push(n),
                None => fits.push(f),
            }
        }
    }
    let best_rms = fits.iter().map(|f| f.rms).fold(f64::INFINITY, f64::min);
    if !best_rms.is_finite() {
        return None;
    }
    let floor = 1e-6 * extent.max(1.0);
    let chosen = fits.iter().find(|f| f.rms <= best_rms * RELATIVE + floor)?.clone();
    let candidates = fits.iter().map(|f| (f.primitive.kind(), f.rms)).collect();
    let simplest_better =
        fits.iter().filter(|f| f.primitive.kind() > chosen.primitive.kind()).map(|f| f.primitive.kind()).next();
    let note = notes.into_iter().next().or_else(|| {
        simplest_better.map(|k| format!("{k:?} fits only marginally better; {:?} chosen", chosen.primitive.kind()))
    });
    Some(AutoFit { best: chosen, candidates, note })
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Mesh, Rng, synth};

    fn noisy(mut m: Mesh, sigma: f64, seed: u64) -> (Vec<Point3>, Vec<Vec3>) {
        let n = m.vertex_normals();
        synth::add_noise(&mut m, sigma, &mut Rng::new(seed));
        (m.positions, n)
    }

    /// Gate M2: the automatic type is right for every kind on noisy synthetic data.
    #[test]
    fn gate_m2_auto_type_is_correct() {
        let axis = Vec3::new(0.3, -0.2, 1.0).normalize();
        let cases: Vec<(PrimitiveKind, Mesh)> = vec![
            (PrimitiveKind::Plane, synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 40.0, 30.0, 40, 30)),
            (PrimitiveKind::Sphere, synth::sphere(Point3::new(3.0, 4.0, 5.0), 15.0, 64, 32)),
            (PrimitiveKind::Cylinder, synth::cylinder(Point3::origin(), axis, 8.0, 30.0, 64, 20)),
            (PrimitiveKind::Cone, synth::cone(Point3::origin(), axis, 12.0, 5.0, 20.0, 64, 20)),
            (PrimitiveKind::Torus, synth::torus(Point3::origin(), axis, 20.0, 5.0, 96, 32)),
        ];
        for (seed, (kind, mesh)) in cases.into_iter().enumerate() {
            let (p, n) = noisy(mesh, 0.05, seed as u64 + 1);
            let a = fit_auto(&p, &n).unwrap();
            eprintln!(
                "gate_m2 auto {kind:?}: chose {:?} rms {:.4} candidates {:?}",
                a.best.primitive.kind(),
                a.best.rms,
                a.candidates
            );
            assert_eq!(a.best.primitive.kind(), kind, "{a:?}");
            assert!(a.best.rms <= 0.1);
        }
    }

    #[test]
    fn nearly_cylindrical_cone_is_a_cylinder() {
        // half-angle 0.05°: indistinguishable from a cylinder under noise
        let h = 30.0;
        let r0 = 8.0;
        let r1 = r0 + h * (0.05f64.to_radians()).tan();
        let (p, n) = noisy(synth::cone(Point3::origin(), Vec3::z(), r0, r1, h, 64, 20), 0.05, 9);
        let a = fit_auto(&p, &n).unwrap();
        assert_eq!(a.best.primitive.kind(), PrimitiveKind::Cylinder, "{a:?}");
    }
}

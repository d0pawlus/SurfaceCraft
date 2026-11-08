//! Display tessellation of records through `solidcore-tess` (design doc §5: render B-rep via tess).
use crate::record::brep::Brep;
use crate::{Result, op_err};
use surfacecraft_geom::{Mesh, Point3};

/// A display mesh of `body` with chord tolerance relative to its size (`None` = default).
/// Faces the mesher refuses are left out; the rest is meshed.
pub fn tessellate(body: &Brep, chord: Option<f64>) -> Result<Mesh> {
    let size = bbox_size(body).max(1e-3);
    let tol = match chord {
        Some(c) if c.is_finite() && c > 0.0 => solidcore_tess::MeshTolerance::display(c),
        _ => solidcore_tess::MeshTolerance::display_of(size),
    };
    let (plan, _stats) = solidcore_tess::plan(body, &tol, size).map_err(|e| op_err("tess::plan", e))?;
    let (m, _refused) = plan.evaluate();
    let positions: Vec<Point3> = m.positions.iter().map(|p| Point3::new(p.x, p.y, p.z)).collect();
    let mut mesh = Mesh { positions, triangles: m.triangles };
    // the mesher never emits bad indices, but the record came from outside: validate anyway
    if mesh.validate().is_err() {
        mesh = Mesh::default();
    }
    Ok(mesh)
}

fn bbox_size(body: &Brep) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for v in &body.vertices {
        let p = v.point;
        for i in 0..3 {
            if let (Some(l), Some(h), Some(x)) = (lo.get_mut(i), hi.get_mut(i), p.get(i)) {
                *l = l.min(*x);
                *h = h.max(*x);
            }
        }
    }
    let d: f64 = (0..3).map(|i| (hi[i] - lo[i]).powi(2)).sum::<f64>().sqrt();
    if d.is_finite() { d } else { 1.0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_tessellates_closed() {
        let b = crate::solid::make_box([0.0; 3], [20.0, 10.0, 5.0]).unwrap();
        let m = tessellate(&b, None).unwrap();
        assert!(m.triangle_count() >= 12);
        assert!((m.area() - 2.0 * (200.0 + 100.0 + 50.0)).abs() < 1e-6);
    }
}

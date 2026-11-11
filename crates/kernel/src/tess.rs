//! Display tessellation of records through `solidcore-tess` (design doc §5: render B-rep via tess).
use crate::record::brep::Brep;
use crate::{Result, op_err};
use surfacecraft_geom::{Mesh, Point3};

/// A display mesh of `body` with chord tolerance relative to its size (`None` = default).
/// Faces the mesher refuses are left out; the rest is meshed.
pub fn tessellate(body: &Brep, chord: Option<f64>) -> Result<Mesh> {
    tessellate_topo(body, chord).map(|t| t.mesh)
}

/// Like [`tessellate`], but an error when the mesher refused a face (the mesh would silently lack
/// it): for callers that measure or deliver the mesh rather than draw it.
pub fn tessellate_all(body: &Brep, chord: Option<f64>) -> Result<Mesh> {
    let size = bbox_size(body).max(1e-3);
    let tol = match chord {
        Some(c) if c.is_finite() && c > 0.0 => solidcore_tess::MeshTolerance::display(c),
        _ => solidcore_tess::MeshTolerance::display_of(size),
    };
    let (plan, _stats) = solidcore_tess::plan(body, &tol, size).map_err(|e| op_err("tess::plan", e))?;
    let (m, refused) = plan.evaluate();
    if !refused.is_empty() {
        return Err(op_err("tess::evaluate", format!("the mesher refused {} face(s) of the sheet", refused.len())));
    }
    let positions: Vec<Point3> = m.positions.iter().map(|p| Point3::new(p.x, p.y, p.z)).collect();
    let mesh = Mesh { positions, triangles: m.triangles };
    mesh.validate().map_err(|e| op_err("tess::evaluate", e.to_string()))?;
    Ok(mesh)
}

/// A display mesh together with the B-rep identities the viewport picks by.
#[derive(Clone, Debug, Default)]
pub struct TopoMesh {
    /// The display mesh.
    pub mesh: Mesh,
    /// B-rep face index (as in the body record) of every triangle of `mesh` (empty when the
    /// mesher gave none).
    pub tri_face: Vec<u32>,
    /// B-rep edges as polylines, in the body's edge order (empty for an edge that could not be
    /// sampled).
    pub edges: Vec<Vec<Point3>>,
}

/// Like [`tessellate`], and keeps which face each triangle belongs to and the edge polylines, so
/// the viewport can pick faces and edges of the body and hand their indices to commands.
pub fn tessellate_topo(body: &Brep, chord: Option<f64>) -> Result<TopoMesh> {
    let size = bbox_size(body).max(1e-3);
    let tol = match chord {
        Some(c) if c.is_finite() && c > 0.0 => solidcore_tess::MeshTolerance::display(c),
        _ => solidcore_tess::MeshTolerance::display_of(size),
    };
    let (plan, _stats) = solidcore_tess::plan(body, &tol, size).map_err(|e| op_err("tess::plan", e))?;
    let (m, _refused) = plan.evaluate();
    let positions: Vec<Point3> = m.positions.iter().map(|p| Point3::new(p.x, p.y, p.z)).collect();
    let mut mesh = Mesh { positions, triangles: m.triangles };
    let mut tri_face = m.tri_face;
    // the mesher never emits bad indices, but the record came from outside: validate anyway
    if mesh.validate().is_err() {
        mesh = Mesh::default();
    }
    if tri_face.len() != mesh.triangles.len() {
        tri_face.clear();
    }
    let edges = m.edges.iter().map(|e| e.iter().map(|p| Point3::new(p.x, p.y, p.z)).collect()).collect();
    Ok(TopoMesh { mesh, tri_face, edges })
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

    #[test]
    fn box_topology_names_six_faces_and_twelve_edges() {
        let b = crate::solid::make_box([0.0; 3], [20.0, 10.0, 5.0]).unwrap();
        let t = tessellate_topo(&b, None).unwrap();
        assert_eq!(t.tri_face.len(), t.mesh.triangle_count());
        let mut faces = t.tri_face.clone();
        faces.sort_unstable();
        faces.dedup();
        assert_eq!(faces.len(), 6);
        assert_eq!(t.edges.iter().filter(|e| e.len() >= 2).count(), 12);
    }
}

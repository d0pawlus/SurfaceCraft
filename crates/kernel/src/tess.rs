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
    let (mut mesh, mut tri_face) = evaluate_plan(&plan);
    // the mesher never emits bad indices, but the record came from outside: validate anyway
    if mesh.validate().is_err() {
        mesh = Mesh::default();
    }
    if tri_face.len() != mesh.triangles.len() {
        tri_face.clear();
    }
    let edges = plan.edges.iter().map(|e| e.iter().map(|p| Point3::new(p.x, p.y, p.z)).collect()).collect();
    Ok(TopoMesh { mesh, tri_face, edges })
}

/// Faces evaluated per block: the temporary vectors of a block are dropped before the next one
/// starts, so the evaluation holds a block of faces next to the mesh, not a second copy of it.
const FACES_PER_BLOCK: usize = 2048;

/// The vertices and triangles of one face of a plan: what `TessPlan::evaluate` makes of it, with
/// the triangle indices local to the face and without the normals (which nothing here reads).
struct FaceMesh {
    positions: Vec<Point3>,
    triangles: Vec<[u32; 3]>,
}

fn evaluate_face(plan: &solidcore_tess::TessPlan, f: &solidcore_tess::FacePlan) -> FaceMesh {
    let surface = f.surface.and_then(|i| plan.surfaces.get(i as usize));
    let positions =
        f.uv.iter()
            .enumerate()
            .map(|(k, uv)| {
                let p = f
                    .fixed
                    .get(k)
                    .copied()
                    .or_else(|| surface.map(|s| solidcore_convert::record_pair(s.eval(uv[0], uv[1])).0))
                    .unwrap_or(solidcore_record::Vec3::ZERO);
                Point3::new(p.x, p.y, p.z)
            })
            .collect();
    // a triangle that names a vertex the face does not have is left out, as `evaluate` does
    let triangles = f.triangles.iter().filter(|t| t.iter().all(|&i| (i as usize) < f.uv.len())).copied().collect();
    FaceMesh { positions, triangles }
}

/// The display mesh of a plan and the B-rep face of every triangle: the vertices and triangles
/// that `TessPlan::evaluate` gives, bit for bit, evaluated face by face in parallel (faces are
/// independent and the blocks are joined in face order, so the result does not depend on the
/// threads) and without building the mesher's own mesh (which also holds normals and a second
/// copy of the positions).
fn evaluate_plan(plan: &solidcore_tess::TessPlan) -> (Mesh, Vec<u32>) {
    let vertices = plan.vertex_count();
    let mut positions: Vec<Point3> = Vec::with_capacity(vertices);
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    let mut tri_face: Vec<u32> = Vec::new();
    for block in plan.faces.chunks(FACES_PER_BLOCK) {
        let meshes = surfacecraft_geom::par::map(block.len(), |i| block.get(i).map(|f| evaluate_face(plan, f)));
        for (f, m) in block.iter().zip(meshes) {
            let Some(m) = m else { continue };
            let base = positions.len();
            positions.extend(m.positions);
            let at = |i: u32| u32::try_from(base + i as usize).unwrap_or(u32::MAX);
            tri_face.extend(std::iter::repeat_n(f.face, m.triangles.len()));
            triangles.extend(m.triangles.iter().map(|t| [at(t[0]), at(t[1]), at(t[2])]));
        }
    }
    (Mesh { positions, triangles }, tri_face)
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

    /// The mesh of the plan as the mesher's own `evaluate` gives it: the reference of the faster
    /// evaluation (bit for bit).
    fn reference(plan: &solidcore_tess::TessPlan) -> (Vec<[f64; 3]>, Vec<[u32; 3]>, Vec<u32>) {
        let (m, refused) = plan.evaluate();
        assert!(refused.is_empty());
        (m.positions.iter().map(|p| [p.x, p.y, p.z]).collect(), m.triangles, m.tri_face)
    }

    fn assert_same_as_evaluate(body: &Brep, chord: f64) {
        let size = bbox_size(body).max(1e-3);
        let tol = solidcore_tess::MeshTolerance::display(chord);
        let (plan, _) = solidcore_tess::plan(body, &tol, size).unwrap();
        let (want_p, want_t, want_f) = reference(&plan);
        let (mesh, tri_face) = evaluate_plan(&plan);
        let got_p: Vec<[f64; 3]> = mesh.positions.iter().map(|p| [p.x, p.y, p.z]).collect();
        assert!(got_p.len() > 8, "a trivial mesh proves nothing");
        assert!(
            got_p.iter().flatten().zip(want_p.iter().flatten()).all(|(a, b)| a.to_bits() == b.to_bits()),
            "positions differ"
        );
        assert_eq!(got_p.len(), want_p.len());
        assert_eq!(mesh.triangles, want_t);
        assert_eq!(tri_face, want_f);
    }

    #[test]
    fn parallel_evaluation_is_bit_identical_to_the_meshers_on_primitives_and_blends() {
        let b = crate::solid::make_box([0.0; 3], [20.0, 10.0, 5.0]).unwrap();
        assert_same_as_evaluate(&b, 0.01);
        let s = crate::solid::make_sphere([1.0, 2.0, 3.0], 7.0).unwrap();
        assert_same_as_evaluate(&s, 0.01);
        let c = crate::solid::make_cylinder([0.0; 3], 4.0, 9.0).unwrap();
        assert_same_as_evaluate(&c, 0.005);
        let edges: Vec<usize> = (0..4).collect();
        let f = crate::solid::fillet(&b, &edges, 1.5).unwrap();
        assert_same_as_evaluate(&f, 0.01);
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

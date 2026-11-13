//! `tessellate_topo` evaluates the faces of the mesher's plan in parallel blocks and builds the
//! display mesh itself; its output is the mesher's own `TessPlan::evaluate`, bit for bit, at any
//! thread count (P9 performance).
mod common;

use common::{cube_sphere, n_for};
use surfacecraft_kernel::nurbs::sheet_from_bezier_network_as_is;
use surfacecraft_kernel::tess::tessellate_topo;

const CHORD: f64 = 0.05;

/// More faces than one block of the evaluation (2048 faces), so the join of the blocks is used.
const PATCHES: usize = 6_000;

fn bits(p: &[f64; 3]) -> [u64; 3] {
    p.map(f64::to_bits)
}

#[test]
fn a_network_of_several_blocks_tessellates_as_the_meshers_evaluate() {
    let net = cube_sphere(n_for(PATCHES), 40.0);
    let body = sheet_from_bezier_network_as_is(&net.patches, &net.corners).unwrap();
    let topo = tessellate_topo(&body, Some(CHORD)).unwrap();
    // the reference: the mesher's plan evaluated by the mesher
    let (plan, _) = {
        let tol = solidcore_tess::MeshTolerance::display(CHORD);
        solidcore_tess::plan(&body, &tol, bbox_diagonal(&body)).unwrap()
    };
    let (m, refused) = plan.evaluate();
    assert!(refused.is_empty());
    assert!(body.faces.len() > 2 * 2048);
    assert_eq!(topo.mesh.positions.len(), m.positions.len());
    for (a, b) in topo.mesh.positions.iter().zip(&m.positions) {
        assert_eq!(bits(&[a.x, a.y, a.z]), bits(&[b.x, b.y, b.z]));
    }
    assert_eq!(topo.mesh.triangles, m.triangles);
    assert_eq!(topo.tri_face, m.tri_face);
    assert_eq!(topo.edges.len(), m.edges.len());
}

/// The size the kernel passes to the mesher: the diagonal of the vertices' box.
fn bbox_diagonal(body: &solidcore_record::brep::Brep) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for v in &body.vertices {
        for i in 0..3 {
            lo[i] = lo[i].min(v.point[i]);
            hi[i] = hi[i].max(v.point[i]);
        }
    }
    (0..3).map(|i| (hi[i] - lo[i]).powi(2)).sum::<f64>().sqrt().max(1e-3)
}

/// Runs `f` on a pool of `threads` threads.
fn on<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

#[test]
fn tessellation_is_bit_identical_on_one_and_on_eight_threads_and_between_runs() {
    let net = cube_sphere(n_for(PATCHES), 40.0);
    let body = sheet_from_bezier_network_as_is(&net.patches, &net.corners).unwrap();
    let run = |threads: usize| on(threads, || tessellate_topo(&body, Some(CHORD)).unwrap());
    let (one, eight, again) = (run(1), run(8), run(8));
    for other in [&eight, &again] {
        assert_eq!(one.mesh.positions.len(), other.mesh.positions.len());
        for (a, b) in one.mesh.positions.iter().zip(&other.mesh.positions) {
            assert_eq!(bits(&[a.x, a.y, a.z]), bits(&[b.x, b.y, b.z]));
        }
        assert_eq!(one.mesh.triangles, other.mesh.triangles);
        assert_eq!(one.tri_face, other.tri_face);
    }
}

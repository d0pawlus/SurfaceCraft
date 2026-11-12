use super::field::compat4;
use super::*;
use surfacecraft_geom::synth;

fn mismatch(l: &Level, o: &[Vec3]) -> f64 {
    let (mut bad, mut tot) = (0usize, 0usize);
    for i in 0..l.v.len() {
        for &j in &l.nb[i] {
            let (a, b) = compat4(&o[i], &l.n[i], &o[j as usize], &l.n[j as usize]);
            bad += usize::from(a.dot(&b) < 0.94);
            tot += 1;
        }
    }
    bad as f64 / tot.max(1) as f64
}

#[test]
fn plane_layout_is_a_grid() {
    let m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 20.0, 20.0, 24, 24);
    let (poly, rep) = retopo(&m, &RetopoOptions { target_faces: 25, ..RetopoOptions::default() }).unwrap();
    eprintln!("plane: {rep:?}, faces {}", poly.faces.len());
    assert_eq!(rep.layout, Layout::Field);
    assert!(poly.faces.iter().all(|f| f.len() == 4));
}

#[test]
fn orientation_field_smooths_on_a_plane_and_a_sphere() {
    let m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 30.0, 30.0, 20, 20);
    let l = level_of(&m);
    let mut o = init_orientation(&l);
    smooth_orientation(&l, &mut o, 50);
    assert!(mismatch(&l, &o) < 0.02, "plane {}", mismatch(&l, &o));
    let m = synth::sphere(Point3::origin(), 20.0, 40, 20);
    let l = level_of(&m);
    let mut o = init_orientation(&l);
    eprintln!("sphere init mismatch {:.3}", mismatch(&l, &o));
    smooth_orientation(&l, &mut o, 200);
    eprintln!("sphere smoothed mismatch {:.3}", mismatch(&l, &o));
    assert!(mismatch(&l, &o) < 0.2);
}

#[test]
fn sphere_layout_is_quad_dominant_and_closed() {
    let m = synth::sphere(Point3::origin(), 20.0, 80, 40);
    let (poly, rep) = retopo(&m, &RetopoOptions { target_faces: 150, ..RetopoOptions::default() }).unwrap();
    eprintln!("sphere: {rep:?}, faces {}", poly.faces.len());
    assert!(poly.faces.len() > 40);
    assert!(rep.euler_ok);
    // every vertex lies on the sphere (within the working-mesh chord)
    for v in &poly.vertices {
        assert!((v.coords.norm() - 20.0).abs() < 0.5);
    }
    let q = poly.split_to_quads(&|p, n| Some((*p, *n)));
    // closed 2-manifold: every edge used twice, sphere Euler characteristic
    assert!(q.boundary_edges().is_empty());
    assert_eq!(q.euler(), 2);
}

/// A layout of a plate that stops short of a notched boundary reaches the notch after conforming.
#[test]
fn an_open_layout_reaches_the_outline_and_a_closed_one_is_untouched() {
    let sphere = synth::sphere(Point3::origin(), 20.0, 80, 40);
    let (_, rep) = retopo(&sphere, &RetopoOptions { target_faces: 150, ..RetopoOptions::default() }).unwrap();
    assert_eq!(rep.boundary, ConformReport::default());
    // a plate with a rectangular bite out of one side
    let plate = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 40.0, 30.0, 56, 42);
    let keep: Vec<u32> = (0..plate.triangle_count() as u32)
        .filter(|&t| plate.face_centroid(t as usize).is_some_and(|c| !(c.x > 28.0 && (c.y - 15.0).abs() < 5.0)))
        .collect();
    let region = plate.submesh(&keep);
    let (poly, rep) = retopo(&region, &RetopoOptions { target_faces: 60, ..RetopoOptions::default() }).unwrap();
    eprintln!("notched plate: {:?}, faces {}", rep.boundary, poly.faces.len());
    assert_eq!(rep.boundary.conformed, 1, "{:?}", rep.boundary);
    assert_eq!(rep.boundary.skipped, 0);
    // every vertex of the region's outline corner area is reached: some layout vertex lies within
    // a fraction of the edge length of each corner of the bite and of the plate
    let corners =
        [(28.0, 10.0), (28.0, 20.0), (40.0, 10.0), (40.0, 20.0), (0.0, 0.0), (40.0, 0.0), (0.0, 30.0), (40.0, 30.0)];
    for (x, y) in corners {
        let near =
            poly.vertices.iter().map(|v| ((v.x - x).powi(2) + (v.y - y).powi(2)).sqrt()).fold(f64::INFINITY, f64::min);
        assert!(near < 0.5 * rep.edge_length, "corner ({x}, {y}): nearest layout vertex {near}");
    }
}

/// A polygon mesh with a vertex but no normals (hand-built) is refused by the relaxation, not indexed.
#[test]
fn relaxing_a_mesh_with_missing_normals_does_not_panic() {
    let mut p = PolyMesh { vertices: vec![Point3::origin()], normals: Vec::new(), faces: Vec::new() };
    p.relax(1, &|q, n| Some((*q, *n)));
    let mut q = QuadMesh { vertices: vec![Point3::origin()], normals: Vec::new(), faces: Vec::new() };
    q.relax(1, &|a, n| Some((*a, *n)));
    assert_eq!(p.vertices.len(), 1);
}

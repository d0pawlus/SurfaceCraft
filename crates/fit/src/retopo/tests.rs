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

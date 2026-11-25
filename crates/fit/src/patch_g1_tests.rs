use super::*;
use surfacecraft_geom::{Rng, synth};

fn wrap(coarse: &Mesh, scan: &Mesh, g1: bool) -> G1Wrap {
    let surface = ScanSurface::oriented(&scan.positions, &scan.vertex_normals());
    let bvh = Bvh::build(scan);
    wrap_patches_g1(coarse, &bvh, &surface, 4.0, &G1Options { g1, crease_deg: 35.0 })
}

fn sphere_pair() -> (Mesh, Mesh) {
    let mut scan = synth::sphere(Point3::origin(), 20.0, 120, 60);
    synth::add_noise(&mut scan, 0.03, &mut Rng::new(5));
    (synth::sphere(Point3::origin(), 20.0, 16, 8), scan)
}

#[test]
fn tangent_patches_on_a_sphere_meet_within_a_degree_and_g0_ones_do_not() {
    // Arrange
    let (coarse, scan) = sphere_pair();

    // Act
    let g1 = wrap(&coarse, &scan, true);
    let g0 = wrap(&coarse, &scan, false);
    let (tangent, _) = jump_stats(&g1.set, &g1.creases);
    let (free, _) = jump_stats(&g0.set, &g0.creases);

    // Assert
    assert_eq!(g1.crease_edges, 0, "a sphere has no creases");
    assert_eq!(g1.set.nets.len(), g0.set.nets.len());
    assert!(tangent.count > 0 && tangent.mean_deg < 1.0, "{tangent:?}");
    assert!(free.mean_deg > 2.0 * tangent.mean_deg, "g0 {free:?} g1 {tangent:?}");
}

#[test]
fn a_box_keeps_its_edges_sharp_and_its_faces_tangent() {
    // Arrange: a coarse box (two triangles per face) over a dense noisy one
    let mut scan = synth::cuboid(Point3::origin(), Vec3::new(20.0, 20.0, 20.0), 20);
    synth::add_noise(&mut scan, 0.02, &mut Rng::new(3));
    let coarse = synth::cuboid(Point3::origin(), Vec3::new(20.0, 20.0, 20.0), 2);

    // Act
    let w = wrap(&coarse, &scan, true);
    let (smooth, sharp) = jump_stats(&w.set, &w.creases);

    // Assert
    assert!(w.crease_edges >= 12, "the box edges are creases: {}", w.crease_edges);
    assert!(sharp.count > 0 && (sharp.mean_deg - 90.0).abs() < 3.0, "{sharp:?}");
    assert!(smooth.mean_deg < 1.0, "{smooth:?}");
}

fn rotated(deg: f64) -> Vec3 {
    let a = deg.to_radians();
    Vec3::new(a.sin(), 0.0, a.cos())
}

#[test]
fn a_shallow_crease_direction_lies_in_both_side_planes() {
    // Arrange: sides 5 degrees apart (far above rounding, below the old 11.5 degree cut-off)
    let (n1, n2) = (rotated(-2.5), rotated(2.5));

    // Act
    let d = crease_direction(&n1, &n2, &Vec3::y());

    // Assert
    assert!(d.dot(&n1).abs() < 1e-12 && d.dot(&n2).abs() < 1e-12, "{d:?}");
    assert!(d.dot(&Vec3::y()) > 0.99, "{d:?}");
}

#[test]
fn a_crease_direction_lies_in_both_planes_down_to_parallel_normals() {
    for deg in [180.0, 90.0, 1.0, 1e-3, 1e-8, 1e-12, 0.0] {
        let (n1, n2) = (rotated(0.0), rotated(deg));
        let d = crease_direction(&n1, &n2, &Vec3::y());
        assert!(d.dot(&n1).abs() < 1e-9 && d.dot(&n2).abs() < 1e-9, "{deg}: {d:?}");
        assert!((d.norm() - 1.0).abs() < 1e-12, "{deg}: {d:?}");
    }
}

#[test]
fn a_collapsed_patch_corner_does_not_follow_the_face() {
    // Arrange: the whole side u = 0 collapsed to one point, so the normal is undefined along it
    let mut net = [[[0.0; 3]; 4]; 4];
    for (i, row) in net.iter_mut().enumerate() {
        for (j, p) in row.iter_mut().enumerate() {
            *p = [i as f64, j as f64, 0.0];
        }
    }
    for p in net[0].iter_mut() {
        *p = [0.0, 0.0, 0.0];
    }

    // Act / Assert
    assert!(normal(&net, 0.0, 0.5).is_none());
    assert!(!follows(&net, &Vec3::z()), "an undefined normal must not be accepted");
}

/// A height field with a curved crease: `z = SLOPE |x - 1.5 sin(0.4 y)|`, whose two sides meet at
/// 5 degrees. `grid(n, step)` has its lines along the crease curve.
fn curved_crease_surface(nx: usize, ny: usize, step: f64) -> Mesh {
    const SLOPE: f64 = 0.043_66;
    let mut m = Mesh::default();
    for j in 0..=ny {
        for i in 0..=nx {
            let (u, y) = ((i as f64 - nx as f64 / 2.0) * step, j as f64 * step);
            let x = u + 1.5 * (0.4 * y).sin();
            m.positions.push(Point3::new(x, y, SLOPE * u.abs()));
        }
    }
    for j in 0..ny {
        for i in 0..nx {
            let a = (j * (nx + 1) + i) as u32;
            let w = (nx + 1) as u32;
            m.triangles.push([a, a + 1, a + w + 1]);
            m.triangles.push([a, a + w + 1, a + w]);
        }
    }
    m
}

#[test]
fn a_curved_shallow_crease_leaves_the_smooth_edges_beside_it_tangent() {
    // Arrange: crease_deg 1 makes the 5 degree crease a crease
    let scan = curved_crease_surface(40, 40, 0.5);
    let coarse = curved_crease_surface(4, 4, 5.0);
    let surface = ScanSurface::oriented(&scan.positions, &scan.vertex_normals());
    let bvh = Bvh::build(&scan);

    // Act
    let w = wrap_patches_g1(&coarse, &bvh, &surface, 4.0, &G1Options { g1: true, crease_deg: 1.0 });
    let (smooth, _) = jump_stats(&w.set, &w.creases);

    // Assert
    assert!(w.crease_edges >= 2, "the 5 degree crease is found: {}", w.crease_edges);
    assert_eq!((smooth.invalid_samples, smooth.invalid_edges), (0, 0), "{smooth:?}");
    assert!(
        smooth.count > 0 && smooth.mean_deg < 1.0 && smooth.p99_deg < 5.0,
        "smooth edges beside the crease: {smooth:?}"
    );
}

#[test]
fn the_wrap_is_deterministic() {
    let (coarse, scan) = sphere_pair();
    let (a, b) = (wrap(&coarse, &scan, true), wrap(&coarse, &scan, true));
    assert_eq!(a.set.nets, b.set.nets);
    assert_eq!(a.creases, b.creases);
}

#[test]
fn hostile_coarse_meshes_do_not_panic() {
    let (_, scan) = sphere_pair();
    let nan = Point3::new(f64::NAN, 0.0, 0.0);
    let meshes = [
        Mesh::default(),
        Mesh { positions: vec![Point3::origin(); 3], triangles: vec![[0, 1, 2]] },
        Mesh { positions: vec![nan, Point3::origin(), Point3::new(1.0, 0.0, 0.0)], triangles: vec![[0, 1, 2]] },
        Mesh { positions: vec![Point3::origin()], triangles: vec![[0, 5, 9]] },
    ];
    for m in &meshes {
        let w = wrap(m, &scan, true);
        assert!(w.set.nets.len() <= 3 * m.triangles.len());
    }
}

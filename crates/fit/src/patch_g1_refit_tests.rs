//! The least-squares refit inside the wrap: where it is used and where it is not.
use super::*;
use surfacecraft_geom::{Rng, synth};

const REACH: f64 = 4.0;

fn run(coarse: &Mesh, scan: &Mesh, g1: bool, refit: bool) -> G1Wrap {
    let surface = ScanSurface::oriented(&scan.positions, &scan.vertex_normals());
    let bvh = Bvh::build(scan);
    wrap_with(coarse, (&bvh, &surface), REACH, &G1Options { g1, crease_deg: 35.0 }, refit)
}

/// The upper half of a sphere of radius 20: an open sheet with one rim.
fn dome(nu: usize, nv: usize) -> Mesh {
    let ball = synth::sphere(Point3::origin(), 20.0, nu, nv);
    let keep: Vec<u32> = (0..ball.triangle_count())
        .filter(|&t| ball.face_centroid(t).is_some_and(|c| c.z > 0.0))
        .map(|t| t as u32)
        .collect();
    ball.submesh(&keep)
}

/// A noisy scan of the dome and a coarse mesh of it.
fn dome_pair() -> (Mesh, Mesh) {
    let mut scan = dome(160, 80);
    synth::add_noise(&mut scan, 0.03, &mut Rng::new(11));
    (dome(16, 8), scan)
}

/// Triangles of `coarse` with an edge that has one face only.
fn rim_triangles(coarse: &Mesh) -> Vec<usize> {
    let edges = edge_faces(coarse);
    let free: BTreeSet<EdgeKey> = edges.iter().filter(|(_, f)| f.len() != 2).map(|(k, _)| *k).collect();
    (0..coarse.triangle_count())
        .filter(|&f| (0..3).any(|e| coarse.triangles.get(f).is_some_and(|t| free.contains(&key(t[e], t[(e + 1) % 3])))))
        .collect()
}

/// The three patches of coarse triangle `f` (every triangle gives three, in order).
fn patches_of(w: &G1Wrap, f: usize) -> &[crate::patch::Net] {
    w.set.nets.get(3 * f..3 * f + 3).unwrap_or(&[])
}

#[test]
fn on_the_open_rim_curves_and_patches_are_the_plain_construction() {
    // Arrange
    let (coarse, scan) = dome_pair();
    let rim = rim_triangles(&coarse);
    assert!(!rim.is_empty() && rim.len() < coarse.triangle_count(), "{} of {}", rim.len(), coarse.triangle_count());

    // Act
    let refitted = run(&coarse, &scan, false, true);
    let plain = run(&coarse, &scan, false, false);

    // Assert: every patch of a rim triangle (its rim curve, its edges that end on the rim, its spokes
    // and its inside) is exactly the plain one; away from the rim the refit is at work
    assert_eq!(refitted.set.nets.len(), 3 * coarse.triangle_count());
    assert_eq!(refitted.set.nets.len(), plain.set.nets.len());
    for &f in &rim {
        assert_eq!(patches_of(&refitted, f), patches_of(&plain, f), "rim triangle {f}");
    }
    assert_ne!(refitted.set.nets, plain.set.nets, "the refit changes the patches away from the rim");
}

#[test]
fn tangent_patches_are_built_without_the_refit() {
    // Arrange
    let (coarse, scan) = dome_pair();

    // Act
    let with = run(&coarse, &scan, true, true);
    let without = run(&coarse, &scan, true, false);

    // Assert
    assert_eq!(with.set.nets, without.set.nets);
    assert_eq!(with.set.vertices, without.set.vertices);
}

#[test]
fn the_refitted_wrap_is_deterministic() {
    // Arrange
    let (coarse, scan) = dome_pair();

    // Act
    let (a, b) = (run(&coarse, &scan, false, true), run(&coarse, &scan, false, true));

    // Assert: bit for bit
    let bits = |w: &G1Wrap| w.set.nets.iter().flatten().flatten().flatten().map(|x| x.to_bits()).collect::<Vec<u64>>();
    assert_eq!(bits(&a), bits(&b));
}

#[test]
fn on_a_thin_wall_no_refitted_point_moves_to_the_other_side() {
    // Arrange: a plate 0.6 mm thick (both faces within the reach of each other), scanned with noise
    const THICK: f64 = 0.6;
    let mut scan = synth::cuboid(Point3::origin(), Vec3::new(30.0, 30.0, THICK), 60);
    scan.weld(1e-9);
    synth::add_noise(&mut scan, 0.02, &mut Rng::new(4));
    let mut coarse = synth::cuboid(Point3::origin(), Vec3::new(30.0, 30.0, THICK), 3);
    coarse.weld(1e-9);

    // Act
    let refitted = run(&coarse, &scan, false, true);
    let plain = run(&coarse, &scan, false, false);

    // Assert: the patches of the top face (all corners on it) keep every control point nearer the
    // top than the bottom, and the bottom ones likewise
    let corner_z = |w: &G1Wrap, k: usize| -> Vec<f64> {
        w.set.corners[k].iter().map(|&v| w.set.vertices[v as usize][2]).collect()
    };
    let mut checked = 0;
    for (k, net) in refitted.set.nets.iter().enumerate() {
        let zs = corner_z(&refitted, k);
        let top = zs.iter().all(|z| *z > 0.9 * THICK);
        let bottom = zs.iter().all(|z| *z < 0.1 * THICK);
        for p in net.iter().flatten() {
            assert!(!top || p[2] > 0.5 * THICK, "top patch {k} has a control point at z {}", p[2]);
            assert!(!bottom || p[2] < 0.5 * THICK, "bottom patch {k} has a control point at z {}", p[2]);
        }
        checked += usize::from(top || bottom);
    }
    assert!(checked > 0);
    assert_ne!(refitted.set.nets, plain.set.nets, "the refit is at work on the plate");
}

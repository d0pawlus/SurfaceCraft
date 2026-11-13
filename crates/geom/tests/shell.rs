//! Thickening open meshes into closed shells (`surfacecraft_geom::shell`): volumes against exact
//! shells, closedness, orientation, folds, windows, signs, determinism and hostile input.
use surfacecraft_geom::quality::edge_census;
use surfacecraft_geom::shell::{ScanLayer, ThickenError, ThickenParams, thicken};
use surfacecraft_geom::{Bvh, Mesh, Point3, Rng, Vec3, synth, tri};

const RADIUS: f64 = 30.0;

fn params(thickness: f64) -> ThickenParams {
    ThickenParams { thickness, symmetric: false }
}

/// Signed volume of a closed oriented mesh.
fn volume(m: &Mesh) -> f64 {
    m.triangles
        .iter()
        .map(|t| {
            let (a, b, c) = (m.positions[t[0] as usize], m.positions[t[1] as usize], m.positions[t[2] as usize]);
            a.coords.dot(&b.coords.cross(&c.coords)) / 6.0
        })
        .sum()
}

/// Closed (every edge has its opposite), manifold, outward (positive volume).
fn assert_closed_outward(m: &Mesh) {
    let c = edge_census(m);
    assert_eq!((c.boundary, c.non_manifold), (0, 0), "boundary or non-manifold edges: {c:?}");
    let mut directed: Vec<(u32, u32)> =
        m.triangles.iter().flat_map(|t| [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]).collect();
    directed.sort_unstable();
    assert!(directed.windows(2).all(|w| w[0] != w[1]), "an edge is used twice in one direction");
    assert!(directed.iter().all(|&(a, b)| directed.binary_search(&(b, a)).is_ok()), "an edge has no partner");
    assert!(volume(m) > 0.0, "the shell is inside out");
}

/// Euler characteristic V - E + F of a mesh over its used vertices.
fn euler(m: &Mesh) -> i64 {
    let used: std::collections::BTreeSet<u32> = m.triangles.iter().flatten().copied().collect();
    used.len() as i64 - edge_census(m).edges as i64 + m.triangles.len() as i64
}

/// Half width of the square region (mm) the cap covers when seen from above.
const CAP_HALF: f64 = 18.0;
/// Cells along the side of the cap (about 0.8 mm each).
const CAP_CELLS: usize = 45;

fn sphere_z(x: f64, y: f64) -> f64 {
    (RADIUS * RADIUS - x * x - y * y).sqrt()
}

/// Solid angle (sr) of the cone of directions through the square region of the cap.
fn cap_solid_angle() -> f64 {
    let n = 600;
    let h = 2.0 * CAP_HALF / n as f64;
    let mut omega = 0.0;
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (-CAP_HALF + (i as f64 + 0.5) * h, -CAP_HALF + (j as f64 + 0.5) * h);
            omega += h * h / (RADIUS * sphere_z(x, y));
        }
    }
    omega
}

/// A square cap of a sphere (outward normals, uniform sampling), noisy, with a few holes. Also the
/// area removed by the holes.
fn noisy_cap(sigma: f64, holes: usize) -> (Mesh, f64) {
    let mut cap = synth::plane(
        Point3::new(-CAP_HALF, -CAP_HALF, 0.0),
        Vec3::x(),
        Vec3::y(),
        2.0 * CAP_HALF,
        2.0 * CAP_HALF,
        CAP_CELLS,
        CAP_CELLS,
    );
    for p in &mut cap.positions {
        p.z = sphere_z(p.x, p.y);
    }
    let mut rng = Rng::new(7);
    let centres: Vec<Point3> = cap.sample_points(holes, &mut rng).into_iter().map(|(p, _)| p).collect();
    let kept: Vec<u32> = (0..cap.triangle_count() as u32)
        .filter(|&t| cap.face_centroid(t as usize).is_some_and(|c| centres.iter().all(|h| (c - h).norm() > 1.0)))
        .collect();
    let removed = cap.area() - cap.submesh(&kept).area();
    let mut cap = cap.submesh(&kept);
    synth::add_noise(&mut cap, sigma, &mut rng);
    (cap, removed)
}

/// Volume of the part of a spherical shell between radii `r0 < r1` inside the cone of the cap.
fn shell_volume(r0: f64, r1: f64) -> f64 {
    (r1.powi(3) - r0.powi(3)) / 3.0 * cap_solid_angle()
}

fn relative(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

#[test]
fn a_noisy_cap_with_holes_thickens_to_a_closed_outward_shell_of_the_exact_volume() {
    // Arrange
    let (cap, removed) = noisy_cap(0.03, 3);
    let t = 2.0;

    // Act
    let r = thicken(&cap, &params(t)).expect("thickens");

    // Assert
    assert_closed_outward(&r.mesh);
    let exact = shell_volume(RADIUS - t, RADIUS) - t * removed;
    assert!(relative(volume(&r.mesh), exact) < 0.02, "{} vs {exact}", volume(&r.mesh));
    let loops = surfacecraft_geom::edit::boundary_loops(&cap).len();
    assert!(loops >= 3, "the rim and at least two holes: {loops}");
    assert_eq!(r.report.boundary_loops, loops);
    assert_eq!(r.report.clamped_vertices, 0);
    assert_eq!(r.scan_layer, ScanLayer::Outer);
    assert_eq!(r.mesh.triangle_count(), r.report.triangles);
}

#[test]
fn a_wavy_panel_thickens_to_a_closed_outward_shell_of_thickness_times_area() {
    // Arrange: two sine waves, radius of curvature about 80 mm against a 2 mm wall
    let k = |x: f64, y: f64| 1.0 * (x / 9.0).sin() + 0.7 * (y / 7.0).cos();
    let mut panel = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 60.0, 40.0, 120, 80);
    for p in &mut panel.positions {
        p.z = k(p.x, p.y);
    }
    let mut rng = Rng::new(3);
    synth::add_noise(&mut panel, 0.03, &mut rng);
    let t = 2.0;

    // Act
    let r = thicken(&panel, &params(t)).expect("thickens");

    // Assert
    assert_closed_outward(&r.mesh);
    assert!(relative(volume(&r.mesh), t * panel.area()) < 0.03, "{} vs {}", volume(&r.mesh), t * panel.area());
    assert_eq!(r.report.clamped_vertices, 0);
    assert_eq!(r.report.boundary_loops, 1);
}

/// A flat panel with a ridge along y of height `h` and width `w` (Gaussian), upward normals.
fn ridge_panel(h: f64, w: f64) -> Mesh {
    let mut panel = synth::plane(Point3::new(-30.0, 0.0, 0.0), Vec3::x(), Vec3::y(), 60.0, 20.0, 300, 100);
    for p in &mut panel.positions {
        p.z = h * (-(p.x * p.x) / (2.0 * w * w)).exp();
    }
    panel
}

/// True when two triangles that share no vertex cross (an edge of one passes through the other).
fn self_intersections(m: &Mesh) -> usize {
    let bvh = Bvh::build(m);
    let corners = |t: usize| -> [Point3; 3] { m.triangles[t].map(|v| m.positions[v as usize]) };
    let crosses = |a: usize, b: usize| -> bool {
        let (ta, tb) = (corners(a), corners(b));
        (0..3).any(|k| {
            let (p, q) = (ta[k], ta[(k + 1) % 3]);
            tri::ray_hit(&p, &(q - p), &tb[0], &tb[1], &tb[2]).is_some_and(|s| s > 1e-6 && s < 1.0 - 1e-6)
        })
    };
    let mut count = 0;
    for a in 0..m.triangles.len() {
        let ta = corners(a);
        let bb = surfacecraft_geom::Aabb::from_points(ta.iter());
        let near = bvh
            .candidates(&|b: &surfacecraft_geom::Aabb| (0..3).all(|k| b.min[k] <= bb.max[k] && bb.min[k] <= b.max[k]));
        for b in near.into_iter().map(|b| b as usize).filter(|&b| b > a) {
            let shares = m.triangles[a].iter().any(|v| m.triangles[b].contains(v));
            if !shares && (crosses(a, b) || crosses(b, a)) {
                count += 1;
            }
        }
    }
    count
}

#[test]
fn a_tight_bead_on_the_offset_side_is_relaxed_locally_and_the_shell_does_not_intersect_itself() {
    // Arrange: radius of curvature at the crest 0.72 mm against a 2 mm wall
    let panel = ridge_panel(2.0, 1.2);

    // Act
    let r = thicken(&panel, &params(2.0)).expect("relaxed, not refused");

    // Assert
    assert_closed_outward(&r.mesh);
    assert!(r.report.clamped_vertices > 0, "the bead must be found");
    assert!((r.report.clamped_vertices as f64) < 0.15 * panel.vertex_count() as f64);
    assert!(r.report.largest_reduction > 0.5 && r.report.largest_reduction < 2.0, "{:?}", r.report);
    assert_eq!(self_intersections(&r.mesh), 0);
}

#[test]
fn the_same_bead_offset_the_other_way_needs_no_relaxing() {
    // Arrange
    let panel = ridge_panel(2.0, 1.2);

    // Act
    let r = thicken(&panel, &params(-2.0)).expect("thickens");

    // Assert
    assert_eq!(r.report.clamped_vertices, 0);
    assert_closed_outward(&r.mesh);
    assert_eq!(self_intersections(&r.mesh), 0);
}

#[test]
fn a_surface_with_radii_far_below_the_thickness_is_refused_with_the_share_and_a_suggestion() {
    // Arrange: a cap of a sphere of radius 1.5 mm against a 2 mm wall folds everywhere
    let full = synth::sphere(Point3::origin(), 1.5, 48, 24);
    let keep: Vec<u32> = (0..full.triangle_count() as u32)
        .filter(|&t| full.triangles[t as usize].iter().all(|&v| full.positions[v as usize].z > 0.2))
        .collect();
    let cap = full.submesh(&keep);

    // Act
    let err = thicken(&cap, &params(2.0)).expect_err("refused");

    // Assert
    match err {
        ThickenError::Folds { share_percent, suggested, thickness, .. } => {
            assert!(share_percent > 15.0, "{share_percent}");
            assert!(suggested < thickness && suggested > 0.0);
        }
        other => panic!("{other:?}"),
    }
    assert!(thicken(&cap, &params(0.5)).is_ok(), "the radius is large enough for a thinner wall");
}

#[test]
fn a_panel_with_a_window_has_two_loops_stitched_and_the_genus_of_a_torus() {
    // Arrange: 40 x 40 mm, 1 mm cells, a 10 x 10 mm window
    let plate = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 40.0, 40.0, 40, 40);
    let keep: Vec<u32> = (0..plate.triangle_count() as u32)
        .filter(|&t| {
            let c = plate.face_centroid(t as usize).unwrap();
            !((15.0..25.0).contains(&c.x) && (15.0..25.0).contains(&c.y))
        })
        .collect();
    let panel = plate.submesh(&keep);

    // Act
    let r = thicken(&panel, &params(2.0)).expect("thickens");

    // Assert
    assert_closed_outward(&r.mesh);
    assert_eq!(r.report.boundary_loops, 2);
    assert_eq!(euler(&r.mesh), 0, "a torus has Euler characteristic 0");
    assert!(relative(volume(&r.mesh), 2.0 * (1600.0 - 100.0)) < 1e-9, "{}", volume(&r.mesh));
}

/// Signed distance of `p` to the closed shell (negative inside), by the nearest triangle's normal.
fn signed_distance(bvh: &Bvh, p: &Point3) -> f64 {
    let h = bvh.closest(p, f64::INFINITY).expect("a hit");
    let n = bvh.triangle_normal(h.triangle).expect("normal");
    if (p - h.point).dot(&n) >= 0.0 { h.distance } else { -h.distance }
}

#[test]
fn negative_symmetric_and_positive_thickness_put_the_material_on_the_expected_side() {
    // Arrange
    let (cap, _) = noisy_cap(0.0, 0);
    let normals = cap.vertex_normals();
    let ring = shell_volume;
    // (thickness, symmetric, material radii, scan layer)
    let cases = [
        (2.0, false, (RADIUS - 2.0, RADIUS), ScanLayer::Outer),
        (-2.0, false, (RADIUS, RADIUS + 2.0), ScanLayer::Inner),
        (2.0, true, (RADIUS - 1.0, RADIUS + 1.0), ScanLayer::Outer),
        (-2.0, true, (RADIUS - 1.0, RADIUS + 1.0), ScanLayer::Outer),
    ];
    for (thickness, symmetric, (lo, hi), layer) in cases {
        // Act
        let r = thicken(&cap, &ThickenParams { thickness, symmetric }).expect("thickens");

        // Assert: volume
        assert_closed_outward(&r.mesh);
        assert!(relative(volume(&r.mesh), ring(lo, hi)) < 0.02, "{thickness} {symmetric}: {}", volume(&r.mesh));
        assert_eq!(r.scan_layer, layer);
        // the scan's own face is on the shell (or half a wall inside it for symmetric)
        let bvh = Bvh::build(&r.mesh);
        let z_mid = RADIUS * 0.9;
        let sample: Vec<usize> = (0..cap.vertex_count()).filter(|&i| cap.positions[i].z > z_mid).step_by(37).collect();
        assert!(!sample.is_empty());
        for i in sample {
            let (p, n) = (cap.positions[i], normals[i]);
            let behind = signed_distance(&bvh, &(p - n * 0.5));
            let front = signed_distance(&bvh, &(p + n * 0.5));
            match (thickness > 0.0, symmetric) {
                (true, false) => {
                    assert!((behind + 0.5).abs() < 0.02 && (front - 0.5).abs() < 0.02, "t>0: {behind} {front}")
                }
                (false, false) => {
                    assert!((front + 0.5).abs() < 0.02 && (behind - 0.5).abs() < 0.02, "t<0: {behind} {front}")
                }
                (_, true) => assert!(behind < -0.4 && front < -0.4, "symmetric: {behind} {front}"),
            }
        }
        let face = r.scan_face();
        assert_eq!(face.vertex_count(), cap.vertex_count());
        assert_eq!(face.triangle_count(), cap.triangle_count());
        if layer == ScanLayer::Inner {
            let (a, b) = (face.vertex_normals()[0], normals[0]);
            assert!(a.dot(&b) < -0.9, "the scan face of a negative thickness is wound against the scan's normals");
        }
    }
}

#[test]
fn the_result_is_bit_identical_for_one_and_eight_threads() {
    // Arrange
    let (cap, _) = noisy_cap(0.03, 3);
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| thicken(&cap, &params(2.0)).unwrap())
    };

    // Act
    let (one, eight) = (run(1), run(8));

    // Assert
    assert_eq!(one.mesh.triangles, eight.mesh.triangles);
    let bits =
        |m: &Mesh| m.positions.iter().flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).collect::<Vec<_>>();
    assert_eq!(bits(&one.mesh), bits(&eight.mesh));
    assert_eq!(one.report, eight.report);
}

#[test]
fn a_single_triangle_becomes_a_prism_of_the_exact_volume() {
    // Arrange
    let m = Mesh {
        positions: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, 0.0, 0.0), Point3::new(0.0, 3.0, 0.0)],
        triangles: vec![[0, 1, 2]],
    };

    // Act
    let r = thicken(&m, &params(1.0)).expect("thickens");

    // Assert
    assert_closed_outward(&r.mesh);
    assert!(relative(volume(&r.mesh), 6.0) < 1e-12);
    assert_eq!(r.report.boundary_loops, 1);
}

#[test]
fn boundary_loops_that_touch_at_a_vertex_are_split_and_stitched() {
    // Arrange: two triangles that share only the vertex 0
    let p = |x: f64, y: f64| Point3::new(x, y, 0.0);
    let m = Mesh {
        positions: vec![p(0.0, 0.0), p(2.0, 0.0), p(0.0, 2.0), p(-2.0, 0.0), p(0.0, -2.0)],
        triangles: vec![[0, 1, 2], [0, 3, 4]],
    };

    // Act
    let r = thicken(&m, &params(1.0)).expect("thickens");

    // Assert
    assert_closed_outward(&r.mesh);
    assert_eq!(r.report.pinch_splits, 1);
    assert_eq!(r.report.boundary_loops, 2);
    assert!(relative(volume(&r.mesh), 4.0) < 1e-12);
}

#[test]
fn degenerate_triangles_are_dropped_and_counted() {
    // Arrange
    let (cap, _) = noisy_cap(0.0, 0);
    let mut m = cap.clone();
    m.triangles.push([0, 0, 1]);
    m.triangles.push([2, 2, 2]);

    // Act
    let r = thicken(&m, &params(1.0)).expect("thickens");

    // Assert
    assert_eq!(r.report.dropped_triangles, 2);
    assert_closed_outward(&r.mesh);
}

#[test]
fn hostile_meshes_and_thicknesses_are_errors_not_panics() {
    // Arrange
    let (cap, _) = noisy_cap(0.0, 0);
    let closed = synth::sphere(Point3::origin(), 5.0, 16, 8);
    let nan_vertex = Mesh {
        positions: vec![Point3::new(f64::NAN, 0.0, 0.0), Point3::origin(), Point3::new(1.0, 1.0, 0.0)],
        triangles: vec![[0, 1, 2]],
    };
    let out_of_range =
        Mesh { positions: vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)], triangles: vec![[0, 1, 7]] };
    let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
    let three_on_an_edge = Mesh {
        positions: vec![p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0), p(0.0, -1.0, 0.0), p(0.0, 0.0, 1.0)],
        triangles: vec![[0, 1, 2], [1, 0, 3], [0, 1, 4]],
    };
    let flipped = Mesh {
        positions: vec![p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0), p(1.0, 1.0, 0.0)],
        triangles: vec![[0, 1, 2], [1, 2, 3]],
    };
    let flat = Mesh { positions: vec![p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0)], triangles: vec![[0, 1, 1]] };

    // Act and assert
    assert_eq!(thicken(&Mesh::default(), &params(1.0)).unwrap_err(), ThickenError::Empty);
    assert_eq!(thicken(&flat, &params(1.0)).unwrap_err(), ThickenError::Empty);
    assert_eq!(thicken(&closed, &params(1.0)).unwrap_err(), ThickenError::Closed);
    assert!(matches!(thicken(&nan_vertex, &params(1.0)), Err(ThickenError::Invalid(_))));
    assert!(matches!(thicken(&out_of_range, &params(1.0)), Err(ThickenError::Invalid(_))));
    assert!(thicken(&flipped, &params(0.1)).is_ok(), "a consistently orientable mesh is repaired");
    assert_eq!(
        thicken(&three_on_an_edge, &params(0.1)).unwrap_err(),
        ThickenError::Empty,
        "every triangle is on the edge"
    );
    for t in [0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -0.0] {
        assert!(matches!(thicken(&cap, &params(t)), Err(ThickenError::BadThickness(_))), "{t}");
    }
    for t in [1e300, -1e300, 1e9, 70.0] {
        assert!(matches!(thicken(&cap, &params(t)), Err(ThickenError::TooThick { .. })), "{t}");
    }
    assert!(thicken(&cap, &params(1e-12)).is_ok() || thicken(&cap, &params(1e-12)).is_err());
}

#[test]
fn a_scan_with_inconsistent_orientation_and_a_fin_is_repaired() {
    // Arrange: the cap with every tenth triangle turned around and a third triangle on an edge
    let mut m = noisy_cap(0.0, 0).0;
    let turned = m.triangles.iter().step_by(10).count();
    for t in m.triangles.iter_mut().step_by(10) {
        t.swap(1, 2);
    }
    let [a, b, _] = m.triangles[1];
    let apex = m.positions[a as usize] + Vec3::new(0.0, 0.0, 3.0);
    m.positions.push(apex);
    m.triangles.push([b, a, m.positions.len() as u32 - 1]);
    let fin_edges_removed = 1;

    // Act
    let r = thicken(&m, &params(1.0)).expect("repaired and thickened");

    // Assert
    assert_closed_outward(&r.mesh);
    assert!(r.report.flipped_triangles >= turned - 3 * fin_edges_removed - 10, "{:?}", r.report);
    assert!(r.report.removed_non_manifold >= 1, "{:?}", r.report);
}

#[test]
fn a_fan_of_two_hundred_thousand_triangles_on_one_edge_is_handled_without_hanging() {
    // Arrange: a flat panel plus 200 000 fins on one of its edges
    let mut m = noisy_cap(0.0, 0).0;
    let [a, b, _] = m.triangles[0];
    for i in 0..200_000u32 {
        m.positions.push(Point3::new(0.0, 0.0, 1.0 + f64::from(i)));
        m.triangles.push([a, b, m.positions.len() as u32 - 1]);
    }

    // Act
    let r = thicken(&m, &params(1.0));

    // Assert: repaired, closed, and no edge with more than two triangles
    let r = r.expect("repaired");
    assert_closed_outward(&r.mesh);
    assert!(r.report.removed_non_manifold >= 200_000, "{:?}", r.report);
}

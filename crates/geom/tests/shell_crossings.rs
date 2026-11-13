//! Thickened shells have no local, sheet-to-sheet or rim-strip crossing (`surfacecraft_geom::shell`):
//! every test counts crossings of the result exhaustively (all pairs of triangles that can cross).
use surfacecraft_geom::shell::{ThickenError, ThickenParams, Thickened, thicken};
use surfacecraft_geom::{Aabb, Bvh, Mesh, Point3, Rng, Vec3, synth, tri};

fn params(thickness: f64) -> ThickenParams {
    ThickenParams { thickness, symmetric: false }
}

/// Crossing pairs of the shell: triangles that share no vertex and pass through each other, triangles
/// that share a vertex and whose opposite edges pass through each other, triangles that share an
/// edge and lie over each other.
fn crossing_pairs(m: &Mesh) -> Vec<(usize, usize)> {
    let bvh = Bvh::build(m);
    let corners = |t: usize| -> [Point3; 3] { m.triangles[t].map(|v| m.positions[v as usize]) };
    let mut out = Vec::new();
    for a in 0..m.triangles.len() {
        let ta = corners(a);
        let bb = Aabb::from_points(ta.iter());
        let near = bvh.candidates(&|b: &Aabb| (0..3).all(|k| b.min[k] <= bb.max[k] && bb.min[k] <= b.max[k]));
        for b in near.into_iter().map(|b| b as usize).filter(|&b| b > a) {
            if pair_crosses(m, a, b) {
                out.push((a, b));
            }
        }
    }
    out
}

fn pair_crosses(m: &Mesh, a: usize, b: usize) -> bool {
    let (ia, ib) = (m.triangles[a], m.triangles[b]);
    let shared: Vec<u32> = ia.iter().copied().filter(|v| ib.contains(v)).collect();
    let pt = |v: u32| m.positions[v as usize];
    let pierces = |x: &[u32; 3], y: &[u32; 3], skip: Option<u32>| -> bool {
        (0..3).any(|k| {
            let (u, v) = (x[k], x[(k + 1) % 3]);
            if skip.is_some_and(|s| u == s || v == s) {
                return false;
            }
            let (p, q) = (pt(u), pt(v));
            tri::ray_hit(&p, &(q - p), &pt(y[0]), &pt(y[1]), &pt(y[2])).is_some_and(|s| s > 1e-9 && s < 1.0 - 1e-9)
        })
    };
    match shared.len() {
        0 => pierces(&ia, &ib, None) || pierces(&ib, &ia, None),
        1 => pierces(&ia, &ib, Some(shared[0])) || pierces(&ib, &ia, Some(shared[0])),
        2 => {
            // lying over each other: the opposite vertices on the same side of the shared edge and the
            // triangles (almost) coplanar
            let (e0, e1) = (pt(shared[0]), pt(shared[1]));
            let (oa, ob) = (
                ia.iter().copied().find(|v| !shared.contains(v)).map(pt),
                ib.iter().copied().find(|v| !shared.contains(v)).map(pt),
            );
            let (Some(oa), Some(ob)) = (oa, ob) else { return false };
            let (na, nb) =
                (tri::normal(&pt(ia[0]), &pt(ia[1]), &pt(ia[2])), tri::normal(&pt(ib[0]), &pt(ib[1]), &pt(ib[2])));
            let (Some(na), Some(nb)) = (na, nb) else { return false };
            let edge = e1 - e0;
            let side = |o: Point3| (o - e0) - edge * ((o - e0).dot(&edge) / edge.norm_squared());
            let same_side = side(oa).dot(&side(ob)) > 0.0;
            same_side && na.dot(&nb).abs() > 0.999_999
        }
        _ => false,
    }
}

/// A grid panel `w` x `h` mm with cells of `cell` mm, heights from `z`, noise of `sigma`.
fn heightfield(w: f64, h: f64, cell: f64, sigma: f64, z: impl Fn(f64, f64) -> f64) -> Mesh {
    let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), w, h, (w / cell) as usize, (h / cell) as usize);
    for p in &mut m.positions {
        p.z = z(p.x, p.y);
    }
    if sigma > 0.0 {
        synth::add_noise(&mut m, sigma, &mut Rng::new(11));
    }
    m
}

fn plate_with_holes(side: f64, cell: f64, radius: f64, sigma: f64) -> Mesh {
    let plate = heightfield(side, side, cell, 0.0, |_, _| 0.0);
    let c = Point3::new(side / 2.0, side / 2.0, 0.0);
    let kept: Vec<u32> = (0..plate.triangle_count() as u32)
        .filter(|&t| plate.face_centroid(t as usize).is_some_and(|f| (f - c).norm() > radius))
        .collect();
    let mut m = plate.submesh(&kept);
    if sigma > 0.0 {
        synth::add_noise(&mut m, sigma, &mut Rng::new(5));
    }
    m
}

fn rolled(radius: f64, span_deg: f64, cell: f64, sigma: f64) -> Mesh {
    let span = span_deg.to_radians();
    let mut m = synth::plane(
        Point3::origin(),
        Vec3::x(),
        Vec3::y(),
        span * radius,
        8.0,
        (span * radius / cell) as usize,
        (8.0 / cell) as usize,
    );
    for p in &mut m.positions {
        let a = p.x / radius - span / 2.0;
        *p = Point3::new(radius * a.sin(), p.y, radius * a.cos());
    }
    if sigma > 0.0 {
        synth::add_noise(&mut m, sigma, &mut Rng::new(5));
    }
    m
}

/// A bead along y: a Gaussian ridge of height `h` and width `w`, noise `sigma`, cells `cell` mm.
fn bead(w: f64, h: f64, cell: f64, sigma: f64) -> Mesh {
    heightfield(14.0, 14.0, cell, sigma, move |x, _| h * (-((x - 7.0).powi(2)) / (2.0 * w * w)).exp())
}

/// What a shell must satisfy wherever the thickening did not report vertices it could not relax:
/// no pair of triangles anywhere crosses (these scans have no channels), and the sampled estimate
/// agrees.
fn assert_free_of_crossings(r: &Thickened, what: &str) {
    let pairs = crossing_pairs(&r.mesh);
    assert_eq!(r.report.unresolved_vertices, 0, "{what}: {:?}", r.report);
    assert!(pairs.is_empty(), "{what}: {} crossing pairs, first {:?}: {:?}", pairs.len(), pairs.first(), r.report);
    assert_eq!(r.report.crossing_local, 0, "{what}: {:?}", r.report);
    assert_eq!(r.report.crossing_triangles, 0, "{what}: {:?}", r.report);
}

#[test]
fn a_bead_too_narrow_for_the_floor_is_refused_as_unresolved_instead_of_returned_with_crossing_sheets() {
    // Arrange: ridges 0.3 and 0.35 mm wide with noise of 0.07 mm: the crest curves tighter than the
    // floor of the offset (a tenth of the wall), so the two sheets of the wall cannot be kept apart;
    // on main the shell is returned with 4 to 8 pairs of triangles of the two sheets cut through each other
    let cases = [(bead(0.35, 1.3, 0.25, 0.07), 1.0), (bead(0.3, 1.0, 0.2, 0.07), 0.8)];

    for (k, (panel, t)) in cases.into_iter().enumerate() {
        // Act
        let result = thicken(&panel, &params(t));

        // Assert: a refusal, or a shell whose crossings are all accounted for
        match result {
            Err(ThickenError::Unresolved(n)) => assert!(n > 0, "bead {k}"),
            Err(other) => panic!("bead {k}: {other}"),
            Ok(r) => assert!(
                r.report.unresolved_vertices > 0 || crossing_pairs(&r.mesh).is_empty(),
                "bead {k}: crossings that are not unresolved vertices: {:?}",
                r.report
            ),
        }
    }
}

#[test]
fn creases_the_floor_can_carry_are_bounded_by_their_feature_size_and_leave_the_sheets_apart() {
    // Arrange: sharp ridges (half angle 27 and 18 degrees) whose flanks are close along the sheet;
    // wall 0.5 mm behind the face, 0.15 mm cells
    let cases = [2.0, 3.0].map(|k| heightfield(14.0, 8.0, 0.15, 0.0, move |x, _| -k * (x - 7.0).abs()));

    for (k, panel) in cases.into_iter().enumerate() {
        // Act
        let r = thicken(&panel, &params(0.5)).unwrap_or_else(|e| panic!("crease {k}: {e}"));

        // Assert: vertices near the crest were bounded before any fold was looked for
        assert!(r.report.limited_vertices > 0, "crease {k}: {:?}", r.report);
        assert_free_of_crossings(&r, &format!("crease {k}"));
        assert!(r.report.min_wall > 0.0 && r.report.min_wall < 0.5, "crease {k}: {:?}", r.report);
    }
}

#[test]
fn the_strip_along_the_rim_of_a_noisy_flat_scan_does_not_cross_the_sheets_or_itself() {
    // Arrange: noise of half a cell: the rim normals of a one-sided average tilt the rows of the strip
    // into each other (on main 28 and 59 pairs of triangles cross)
    let panel = heightfield(16.0, 16.0, 0.2, 0.1, |_, _| 0.0);

    for t in [1.0, -1.0] {
        // Act
        let r = thicken(&panel, &params(t)).expect("relaxed, not refused");

        // Assert
        assert!(r.report.strip_rows > 1, "the real multi-row strip is tested: {:?}", r.report);
        assert_free_of_crossings(&r, &format!("flat t={t}"));
    }
}

#[test]
fn neighbouring_offset_triangles_of_a_noisy_scan_do_not_cross_although_few_vertices_fold() {
    // Arrange: on main 11 pairs of triangles of the offset layer cross where the area and normal tests
    // of single triangles pass (wall in front of the face)
    let panel = heightfield(16.0, 16.0, 0.2, 0.1, |_, _| 0.0);

    // Act
    let r = thicken(&panel, &params(-1.0)).expect("relaxed, not refused");

    // Assert
    assert_free_of_crossings(&r, "noisy flat, wall in front");
    assert!(r.report.clamped_vertices > 0, "{:?}", r.report);
}

#[test]
fn the_rim_of_a_window_and_of_a_rolled_scan_does_not_cut_the_wall() {
    // Arrange: windows 1 and 2 mm wide in a noisy plate (the strip around a window is a tube whose
    // rows cross on main), and a rolled sheet of radius 2.5 mm whose rim curves tightly
    let cases: Vec<(&str, Mesh, f64)> = vec![
        ("window r0.5", plate_with_holes(10.0, 0.15, 0.5, 0.06), 1.0),
        ("window r1 t=2", plate_with_holes(10.0, 0.15, 1.0, 0.06), 2.0),
        ("window r1 in front", plate_with_holes(10.0, 0.15, 1.0, 0.06), -1.0),
        ("window r2", plate_with_holes(10.0, 0.15, 2.0, 0.06), 1.0),
        ("rolled r2.5", rolled(2.5, 180.0, 0.2, 0.06), 1.0),
    ];
    for (name, panel, t) in cases {
        // Act
        let r = thicken(&panel, &params(t)).expect("relaxed, not refused");

        // Assert
        assert_free_of_crossings(&r, name);
    }
}

#[test]
fn crossings_that_cannot_be_relaxed_away_are_counted_as_unresolved_vertices() {
    // Arrange: noise as large as half a cell leaves a cliff of the scan at the rim where even the
    // floor of the offset crosses
    let cases = [
        bead(0.8, 1.5, 0.2, 0.1),
        heightfield(14.0, 14.0, 0.2, 0.1, |x, y| {
            1.5 * (-((x - 7.0).powi(2) + (y - 7.0).powi(2)) / (2.0 * 0.35f64.powi(2))).exp()
        }),
    ];
    let mut left_over = 0;

    for panel in cases {
        // Act
        let r = thicken(&panel, &params(1.0)).expect("a few unresolved vertices are no refusal");

        // Assert: what crosses is on the account of the unresolved vertices
        let pairs = crossing_pairs(&r.mesh).len();
        left_over += pairs;
        assert!(pairs == 0 || r.report.unresolved_vertices > 0, "{pairs} crossings, {:?}", r.report);
        assert!(r.report.unresolved_vertices as f64 <= 0.001 * panel.vertex_count() as f64);
    }
    assert!(left_over > 0, "the test needs a shell that keeps a crossing");
}

#[test]
fn crossings_far_apart_along_the_surface_are_not_relaxed_and_not_local() {
    // Arrange: a channel 3 mm wide whose walls are 63 mm apart along the sheet, wall 2 mm
    let walls = {
        let path = [(-1.5, 12.0), (-1.5, 0.0), (1.5, 0.0), (1.5, 12.0)];
        let (rows, steps) = (30usize, 40usize);
        let mut positions = Vec::new();
        for j in 0..=rows {
            let y = 10.0 * j as f64 / rows as f64;
            for (k, ((x0, z0), (x1, z1))) in path.iter().zip(path.iter().skip(1)).map(|(a, b)| (*a, *b)).enumerate() {
                let n = if k == 1 { 10 } else { steps };
                for i in usize::from(k > 0)..=n {
                    let f = i as f64 / n as f64;
                    positions.push(Point3::new(x0 + (x1 - x0) * f, y, z0 + (z1 - z0) * f));
                }
            }
        }
        let per_row = positions.len() / (rows + 1);
        let mut triangles = Vec::new();
        for j in 0..rows {
            for i in 0..per_row - 1 {
                let (a, b) = ((j * per_row + i) as u32, (j * per_row + i + 1) as u32);
                let (c, d) = (((j + 1) * per_row + i + 1) as u32, ((j + 1) * per_row + i) as u32);
                triangles.push([a, b, c]);
                triangles.push([a, c, d]);
            }
        }
        let mut m = Mesh { positions, triangles };
        if m.vertex_normals()[per_row / 6].x > 0.0 {
            for t in &mut m.triangles {
                t.swap(1, 2);
            }
        }
        m
    };

    // Act
    let r = thicken(&walls, &params(2.0)).expect("thickens");

    // Assert: the crossing between the walls stays, and it is not counted as local
    assert!(r.report.crossing_triangles > 0, "{:?}", r.report);
    assert_eq!(r.report.crossing_local, 0, "{:?}", r.report);
}

#[test]
fn the_bounds_and_the_crossing_rounds_give_bit_identical_shells_for_one_and_eight_threads() {
    // Arrange: a crease whose vertices are bounded by their feature size, and a noisy plate with a window
    // whose strip is relaxed
    let cases = [
        (heightfield(14.0, 8.0, 0.15, 0.0, |x, _| -3.0 * (x - 7.0).abs()), 0.5),
        (plate_with_holes(10.0, 0.15, 1.0, 0.06), 2.0),
    ];
    for (panel, t) in cases {
        let run = |threads: usize| {
            rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap()
                .install(|| thicken(&panel, &params(t)).unwrap())
        };

        // Act
        let (one, eight) = (run(1), run(8));

        // Assert
        let bits = |m: &Mesh| {
            m.positions.iter().flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).collect::<Vec<_>>()
        };
        assert_eq!(one.mesh.triangles, eight.mesh.triangles);
        assert_eq!(bits(&one.mesh), bits(&eight.mesh));
        assert_eq!(one.report, eight.report);
        assert!(one.report.clamped_vertices > 0, "{:?}", one.report);
    }
}

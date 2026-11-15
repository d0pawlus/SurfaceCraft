//! Tests of the merge on noisy views of a lumpy blob (the shape of the engine's alignment fixtures).
use super::*;
use crate::{Rng, Transform, Vec3, synth};

const SIGMA: f64 = 0.05;
const TOL: f64 = 0.3;

fn blob(nu: usize) -> Mesh {
    let mut m = synth::sphere(Point3::origin(), 20.0, nu, nu / 2);
    for p in &mut m.positions {
        let lump = 1.0 + 0.12 * (0.21 * p.x).sin() * (0.17 * p.y).cos() + 0.08 * (0.3 * p.z).sin();
        *p = Point3::new(p.x * 1.6 * lump, p.y * lump, p.z * 0.7 * lump);
    }
    m
}

fn cut(m: &Mesh, keep: impl Fn(&Point3) -> bool) -> Mesh {
    let ids: Vec<u32> =
        (0..m.triangle_count()).filter(|&i| m.face_centroid(i).is_some_and(|c| keep(&c))).map(|i| i as u32).collect();
    m.submesh(&ids)
}

fn noisy(mut m: Mesh, seed: u64) -> Mesh {
    synth::add_noise(&mut m, SIGMA, &mut Rng::new(seed));
    m
}

/// View `k` of the blob: its own tessellation and noise, the side of `dir` up to `reach` mm beyond the centre.
fn view(k: usize, dir: Vec3, reach: f64) -> Mesh {
    cut(&noisy(blob(48 + 4 * k), 200 + k as u64), |p| p.coords.dot(&dir) >= -reach)
}

fn run(views: &[Mesh], tol: f64) -> (Option<Mesh>, Report) {
    merge(views, &Params { tolerance: tol }, &Cancel::new(), &mut |_| {}).unwrap()
}

/// The rows of the scans add up: every triangle given is kept, put back, collapsed or removed; the
/// kept and put back ones are in the result; the removed area is the area the coverage check sampled.
fn assert_rows(rep: &Report, mesh: &Mesh) {
    for r in &rep.scans {
        assert_eq!(r.triangles, r.kept + r.restored + r.collapsed + r.removed, "{r:?}");
    }
    let in_result: usize = rep.scans.iter().map(|r| r.kept + r.restored).sum();
    assert!(in_result <= mesh.triangle_count());
    let removed: f64 = rep.scans.iter().map(|r| r.removed_area).sum();
    assert!(
        (removed - rep.checks.removed.area).abs() <= 1e-6 * removed.max(1.0),
        "{removed} {}",
        rep.checks.removed.area
    );
}

fn print(rep: &Report) {
    eprintln!(
        "before {:?}\nafter {:?}\nseam {:?}\nchecks {:?}\neaten {} issues {:?}",
        rep.before, rep.after, rep.seam, rep.checks, rep.eaten, rep.issues
    );
    for (k, r) in rep.scans.iter().enumerate() {
        eprintln!("scan {k}: {r:?}");
    }
    for p in &rep.pairs {
        eprintln!(
            "pair {}-{}: near {} within {} opposed {} rms {:.4} p95 {:.4} worst {:.4}",
            p.a, p.b, p.near, p.within, p.opposed, p.rms, p.p95, p.worst
        );
    }
}

#[test]
fn two_views_of_the_blob_merge_into_one_piece_with_at_most_two_open_loops() {
    let views = [view(0, Vec3::x(), 8.0), view(1, -Vec3::x(), 8.0)];
    let (mesh, rep) = run(&views, TOL);
    print(&rep);
    let mesh = mesh.unwrap();
    // exact by construction: manifold, oriented, no added triangle crossing another
    assert_eq!(rep.after.non_manifold_edges, 0);
    assert_eq!(rep.after.unoriented_edges, 0);
    assert_eq!(rep.checks.added_crossings, 0);
    // measured 1 piece, 0 loops, 0 open seam edges: bounded, a last-bit difference of the math
    // library may move a triangle
    assert!(rep.after.pieces == 1 && rep.after.loops <= 2, "{:?}", rep.after);
    assert!(rep.seam.open_edges * 50 <= rep.seam.front_edges.max(1), "{:?}", rep.seam);
    assert_rows(&rep, &mesh);
}

#[test]
fn identical_scans_keep_one_copy() {
    let a = view(0, Vec3::x(), 8.0);
    let (mesh, rep) = run(&[a.clone(), a.clone()], TOL);
    print(&rep);
    let mesh = mesh.unwrap();
    assert_eq!(mesh.triangle_count(), a.triangle_count());
    assert!(rep.issues.contains(&Issue::FullyCovered { scan: 1 }));
}

#[test]
fn three_pieces_of_one_scan_merge_back_into_it() {
    let whole = noisy(blob(64), 5);
    let pieces = [
        cut(&whole, |p| p.x <= 8.0),
        cut(&whole, |p| p.x >= -8.0 && p.y <= 4.0),
        cut(&whole, |p| p.x >= -8.0 && p.y >= -4.0),
    ];
    let (mesh, rep) = run(&pieces, TOL);
    print(&rep);
    let mesh = mesh.unwrap();
    // measured 1 piece, 0 loops, 261 triangles restored (every vertex re-welded)
    assert!(rep.after.pieces <= 2 && rep.after.loops <= 2, "{:?}", rep.after);
    assert!(rep.seam.restored >= 100, "{:?}", rep.seam);
    assert_rows(&rep, &mesh);
    eprintln!("{} triangles against {}", mesh.triangle_count(), whole.triangle_count());
}

#[test]
fn a_view_moved_by_several_tolerances_is_refused() {
    let moved = view(1, -Vec3::x(), 8.0).transformed(&Transform::translation(Vec3::new(0.0, 0.0, 3.0 * TOL)));
    let (mesh, rep) = run(&[view(0, Vec3::x(), 8.0), moved], TOL);
    print(&rep);
    assert!(mesh.is_none() && rep.refused);
    assert!(rep.issues.iter().any(|i| matches!(i, Issue::Misaligned { .. } | Issue::Displaced { .. })));
}

fn views_of(n: usize) -> Vec<Mesh> {
    let d = 0.577_35;
    let sides: [([f64; 3], f64); 8] = [
        ([1.0, 0.0, 0.0], 8.0),
        ([-1.0, 0.0, 0.0], 8.0),
        ([0.0, 1.0, 0.0], 5.0),
        ([0.0, -1.0, 0.0], 5.0),
        ([0.0, 0.0, 1.0], 3.5),
        ([0.0, 0.0, -1.0], 3.5),
        ([d, d, d], 6.0),
        ([-d, -d, d], 6.0),
    ];
    sides.iter().take(n).enumerate().map(|(k, (v, r))| view(k, Vec3::new(v[0], v[1], v[2]), *r)).collect()
}

#[test]
fn many_views_of_the_blob_merge_manifold_and_oriented() {
    for n in [4, 6, 8] {
        let (mesh, rep) = run(&views_of(n), TOL);
        eprintln!("== {n} views");
        print(&rep);
        let mesh = mesh.unwrap();
        assert_eq!(rep.after.non_manifold_edges, 0, "{n}");
        assert_eq!(rep.after.unoriented_edges, 0, "{n}");
        assert_rows(&rep, &mesh);
    }
}

#[test]
fn hostile_input_is_an_error_not_a_panic() {
    let a = view(0, Vec3::x(), 8.0);
    let cancel = Cancel::new();
    let go = |scans: &[Mesh], tol: f64| merge(scans, &Params { tolerance: tol }, &cancel, &mut |_| {});
    assert!(matches!(go(std::slice::from_ref(&a), TOL), Err(MergeError::Params(_))), "one scan");
    assert!(matches!(go(&[], TOL), Err(MergeError::Params(_))), "none");
    assert!(matches!(go(&[a.clone(), Mesh::default()], TOL), Err(MergeError::Params(_))), "empty");
    for tol in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(matches!(go(&[a.clone(), a.clone()], tol), Err(MergeError::Params(_))), "{tol}");
    }
    let mut nan = a.clone();
    nan.positions[3] = Point3::new(0.0, f64::NAN, 0.0);
    assert!(matches!(go(&[a.clone(), nan], TOL), Err(MergeError::Invalid(1, _))));
    let mut bad = a.clone();
    bad.triangles.push([0, 1, 1_000_000]);
    assert!(matches!(go(&[a.clone(), bad], TOL), Err(MergeError::Invalid(1, _))));
    // degenerate but valid: collapsed triangles, a scan of one triangle, coincident points
    let mut collapsed = a.clone();
    collapsed.triangles.extend([[0, 0, 1], [2, 2, 2]]);
    let one = Mesh {
        positions: vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
        triangles: vec![[0, 1, 2]],
    };
    let dot = Mesh { positions: vec![Point3::origin(); 3], triangles: vec![[0, 1, 2]] };
    for scans in [
        vec![a.clone(), collapsed],
        vec![one.clone(), one],
        vec![dot.clone(), dot],
        vec![a.clone(), a.transformed(&Transform::translation(Vec3::new(1e8, 0.0, 0.0)))],
    ] {
        assert!(go(&scans, TOL).is_ok());
    }
}

#[test]
fn a_cancelled_merge_returns_cancelled() {
    let cancel = Cancel::new();
    cancel.cancel();
    let r = merge(&views_of(2), &Params { tolerance: TOL }, &cancel, &mut |_| {});
    assert!(matches!(r, Err(MergeError::Cancelled)), "{r:?}");
}

#[test]
fn the_stages_are_announced_in_order() {
    let mut seen = Vec::new();
    merge(&views_of(2), &Params { tolerance: TOL }, &Cancel::new(), &mut |s| seen.push(s)).unwrap();
    assert_eq!(seen, vec![Stage::Prepare, Stage::Overlap, Stage::Ownership, Stage::Union, Stage::Seam, Stage::Check]);
}

#[test]
fn a_cancellation_during_any_stage_is_honoured() {
    // open views (with a seam) and closed scans (no border: the border distance never runs)
    let closed = vec![noisy(blob(40), 1), noisy(blob(44), 2)];
    for scans in [views_of(2), closed] {
        for at in [Stage::Prepare, Stage::Overlap, Stage::Ownership, Stage::Union, Stage::Seam, Stage::Check] {
            let cancel = Cancel::new();
            let flag = cancel.clone();
            // the flag is set when the stage starts: the stage itself must see it
            let mut seen = Vec::new();
            let r = merge(&scans, &Params { tolerance: TOL }, &cancel, &mut |s| {
                seen.push(s);
                if s == at {
                    flag.cancel();
                }
            });
            assert!(matches!(r, Err(MergeError::Cancelled)), "{at:?}: {r:?}");
            assert_eq!(seen.last(), Some(&at), "no later stage started after {at:?}");
        }
    }
}

/// The twelve corners and twenty faces of an icosahedron.
fn icosahedron() -> (Vec<Point3>, Vec<[u32; 3]>) {
    let t = (1.0 + 5f64.sqrt()) / 2.0;
    let pos: Vec<Point3> = [
        (-1.0, t, 0.0),
        (1.0, t, 0.0),
        (-1.0, -t, 0.0),
        (1.0, -t, 0.0),
        (0.0, -1.0, t),
        (0.0, 1.0, t),
        (0.0, -1.0, -t),
        (0.0, 1.0, -t),
        (t, 0.0, -1.0),
        (t, 0.0, 1.0),
        (-t, 0.0, -1.0),
        (-t, 0.0, 1.0),
    ]
    .iter()
    .map(|&(x, y, z)| Point3::new(x, y, z))
    .collect();
    let tris: Vec<[u32; 3]> = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    (pos, tris)
}

/// An icosphere of radius 20 subdivided `k` times, turned by `turn` (its own tessellation), with the
/// lumps of the blob: an organic surface with near-equilateral triangles everywhere.
fn ico_blob(k: usize, turn: &Transform) -> Mesh {
    let (mut pos, mut tris) = icosahedron();
    for _ in 0..k {
        let mut mid = std::collections::HashMap::new();
        let mut out = Vec::with_capacity(tris.len() * 4);
        let mut m = |a: u32, b: u32, pos: &mut Vec<Point3>| {
            *mid.entry((a.min(b), a.max(b))).or_insert_with(|| {
                pos.push(Point3::from((pos[a as usize].coords + pos[b as usize].coords) * 0.5));
                (pos.len() - 1) as u32
            })
        };
        for [a, b, c] in tris {
            let (ab, bc, ca) = (m(a, b, &mut pos), m(b, c, &mut pos), m(c, a, &mut pos));
            out.extend([[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]);
        }
        tris = out;
    }
    let positions = pos
        .iter()
        .map(|p| {
            let p = turn.apply(&Point3::from(p.coords.normalize() * 20.0));
            let lump = 1.0 + 0.12 * (0.21 * p.x).sin() * (0.17 * p.y).cos() + 0.08 * (0.3 * p.z).sin();
            Point3::new(p.x * 1.6 * lump, p.y * lump, p.z * 0.7 * lump)
        })
        .collect();
    Mesh { positions, triangles: tris }
}

/// Two noisy halves (sigma 0.02) of the icosphere blob subdivided `k` times, each its own tessellation.
fn ico_halves(k: usize) -> Vec<Mesh> {
    [(1.0, 0u64), (-1.0, 1)]
        .iter()
        .map(|&(side, s)| {
            let turn = Transform::rotation_axis_angle(&Rng::new(40 + s).unit_vector(), 0.3 + s as f64);
            let mut m = ico_blob(k, &turn);
            synth::add_noise(&mut m, 0.02, &mut Rng::new(300 + s));
            cut(&m, |p| side * p.x >= -8.0)
        })
        .collect()
}

/// The fixtures' blob is a UV sphere whose poles are fans of slivers right on the seam; on a surface
/// of near-equilateral triangles the merge is closed (k = 5 and 6 here, about 13 000 and 51 000
/// triangles a half; k = 7, about 200 000 a half, closed too when tried; 0 open seam edges measured).
#[test]
fn halves_of_an_even_organic_surface_merge_closed_at_fine_resolution() {
    for k in [5, 6] {
        let (mesh, rep) = run(&ico_halves(k), 0.15);
        assert!(mesh.is_some(), "{k}");
        // exact by construction
        assert_eq!((rep.after.non_manifold_edges, rep.after.unoriented_edges), (0, 0), "{k}");
        assert_eq!(rep.checks.added_crossings, 0, "{k}");
        // measured 1 piece, 0 loops, 0 open edges, 0 pinched, 0 unwelded: bounded
        assert!(rep.after.pieces <= 2 && rep.after.loops <= 2, "{k}: {:?}", rep.after);
        assert!(rep.seam.open_edges * 50 <= rep.seam.front_edges.max(1), "{k}: {:?}", rep.seam);
        assert!(rep.after.pinched_vertices <= 2 && rep.checks.unwelded_vertices <= 2, "{k}");
    }
}

/// Timing of a large merge (two halves of about 0.65 million triangles each), printed, not asserted.
#[test]
#[ignore = "timing run"]
fn large_merge_timing() {
    let halves = ico_halves(8);
    let t0 = std::time::Instant::now();
    let (_, rep) = run(&halves, 0.15);
    eprintln!(
        "{} + {} triangles: {:.1} s; pieces {} loops {} open {} unwelded {} removed within {:.4}",
        halves[0].triangle_count(),
        halves[1].triangle_count(),
        t0.elapsed().as_secs_f64(),
        rep.after.pieces,
        rep.after.loops,
        rep.seam.open_edges,
        rep.checks.unwelded_vertices,
        rep.checks.removed.share()
    );
}

/// A flat grid of `n` × `n` unit cells in the plane z = 0, offset by `(x0, y0)`, facing +z.
fn grid(n: usize, x0: f64, y0: f64) -> Mesh {
    let mut m = Mesh::default();
    for j in 0..=n {
        for i in 0..=n {
            m.positions.push(Point3::new(x0 + i as f64, y0 + j as f64, 0.0));
        }
    }
    let at = |i: usize, j: usize| (j * (n + 1) + i) as u32;
    for j in 0..n {
        for i in 0..n {
            m.triangles.push([at(i, j), at(i + 1, j), at(i + 1, j + 1)]);
            m.triangles.push([at(i, j), at(i + 1, j + 1), at(i, j + 1)]);
        }
    }
    m
}

#[test]
fn a_removed_region_the_seam_left_open_comes_back_whole() {
    let cancel = Cancel::new();
    let scans = vec![prep::prepare(&grid(10, 0.0, 0.0), &cancel).unwrap()];
    let centre = Point3::new(5.0, 5.0, 0.0);
    let removed: Vec<Vec<bool>> = vec![
        (0..scans[0].mesh.triangle_count())
            .map(|t| (scans[0].mesh.face_centroid(t).unwrap() - centre).norm() < 2.5)
            .collect(),
    ];
    // a triangle whose three corners all lie inside the removed region: none of them is in the union
    let inner = (0..scans[0].mesh.triangle_count())
        .find(|&t| scans[0].mesh.triangle(t).unwrap().iter().all(|p| (p - centre).norm() < 1.5))
        .unwrap();
    let mut u = union::assemble(&scans, &removed);
    assert!(scans[0].mesh.triangles[inner].iter().all(|&v| u.vertex_of[0][v as usize] == prep::NONE));
    let mut guard = guard::Guard::new(&scans, &u, 0.1, 4.0);
    let rep = recover::recover(&mut u, &scans, &removed, &mut guard, (0.1, &cancel)).unwrap();
    assert!(rep.triangles > 0 && rep.triangles == rep.uncovered, "{rep:?}");
    assert!(u.origin.contains(&union::Origin::Recovered(0, inner as u32)));
    // no vertex is left that no triangle uses
    let mut used = vec![false; u.positions.len()];
    u.triangles.iter().flatten().for_each(|&v| used[v as usize] = true);
    assert!(used.iter().all(|&f| f));
}

#[test]
fn a_refused_recovery_leaves_no_vertex_behind() {
    let cancel = Cancel::new();
    let scans = vec![prep::prepare(&grid(6, 0.0, 0.0), &cancel).unwrap()];
    let removed = vec![vec![true; scans[0].mesh.triangle_count()]];
    let mut u = union::assemble(&scans, &removed);
    // a wall over the whole grid, recorded in the guard, refuses every triangle that would come back
    let mut guard = guard::Guard::new(&scans, &u, 0.1, 4.0);
    let wall = [0u32, 1, 2];
    u.positions.extend([Point3::new(-1.0, 3.0, -1.0), Point3::new(8.0, 3.0, -1.0), Point3::new(3.0, 3.0, 5.0)]);
    u.normals.extend([Vec3::y(); 3]);
    u.scan_of.extend([1; 3]);
    assert!(guard.restorable(guard::Pts::of(&u), wall));
    let before = u.positions.len();
    let rep = recover::recover(&mut u, &scans, &removed, &mut guard, (0.1, &cancel)).unwrap();
    assert!(rep.triangles < rep.uncovered, "{rep:?}");
    let mut used = vec![false; u.positions.len()];
    u.triangles.iter().flatten().for_each(|&v| used[v as usize] = true);
    assert!(used[before..].iter().all(|&f| f), "every vertex added is used");
    // the rollback leaves the per-vertex lists in step and no scan vertex pointing past them
    assert_eq!(u.normals.len(), u.positions.len());
    assert_eq!(u.scan_of.len(), u.positions.len());
    assert!(u.vertex_of.iter().flatten().all(|&x| x == prep::NONE || (x as usize) < u.positions.len()));
}

#[test]
fn removed_surface_the_recovery_could_not_put_back_is_reported_uncovered() {
    let cancel = Cancel::new();
    let scans = vec![prep::prepare(&grid(6, 0.0, 0.0), &cancel).unwrap()];
    let removed = vec![vec![true; scans[0].mesh.triangle_count()]];
    let mut u = union::assemble(&scans, &removed);
    let mut guard = guard::Guard::new(&scans, &u, 0.1, 4.0);
    // a wall through the whole grid refuses every triangle that would come back
    let first = u.positions.len() as u32;
    u.positions.extend([Point3::new(-1.0, 3.0, -1.0), Point3::new(8.0, 3.0, -1.0), Point3::new(3.0, 3.0, 5.0)]);
    u.normals.extend([Vec3::y(); 3]);
    u.scan_of.extend([1; 3]);
    let wall = [first, first + 1, first + 2];
    assert!(guard.restorable(guard::Pts::of(&u), wall));
    let mut rep = seam::SeamReport::default();
    assert!(recover_rounds(&mut u, &scans, &removed, &mut guard, 0.1, &mut rep, &cancel).is_some());
    assert!(rep.recovered.uncovered > 0, "{:?}", rep.recovered);
    assert!(rep.recovered.uncovered + rep.recovered.triangles <= scans[0].mesh.triangle_count());
}

#[test]
fn a_cancellation_at_any_look_stops_the_merge() {
    // a token that never fires within the run counts its looks: L in all
    let views = views_of(2);
    let big = 1usize << 40;
    let counter = Cancel::after_looks(big);
    merge(&views, &Params { tolerance: TOL }, &counter, &mut |_| {}).unwrap();
    let looks = big - counter.looks_left();
    eprintln!("{looks} looks at the flag in one merge of two views");
    assert!(looks > 20, "{looks}");
    // fired at its n-th look, for n spread over the whole run: every look that sees it stops the merge
    for n in (1..=looks).step_by((looks / 60).max(1)).chain([looks]) {
        let r = merge(&views, &Params { tolerance: TOL }, &Cancel::after_looks(n), &mut |_| {});
        assert!(matches!(r, Err(MergeError::Cancelled)), "fired at look {n} of {looks}: {r:?}");
    }
    assert!(merge(&views, &Params { tolerance: TOL }, &Cancel::after_looks(looks + 1), &mut |_| {}).is_ok());
}

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
fn two_views_of_the_blob_merge_into_one_closed_piece() {
    let views = [view(0, Vec3::x(), 8.0), view(1, -Vec3::x(), 8.0)];
    let (mesh, rep) = run(&views, TOL);
    print(&rep);
    let mesh = mesh.unwrap();
    assert_eq!(rep.after.pieces, 1);
    assert_eq!(rep.after.non_manifold_edges, 0);
    assert_eq!(rep.after.unoriented_edges, 0);
    assert_eq!(rep.checks.added_crossings, 0);
    assert!(mesh.triangle_count() > 0);
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
    assert_eq!(rep.after.pieces, 1);
    assert_eq!(rep.after.loops, 0);
    eprintln!("{} triangles against {}", mesh.triangle_count(), whole.triangle_count());
}

#[test]
fn a_view_moved_by_several_tolerances_is_refused() {
    let moved = view(1, -Vec3::x(), 8.0).transformed(&Transform::translation(Vec3::new(0.0, 0.0, 3.0 * TOL)));
    let (mesh, rep) = run(&[view(0, Vec3::x(), 8.0), moved], TOL);
    print(&rep);
    assert!(mesh.is_none() && rep.refused);
    assert!(rep.issues.iter().any(|i| matches!(i, Issue::Misaligned { .. })));
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
fn many_views_of_the_blob_merge_into_one_closed_piece() {
    for n in [4, 6, 8] {
        let (mesh, rep) = run(&views_of(n), TOL);
        eprintln!("== {n} views");
        print(&rep);
        assert!(mesh.is_some());
        assert_eq!(rep.after.non_manifold_edges, 0, "{n}");
        assert_eq!(rep.after.unoriented_edges, 0, "{n}");
    }
}

#[test]
fn hostile_input_is_an_error_not_a_panic() {
    let a = view(0, Vec3::x(), 8.0);
    let cancel = Cancel::new();
    let go = |scans: &[Mesh], tol: f64| merge(scans, &Params { tolerance: tol }, &cancel, &mut |_| {});
    assert!(matches!(go(&[a.clone()], TOL), Err(MergeError::Params(_))), "one scan");
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

//! P9-M4 gates: `mesh.merge` on 2, 4 and 8 noisy views of the organic blob and of the bracket (the
//! fixtures of `align.global`: every view its own tessellation with independent noise, sigma 0.05,
//! cut to the side it sees) at their true poses and at poses perturbed within the tolerance. The
//! result is measured against the true shape (share within the tolerance, worst distance), its
//! boundary loops and pieces against what the union of the views really leaves open, and it must
//! have no non-manifold edge, no inconsistent edge and no added triangle crossing another. The
//! merged blob is then wrapped (`scan.to_solid`, freeform) and its verdict printed next to the wrap
//! of one full scan of the blob.
//!
//! Every bound has the measured value next to it and lies at least 1.5 times from it; counts that can
//! differ between platforms (triangles, seam triangles, pieces, loops) are printed and bounded, never
//! asserted exactly. Exact zeros are asserted only for what holds by construction: no non-manifold
//! edge, no unoriented edge, no added triangle crossing another.
mod global_common;
mod pair_common;
use global_common::*;
use pair_common::*;
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Point3, Rng, Transform, Vec3};

/// Views of the blob: the six of the fixture and two more from diagonal directions.
fn blob_views_n(n: usize) -> Vec<Mesh> {
    let mut v = blob_views();
    let d = 0.577_35;
    for (k, dir) in [Vec3::new(d, d, d), Vec3::new(-d, -d, d)].into_iter().enumerate() {
        v.push(cut(&noisy(blob_at(72 + 4 * k), 260 + k as u64), |p| p.coords.dot(&dir) >= -6.0));
    }
    // 2: the two halves along x; 4: and along y; 8: all
    let order: &[usize] = match n {
        2 => &[0, 1],
        4 => &[0, 1, 2, 3],
        _ => &[0, 1, 2, 3, 4, 5, 6, 7],
    };
    order.iter().map(|&k| v[k].clone()).collect()
}

/// [`blob_views_n`] with the noise of series `series`.
fn blob_views_series(n: usize, series: u64) -> Vec<Mesh> {
    let mut v = blob_views_s(series);
    let d = 0.577_35;
    for (k, dir) in [Vec3::new(d, d, d), Vec3::new(-d, -d, d)].into_iter().enumerate() {
        v.push(cut(&noisy(blob_at(72 + 4 * k), 260 + k as u64 + 1000 * series), |p| p.coords.dot(&dir) >= -6.0));
    }
    let order: &[usize] = match n {
        2 => &[0, 1],
        4 => &[0, 1, 2, 3],
        _ => &[0, 1, 2, 3, 4, 5, 6, 7],
    };
    order.iter().map(|&k| v[k].clone()).collect()
}

/// Views of the bracket: the six of the fixture and two more (from below and a lower diagonal).
fn bracket_views_n(n: usize) -> Vec<Mesh> {
    let mut v = bracket_views_wide(6);
    let d = 0.577_35;
    for (k, (dir, reach)) in [(Vec3::new(0.0, 0.0, -1.0), 12.0), (Vec3::new(-d, -d, -d), 13.0)].into_iter().enumerate()
    {
        v.push(bracket_view_s(6 + k, dir, reach, 0));
    }
    let order: &[usize] = match n {
        2 => &[0, 1],
        4 => &[0, 1, 2, 3],
        _ => &[0, 1, 2, 3, 4, 5, 6, 7],
    };
    order.iter().map(|&k| v[k].clone()).collect()
}

/// A small rigid motion: about `angle` degrees about a random axis through `centre` and a shift of
/// `shift` mm.
fn nudge(seed: u64, centre: Point3, angle: f64, shift: f64) -> Transform {
    let mut rng = Rng::new(seed);
    let to = Transform::translation(-centre.coords);
    let turn = Transform::rotation_axis_angle(&rng.unit_vector(), angle.to_radians());
    let back = Transform::translation(centre.coords + rng.unit_vector() * shift);
    to.then_after(&turn).then_after(&back)
}

/// A session with the views as meshes `v0`, `v1`, ...
fn session(views: &[Mesh]) -> Session {
    let mut s = Session::new();
    for (k, v) in views.iter().enumerate() {
        s.doc.add_mesh(&format!("v{k}"), v.clone());
    }
    s
}

fn merge(s: &mut Session, n: usize, extra: Value) -> Value {
    let names: Vec<String> = (0..n).map(|k| format!("v{k}")).collect();
    let mut p = json!({ "objects": names });
    p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
    s.execute("mesh.merge", p).unwrap()
}

fn mesh_of(s: &Session, id: &Value) -> Mesh {
    s.doc.mesh(surfacecraft_doc::ObjectId(id.as_u64().unwrap())).unwrap().mesh.as_ref().clone()
}

/// (share of points sampled over `m` within `tol` of `truth`, worst distance of a vertex of `m`).
fn against_truth(m: &Mesh, truth: &Mesh, tol: f64) -> (f64, f64) {
    let bvh = Bvh::build(truth);
    let pts = m.sample_points(20_000, &mut Rng::new(9));
    let within = pts.iter().filter(|(p, _)| bvh.closest(p, tol).is_some()).count() as f64 / pts.len() as f64;
    let worst = m.positions.iter().map(|p| bvh.closest(p, f64::INFINITY).unwrap().distance).fold(0.0, f64::max);
    (within, worst)
}

/// Share of points sampled over `truth` within `tol` of `m` (how much of the part the merge covers).
fn coverage_of_truth(m: &Mesh, truth: &Mesh, tol: f64) -> f64 {
    let bvh = Bvh::build(m);
    let pts = truth.sample_points(20_000, &mut Rng::new(10));
    pts.iter().filter(|(p, _)| bvh.closest(p, tol).is_some()).count() as f64 / pts.len() as f64
}

fn codes(out: &Value) -> Vec<String> {
    out["issues"].as_array().unwrap().iter().map(|i| i["code"].as_str().unwrap().to_string()).collect()
}

struct Measured {
    within: f64,
    worst: f64,
    /// The same two of the views together (what the merge starts from).
    within_in: f64,
    worst_in: f64,
    covered: f64,
    out: Value,
    mesh: Mesh,
}

fn run_case(label: &str, views: &[Mesh], truth: &Mesh) -> Measured {
    let mut s = session(views);
    let t0 = std::time::Instant::now();
    let out = merge(&mut s, views.len(), json!({}));
    let secs = t0.elapsed().as_secs_f64();
    let tol = out["tolerance"].as_f64().unwrap();
    let mesh = mesh_of(&s, &out["id"]);
    let (within, worst) = against_truth(&mesh, truth, tol);
    let covered = coverage_of_truth(&mesh, truth, tol);
    let mut all = Mesh::default();
    views.iter().for_each(|v| all.append(v));
    let (within_in, worst_in) = against_truth(&all, truth, tol);
    eprintln!(
        "{label}: tol {tol:.3} | pieces {} loops {} genus {} nm {} unoriented {} pinched {} | truth within {within:.4} (views {within_in:.4}) worst {worst:.3} (views {worst_in:.3}) covered {covered:.4} | removed within {:.4} worst {:.3} | seam cut {} created {} open {} ({:.1} mm) strays {} | crossings {} unwelded {} stacked {:.4} | {} triangles | {secs:.2} s | issues {:?}",
        out["after"]["pieces"],
        out["after"]["loops"],
        out["after"]["genus"],
        out["after"]["non_manifold_edges"],
        out["after"]["unoriented_edges"],
        out["after"]["pinched_vertices"],
        out["checks"]["removed_within_share"],
        out["checks"]["removed_worst"].as_f64().unwrap(),
        out["seam"]["cut_edges"],
        out["seam"]["triangles_created"],
        out["seam"]["open_edges"],
        out["seam"]["open_length"].as_f64().unwrap(),
        out["seam"]["strays_dropped"],
        out["checks"]["added_crossings"],
        out["checks"]["unwelded_vertices"],
        out["checks"]["stacked_share"].as_f64().unwrap(),
        out["triangles"],
        codes(&out)
    );
    Measured { within, worst, within_in, worst_in, covered, out, mesh }
}

/// The checks every merged result passes: manifold, oriented, no added triangle crosses another,
/// welds tiny; and every issue raised exactly when its numbers say so.
fn assert_clean(label: &str, m: &Measured) {
    let (a, c, seam) = (&m.out["after"], &m.out["checks"], &m.out["seam"]);
    assert_eq!(a["non_manifold_edges"], json!(0), "{label}");
    assert_eq!(a["unoriented_edges"], json!(0), "{label}");
    assert_eq!(c["added_crossings"], json!(0), "{label}");
    // a weld moves a seam vertex by at most the weld share of the tolerance
    let tol = m.out["tolerance"].as_f64().unwrap();
    let bound = surfacecraft_geom::merge::WELD_SHARE * tol * (1.0 + 1e-9);
    assert!(c["max_vertex_move"].as_f64().unwrap() <= bound, "{label}: welds are bounded");
    assert!(!m.out["refused"].as_bool().unwrap(), "{label}");
    let has = |code: &str| codes(&m.out).iter().any(|k| k == code);
    assert_eq!(has("seam_open"), seam["open_edges"].as_u64().unwrap() > 0, "{label}");
    assert_eq!(has("removed_not_covered"), c["removed_within_share"].as_f64().unwrap() < 0.99, "{label}");
    let pinched = a["pinched_vertices"].as_u64().unwrap() > m.out["before"]["pinched_vertices"].as_u64().unwrap();
    assert_eq!(has("pinched_vertices"), pinched, "{label}");
    assert_eq!(has("unwelded_vertices"), c["unwelded_vertices"].as_u64().unwrap() > 0, "{label}");
    assert_eq!(has("layers_stacked"), c["stacked_share"].as_f64().unwrap() > 0.005, "{label}");
}

#[test]
fn gate_m4_merge_blob_views_into_one_piece_with_at_most_two_open_loops() {
    let truth = blob_at(240);
    for n in [2, 4, 8] {
        for perturbed in [false, true] {
            let mut views = blob_views_n(n);
            if perturbed {
                // within the tolerance: at most 0.06 degrees about the centre and 0.1 mm (largest
                // vertex displacement about 0.13 mm against a tolerance near 0.4 mm)
                for (k, v) in views.iter_mut().enumerate().skip(1) {
                    *v = v.transformed(&nudge(40 + k as u64, Point3::origin(), 0.06, 0.1));
                }
            }
            let label = format!("blob {n} views{}", if perturbed { " perturbed" } else { "" });
            let m = run_case(&label, &views, &truth);
            assert_clean(&label, &m);
            // the union of the views covers the whole blob: measured one closed piece of genus 0, no
            // point twice, no open seam edge, no second layer (stacked share 0.0000). Bounded, not
            // exact (the blob is made with sin and cos, whose last bit differs between the math
            // libraries): one piece (a second piece needs the seam to leave a whole band around the
            // blob open, not a triangle or two that a last bit can move; measured one piece on all
            // three systems in CI and on 90 of 90 noise draws of the sweep), at most 2 open loops (the noise sweep below finds a loop left at
            // the pole in a few draws), open seam edges at most 2 % of the cut edges, unwelded
            // vertices at most 2, stacked share at most 0.001
            let at = |k: &str| m.out["after"][k].as_u64().unwrap();
            let seam = |k: &str| m.out["seam"][k].as_u64().unwrap();
            assert!(at("pieces") == 1 && at("loops") <= 2, "{label}: {}", m.out["after"]);
            assert!(seam("open_edges") * 50 <= seam("cut_edges").max(1), "{label}: {}", m.out["seam"]);
            assert!(m.out["checks"]["unwelded_vertices"].as_u64().unwrap() <= 2, "{label}");
            assert!(m.out["checks"]["stacked_share"].as_f64().unwrap() <= 0.001, "{label}");
            // measured: 1.0000 of the merge within the tolerance of the true blob and 1.0000 of the
            // blob covered (bounds 0.98: a miss share of 0.02 against 0.0000); worst vertex 0.25 mm,
            // as the views themselves (bound 1.0)
            assert!(m.within >= 0.98, "{label}: {}", m.within);
            assert!(m.covered >= 0.98, "{label}: {}", m.covered);
            assert!(m.worst <= 1.0, "{label}: {}", m.worst);
            // measured 1.0000 of the removed area within the tolerance of the result (bound 0.98)
            assert!(m.out["checks"]["removed_within_share"].as_f64().unwrap() >= 0.98, "{label}");
        }
    }
}

#[test]
fn gate_m4_merge_bracket_views_leave_what_the_union_leaves_open() {
    let truth = visible_bracket_at(40, 160, 40);
    let t = surfacecraft_geom::merge::topology(&truth);
    eprintln!("visible bracket: {t:?}");
    for n in [2, 4, 8] {
        for perturbed in [false, true] {
            let mut views = bracket_views_n(n);
            if perturbed {
                for (k, v) in views.iter_mut().enumerate().skip(1) {
                    *v = v.transformed(&nudge(50 + k as u64, Point3::new(30.0, 20.0, 15.0), 0.04, 0.1));
                }
            }
            let label = format!("bracket {n} views{}", if perturbed { " perturbed" } else { "" });
            let m = run_case(&label, &views, &truth);
            assert_clean(&label, &m);
            // the visible bracket is three pieces (box with two openings, boss, half sphere) whose
            // rims touch without sharing vertices: what the union of the views leaves open is its own
            // the merge never joins the three pieces and never closes an opening of the part: at least
            // the pieces and loops of the truth (the seam that runs into the rims of the pieces stays
            // partly open: reported)
            // measured: pieces 3 (4 with 8 perturbed views: a patch the seam did not join), loops 5 to 9
            assert!(m.out["after"]["pieces"].as_u64().unwrap() as usize >= t.pieces, "{label}");
            assert!(m.out["after"]["loops"].as_u64().unwrap() as usize >= t.loops, "{label}");
            // the views themselves are 3 mm off the truth in places (coarse triangles over the cut
            // holes): the merge is measured against them. Measured: within 0.9983 to 0.9991 against
            // 0.9991 to 0.9995 of the views (bound: 0.01 below); worst 1.5 to 3.0 mm against 3.0
            // (bound: the views' worst plus the weld radius, an invariant: every vertex of the merge is
            // a vertex of a view at its place, or one welded by at most the weld radius, a quarter of
            // the tolerance; seam triangles add no vertex); covered 0.9922 to 0.9979 (bound 0.95: a miss share 6 times the
            // largest measured)
            assert!(m.within >= m.within_in - 0.01, "{label}: {} against {}", m.within, m.within_in);
            let tol = m.out["tolerance"].as_f64().unwrap();
            assert!(
                m.worst <= m.worst_in + surfacecraft_geom::merge::WELD_SHARE * tol,
                "{label}: {} against {}",
                m.worst,
                m.worst_in
            );
            assert!(m.covered >= 0.95, "{label}: {}", m.covered);
        }
    }
}

fn to_solid(mesh: &Mesh) -> Value {
    let mut s = Session::new();
    s.doc.add_mesh("scan", mesh.clone());
    s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap()
}

#[test]
fn gate_m4_merged_blob_wraps_like_one_full_scan() {
    let case = run_case("blob 4 views for the wrap", &blob_views_n(4), &blob_at(240));
    assert_eq!(case.out["after"]["pieces"], json!(1), "the four views merge into one piece");
    let merged = case.mesh;
    let single = noisy(blob_at(56), 900);
    let (a, b) = (to_solid(&merged), to_solid(&single));
    for (label, out) in [("merged", &a), ("single", &b)] {
        eprintln!(
            "wrap of the {label} blob: verdict {} patches {} within {} max {} coverage {} issues {:?}",
            out["verdict"],
            out["patches"],
            out["deviation"]["within"],
            out["deviation"]["max"],
            out["deviation"]["coverage"],
            out["issues"].as_array().map(|v| v.iter().map(|i| i["code"].clone()).collect::<Vec<_>>())
        );
    }
    let rank = |v: &Value| match v["verdict"].as_str() {
        Some("pass") => 2,
        Some("review") => 1,
        _ => 0,
    };
    assert!(
        rank(&a) >= rank(&b),
        "the merged blob wraps no worse than one full scan: {} against {}",
        a["verdict"],
        b["verdict"]
    );
}

/// A regression alarm for the sweep below, well under the measured 83 of 90 closed (88 before the
/// round-1 review fixes, c0ef6b3; the open ones leave one loop at the pole of the blob where the views
/// meet in a point). The per-case invariants (`assert_clean`, one piece) are the gate.
const SWEEP_CLOSED_FLOOR: usize = 60;

/// The blob gate over thirty more noise series (local: the topology of the gate must not depend on
/// the noise draw). Fails when fewer cases than the floor close.
#[test]
#[ignore = "sweep over noise series, run locally"]
fn blob_gate_over_noise_series() {
    let truth = blob_at(240);
    let mut closed_cases = 0;
    for series in 1..=30 {
        for n in [2, 4, 8] {
            let m = run_case(&format!("series {series} blob {n} views"), &blob_views_series(n, series), &truth);
            assert_clean("sweep", &m);
            let closed = m.out["after"]["pieces"] == json!(1) && m.out["after"]["loops"] == json!(0);
            eprintln!("SWEEP series {series} n {n}: {}", if closed { "closed" } else { "OPEN" });
            closed_cases += usize::from(closed);
            assert_eq!(m.out["after"]["pieces"], json!(1), "series {series} n {n}: one piece on every draw");
        }
    }
    eprintln!("SWEEP closed {closed_cases} of 90");
    assert!(closed_cases >= SWEEP_CLOSED_FLOOR, "{closed_cases} closed against a floor of {SWEEP_CLOSED_FLOOR}");
}

//! `scan.to_solid` paths of P9-O7 that only had unit tests and the local corpus, now through the
//! public command on synthetic scans: the refusal to rebuild the mesh, the list of attempts and the
//! shared budget of extra attempts, and the retry with a larger patch limit.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::synth_wall::{Wall, wall};
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

fn weld(mut m: Mesh) -> Mesh {
    m.weld(1e-9);
    m
}

fn session_with(m: Mesh) -> Session {
    let mut s = Session::new();
    s.doc.add_mesh("scan", m);
    s
}

fn codes(list: &Value) -> Vec<&str> {
    list.as_array().into_iter().flatten().filter_map(|i| i["code"].as_str()).collect()
}

/// A closed organic scan: a sphere of radius `r` with a wavy skin, noise 0.02.
fn wavy(r: f64, res: usize) -> Mesh {
    let mut m = weld(synth::sphere(Point3::origin(), 1.0, res, res / 2));
    for p in &mut m.positions {
        let d = p.coords;
        p.coords = d * r * (1.0 + 0.08 * (3.0 * d.x).sin() * (3.0 * 0.7 * d.y).cos());
    }
    synth::add_noise(&mut m, 0.02, &mut Rng::new(3));
    m
}

/// Fins: a triangle on an existing edge, its tip 0.15 mm off the surface (an edge of three triangles).
fn add_fins(m: &mut Mesh, count: usize, rng: &mut Rng) {
    let normals = m.vertex_normals();
    for _ in 0..count {
        let t = m.triangles[rng.index(m.triangles.len())];
        let (a, b) = (t[0] as usize, t[1] as usize);
        let tip = Point3::from(
            (m.positions[a].coords + m.positions[b].coords) / 2.0 + (normals[a] + normals[b]).normalize() * 0.15,
        );
        m.positions.push(tip);
        let n = (m.positions.len() - 1) as u32;
        m.triangles.push([t[1], t[0], n]);
    }
}

/// A closed scan with thirty fins (strong evidence of topological noise) whose orientation is spoilt
/// by turning every `every`th triangle around, so that the rebuild cannot read inside from outside.
fn finned_scan(every: usize) -> Mesh {
    let mut m = wavy(8.0, 80);
    m.triangles.iter_mut().step_by(every).for_each(|t| t.swap(1, 2));
    add_fins(&mut m, 30, &mut Rng::new(4));
    m
}

fn to_solid(m: Mesh, params: Value) -> (Session, Value) {
    let mut s = session_with(m);
    let mut all = json!({"scan": "scan", "method": "freeform"});
    if let (Some(a), Some(b)) = (all.as_object_mut(), params.as_object()) {
        a.extend(b.clone());
    }
    let out = s.execute("scan.to_solid", all).unwrap();
    (s, out)
}

fn mesh_names(s: &Session) -> Vec<String> {
    s.doc.meshes.iter().map(|m| m.name.clone()).collect()
}

/// Strong topology evidence (thirty fins) but a scan whose orientation cannot be read: `mesh.remesh`
/// refuses, and the pipeline wraps the scan as it is. The refusal is a `remesh` step that says why and
/// carries the evidence; nothing of the rebuild is left in the document; the result is exactly that of
/// `rebuild: false` (same wrap, same figures), and, being a wrap of a scan with fins and a reversed
/// fifth of its triangles, it is not a pass but a `review` that names the defects it could not fix.
#[test]
fn a_refused_rebuild_wraps_the_scan_as_it_is_and_says_why() {
    // Act
    let (s, out) = to_solid(finned_scan(5), json!({}));
    let (_, plain) = to_solid(finned_scan(5), json!({"rebuild": false}));

    // Assert: the refusal is reported, with the evidence that called for the rebuild
    let steps = out["preparation"].as_array().unwrap();
    let refused: Vec<&Value> = steps.iter().filter(|p| p["step"] == "remesh").collect();
    assert_eq!(refused.len(), 1, "{steps:?}");
    let step = refused[0];
    assert!(step["numbers"]["refused"].as_str().unwrap().contains("not consistently oriented"), "{step}");
    assert!(step["reason"].as_str().unwrap().contains("not done:"), "{step}");
    assert!(step["numbers"]["evidence"]["non_manifold_edges"].as_u64().unwrap() >= 20, "{step}");
    // the scan was processed without the rebuild: no rebuilt mesh, one body, a wrap of the scan itself
    assert_eq!(mesh_names(&s), ["scan"]);
    assert_eq!(s.doc.bodies.len(), 1);
    assert!(out["body"].is_object() && out["patches"].as_u64().unwrap() > 0, "{out}");
    // the same result as without the rebuild, which has no `remesh` step at all
    assert!(plain["preparation"].as_array().unwrap().is_empty(), "{}", plain["preparation"]);
    for field in ["verdict", "patches", "deviation", "issues"] {
        assert_eq!(out[field], plain[field], "{field}");
    }
    // the verdict the code documents for it: issues, but the solid still represents the scan: review
    assert_eq!(out["verdict"], "review", "{out}");
    let found = codes(&out["issues"]);
    assert!(found.contains(&"non_manifold_edges"), "{found:?}");
}

/// The same scan with its orientation intact is rebuilt, which is what the refusal above withholds:
/// a `remesh` step with figures and a rebuilt mesh in the document.
#[test]
fn a_scan_with_the_same_evidence_and_readable_orientation_is_rebuilt() {
    let (s, out) = to_solid(finned_scan(41), json!({}));

    let step = out["preparation"].as_array().unwrap().iter().find(|p| p["step"] == "remesh").unwrap();
    assert!(step["numbers"]["refused"].is_null() && step["numbers"]["triangles"].as_u64().unwrap() > 0, "{step}");
    assert_eq!(mesh_names(&s), ["scan", "solid-remesh"]);
}

/// A box 40 x 30 x 20 whose top is a wave of 2.5 mm: the five flat sides are most of the surface (the
/// primitives method is tried first), the top is not a primitive (it ends in a `fail`), and the wrap of
/// the whole part passes.
fn wavy_top_box(dx: f64) -> Mesh {
    let mut m = weld(synth::cuboid(Point3::origin(), Vec3::new(40.0, 30.0, 20.0), 24));
    let top = m.positions.iter().map(|p| p.z).fold(f64::MIN, f64::max);
    for p in &mut m.positions {
        if p.z > top - 1e-9 {
            p.z += 2.5 * (p.x / 5.0).sin() * (p.y / 4.0).cos();
        }
        p.x += dx;
    }
    m
}

/// `(method, verdict, returned, skipped reason)` of each entry of an attempt list.
fn attempts_of(component: &Value) -> Vec<(String, Option<String>, bool, Option<String>)> {
    component["attempts"]
        .as_array()
        .unwrap()
        .iter()
        .map(|a| {
            (
                a["method"].as_str().unwrap().to_string(),
                a["verdict"].as_str().map(str::to_string),
                a["returned"] == true,
                a["skipped"]["reason"].as_str().map(str::to_string),
            )
        })
        .collect()
}

/// Three such parts in one scan, every default. Each part tries the primitives first and fails; the
/// wrap that follows is an extra attempt of the call's one budget of two. The first two parts get it
/// (their lists name primitives `fail`, then freeform `pass`, the second returned); the third finds the
/// budget spent: its list names the primitives result as returned and the wrap and the hybrid attempt
/// as skipped, with the reason, and no extra attempt ran beyond the budget.
#[test]
fn the_attempts_of_the_parts_of_one_call_share_a_budget_of_two_extra_attempts() {
    // Arrange
    let mut all = Mesh::default();
    (0..3).for_each(|k| all.append(&wavy_top_box(60.0 * f64::from(k))));
    let mut s = session_with(all);

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan"})).unwrap();

    // Assert
    let parts = out["components"].as_array().unwrap();
    assert_eq!(parts.len(), 3, "{out}");
    let me = |m: &str, v: Option<&str>, r: bool, why: Option<&str>| {
        (m.to_string(), v.map(str::to_string), r, why.map(str::to_string))
    };
    for part in &parts[..2] {
        assert_eq!(
            attempts_of(part),
            [me("primitives", Some("fail"), false, None), me("freeform", Some("pass"), true, None)],
            "{part}"
        );
        assert_eq!((part["method"].as_str(), part["verdict"].as_str()), (Some("freeform"), Some("pass")));
    }
    let spent = "the call has used its 2 extra attempts";
    assert_eq!(
        attempts_of(&parts[2]),
        [
            me("primitives", Some("fail"), true, None),
            me("freeform", None, false, Some(spent)),
            me("hybrid", None, false, Some(spent)),
        ],
        "{}",
        parts[2]
    );
    let skipped = &parts[2]["attempts"][1]["skipped"]["call"];
    assert_eq!(
        (skipped["command"].as_str(), skipped["params"]["method"].as_str()),
        (Some("scan.to_solid"), Some("freeform"))
    );
    // the third part keeps the primitives result, which the entry marks as the returned one
    assert_eq!((parts[2]["method"].as_str(), parts[2]["verdict"].as_str()), (Some("primitives"), Some("fail")));
    // across the call: three first attempts and exactly two extra ones that ran
    let ran: usize = parts.iter().map(|p| attempts_of(p).iter().filter(|a| a.3.is_none()).count()).sum();
    assert_eq!(ran, 3 + 2);
    assert_eq!(out["verdict"], "fail", "the worst part's verdict: {out}");
    assert_eq!(s.doc.bodies.len(), 3);
}

/// The first attempt that stops at an explicit `max_patches` is never retried (an explicit limit is a
/// promise): no `retry`, the issue says the limit stopped it, and the larger limit is offered as a next
/// step instead.
#[test]
fn an_explicit_patch_limit_is_not_retried_and_the_larger_one_is_offered() {
    let (s, out) = to_solid(wavy(8.0, 80), json!({"max_patches": 12}));

    assert!(out["retry"].is_null(), "{}", out["retry"]);
    let message = out["issues"].as_array().unwrap().iter().find(|i| i["code"] == "out_of_tolerance").unwrap();
    assert!(message["message"].as_str().unwrap().contains("stopped by the patch limit"), "{message}");
    let offer = out["next_steps"].as_array().unwrap().iter().find(|n| n["command"] == "surface.wrap").unwrap();
    assert_eq!(offer["params"]["max_patches"], 60_000, "{offer}");
    assert_eq!(s.doc.bodies.len(), 1);
}

/// A wall that is all tunnels: one-cell tunnels every three cells over the whole panel.
fn sponge_wall() -> Mesh {
    let tubes: Vec<(usize, usize, usize)> =
        (2..57).step_by(3).flat_map(|j| (2..57).step_by(3).map(move |i| (i, j, 1))).collect();
    let mut m = wall(&Wall { size: 60.0, cells: 60, gap: 0.6, bend: 5.0, tubes });
    synth::add_noise(&mut m, 0.01, &mut Rng::new(12));
    m
}

/// The first wrap of the sponge wall stops at the default limit of 12 000 patches far short of the
/// tolerance, so it is wrapped once more with a limit five times larger; the second network is clearly
/// better and replaces the first, and the result reports the retry: both limits, both patch counts,
/// both shares within the tolerance, that it was kept and why. One body is left, under the one name.
/// (A second network that overflows its own limit would be the 60 000th patch: about 450 s on a real
/// scan, so the rule that it is not retried again stays with the unit tests, which force the flag.)
#[test]
fn a_wrap_stopped_by_the_default_patch_limit_is_retried_once_with_a_larger_limit() {
    let (s, out) = to_solid(sponge_wall(), json!({}));

    let retry = &out["retry"];
    assert!(retry.is_object(), "{out}");
    assert_eq!((retry["cap_from"].as_u64(), retry["cap_to"].as_u64()), (Some(12_000), Some(60_000)), "{retry}");
    // measured 12 000 (macOS); the first network stops at its limit
    assert!((7_900..=12_000).contains(&retry["patches_from"].as_u64().unwrap()), "{retry}");
    assert_eq!((retry["kept"].clone(), retry["reason"].clone()), (json!(true), json!("better")), "{retry}");
    // measured 0.85 before and 0.99 after: far outside the margin the retry needs
    let (from, to) = (retry["within_from"].as_f64().unwrap(), retry["within_to"].as_f64().unwrap());
    assert!(from < 0.9 && to > 0.95 && to > from + 0.05, "{retry}");
    // the network that is the result is the second one
    assert_eq!(out["patches"], retry["patches_to"], "{out}");
    assert!(out["patches"].as_u64().unwrap() > retry["patches_from"].as_u64().unwrap());
    assert_eq!(s.doc.bodies.len(), 1, "the first network was removed");
    assert_eq!(s.doc.bodies[0].name, "solid");
    // a larger limit that was tried in this call is not offered again
    assert!(
        out["next_steps"].as_array().unwrap().iter().all(|n| n["command"] != "surface.wrap"),
        "{}",
        out["next_steps"]
    );
}

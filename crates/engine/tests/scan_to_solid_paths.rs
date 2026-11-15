//! `scan.to_solid` paths of P9-O7 that only had unit tests and the local corpus, now through the
//! public command on synthetic scans: the refusal to rebuild the mesh, the list of attempts and the
//! shared budget of extra attempts, and the retry with a larger patch limit.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
mod scan_common;
use scan_common::{inject_fins, session_with, wavy, weld};
use surfacecraft_geom::synth_wall::{Wall, wall};
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

fn codes(list: &Value) -> Vec<&str> {
    list.as_array().into_iter().flatten().filter_map(|i| i["code"].as_str()).collect()
}

/// A closed scan with thirty fins (strong evidence of topological noise) whose orientation is spoilt
/// by turning every `every`th triangle around, so that the rebuild cannot read inside from outside.
fn finned_scan(every: usize) -> Mesh {
    let mut m = wavy(Point3::origin(), 8.0, 3.0, 80);
    m.triangles.iter_mut().step_by(every).for_each(|t| t.swap(1, 2));
    inject_fins(&mut m, 30, &mut Rng::new(4));
    m
}

fn to_solid(m: Mesh, params: Value) -> (Session, Value) {
    let mut s = session_with("scan", m);
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
/// third of its triangles, it is not a pass and names the defects it could not fix. Measured (macOS):
/// 56 % of the columns of the scan irregular against the limit of 25 % (2.2 times), the step 3 being
/// coprime with the 160 triangles of a row, so the reversed ones do not line up.
#[test]
fn a_refused_rebuild_wraps_the_scan_as_it_is_and_says_why() {
    // Act
    let (s, out) = to_solid(finned_scan(3), json!({}));
    let (_, plain) = to_solid(finned_scan(3), json!({"rebuild": false}));

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
    // the rebuild that would have repaired the scan did not happen, so the defects are still there
    assert_ne!(out["verdict"], "pass", "{out}");
    let found = codes(&out["issues"]);
    assert!(found.contains(&"non_manifold_edges"), "{found:?}");
}

/// The same scan with its orientation intact is rebuilt, which is what the refusal above withholds:
/// a `remesh` step with figures. The rebuilt mesh stays in the document when the wrap of it is the
/// result and is removed when the wrap of the scan as it is was judged better; the step says which.
#[test]
fn a_scan_with_the_same_evidence_and_readable_orientation_is_rebuilt() {
    let (s, out) = to_solid(finned_scan(41), json!({}));

    let step = out["preparation"].as_array().unwrap().iter().find(|p| p["step"] == "remesh").unwrap();
    assert!(step["numbers"]["refused"].is_null() && step["numbers"]["triangles"].as_u64().unwrap() > 0, "{step}");
    let said_discarded = out["preparation"]
        .as_array()
        .unwrap()
        .iter()
        .any(|p| p["step"] == "remesh" && p["reason"].as_str().is_some_and(|r| r.contains("discarded")));
    // exclusive and complete: discarded means only the scan is left, else the rebuilt mesh stays
    let expected: &[&str] = if said_discarded { &["scan"] } else { &["scan", "solid-remesh"] };
    assert_eq!(mesh_names(&s), expected, "{}", out["preparation"]);
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

/// Three such parts in one scan, every default. Each part tries the primitives first and cannot
/// close the wavy top, so each wants extra attempts (the wrap, and the hybrid method after a wrap that
/// is no pass); they draw from the call's one budget of two. Whatever the wrap of a part comes to, the
/// call runs exactly two extra attempts, and once they are spent the lists of the parts that come
/// later name the wrap and the hybrid method as skipped with the reason. Measured (macOS): the wrap of
/// a part puts 0.996 of it within the tolerance and passes, so parts 1 and 2 take one extra each and
/// part 3 is refused both. The assertions do not depend on the verdicts: only that exactly two extra
/// attempts run, and that a skipped entry sits in a part at or after the last part that ran one (that
/// part may itself have spent the last attempt and had a later attempt of its own refused).
#[test]
fn the_attempts_of_the_parts_of_one_call_share_a_budget_of_two_extra_attempts() {
    // Arrange
    let mut all = Mesh::default();
    (0..3).for_each(|k| all.append(&wavy_top_box(60.0 * f64::from(k))));
    let mut s = session_with("scan", all);

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan"})).unwrap();

    // Assert
    let parts = out["components"].as_array().unwrap();
    assert_eq!(parts.len(), 3, "{out}");
    let lists: Vec<_> = parts.iter().map(attempts_of).collect();
    let ran =
        |list: &[(String, Option<String>, bool, Option<String>)]| list.iter().filter(|a| a.3.is_none()).count() - 1;
    // every part starts with the primitives; the first part, with a full budget, runs the wrap
    assert!(lists.iter().all(|l| l[0].0 == "primitives" && l[0].3.is_none()), "{lists:?}");
    assert_eq!((lists[0][1].0.as_str(), lists[0][1].3.as_deref()), ("freeform", None), "{lists:?}");
    // the call ran exactly two extra attempts, however it spent them
    assert_eq!(lists.iter().map(|l| ran(l)).sum::<usize>(), 2, "{lists:?}");
    // an entry is skipped only for the spent budget, and only in parts at or after the last that spent it
    let spent = "the call has used its 2 extra attempts";
    let last_spender = lists.iter().rposition(|l| ran(l) > 0).unwrap();
    let skipped_parts: Vec<usize> =
        lists.iter().enumerate().filter(|(_, l)| l.iter().any(|a| a.3.is_some())).map(|(k, _)| k).collect();
    assert!(!skipped_parts.is_empty() && skipped_parts.iter().all(|k| *k >= last_spender), "{lists:?}");
    for l in lists.iter().flatten().filter(|a| a.3.is_some()) {
        assert_eq!(l.3.as_deref(), Some(spent), "{l:?}");
    }
    // a skipped entry carries the call that runs that method alone
    let part = &parts[skipped_parts[0]];
    let entry = part["attempts"].as_array().unwrap().iter().find(|a| a["skipped"].is_object()).unwrap();
    assert_eq!(entry["skipped"]["call"]["command"], "scan.to_solid", "{entry}");
    assert_eq!(entry["skipped"]["call"]["params"]["method"], entry["method"], "{entry}");
    // a part whose extra attempts were refused keeps the primitives result, marked as returned
    assert!(lists[skipped_parts[0]][0].2, "{lists:?}");
    assert_eq!(s.doc.bodies.len(), 3);
}

/// The first attempt that stops at an explicit `max_patches` is never retried (an explicit limit is a
/// promise): no `retry`, the issue says the limit stopped it, and the larger limit is offered as a next
/// step instead.
#[test]
fn an_explicit_patch_limit_is_not_retried_and_the_larger_one_is_offered() {
    let (s, out) = to_solid(wavy(Point3::origin(), 8.0, 3.0, 80), json!({"max_patches": 12}));

    // the retry rule would otherwise apply: capped and 0.67 within the tolerance (it needs 0.25), so
    // only the explicit limit keeps it from running
    assert!(out["deviation"]["within"].as_f64().unwrap() >= 0.4, "{}", out["deviation"]);
    assert!(out["retry"].is_null(), "{}", out["retry"]);
    let message = out["issues"].as_array().unwrap().iter().find(|i| i["code"] == "out_of_tolerance").unwrap();
    assert!(message["message"].as_str().unwrap().contains("stopped by the patch limit"), "{message}");
    let offer = out["next_steps"].as_array().unwrap().iter().find(|n| n["command"] == "surface.wrap").unwrap();
    // the patch limit of the command (`PATCH_LIMIT` in `cmd/wrap.rs`, not public)
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
    // measured 12 000 (macOS): the first network stopped at its limit, short of the tolerance
    assert!(retry["patches_from"].as_u64().unwrap() >= 6_000, "{retry}");
    assert_eq!((retry["kept"].clone(), retry["reason"].clone()), (json!(true), json!("better")), "{retry}");
    // measured 0.85 before (the retry needs 0.25) and 0.99 after
    let (from, to) = (retry["within_from"].as_f64().unwrap(), retry["within_to"].as_f64().unwrap());
    assert!(from >= 0.25 && to > from, "{retry}");
    // the network that is the result is the second one
    assert_eq!(out["patches"], retry["patches_to"], "{out}");
    assert!(out["patches"].as_u64().unwrap() > retry["patches_from"].as_u64().unwrap());
    assert_eq!(s.doc.bodies.len(), 1, "the first network was removed");
    assert_eq!(s.doc.bodies[0].name, "solid");
}

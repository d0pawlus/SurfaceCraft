//! `align.pair_coarse` does not trust an orientation it cannot verify: a scan wound inconsistently,
//! with duplicated triangles, closed and turned inside out, or an open scan turned inside out against
//! the other, is never `unique` (the free-space conflicts read the side a triangle faces: with the
//! wrong side every conflict vanishes). A correctly wound pair is `unique` as before. Two open scans
//! that are both inverted are not detectable (see `align::pair::orientation`).
mod pair_common;
use pair_common::*;
use serde_json::{Value, json};
use surfacecraft_geom::Mesh;

fn run(fixed: &Mesh, moving: &Mesh) -> Value {
    let mut s = session(fixed, moving, &random_motion(3));
    pair(&mut s, json!({}))
}

/// The triangles of `m` with every `every`-th one turned over.
fn turned(m: &Mesh, every: usize) -> Mesh {
    let mut out = m.clone();
    for (i, t) in out.triangles.iter_mut().enumerate() {
        if i % every == 0 {
            t.swap(1, 2);
        }
    }
    out
}

fn assert_refused(name: &str, out: &Value, cause: &str) {
    eprintln!("{name}: {} confidence {} issue {}", out["verdict"], out["confidence"], out["orientation_issue"]);
    assert_ne!(out["verdict"], "unique", "{name}: {out}");
    assert!(out["confidence"].as_f64().unwrap() <= 0.25, "{name}");
    let issue = out["orientation_issue"].as_str().unwrap_or_default();
    assert!(issue.contains(cause), "{name}: {issue:?} does not name `{cause}`");
    assert!(out["note"].as_str().unwrap().contains("orientation"), "{name}: {}", out["note"]);
}

#[test]
fn a_correctly_wound_pair_is_unique_and_has_no_orientation_issue() {
    let (fixed, moving) = pair_of_trial("blob", 0);
    let out = run(&fixed, &moving);
    assert_eq!(out["verdict"], "unique", "{out}");
    assert!(out.get("orientation_issue").is_none(), "{out}");
}

#[test]
fn a_closed_scan_turned_inside_out_is_never_unique() {
    let (fixed, moving) = pair_of_trial("blob", 0);
    assert_refused("closed, inverted", &run(&fixed, &turned(&moving, 1)), "inside out");
}

#[test]
fn an_open_scan_turned_inside_out_against_the_other_is_never_unique() {
    let (full_f, full_m) = pair_of_trial("blob", 0);
    let fixed = cut(&full_f, |c| c.x <= 22.0 && c.z > -8.0);
    let moving = cut(&full_m, |c| c.x >= -22.0 && c.z > -8.0);
    assert_eq!(run(&fixed, &moving)["verdict"], "unique", "the pair is found when wound alike");
    assert_refused("open, inverted", &run(&fixed, &turned(&moving, 1)), "opposite ways");
}

#[test]
fn mixed_winding_is_never_unique() {
    let (fixed, moving) = pair_of_trial("blob", 0);
    assert_refused("mixed winding", &run(&fixed, &turned(&moving, 4)), "winding");
    assert_refused("mixed winding, fixed", &run(&turned(&fixed, 4), &moving), "winding");
}

#[test]
fn duplicated_triangles_are_never_unique() {
    let (fixed, moving) = pair_of_trial("blob", 0);
    let mut doubled = moving.clone();
    let extra: Vec<[u32; 3]> = moving.triangles.iter().step_by(5).copied().collect();
    doubled.triangles.extend(extra);
    assert_refused("duplicated triangles", &run(&fixed, &doubled), "winding");
}

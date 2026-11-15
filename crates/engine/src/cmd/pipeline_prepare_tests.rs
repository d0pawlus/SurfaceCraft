//! Tests of `pipeline_prepare.rs`.
use super::super::parts_report::Tally;
use super::*;
use serde_json::json;
use surfacecraft_geom::{Mesh, Point3, synth};

/// `n` equal closed balls far apart.
fn balls(n: usize) -> Mesh {
    let mut all = Mesh::default();
    for k in 0..n {
        let mut m = synth::sphere(Point3::new(30.0 * k as f64, 0.0, 0.0), 5.0, 16, 8);
        m.weld(1e-9);
        all.append(&m);
    }
    all
}

use super::super::parts::{PART_SHARE, plan_of};
use super::super::parts_report::{UNMODELLED_REVIEW_SHARE, collect_issues};
use crate::cmd::scan_split::Analysis;

const TOL: f64 = 0.2;

fn tally(left: (usize, f64)) -> Tally {
    Tally {
        left_out: left,
        by_reason: vec![(LeftOutReason::Debris, left.0, left.1)],
        cavities: (0, 0.0),
        beyond_cap: (0, 0.0),
        not_requested: (0, 0.0),
        modelled: 1,
        co_equal: 1,
        debris_pieces: left.0,
        listed: Vec::new(),
        total_area: 1.0,
        ms: 0,
    }
}

fn plan(a: Analysis) -> super::super::parts::Plan {
    let mesh = Mesh::default();
    plan_of(&Pieces::of(&mesh, a), (PART_SHARE, TOL), None)
}

fn issues_of(parts: &[(ComponentOut, ToSolidOut)], t: &Tally) -> (Vec<Issue>, bool) {
    let f = collect_issues(parts, t);
    (f.issues, f.cap)
}

fn settings_for_tests() -> Settings {
    Settings {
        scan: ObjectId(0),
        tolerance: 0.2,
        min_within: 0.99,
        min_triangles: None,
        max_patches: None,
        allow_retry: true,
        budget: super::super::budget::CallBudget::default(),
        rebuild: true,
        close_handles: false,
        handle_size: None,
        name: "solid".into(),
    }
}

/// A modelled part of `area` (of a scan of area 1) with `verdict`.
fn part(co_equal: bool, area: f64, verdict: Verdict) -> (ComponentOut, ToSolidOut) {
    let mut out = failure(&settings_for_tests(), ObjectId(1), "x");
    out.verdict = verdict;
    if verdict != Verdict::Fail {
        out.issues.clear();
    }
    let mut c = component_for_tests(&out);
    c.co_equal = co_equal;
    c.area = area;
    c.name = format!("Scan-part-{}", (area * 1000.0) as usize);
    (c, out)
}

#[test]
fn a_requested_subset_is_no_issue_and_parts_beyond_the_cap_are() {
    let none = tally((0, 0.0));
    let asked = Tally { not_requested: (25, 0.96), ..tally((0, 0.0)) };
    let (issues, capped_verdict) = issues_of(&[], &asked);
    assert!(issues.is_empty() && !capped_verdict, "a requested subset is no issue");
    // parts beyond the cap always keep the call from a pass, even at 0.7 % of the surface
    let over = Tally { beyond_cap: (2, 0.007), ..none };
    let (issues, capped_verdict) = issues_of(&[], &over);
    assert!(
        issues.iter().any(|i| i.code == "parts_beyond_limit" && i.message.contains("not modelled: 2 ("))
            && capped_verdict,
        "{issues:?}"
    );
    assert!(!issues.iter().any(|i| i.code == "unmodelled_pieces"), "no piece was left out: {issues:?}");
}

/// Pieces left out (1.5 %) and parts beyond the cap (0.7 %), each under 2 %, together over it: the
/// bound applies to their sum and `unmodelled_pieces` is raised; the left-out pieces alone are no issue.
#[test]
fn pieces_left_out_and_parts_beyond_the_cap_share_the_two_percent_bound() {
    let (issues, capped) = issues_of(&[], &tally((4, 0.015)));
    assert!(issues.is_empty() && !capped, "{issues:?}");
    let both = Tally { beyond_cap: (1, 0.007), ..tally((4, 0.015)) };
    let (issues, capped) = issues_of(&[], &both);
    let codes: Vec<&str> = issues.iter().map(|i| i.code.as_str()).collect();
    assert!(capped && codes.contains(&"unmodelled_pieces") && codes.contains(&"parts_beyond_limit"), "{codes:?}");
    // pieces left out (1.2 %) and a small part without a pass (1.0 %): each under, together over
    let parts = [part(true, 0.9, Verdict::Pass), part(false, 0.01, Verdict::Fail)];
    let (issues, capped) = issues_of(&parts, &tally((4, 0.012)));
    let codes: Vec<&str> = issues.iter().map(|i| i.code.as_str()).collect();
    assert!(capped && codes.contains(&"unmodelled_pieces") && codes.contains(&"parts_not_passed"), "{codes:?}");
    let (issues, capped) = issues_of(&parts, &tally((4, 0.009)));
    assert!(issues.is_empty() && !capped, "1.9 % in all: {issues:?}");
}

/// A small part without a pass counts against the 2 % bound with the pieces left out: at 1 % it is no
/// issue (its own verdict stays in its component), at 3 % the call is `review` with `parts_not_passed`;
/// a small part never fails the call, and a next step names the parts that did not pass either way.
#[test]
fn small_parts_without_a_pass_count_against_the_two_percent_bound() {
    let none = tally((0, 0.0));
    for (share, verdict, expected) in [
        (0.01, Verdict::Fail, false),
        (0.01, Verdict::Review, false),
        (0.03, Verdict::Fail, true),
        (0.03, Verdict::Review, true),
        (0.03, Verdict::Pass, false),
    ] {
        let parts = [part(true, 0.9, Verdict::Pass), part(false, share, verdict)];
        let (issues, capped) = issues_of(&parts, &none);
        assert_eq!(capped, expected, "{share} {verdict:?}: {issues:?}");
        assert_eq!(issues.iter().any(|i| i.code == "parts_not_passed"), expected, "{issues:?}");
        assert!(issues.iter().all(|i| i.code != "no_solid"), "a small part's own issues stay in its component");
        let steps = super::super::parts_report::next_steps_of(&parts, ObjectId(0), false);
        let named = steps.iter().any(|st| st.command == "scan.to_solid" && st.why.contains("did not pass: 1 ("));
        assert_eq!(named, verdict != Verdict::Pass, "{steps:?}");
    }
    let (issues, _) = issues_of(&[part(true, 0.9, Verdict::Pass), part(false, 0.03, Verdict::Fail)], &none);
    let message = &issues[0].message;
    assert!(
        message.contains("Scan-part-30 (fail)") && message.contains(": 1 (3.0 %"),
        "names keep their case: {message}"
    );
}

/// Small parts without a pass and pieces left out over the bound in one call: both issues.
#[test]
fn small_parts_not_passed_and_pieces_left_out_are_reported_together() {
    let parts = [part(true, 0.9, Verdict::Pass), part(false, 0.02, Verdict::Fail)];
    let (issues, capped) = issues_of(&parts, &tally((6, 0.03)));
    let codes: Vec<&str> = issues.iter().map(|i| i.code.as_str()).collect();
    assert!(capped && codes.contains(&"parts_not_passed") && codes.contains(&"unmodelled_pieces"), "{codes:?}");
}

#[test]
fn a_small_ball_is_a_small_part_and_the_components_word_is_checked() {
    let mut m = synth::sphere(Point3::new(0.0, 0.0, 0.0), 20.0, 64, 32);
    m.weld(1e-9);
    let mut small = synth::sphere(Point3::new(60.0, 0.0, 0.0), 4.0, 16, 8);
    small.weld(1e-9);
    m.append(&small);
    let a = scan_split::analyse(&m, MIN_SHARE, false);
    let p = plan_of(&Pieces::of(&m, a), (PART_SHARE, TOL), None);
    assert_eq!(
        (p.chosen.len(), p.co_equal, p.left_out.len()),
        (2, 1, 0),
        "4 % of the largest, 13.9 mm across: a small part"
    );
    assert_eq!(Components::requested(&Some(Components::Word("largest".into()))).unwrap(), Some(1));
    assert_eq!(Components::requested(&None).unwrap(), None);
    assert!(Components::requested(&Some(Components::Word("some".into()))).is_err());
    assert!(Components::requested(&Some(Components::Count(0))).is_err());
}

fn piece(area: f64) -> surfacecraft_geom::pieces::Piece {
    surfacecraft_geom::pieces::Piece {
        triangles: vec![0],
        vertices: 3,
        area,
        bbox: surfacecraft_geom::Aabb::default(),
        boundary_edges: 0,
        loops: Vec::new(),
        non_manifold_edges: 0,
        edges: 3,
        euler: 2,
        pinched_vertices: 0,
        unoriented_edges: 0,
    }
}

/// An outer shell of 1 000 mm2 around a cavity skin of 12 000 mm2: the shell is the part, whatever
/// the area of what is inside it.
#[test]
fn a_cavity_skin_larger_than_its_shell_does_not_turn_the_shell_into_a_minor_piece() {
    let a = Analysis {
        parts: vec![piece(1000.0), piece(12_000.0)],
        extra: Vec::new(),
        rest: Vec::new(),
        nests: vec![None, Some((0, true))],
    };
    let p = plan(a);
    assert_eq!(p.chosen, vec![0], "the shell is modelled");
    assert_eq!(p.cavities, vec![1]);
    assert!(p.left_out.is_empty());
    let none = Analysis {
        parts: vec![piece(1000.0), piece(12_000.0)],
        extra: Vec::new(),
        rest: Vec::new(),
        nests: vec![Some((1, true)), Some((0, true))],
    };
    assert_eq!(plan(none).chosen.len(), 1, "never an empty choice");
}

#[test]
fn two_percent_of_unmodelled_surface_is_the_boundary_of_the_review_cap() {
    let (issues, capped) = issues_of(&[], &tally((3, UNMODELLED_REVIEW_SHARE)));
    assert!(issues.is_empty() && !capped, "exactly 2 % is not above 2 %");
    let (issues, capped) = issues_of(&[], &tally((3, UNMODELLED_REVIEW_SHARE + 1e-4)));
    assert!(capped && issues[0].code == "unmodelled_pieces" && issues[0].message.contains("part_share"));
    assert!(issues[0].message.contains("debris 3, open 0"), "{}", issues[0].message);
    let (issues, capped) = issues_of(&[], &tally((3, UNMODELLED_REVIEW_SHARE - 1e-4)));
    assert!(issues.is_empty() && !capped);
}

#[test]
fn a_failed_primitives_result_gets_the_one_freeform_wrap_and_the_better_is_kept() {
    let set = settings_for_tests();
    let refusal = set.budget.refusal();
    let mut out = failure(&set, ObjectId(1), "x");
    assert!(!needs_freeform(Method::Auto, &out, &refusal), "a freeform failure is not repeated");
    out.method = Method::Primitives;
    assert!(needs_freeform(Method::Auto, &out, &refusal));
    assert!(!needs_freeform(Method::Freeform, &out, &refusal));
    assert!(!needs_freeform(Method::Primitives, &out, &refusal));
    // a primitives solid under review whose freeform attempt was refused by the spent budget gets the
    // part's wrap; refused for another reason (or not refused) it does not
    let body = BodyOut { id: ObjectId(7), faces: 6, edges: 12, vertices: 8, volume: Some(1.0) };
    out.body = Some(body.clone());
    out.verdict = Verdict::Review;
    assert!(!needs_freeform(Method::Auto, &out, &refusal));
    let skipped = |reason: &str| super::super::auto::Attempt {
        method: Method::Freeform,
        verdict: None,
        issues: Vec::new(),
        seconds: 0.0,
        returned: false,
        error: None,
        skipped: Some(super::super::auto::Skipped {
            reason: reason.to_string(),
            call: super::super::NextStep { command: "scan.to_solid".into(), params: json!({}), why: String::new() },
        }),
    };
    out.attempts = vec![skipped("the caller asked for one method")];
    assert!(!needs_freeform(Method::Auto, &out, &refusal));
    out.attempts = vec![skipped(&refusal)];
    assert!(needs_freeform(Method::Auto, &out, &refusal));
    out.verdict = Verdict::Fail;
    let mut wrapped = failure(&set, ObjectId(1), "y");
    assert!(!wins(&wrapped, &out), "no solid never wins");
    wrapped.body = Some(body);
    wrapped.verdict = Verdict::Pass;
    assert!(wins(&wrapped, &out));
    assert!(!wins(&out, &wrapped), "a failed result does not beat a pass");
}

/// A cancellation between two parts (set by the hook after the first part has been modelled, so no
/// timing is involved) is a cancelled error and leaves neither the bodies nor the meshes of the parts.
#[test]
fn a_cancel_between_parts_leaves_no_body_and_no_part_mesh() {
    let mut s = Session::new();
    let mut all = Mesh::default();
    for k in 0..3 {
        let mut m = synth::sphere(Point3::new(40.0 * f64::from(k), 0.0, 0.0), 8.0, 40, 20);
        m.weld(1e-9);
        all.append(&m);
    }
    // two small parts after the co-equal ones: the loop is cancelled before they are reached
    for k in 0..2 {
        let mut m = synth::sphere(Point3::new(200.0 + 30.0 * f64::from(k), 0.0, 0.0), 6.0, 24, 12);
        m.weld(1e-9);
        all.append(&m);
    }
    let raw = all.clone();
    s.doc.add_mesh("scan", all);
    let flag = s.cancel.clone();
    let seen = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count = seen.clone();
    *AFTER_PART.lock().unwrap() = Some(Box::new(move |n| {
        count.store(n + 1, std::sync::atomic::Ordering::SeqCst);
        if n == 0 {
            flag.cancel();
        }
    }));
    let r = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "part_share": 0.5}));
    *AFTER_PART.lock().unwrap() = None;
    let e = r.expect_err("cancelled between parts").to_string();
    assert!(e.contains("cancelled"), "{e}");
    assert_eq!(seen.load(std::sync::atomic::Ordering::SeqCst), 1, "the second part was never started");
    assert!(s.doc.bodies.is_empty() && s.doc.meshes.len() == 1);
    assert!(s.doc.regions.is_empty() && s.doc.primitives.is_empty());
    let scan = s.doc.meshes.first().map(|m| m.mesh.clone()).unwrap();
    assert_eq!(
        (scan.positions.len(), scan.triangles.clone()),
        (raw.positions.len(), raw.triangles),
        "the scan as it was"
    );
}

/// A small part whose run ends in an error: the call goes on, the part is a failed component with
/// `no_solid`, and the call reports it (`parts_not_passed`, `review`).
#[test]
fn an_error_inside_a_small_part_is_a_failed_component_and_the_call_goes_on() {
    let mut s = Session::new();
    let mut all = synth::sphere(Point3::origin(), 30.0, 96, 48);
    all.weld(1e-9);
    for k in 0..2 {
        let mut m = synth::sphere(Point3::new(80.0 + 30.0 * f64::from(k), 0.0, 0.0), 6.0, 24, 12);
        m.weld(1e-9);
        all.append(&m);
    }
    s.doc.add_mesh("scan", all);
    *FAIL_NAME.lock().unwrap() = Some("probe-errors-3".into());
    let r = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "name": "probe-errors"}));
    *FAIL_NAME.lock().unwrap() = None;
    let out = r.unwrap();
    let comps = out["components"].as_array().unwrap();
    assert_eq!(comps.len(), 3, "{out}");
    let failed = &comps[2];
    assert_eq!((failed["co_equal"].clone(), failed["verdict"].clone()), (json!(false), json!("fail")), "{out}");
    assert!(failed["issues"].as_array().unwrap().iter().any(|i| i["code"] == "no_solid"), "{failed}");
    assert_eq!(comps[1]["verdict"], "pass", "the next part ran: {out}");
    assert_eq!(out["verdict"], "review", "{out}");
    assert!(out["issues"].as_array().unwrap().iter().any(|i| i["code"] == "parts_not_passed"), "{out}");
}

/// Exactly a tenth of the largest piece is co-equal; a hair less is not (and with no mesh behind it
/// it is left out as debris).
#[test]
fn the_co_equal_boundary_is_exactly_a_tenth_of_the_largest_area() {
    let at = |small: f64| Analysis {
        parts: vec![piece(1000.0), piece(small)],
        extra: Vec::new(),
        rest: Vec::new(),
        nests: vec![None, None],
    };
    let on = plan(at(100.0));
    assert_eq!((on.chosen.len(), on.left_out.len()), (2, 0), "exactly 0.1 x the largest is co-equal");
    let below = plan(at(99.99));
    assert_eq!((below.chosen.len(), below.left_out.len()), (1, 1), "just below is not");
}

/// A failed part takes back what its run added (a body, a mesh), a cancellation does not touch
/// the document, and a success keeps everything.
#[test]
fn a_failed_part_leaves_nothing_of_its_run_in_the_document() {
    // Arrange
    let mut s = Session::new();
    let keep = s.doc.add_mesh("part", balls(1));
    let add = |s: &mut Session| {
        s.doc.add_mesh("rebuilt", balls(1));
        Ok(())
    };

    // Act
    let failed: CmdResult<Result<(), CmdError>> = restoring_on_failure(&mut s, |s| {
        add(s)?;
        Err(CmdError::Failed("boom".into()))
    });
    let failed_meshes = s.doc.meshes.len();
    let cancelled_run: CmdResult<Result<(), CmdError>> = restoring_on_failure(&mut s, |s| {
        add(s)?;
        Err(cancelled())
    });
    let after_cancel = s.doc.meshes.len();
    let fine = restoring_on_failure(&mut s, add);

    // Assert
    assert!(matches!(failed, Ok(Err(_))), "a failure is returned as the inner error");
    assert_eq!(failed_meshes, 1, "the mesh the run added is gone");
    assert!(s.doc.mesh(keep).is_some());
    assert!(cancelled_run.is_err_and(|e| is_cancelled(&e)));
    assert_eq!(after_cancel, 2, "a cancellation leaves the clean-up to the caller");
    assert!(matches!(fine, Ok(Ok(()))));
    assert_eq!(s.doc.meshes.len(), 3);
}

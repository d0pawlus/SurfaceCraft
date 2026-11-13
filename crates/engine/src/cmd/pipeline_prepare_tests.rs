//! Tests of `pipeline_prepare.rs`.
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

#[test]
fn only_the_cap_leaves_co_equal_parts_beyond_the_limit_and_a_request_does_not() {
    let a = scan_split::analyse(&balls(26), MIN_SHARE, false);
    let capped = plan_of(&a, PART_SHARE, None);
    assert_eq!((capped.chosen.len(), capped.beyond_cap.len(), capped.not_requested.len()), (24, 2, 0));
    let asked = plan_of(&a, PART_SHARE, Some(1));
    assert_eq!((asked.chosen.len(), asked.beyond_cap.len(), asked.not_requested.len()), (1, 0, 25));
    let tally = Tally {
        minor: (0, 0.0),
        cavities: (0, 0.0),
        beyond_cap: (0, 0.0),
        not_requested: (25, 0.96),
        modelled: 1,
        debris_pieces: 0,
        ms: 0,
    };
    let (issues, capped_verdict) = collect_issues(&[], &tally);
    assert!(issues.is_empty() && !capped_verdict, "a requested subset is no issue");
    let over = Tally { beyond_cap: (2, 0.07), ..tally };
    let (issues, capped_verdict) = collect_issues(&[], &over);
    assert!(
        issues.iter().any(|i| i.code == "parts_beyond_limit" && i.message.contains("2 more parts")) && capped_verdict
    );
}

#[test]
fn a_piece_below_a_tenth_of_the_largest_is_minor_and_the_components_word_is_checked() {
    let mut m = balls(1);
    let mut small = synth::sphere(Point3::new(60.0, 0.0, 0.0), 1.2, 16, 8);
    small.weld(1e-9);
    m.append(&small);
    let a = scan_split::analyse(&m, MIN_SHARE, false);
    let p = plan_of(&a, PART_SHARE, None);
    assert_eq!((p.chosen.len(), p.minor.len()), (1, 1));
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
    let p = plan_of(&a, PART_SHARE, None);
    assert_eq!(p.chosen, vec![0], "the shell is modelled");
    assert_eq!(p.cavities, vec![1]);
    assert!(p.minor.is_empty());
    let none = Analysis { nests: vec![Some((1, true)), Some((0, true))], ..a };
    assert_eq!(plan_of(&none, PART_SHARE, None).chosen.len(), 1, "never an empty choice");
}

#[test]
fn pieces_beyond_the_listed_parts_stay_in_the_tally() {
    let a = scan_split::analyse(&balls(80), MIN_SHARE, false);
    assert_eq!((a.parts.len(), a.extra.len()), (scan_split::MAX_PARTS, 16));
    let plan = plan_of(&a, PART_SHARE, None);
    let tally = tally_of(&a, &plan, 0);
    assert_eq!(tally.beyond_cap.0, 56, "40 past the cap of 24, 16 past the listed parts");
    assert!((tally.beyond_cap.1 - 56.0 / 80.0).abs() < 1e-9, "{}", tally.beyond_cap.1);
    let (issues, capped) = collect_issues(&[], &tally);
    assert!(capped && issues.iter().any(|i| i.code == "parts_beyond_limit" && i.message.contains("56 more parts")));
}

#[test]
fn two_percent_of_unmodelled_surface_is_the_boundary_of_the_review_cap() {
    let tally = |share: f64| Tally {
        minor: (3, share),
        cavities: (0, 0.0),
        beyond_cap: (0, 0.0),
        not_requested: (0, 0.0),
        modelled: 1,
        debris_pieces: 3,
        ms: 0,
    };
    let (issues, capped) = collect_issues(&[], &tally(UNMODELLED_REVIEW_SHARE));
    assert!(issues.is_empty() && !capped, "exactly 2 % is not above 2 %");
    let (issues, capped) = collect_issues(&[], &tally(UNMODELLED_REVIEW_SHARE + 1e-4));
    assert!(capped && issues[0].code == "unmodelled_pieces" && issues[0].message.contains("part_share"));
    let (issues, capped) = collect_issues(&[], &tally(UNMODELLED_REVIEW_SHARE - 1e-4));
    assert!(issues.is_empty() && !capped);
}

#[test]
fn only_a_primitives_failure_without_a_body_is_wrapped_as_freeform() {
    let set = Settings {
        scan: ObjectId(0),
        tolerance: 0.2,
        min_within: 0.99,
        min_triangles: None,
        max_patches: None,
        allow_retry: true,
        budget: super::super::budget::CallBudget::default(),
        rebuild: true,
        name: "solid".into(),
    };
    let mut out = failure(&set, ObjectId(1), "x");
    assert!(!needs_freeform(Method::Auto, &out), "a freeform failure is not repeated");
    out.method = Method::Primitives;
    assert!(needs_freeform(Method::Auto, &out));
    assert!(!needs_freeform(Method::Freeform, &out));
    assert!(!needs_freeform(Method::Primitives, &out));
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
}

/// Exactly a tenth of the largest piece is co-equal; a hair less is a minor piece.
#[test]
fn the_co_equal_boundary_is_exactly_a_tenth_of_the_largest_area() {
    let at = |small: f64| Analysis {
        parts: vec![piece(1000.0), piece(small)],
        extra: Vec::new(),
        rest: Vec::new(),
        nests: vec![None, None],
    };
    let on = plan_of(&at(100.0), PART_SHARE, None);
    assert_eq!((on.chosen.len(), on.minor.len()), (2, 0), "exactly 0.1 x the largest is co-equal");
    let below = plan_of(&at(99.99), PART_SHARE, None);
    assert_eq!((below.chosen.len(), below.minor.len()), (1, 1), "just below is minor");
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

//! P9-M3: `align.global` on hostile input: bad lists, bad meshes, a huge list, cancellation.
//! (`hostile_params_never_panic` in `commands.rs` covers the command with the shared hostile values;
//! these are the cases that need meshes of the right kind.)
use serde_json::{Value, json};
use std::sync::Arc;
use std::time::{Duration, Instant};
use surfacecraft_engine::{EngineError, Job, JobPoll, Session, cmd};
use surfacecraft_geom::{Mesh, Point3, Vec3, synth};

fn cube(name: &str, s: &mut Session) {
    s.doc.add_mesh(name, synth::cuboid(Point3::origin(), Vec3::new(10.0, 20.0, 30.0), 4));
}

fn session() -> Session {
    let mut s = Session::new();
    for n in ["a", "b", "c"] {
        cube(n, &mut s);
    }
    s
}

fn error_of(s: &mut Session, params: Value) -> String {
    match s.execute("align.global", params) {
        Err(e) => e.to_string(),
        Ok(v) => panic!("expected an error, got {v}"),
    }
}

#[test]
fn a_list_that_is_too_short_or_too_long_is_an_error_with_a_clear_message() {
    let mut s = session();
    assert!(error_of(&mut s, json!({"objects": []})).contains("between 2 and 24"));
    assert!(error_of(&mut s, json!({"objects": ["a"]})).contains("between 2 and 24"));
    let many: Vec<String> = (0..25).map(|k| format!("m{k}")).collect();
    assert!(
        error_of(&mut s, json!({"objects": many})).contains("at most 24"),
        "the cap is checked before anything is resolved"
    );
}

#[test]
fn a_mesh_listed_twice_is_an_error_by_name_and_by_id() {
    let mut s = session();
    assert!(error_of(&mut s, json!({"objects": ["a", "b", "a"]})).contains("listed twice"));
    let id = s.doc.add_mesh("d", synth::cuboid(Point3::origin(), Vec3::new(10.0, 20.0, 30.0), 4));
    assert!(error_of(&mut s, json!({"objects": ["d", "b", id]})).contains("listed twice"));
}

#[test]
fn bad_meshes_and_bad_options_are_errors_naming_the_cause() {
    let mut s = session();
    s.doc.add_mesh("empty", Mesh::default());
    assert!(error_of(&mut s, json!({"objects": ["a", "empty"]})).contains("scan 1"));
    let mut nan = synth::cuboid(Point3::origin(), Vec3::new(10.0, 20.0, 30.0), 4);
    nan.positions[3] = Point3::new(f64::NAN, 0.0, 0.0);
    s.doc.add_mesh("nan", nan);
    assert!(error_of(&mut s, json!({"objects": ["nan", "b"]})).contains("scan 0"));
    assert!(error_of(&mut s, json!({"objects": ["a", "b"], "fixed": "c"})).contains("not in `objects`"));
    assert!(error_of(&mut s, json!({"objects": ["a", "b"], "max_pairs": 0})).contains("max_pairs"));
    assert!(error_of(&mut s, json!({"objects": ["a", "b"], "max_pairs": 100000})).contains("max_pairs"));
    for tolerance in [json!(0.0), json!(-1.0), json!("NaN")] {
        let params = json!({"objects": ["a", "b"], "tolerance": tolerance});
        assert!(s.execute("align.global", params).is_err());
    }
    assert!(s.execute("align.global", json!({"objects": ["a", "b"], "start": "teleport"})).is_err());
    assert!(s.execute("align.global", json!({"objects": "a"})).is_err());
}

/// A proposal records no history entry and moves nothing; a refused `apply` (nothing connected)
/// moves nothing either.
#[test]
fn a_proposal_and_a_refused_apply_leave_the_document_alone() {
    let mut s = Session::new();
    let boxes: Vec<_> = (0..2u64)
        .map(|k| {
            let m = synth::cuboid(Point3::origin(), Vec3::new(40.0, 30.0, 20.0), 8 + 3 * k as usize);
            (format!("box{k}"), m)
        })
        .collect();
    let ids: Vec<_> = boxes.iter().map(|(n, m)| s.doc.add_mesh(n, m.clone())).collect();
    let before: Vec<_> = ids.iter().map(|id| s.doc.mesh(*id).unwrap().mesh.positions.clone()).collect();
    let undo = s.history.undo_labels().len();
    for apply in [false, true] {
        let out = s.execute("align.global", json!({"objects": ["box0", "box1"], "apply": apply})).unwrap();
        assert_ne!(out["verdict"], "aligned", "two copies of a box are ambiguous: {out}");
        assert_eq!(out["applied"], false);
    }
    let after: Vec<_> = ids.iter().map(|id| s.doc.mesh(*id).unwrap().mesh.positions.clone()).collect();
    assert_eq!(before, after);
    assert_eq!(s.history.undo_labels().len(), undo, "no undo step");
}

/// Cancelling a running `align.global` ends it with the cancelled error and leaves no trace.
#[test]
fn a_cancelled_global_alignment_stops_early_and_changes_nothing() {
    let mut s = Session::new();
    for k in 0..6u64 {
        let m = synth::sphere(Point3::new(0.0, 0.0, 0.0), 20.0 + k as f64, 120, 60);
        s.doc.add_mesh(&format!("s{k}"), m);
    }
    let names: Vec<String> = (0..6).map(|k| format!("s{k}")).collect();
    let job = Job::start(&s, "align.global", Arc::new(json!({"objects": names, "apply": true}))).unwrap();
    std::thread::sleep(Duration::from_millis(100));
    job.cancel();
    let started = Instant::now();
    let done = loop {
        match job.poll() {
            JobPoll::Finished(d) => break *d,
            JobPoll::Lost => panic!("the job was lost"),
            JobPoll::Running => {
                assert!(started.elapsed() < Duration::from_secs(120), "cancelling took too long");
                std::thread::sleep(Duration::from_millis(5));
            }
        }
    };
    assert_eq!(done.result, Err(EngineError::Failed(cmd::CANCELLED.into())));
    assert_eq!(done.session.history.undo_labels().len(), s.history.undo_labels().len());
}

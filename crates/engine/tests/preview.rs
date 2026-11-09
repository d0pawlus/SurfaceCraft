//! `Session::preview` runs a command on a copy: the session, its history and its log stay as
//! they were; read-only and heavy commands are refused.
use serde_json::json;
use surfacecraft_engine::{EngineError, Session, cmd};

#[test]
fn preview_leaves_the_session_untouched() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "sphere", "size": [10.0], "resolution": 12})).unwrap();
    let (summaries, undo, log) = (s.doc.summaries(), s.history.undo_labels().len(), s.log.len());
    let copy = s.preview("body.box", json!({"size": [20.0, 10.0, 5.0]})).unwrap();
    assert_eq!(copy.doc.bodies.len(), 1, "the copy has the box");
    assert!(!copy.allow_files);
    assert_eq!(s.doc.summaries(), summaries);
    assert_eq!(s.history.undo_labels().len(), undo);
    assert_eq!(s.log.len(), log);
    assert!(s.doc.bodies.is_empty());
}

#[test]
fn preview_reports_the_commands_error_and_refuses_read_only_and_heavy_commands() {
    let s = Session::new();
    assert!(matches!(s.preview("body.box", json!({"size": "x"})), Err(EngineError::InvalidParams(_))));
    assert!(matches!(s.preview("mesh.info", json!({"object": 0})), Err(EngineError::Unavailable(_))));
    assert!(matches!(s.preview("region.auto_segment", json!({"mesh": 0})), Err(EngineError::Unavailable(_))));
    assert!(matches!(s.preview("no.such", json!({})), Err(EngineError::UnknownCommand(_))));
    assert!(cmd::is_heavy("mesh.reconstruct") && !cmd::is_heavy("body.box"));
}

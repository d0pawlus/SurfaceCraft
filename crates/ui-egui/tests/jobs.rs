#![cfg(not(target_arch = "wasm32"))]
//! Heavy commands launched from the UI run in the background: the app stays usable for drawing,
//! refuses other commands meanwhile, and ends in the state a run in place ends in.
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::json;
use std::time::{Duration, Instant};
use surfacecraft_engine::EngineError;
use surfacecraft_ui_egui::SurfaceCraftApp;

/// Longest a test waits for a background command.
const PATIENCE: Duration = Duration::from_secs(300);

fn app_with(shape: serde_json::Value, background: bool) -> SurfaceCraftApp {
    let mut app = SurfaceCraftApp::headless();
    app.run_command("mesh.synth", shape).unwrap();
    app.background_jobs = background;
    app
}

fn bracket(background: bool) -> SurfaceCraftApp {
    app_with(json!({"shape": "bracket", "name": "scan"}), background)
}

/// A noisy sphere of about 90 thousand triangles: its segmentation takes seconds.
fn slow_scan() -> SurfaceCraftApp {
    app_with(json!({"shape": "sphere", "resolution": 300, "noise": 0.05, "name": "scan"}), true)
}

fn open_auto_segment(app: &mut SurfaceCraftApp, mesh: &str) {
    app.choose_command("region.auto_segment");
    app.palette_params = json!({"mesh": mesh}).to_string();
}

/// Polls like the UI does every frame until the command is done.
fn wait(app: &mut SurfaceCraftApp) {
    let started = Instant::now();
    while app.advance_job(started.elapsed().as_secs_f64()) {
        assert!(started.elapsed() < PATIENCE, "the command did not finish");
        std::thread::sleep(Duration::from_millis(2));
    }
}

#[test]
fn a_background_run_ends_like_a_run_in_place() {
    let mut in_place = bracket(false);
    open_auto_segment(&mut in_place, "scan");
    assert!(in_place.run_palette());
    assert!(!in_place.is_busy(), "a headless app runs every command before the call returns");

    let mut app = bracket(true);
    open_auto_segment(&mut app, "scan");
    assert!(app.run_palette());
    assert!(app.is_busy());
    assert!(app.session.doc.regions.is_empty(), "the app shows the document from before the command");
    assert_eq!(app.status, "Auto segment: running…");
    wait(&mut app);

    assert_eq!(app.session.log, in_place.session.log);
    assert_eq!(app.session.history.undo_labels(), in_place.session.history.undo_labels());
    assert_eq!(app.session.doc.summaries(), in_place.session.doc.summaries());
    assert_eq!(app.status, in_place.status);
    assert_eq!(app.palette_command, in_place.palette_command, "the tool closes as after a run in place");
    assert!(!app.session.doc.regions.is_empty());
}

#[test]
fn other_commands_are_refused_while_one_runs() {
    let mut app = slow_scan();
    open_auto_segment(&mut app, "scan");
    assert!(app.run_palette());
    let log = app.session.log.len();

    let refused = app.run_command("mesh.synth", json!({"shape": "box"}));
    assert!(matches!(refused, Err(EngineError::Unavailable(_))), "{refused:?}");
    assert!(!app.run_palette(), "a second launch is refused too");
    assert_eq!(app.status, "Auto segment is still running");
    assert_eq!(app.session.log.len(), log, "a refused command is not run and not logged");
    assert_eq!(app.session.doc.meshes.len(), 1);
    assert!(app.is_busy());
    app.cancel_job();
}

#[test]
fn cancel_gives_control_back_with_the_document_from_before() {
    let mut app = slow_scan();
    let before = app.session.doc.summaries();
    open_auto_segment(&mut app, "scan");
    assert!(app.run_palette());
    let started = Instant::now();
    app.cancel_job();

    assert!(started.elapsed() < Duration::from_secs(1), "cancel does not wait for the command");
    assert!(!app.is_busy());
    assert_eq!(app.session.doc.summaries(), before);
    assert_eq!(app.session.history.undo_labels(), ["mesh.synth"], "no undo step for the command");
    assert_eq!(app.status, "Auto segment: cancelled");
    assert_eq!(app.palette_command, Some("region.auto_segment"), "the tool stays open to try again");
    let last = app.session.log.last().unwrap();
    assert_eq!((last.command.as_str(), last.ok), ("region.auto_segment", false));
    assert_eq!(last.outcome, json!({"kind": "failed", "message": "cancelled"}));
    // the app works on: the next command runs
    assert!(app.run_command("mesh.synth", json!({"shape": "box"})).is_ok());
}

#[test]
fn an_error_in_a_background_command_is_reported_and_the_session_survives() {
    let mut in_place = bracket(false);
    open_auto_segment(&mut in_place, "no such scan");
    assert!(!in_place.run_palette());

    let mut app = bracket(true);
    let before = app.session.doc.summaries();
    open_auto_segment(&mut app, "no such scan");
    assert!(app.run_palette(), "the command was started");
    wait(&mut app);

    assert_eq!(app.status, in_place.status);
    assert!(app.status.starts_with("Auto segment: "), "{}", app.status);
    assert_eq!(app.session.log, in_place.session.log);
    assert_eq!(app.session.doc.summaries(), before);
    assert_eq!(app.palette_command, Some("region.auto_segment"), "a failed tool stays open");
    assert!(app.run_command("mesh.synth", json!({"shape": "box"})).is_ok());
}

#[test]
fn quick_commands_run_in_place_even_with_background_jobs_on() {
    let mut app = bracket(true);
    app.choose_command("mesh.synth");
    app.palette_params = json!({"shape": "box"}).to_string();
    assert!(app.run_palette());
    assert!(!app.is_busy());
    assert_eq!(app.session.doc.meshes.len(), 2);
}

#[test]
fn the_window_says_what_runs_and_cancel_stops_it() {
    let mut app = slow_scan();
    open_auto_segment(&mut app, "scan");
    let mut h = Harness::builder()
        .with_size((1600.0, 1000.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app);
    h.run_steps(2);
    assert!(h.state_mut().run_palette());
    h.run_steps(2);
    h.get_by_label("Running: Auto segment");
    h.get_by_label("Cancel command").click();
    h.run_steps(2);
    assert!(!h.state().is_busy());
    assert_eq!(h.state().status, "Auto segment: cancelled");
    assert!(h.query_by_label("Running: Auto segment").is_none());
}

/// Tries a heavy command until the app takes it (a cancelled one may still be finishing).
fn launch_when_free(app: &mut SurfaceCraftApp) {
    let started = Instant::now();
    while !app.run_palette() {
        assert!(started.elapsed() < PATIENCE, "the cancelled command never finished: {}", app.status);
        std::thread::sleep(Duration::from_millis(5));
    }
}

#[test]
fn a_new_heavy_command_waits_for_a_cancelled_one_that_cannot_stop() {
    let mut app = slow_scan();
    // decimation does not look at the cancellation flag: it runs to its end after Cancel
    app.choose_command("mesh.decimate");
    app.palette_params = json!({"object": "scan", "target": 500}).to_string();
    assert!(app.run_palette());
    app.cancel_job();
    assert!(!app.is_busy());

    open_auto_segment(&mut app, "scan");
    assert!(!app.run_palette(), "a heavy command is refused while the cancelled one still runs");
    assert_eq!(app.status, "The cancelled command is still finishing; try again in a moment");
    assert!(!app.is_busy());
    // quick commands are not held up
    assert!(app.run_command("mesh.synth", json!({"shape": "box"})).is_ok());
    // and once the cancelled command has ended, heavy ones start again
    launch_when_free(&mut app);
    assert!(app.is_busy());
    app.cancel_job();
    assert_eq!(app.session.doc.meshes.len(), 2, "the cancelled decimation changed nothing");
}

#[test]
fn the_assistant_is_refused_while_a_command_runs() {
    let mut app = slow_scan();
    app.set_provider(Box::new(surfacecraft_ai::ScriptedProvider::default()));
    open_auto_segment(&mut app, "scan");
    assert!(app.run_palette());
    let log = app.session.log.len();

    app.ask_assistant();

    assert_eq!(app.status, "Auto segment is still running");
    assert_eq!(app.session.log.len(), log);
    assert!(app.is_busy());
    app.cancel_job();
}

#[test]
fn a_heavy_command_from_the_control_channel_runs_in_the_background_and_answers() {
    use surfacecraft_ui_egui::control::Request;
    let mut in_place = bracket(false);
    let expected = in_place.run_command("region.auto_segment", json!({"mesh": "scan"})).unwrap();

    let mut app = bracket(true);
    let (requests, inbox) = std::sync::mpsc::channel();
    app.attach_control(inbox);
    let ask = |command: &str, params: serde_json::Value| {
        let (reply, answer) = std::sync::mpsc::channel();
        let params = json!({"command": command, "params": params});
        requests.send(Request { method: "engine.execute".into(), params, reply }).unwrap();
        answer
    };
    let segmented = ask("region.auto_segment", json!({"mesh": "scan"}));
    let listed = ask("region.list", json!({}));

    app.poll_control();
    assert!(app.is_busy(), "the heavy command runs in the background");
    assert!(segmented.try_recv().is_err(), "no answer before it has finished");
    assert!(listed.try_recv().is_err(), "the request behind it waits in the channel");
    wait(&mut app);
    app.poll_control();

    assert_eq!(segmented.try_recv().unwrap(), json!({"ok": expected}));
    let regions = listed.try_recv().unwrap();
    assert_eq!(regions["ok"]["regions"].as_array().map(Vec::len), Some(app.session.doc.regions.len()));
    assert_eq!(app.session.log, in_place_log_with_list(&mut in_place));
}

/// The log of `app` after it has also listed the regions.
fn in_place_log_with_list(app: &mut SurfaceCraftApp) -> Vec<std::sync::Arc<surfacecraft_engine::LogEntry>> {
    app.run_command("region.list", json!({})).unwrap();
    app.session.log.clone()
}

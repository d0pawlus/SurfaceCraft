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

// ---------------------------------------------------------- the camera on the control channel

use serde_json::Value;
use std::sync::mpsc::{Sender, channel};
use surfacecraft_ui_egui::camera::{Camera, POLE_PITCH, StandardView};
use surfacecraft_ui_egui::control::Request;

/// A headless app with a scan, and the way to send it control requests like the desktop app gets.
struct Remote {
    app: SurfaceCraftApp,
    requests: Sender<Request>,
}

impl Remote {
    fn new() -> Self {
        let mut app = bracket(false);
        let (requests, inbox) = channel();
        app.attach_control(inbox);
        Self { app, requests }
    }

    /// Sends one request and returns the reply.
    fn ask(&mut self, method: &str, params: Value) -> Value {
        let (reply, answer) = channel();
        self.requests.send(Request { method: method.into(), params, reply }).unwrap();
        self.app.poll_control();
        answer.try_recv().expect("the app answers a camera request at once")
    }

    /// `ui.camera_get`, the camera of the reply.
    fn camera(&mut self) -> Value {
        let reply = self.ask("ui.camera_get", Value::Null);
        assert!(reply.get("err").is_none(), "{reply}");
        reply["ok"].clone()
    }

    /// `ui.camera_set`, the whole reply.
    fn set(&mut self, params: Value) -> Value {
        self.ask("ui.camera_set", params)
    }

    /// `ui.camera_set` that must succeed: the camera of the reply.
    fn set_ok(&mut self, params: Value) -> Value {
        let reply = self.set(params.clone());
        assert!(reply.get("err").is_none(), "{params} was refused: {reply}");
        reply["ok"].clone()
    }
}

/// The numbers of a camera object: target, distance, yaw, pitch, field of view.
fn numbers(camera: &Value) -> Vec<f64> {
    let mut out: Vec<f64> = camera["target"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
    out.extend(["distance", "yaw_rad", "pitch_rad", "fov_rad"].map(|k| camera[k].as_f64().unwrap()));
    out
}

/// Two camera objects hold the same camera, within a relative 1e-9.
fn assert_same(a: &Value, b: &Value) {
    let (x, y) = (numbers(a), numbers(b));
    assert_eq!(x.len(), y.len());
    for (p, q) in x.iter().zip(&y) {
        assert!((p - q).abs() <= 1e-9 * p.abs().max(q.abs()).max(1.0), "{a} is not {b}");
    }
    assert_eq!(a["projection"], b["projection"]);
}

/// A camera as `ui.camera_get` shows it.
fn as_json(c: &Camera) -> Value {
    json!({
        "projection": "perspective",
        "target": [c.target.x, c.target.y, c.target.z],
        "distance": c.distance,
        "yaw_rad": c.yaw,
        "pitch_rad": c.pitch,
        "fov_rad": c.fov,
    })
}

/// An app whose camera is not the default.
fn moved_camera() -> Remote {
    let mut r = Remote::new();
    r.app.camera.orbit(70.0, 25.0);
    r.app.camera.pan(30.0, -12.0, 600.0);
    r.app.camera.zoom(180.0);
    r
}

#[test]
fn camera_get_then_set_with_the_returned_value_gives_the_same_camera() {
    let mut r = moved_camera();
    let first = r.camera();
    assert_eq!(first["projection"], "perspective");
    let echoed = r.set_ok(first.clone());
    assert_same(&echoed, &first);
    assert_same(&r.camera(), &first);
    assert_same(&as_json(&r.app.camera), &first);
}

#[test]
fn camera_set_puts_a_moved_camera_back() {
    let mut r = moved_camera();
    let saved = r.camera();
    let before = r.app.camera.clone();
    r.app.camera.orbit(-200.0, 60.0);
    r.app.camera.pan(80.0, 40.0, 600.0);
    r.app.camera.zoom(-300.0);
    assert!(r.app.camera != before);
    let moved = r.camera();
    let gap = (moved["distance"].as_f64().unwrap() - saved["distance"].as_f64().unwrap()).abs();
    assert!(gap > 1.0, "the camera moved by {gap}");

    r.set_ok(saved.clone());
    assert_same(&r.camera(), &saved);
    assert!((r.app.camera.target - before.target).norm() < 1e-9);
    assert!((r.app.camera.yaw - before.yaw).abs() < 1e-9 && (r.app.camera.fov - before.fov).abs() < 1e-9);
}

#[test]
fn a_partial_camera_set_changes_only_the_given_field() {
    let mut r = moved_camera();
    let fields = [
        ("target", json!([1.5, -2.5, 3.5])),
        ("distance", json!(321.5)),
        ("yaw_rad", json!(1.25)),
        ("pitch_rad", json!(-0.5)),
        ("fov_rad", json!(0.9)),
    ];
    for (key, value) in fields {
        let before = r.camera();
        let given: serde_json::Map<String, Value> = std::iter::once((key.to_string(), value.clone())).collect();
        let after = r.set_ok(Value::Object(given));
        assert_eq!(after[key], value, "{key} is written");
        for other in ["target", "distance", "yaw_rad", "pitch_rad", "fov_rad"].into_iter().filter(|k| *k != key) {
            assert_eq!(after[other], before[other], "{other} stays when only {key} is given");
        }
    }
    // nothing given, or only nulls: nothing changes
    let before = r.camera();
    for none in [json!({}), Value::Null, json!({"distance": null, "view": null, "fit": null})] {
        assert_same(&r.set_ok(none), &before);
    }
}

#[test]
fn named_views_set_the_orientation_and_keep_the_rest() {
    let mut r = moved_camera();
    let start = r.camera();
    let views = [
        ("front", StandardView::Front),
        ("back", StandardView::Back),
        ("left", StandardView::Left),
        ("right", StandardView::Right),
        ("top", StandardView::Top),
        ("bottom", StandardView::Bottom),
        ("iso", StandardView::Iso),
    ];
    for (name, view) in views {
        let c = r.set_ok(json!({"view": name}));
        let (yaw, pitch) = view.angles();
        assert!((c["yaw_rad"].as_f64().unwrap() - yaw).abs() < 1e-9, "{name}: {c}");
        assert!((c["pitch_rad"].as_f64().unwrap() - pitch).abs() < 1e-9, "{name}: {c}");
        for kept in ["target", "distance", "fov_rad"] {
            assert_eq!(c[kept], start[kept], "{name} keeps {kept}");
        }
        assert!(numbers(&c).iter().all(|x| x.is_finite()));
        // the view, read back and written again, is the same view (the poles included)
        r.set_ok(c.clone());
        assert_same(&r.camera(), &c);
    }
    // the name is not case sensitive, and an explicit field after the view wins over it
    let c = r.set_ok(json!({"view": " Top ", "yaw_rad": 0.25}));
    assert!((c["yaw_rad"].as_f64().unwrap() - 0.25).abs() < 1e-12);
    assert!((c["pitch_rad"].as_f64().unwrap() - POLE_PITCH).abs() < 1e-12);
    // a pitch a hair past the pole is the pole
    let c = r.set_ok(json!({"pitch_rad": POLE_PITCH + 1e-7}));
    assert!((c["pitch_rad"].as_f64().unwrap() - POLE_PITCH).abs() < 1e-12);
}

#[test]
fn fit_frames_the_document_after_the_view_and_the_explicit_fields() {
    let mut r = Remote::new();
    r.set_ok(json!({"target": [5e4, -3e4, 1e4], "distance": 9e5}));
    let far = r.app.camera.clone();
    let bbox = r.app.session.doc.bbox();

    let c = r.set_ok(json!({"fit": true}));
    let mut want = far.clone();
    want.fit(&bbox);
    assert_same(&c, &as_json(&want));
    assert!(numbers(&c).iter().all(|x| x.is_finite()));
    assert!(c["distance"].as_f64().unwrap() < 9e5, "fitted to the scan, not to where the camera was");

    // view first, then the explicit fields, then the fit: it replaces target and distance and
    // uses the field of view given in the same request
    let (yaw, pitch) = StandardView::Front.angles();
    let c = r.set_ok(json!({"fit": true, "fov_rad": 1.0, "distance": 5.0, "view": "front", "target": [1.0, 2.0, 3.0]}));
    let mut want = Camera { yaw, pitch, fov: 1.0, ..far };
    want.fit(&bbox);
    assert_same(&c, &as_json(&want));
}

#[test]
fn the_camera_requests_answer_with_what_the_next_frame_draws() {
    let mut r = Remote::new();
    // a running view animation jumps to its end
    r.app.set_standard_view(StandardView::Left);
    let (yaw, pitch) = StandardView::Left.angles();
    // the animation takes the short way round, so the angle may be a turn away: compare directions
    let c = r.camera();
    let same_angle = |a: f64, b: f64| (a.sin() - b.sin()).abs() < 1e-9 && (a.cos() - b.cos()).abs() < 1e-9;
    assert!(same_angle(c["yaw_rad"].as_f64().unwrap(), yaw), "{c}");
    assert!((c["pitch_rad"].as_f64().unwrap() - pitch).abs() < 1e-9, "{c}");
    assert!(same_angle(r.app.camera.yaw, yaw), "the app's camera is the one answered");
}

#[test]
fn hostile_camera_parameters_are_refused_and_change_nothing() {
    let mut r = moved_camera();
    let before = r.app.camera.clone();
    let long_name = "x".repeat(100_000);
    let cases = [
        json!({"distance": -1.0}),
        json!({"distance": 0}),
        json!({"distance": 1e300}),
        json!({"distance": 1e-12}),
        json!({"distance": "far"}),
        json!({"distance": [1]}),
        json!({"target": [0, 0]}),
        json!({"target": [0, 0, 0, 0]}),
        json!({"target": [1e300, 0, 0]}),
        json!({"target": [0, "a", 0]}),
        json!({"target": "origin"}),
        json!({"target": {"x": 1}}),
        json!({"yaw_rad": "1"}),
        json!({"yaw_rad": 1e300}),
        json!({"pitch_rad": 2.0}),
        json!({"pitch_rad": -1.6}),
        json!({"pitch_rad": 1e300}),
        json!({"fov_rad": 0}),
        json!({"fov_rad": -1}),
        json!({"fov_rad": 3.2}),
        json!({"fov_rad": 1e300}),
        json!({"view": "sideways"}),
        json!({"view": 5}),
        json!({"view": ""}),
        json!({"view": long_name}),
        json!({"fit": "yes"}),
        json!({"fit": 1}),
        json!({"projection": "orthographic"}),
        json!({"projection": "fisheye"}),
        json!({"projection": 1}),
        json!({"nope": 1}),
        json!({"distance": 100.0, "extra": true}),
        // a refused request applies none of its fields
        json!({"view": "top", "distance": -1.0}),
        json!({"target": [1, 2, 3], "fov_rad": 0}),
        json!([1, 2]),
        json!("front"),
        json!(5),
        json!(true),
    ];
    for params in cases {
        let reply = r.set(params.clone());
        let shown = reply.to_string();
        let message = reply["err"].as_str().unwrap_or_else(|| panic!("{params:.80} was accepted: {shown:.200}"));
        assert!(message.starts_with("ui.camera_set:"), "{message}");
        assert!(message.chars().count() <= 400, "a hostile value is not echoed in full: {message}");
        assert!(reply.get("ok").is_none());
        assert_eq!(r.app.camera, before, "{params:.80} changed the camera");
    }
    for params in [json!({"x": 1}), json!([1]), json!(5), json!("a")] {
        let reply = r.ask("ui.camera_get", params.clone());
        assert!(reply["err"].as_str().is_some_and(|m| m.starts_with("ui.camera_get")), "{params}: {reply}");
        assert_eq!(r.app.camera, before);
    }
}

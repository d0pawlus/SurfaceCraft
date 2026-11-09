//! Picking form fields in the viewport (no GPU): the 3-2-1 datums of `align.datum` from clicks on
//! the scan, a list of points, and the `edges` of a fillet from clicks on the body. The tests click
//! the way a user does (the "Pick in viewport" button, then pointer events over the viewport) and
//! read the parameters back.
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::{Value, json};
use surfacecraft_geom::Point3;
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::camera::StandardView;
use surfacecraft_ui_egui::fieldpick::{PickKind, datum_value};
use surfacecraft_ui_egui::i18n::{self, Lang};

fn harness(app: SurfaceCraftApp) -> Harness<'static, SurfaceCraftApp> {
    let h = Harness::builder()
        .with_size((1600.0, 1000.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app);
    // the harness steps are slow, but two clicks on one spot must stay two clicks
    h.ctx.options_mut(|o| o.input_options.max_double_click_delay = 0.0);
    h
}

fn settle(h: &mut Harness<'static, SurfaceCraftApp>) {
    for _ in 0..3 {
        h.run();
    }
}

fn params(h: &Harness<'static, SurfaceCraftApp>) -> Value {
    serde_json::from_str(&h.state().palette_params).expect("the parameters are JSON")
}

/// Where model point `p` is on the screen.
fn screen(h: &Harness<'static, SurfaceCraftApp>, p: Point3) -> egui::Pos2 {
    let app = h.state();
    let rect = app.viewport_rect.expect("the viewport was drawn");
    let s = app.camera.project(&p, f64::from(rect.width()), f64::from(rect.height())).expect("on screen");
    rect.min + egui::vec2(s[0] as f32, s[1] as f32)
}

fn click_at(h: &mut Harness<'static, SurfaceCraftApp>, pos: egui::Pos2) {
    h.hover_at(pos);
    h.run();
    h.drag_at(pos);
    h.drop_at(pos);
    h.run();
}

fn right_click_at(h: &mut Harness<'static, SurfaceCraftApp>, pos: egui::Pos2) {
    h.hover_at(pos);
    h.run();
    for pressed in [true, false] {
        h.event(egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Secondary,
            pressed,
            modifiers: egui::Modifiers::NONE,
        });
    }
    h.run();
}

fn click_model(h: &mut Harness<'static, SurfaceCraftApp>, p: Point3) {
    let pos = screen(h, p);
    click_at(h, pos);
}

fn right_click_model(h: &mut Harness<'static, SurfaceCraftApp>, p: Point3) {
    let pos = screen(h, p);
    right_click_at(h, pos);
}

fn pick_button(h: &mut Harness<'static, SurfaceCraftApp>, n: usize) {
    let label = i18n::lookup(Lang::En, "form.pick");
    h.get_all_by_label(label).nth(n).expect("a pick button").click();
    settle(h);
}

/// A scan seen from the front with its front face on screen; `at(fx, fz)` is the point of that
/// face at the fractions `fx` along X and `fz` along Z.
fn scan_harness() -> (Harness<'static, SurfaceCraftApp>, impl Fn(f64, f64) -> Point3) {
    let mut app = SurfaceCraftApp::headless();
    app.run_command("mesh.synth", json!({"shape": "box", "size": [40, 30, 20], "resolution": 12})).expect("a scan");
    app.camera.fit(&app.session.doc.bbox());
    app.camera.set_standard(StandardView::Front);
    let b = app.session.doc.bbox();
    let at = move |fx: f64, fz: f64| {
        Point3::new(b.min.x + fx * (b.max.x - b.min.x), b.min.y, b.min.z + fz * (b.max.z - b.min.z))
    };
    let mut h = harness(app);
    settle(&mut h);
    (h, at)
}

fn vec_of(v: &Value) -> [f64; 3] {
    let a = v.as_array().expect("a vector");
    [a[0].as_f64().expect("x"), a[1].as_f64().expect("y"), a[2].as_f64().expect("z")]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn close(a: [f64; 3], b: [f64; 3]) -> bool {
    sub(a, b).iter().all(|d| d.abs() < 1e-9)
}

/// Clicking the scan fills the primary plane (3 points), then the secondary axis (2) and the
/// tertiary point (1) by itself; the fields then hold exactly what the clicks hit, and the
/// alignment runs.
#[test]
fn clicks_on_the_scan_fill_the_321_datums_in_order() {
    let (mut h, at) = scan_harness();
    h.state_mut().choose_command("align.datum");
    settle(&mut h);
    assert!(
        params(&h)["primary"]["plane"].is_object(),
        "the datums start as the world planes: {}",
        h.state().palette_params
    );

    pick_button(&mut h, 0);
    let fp = h.state().field_pick.as_ref().expect("picking started");
    assert_eq!((fp.field(), fp.kind()), (Some("primary"), Some(PickKind::Datum { need: 3 })));
    assert!(h.state().field_pick_hint().is_some_and(|(_, hint)| hint.contains("point 1 of 3")));

    // the primary plane: three points that span a plane
    let before = params(&h)["primary"].clone();
    click_model(&mut h, at(0.3, 0.3));
    click_model(&mut h, at(0.7, 0.3));
    assert_eq!(params(&h)["primary"], before, "two points are not a plane yet");
    assert_eq!(h.state().field_pick.as_ref().map(|f| f.marks.len()), Some(2));
    assert!(h.state().field_pick_hint().is_some_and(|(_, hint)| hint.contains("point 3 of 3")));
    click_model(&mut h, at(0.5, 0.7));
    let marks = h.state().field_pick.as_ref().expect("still picking").marks.clone();
    assert_eq!(marks.len(), 3);
    let primary = params(&h)["primary"].clone();
    let plane = &primary["plane"];
    assert!(close(vec_of(&plane["point"]), marks[0]), "{primary}");
    let normal = cross(sub(marks[1], marks[0]), sub(marks[2], marks[0]));
    assert!(close(vec_of(&plane["normal"]), normal), "{primary}");
    assert!(normal[1].abs() > 1e-6, "the front face has a normal along Y: {normal:?}");

    // it moved on to the secondary axis
    let fp = h.state().field_pick.as_ref().expect("still picking");
    assert_eq!((fp.field(), fp.kind()), (Some("secondary"), Some(PickKind::Datum { need: 2 })));
    click_model(&mut h, at(0.2, 0.5));
    click_model(&mut h, at(0.8, 0.5));
    let marks = h.state().field_pick.as_ref().expect("still picking").marks.clone();
    assert_eq!(marks.len(), 5);
    let secondary = params(&h)["secondary"].clone();
    assert!(close(vec_of(&secondary["axis"]["point"]), marks[3]), "{secondary}");
    assert!(close(vec_of(&secondary["axis"]["direction"]), sub(marks[4], marks[3])), "{secondary}");
    let fp = h.state().field_pick.as_ref().expect("still picking");
    assert_eq!((fp.field(), fp.kind()), (Some("tertiary"), Some(PickKind::Datum { need: 1 })));

    // the tertiary point ends the pick; it lies on the scan
    click_model(&mut h, at(0.5, 0.4));
    assert!(h.state().field_pick.is_none(), "the last datum ends the pick");
    let tertiary = params(&h)["tertiary"].clone();
    let p = vec_of(&tertiary["point"]["point"]);
    let b = h.state().session.doc.bbox();
    assert!((p[1] - b.min.y).abs() < 1e-6, "on the front face: {p:?}");
    assert!(p[0] > b.min.x && p[0] < b.max.x && p[2] > b.min.z && p[2] < b.max.z);

    // the picked datums are valid input of the command
    h.state_mut().run_palette();
    let last = h.state().session.log.last().expect("a command ran");
    assert_eq!(last.command, "align.datum");
    assert!(last.ok, "{:?}", last.outcome);
}

/// Three points on one line give no plane: the pick says so and asks for the point again.
#[test]
fn collinear_points_do_not_make_a_plane() {
    let (mut h, at) = scan_harness();
    h.state_mut().choose_command("align.datum");
    settle(&mut h);
    pick_button(&mut h, 0);
    let before = params(&h)["primary"].clone();
    for fx in [0.2, 0.5, 0.8] {
        click_model(&mut h, at(fx, 0.5));
    }
    assert_eq!(params(&h)["primary"], before);
    let fp = h.state().field_pick.as_ref().expect("still picking the primary datum");
    assert_eq!((fp.field(), fp.buffer.len(), fp.marks.len()), (Some("primary"), 0, 0));
    assert_eq!(h.state().status, i18n::lookup(Lang::En, "fieldpick.degenerate_plane"));
}

/// Esc and a right click end picking and keep what was picked; a click on the background picks
/// nothing; while picking, the toolbar tool does not act on clicks.
#[test]
fn escape_and_right_click_end_picking() {
    let (mut h, at) = scan_harness();
    h.state_mut().choose_command("align.n_point");
    settle(&mut h);
    h.state_mut().start_field_pick("from", PickKind::Points);
    settle(&mut h);
    click_model(&mut h, at(0.3, 0.4));
    click_model(&mut h, at(0.6, 0.4));
    let from = params(&h)["from"].clone();
    assert_eq!(from.as_array().map(Vec::len), Some(2), "{from}");
    // a click far from the scan picks nothing
    let rect = h.state().viewport_rect.expect("viewport");
    click_at(&mut h, rect.min + egui::vec2(rect.width() * 0.3, rect.height() * 0.95));
    assert_eq!(params(&h)["from"], from);
    assert!(h.state().field_pick.is_some());
    // Esc ends it, the points stay
    h.key_press(egui::Key::Escape);
    settle(&mut h);
    assert!(h.state().field_pick.is_none());
    assert_eq!(params(&h)["from"], from);

    // a right click ends it too, and a click afterwards is an ordinary pick of the scan
    h.state_mut().start_field_pick("to", PickKind::Points);
    settle(&mut h);
    click_model(&mut h, at(0.4, 0.6));
    assert_eq!(params(&h)["to"].as_array().map(Vec::len), Some(1));
    right_click_model(&mut h, at(0.5, 0.5));
    assert!(h.state().field_pick.is_none());
    click_model(&mut h, at(0.5, 0.5));
    assert_eq!(params(&h)["to"].as_array().map(Vec::len), Some(1), "no longer picking into the field");
    assert_eq!(h.state().picks.len(), 1, "an ordinary click picks a point");
}

/// A body with its front face on screen; the top and bottom edges of that face are on screen at
/// `top` and `bottom`.
fn body_harness() -> (Harness<'static, SurfaceCraftApp>, Point3, Point3) {
    let mut app = SurfaceCraftApp::headless();
    app.run_command("body.box", json!({"size": [20.0, 10.0, 5.0]})).expect("a body");
    app.camera.fit(&app.session.doc.bbox());
    app.camera.set_standard(StandardView::Front);
    let mut h = harness(app);
    settle(&mut h);
    (h, Point3::new(10.0, 0.0, 5.0), Point3::new(10.0, 0.0, 0.0))
}

/// Clicking body edges fills the `edges` of a fillet: a click adds an edge, a second click on it
/// removes it, the body parameter follows the clicked body, typed values stay editable, and the
/// fillet runs on the picked edge.
#[test]
fn clicks_on_edges_fill_the_fillet_and_toggle() {
    let (mut h, top, bottom) = body_harness();
    h.state_mut().choose_command("body.fillet");
    settle(&mut h);
    assert_eq!(params(&h)["edges"], json!([]));
    pick_button(&mut h, 0);
    assert_eq!(h.state().field_pick.as_ref().and_then(|f| f.kind()), Some(PickKind::Edges));
    let body = h.state().session.doc.bodies[0].id;

    // a little off the line: the pixel tolerance finds the edge
    let top_px = screen(&h, top) + egui::vec2(0.0, 2.0);
    click_at(&mut h, top_px);
    let first = params(&h)["edges"].clone();
    assert_eq!(first.as_array().map(Vec::len), Some(1), "{first}");
    assert_eq!(params(&h)["object"], json!(body.0));
    assert_eq!(h.state().tools.brep.edge_list().len(), 1, "the edge is highlighted");

    let bottom_px = screen(&h, bottom) - egui::vec2(0.0, 2.0);
    click_at(&mut h, bottom_px);
    let both = params(&h)["edges"].clone();
    assert_eq!(both.as_array().map(Vec::len), Some(2), "{both}");
    assert_eq!(both[0], first[0], "the edges keep the order of the clicks");

    // the second click on an edge removes it again
    click_at(&mut h, bottom_px);
    assert_eq!(params(&h)["edges"], first);

    // the field can be typed over while picking; the next click works on what is there now
    let mut typed = params(&h);
    typed["edges"] = json!([]);
    h.state_mut().palette_params = typed.to_string();
    settle(&mut h);
    click_at(&mut h, bottom_px);
    assert_eq!(params(&h)["edges"].as_array().map(Vec::len), Some(1));
    click_at(&mut h, top_px);
    click_at(&mut h, bottom_px);
    assert_eq!(params(&h)["edges"], first, "only the top edge is left");

    // a click on nothing leaves the list alone; Esc ends picking and the fillet runs
    let rect = h.state().viewport_rect.expect("viewport");
    click_at(&mut h, rect.min + egui::vec2(rect.width() * 0.1, rect.height() * 0.9));
    assert_eq!(params(&h)["edges"], first);
    h.key_press(egui::Key::Escape);
    settle(&mut h);
    assert!(h.state().field_pick.is_none());
    let mut p = params(&h);
    p["size"] = json!(1.0);
    h.state_mut().palette_params = p.to_string();
    h.state_mut().run_palette();
    assert!(h.state().status.contains("done"), "fillet of the picked edge: {}", h.state().status);
}

/// The chamfer shares the form machinery, so its edges are picked the same way.
#[test]
fn the_chamfer_form_picks_edges_too() {
    let (mut h, top, _) = body_harness();
    h.state_mut().choose_command("body.chamfer");
    settle(&mut h);
    pick_button(&mut h, 0);
    let top_px = screen(&h, top) + egui::vec2(0.0, 2.0);
    click_at(&mut h, top_px);
    assert_eq!(params(&h)["edges"].as_array().map(Vec::len), Some(1));
}

/// Choosing another command drops a pick in progress.
#[test]
fn another_command_ends_the_pick() {
    let (mut h, _) = scan_harness();
    h.state_mut().choose_command("align.datum");
    settle(&mut h);
    pick_button(&mut h, 0);
    assert!(h.state().field_pick.is_some());
    h.state_mut().choose_command("align.principal");
    settle(&mut h);
    assert!(h.state().field_pick.is_none());
}

#[test]
fn datum_values_match_the_clicked_points() {
    let v = datum_value(&[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]).expect("a plane");
    assert_eq!(vec_of(&v["plane"]["normal"]), [0.0, 0.0, 1.0]);
}

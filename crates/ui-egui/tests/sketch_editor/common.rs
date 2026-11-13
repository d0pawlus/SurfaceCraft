//! Helpers shared by the sketch editor tests: sketches, the editor in a tool, clicks, sessions and the UI harness.
use egui_kittest::Harness;
use serde_json::{Value, json};
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::Session;
use surfacecraft_sketch::{Entity, Sketch};
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::sketch_editor::snap::{Snap, SnapConfig};
use surfacecraft_ui_egui::sketch_editor::view::PlaneView;
use surfacecraft_ui_egui::sketch_editor::{Cmd, SketchEditor, Tool};

/// A 10 x 5 rectangle (lines 0..=3) and a circle of radius 3 at (20, 0) (entity 4, point 4).
pub(super) fn rect_sketch() -> Sketch {
    let mut s = Sketch::default();
    for p in [[0.0, 0.0], [10.0, 0.0], [10.0, 5.0], [0.0, 5.0], [20.0, 0.0]] {
        s.add_point(p);
    }
    for (a, b) in [(0, 1), (1, 2), (2, 3), (3, 0)] {
        s.entities.push(Entity::Line { a, b });
    }
    s.entities.push(Entity::Circle { center: 4, radius: 3.0 });
    s
}

pub(super) fn editor(tool: Tool) -> SketchEditor {
    let mut e = SketchEditor::default();
    e.begin(ObjectId(1));
    e.set_tool(tool);
    e
}

pub(super) fn click(e: &mut SketchEditor, sk: &Sketch, p: [f64; 2]) -> Vec<Cmd> {
    e.click(sk, Snap::free(p), 0.5)
}

pub(super) fn constrain(c: Value) -> Vec<Cmd> {
    vec![Cmd { id: "sketch.constrain", params: json!({"sketch": 1, "constraints": [c]}) }]
}
/// Runs editor commands through a session, as the app does.
pub(super) fn run(s: &mut Session, cmds: &[Cmd]) {
    for c in cmds {
        s.execute(c.id, c.params.clone()).unwrap_or_else(|err| panic!("{}: {err}", c.id));
    }
}

pub(super) fn new_sketch(s: &mut Session, params: Value) -> ObjectId {
    let v = s.execute("sketch.create", params).expect("sketch.create");
    ObjectId(v["id"].as_u64().expect("id"))
}

pub(super) fn harness(app: SurfaceCraftApp) -> Harness<'static, SurfaceCraftApp> {
    Harness::builder().with_size((1600.0, 1000.0)).build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app)
}

pub(super) fn screen_of(app: &SurfaceCraftApp, id: ObjectId, p: [f64; 2]) -> egui::Pos2 {
    let rect = app.viewport_rect.expect("viewport");
    let plane = app.session.doc.sketch(id).expect("sketch").sketch.plane.clone();
    let q = PlaneView::new(&app.camera, &plane, rect.width(), rect.height()).to_screen(p).expect("on screen");
    rect.min + egui::vec2(q[0], q[1])
}

pub(super) fn click_at(h: &mut Harness<'static, SurfaceCraftApp>, pos: egui::Pos2) {
    h.hover_at(pos);
    h.run();
    h.drag_at(pos);
    h.drop_at(pos);
    h.run();
}

pub(super) fn no_snaps(h: &mut Harness<'static, SurfaceCraftApp>) {
    h.state_mut().sketch_editor.snap = SnapConfig { points: false, midpoints: false, inference: false, grid: false };
}

pub(super) fn set_text(e: &mut SketchEditor, text: &str) {
    if let Some(p) = e.pending.as_mut() {
        p.text = text.into();
    }
}

/// The polyline of the drag tests: points (0, 0), (10, 0), (10, 5); lines 0-1 and 1-2.
pub(super) fn polyline() -> Sketch {
    let mut s = Sketch::default();
    for p in [[0.0, 0.0], [10.0, 0.0], [10.0, 5.0]] {
        s.add_point(p);
    }
    s.entities.push(Entity::Line { a: 0, b: 1 });
    s.entities.push(Entity::Line { a: 1, b: 2 });
    s
}

/// A sketch on the XY plane, edited (the camera looks onto it), as the tests above start.
pub(super) fn new_sketch_on_xy(app: &mut SurfaceCraftApp) -> ObjectId {
    app.new_sketch_on([0.0, 0.0, 1.0]);
    app.sketch_editor.target.expect("editing")
}

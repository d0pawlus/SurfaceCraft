//! Screenshots of the shell for the owner's UX review (off by default: they need a GPU).
//!
//! `SURFACECRAFT_SHOTS=<dir> cargo test -p surfacecraft-ui-egui --test shell_screenshots` writes
//! `p6-*.png` into `<dir>`: the egui UI is rendered by `egui_kittest`'s wgpu renderer and the
//! 3D viewport, which is a wgpu callback the test renderer does not run, is rendered offscreen
//! by the application's own renderer ([`SurfaceCraftApp::viewport_frame`]) and composited under
//! the overlays the UI paints over the viewport (gizmo, view cube, lasso, sketch, toolbar).
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::json;
use std::collections::HashMap;
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::camera::StandardView;
use surfacecraft_ui_egui::i18n::Lang;
use surfacecraft_ui_egui::sectionview::SectionAxis;
use surfacecraft_ui_egui::select::SelectOp;
use surfacecraft_ui_egui::settings::UiSettings;
use surfacecraft_ui_egui::theme::{Density, Preset, Theme};
use surfacecraft_ui_egui::viewport;
use surfacecraft_ui_egui::viewtools::Tool;

type H = Harness<'static, SurfaceCraftApp>;

fn run(app: &mut SurfaceCraftApp, id: &str, p: serde_json::Value) {
    app.run_command(id, p).unwrap_or_else(|e| panic!("{id}: {e}"));
}

/// A scene with a scan, a region, a body and a sketch.
fn scene(app: &mut SurfaceCraftApp) {
    run(app, "mesh.synth", json!({"shape": "wave", "resolution": 48, "name": "scan"}));
    let n = app.session.doc.meshes[0].mesh.triangle_count() as u32;
    run(
        app,
        "region.create",
        json!({"mesh": "scan", "triangles": (0..n / 2).collect::<Vec<u32>>(), "name": "left half"}),
    );
    run(app, "body.box", json!({"size": [24.0, 16.0, 8.0], "name": "block"}));
    app.camera.fit(&app.session.doc.bbox());
    app.camera.orbit(70.0, -45.0);
}

/// Renders the harness: the UI through egui_kittest, the viewport offscreen and composited under
/// the pixels the UI painted over it (anything that is not the empty viewport's fill).
fn render(h: &mut H, dir: &std::path::Path, file: &str) {
    h.run();
    h.run();
    let rect = h.state().viewport_rect.expect("viewport drawn");
    let (vw, vh) = (rect.width().round() as u32, rect.height().round() as u32);
    let frame = h.state_mut().viewport_frame([vw, vh]);
    let mut ui_image = match h.render() {
        Ok(i) => i,
        Err(e) => panic!("render: {e}"),
    };
    let Some(vp) = viewport::render_offscreen(&frame) else {
        eprintln!("no GPU for the offscreen viewport; {file} not written");
        return;
    };
    let (iw, ih) = (ui_image.width(), ui_image.height());
    let raw: &mut [u8] = &mut ui_image;
    let (x0, y0) = (rect.min.x.round() as u32, rect.min.y.round() as u32);
    let at = |x: u32, y: u32| ((y * iw + x) * 4) as usize;
    // the empty viewport's fill is the most frequent colour inside its rectangle
    let mut counts: HashMap<[u8; 4], usize> = HashMap::new();
    for y in 0..vh.min(ih.saturating_sub(y0)) {
        for x in 0..vw.min(iw.saturating_sub(x0)) {
            let d = at(x0 + x, y0 + y);
            *counts.entry([raw[d], raw[d + 1], raw[d + 2], raw[d + 3]]).or_default() += 1;
        }
    }
    let hole = counts.into_iter().max_by_key(|(_, n)| *n).map(|(c, _)| c).unwrap_or([0; 4]);
    for y in 0..vh {
        for x in 0..vw {
            let (dx, dy) = (x0 + x, y0 + y);
            if dx >= iw || dy >= ih {
                continue;
            }
            let d = at(dx, dy);
            if raw[d..d + 4] == hole {
                let s = ((y * vw + x) * 4) as usize;
                raw[d..d + 4].copy_from_slice(&vp[s..s + 4]);
            }
        }
    }
    let png = surfacecraft_ui_egui::shot::encode_png(&ui_image, iw, ih).expect("png");
    std::fs::write(dir.join(file), png).expect("write the screenshot");
}

fn harness(app: SurfaceCraftApp) -> H {
    harness_sized(app, (1500.0, 900.0))
}

fn harness_sized(mut app: SurfaceCraftApp, size: (f32, f32)) -> H {
    app.lay_out_for_gpu();
    let mut h = Harness::builder()
        .with_size(size)
        .with_pixels_per_point(1.0)
        .wgpu()
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app);
    h.run();
    h
}

fn shell(dir: &std::path::Path, name: &str, theme: Theme, lang: Lang, log_hidden: bool) {
    let mut app = SurfaceCraftApp::with_settings(UiSettings { theme, lang, workspace: None });
    scene(&mut app);
    if log_hidden {
        let _ =
            surfacecraft_ui_egui::dock::set_visible(&mut app.workspace, surfacecraft_ui_egui::dock::Panel::Log, false);
    }
    let mut h = harness(app);
    render(&mut h, dir, &format!("p6-g1-{name}.png"));
}

/// A noisy plane measured against the clean plane: the Deviation panel with its map.
fn deviation(dir: &std::path::Path) {
    let mut app = SurfaceCraftApp::headless();
    let noisy =
        json!({"shape": "plane", "size": [40.0, 40.0], "resolution": 60, "noise": 0.04, "seed": 7, "name": "scan"});
    run(&mut app, "mesh.synth", noisy);
    run(&mut app, "mesh.synth", json!({"shape": "plane", "size": [40.0, 40.0], "resolution": 60, "name": "ref"}));
    run(&mut app, "deviation.compute", json!({"object": "scan", "against": "ref", "tolerance": 0.1}));
    app.camera.fit(&app.session.doc.bbox());
    app.camera.orbit(40.0, -50.0);
    let mut h = harness(app);
    h.get_by_label("Deviation").click();
    h.run();
    render(&mut h, dir, "p6-deviation.png");
}

/// The sketch editor with a fully constrained 40 x 20 rectangle and its dimensions.
fn sketch(dir: &std::path::Path) {
    let mut app = SurfaceCraftApp::headless();
    app.new_sketch_on([0.0, 0.0, 1.0]);
    let id = app.sketch_editor.target.expect("editing").0;
    let pts = [[0.0, 0.0], [40.0, 0.0], [40.0, 20.0], [0.0, 20.0]];
    run(&mut app, "sketch.add", json!({"sketch": id, "entities": [{"polygon": {"points": pts}}]}));
    run(
        &mut app,
        "sketch.constrain",
        json!({"sketch": id, "constraints": [
            {"type": "fix", "point": 0, "at": [0.0, 0.0]},
            {"type": "horizontal", "line": 0}, {"type": "vertical", "line": 1},
            {"type": "horizontal", "line": 2}, {"type": "vertical", "line": 3},
            {"type": "length", "line": 0, "value": 40.0}, {"type": "length", "line": 1, "value": 20.0}]}),
    );
    app.begin_sketch_edit(surfacecraft_doc::ObjectId(id));
    let mut h = harness(app);
    render(&mut h, dir, "p6-sketch.png");
}

/// The Properties panel's form for `feature.extrude` with the live preview of the result.
fn form(dir: &std::path::Path) {
    let mut app = SurfaceCraftApp::headless();
    run(&mut app, "sketch.create", json!({"name": "base"}));
    let pts = [[0.0, 0.0], [40.0, 0.0], [40.0, 20.0], [0.0, 20.0]];
    run(&mut app, "sketch.add", json!({"sketch": "base", "entities": [{"polygon": {"points": pts}}]}));
    app.camera.fit(&app.session.doc.bbox());
    app.camera.orbit(50.0, -35.0);
    app.choose_command("feature.extrude");
    app.palette_params = json!({"sketch": "base", "distance": 12.0, "name": "plate"}).to_string();
    let mut h = harness(app);
    for _ in 0..3 {
        h.run();
    }
    h.get_by_label("Preview").click();
    for _ in 0..3 {
        h.run();
    }
    render(&mut h, dir, "p6-form-extrude.png");
}

/// The Move gizmo on a selected scan and the view cube.
fn gizmo(dir: &std::path::Path) {
    let mut app = SurfaceCraftApp::headless();
    scene(&mut app);
    app.selected_object = Some(app.session.doc.meshes[0].id);
    app.tools.tool = Tool::Move;
    let mut h = harness(app);
    render(&mut h, dir, "p6-gizmo-viewcube.png");
}

/// A narrow window: the viewport's tool strip wraps before the view cube's corner.
fn narrow(dir: &std::path::Path) {
    let mut app = SurfaceCraftApp::headless();
    scene(&mut app);
    app.selected_object = Some(app.session.doc.meshes[0].id);
    app.tools.tool = Tool::Move;
    let mut h = harness_sized(app, (980.0, 760.0));
    render(&mut h, dir, "p6-narrow-viewcube.png");
}

/// A lasso selection on a sphere scan: the outline and the selected triangles.
fn lasso(dir: &std::path::Path) {
    let mut app = SurfaceCraftApp::headless();
    run(&mut app, "mesh.synth", json!({"shape": "sphere", "size": [20.0], "resolution": 96, "name": "ball"}));
    app.camera.fit(&app.session.doc.bbox());
    app.camera.set_standard(StandardView::Front);
    app.tools.tool = Tool::Lasso;
    let mut h = harness(app);
    h.run();
    let rect = h.state().viewport_rect.expect("viewport drawn");
    let (w, hgt) = (f64::from(rect.width()), f64::from(rect.height()));
    let poly: Vec<[f64; 2]> = [[0.38, 0.25], [0.62, 0.30], [0.66, 0.55], [0.50, 0.72], [0.34, 0.58]]
        .iter()
        .map(|p| [p[0] * w, p[1] * hgt])
        .collect();
    h.state_mut().select_lasso(&poly, w, hgt, SelectOp::Replace);
    h.state_mut().tools.lasso = poly;
    render(&mut h, dir, "p6-lasso.png");
}

/// A section plane through the scene.
fn section(dir: &std::path::Path) {
    let mut app = SurfaceCraftApp::headless();
    scene(&mut app);
    app.enable_section(SectionAxis::Z);
    let mut h = harness(app);
    render(&mut h, dir, "p6-section.png");
}

#[test]
fn write_shell_screenshots() {
    let Some(dir) = std::env::var_os("SURFACECRAFT_SHOTS") else { return };
    let dir = std::path::PathBuf::from(dir);
    std::fs::create_dir_all(&dir).expect("screenshot directory");
    let t = |preset, density| Theme { preset, density };
    shell(&dir, "dark", t(Preset::Dark, Density::Comfortable), Lang::En, false);
    shell(&dir, "light", t(Preset::Light, Density::Comfortable), Lang::En, false);
    shell(&dir, "high-contrast", t(Preset::HighContrast, Density::Comfortable), Lang::En, false);
    shell(&dir, "dark-compact-pl", t(Preset::Dark, Density::Compact), Lang::Pl, true);
    deviation(&dir);
    sketch(&dir);
    form(&dir);
    gizmo(&dir);
    narrow(&dir);
    lasso(&dir);
    section(&dir);
}

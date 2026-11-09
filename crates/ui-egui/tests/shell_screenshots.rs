//! Screenshots of the shell for the owner's UX review (off by default: they need a GPU).
//!
//! `SURFACECRAFT_SHOTS=<dir> cargo test -p surfacecraft-ui-egui --test shell_screenshots` writes
//! `p6-g1-*.png` into `<dir>`: the egui UI is rendered by `egui_kittest`'s wgpu renderer and the
//! 3D viewport, which is a wgpu callback the test renderer does not run, is rendered offscreen
//! by the application's own renderer and placed where the viewport panel was drawn.
use egui_kittest::Harness;
use serde_json::json;
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::camera::Camera;
use surfacecraft_ui_egui::i18n::Lang;
use surfacecraft_ui_egui::settings::UiSettings;
use surfacecraft_ui_egui::theme::{Density, Preset, Theme, Tokens};
use surfacecraft_ui_egui::viewport::{self, DrawItem, Frame};

/// A scene with a scan, a region, a body and a sketch.
fn scene(app: &mut SurfaceCraftApp) {
    let run = |app: &mut SurfaceCraftApp, id: &str, p: serde_json::Value| {
        app.run_command(id, p).unwrap_or_else(|e| panic!("{id}: {e}"));
    };
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

/// The viewport of `app` rendered offscreen at `w × h` pixels with the preset's colours.
fn viewport_image(app: &mut SurfaceCraftApp, tokens: &Tokens, w: u32, h: u32) -> Option<Vec<u8>> {
    let items: Vec<DrawItem> = app
        .session
        .display_items()
        .into_iter()
        .map(|d| DrawItem {
            key: d.key,
            color: tokens.object(d.kind),
            colors: d.colors.clone(),
            colors_key: d.colors_key,
            highlight: None,
            mesh: d.mesh,
        })
        .collect();
    let mut camera: Camera = app.camera.clone();
    camera.fit(&app.session.doc.bbox());
    camera.orbit(70.0, -45.0);
    let frame = Frame {
        size: [w, h],
        origin: camera.target,
        camera,
        items,
        background: tokens.viewport_background().map(f64::from),
        overlay: Default::default(),
    };
    viewport::render_offscreen(&frame)
}

fn shoot(dir: &std::path::Path, name: &str, theme: Theme, lang: Lang, panels_hidden: bool) {
    let settings = UiSettings { theme, lang, workspace: None };
    let mut app = SurfaceCraftApp::with_settings(settings);
    scene(&mut app);
    if panels_hidden {
        let _ =
            surfacecraft_ui_egui::dock::set_visible(&mut app.workspace, surfacecraft_ui_egui::dock::Panel::Log, false);
    }
    let mut h = Harness::builder()
        .with_size((1500.0, 900.0))
        .with_pixels_per_point(1.0)
        .wgpu()
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app);
    h.run();
    h.run();
    let tokens = h.state().tokens.clone();
    let rect = h.state().viewport_rect.expect("viewport drawn");
    let mut ui_image = match h.render() {
        Ok(i) => i,
        Err(e) => panic!("render: {e}"),
    };
    let (vw, vh) = (rect.width().round() as u32, rect.height().round() as u32);
    let Some(vp) = viewport_image(h.state_mut(), &tokens, vw, vh) else {
        eprintln!("no GPU for the offscreen viewport; {name} not written");
        return;
    };
    let (iw, ih) = (ui_image.width(), ui_image.height());
    let raw: &mut [u8] = &mut ui_image;
    for y in 0..vh {
        for x in 0..vw {
            let (dx, dy) = (rect.min.x.round() as u32 + x, rect.min.y.round() as u32 + y);
            if dx >= iw || dy >= ih {
                continue;
            }
            let s = ((y * vw + x) * 4) as usize;
            let d = ((dy * iw + dx) * 4) as usize;
            raw[d..d + 4].copy_from_slice(&vp[s..s + 4]);
        }
    }
    let png = surfacecraft_ui_egui::shot::encode_png(&ui_image, iw, ih).expect("png");
    std::fs::write(dir.join(format!("p6-g1-{name}.png")), png).expect("write the screenshot");
}

#[test]
fn write_shell_screenshots() {
    let Some(dir) = std::env::var_os("SURFACECRAFT_SHOTS") else { return };
    let dir = std::path::PathBuf::from(dir);
    std::fs::create_dir_all(&dir).expect("screenshot directory");
    let t = |preset, density| Theme { preset, density };
    shoot(&dir, "dark", t(Preset::Dark, Density::Comfortable), Lang::En, false);
    shoot(&dir, "light", t(Preset::Light, Density::Comfortable), Lang::En, false);
    shoot(&dir, "high-contrast", t(Preset::HighContrast, Density::Comfortable), Lang::En, false);
    shoot(&dir, "dark-compact-pl", t(Preset::Dark, Density::Compact), Lang::Pl, true);
}

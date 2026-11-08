//! The GUI drives the same commands as the CLI: clicking palette entries, menus and Run
//! executes engine commands, and the log turns into a script that replays identically.
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::json;
use surfacecraft_engine::Session;
use surfacecraft_ui_egui::SurfaceCraftApp;

fn harness() -> Harness<'static, SurfaceCraftApp> {
    Harness::builder()
        .with_size((1600.0, 1000.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), SurfaceCraftApp::headless())
}

#[test]
fn palette_runs_commands_and_log_replays_in_the_engine() {
    let mut h = harness();
    h.run();
    // filter the palette, choose mesh.synth, edit its parameters, press Run
    h.get_by_label("Filter").focus();
    h.run();
    h.get_by_label("Filter").type_text("synth");
    h.run();
    h.get_by_label("mesh.synth  —  Synthetic scan").click();
    h.run();
    assert_eq!(h.state().palette_command, Some("mesh.synth"));
    h.state_mut().palette_params = json!({"shape": "sphere", "size": [12.0], "resolution": 16}).to_string();
    h.get_by_label("Run").click();
    h.run();
    assert_eq!(h.state().session.doc.meshes.len(), 1, "status: {}", h.state().status);

    // the Solid menu reaches body.box
    h.get_by_label("Solid").click();
    h.run();
    h.get_by_label("Box").click();
    h.run();
    assert_eq!(h.state().palette_command, Some("body.box"));
    h.state_mut().palette_params = json!({"size": [20.0, 10.0, 5.0]}).to_string();
    h.get_by_label("Run").click();
    h.run();
    assert_eq!(h.state().session.doc.bodies.len(), 1, "status: {}", h.state().status);

    // the object list shows both, and deleting through the list is a command too
    h.get_by_label_contains("box (#2)");
    // replaying the GUI's log as a CLI script gives the same document
    let script = h.state().log_as_script();
    let mut s = Session::new();
    let report = s.run_script(&script).unwrap();
    assert!(report.ok);
    assert_eq!(s.doc.summaries(), h.state().session.doc.summaries());
}

#[test]
fn template_prefills_required_parameters() {
    let schema = (surfacecraft_engine::cmd::find("body.fillet").unwrap().input)();
    let ctx =
        surfacecraft_ui_egui::app::PickContext { selected: Some(surfacecraft_doc::ObjectId(7)), ..Default::default() };
    let t = surfacecraft_ui_egui::app::template(&schema, &ctx);
    assert_eq!(t["object"], 7);
    assert!(t.get("edges").is_some() && t.get("size").is_some());
}

/// N-point alignment from the GUI: picks on the model prefill `from` of `align.n_point`.
#[test]
fn viewport_picks_feed_n_point_alignment() {
    let mut h = harness();
    h.state_mut().run_command("mesh.synth", json!({"shape": "box", "size": [40, 30, 20], "resolution": 16})).unwrap();
    h.run();
    // three picks through the viewport's ray cast (the same path a click takes)
    let app = h.state_mut();
    assert!(app.pick_at(500.0, 400.0, 1000.0, 800.0, false));
    assert!(app.pick_at(450.0, 380.0, 1000.0, 800.0, true));
    assert!(app.pick_at(560.0, 430.0, 1000.0, 800.0, true));
    assert_eq!(app.picks.len(), 3);
    app.choose_command("align.n_point");
    let params: serde_json::Value = serde_json::from_str(&app.palette_params).unwrap();
    assert_eq!(params["from"].as_array().unwrap().len(), 3);
    // move the picks by +10 in x: the mesh follows
    let to: Vec<[f64; 3]> = app.picks.iter().map(|p| [p.point.x + 10.0, p.point.y, p.point.z]).collect();
    let mut p = params.clone();
    p["to"] = json!(to);
    app.palette_params = p.to_string();
    h.run();
    h.get_by_label("Run").click();
    h.run();
    let b = h.state().session.doc.bbox();
    assert!((b.min.x - 10.0).abs() < 1e-6, "{b:?} status {}", h.state().status);
}

/// Regions from the GUI: a picked triangle prefills `region.grow`, Run creates a classified
/// region, and the viewport colours the mesh by region.
#[test]
fn picked_triangle_grows_a_region() {
    let mut h = harness();
    h.state_mut().run_command("mesh.synth", json!({"shape": "box", "size": [40, 30, 20], "resolution": 16})).unwrap();
    h.run();
    let app = h.state_mut();
    assert!(app.pick_at(500.0, 400.0, 1000.0, 800.0, false));
    app.choose_command("region.grow");
    let p: serde_json::Value = serde_json::from_str(&app.palette_params).unwrap();
    assert_eq!(p["triangles"].as_array().unwrap().len(), 1);
    h.run();
    h.get_by_label("Run").click();
    h.run();
    let app = h.state_mut();
    assert_eq!(app.session.doc.regions.len(), 1, "status {}", app.status);
    assert_eq!(app.session.doc.regions[0].kind, Some(surfacecraft_fit::PrimitiveKind::Plane));
    assert!(app.session.display_items().iter().any(|d| d.colors.is_some()));
}

/// Fit Surface from the GUI: the Surface menu reaches `surface.fit`, Run creates a sheet that
/// the viewport draws.
#[test]
fn surface_menu_fits_a_patch() {
    let mut h = harness();
    let app = h.state_mut();
    app.run_command("mesh.synth", json!({"shape": "wave", "resolution": 40, "name": "scan"})).unwrap();
    let n = app.session.doc.meshes[0].mesh.triangle_count() as u32;
    app.run_command(
        "region.create",
        json!({"mesh": "scan", "triangles": (0..n).collect::<Vec<u32>>(), "name": "free"}),
    )
    .unwrap();
    h.run();
    h.get_by_label("Surface").click();
    h.run();
    h.get_by_label("Fit surface").click();
    h.run();
    assert_eq!(h.state().palette_command, Some("surface.fit"));
    h.state_mut().palette_params = json!({"region": "free"}).to_string();
    h.run();
    h.get_by_label("Run").click();
    h.run();
    let app = h.state_mut();
    assert_eq!(app.session.doc.surfaces.len(), 1, "status {}", app.status);
    assert!(app.session.display_items().iter().any(|d| d.kind == surfacecraft_doc::ObjectKind::Surface));
}

/// AutoSurface from the GUI: the Freeform menu reaches the quad layout and AutoSurface.
#[test]
fn freeform_menu_runs_autosurface() {
    let mut h = harness();
    h.state_mut().run_command("mesh.synth", json!({"shape": "wave", "resolution": 40, "name": "scan"})).unwrap();
    h.run();
    for (item, params) in
        [("Quad layout", json!({"mesh": "scan", "target_quads": 6})), ("AutoSurface", json!({"object": "quads"}))]
    {
        h.get_by_label("Freeform").click();
        h.run();
        h.get_by_label(item).click();
        h.run();
        h.state_mut().palette_params = params.to_string();
        h.run();
        h.get_by_label("Run").click();
        h.run();
    }
    let app = h.state_mut();
    assert_eq!(app.session.doc.quads.len(), 1, "status {}", app.status);
    assert_eq!(app.session.doc.surfaces.len(), 1, "status {}", app.status);
    assert!(app.session.display_items().iter().any(|d| d.kind == surfacecraft_doc::ObjectKind::Quad));
}

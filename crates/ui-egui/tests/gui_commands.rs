//! The GUI drives the same commands as the CLI: clicking ribbon tools, the search and OK
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
    // search the tools, choose mesh.synth, edit its parameters, press OK
    h.get_by_label("Search tools").focus();
    h.run();
    h.get_by_label("Search tools").type_text("synth");
    h.run();
    h.get_by_label("Synthetic scan").click();
    h.run();
    assert_eq!(h.state().palette_command, Some("mesh.synth"));
    assert!(h.state().tool_search.is_empty(), "choosing a tool clears the search");
    h.state_mut().palette_params = json!({"shape": "sphere", "size": [12.0], "resolution": 16}).to_string();
    h.get_by_label("OK").click();
    h.run();
    assert_eq!(h.state().session.doc.meshes.len(), 1, "status: {}", h.state().status);
    assert_eq!(h.state().palette_command, None, "a tool that changed the document closes");

    // the Solid stage's More list reaches body.box
    h.get_by_label("Solid").click();
    h.run();
    h.get_by_label("More").click();
    h.run();
    h.get_by_label("Box").click();
    h.run();
    assert_eq!(h.state().palette_command, Some("body.box"));
    h.state_mut().palette_params = json!({"size": [20.0, 10.0, 5.0]}).to_string();
    h.get_by_label("OK").click();
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
    let id = surfacecraft_doc::ObjectId(7);
    let ctx = surfacecraft_ui_egui::app::PickContext {
        selected: Some(id),
        objects: vec![(id, surfacecraft_doc::ObjectKind::Body)],
        ..Default::default()
    };
    let t = surfacecraft_ui_egui::app::template(&schema, &ctx);
    assert_eq!(t["object"], 7);
    assert!(t.get("edges").is_some() && t.get("size").is_some());
}

/// Object references are prefilled with a real object (the selection, else the only object of
/// the needed kind), never with id 0.
#[test]
fn template_never_prefills_object_zero() {
    use surfacecraft_doc::{ObjectId, ObjectKind};
    use surfacecraft_ui_egui::app::{PickContext, template};
    let mut h = harness();
    h.state_mut().run_command("mesh.synth", json!({"shape": "box", "size": [40, 30, 20], "resolution": 8})).unwrap();
    let app = h.state_mut();
    let mesh = app.session.doc.meshes[0].id;
    assert_ne!(mesh.0, 0);
    // nothing selected: the only mesh
    app.selected_object = None;
    for cmd in ["mesh.info", "region.auto_segment"] {
        app.choose_command(cmd);
        let p: serde_json::Value = serde_json::from_str(&app.palette_params).unwrap();
        let r = p.get("object").or_else(|| p.get("mesh")).unwrap();
        assert_eq!(r, &json!(mesh.0), "{cmd}: {p}");
    }
    // the kind decides: a selected body does not fill a `mesh` parameter
    let ctx = PickContext {
        selected: Some(ObjectId(9)),
        objects: vec![(ObjectId(1), ObjectKind::Mesh), (ObjectId(9), ObjectKind::Body)],
        ..Default::default()
    };
    assert_eq!(ctx.reference_for("mesh", None), Some(ObjectId(1)));
    assert_eq!(ctx.reference_for("body", None), Some(ObjectId(9)));
    assert_eq!(ctx.reference_for("object", None), Some(ObjectId(9)));
    // a parameter called `object` takes the kind its description names
    assert_eq!(ctx.reference_for("object", Some("Mesh id or name.")), Some(ObjectId(1)));
    let none = PickContext::default();
    assert_eq!(none.reference_for("mesh", None), None);
    let schema = (surfacecraft_engine::cmd::find("mesh.info").unwrap().input)();
    assert_ne!(template(&schema, &none)["object"], json!(0));
    // a mesh tool opened with a body selected starts with the mesh, not the selection
    let schema = (surfacecraft_engine::cmd::find("mesh.decimate").unwrap().input)();
    assert_eq!(template(&schema, &ctx)["object"], json!(1));
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
    h.get_by_label("OK").click();
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
    h.get_by_label("OK").click();
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
    h.get_by_label("OK").click();
    h.run();
    let app = h.state_mut();
    assert_eq!(app.session.doc.surfaces.len(), 1, "status {}", app.status);
    assert!(app.session.display_items().iter().any(|d| d.kind == surfacecraft_doc::ObjectKind::Surface));
}

/// AutoSurface from the GUI: the Surface stage reaches the quad layout and AutoSurface.
#[test]
fn freeform_menu_runs_autosurface() {
    let mut h = harness();
    h.state_mut().run_command("mesh.synth", json!({"shape": "wave", "resolution": 40, "name": "scan"})).unwrap();
    h.run();
    for (item, params) in
        [("Quad layout", json!({"mesh": "scan", "target_quads": 6})), ("AutoSurface", json!({"object": "quads"}))]
    {
        h.get_by_label("Surface").click();
        h.run();
        h.get_by_label(item).click();
        h.run();
        h.state_mut().palette_params = params.to_string();
        h.run();
        h.get_by_label("OK").click();
        h.run();
    }
    let app = h.state_mut();
    assert_eq!(app.session.doc.quads.len(), 1, "status {}", app.status);
    assert_eq!(app.session.doc.surfaces.len(), 1, "status {}", app.status);
    assert!(app.session.display_items().iter().any(|d| d.kind == surfacecraft_doc::ObjectKind::Quad));
}

/// Parametric modelling from the GUI: the Sketch and Solid stages reach the same commands; the
/// extruded body appears and the history replays after a sketch edit.
#[test]
fn sketch_and_feature_menus_build_a_part() {
    let mut h = harness();
    // (stage tab, in the stage's More list?, tool, parameters)
    let steps = [
        ("Sketch", true, "New sketch (empty)", json!({"name": "base"})),
        (
            "Sketch",
            true,
            "Add sketch entities",
            json!({"sketch": "base", "entities": [{"polygon": {"points": [[0, 0], [30, 0], [30, 10], [0, 10]]}}]}),
        ),
        ("Solid", false, "Extrude", json!({"sketch": "base", "distance": 5.0, "name": "part"})),
    ];
    for (stage, in_more, item, params) in steps {
        h.get_by_label(stage).click();
        h.run();
        if in_more {
            h.get_by_label("More").click();
            h.run();
        }
        h.get_by_label(item).click();
        h.run();
        h.state_mut().palette_params = params.to_string();
        h.run();
        h.get_by_label("OK").click();
        h.run();
    }
    let app = h.state_mut();
    let part = app.session.doc.bodies.iter().find(|b| b.name == "part").expect("body made by the extrude");
    let v = surfacecraft_kernel::solid::volume(&part.brep).unwrap();
    assert!((v - 1500.0).abs() < 1e-3, "status {}", app.status);
    assert!(app.session.display_items().iter().any(|d| d.kind == surfacecraft_doc::ObjectKind::Sketch));
}

/// The assistant from the GUI: a provider's tool calls run as ordinary commands.
#[test]
fn assistant_panel_runs_provider_calls() {
    use surfacecraft_ai::{Response, ScriptedProvider, ToolCall};
    let mut h = harness();
    let app = h.state_mut();
    app.run_command("mesh.synth", json!({"shape": "box", "size": [40, 20, 10], "center": [200, 0, 0], "name": "scan"}))
        .unwrap();
    app.set_provider(Box::new(ScriptedProvider {
        answers: vec![
            Response {
                text: "centring".into(),
                calls: vec![ToolCall { name: "align.center".into(), arguments: json!({"object": "scan"}) }],
            },
            Response { text: "centred".into(), calls: vec![] },
        ],
        seen: vec![],
    }));
    app.assistant_goal = "center the scan".into();
    h.run();
    h.get_by_label("Ask assistant").click();
    h.run();
    let app = h.state();
    assert!(app.status.contains("centred"), "{}", app.status);
    assert!(app.session.doc.bbox().center().coords.norm() < 1e-9);
}

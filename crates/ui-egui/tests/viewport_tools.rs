//! The viewport tools of G3 through the application (no GPU): selection on a scan drawn through the
//! level of detail, face and edge picking on a body, the gizmo as one undoable command, Esc, the
//! view cube and the toolbar.
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::{Value, json};
use surfacecraft_geom::{Point3, Vec3};
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::camera::StandardView;
use surfacecraft_ui_egui::gizmo::{Gizmo, GizmoMode};
use surfacecraft_ui_egui::select::{SelectOp, View};
use surfacecraft_ui_egui::viewtools::Tool;

const W: f64 = 800.0;
const H: f64 = 600.0;

fn app_with_sphere(budget: usize) -> SurfaceCraftApp {
    let mut app = SurfaceCraftApp::headless();
    app.session.tess.budget = budget;
    app.run_command("mesh.synth", json!({"shape": "sphere", "size": [20.0], "resolution": 96})).unwrap();
    app.camera.fit(&app.session.doc.bbox());
    app.camera.set_standard(StandardView::Front);
    app
}

fn mesh_id(app: &SurfaceCraftApp) -> surfacecraft_doc::ObjectId {
    app.session.doc.meshes[0].id
}

/// Brush, lasso and smart select on a scan drawn through the level of detail: the ids are those
/// of the full mesh, they map onto fewer drawn triangles, and they reach `region.create`.
#[test]
fn selections_use_original_triangle_ids_under_lod_and_feed_region_create() {
    let mut app = app_with_sphere(600);
    let full = app.session.doc.meshes[0].mesh.triangle_count();
    let item = app.session.display_items().remove(0);
    let lod = item.lod.clone().expect("the scan is above the budget, so it is drawn coarse");
    assert!(lod.mesh.triangle_count() < full);

    app.select_brush(None, [W / 2.0, H / 2.0], W, H, false);
    let brushed = app.tools.selection.to_vec();
    assert!(brushed.len() > 20, "brushed {}", brushed.len());
    assert!(brushed.iter().all(|&t| (t as usize) < full), "ids of the full mesh");
    assert!(
        brushed.iter().any(|&t| (t as usize) >= lod.mesh.triangle_count()),
        "ids beyond the coarse mesh prove they are not display ids"
    );
    let drawn = item.drawn_triangles(&brushed);
    assert!(!drawn.is_empty() && drawn.len() < brushed.len(), "{} drawn for {} selected", drawn.len(), brushed.len());

    // erasing the same dab empties the selection; a lasso replaces, Shift-style add extends it
    app.select_brush(None, [W / 2.0, H / 2.0], W, H, true);
    assert!(app.tools.selection.is_empty());
    let poly = [[300.0, 200.0], [500.0, 200.0], [500.0, 400.0], [300.0, 400.0]];
    app.select_lasso(&poly, W, H, SelectOp::Replace);
    let lassoed = app.tools.selection.tris.len();
    assert!(lassoed > 20);
    app.select_lasso(&[[0.0, 0.0], [60.0, 0.0], [60.0, 60.0]], W, H, SelectOp::Add);
    assert!(app.tools.selection.tris.len() >= lassoed);

    // smart select on a sphere with a tight angle gives a patch, with a wide one the whole sphere
    app.tools.smart_angle = 2.0;
    assert!(app.select_smart(W / 2.0, H / 2.0, W, H, SelectOp::Replace));
    let patch = app.tools.selection.tris.len();
    assert!(patch > 0 && patch < full, "{patch} of {full}");
    app.tools.smart_angle = 80.0;
    assert!(app.select_smart(W / 2.0, H / 2.0, W, H, SelectOp::Replace));
    assert!(app.tools.selection.tris.len() * 10 > full * 9, "a sphere is smooth within 80 degrees");
    assert!(!app.select_smart(5.0, 5.0, W, H, SelectOp::Replace), "a click on the background selects nothing");

    // the selection reaches commands: the palette's `triangles` and region.create
    app.select_lasso(&poly, W, H, SelectOp::Replace);
    let expected = app.tools.selection.to_vec();
    app.choose_command("region.create");
    let params: Value = serde_json::from_str(&app.palette_params).unwrap();
    assert_eq!(params["triangles"], json!(expected));
    assert!(app.create_region_from_selection());
    let last = app.session.log.last().unwrap();
    assert_eq!(last.command, "region.create");
    assert_eq!(last.params["triangles"], json!(expected));
    assert_eq!(
        app.session.doc.summaries().iter().filter(|s| s.kind == surfacecraft_doc::ObjectKind::Region).count(),
        1
    );
}

/// A lasso over the middle of a closed scan takes only what faces the camera; through takes both
/// sides.
#[test]
fn through_selection_reaches_the_far_side() {
    let mut app = app_with_sphere(usize::MAX);
    let poly = [[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]];
    app.select_lasso(&poly, W, H, SelectOp::Replace);
    let front = app.tools.selection.tris.len();
    app.tools.through = true;
    app.select_lasso(&poly, W, H, SelectOp::Replace);
    let all = app.tools.selection.tris.len();
    assert!(front > 0 && all > front + front / 4, "front {front}, through {all}");
}

/// Dragging the X handle gives exactly one `mesh.transform` with the dragged translation and one
/// undo step; during the drag nothing has run yet; undo restores the scan.
#[test]
fn gizmo_drag_is_one_mesh_transform_and_one_undo_step() {
    let mut app = app_with_sphere(usize::MAX);
    let id = mesh_id(&app);
    app.selected_object = Some(id);
    app.tools.tool = Tool::Move;
    let before_cmds = app.session.log.len();
    let before_undo = app.session.history.undo_labels().len();
    let before_x = app.session.doc.meshes[0].mesh.bbox().center().x;

    let centre = app.session.doc.meshes[0].mesh.bbox().center();
    let len = Gizmo::new(GizmoMode::Translate, centre).handle_length(&View { camera: &app.camera, w: W, h: H });
    let px = |app: &SurfaceCraftApp, d: f64| {
        let s = app.camera.project(&(centre + Vec3::x() * d), W, H).unwrap();
        (s[0], s[1])
    };
    let (sx, sy) = px(&app, len * 0.5);
    assert!(app.gizmo_begin(sx, sy, W, H), "the press is on the X handle");
    let (ex, ey) = px(&app, len * 0.5 + 7.0);
    app.gizmo_drag(ex, ey, W, H);
    assert_eq!(app.session.log.len(), before_cmds, "a drag only previews");
    assert!(app.tools.gizmo.dragging());
    assert!(app.gizmo_end());

    assert_eq!(app.session.log.len(), before_cmds + 1, "one command for the whole drag");
    let last = app.session.log.last().unwrap();
    assert_eq!(last.command, "mesh.transform");
    let m = &last.params["matrix"];
    assert!((m[0][3].as_f64().unwrap() - 7.0).abs() < 1e-6, "{m}");
    assert!(m[1][3].as_f64().unwrap().abs() < 1e-9 && m[2][3].as_f64().unwrap().abs() < 1e-9);
    assert_eq!(app.session.history.undo_labels().len(), before_undo + 1, "one undo step");
    let after_x = app.session.doc.meshes[0].mesh.bbox().center().x;
    assert!((after_x - before_x - 7.0).abs() < 1e-6);
    app.run_command("edit.undo", Value::Null).unwrap();
    assert!((app.session.doc.meshes[0].mesh.bbox().center().x - before_x).abs() < 1e-9);
}

/// Esc abandons a drag: no command, no history entry, the mesh stays.
#[test]
fn escape_cancels_a_gizmo_drag_without_a_command() {
    let mut app = app_with_sphere(usize::MAX);
    app.selected_object = Some(mesh_id(&app));
    app.tools.tool = Tool::Rotate;
    app.camera.set_standard(StandardView::Top);
    let centre = app.session.doc.meshes[0].mesh.bbox().center();
    let len = Gizmo::new(GizmoMode::Rotate, centre).handle_length(&View { camera: &app.camera, w: W, h: H });
    // on the Z ring, a full circle in the top view
    let on = |a: f64| {
        let p = centre + (Vec3::x() * a.cos() + Vec3::y() * a.sin()) * len * 0.8;
        let s = app.camera.project(&p, W, H).unwrap();
        (s[0], s[1])
    };
    let (sx, sy) = on(0.3);
    let (ex, ey) = on(0.9);
    let cmds = app.session.log.len();
    let undo = app.session.history.undo_labels().len();
    assert!(app.gizmo_begin(sx, sy, W, H));
    app.gizmo_drag(ex, ey, W, H);
    assert!(app.tools.gizmo.dragging());
    app.cancel_interaction();
    assert!(!app.tools.gizmo.dragging());
    assert!(!app.gizmo_end(), "nothing left to apply");
    assert_eq!(app.session.log.len(), cmds);
    assert_eq!(app.session.history.undo_labels().len(), undo);
}

/// Faces and edges of a body are picked through the kernel's tessellation and their indices fill
/// the parameters of the feature commands, which accept them.
#[test]
fn picked_faces_and_edges_fill_feature_command_parameters() {
    let mut app = SurfaceCraftApp::headless();
    app.run_command("body.box", json!({"size": [20.0, 10.0, 5.0]})).unwrap();
    app.camera.fit(&app.session.doc.bbox());
    app.camera.set_standard(StandardView::Front);

    // a face
    app.tools.tool = Tool::Faces;
    assert!(app.brep_click(W / 2.0, H / 2.0, W, H, false));
    assert_eq!(app.tools.brep.faces.len(), 1);
    let item = app.session.display_items().remove(0);
    let topo = item.topo.clone().expect("a body carries its topology");
    assert!(!topo.triangles_of_faces(&app.tools.brep.face_list()).is_empty());
    assert_eq!(app.tools.brep.body, Some(item.id));
    app.choose_command("body.shell");
    let p: Value = serde_json::from_str(&app.palette_params).unwrap_or(Value::Null);
    if let Some(f) = p.get("faces") {
        assert_eq!(f, &json!(app.tools.brep.face_list()));
    }

    // an edge: the top edge of the front face, picked a little below the line
    app.tools.tool = Tool::Edges;
    app.tools.brep = Default::default();
    let s = app.camera.project(&Point3::new(10.0, 0.0, 5.0), W, H).unwrap();
    assert!(app.brep_click(s[0], s[1] + 2.0, W, H, false), "an edge is under the pointer");
    assert_eq!(app.tools.brep.edges.len(), 1);
    // a click far from any edge or face clears
    assert!(!app.brep_click(5.0, 5.0, W, H, false));
    assert!(app.tools.brep.is_empty());
    assert!(app.brep_click(s[0], s[1] + 2.0, W, H, false));

    app.choose_command("body.fillet");
    let mut params: Value = serde_json::from_str(&app.palette_params).unwrap();
    assert_eq!(params["edges"], json!(app.tools.brep.edge_list()));
    assert_eq!(params["object"], json!(item.id.0));
    params["size"] = json!(1.0);
    app.palette_params = params.to_string();
    app.run_palette();
    assert!(app.status.contains("done"), "fillet of the picked edge: {}", app.status);
}

#[test]
fn standard_views_animate_and_the_toolbar_and_cube_are_on_screen() {
    let mut app = SurfaceCraftApp::headless();
    app.run_command("mesh.synth", json!({"shape": "box", "size": [40, 30, 20], "resolution": 12})).unwrap();
    app.camera.fit(&app.session.doc.bbox());
    app.set_standard_view(StandardView::Top);
    assert!(app.tick_view_animation(0.05));
    assert!(!app.tick_view_animation(1.0));
    let (_, up, fwd) = app.camera.basis();
    assert!((fwd + Vec3::z()).norm() < 2e-3 && (up - Vec3::y()).norm() < 2e-3);

    let mut h = Harness::builder()
        .with_size((1600.0, 1000.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app);
    h.run();
    for label in ["Navigate", "Brush", "Lasso", "Smart select", "Faces", "Edges", "Move", "Rotate", "Section plane"] {
        assert!(h.query_by_label(label).is_some(), "toolbar entry {label}");
    }
    h.get_by_label("Lasso").click();
    h.run();
    assert_eq!(h.state().tools.tool, Tool::Lasso);
    h.get_by_label("Section plane").click();
    h.run();
    assert!(h.state().tools.section.enabled);
    // the cube shows the top face (the view is the top view)
    assert!(h.state().camera.pitch > 1.5);
}

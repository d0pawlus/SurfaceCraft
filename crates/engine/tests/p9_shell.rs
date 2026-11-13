//! P9 shell: a solid of constant thickness from an open, one-sided scan (`mesh.thicken`,
//! `scan.to_solid` with `thickness`). The scans are synthetic with exact volumes: a square cap of a
//! sphere, a wavy panel, a panel with a window and one with a tight bead, with scanner noise.
use serde_json::{Value, json};
use surfacecraft_engine::{EngineError, Session};
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

const RADIUS: f64 = 30.0;
const CAP_HALF: f64 = 18.0;
const CAP_CELLS: usize = 45;
const THICKNESS: f64 = 2.0;
const NOISE: f64 = 0.03;
/// Share of the exact volume the wrapped solid may differ by.
const MAX_VOLUME_ERROR: f64 = 0.03;

fn sphere_z(x: f64, y: f64) -> f64 {
    (RADIUS * RADIUS - x * x - y * y).sqrt()
}

/// Solid angle (sr) of the cone through the square cap.
fn cap_solid_angle() -> f64 {
    let n = 600;
    let h = 2.0 * CAP_HALF / n as f64;
    let mut omega = 0.0;
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (-CAP_HALF + (i as f64 + 0.5) * h, -CAP_HALF + (j as f64 + 0.5) * h);
            omega += h * h / (RADIUS * sphere_z(x, y));
        }
    }
    omega
}

fn shell_volume(r0: f64, r1: f64) -> f64 {
    (r1.powi(3) - r0.powi(3)) / 3.0 * cap_solid_angle()
}

/// Removes the triangles whose centroid lies within `radius` of one of `holes` sample points (the
/// holes are small: `mesh.clean` fills them again).
fn with_holes(m: &Mesh, holes: usize, radius: f64, rng: &mut Rng) -> Mesh {
    let centres: Vec<Point3> = m.sample_points(holes, rng).into_iter().map(|(p, _)| p).collect();
    let kept: Vec<u32> = (0..m.triangle_count() as u32)
        .filter(|&t| m.face_centroid(t as usize).is_some_and(|c| centres.iter().all(|h| (c - h).norm() > radius)))
        .collect();
    m.submesh(&kept)
}

/// A noisy square cap of a sphere with a few holes.
fn noisy_cap() -> Mesh {
    let side = 2.0 * CAP_HALF;
    let mut cap =
        synth::plane(Point3::new(-CAP_HALF, -CAP_HALF, 0.0), Vec3::x(), Vec3::y(), side, side, CAP_CELLS, CAP_CELLS);
    for p in &mut cap.positions {
        p.z = sphere_z(p.x, p.y);
    }
    let mut rng = Rng::new(7);
    let mut cap = with_holes(&cap, 3, 1.0, &mut rng);
    synth::add_noise(&mut cap, NOISE, &mut rng);
    cap
}

/// A noisy wavy panel (two sine waves, radius of curvature about 80 mm) with a few holes.
fn wavy_panel() -> Mesh {
    let mut panel = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 60.0, 40.0, 75, 50);
    for p in &mut panel.positions {
        p.z = (p.x / 9.0).sin() + 0.7 * (p.y / 7.0).cos();
    }
    let mut rng = Rng::new(11);
    let mut panel = with_holes(&panel, 2, 1.0, &mut rng);
    synth::add_noise(&mut panel, NOISE, &mut rng);
    panel
}

fn session_with(mesh: Mesh) -> Session {
    let mut s = Session::new();
    s.doc.add_mesh("scan", mesh);
    s
}

fn volume(out: &Value) -> f64 {
    out["body"]["volume"].as_f64().unwrap_or(0.0)
}

fn assert_solid_of(out: &Value, exact: f64) {
    eprintln!(
        "gate: volume {:.1} vs exact {exact:.1} ({:.2} %), within {}, coverage {}, not_facing {}, patches {}, clamped {}",
        volume(out),
        100.0 * (volume(out) - exact) / exact,
        out["deviation"]["within"],
        out["deviation"]["coverage"],
        out["deviation"]["not_facing"],
        out["patches"],
        out["thickened"]["clamped_vertices"],
    );
    assert_eq!(out["verdict"], "pass", "{out}");
    assert_eq!(out["method"], "freeform", "{out}");
    assert!(out["body"]["closed"].as_bool().unwrap_or(true), "{out}");
    let err = (volume(out) - exact).abs() / exact;
    assert!(err < MAX_VOLUME_ERROR, "volume {} vs exact {exact}: {:.2} %", volume(out), 100.0 * err);
    assert!(out["deviation"]["within"].as_f64().unwrap() >= 0.99, "{out}");
    assert!(out["deviation"]["coverage"].as_f64().unwrap() >= 0.97, "{out}");
    assert!(out["deviation"]["not_facing"].as_f64().unwrap() < 0.03, "{out}");
}

#[test]
fn gate_a_noisy_open_cap_becomes_a_solid_of_the_exact_shell_volume() {
    // Arrange
    let cap = noisy_cap();
    let mut s = session_with(cap);

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "thickness": THICKNESS})).unwrap();

    // Assert
    assert_solid_of(&out, shell_volume(RADIUS - THICKNESS, RADIUS));
    assert_eq!(out["thickened"]["self_intersections"], 0, "{out}");
    assert_eq!(out["thickened"]["self_intersections_local"], 0, "{out}");
    assert_eq!(out["thickened"]["unresolved_vertices"], 0, "{out}");
    assert_eq!(out["thickened"]["thickness"], THICKNESS, "{out}");
    assert_eq!(out["thickened"]["clamped_vertices"], 0, "{out}");
    assert!(out["thickened"]["boundary_loops"].as_u64().unwrap() >= 1, "{out}");
    assert_eq!(out["scan_report"]["holes_left"], 1, "the rim stays open: {out}");
}

#[test]
fn gate_a_wavy_open_panel_becomes_a_solid_of_thickness_times_area() {
    // Arrange
    let panel = wavy_panel();
    let area = panel.area();
    let mut s = session_with(panel);

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "thickness": THICKNESS})).unwrap();

    // Assert
    assert_solid_of(&out, THICKNESS * area);
}

#[test]
fn negative_and_symmetric_thickness_are_measured_against_the_scan_and_give_the_same_volume() {
    // Arrange
    let exact = |lo: f64, hi: f64| shell_volume(lo, hi);
    for (params, lo, hi) in [
        (json!({"thickness": -THICKNESS}), RADIUS, RADIUS + THICKNESS),
        (json!({"thickness": THICKNESS, "symmetric": true}), RADIUS - 1.0, RADIUS + 1.0),
    ] {
        let cap = noisy_cap();
        let mut s = session_with(cap);
        let mut p = json!({"scan": "scan"});
        p.as_object_mut().unwrap().extend(params.as_object().unwrap().clone());

        // Act
        let out = s.execute("scan.to_solid", p.clone()).unwrap();

        // Assert: neither the back face nor the turned normals read as missing or inside-out scan
        assert_solid_of(&out, exact(lo, hi));
        assert!(!out["issues"].to_string().contains("scan_inside_out"), "{p}: {out}");
    }
}

#[test]
fn the_scan_is_kept_the_thickened_mesh_is_removed_and_the_call_is_one_undo_step() {
    // Arrange
    let cap = noisy_cap();
    let mut s = session_with(cap);
    let scan_triangles = s.doc.meshes[0].mesh.triangle_count();
    let steps = s.history.undo_labels().len();

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "thickness": THICKNESS})).unwrap();

    // Assert
    assert_eq!(s.doc.meshes.len(), 1, "no thickened mesh is left: {out}");
    assert!(s.doc.meshes[0].mesh.triangle_count() >= scan_triangles, "the scan stays (cleaned)");
    assert_eq!(s.history.undo_labels().len(), steps + 1);
    s.execute("edit.undo", Value::Null).unwrap();
    assert!(s.doc.bodies.is_empty());
    assert_eq!(s.doc.meshes[0].mesh.triangle_count(), scan_triangles, "undo restores the raw scan");
}

#[test]
fn keep_thickened_leaves_the_thickened_mesh_in_the_document() {
    // Arrange
    let cap = noisy_cap();
    let mut s = session_with(cap);

    // Act
    let out =
        s.execute("scan.to_solid", json!({"scan": "scan", "thickness": THICKNESS, "keep_thickened": true})).unwrap();

    // Assert
    assert_eq!(s.doc.meshes.len(), 2, "{out}");
    let kept = out["thickened"]["mesh"].as_u64().expect("its id");
    assert!(s.doc.mesh(surfacecraft_doc::ObjectId(kept)).is_some());
}

#[test]
fn a_closed_scan_with_a_thickness_is_an_error_that_says_so() {
    // Arrange
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "sphere", "resolution": 24, "name": "scan"})).unwrap();

    // Act
    let r = s.execute("scan.to_solid", json!({"scan": "scan", "thickness": 1.0}));

    // Assert
    let Err(EngineError::Failed(msg) | EngineError::InvalidParams(msg)) = r else { panic!("{r:?}") };
    assert!(msg.contains("closed") && msg.contains("thickness"), "{msg}");
    assert!(s.doc.bodies.is_empty());
}

#[test]
fn thickness_with_primitives_or_reused_regions_or_alone_symmetric_is_refused() {
    let cap = noisy_cap();
    let mut s = session_with(cap);
    for p in [
        json!({"scan": "scan", "thickness": 1.0, "method": "primitives"}),
        json!({"scan": "scan", "thickness": 1.0, "reuse_regions": true}),
        json!({"scan": "scan", "symmetric": true}),
        json!({"scan": "scan", "keep_thickened": true}),
        json!({"scan": "scan", "thickness": 0.0}),
        json!({"scan": "scan", "thickness": 1e308}),
    ] {
        let r = s.execute("scan.to_solid", p.clone());
        assert!(matches!(r, Err(EngineError::InvalidParams(_) | EngineError::Failed(_))), "{p}: {r:?}");
    }
    assert!(s.doc.bodies.is_empty());
}

#[test]
fn an_open_scan_without_thickness_offers_the_thickness_as_its_first_next_step() {
    // Arrange
    let cap = noisy_cap();
    let mut s = session_with(cap);

    // Act: without `thickness` the cap is a sheet under review, as before
    let out =
        s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "fill_holes_max_edges": 10})).unwrap();

    // Assert
    assert_eq!(out["verdict"], "review", "{out}");
    let first = &out["next_steps"][0];
    assert_eq!(first["command"], "scan.to_solid", "{out}");
    assert!(first["params"]["thickness"].as_f64().is_some_and(|t| t > 0.0), "{first}");
    assert!(first["why"].as_str().unwrap().contains("placeholder"), "{first}");
    let again = s.execute("scan.to_solid", first["params"].clone()).unwrap();
    assert_eq!(again["thickened"]["thickness"], first["params"]["thickness"], "{again}");
}

/// A flat panel with a ridge along y of height 2 mm and width 1.2 mm (radius 0.72 mm at the crest).
fn bead_panel() -> Mesh {
    let mut panel = synth::plane(Point3::new(-30.0, 0.0, 0.0), Vec3::x(), Vec3::y(), 60.0, 20.0, 150, 50);
    for p in &mut panel.positions {
        p.z = 2.0 * (-(p.x * p.x) / (2.0 * 1.2 * 1.2)).exp();
    }
    panel
}

#[test]
fn mesh_thicken_makes_a_new_closed_mesh_and_reports_the_bead_it_relaxed() {
    // Arrange
    let mut s = session_with(bead_panel());

    // Act
    let out = s.execute("mesh.thicken", json!({"object": "scan", "thickness": THICKNESS, "name": "wall"})).unwrap();

    // Assert
    assert_eq!(out["closed"], true, "{out}");
    assert_eq!(out["name"], "wall");
    assert_eq!(out["boundary_loops"], 1);
    assert!(out["clamped_vertices"].as_u64().unwrap() > 0 && out["largest_reduction"].as_f64().unwrap() > 0.0, "{out}");
    assert_eq!(s.doc.meshes.len(), 2, "the scan is kept");
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(s.doc.meshes.len(), 1);
}

#[test]
fn mesh_thicken_stitches_the_window_and_the_rim_of_a_panel() {
    // Arrange: 40 x 40 mm, a 10 x 10 mm window
    let plate = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 40.0, 40.0, 40, 40);
    let keep: Vec<u32> = (0..plate.triangle_count() as u32)
        .filter(|&t| {
            let c = plate.face_centroid(t as usize).unwrap();
            !((15.0..25.0).contains(&c.x) && (15.0..25.0).contains(&c.y))
        })
        .collect();
    let mut s = session_with(plate.submesh(&keep));

    // Act
    let out = s.execute("mesh.thicken", json!({"object": "scan", "thickness": 2.0})).unwrap();

    // Assert
    assert_eq!(out["boundary_loops"], 2, "{out}");
    assert_eq!(out["closed"], true);
}

#[test]
fn mesh_thicken_refuses_closed_meshes_and_unusable_thicknesses_without_changing_the_document() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "box", "resolution": 8, "name": "box"})).unwrap();
    let cap = noisy_cap();
    s.doc.add_mesh("cap", cap);
    for p in [
        json!({"object": "box", "thickness": 1.0}),
        json!({"object": "cap", "thickness": 0.0}),
        json!({"object": "cap", "thickness": 1e300}),
        json!({"object": "cap", "thickness": -1e300, "symmetric": true}),
        json!({"object": "nope", "thickness": 1.0}),
    ] {
        let r = s.execute("mesh.thicken", p.clone());
        assert!(matches!(r, Err(EngineError::InvalidParams(_) | EngineError::Failed(_))), "{p}: {r:?}");
    }
    assert_eq!(s.doc.meshes.len(), 2);
}

#[test]
fn hostile_parameters_never_panic_on_an_open_mesh() {
    // Arrange: the commands that take a thickness, on an open mesh and with hostile values
    let hostile = [
        json!(null),
        json!({"thickness": 1e308}),
        json!({"thickness": -1e308, "symmetric": true}),
        json!({"thickness": 1e-300}),
        json!({"thickness": 5e-324, "symmetric": true}),
        json!({"thickness": 0.0}),
        json!({"thickness": "2"}),
        json!({"thickness": [2.0]}),
        json!({"thickness": 1e9, "keep_thickened": true}),
        json!({"thickness": 1.0, "symmetric": 3}),
        json!({"thickness": 1.0, "tolerance": 1e308, "max_patches": 0}),
    ];
    for (command, key) in [("mesh.thicken", "object"), ("scan.to_solid", "scan")] {
        for h in &hostile {
            let mut s = session_with(bead_panel());
            let mut p = json!({ key: "scan" });
            if let Some(extra) = h.as_object() {
                p.as_object_mut().unwrap().extend(extra.clone());
            }
            // Act
            let r = s.execute(command, p.clone());

            // Assert
            assert!(!matches!(r, Err(EngineError::Internal(_))), "{command} {p} panicked");
        }
    }
}

#[test]
fn a_shell_that_misses_the_tolerance_does_not_offer_to_fill_the_rim_or_drop_the_thickness() {
    // Arrange: a patch limit far too small for the shell, so the run has an issue
    let cap = noisy_cap();
    let mut s = session_with(cap);

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "thickness": THICKNESS, "max_patches": 24})).unwrap();

    // Assert: the rim left open on purpose is no scan defect
    let codes: Vec<&str> = out["issues"].as_array().unwrap().iter().map(|i| i["code"].as_str().unwrap()).collect();
    assert!(codes.contains(&"out_of_tolerance"), "{out}");
    assert!(!codes.contains(&"scan_defects"), "{out}");
    assert_eq!(out["scan_report"]["holes_left"], 1, "the true number stays in the report: {out}");
    let steps = out["next_steps"].as_array().unwrap();
    assert!(
        steps.iter().any(|st| st["command"] == "scan.to_solid"
            && st["params"]["keep_thickened"] == true
            && st["params"]["thickness"] == THICKNESS
            && st["params"]["max_patches"].as_u64().is_some_and(|m| m > 24)),
        "a larger patch limit with the same thickness: {out}"
    );
    for st in steps {
        assert_ne!(st["command"], "surface.wrap", "that would wrap the open scan, not the shell: {st}");
        assert_ne!(st["command"], "mesh.fill_holes", "{st}");
        if st["command"] == "scan.to_solid" {
            assert!(st["params"]["thickness"].is_number(), "{st}");
        }
    }
}

/// A flat 40 x 40 mm panel of 1 mm cells with a 10 x 10 mm window.
fn panel_with_window() -> Mesh {
    let plate = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 40.0, 40.0, 40, 40);
    let keep: Vec<u32> = (0..plate.triangle_count() as u32)
        .filter(|&t| {
            let c = plate.face_centroid(t as usize).unwrap();
            !((15.0..25.0).contains(&c.x) && (15.0..25.0).contains(&c.y))
        })
        .collect();
    plate.submesh(&keep)
}

#[test]
fn a_window_that_was_not_filled_is_reported_not_silenced() {
    // Arrange: the window is a hole of 40 edges; filling is switched off
    let mut s = session_with(panel_with_window());

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "thickness": 2.0, "fill_holes_max_edges": 0})).unwrap();

    // Assert
    let codes: Vec<&str> = out["issues"].as_array().unwrap().iter().map(|i| i["code"].as_str().unwrap()).collect();
    assert!(codes.contains(&"windows_stitched"), "{out}");
    assert_ne!(out["verdict"], "pass", "{out}");
    assert_eq!(out["thickened"]["windows"], 1, "{out}");
    assert_eq!(out["thickened"]["rim_edges"], 160, "{out}");
    let fill = out["next_steps"].as_array().unwrap().iter().find(|st| st["params"]["fill_holes_max_edges"] == 159);
    assert!(fill.is_some(), "a step that fills all but the rim: {out}");
}

#[test]
fn the_rim_is_chosen_after_the_cleaning_so_debris_with_a_longer_loop_does_not_win() {
    // Arrange: a 4 x 4 panel (rim of 16 edges) and a debris fan of 18 triangles (loop of 18 edges)
    let mut panel = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 4.0, 4.0, 4, 4);
    let centre = panel.positions.len() as u32;
    panel.positions.push(Point3::new(50.0, 50.0, 0.0));
    for k in 0..18u32 {
        let a = std::f64::consts::TAU * f64::from(k) / 18.0;
        panel.positions.push(Point3::new(50.0 + a.cos(), 50.0 + a.sin(), 0.0));
    }
    for k in 0..18u32 {
        panel.triangles.push([centre, centre + 1 + k, centre + 1 + (k + 1) % 18]);
    }
    let mut s = session_with(panel);

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "thickness": 1.0})).unwrap();

    // Assert: the debris is gone, the rim stayed open and was thickened
    assert_eq!(out["thickened"]["boundary_loops"], 1, "{out}");
    assert_eq!(out["thickened"]["rim_edges"], 16, "{out}");
}

#[test]
fn a_spike_on_the_rim_does_not_make_a_window_or_close_the_scan() {
    // Arrange
    let mut cap = noisy_cap();
    let rim_vertex =
        (0..cap.vertex_count()).min_by(|&a, &b| cap.positions[a].x.total_cmp(&cap.positions[b].x)).unwrap();
    cap.positions[rim_vertex].z += 15.0;
    let mut s = session_with(cap);

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "thickness": THICKNESS})).unwrap();

    // Assert
    assert!(out["thickened"].is_object(), "{out}");
    assert_eq!(out["thickened"]["windows"], 0, "{out}");
}

#[test]
fn thickness_with_parts_or_several_co_equal_pieces_is_refused_and_one_piece_is_unchanged() {
    // Arrange: two co-equal panels
    let mut two = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 20.0, 20.0, 20, 20);
    let second = synth::plane(Point3::new(100.0, 0.0, 0.0), Vec3::x(), Vec3::y(), 20.0, 20.0, 20, 20);
    two.append(&second);
    let mut s = session_with(two);

    // Act and assert: explicit parameters of the split and several co-equal pieces are refused
    for p in [
        json!({"scan": "scan", "thickness": 1.0, "components": "largest"}),
        json!({"scan": "scan", "thickness": 1.0, "part_share": 0.5}),
        json!({"scan": "scan", "thickness": 1.0}),
    ] {
        let r = s.execute("scan.to_solid", p.clone());
        let Err(EngineError::Failed(m) | EngineError::InvalidParams(m)) = r else { panic!("{p}: {r:?}") };
        assert!(m.contains("scan.split"), "{m}");
    }
    assert!(s.doc.bodies.is_empty() && s.doc.meshes.len() == 1);

    // a single open piece goes through the preparation stage unchanged: one thickened solid, no parts
    let mut one = session_with(panel_with_window());
    let out = one.execute("scan.to_solid", json!({"scan": "scan", "thickness": 1.0, "rebuild": false})).unwrap();
    assert!(out["thickened"].is_object() && out["prep"]["components"].as_array().is_none_or(Vec::is_empty), "{out}");
}

/// A U-shaped channel 3 mm wide and 30 mm deep as an open sheet whose normals point out of it.
fn narrow_channel() -> Mesh {
    let path = [(-1.5, 30.0), (-1.5, 0.0), (1.5, 0.0), (1.5, 30.0)];
    let (steps, rows) = (60usize, 67usize);
    let mut positions = Vec::new();
    for j in 0..=rows {
        let y = 20.0 * j as f64 / rows as f64;
        for k in 0..(path.len() - 1) {
            let ((x0, z0), (x1, z1)) = (path[k], path[k + 1]);
            let n = if k == 1 { 10 } else { steps };
            for i in 0..=n {
                if k > 0 && i == 0 {
                    continue;
                }
                let f = i as f64 / n as f64;
                positions.push(Point3::new(x0 + (x1 - x0) * f, y, z0 + (z1 - z0) * f));
            }
        }
    }
    let per_row = positions.len() / (rows + 1);
    let mut triangles = Vec::new();
    for j in 0..rows {
        for i in 0..per_row - 1 {
            let (a, b) = ((j * per_row + i) as u32, (j * per_row + i + 1) as u32);
            let (c, d) = (((j + 1) * per_row + i + 1) as u32, ((j + 1) * per_row + i) as u32);
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    let mut m = Mesh { positions, triangles };
    if m.vertex_normals()[per_row / 6].x > 0.0 {
        for t in &mut m.triangles {
            t.swap(1, 2);
        }
    }
    m
}

#[test]
fn a_channel_narrower_than_the_wall_is_reported_as_crossing_itself_and_caps_the_verdict() {
    // Arrange
    let mut s = session_with(narrow_channel());

    // Act
    let thick = s.execute("mesh.thicken", json!({"object": "scan", "thickness": 2.0})).unwrap();
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "thickness": 2.0})).unwrap();

    // Assert
    assert_eq!(thick["self_intersection_checked"], true, "{thick}");
    assert!(thick["self_intersections"].as_u64().unwrap() > 0, "{thick}");
    assert_eq!(thick["self_intersections_local"], 0, "a channel is real geometry, not a fold: {thick}");
    assert_eq!(out["thickened"]["self_intersection_checked"], true);
    let codes: Vec<&str> = out["issues"].as_array().unwrap().iter().map(|i| i["code"].as_str().unwrap()).collect();
    assert!(codes.contains(&"self_intersection") && out["verdict"] != "pass", "{out}");
}

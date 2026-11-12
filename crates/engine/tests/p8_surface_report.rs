//! P8-H11 gates: what `surface.fit` says about its own measurement.
//!
//! - Exclusion masks: a wavy patch (sigma = 0.02 mm) with a 3 mm boss over 5 % of its area. Without
//!   a mask the boss spoils the fit (max deviation far above 0.2 mm); with a sphere mask over it,
//!   or the equivalent triangle list, the max over the points that are not excluded is within
//!   0.2 mm, the source region keeps all its triangles and the counters add up. A triangle is in a
//!   sphere when its centroid is.
//! - Coverage: a patch with a round hole, the region an ellipse smaller than the fitted rectangle:
//!   hole cells are `H`, cells outside the outline `E`, and the shares match the analytic areas
//!   within 3 percentage points.
//! - Counters: eligible = measured + excluded + failed, everywhere.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Point3, Rng, Vec3, synth};
use surfacecraft_kernel::tess;

const WIDTH: f64 = 60.0;
const HEIGHT: f64 = 40.0;
const AMPLITUDE: f64 = 3.0;
const BOSS_HEIGHT: f64 = 3.0;
/// A boss of 5 % of the patch's area: pi r^2 = 0.05 * 60 * 40.
const BOSS_RADIUS: f64 = 6.18;
const BOSS_CENTRE: (f64, f64) = (42.0, 22.0);
const TOLERANCE: f64 = 0.2;

fn centroid(m: &Mesh, t: usize) -> Point3 {
    let [a, b, c] = m.triangle(t).unwrap();
    Point3::from((a.coords + b.coords + c.coords) / 3.0)
}

fn on_boss(p: &Point3) -> bool {
    (p.x - BOSS_CENTRE.0).hypot(p.y - BOSS_CENTRE.1) <= BOSS_RADIUS
}

/// The wavy patch with the boss, 90 x 60 vertices, and the triangles of the boss's skirt.
fn bossed() -> Mesh {
    let mut m = synth::wave(Point3::origin(), WIDTH, HEIGHT, AMPLITUDE, 90, 60);
    synth::add_noise(&mut m, 0.02, &mut Rng::new(11));
    for p in &mut m.positions {
        if on_boss(p) {
            p.z += BOSS_HEIGHT;
        }
    }
    m
}

fn scene(mesh: Mesh, ids: &[u32]) -> Session {
    let mut s = Session::new();
    s.doc.add_mesh("scan", mesh);
    s.execute("region.create", json!({"mesh": "scan", "triangles": ids, "name": "free"})).unwrap();
    s
}

fn scan(s: &Session) -> std::sync::Arc<Mesh> {
    s.doc.mesh(s.doc.find_by_name("scan").unwrap().0).unwrap().mesh.clone()
}

fn all_triangles(m: &Mesh) -> Vec<u32> {
    (0..m.triangle_count() as u32).collect()
}

/// Radius of the sphere around the boss that holds the centroid of every triangle touching a
/// raised vertex (so no raised vertex is left in a kept triangle), and its centre.
fn boss_sphere(m: &Mesh) -> ([f64; 3], f64) {
    let raised: Vec<bool> = m.positions.iter().map(on_boss).collect();
    let z = synth::wave_height(AMPLITUDE, BOSS_CENTRE.0, BOSS_CENTRE.1) + BOSS_HEIGHT / 2.0;
    let centre = Point3::new(BOSS_CENTRE.0, BOSS_CENTRE.1, z);
    let radius = (0..m.triangle_count())
        .filter(|&t| m.triangles[t].iter().any(|&v| raised[v as usize]))
        .map(|t| (centroid(m, t) - centre).norm())
        .fold(0.0, f64::max);
    ([centre.x, centre.y, centre.z], radius + 0.01)
}

fn centroid_in_sphere(m: &Mesh, c: [f64; 3], r: f64) -> Vec<u32> {
    let c = Point3::new(c[0], c[1], c[2]);
    (0..m.triangle_count()).filter(|&t| (centroid(m, t) - c).norm() <= r).map(|t| t as u32).collect()
}

/// Distinct vertices of the triangles `ids`.
fn vertices(m: &Mesh, ids: &[u32]) -> Vec<usize> {
    let mut v: Vec<usize> = ids.iter().flat_map(|&t| m.triangles[t as usize]).map(|v| v as usize).collect();
    v.sort_unstable();
    v.dedup();
    v
}

fn counters(r: &Value) -> [u64; 4] {
    let s = &r["samples"];
    ["eligible", "measured", "excluded", "failed"].map(|k| s[k].as_u64().unwrap_or_else(|| panic!("{k} in {r}")))
}

fn assert_counters_add_up(r: &Value) {
    let [eligible, measured, excluded, failed] = counters(r);
    assert_eq!(eligible, measured + excluded + failed, "{}", r["samples"]);
    assert_eq!(r["measured"].as_u64().unwrap(), measured, "top-level measured is samples.measured");
}

/// Largest distance from the vertices `ids` to the delivered sheet `id`, tessellated finely.
fn max_to_sheet(s: &Session, m: &Mesh, ids: &[u32], id: &Value) -> f64 {
    let o = s.doc.surfaces.iter().find(|o| o.id.0 == id.as_u64().unwrap()).unwrap();
    let bvh = Bvh::build(&tess::tessellate(&o.sheet, Some(0.01)).unwrap());
    vertices(m, ids)
        .iter()
        .filter_map(|&v| bvh.closest(&m.positions[v], f64::INFINITY))
        .map(|h| h.distance)
        .fold(0.0, f64::max)
}

#[test]
fn gate_h11_sphere_mask_removes_the_boss_from_the_fit() {
    let m = bossed();
    let ids = all_triangles(&m);
    let mut s = scene(m.clone(), &ids);
    let plain = s.execute("surface.fit", json!({"region": "free", "name": "plain"})).unwrap();
    let (centre, radius) = boss_sphere(&m);
    let masked = s
        .execute(
            "surface.fit",
            json!({"region": "free", "name": "masked", "exclude_spheres": [{"center": centre, "radius": radius}]}),
        )
        .unwrap();
    eprintln!(
        "h11 sphere: plain max {} rms {}; masked max {} rms {}",
        plain["max"], plain["rms"], masked["max"], masked["rms"]
    );
    eprintln!("h11 sphere: radius {radius:.2}, samples {}, exclusion {}", masked["samples"], masked["exclusion"]);
    // the boss pulls the unmasked surface
    assert!(plain["max"].as_f64().unwrap() > TOLERANCE, "unmasked max {}", plain["max"]);
    assert!(plain.get("exclusion").is_none(), "no mask, no exclusion block: {plain}");
    assert_eq!(counters(&plain)[2], 0);
    assert_counters_add_up(&plain);
    // masked: within 0.2 mm over the points that are not excluded
    assert!(masked["max"].as_f64().unwrap() <= TOLERANCE, "masked max {}", masked["max"]);
    assert_eq!(masked["within_tolerance"], true, "{masked}");
    let inside = centroid_in_sphere(&m, centre, radius);
    let kept: Vec<u32> = ids.iter().copied().filter(|t| !inside.contains(t)).collect();
    let independent = max_to_sheet(&s, &m, &kept, &masked["id"]);
    eprintln!("h11 sphere: kept vertices -> sheet max {independent:.4} mm");
    assert!(independent <= TOLERANCE + 0.02, "independent max {independent}");
    // the excluded triangles are exactly those whose centroid is inside the sphere
    let ex = &masked["exclusion"];
    assert_eq!(ex["triangles"].as_u64().unwrap() as usize, inside.len(), "{ex}");
    assert_eq!(ex["by_volume"].as_u64().unwrap() as usize, inside.len());
    assert_eq!(ex["by_index"], 0);
    assert_eq!(ex["spheres"][0]["radius"].as_f64().unwrap(), radius, "the masks are echoed: {ex}");
    // points: those whose triangles are all inside
    let [eligible, measured, excluded, failed] = counters(&masked);
    let kept_points = vertices(&m, &kept).len();
    assert_eq!(eligible as usize, vertices(&m, &ids).len());
    assert_eq!(measured as usize, kept_points, "{}", masked["samples"]);
    assert_eq!((excluded as usize, failed), (eligible as usize - kept_points, 0));
    assert_eq!(ex["points"].as_u64().unwrap(), excluded);
    assert_counters_add_up(&masked);
    assert!(excluded > 100, "a boss of about 5 % has points: {excluded}");
    // the source is untouched
    let region = s.doc.region(s.doc.find_by_name("free").unwrap().0).unwrap();
    assert_eq!(region.triangles.len(), ids.len());
    assert_eq!(scan(&s).triangle_count(), ids.len());
    assert!(masked["note"].as_str().unwrap().contains("excluded by mask"), "{masked}");
}

#[test]
fn gate_h11_triangle_mask_equals_the_equivalent_sphere() {
    let m = bossed();
    let ids = all_triangles(&m);
    let mut s = scene(m.clone(), &ids);
    let (centre, radius) = boss_sphere(&m);
    let inside = centroid_in_sphere(&m, centre, radius);
    let by_sphere = s
        .execute("surface.fit", json!({"region": "free", "exclude_spheres": [{"center": centre, "radius": radius}]}))
        .unwrap();
    let by_index = s.execute("surface.fit", json!({"region": "free", "exclude_triangles": inside})).unwrap();
    for key in ["max", "rms", "control", "samples", "surface_deviation", "measured", "within_tolerance"] {
        assert_eq!(by_sphere[key], by_index[key], "{key}: {} vs {}", by_sphere[key], by_index[key]);
    }
    let ex = &by_index["exclusion"];
    assert_eq!((ex["by_index"].as_u64().unwrap() as usize, ex["by_volume"].as_u64().unwrap()), (inside.len(), 0));
    assert_eq!(ex["indices_given"].as_u64().unwrap() as usize, inside.len());
    assert_eq!(ex["indices"].as_array().unwrap().len(), 100, "only the first indices are echoed");
    assert_counters_add_up(&by_index);
    // a box over the same triangles, big enough to hold them
    let bx = s
        .execute(
            "surface.fit",
            json!({"region": "free", "exclude_boxes": [{"min": [35.0, 15.0, -20.0], "max": [49.0, 29.0, 20.0]}]}),
        )
        .unwrap();
    assert!(bx["max"].as_f64().unwrap() <= TOLERANCE, "{}", bx["max"]);
    assert_counters_add_up(&bx);
}

#[test]
fn indices_outside_the_region_are_ignored_and_counted() {
    let m = bossed();
    let half: Vec<u32> = (0..m.triangle_count() as u32).filter(|&t| centroid(&m, t as usize).x < 30.0).collect();
    let mut s = scene(m.clone(), &half);
    let outside: Vec<u32> =
        (0..m.triangle_count() as u32).filter(|&t| centroid(&m, t as usize).x > 50.0).take(7).collect();
    let r = s.execute("surface.fit", json!({"region": "free", "exclude_triangles": outside})).unwrap();
    assert_eq!(r["exclusion"]["indices_ignored"], 7, "{r}");
    assert_eq!(r["exclusion"]["triangles"], 0);
    assert_eq!(counters(&r)[2], 0);
    assert_counters_add_up(&r);
}

#[test]
fn hostile_masks_are_errors_never_panics() {
    let m = bossed();
    let ids = all_triangles(&m);
    let mut s = scene(m.clone(), &ids);
    let n = m.triangle_count() as u64;
    let bad = [
        json!({"exclude_triangles": [n]}),
        json!({"exclude_triangles": [u32::MAX]}),
        json!({"exclude_spheres": [{"center": [0, 0, 0], "radius": -1.0}]}),
        json!({"exclude_spheres": [{"center": [0, 0, null], "radius": 1.0}]}),
        json!({"exclude_spheres": [{"center": [0, 0, 0], "radius": f64::MAX}]}),
        json!({"exclude_spheres": vec![json!({"center": [0, 0, 0], "radius": 1.0}); 300]}),
        json!({"exclude_boxes": [{"min": [1, 0, 0], "max": [0, 1, 1]}]}),
        json!({"exclude_boxes": [{"min": [0, 0, 0]}]}),
        json!({"exclude_triangles": "all"}),
        // everything excluded: nothing is left to fit
        json!({"exclude_boxes": [{"min": [-1e9, -1e9, -1e9], "max": [1e9, 1e9, 1e9]}]}),
        json!({"exclude_spheres": [{"center": [1e308, 1e308, 1e308], "radius": 1.7e308}], "exclude_triangles": ids}),
    ];
    for extra in bad {
        let mut p = json!({"region": "free"});
        p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
        assert!(s.execute("surface.fit", p).is_err(), "{extra}");
    }
    // a mask that excludes nothing is a mask all the same
    let r = s.execute("surface.fit", json!({"region": "free", "exclude_triangles": []})).unwrap();
    assert!(r.get("exclusion").is_none());
    let r = s
        .execute("surface.fit", json!({"region": "free", "exclude_spheres": [{"center": [1e6, 0, 0], "radius": 1.0}]}))
        .unwrap();
    assert_eq!(r["exclusion"]["triangles"], 0);
    assert_eq!(r["samples"]["excluded"], 0);
    assert_counters_add_up(&r);
}

/// The region of the coverage scene: an ellipse with a round hole (by triangle centroid).
fn in_region(x: f64, y: f64) -> bool {
    let ellipse = ((x - 30.0) / 27.0).powi(2) + ((y - 20.0) / 17.0).powi(2) <= 1.0;
    ellipse && (x - 22.0).hypot(y - 18.0) > 5.0
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[test]
fn gate_h11_coverage_marks_holes_and_extrapolation() {
    let mut m = synth::wave(Point3::origin(), WIDTH, HEIGHT, AMPLITUDE, 90, 60);
    synth::add_noise(&mut m, 0.02, &mut Rng::new(5));
    let ids: Vec<u32> = (0..m.triangle_count() as u32)
        .filter(|&t| {
            let c = centroid(&m, t as usize);
            in_region(c.x, c.y)
        })
        .collect();
    let mut s = scene(m.clone(), &ids);
    let r = s.execute("surface.fit", json!({"region": "free"})).unwrap();
    let cov = &r["coverage"];
    eprintln!("h11 coverage: shares {}, grid {}x{}", cov["shares"], cov["cols"], cov["rows"]);
    for row in cov["grid"].as_array().unwrap() {
        eprintln!("h11 coverage: {}", row.as_str().unwrap());
    }
    let (cols, rows) = (cov["cols"].as_u64().unwrap() as usize, cov["rows"].as_u64().unwrap() as usize);
    assert!(cols <= 32 && rows <= 32 && cols >= 16 && rows >= 16, "{cols} x {rows}");
    let grid: Vec<&str> = cov["grid"].as_array().unwrap().iter().map(|r| r.as_str().unwrap()).collect();
    assert_eq!(grid.len(), rows);
    assert!(grid.iter().all(|g| g.len() == cols && g.chars().all(|c| "MHE".contains(c))));
    assert!(cov.to_string().len() < 2500, "the coverage is small: {} bytes", cov.to_string().len());
    // analytic shares on the domain rectangle of the fit (the frame is in the result)
    let vec3 = |k: &str| -> [f64; 3] { serde_json::from_value(cov[k].clone()).unwrap() };
    let (o, u, v) = (vec3("origin"), vec3("u_axis"), vec3("v_axis"));
    let (ur, vr): ([f64; 2], [f64; 2]) = (
        serde_json::from_value(cov["u_range"].clone()).unwrap(),
        serde_json::from_value(cov["v_range"].clone()).unwrap(),
    );
    let at = |x: f64, y: f64| (o[0] + u[0] * x + v[0] * y, o[1] + u[1] * x + v[1] * y);
    let n = 600;
    let mut count = [0usize; 3];
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (
                ur[0] + (ur[1] - ur[0]) * (i as f64 + 0.5) / n as f64,
                vr[0] + (vr[1] - vr[0]) * (j as f64 + 0.5) / n as f64,
            );
            let (px, py) = at(x, y);
            let ellipse = ((px - 30.0) / 27.0).powi(2) + ((py - 20.0) / 17.0).powi(2) <= 1.0;
            count[if in_region(px, py) {
                0
            } else if ellipse {
                1
            } else {
                2
            }] += 1;
        }
    }
    let truth = count.map(|c| c as f64 / (n * n) as f64);
    let got = ["measured", "hole", "extrapolated"].map(|k| cov["shares"][k].as_f64().unwrap());
    eprintln!("h11 coverage: shares {got:.3?} vs analytic {truth:.3?}");
    for k in 0..3 {
        assert!((got[k] - truth[k]).abs() < 0.03, "class {k}: {} vs analytic {}", got[k], truth[k]);
    }
    assert!((got.iter().sum::<f64>() - 1.0).abs() < 1e-9);
    // the cell under the hole's centre is H, the cell at a corner of the rectangle E, one in the data M
    let cell = |p: [f64; 3]| {
        let d = [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
        let (x, y) = (dot(d, u), dot(d, v));
        let i = (((x - ur[0]) / (ur[1] - ur[0])) * cols as f64).floor() as usize;
        let j = (((y - vr[0]) / (vr[1] - vr[0])) * rows as f64).floor() as usize;
        grid[j.min(rows - 1)].as_bytes()[i.min(cols - 1)] as char
    };
    let on = |x: f64, y: f64| [x, y, synth::wave_height(AMPLITUDE, x, y)];
    assert_eq!(cell(on(22.0, 18.0)), 'H', "hole centre");
    assert_eq!(cell(on(30.0, 30.0)), 'M', "data");
    assert_eq!(cell(on(4.0, 34.0)), 'E', "outside the outline");
    // sheet shares per piece are not given for one piece; the counters still add up
    assert_counters_add_up(&r);
    assert_eq!(counters(&r)[2], 0);
}

#[test]
fn the_excluded_bump_is_a_hole_in_the_coverage() {
    let m = bossed();
    let ids = all_triangles(&m);
    let mut s = scene(m.clone(), &ids);
    let (centre, radius) = boss_sphere(&m);
    let plain = s.execute("surface.fit", json!({"region": "free"})).unwrap();
    let masked = s
        .execute("surface.fit", json!({"region": "free", "exclude_spheres": [{"center": centre, "radius": radius}]}))
        .unwrap();
    assert!(plain["coverage"]["shares"]["hole"].as_f64().unwrap() < 0.005, "{}", plain["coverage"]["shares"]);
    let hole = masked["coverage"]["shares"]["hole"].as_f64().unwrap();
    eprintln!("h11 bump: hole share {hole:.3}");
    assert!(hole > 0.04, "the excluded boss and its skirt are not measured: {hole}");
}

/// A split fit: the counters add up over several pieces (a shared vertex is counted once).
#[test]
fn counters_add_up_over_split_pieces() {
    let m = synth::cylinder(Point3::origin(), Vec3::z(), 20.0, 40.0, 90, 40);
    let ids: Vec<u32> = (0..m.triangle_count() as u32)
        .filter(|&t| {
            let c = centroid(&m, t as usize);
            c.y.atan2(c.x).abs() <= 120f64.to_radians()
        })
        .collect();
    let mut s = scene(m, &ids);
    let mask = json!([{"min": [10.0, -3.0, 10.0], "max": [25.0, 3.0, 14.0]}]);
    let r = s.execute("surface.fit", json!({"region": "free", "split": true, "exclude_boxes": mask})).unwrap();
    eprintln!("h11 split: {} pieces, samples {}", r["pieces"].as_array().map_or(1, Vec::len), r["samples"]);
    assert!(r["pieces"].as_array().is_some_and(|p| p.len() > 1), "{r}");
    assert_counters_add_up(&r);
    assert!(r["pieces"].as_array().unwrap().iter().all(|p| p["coverage"].is_object()));
}

/// Finite but huge parameters must not wrap round into "nothing is inside": a sphere of radius
/// 2e200 around [1e200, 0, 0] holds the whole mesh, so nothing is left to fit.
#[test]
fn huge_finite_volumes_still_exclude() {
    let m = bossed();
    let mut s = scene(m.clone(), &all_triangles(&m));
    for mask in [
        json!({"exclude_spheres": [{"center": [1e200, 0.0, 0.0], "radius": 2e200}]}),
        json!({"exclude_spheres": [{"center": [-1e200, 1e200, 1e200], "radius": f64::MAX}]}),
        json!({"exclude_boxes": [{"min": [-f64::MAX, -f64::MAX, -f64::MAX], "max": [f64::MAX, f64::MAX, f64::MAX]}]}),
    ] {
        let mut p = json!({"region": "free"});
        p.as_object_mut().unwrap().extend(mask.as_object().unwrap().clone());
        let e = s.execute("surface.fit", p).unwrap_err().to_string();
        assert!(e.contains("too few points"), "{mask}: {e}");
    }
}

/// Boundary samples a mask excludes are in no deviation number either: a 4 mm bump along a
/// diagonal stretch of an elliptical region makes the surface (and with it the closest points of
/// the boundary vertices) bulge, so the unmasked outline deviation is large; masking the stretch
/// removes those vertices from the measurement while the trim outline still follows the whole
/// region (same corners, area within 1 %). The exact filter is unit-tested in `surface_mask`: this
/// scene is the behaviour check (it passed before the filter too, the value being small once the
/// surface ignores the bump).
#[test]
fn excluded_boundary_points_do_not_enter_the_outline_deviation() {
    let ellipse = |x: f64, y: f64| ((x - 30.0) / 27.0).powi(2) + ((y - 20.0) / 17.0).powi(2);
    let mut m = synth::wave(Point3::origin(), WIDTH, HEIGHT, AMPLITUDE, 90, 60);
    synth::add_noise(&mut m, 0.02, &mut Rng::new(3));
    for p in &mut m.positions {
        if p.x > 40.0 && p.y > 28.0 && ellipse(p.x, p.y) > 0.8 {
            p.z += 4.0;
        }
    }
    let ids: Vec<u32> = (0..m.triangle_count() as u32)
        .filter(|&t| {
            let c = centroid(&m, t as usize);
            ellipse(c.x, c.y) <= 1.0
        })
        .collect();
    let mut s = scene(m.clone(), &ids);
    let plain = s.execute("surface.fit", json!({"region": "free", "trim": true})).unwrap();
    let mask = json!([{"min": [39.0, 27.0, -1e6], "max": [1e6, 1e6, 1e6]}]);
    let masked = s.execute("surface.fit", json!({"region": "free", "trim": true, "exclude_boxes": mask})).unwrap();
    let (a, b) = (plain["outline_deviation"].as_f64().unwrap(), masked["outline_deviation"].as_f64().unwrap());
    eprintln!("h11 outline: unmasked {a:.4} mm, masked {b:.4} mm; trimmed {}", masked["trimmed"]);
    assert_eq!(masked["trimmed"], true, "{masked}");
    assert!(a > 0.2, "the raised vertices spoil the unmasked outline deviation: {a}");
    assert!(b <= 0.1, "masked outline deviation {b}");
    // the trim follows the whole region: same outline loops and the same area as unmasked
    assert_eq!(masked["outline"]["corners"], plain["outline"]["corners"]);
    let (area_a, area_b) = (plain["outline"]["area"].as_f64().unwrap(), masked["outline"]["area"].as_f64().unwrap());
    assert!((area_a - area_b).abs() < 0.01 * area_a, "outline areas {area_a} vs {area_b} (each in its own fit plane)");
}

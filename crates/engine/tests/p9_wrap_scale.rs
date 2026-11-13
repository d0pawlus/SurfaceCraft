//! Cost of big patch networks: time of `surface.wrap`, `body.compare`, STEP export and STEP
//! re-import on a synthetic closed shape wrapped in `SURFACECRAFT_SCALE_PATCHES` patches (default
//! 30000). Measurement only, nothing is asserted; ignored by default.
//!
//! Run (release; peak memory from `/usr/bin/time -l`, one size per process):
//! `SURFACECRAFT_SCALE_PATCHES=60000 /usr/bin/time -l cargo test --release -p surfacecraft-engine --test p9_wrap_scale -- --ignored --nocapture`
use serde_json::json;
use std::time::Instant;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Vec3, synth};
use surfacecraft_kernel::tess::tessellate_topo;

const RADIUS: f64 = 20.0;
/// Ripples fine enough that the coarse mesh keeps tens of thousands of triangles at this tolerance.
const AMPLITUDE: f64 = 0.2;
const PERIOD: f64 = 2.0;
const TOLERANCE: f64 = 0.005;
/// The tolerance of the comparison, as in `scan.to_solid` (its chord is a twentieth of it).
const COMPARE_TOLERANCE: f64 = 0.2;

fn rippled_sphere() -> Mesh {
    let k = std::f64::consts::TAU * RADIUS / PERIOD;
    let mut m = synth::sphere(Point3::origin(), 1.0, 700, 350);
    for p in &mut m.positions {
        let d: Vec3 = p.coords;
        let wave = (k * d.x + 0.3).sin() * (k * d.y + 0.7).sin() * (0.5 * k * d.z).cos();
        p.coords = d * (RADIUS + AMPLITUDE * wave);
    }
    m
}

fn seconds(since: Instant) -> f64 {
    since.elapsed().as_secs_f64()
}

#[test]
#[ignore = "measurement, slow"]
fn big_network_costs() {
    let cap: usize = std::env::var("SURFACECRAFT_SCALE_PATCHES").ok().and_then(|v| v.parse().ok()).unwrap_or(30_000);
    let mut s = Session::new();
    s.doc.add_mesh("scan", rippled_sphere());
    let t = Instant::now();
    let w = s
        .execute("surface.wrap", json!({"mesh": "scan", "tolerance": TOLERANCE, "max_patches": cap, "min_within": 1.0}))
        .unwrap();
    eprintln!(
        "SCALE cap {cap}: wrap {:.1} s patches {} within {:.4} capped {} rounds {} closed {}",
        seconds(t),
        w["patches"],
        w["deviation"]["within"],
        w["capped"],
        w["rounds"],
        w["closed"]
    );
    if std::env::var("SURFACECRAFT_SCALE_WRAP_ONLY").is_ok() {
        return;
    }
    let body = w["body"]["id"].clone();
    let id = surfacecraft_doc::ObjectId(body.as_u64().unwrap());
    let brep = s.doc.body(id).unwrap().brep.clone();
    let t = Instant::now();
    let info = surfacecraft_kernel::solid::info(&brep);
    eprintln!("SCALE   solid::info {:.1} s: volume {:?}", seconds(t), info.volume);
    let t = Instant::now();
    let topo = tessellate_topo(&brep, Some(0.01)).unwrap();
    eprintln!("SCALE   tessellate (chord 0.01) {:.1} s: {} triangles", seconds(t), topo.mesh.triangle_count());
    let t = Instant::now();
    let bvh = surfacecraft_geom::Bvh::build(&topo.mesh);
    eprintln!("SCALE   bvh over the tessellation {:.1} s ({} triangles)", seconds(t), bvh.len());
    let scan = s.doc.mesh(s.doc.find_by_name("scan").unwrap().0).unwrap().mesh.clone();
    let t = Instant::now();
    let near = scan.positions.iter().filter(|p| bvh.closest(p, 5.0).is_some()).count();
    eprintln!("SCALE   {} closest-point queries {:.1} s ({near} hit)", scan.positions.len(), seconds(t));
    let t = Instant::now();
    let c = s.execute("body.compare", json!({"body": body, "scan": "scan", "tolerance": COMPARE_TOLERANCE})).unwrap();
    eprintln!("SCALE   compare {:.1} s within {} chord {}", seconds(t), c["totals"]["within"], c["chord"]);
    let file = std::env::temp_dir().join(format!("scale_{cap}.step"));
    let t = Instant::now();
    let x = s.execute("export.step", json!({"objects": [body], "path": file.to_string_lossy()})).unwrap();
    eprintln!("SCALE   export.step {:.1} s: {} bytes", seconds(t), x["bytes"]);
    let t = Instant::now();
    let r = s.execute("import.step", json!({"path": file.to_string_lossy(), "name": "back"}));
    eprintln!(
        "SCALE   import.step {:.1} s: {}",
        seconds(t),
        r.map_or_else(|e| format!("error {e}"), |v| v["faces"].to_string())
    );
    let _ = std::fs::remove_file(&file);
}

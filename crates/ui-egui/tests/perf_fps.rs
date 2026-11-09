//! P2: viewport frame rate with a 10 M-triangle scan drawn through the display level of detail.
//! Ignored by default (needs a GPU and several GB of RAM); skips without a GPU adapter:
//! `cargo test --release -p surfacecraft-ui-egui --test perf_fps -- --ignored --nocapture`
//! `SURFACECRAFT_PERF_FPS_MTRIS=10` picks the size in millions of triangles.
#![cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Point3, synth};
use surfacecraft_ui_egui::camera::Camera;
use surfacecraft_ui_egui::shot::draw_items;
use surfacecraft_ui_egui::viewport::{Frame, bench_offscreen};

#[test]
#[ignore = "needs a GPU and several GB of RAM; run explicitly in release"]
fn fps_with_a_large_scan() {
    let millions: f64 = std::env::var("SURFACECRAFT_PERF_FPS_MTRIS").ok().and_then(|v| v.parse().ok()).unwrap_or(10.0);
    // a sphere has 2 · segments · rings / 1 triangles
    let rings = ((millions * 1e6 / 4.0).sqrt()).round() as usize;
    let mesh = synth::sphere(Point3::origin(), 50.0, rings * 2, rings);
    let n = mesh.triangle_count();
    let mut s = Session::new();
    s.doc.add_mesh("scan", mesh);
    let t = Instant::now();
    let items = draw_items(&mut s);
    let prep = t.elapsed().as_secs_f64();
    let drawn: usize = items.iter().map(|i| i.mesh.triangle_count()).sum();
    let mut camera = Camera::default();
    camera.fit(&s.doc.bbox());
    let frame = Frame { size: [1280, 800], origin: camera.target, camera, items, background: [0.1, 0.1, 0.12] };
    let Some(b) = bench_offscreen(&frame, 60) else {
        eprintln!("skipping: no GPU adapter");
        return;
    };
    eprintln!(
        "fps {millions} M: {n} tris, display prep (LOD) {prep:.2} s ({drawn} drawn), first frame {:.2} s, {:.1} fps over {} frames, {} upload(s)",
        b.first_frame_s, b.fps, b.frames, b.uploads
    );
    assert_eq!(b.uploads, 1, "the camera moving must not re-upload geometry");
}

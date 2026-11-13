//! GPU tests of the offscreen renderer (opt-in) and of the colour-state logic.
use super::offscreen::*;
use super::scene::*;
use super::*;
use crate::camera::Camera;
use std::sync::Arc;
use surfacecraft_geom::synth;

/// GPU tests are opt-in: set `SURFACECRAFT_GPU_TESTS=1` on a machine with a real GPU. CI
/// runners only have software adapters (WARP, lavapipe); creating wgpu devices there, from
/// several test threads, intermittently dies with STATUS_ACCESS_VIOLATION on Windows.
/// Returns a guard that serialises device creation, or `None` to skip the test.
fn gpu_test_guard() -> Option<std::sync::MutexGuard<'static, ()>> {
    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    if std::env::var_os("SURFACECRAFT_GPU_TESTS").is_none_or(|v| v.is_empty() || v == "0") {
        eprintln!("skipping: set SURFACECRAFT_GPU_TESTS=1 to run GPU tests");
        return None;
    }
    Some(LOCK.lock().unwrap_or_else(std::sync::PoisonError::into_inner))
}

/// Renders a sphere offscreen: the centre pixel is the sphere, a corner is background.
/// Opt-in (`SURFACECRAFT_GPU_TESTS=1`); skipped without a GPU adapter.
#[test]
fn offscreen_render_shows_the_mesh() {
    let Some(_gpu) = gpu_test_guard() else {
        return;
    };
    let mesh = Arc::new(synth::sphere(Point3::origin(), 10.0, 32, 16));
    let mut camera = Camera::default();
    camera.fit(&mesh.bbox());
    let frame = Frame {
        size: [64, 48],
        origin: camera.target,
        camera,
        items: vec![DrawItem {
            key: 1,
            mesh,
            color: [1.0, 0.0, 0.0, 1.0],
            colors: None,
            colors_key: 0,
            highlight: None,
        }],
        background: [0.0, 0.0, 1.0],
        overlay: Default::default(),
    };
    let Some(px) = render_offscreen(&frame) else {
        eprintln!("skipping: no GPU adapter");
        return;
    };
    assert_eq!(px.len(), 64 * 48 * 4);
    let at = |x: usize, y: usize| px[(y * 64 + x) * 4..(y * 64 + x) * 4 + 4].to_vec();
    let c = at(32, 24);
    assert!(c[0] > 60 && c[2] < 60, "centre pixel {c:?} should be the red sphere");
    let corner = at(1, 1);
    assert!(corner[2] > 200 && corner[0] < 30, "corner pixel {corner:?} should be blue background");
}

/// New colours for the same geometry rewrite the colour buffer only: no second upload, and
/// the new colours reach the picture. Opt-in (`SURFACECRAFT_GPU_TESTS=1`); skipped without a GPU.
#[test]
fn colour_change_does_not_reupload_geometry() {
    let Some(_gpu) = gpu_test_guard() else {
        return;
    };
    let mesh = Arc::new(synth::sphere(Point3::origin(), 10.0, 32, 16));
    let mut camera = Camera::default();
    camera.fit(&mesh.bbox());
    let n = mesh.vertex_count();
    let item = |colors: [f32; 4], key: u64| DrawItem {
        key: 1,
        mesh: mesh.clone(),
        color: [1.0, 1.0, 1.0, 1.0],
        colors: Some(Arc::new(vec![colors; n])),
        colors_key: key,
        highlight: None,
    };
    let frame = |i: DrawItem| Frame {
        size: [64, 48],
        origin: camera.target,
        camera: camera.clone(),
        items: vec![i],
        background: [0.0, 0.0, 1.0],
        overlay: Default::default(),
    };
    let Some((device, queue)) = headless_device() else {
        eprintln!("skipping: no GPU adapter");
        return;
    };
    let mut r = Renderer::new(&device, COLOR_FORMAT);
    let mut draw = |f: &Frame| {
        let mut enc = device.create_command_encoder(&Default::default());
        r.prepare(&device, &queue, &mut enc, f);
        queue.submit([enc.finish()]);
        r.read_rgba(&device, &queue).unwrap_or_default()
    };
    let red = draw(&frame(item([1.0, 0.0, 0.0, 1.0], 1)));
    let green = draw(&frame(item([0.0, 1.0, 0.0, 1.0], 2)));
    let c = |px: &[u8]| px[(24 * 64 + 32) * 4..(24 * 64 + 32) * 4 + 3].to_vec();
    assert!(c(&red)[0] > 60 && c(&red)[1] < 40, "red {:?}", c(&red));
    assert!(c(&green)[1] > 60 && c(&green)[0] < 40, "green {:?}", c(&green));
    assert_eq!((r.uploads, r.color_writes), (1, 1), "geometry once, colours rewritten once");
}

fn plain_item(mesh: &Arc<surfacecraft_geom::Mesh>) -> DrawItem {
    DrawItem { key: 1, mesh: mesh.clone(), color: [0.6, 0.6, 0.6, 1.0], colors: None, colors_key: 0, highlight: None }
}

/// New colours with the same geometry key are uploaded: from none to the very first colour
/// array (its stamp is 1), between arrays, and back to none.
#[test]
fn new_colours_with_the_same_geometry_key_are_uploaded() {
    let mesh = Arc::new(synth::sphere(Point3::origin(), 10.0, 8, 4));
    let n = mesh.vertex_count();
    let plain = plain_item(&mesh);
    let with =
        |key: u64, c: [f32; 4]| DrawItem { colors: Some(Arc::new(vec![c; n])), colors_key: key, ..plain_item(&mesh) };
    let shown = color_state(&plain.color, false, 0);
    assert!(!colors_need_upload(shown, &plain), "unchanged: nothing to write");
    let first = with(1, [1.0, 0.0, 0.0, 1.0]);
    assert!(colors_need_upload(shown, &first), "first colours (key 1) after a plain mesh");
    let shown = color_state(&first.color, true, 1);
    assert!(!colors_need_upload(shown, &first));
    assert!(colors_need_upload(shown, &with(2, [0.0, 1.0, 0.0, 1.0])), "next colour array");
    assert!(colors_need_upload(shown, &plain), "colours removed");
    // every small key differs from the plain state
    for key in 0..64u64 {
        assert!(colors_need_upload(color_state(&plain.color, false, 0), &with(key.max(1), [1.0; 4])));
    }
}

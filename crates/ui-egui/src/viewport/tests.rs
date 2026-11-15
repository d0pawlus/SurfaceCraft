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
            back: None,
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
        back: None,
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
    DrawItem {
        key: 1,
        mesh: mesh.clone(),
        color: [0.6, 0.6, 0.6, 1.0],
        colors: None,
        colors_key: 0,
        highlight: None,
        back: None,
    }
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

/// The model matrix takes a vertex stored relative to its mesh's own origin to the frame's
/// origin: `M (q + mesh origin) - frame origin`, for the identity and for a preview motion.
#[test]
fn model_matrix_takes_mesh_local_vertices_to_the_frame_origin() {
    let mesh_origin = Point3::new(100.0, -20.0, 5.0);
    let frame_origin = Point3::new(103.0, -18.0, 4.0);
    let world = Point3::new(101.0, -19.0, 7.0);
    let q = world - mesh_origin.coords;
    let apply = |m: &[[f32; 4]; 4]| -> [f64; 3] {
        std::array::from_fn(|r| {
            f64::from(m[0][r]) * q.x + f64::from(m[1][r]) * q.y + f64::from(m[2][r]) * q.z + f64::from(m[3][r])
        })
    };
    let near =
        |a: [f64; 3], b: Point3| (a[0] - b.x).abs() < 1e-5 && (a[1] - b.y).abs() < 1e-5 && (a[2] - b.z).abs() < 1e-5;
    let still = apply(&shifted_model(&identity_rows(), &mesh_origin, &frame_origin));
    assert!(near(still, world - frame_origin.coords), "identity {still:?}");
    // a quarter turn about Z and a shift: world (x, y, z) -> (-y + 1, x + 2, z + 3)
    let rows = [[0.0, -1.0, 0.0, 1.0], [1.0, 0.0, 0.0, 2.0], [0.0, 0.0, 1.0, 3.0], [0.0, 0.0, 0.0, 1.0]];
    let moved = apply(&shifted_model(&rows, &mesh_origin, &frame_origin));
    let expect = Point3::new(-world.y + 1.0, world.x + 2.0, world.z + 3.0) - frame_origin.coords;
    assert!(near(moved, expect), "moved {moved:?}");
}

/// Panning moves the frame's precision origin with the camera target: the geometry stays on the
/// GPU (one upload) and the picture follows the pan. Opt-in (`SURFACECRAFT_GPU_TESTS=1`).
#[test]
fn panning_does_not_reupload_geometry() {
    let Some(_gpu) = gpu_test_guard() else {
        return;
    };
    let mesh = Arc::new(synth::sphere(Point3::origin(), 10.0, 32, 16));
    let mut camera = Camera::default();
    camera.fit(&mesh.bbox());
    let frame = |camera: &Camera| Frame {
        size: [64, 48],
        origin: camera.target,
        camera: camera.clone(),
        items: vec![DrawItem { color: [1.0, 0.0, 0.0, 1.0], ..plain_item(&mesh) }],
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
    let is_red = |px: &[u8], x: usize| px[(24 * 64 + x) * 4] > 60 && px[(24 * 64 + x) * 4 + 2] < 60;
    let before = draw(&frame(&camera));
    assert!(is_red(&before, 32) && !is_red(&before, 2), "the sphere starts in the middle");
    // drag right by most of the half width, in steps as a drag does: the sphere leaves the centre
    for _ in 0..7 {
        camera.pan(4.0, 0.0, 48.0);
        draw(&frame(&camera));
    }
    let after = draw(&frame(&camera));
    assert!(!is_red(&after, 4) && is_red(&after, 60), "the sphere followed the pan to the right");
    assert_eq!(r.uploads, 1, "geometry uploaded once");
}

/// A drag across the whole screen keeps the upload; zooming far into a part a long way from the
/// upload origin renews it, and there the new origin keeps features apart that the old one merged.
#[test]
fn upload_origin_is_renewed_only_when_rounding_would_show() {
    let frame = |camera: &Camera| Frame {
        size: [1280, 800],
        origin: camera.target,
        camera: camera.clone(),
        items: Vec::new(),
        background: [0.0; 3],
        overlay: Default::default(),
    };
    let mut camera = Camera { distance: 300.0, ..Camera::default() };
    let uploaded_at = camera.target;
    for _ in 0..4000 {
        camera.pan(3.0, -2.0, 800.0);
        assert!(!origin_too_far(&uploaded_at, &frame(&camera)), "panning keeps the upload");
    }
    // a detail a quarter of the screen away, magnified fifty times
    let mut detail = Camera { distance: 300.0, ..Camera::default() };
    detail.pan(200.0, 0.0, 800.0);
    detail.zoom(2000.0);
    assert!(!origin_too_far(&uploaded_at, &frame(&detail)), "zooming into a scan keeps the upload");
    // a part 500 m from the upload origin, looked at from 5 mm: vertices 0.01 mm apart
    let far = Camera { target: Point3::new(500_000.0, 0.0, 0.0), distance: 5.0, ..Camera::default() };
    assert!(origin_too_far(&uploaded_at, &frame(&far)), "rounding would show");
    assert!(!origin_too_far(&far.target, &frame(&far)), "the renewed upload is kept");
    let stored = |x: f64, origin: &Point3| (x - origin.x) as f32;
    let (a, b) = (far.target.x, far.target.x + 0.01);
    assert_eq!(stored(a, &uploaded_at), stored(b, &uploaded_at), "merged around the old origin");
    assert_ne!(stored(a, &far.target), stored(b, &far.target), "apart around the new one");
}

const FRONT: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
const BACK: [f32; 4] = [0.0, 0.0, 1.0, 1.0];

/// The back colour reaches the shader only for a mesh drawn in its base colour: per-vertex
/// colours (regions, deviation) are data and stay readable from both sides.
#[test]
fn back_colour_applies_only_without_vertex_colours() {
    let mesh = Arc::new(synth::sphere(Point3::origin(), 10.0, 8, 4));
    let n = mesh.vertex_count();
    let two_sided = DrawItem { back: Some(BACK), ..plain_item(&mesh) };
    assert_eq!(back_of(&two_sided), BACK);
    assert_eq!(back_of(&plain_item(&mesh)), [0.0; 4], "no back colour: alpha 0 turns it off");
    let coloured = DrawItem { colors: Some(Arc::new(vec![FRONT; n])), colors_key: 1, ..two_sided.clone() };
    assert_eq!(back_of(&coloured), [0.0; 4]);
    let wrong_length = DrawItem { colors: Some(Arc::new(vec![FRONT; n + 1])), colors_key: 1, ..two_sided };
    assert_eq!(back_of(&wrong_length), BACK, "colours of the wrong length are not drawn");
}

/// A sphere seen from outside shows the front colour; the same sphere with every triangle
/// reversed shows the back colour, and a selection on it keeps the highlight colour.
/// Opt-in (`SURFACECRAFT_GPU_TESTS=1`); skipped without a GPU adapter.
#[test]
fn back_side_is_drawn_in_the_back_colour() {
    let Some(_gpu) = gpu_test_guard() else {
        return;
    };
    let outward = synth::sphere(Point3::origin(), 10.0, 32, 16);
    let normals = outward.vertex_normals();
    assert!(
        outward.positions.iter().zip(normals.iter()).all(|(p, n)| p.coords.dot(n) > 0.0),
        "the synthetic sphere is wound outward"
    );
    let mut inward = outward.clone();
    for t in &mut inward.triangles {
        t.swap(1, 2);
    }
    let mut camera = Camera::default();
    camera.fit(&outward.bbox());
    let centre = |mesh: surfacecraft_geom::Mesh, back: Option<[f32; 4]>, highlight: Option<Arc<Vec<u32>>>| {
        let frame = Frame {
            size: [64, 48],
            origin: camera.target,
            camera: camera.clone(),
            items: vec![DrawItem {
                key: 1,
                mesh: Arc::new(mesh),
                color: FRONT,
                colors: None,
                colors_key: 0,
                highlight,
                back,
            }],
            background: [0.0, 1.0, 0.0],
            overlay: Default::default(),
        };
        render_offscreen(&frame).map(|px| px[(24 * 64 + 32) * 4..(24 * 64 + 32) * 4 + 3].to_vec())
    };
    let Some(front) = centre(outward.clone(), Some(BACK), None) else {
        eprintln!("skipping: no GPU adapter");
        return;
    };
    assert!(front[0] > 60 && front[2] < 60, "outside {front:?} should be the red front colour");
    let back = centre(inward.clone(), Some(BACK), None).unwrap_or_default();
    assert!(back[2] > 60 && back[0] < 60, "inside-out {back:?} should be the blue back colour");
    let one_colour = centre(inward.clone(), None, None).unwrap_or_default();
    assert!(one_colour[0] > 60 && one_colour[2] < 60, "without a back colour {one_colour:?} stays red");
    let all: Vec<u32> = (0..inward.triangles.len() as u32).collect();
    let selected = centre(inward, Some(BACK), Some(Arc::new(all))).unwrap_or_default();
    assert!(selected[0] > 100 && selected[2] < 100, "selection {selected:?} keeps the highlight colour");
}

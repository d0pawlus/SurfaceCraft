//! Headless rendering for screenshots and the frame-rate benchmark (no window; native builds only).
use super::scene::*;
use super::*;

/// Renders `frame` on a fresh headless device and returns RGBA8 pixels (`None` without a GPU).
#[cfg(not(target_arch = "wasm32"))]
#[must_use]
pub fn render_offscreen(frame: &Frame) -> Option<Vec<u8>> {
    let (device, queue) = headless_device()?;
    let mut r = Renderer::new(&device, COLOR_FORMAT);
    let mut enc = device.create_command_encoder(&Default::default());
    r.prepare(&device, &queue, &mut enc, frame);
    queue.submit([enc.finish()]);
    r.read_rgba(&device, &queue)
}

#[cfg(not(target_arch = "wasm32"))]
pub(super) fn headless_device() -> Option<(wgpu::Device, wgpu::Queue)> {
    let instance = wgpu::Instance::default();
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default())).ok()?;
    pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default())).ok()
}

/// Frame rate of the offscreen renderer (P2 criterion: viewport at 10 M triangles).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Clone, Copy, Debug)]
pub struct Bench {
    /// First frame: buffer upload, normals and the first draw (s).
    pub first_frame_s: f64,
    /// Frames timed after the first.
    pub frames: usize,
    /// Mean frames per second over those, camera orbiting, each frame waited for.
    pub fps: f64,
    /// Full mesh uploads during the run.
    pub uploads: u64,
    /// Triangles drawn per frame.
    pub triangles: usize,
}

/// Renders `frame` once (upload) and then `frames` more times with an orbiting camera, waiting for
/// the GPU after every frame. `None` without a GPU adapter.
#[cfg(not(target_arch = "wasm32"))]
#[must_use]
pub fn bench_offscreen(frame: &Frame, frames: usize) -> Option<Bench> {
    let (device, queue) = headless_device()?;
    let mut r = Renderer::new(&device, COLOR_FORMAT);
    let mut run = |f: &Frame| -> Option<()> {
        let mut enc = device.create_command_encoder(&Default::default());
        r.prepare(&device, &queue, &mut enc, f);
        queue.submit([enc.finish()]);
        device.poll(wgpu::PollType::wait_indefinitely()).ok()?;
        Some(())
    };
    let t = std::time::Instant::now();
    run(frame)?;
    let first_frame_s = t.elapsed().as_secs_f64();
    let mut f = frame.clone();
    let t = std::time::Instant::now();
    for _ in 0..frames {
        f.camera.orbit(0.03, 0.002);
        run(&f)?;
    }
    let secs = t.elapsed().as_secs_f64().max(1e-9);
    let triangles = frame.items.iter().map(|i| i.mesh.triangles.len()).sum();
    Some(Bench { first_frame_s, frames, fps: frames as f64 / secs, uploads: r.uploads, triangles })
}

//! Viewport screenshots as PNG (MCP `screenshot`, the AI assistant's view), rendered offscreen.
use crate::camera::Camera;
use crate::viewport::DrawItem;
use surfacecraft_engine::Session;

/// Display items of the session as draw items with the UI's colours.
#[must_use]
pub fn draw_items(session: &mut Session) -> Vec<DrawItem> {
    session
        .display_items()
        .into_iter()
        .map(|d| DrawItem {
            key: d.key,
            color: crate::app::kind_color(d.kind),
            mesh: d.mesh,
            colors: d.colors,
            highlight: None,
        })
        .collect()
}

/// Encodes RGBA8 pixels as PNG.
#[must_use]
pub fn encode_png(rgba: &[u8], w: u32, h: u32) -> Option<Vec<u8>> {
    let mut out = Vec::new();
    {
        let mut enc = png::Encoder::new(&mut out, w, h);
        enc.set_color(png::ColorType::Rgba);
        enc.set_depth(png::BitDepth::Eight);
        let mut wr = enc.write_header().ok()?;
        wr.write_image_data(rgba).ok()?;
    }
    Some(out)
}

/// A PNG of the session's visible objects from `camera` (default: framing everything).
#[cfg(not(target_arch = "wasm32"))]
pub fn screenshot_png(session: &mut Session, camera: Option<&Camera>, w: u32, h: u32) -> Option<Vec<u8>> {
    let (w, h) = (w.clamp(16, 4096), h.clamp(16, 4096));
    let cam = camera.cloned().unwrap_or_else(|| {
        let mut c = Camera::default();
        c.fit(&session.doc.bbox());
        c
    });
    let frame = crate::viewport::Frame {
        size: [w, h],
        origin: cam.target,
        camera: cam,
        items: draw_items(session),
        background: [0.85, 0.87, 0.9],
    };
    let rgba = crate::viewport::render_offscreen(&frame)?;
    encode_png(&rgba, w, h)
}

/// Unused on the web: the window's own device renders there.
#[cfg(target_arch = "wasm32")]
pub fn screenshot_png(_session: &mut Session, _camera: Option<&Camera>, _w: u32, _h: u32) -> Option<Vec<u8>> {
    None
}

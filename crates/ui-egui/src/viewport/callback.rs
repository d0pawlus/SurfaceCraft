//! The paint callback that puts the viewport into egui, and the renderer kept in egui-wgpu's resources.
use super::scene::*;
use super::*;

/// The paint callback carrying one frame.
pub struct ViewportCallback(pub Frame);

impl egui_wgpu::CallbackTrait for ViewportCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen: &egui_wgpu::ScreenDescriptor,
        encoder: &mut wgpu::CommandEncoder,
        resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        if let Some(r) = resources.get_mut::<Stored>() {
            r.prepare(device, queue, encoder, &self.0);
        }
        Vec::new()
    }

    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        pass: &mut wgpu::RenderPass<'static>,
        resources: &egui_wgpu::CallbackResources,
    ) {
        if let Some(r) = resources.get::<Stored>() {
            r.paint(pass);
        }
    }
}

/// The renderer as egui-wgpu's resources keep it. Those must be `Send + Sync`, which wgpu's
/// objects are not in a browser build with threads (they are JavaScript handles of the page's
/// thread), so there the renderer sits in a wrapper that only that thread may open; egui paints
/// on no other.
#[cfg(not(target_arch = "wasm32"))]
type Stored = Renderer;
#[cfg(target_arch = "wasm32")]
type Stored = send_wrapper::SendWrapper<Renderer>;

/// Installs the renderer into egui-wgpu's resources (once, at start-up).
pub fn install(rs: &egui_wgpu::RenderState) {
    let r = Renderer::new(&rs.device, rs.target_format);
    #[cfg(target_arch = "wasm32")]
    let r = send_wrapper::SendWrapper::new(r);
    rs.renderer.write().callback_resources.insert::<Stored>(r);
}

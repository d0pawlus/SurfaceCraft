//! What the GPU can hold. A mesh is drawn from three buffers (geometry and colour per vertex,
//! indices per triangle), and a buffer larger than the device allows is a wgpu validation error
//! (by default a panic). A scan is drawn through its display level of detail (at most
//! `LOD_BUDGET` triangles, `surfacecraft_engine::display`), but a command's preview draws its mesh
//! whole, and so does a scan whose level of detail cannot be built. egui-wgpu asks for wgpu's
//! default limits, whose largest buffer is 256 MiB (about 22 million triangles of indices); the
//! viewport asks the adapter for the largest it has instead, and a mesh that still does not fit is
//! not drawn (the app says so).
use std::sync::Arc;
use surfacecraft_geom::Mesh;

/// Bytes of one vertex of the geometry buffer (position and normal).
const GEO_BYTES: u64 = std::mem::size_of::<super::Geo>() as u64;
/// Bytes of one vertex of the colour buffer.
const COLOR_BYTES: u64 = std::mem::size_of::<[f32; 4]>() as u64;
/// Bytes of one triangle of the index buffer.
const INDEX_BYTES: u64 = std::mem::size_of::<[u32; 3]>() as u64;

/// Bytes of the largest buffer the viewport makes for `mesh`.
#[must_use]
pub fn largest_buffer(mesh: &Mesh) -> u64 {
    let vertices = mesh.positions.len() as u64;
    let triangles = mesh.triangles.len() as u64;
    (vertices * GEO_BYTES).max(vertices * COLOR_BYTES).max(triangles * INDEX_BYTES)
}

/// Can a device whose largest buffer is `max_buffer` bytes draw `mesh`?
#[must_use]
pub fn fits(mesh: &Mesh, max_buffer: u64) -> bool {
    largest_buffer(mesh) <= max_buffer
}

/// `options` with a device that has the adapter's largest buffer, not wgpu's default (for the
/// eframe options of the desktop and the web app).
#[must_use]
pub fn with_largest_buffers(options: egui_wgpu::WgpuConfiguration) -> egui_wgpu::WgpuConfiguration {
    let egui_wgpu::WgpuSetup::CreateNew(create) = options.wgpu_setup else { return options };
    let chosen = Arc::clone(&create.device_descriptor);
    let device_descriptor: Arc<dyn Fn(&wgpu::Adapter) -> wgpu::DeviceDescriptor<'static> + Send + Sync> =
        Arc::new(move |adapter: &wgpu::Adapter| {
            let wanted = chosen(adapter);
            let max_buffer_size = wanted.required_limits.max_buffer_size.max(adapter.limits().max_buffer_size);
            wgpu::DeviceDescriptor {
                required_limits: wgpu::Limits { max_buffer_size, ..wanted.required_limits },
                ..wanted
            }
        });
    egui_wgpu::WgpuConfiguration {
        wgpu_setup: egui_wgpu::WgpuSetup::CreateNew(egui_wgpu::WgpuSetupCreateNew { device_descriptor, ..create }),
        ..options
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::Point3;

    fn mesh(vertices: usize, triangles: usize) -> Mesh {
        Mesh { positions: vec![Point3::origin(); vertices], triangles: vec![[0, 0, 0]; triangles] }
    }

    #[test]
    fn the_largest_buffer_is_the_geometry_or_the_indices() {
        assert_eq!(largest_buffer(&mesh(10, 20)), 240, "20 triangles of indices");
        assert_eq!(largest_buffer(&mesh(30, 20)), 720, "30 vertices of position and normal");
        // 20 million triangles of a closed scan fit wgpu's default 256 MiB, unwelded ones do not
        let default = wgpu::Limits::default().max_buffer_size;
        assert!(fits(&mesh(10_000_000, 20_000_000), default));
        assert!(!fits(&mesh(60_000_000, 20_000_000), default));
    }
}

//! Uploading meshes to the GPU: the offscreen target, the per-mesh buffers and the highlight index buffer.
use super::scene::*;
use super::*;
use surfacecraft_geom::Point3;
use wgpu::util::DeviceExt;

impl Renderer {
    pub(super) fn ensure_target(&mut self, device: &wgpu::Device, size: [u32; 2]) {
        if self.target.as_ref().is_some_and(|t| t.size == size) {
            return;
        }
        let extent = wgpu::Extent3d { width: size[0].max(1), height: size[1].max(1), depth_or_array_layers: 1 };
        let color = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("viewport colour"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: COLOR_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let depth = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("viewport depth"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let color_view = color.create_view(&Default::default());
        let depth_view = depth.create_view(&Default::default());
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("blit"),
            layout: &self.blit_layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&color_view) },
                wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(&self.sampler) },
            ],
        });
        self.target = Some(Target { size, color, color_view, depth_view, bind });
    }

    pub(super) fn upload(
        device: &wgpu::Device,
        model_layout: &wgpu::BindGroupLayout,
        item: &DrawItem,
        origin: &Point3,
    ) -> GpuMesh {
        let m = &item.mesh;
        let normals = m.vertex_normals();
        let verts: Vec<Geo> = m
            .positions
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let q = p - origin.coords;
                let n = normals.get(i).copied().unwrap_or_default();
                Geo { pos: [q.x as f32, q.y as f32, q.z as f32], normal: [n.x as f32, n.y as f32, n.z as f32] }
            })
            .collect();
        drop(normals);
        let indices: Vec<u32> = m.triangles.iter().flatten().copied().collect();
        let vb = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh vb"),
            contents: bytemuck::cast_slice(&verts),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let cb = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh colours"),
            contents: bytemuck::cast_slice(&colors_of(item)),
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        });
        let ib = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh ib"),
            contents: bytemuck::cast_slice(&indices),
            usage: wgpu::BufferUsages::INDEX,
        });
        let model_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh model"),
            contents: bytemuck::cast_slice(&shifted_model(&identity_rows(), origin)),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let model_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("mesh model"),
            layout: model_layout,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: model_buf.as_entire_binding() }],
        });
        GpuMesh {
            model_buf,
            model_bind,
            vb,
            cb,
            ib,
            count: indices.len() as u32,
            vertices: verts.len(),
            origin: [origin.x, origin.y, origin.z],
            color_state: color_state(&item.color, item.colors.is_some(), item.colors_key),
            hl_ids: Vec::new(),
            hl_ib: None,
            hl_count: 0,
            used: true,
        }
    }

    /// Index buffer of the highlighted triangles (indices into the mesh's own vertices).
    pub(super) fn upload_highlight(device: &wgpu::Device, g: &mut GpuMesh, item: &DrawItem) {
        let ids: &[u32] = item.highlight.as_ref().map_or(&[], |h| h.as_slice());
        if g.hl_ids == ids {
            return;
        }
        g.hl_ids = ids.to_vec();
        let indices: Vec<u32> =
            ids.iter().filter_map(|&t| item.mesh.triangles.get(t as usize)).flatten().copied().collect();
        g.hl_count = indices.len() as u32;
        g.hl_ib = (!indices.is_empty()).then(|| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("highlight ib"),
                contents: bytemuck::cast_slice(&indices),
                usage: wgpu::BufferUsages::INDEX,
            })
        });
    }
}

//! Rendering a frame offscreen, copying it into egui's pass, and reading it back.
use super::scene::*;
use super::*;

impl Renderer {
    /// Uploads changed meshes and renders the frame offscreen.
    pub fn prepare(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        f: &Frame,
    ) {
        self.ensure_target(device, f.size);
        for m in self.meshes.values_mut() {
            m.used = false;
        }
        self.draw_list.clear();
        let o = [f.origin.x, f.origin.y, f.origin.z];
        for item in &f.items {
            let k = item.key;
            let fresh = self.meshes.get(&k).is_none_or(|g| g.origin != o || g.vertices != item.mesh.positions.len());
            if fresh {
                let g = Self::upload(device, &self.model_layout, item, &f.origin);
                self.meshes.insert(k, g);
                self.uploads += 1;
            }
            if let Some(g) = self.meshes.get_mut(&k) {
                g.used = true;
                let rows = f.overlay.models.iter().find(|(mk, _)| *mk == k).map_or_else(identity_rows, |(_, m)| *m);
                queue.write_buffer(&g.model_buf, 0, bytemuck::cast_slice(&shifted_model(&rows, &f.origin)));
                if colors_need_upload(g.color_state, item) {
                    queue.write_buffer(&g.cb, 0, bytemuck::cast_slice(&colors_of(item)));
                    g.color_state = color_state(&item.color, item.colors.is_some(), item.colors_key);
                    self.color_writes += 1;
                }
                Self::upload_highlight(device, g, item);
            }
            self.draw_list.push(k);
        }
        self.meshes.retain(|_, g| g.used);
        // globals
        let aspect = f64::from(f.size[0].max(1)) / f64::from(f.size[1].max(1));
        let vp = f.camera.projection(aspect) * f.camera.view(&f.origin);
        let mut view_proj = [[0f32; 4]; 4];
        for (c, col) in view_proj.iter_mut().enumerate() {
            for (r, v) in col.iter_mut().enumerate() {
                *v = vp[(r, c)] as f32;
            }
        }
        let eye = f.camera.eye() - f.origin.coords;
        let light = f.camera.eye() - f.camera.target + surfacecraft_geom::Vec3::new(0.0, 0.0, f.camera.distance);
        let g = Globals {
            view_proj,
            eye: [eye.x as f32, eye.y as f32, eye.z as f32, 1.0],
            light: [light.x as f32, light.y as f32, light.z as f32, 0.0],
            clip: crate::sectionview::shifted_plane(&f.overlay.clip, &f.origin),
        };
        queue.write_buffer(&self.globals, 0, bytemuck::bytes_of(&g));
        // axis triad at the world origin, sized to the view
        let len = (f.camera.distance * 0.15) as f32;
        let oo = [-(f.origin.x as f32), -(f.origin.y as f32), -(f.origin.z as f32)];
        let mk = |d: [f32; 3]| {
            [
                Geo { pos: oo, normal: [0.0; 3] },
                Geo { pos: [oo[0] + d[0] * len, oo[1] + d[1] * len, oo[2] + d[2] * len], normal: [0.0; 3] },
            ]
        };
        let axes: Vec<Geo> = [mk(X_AXIS), mk(Y_AXIS), mk(Z_AXIS)].concat();
        queue.write_buffer(&self.axes, 0, bytemuck::cast_slice(&axes));
        let Some(t) = &self.target else { return };
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("viewport"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &t.color_view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: f.background[0],
                        g: f.background[1],
                        b: f.background[2],
                        a: 1.0,
                    }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &t.depth_view,
                depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Store }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_bind_group(0, &self.globals_bind, &[]);
        pass.set_pipeline(&self.mesh_pipeline);
        for k in &self.draw_list {
            if let Some(g) = self.meshes.get(k) {
                pass.set_bind_group(1, &g.model_bind, &[]);
                pass.set_vertex_buffer(0, g.vb.slice(..));
                pass.set_vertex_buffer(1, g.cb.slice(..));
                pass.set_index_buffer(g.ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..g.count, 0, 0..1);
            }
        }
        pass.set_pipeline(&self.highlight_pipeline);
        pass.set_vertex_buffer(1, self.highlight_color.slice(..));
        for k in &self.draw_list {
            if let Some(g) = self.meshes.get(k)
                && let Some(ib) = &g.hl_ib
            {
                pass.set_bind_group(1, &g.model_bind, &[]);
                pass.set_vertex_buffer(0, g.vb.slice(..));
                pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..g.hl_count, 0, 0..1);
            }
        }
        pass.set_bind_group(1, &self.identity_bind, &[]);
        pass.set_pipeline(&self.line_pipeline);
        pass.set_vertex_buffer(0, self.axes.slice(..));
        pass.set_vertex_buffer(1, self.axes_colors.slice(..));
        pass.draw(0..6, 0..1);
    }

    /// Copies the offscreen image into egui's pass.
    pub fn paint(&self, pass: &mut wgpu::RenderPass<'static>) {
        if let Some(t) = &self.target {
            pass.set_pipeline(&self.blit_pipeline);
            pass.set_bind_group(0, &t.bind, &[]);
            pass.draw(0..3, 0..1);
        }
    }

    /// Copies the offscreen image to a buffer and returns tightly packed RGBA8 rows.
    pub fn read_rgba(&self, device: &wgpu::Device, queue: &wgpu::Queue) -> Option<Vec<u8>> {
        let t = self.target.as_ref()?;
        let [w, h] = [t.size[0].max(1), t.size[1].max(1)];
        let row = w * 4;
        let padded = row.div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT) * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("readback"),
            size: u64::from(padded) * u64::from(h),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut enc = device.create_command_encoder(&Default::default());
        enc.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &t.color,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &buf,
                layout: wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(padded), rows_per_image: Some(h) },
            },
            wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        );
        queue.submit([enc.finish()]);
        let slice = buf.slice(..);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        device.poll(wgpu::PollType::wait_indefinitely()).ok()?;
        let data = slice.get_mapped_range().ok()?;
        let mut out = Vec::with_capacity((row * h) as usize);
        for y in 0..h as usize {
            let start = y * padded as usize;
            out.extend_from_slice(data.get(start..start + row as usize)?);
        }
        Some(out)
    }
}

//! The 3-D viewport: raw wgpu pipelines (design doc §5) drawing into an offscreen colour and
//! depth target in the egui-wgpu `prepare` callback, then copied into egui's pass by a blit.
//! Rendering offscreen keeps the viewport independent of the window's depth buffer and gives
//! screenshots for agents for free.
use crate::camera::Camera;
use bytemuck::{Pod, Zeroable};
use std::collections::HashMap;
use std::sync::Arc;
use surfacecraft_geom::{Mesh, Point3};
use wgpu::util::DeviceExt;

const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Vertex {
    pos: [f32; 3],
    normal: [f32; 3],
    color: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Globals {
    view_proj: [[f32; 4]; 4],
    eye: [f32; 4],
    light: [f32; 4],
}

/// One object to draw this frame.
#[derive(Clone)]
pub struct DrawItem {
    /// Changes when the geometry changes.
    pub key: usize,
    /// Geometry.
    pub mesh: Arc<Mesh>,
    /// Base colour.
    pub color: [f32; 4],
    /// Optional per-vertex colours.
    pub colors: Option<Arc<Vec<[f32; 4]>>>,
    /// Optional per-triangle highlight (selected triangles), drawn in the highlight colour.
    pub highlight: Option<Arc<Vec<u32>>>,
}

/// Everything the GPU needs for one frame.
#[derive(Clone)]
pub struct Frame {
    /// Viewport size in physical pixels.
    pub size: [u32; 2],
    /// Camera.
    pub camera: Camera,
    /// Precision origin (scene centre).
    pub origin: Point3,
    /// Objects.
    pub items: Vec<DrawItem>,
    /// Background colour.
    pub background: [f64; 3],
}

struct GpuMesh {
    vb: wgpu::Buffer,
    ib: wgpu::Buffer,
    count: u32,
    origin: [f64; 3],
    used: bool,
}

struct Target {
    size: [u32; 2],
    color: wgpu::Texture,
    color_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    bind: wgpu::BindGroup,
}

/// GPU state kept in egui-wgpu's callback resources.
pub struct Renderer {
    mesh_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    blit_pipeline: wgpu::RenderPipeline,
    globals: wgpu::Buffer,
    globals_bind: wgpu::BindGroup,
    blit_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    meshes: HashMap<(usize, u64), GpuMesh>,
    axes: wgpu::Buffer,
    target: Option<Target>,
    draw_list: Vec<(usize, u64)>,
}

fn color_hash(c: &[f32; 4], has_colors: bool, hl: usize) -> u64 {
    let mut h = 1469598103934665603u64;
    for v in c {
        h = (h ^ u64::from(v.to_bits())).wrapping_mul(1099511628211);
    }
    h ^= u64::from(has_colors);
    (h ^ hl as u64).wrapping_mul(1099511628211)
}

impl Renderer {
    /// Creates the pipelines for egui's target format.
    #[must_use]
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("surfacecraft mesh"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/mesh.wgsl").into()),
        });
        let blit_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("surfacecraft blit"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/blit.wgsl").into()),
        });
        let globals_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("globals"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let globals = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("globals"),
            size: std::mem::size_of::<Globals>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let globals_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("globals"),
            layout: &globals_layout,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: globals.as_entire_binding() }],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("mesh"),
            bind_group_layouts: &[Some(&globals_layout)],
            immediate_size: 0,
        });
        let attrs = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x4];
        let vlayout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &attrs,
        };
        let make = |topology: wgpu::PrimitiveTopology, label: &str| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs_main"),
                    compilation_options: Default::default(),
                    buffers: &[Some(vlayout.clone())],
                },
                primitive: wgpu::PrimitiveState { topology, cull_mode: None, ..Default::default() },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FORMAT,
                    depth_write_enabled: Some(true),
                    depth_compare: Some(wgpu::CompareFunction::LessEqual),
                    stencil: Default::default(),
                    bias: Default::default(),
                }),
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some("fs_main"),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: COLOR_FORMAT,
                        blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let mesh_pipeline = make(wgpu::PrimitiveTopology::TriangleList, "mesh");
        let line_pipeline = make(wgpu::PrimitiveTopology::LineList, "lines");
        let blit_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("blit"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let blit_pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("blit"),
            bind_group_layouts: &[Some(&blit_layout)],
            immediate_size: 0,
        });
        let blit_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("blit"),
            layout: Some(&blit_pl),
            vertex: wgpu::VertexState {
                module: &blit_shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            primitive: Default::default(),
            depth_stencil: None,
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &blit_shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview_mask: None,
            cache: None,
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("viewport"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let axes = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("axes"),
            size: (6 * std::mem::size_of::<Vertex>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        Self {
            mesh_pipeline,
            line_pipeline,
            blit_pipeline,
            globals,
            globals_bind,
            blit_layout,
            sampler,
            meshes: HashMap::new(),
            axes,
            target: None,
            draw_list: Vec::new(),
        }
    }

    fn ensure_target(&mut self, device: &wgpu::Device, size: [u32; 2]) {
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

    fn upload(device: &wgpu::Device, item: &DrawItem, origin: &Point3) -> GpuMesh {
        let m = &item.mesh;
        let normals = m.vertex_normals();
        let hl_color = [1.0f32, 0.55, 0.1, 1.0];
        let mut verts: Vec<Vertex> = m
            .positions
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let q = p - origin.coords;
                let n = normals.get(i).copied().unwrap_or_default();
                let color = item.colors.as_ref().and_then(|c| c.get(i).copied()).unwrap_or(item.color);
                Vertex {
                    pos: [q.x as f32, q.y as f32, q.z as f32],
                    normal: [n.x as f32, n.y as f32, n.z as f32],
                    color,
                }
            })
            .collect();
        let mut indices: Vec<u32> = m.triangles.iter().flatten().copied().collect();
        // highlighted triangles get their own vertices in the highlight colour
        if let Some(hl) = &item.highlight {
            for &t in hl.iter() {
                let Some(tri) = m.triangles.get(t as usize) else { continue };
                for &vi in tri {
                    if let Some(v) = verts.get(vi as usize).copied() {
                        indices.push(verts.len() as u32);
                        verts.push(Vertex { color: hl_color, ..v });
                    }
                }
            }
        }
        let vb = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh vb"),
            contents: bytemuck::cast_slice(&verts),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let ib = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh ib"),
            contents: bytemuck::cast_slice(&indices),
            usage: wgpu::BufferUsages::INDEX,
        });
        GpuMesh { vb, ib, count: indices.len() as u32, origin: [origin.x, origin.y, origin.z], used: true }
    }

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
            let k = (
                item.key,
                color_hash(
                    &item.color,
                    item.colors.is_some(),
                    item.highlight.as_ref().map_or(0, |h| Arc::as_ptr(h) as usize),
                ),
            );
            let fresh = self.meshes.get(&k).is_none_or(|g| g.origin != o);
            if fresh {
                let g = Self::upload(device, item, &f.origin);
                self.meshes.insert(k, g);
            }
            if let Some(g) = self.meshes.get_mut(&k) {
                g.used = true;
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
        };
        queue.write_buffer(&self.globals, 0, bytemuck::bytes_of(&g));
        // axis triad at the world origin, sized to the view
        let len = (f.camera.distance * 0.15) as f32;
        let oo = [-(f.origin.x as f32), -(f.origin.y as f32), -(f.origin.z as f32)];
        let mk = |d: [f32; 3], c: [f32; 4]| {
            [
                Vertex { pos: oo, normal: [0.0; 3], color: c },
                Vertex {
                    pos: [oo[0] + d[0] * len, oo[1] + d[1] * len, oo[2] + d[2] * len],
                    normal: [0.0; 3],
                    color: c,
                },
            ]
        };
        let axes: Vec<Vertex> = [
            mk([1.0, 0.0, 0.0], [0.9, 0.2, 0.2, 1.0]),
            mk([0.0, 1.0, 0.0], [0.2, 0.8, 0.2, 1.0]),
            mk([0.0, 0.0, 1.0], [0.25, 0.4, 0.95, 1.0]),
        ]
        .concat();
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
                pass.set_vertex_buffer(0, g.vb.slice(..));
                pass.set_index_buffer(g.ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..g.count, 0, 0..1);
            }
        }
        pass.set_pipeline(&self.line_pipeline);
        pass.set_vertex_buffer(0, self.axes.slice(..));
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

/// Renders `frame` on a fresh headless device and returns RGBA8 pixels (`None` without a GPU).
#[cfg(not(target_arch = "wasm32"))]
#[must_use]
pub fn render_offscreen(frame: &Frame) -> Option<Vec<u8>> {
    let instance = wgpu::Instance::default();
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default())).ok()?;
    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default())).ok()?;
    let mut r = Renderer::new(&device, COLOR_FORMAT);
    let mut enc = device.create_command_encoder(&Default::default());
    r.prepare(&device, &queue, &mut enc, frame);
    queue.submit([enc.finish()]);
    r.read_rgba(&device, &queue)
}

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
        if let Some(r) = resources.get_mut::<Renderer>() {
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
        if let Some(r) = resources.get::<Renderer>() {
            r.paint(pass);
        }
    }
}

/// Installs the renderer into egui-wgpu's resources (once, at start-up).
pub fn install(rs: &egui_wgpu::RenderState) {
    let r = Renderer::new(&rs.device, rs.target_format);
    rs.renderer.write().callback_resources.insert(r);
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests {
    use super::*;
    use surfacecraft_geom::synth;

    /// Renders a sphere offscreen: the centre pixel is the sphere, a corner is background.
    /// Skipped when the machine has no GPU adapter (CI runners).
    #[test]
    fn offscreen_render_shows_the_mesh() {
        let mesh = Arc::new(synth::sphere(Point3::origin(), 10.0, 32, 16));
        let mut camera = Camera::default();
        camera.fit(&mesh.bbox());
        let frame = Frame {
            size: [64, 48],
            origin: camera.target,
            camera,
            items: vec![DrawItem { key: 1, mesh, color: [1.0, 0.0, 0.0, 1.0], colors: None, highlight: None }],
            background: [0.0, 0.0, 1.0],
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
}

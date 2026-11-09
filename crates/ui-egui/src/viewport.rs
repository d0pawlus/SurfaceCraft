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

/// Geometry stream of a vertex: uploaded with the mesh, never rewritten by a colour change.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Geo {
    pos: [f32; 3],
    normal: [f32; 3],
}

/// Colour of a triangle selection.
fn highlight() -> [f32; 4] {
    crate::theme::Tokens::default().rgba(crate::theme::Token::Highlight)
}

const X_AXIS: [f32; 3] = [1.0, 0.0, 0.0]; // not a colour
const Y_AXIS: [f32; 3] = [0.0, 1.0, 0.0]; // not a colour
const Z_AXIS: [f32; 3] = [0.0, 0.0, 1.0]; // not a colour

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
    /// Changes when the geometry changes (the GPU mesh is re-uploaded).
    pub key: usize,
    /// Geometry.
    pub mesh: Arc<Mesh>,
    /// Base colour.
    pub color: [f32; 4],
    /// Optional per-vertex colours.
    pub colors: Option<Arc<Vec<[f32; 4]>>>,
    /// Changes when the content of `colors` does (only the colour buffer is rewritten).
    pub colors_key: u64,
    /// Optional per-triangle highlight (selected triangles of `mesh`), drawn in the highlight colour.
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
    /// Positions and normals.
    vb: wgpu::Buffer,
    /// Colours, a separate stream so a colour change is one `write_buffer`.
    cb: wgpu::Buffer,
    ib: wgpu::Buffer,
    count: u32,
    vertices: usize,
    origin: [f64; 3],
    /// Colour state last written to `cb`.
    color_state: u64,
    /// Highlighted triangles (indices into the mesh) and their index buffer.
    hl_ids: Vec<u32>,
    hl_ib: Option<wgpu::Buffer>,
    hl_count: u32,
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
    /// Same geometry stream, one colour for the whole draw (selection highlight).
    highlight_pipeline: wgpu::RenderPipeline,
    highlight_color: wgpu::Buffer,
    line_pipeline: wgpu::RenderPipeline,
    blit_pipeline: wgpu::RenderPipeline,
    globals: wgpu::Buffer,
    globals_bind: wgpu::BindGroup,
    blit_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    meshes: HashMap<usize, GpuMesh>,
    axes: wgpu::Buffer,
    axes_colors: wgpu::Buffer,
    target: Option<Target>,
    draw_list: Vec<usize>,
    /// Number of full mesh uploads so far (tests and the fps benchmark).
    pub uploads: u64,
    /// Number of colour-buffer rewrites.
    pub color_writes: u64,
}

/// What the colour buffer of an item shows: its base colour and which colour array.
fn color_state(c: &[f32; 4], has_colors: bool, key: u64) -> u64 {
    let mut h = 1469598103934665603u64;
    for v in c {
        h = (h ^ u64::from(v.to_bits())).wrapping_mul(1099511628211);
    }
    // fold each field in on its own: XOR-ing `has_colors` and `key` together made
    // (no colours, key 0) and (colours, key 1) - the first colour array ever shown - the same
    // state, so the first region/deviation colours were never uploaded
    for v in [u64::from(has_colors), key] {
        h = (h ^ v).wrapping_mul(1099511628211);
    }
    h
}

/// Whether the colour buffer must be rewritten: what it shows (`shown`) differs from what the
/// item wants now (new colours with the same geometry key included).
fn colors_need_upload(shown: u64, item: &DrawItem) -> bool {
    shown != color_state(&item.color, item.colors.is_some(), item.colors_key)
}

fn colors_of(item: &DrawItem) -> Vec<[f32; 4]> {
    let n = item.mesh.positions.len();
    match &item.colors {
        Some(c) if c.len() == n => c.as_ref().clone(),
        _ => vec![item.color; n],
    }
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
        let geo_attrs = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];
        let color_attrs = wgpu::vertex_attr_array![2 => Float32x4];
        let geo_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Geo>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &geo_attrs,
        };
        let color_layout = |step_mode| wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<[f32; 4]>() as u64,
            step_mode,
            attributes: &color_attrs,
        };
        let make = |topology: wgpu::PrimitiveTopology, label: &str, step: wgpu::VertexStepMode| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs_main"),
                    compilation_options: Default::default(),
                    buffers: &[Some(geo_layout.clone()), Some(color_layout(step))],
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
        let mesh_pipeline = make(wgpu::PrimitiveTopology::TriangleList, "mesh", wgpu::VertexStepMode::Vertex);
        let highlight_pipeline =
            make(wgpu::PrimitiveTopology::TriangleList, "highlight", wgpu::VertexStepMode::Instance);
        let line_pipeline = make(wgpu::PrimitiveTopology::LineList, "lines", wgpu::VertexStepMode::Vertex);
        let highlight_color = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("highlight colour"),
            contents: bytemuck::cast_slice(&[highlight()]),
            usage: wgpu::BufferUsages::VERTEX,
        });
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
            size: (6 * std::mem::size_of::<Geo>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let axes_colors = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("axes colours"),
            contents: bytemuck::cast_slice(&crate::theme::Tokens::default().axis_vertex_colors()),
            usage: wgpu::BufferUsages::VERTEX,
        });
        Self {
            mesh_pipeline,
            highlight_pipeline,
            highlight_color,
            line_pipeline,
            blit_pipeline,
            globals,
            globals_bind,
            blit_layout,
            sampler,
            meshes: HashMap::new(),
            axes,
            axes_colors,
            target: None,
            draw_list: Vec::new(),
            uploads: 0,
            color_writes: 0,
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
        GpuMesh {
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
    fn upload_highlight(device: &wgpu::Device, g: &mut GpuMesh, item: &DrawItem) {
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
                let g = Self::upload(device, item, &f.origin);
                self.meshes.insert(k, g);
                self.uploads += 1;
            }
            if let Some(g) = self.meshes.get_mut(&k) {
                g.used = true;
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
                pass.set_vertex_buffer(0, g.vb.slice(..));
                pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..g.hl_count, 0, 0..1);
            }
        }
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
fn headless_device() -> Option<(wgpu::Device, wgpu::Queue)> {
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
            items: vec![DrawItem {
                key: 1,
                mesh,
                color: [1.0, 0.0, 0.0, 1.0],
                colors: None,
                colors_key: 0,
                highlight: None,
            }],
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

    /// New colours for the same geometry rewrite the colour buffer only: no second upload, and
    /// the new colours reach the picture. Skipped without a GPU.
    #[test]
    fn colour_change_does_not_reupload_geometry() {
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
        }
    }

    /// New colours with the same geometry key are uploaded: from none to the very first colour
    /// array (its stamp is 1), between arrays, and back to none.
    #[test]
    fn new_colours_with_the_same_geometry_key_are_uploaded() {
        let mesh = Arc::new(synth::sphere(Point3::origin(), 10.0, 8, 4));
        let n = mesh.vertex_count();
        let plain = plain_item(&mesh);
        let with = |key: u64, c: [f32; 4]| DrawItem {
            colors: Some(Arc::new(vec![c; n])),
            colors_key: key,
            ..plain_item(&mesh)
        };
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
}

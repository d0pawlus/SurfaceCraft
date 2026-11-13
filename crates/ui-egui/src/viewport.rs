//! The 3-D viewport: raw wgpu pipelines (design doc §5) drawing into an offscreen colour and
//! depth target in the egui-wgpu `prepare` callback, then copied into egui's pass by a blit.
//! Rendering offscreen keeps the viewport independent of the window's depth buffer and gives
//! screenshots for agents for free.
use bytemuck::{Pod, Zeroable};
use std::collections::HashMap;
use surfacecraft_geom::Point3;
use wgpu::util::DeviceExt;

mod callback;
#[cfg(not(target_arch = "wasm32"))]
mod offscreen;
mod render;
mod scene;
#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests;
mod upload;

pub use callback::{ViewportCallback, install};
#[cfg(not(target_arch = "wasm32"))]
pub use offscreen::{Bench, bench_offscreen, render_offscreen};
pub use scene::{DrawItem, Frame, Overlay};
use scene::{identity_rows, shifted_model};

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
    /// Clip plane `(n, d)` in the shifted space: fragments with `n · p > d` are discarded.
    clip: [f32; 4],
}
struct GpuMesh {
    /// Preview motion (column-major mat4 uniform) and its bind group.
    model_buf: wgpu::Buffer,
    model_bind: wgpu::BindGroup,
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
    model_layout: wgpu::BindGroupLayout,
    /// Bind group of the identity model matrix (axes, anything not moved).
    identity_bind: wgpu::BindGroup,
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
        let model_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("model"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let identity_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("identity model"),
            contents: bytemuck::cast_slice(&shifted_model(&identity_rows(), &Point3::origin())),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let identity_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("identity model"),
            layout: &model_layout,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: identity_buf.as_entire_binding() }],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("mesh"),
            bind_group_layouts: &[Some(&globals_layout), Some(&model_layout)],
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
            model_layout,
            identity_bind,
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
}

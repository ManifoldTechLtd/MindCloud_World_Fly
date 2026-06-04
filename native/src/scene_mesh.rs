/// Lightweight mesh/entity system for rendering 3D objects in the scene.
/// Uses the same view-projection as the splat renderer (including VIEWPORT_Y_FLIP).

use cgmath::{Matrix4, SquareMatrix, Vector3, Vector4};
use wgpu::util::DeviceExt;

// ---- Vertex ----

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

// ---- Mesh ----

pub struct Mesh {
    vertex_buf: wgpu::Buffer,
    vertex_count: u32,
    topology: wgpu::PrimitiveTopology,
}

impl Mesh {
    pub fn new(device: &wgpu::Device, vertices: &[Vertex], topology: wgpu::PrimitiveTopology) -> Self {
        let vertex_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh_vb"),
            contents: bytemuck::cast_slice(vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });
        Self { vertex_buf, vertex_count: vertices.len() as u32, topology }
    }

    /// Create a line segment from `a` to `b` with given color.
    pub fn line(device: &wgpu::Device, a: [f32; 3], b: [f32; 3], color: [f32; 4]) -> Self {
        Self::new(device, &[
            Vertex { position: a, color },
            Vertex { position: b, color },
        ], wgpu::PrimitiveTopology::LineList)
    }
}

// ---- Entity ----

pub struct Entity {
    pub mesh: Mesh,
    pub transform: Matrix4<f32>,
    pub visible: bool,
}

impl Entity {
    pub fn new(mesh: Mesh) -> Self {
        Self { mesh, transform: Matrix4::identity(), visible: true }
    }

    pub fn with_transform(mut self, t: Matrix4<f32>) -> Self {
        self.transform = t;
        self
    }
}

// ---- MeshRenderer ----

pub struct MeshRenderer {
    line_pipeline: wgpu::RenderPipeline,
    uniform_buf: wgpu::Buffer,
    uniform_bind_group: wgpu::BindGroup,
}

// VIEWPORT_Y_FLIP — same as web-splat uses internally
const VIEWPORT_Y_FLIP: Matrix4<f32> = Matrix4::new(
    1.0, 0.0, 0.0, 0.0,
    0.0, -1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
);

impl MeshRenderer {
    pub fn new(device: &wgpu::Device, surface_format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("mesh_shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("mesh_uniform_layout"),
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

        let uniform_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh_uniform_buf"),
            contents: bytemuck::cast_slice(&[Matrix4::<f32>::identity()]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("mesh_uniform_bg"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buf.as_entire_binding(),
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("mesh_pipeline_layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let vtx_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute { offset: 0, shader_location: 0, format: wgpu::VertexFormat::Float32x3 },
                wgpu::VertexAttribute { offset: 12, shader_location: 1, format: wgpu::VertexFormat::Float32x4 },
            ],
        };

        let line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("mesh_line_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[vtx_layout],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: surface_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        Self { line_pipeline, uniform_buf, uniform_bind_group }
    }

    /// Render all visible entities using the given camera.
    /// `view_matrix` and `proj_matrix` should come from `camera.view_matrix()` and `camera.proj_matrix()`.
    pub fn render(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        target: &wgpu::TextureView,
        queue: &wgpu::Queue,
        view_matrix: Matrix4<f32>,
        proj_matrix: Matrix4<f32>,
        entities: &[Entity],
    ) {
        let vp = VIEWPORT_Y_FLIP * proj_matrix * view_matrix;

        for entity in entities {
            if !entity.visible { continue; }

            let mvp = vp * entity.transform;
            queue.write_buffer(&self.uniform_buf, 0, bytemuck::cast_slice(&[mvp]));

            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("mesh_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: target,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                    depth_slice: None,
                })],
                ..Default::default()
            });
            pass.set_pipeline(&self.line_pipeline);
            pass.set_bind_group(0, &self.uniform_bind_group, &[]);
            pass.set_vertex_buffer(0, entity.mesh.vertex_buf.slice(..));
            pass.draw(0..entity.mesh.vertex_count, 0..1);
        }
    }
}

/// Create the three world-axis line entities (X=red, Y=green, Z=blue), each from 0 to `len`.
pub fn create_axis_entities(device: &wgpu::Device, len: f32) -> Vec<Entity> {
    vec![
        Entity::new(Mesh::line(device, [0.0, 0.0, 0.0], [len, 0.0, 0.0], [1.0, 0.0, 0.0, 1.0])),
        Entity::new(Mesh::line(device, [0.0, 0.0, 0.0], [0.0, len, 0.0], [0.0, 1.0, 0.0, 1.0])),
        Entity::new(Mesh::line(device, [0.0, 0.0, 0.0], [0.0, 0.0, len], [0.3, 0.5, 1.0, 1.0])),
    ]
}

/// Create a 3D cross marker at the given position (6 line segments forming a 3D cross).
pub fn create_spawn_marker(device: &wgpu::Device, pos: [f32; 3], size: f32) -> Vec<Entity> {
    let s = size * 0.5;
    let c = [1.0, 0.8, 0.0, 1.0]; // yellow
    vec![
        Entity::new(Mesh::line(device, [pos[0]-s, pos[1], pos[2]], [pos[0]+s, pos[1], pos[2]], c)),
        Entity::new(Mesh::line(device, [pos[0], pos[1]-s, pos[2]], [pos[0], pos[1]+s, pos[2]], c)),
        Entity::new(Mesh::line(device, [pos[0], pos[1], pos[2]-s], [pos[0], pos[1], pos[2]+s], c)),
    ]
}

const SHADER: &str = r#"
struct Uniforms {
    mvp: mat4x4<f32>,
};
@group(0) @binding(0) var<uniform> u: Uniforms;

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(@location(0) position: vec3<f32>, @location(1) color: vec4<f32>) -> VsOut {
    var out: VsOut;
    out.pos = u.mvp * vec4<f32>(position, 1.0);
    out.color = color;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    return in.color;
}
"#;

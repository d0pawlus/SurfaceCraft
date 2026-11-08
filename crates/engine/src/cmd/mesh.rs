//! `mesh.*`: scans and other triangle meshes.
use super::{CmdError, CmdResult, CommandSpec, ObjRef, read_file, write_file};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind, base64};
use surfacecraft_geom::{Mesh, Point3, Rng, Transform, Vec3, synth};
use surfacecraft_io::{Limits, MeshFormat, read_mesh, write_mesh};

/// Facts about a mesh.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct MeshInfo {
    /// Object id.
    pub id: ObjectId,
    /// Name.
    pub name: String,
    /// Triangle count.
    pub triangles: usize,
    /// Vertex count.
    pub vertices: usize,
    /// Bounding box minimum.
    pub bbox_min: [f64; 3],
    /// Bounding box maximum.
    pub bbox_max: [f64; 3],
    /// Surface area, mm².
    pub area: f64,
}

pub(crate) fn mesh_info(s: &Session, id: ObjectId) -> CmdResult<MeshInfo> {
    let m = s.doc.mesh(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let b = m.mesh.bbox();
    let (lo, hi) = if b.is_empty() { ([0.0; 3], [0.0; 3]) } else { (b.min.coords.into(), b.max.coords.into()) };
    Ok(MeshInfo {
        id,
        name: m.name.clone(),
        triangles: m.mesh.triangle_count(),
        vertices: m.mesh.vertex_count(),
        bbox_min: lo,
        bbox_max: hi,
        area: m.mesh.area(),
    })
}

/// Import parameters: a `path`, or inline base64 `data` with its `format`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ImportIn {
    /// File path (format from the extension unless `format` is given).
    #[serde(default)]
    pub path: Option<String>,
    /// File contents, base64 (for the web and MCP).
    #[serde(default)]
    pub data: Option<String>,
    /// `stl`, `obj` or `ply` (default: from the extension, else sniffed).
    #[serde(default)]
    pub format: Option<MeshFormat>,
    /// Scale applied on import (e.g. 25.4 for inches → mm). Default 1.
    #[serde(default)]
    pub scale: Option<f64>,
    /// Object name (default: the file name).
    #[serde(default)]
    pub name: Option<String>,
}

fn import(s: &mut Session, i: ImportIn) -> CmdResult<MeshInfo> {
    let (bytes, default_name, ext_fmt) = match (&i.path, &i.data) {
        (Some(p), None) => {
            let path = std::path::Path::new(p);
            let ext = path.extension().and_then(|e| e.to_str()).and_then(MeshFormat::from_extension);
            let stem = path.file_stem().and_then(|n| n.to_str()).unwrap_or("mesh").to_string();
            (read_file(s, p)?, stem, ext)
        }
        (None, Some(d)) => (base64::decode(d).map_err(CmdError::Params)?, "mesh".to_string(), None),
        _ => return Err(CmdError::Params("give exactly one of `path` or `data`".into())),
    };
    let fmt = i
        .format
        .or(ext_fmt)
        .or_else(|| MeshFormat::sniff(&bytes))
        .ok_or_else(|| CmdError::Params("cannot tell the mesh format; pass `format`".into()))?;
    // the browser has far less memory (design doc §11): smaller meshes on the web
    let limits = if cfg!(target_arch = "wasm32") {
        Limits { max_triangles: 5_000_000, max_vertices: 5_000_000, ..Limits::default() }
    } else {
        Limits::default()
    };
    let mut mesh = read_mesh(&bytes, fmt, &limits)?;
    if let Some(sc) = i.scale {
        if !(sc.is_finite() && sc > 0.0) {
            return Err(CmdError::Params("scale must be positive".into()));
        }
        mesh.scale(sc);
    }
    let id = s.doc.add_mesh(i.name.as_deref().unwrap_or(&default_name), mesh);
    mesh_info(s, id)
}

/// Export parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ExportIn {
    /// Mesh id or name.
    pub object: ObjRef,
    /// Output path; omitted → the bytes come back base64 in `data`.
    #[serde(default)]
    pub path: Option<String>,
    /// Format (default: from the path's extension, else STL).
    #[serde(default)]
    pub format: Option<MeshFormat>,
}

/// Export result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ExportOut {
    /// Bytes produced.
    pub bytes: usize,
    /// Path written, if any.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    /// Base64 contents when no path was given.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<String>,
}

fn export(s: &mut Session, i: ExportIn) -> CmdResult<ExportOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let m = s.doc.mesh(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let fmt = i
        .format
        .or_else(|| {
            i.path.as_deref().and_then(|p| {
                std::path::Path::new(p).extension().and_then(|e| e.to_str()).and_then(MeshFormat::from_extension)
            })
        })
        .unwrap_or(MeshFormat::Stl);
    let bytes = write_mesh(&m.mesh, fmt);
    match i.path {
        Some(p) => {
            write_file(s, &p, &bytes)?;
            Ok(ExportOut { bytes: bytes.len(), path: Some(p), data: None })
        }
        None => Ok(ExportOut { bytes: bytes.len(), path: None, data: Some(base64::encode(&bytes)) }),
    }
}

/// One mesh.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshIn {
    /// Mesh id or name.
    pub object: ObjRef,
}

fn info(s: &mut Session, i: MeshIn) -> CmdResult<MeshInfo> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    mesh_info(s, id)
}

/// A rotation about an axis.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Rotation {
    /// Axis direction.
    pub axis: [f64; 3],
    /// Angle in degrees (right-hand rule).
    pub degrees: f64,
    /// Point the axis passes through (default origin).
    #[serde(default)]
    pub center: Option<[f64; 3]>,
}

/// Transform parameters: a full `matrix`, or `rotate` then `translate`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TransformIn {
    /// Mesh id or name.
    pub object: ObjRef,
    /// Row-major rigid 4×4 matrix.
    #[serde(default)]
    pub matrix: Option<[[f64; 4]; 4]>,
    /// Rotation, applied first.
    #[serde(default)]
    pub rotate: Option<Rotation>,
    /// Translation, applied last.
    #[serde(default)]
    pub translate: Option<[f64; 3]>,
}

/// The transform applied.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct TransformOut {
    /// Object id.
    pub id: ObjectId,
    /// Row-major matrix that was applied.
    pub matrix: [[f64; 4]; 4],
}

pub(crate) fn build_transform(
    matrix: Option<[[f64; 4]; 4]>,
    rotate: Option<&Rotation>,
    translate: Option<[f64; 3]>,
) -> CmdResult<Transform> {
    let mut t = Transform::identity();
    if let Some(m) = matrix {
        t = Transform::from_rows(&m)
            .ok_or_else(|| CmdError::Params("matrix is not a finite rigid transform".into()))?;
    }
    if let Some(r) = rotate {
        let axis = Vec3::from(r.axis);
        if !surfacecraft_geom::finite_vec(&axis) || axis.norm() < 1e-12 || !r.degrees.is_finite() {
            return Err(CmdError::Params("rotation needs a finite non-zero axis and angle".into()));
        }
        let c = Vec3::from(r.center.unwrap_or([0.0; 3]));
        if !surfacecraft_geom::finite_vec(&c) {
            return Err(CmdError::Params("rotation centre must be finite".into()));
        }
        let rot = Transform::translation(c)
            .then_after(&Transform::rotation_axis_angle(&axis, r.degrees.to_radians()))
            .then_after(&Transform::translation(-c));
        t = rot.then_after(&t);
    }
    if let Some(tr) = translate {
        let v = Vec3::from(tr);
        if !surfacecraft_geom::finite_vec(&v) {
            return Err(CmdError::Params("translation must be finite".into()));
        }
        t = Transform::translation(v).then_after(&t);
    }
    Ok(t)
}

fn transform(s: &mut Session, i: TransformIn) -> CmdResult<TransformOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let t = build_transform(i.matrix, i.rotate.as_ref(), i.translate)?;
    let obj = s.doc.mesh_mut(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    obj.mesh = Arc::new(obj.mesh.transformed(&t));
    Ok(TransformOut { id, matrix: t.to_rows() })
}

/// Shapes `mesh.synth` can sample.
#[derive(Clone, Copy, Debug, Deserialize, Serialize, JsonSchema, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum SynthShape {
    /// `size[0] × size[1]` rectangle in XY.
    Plane,
    /// Sphere of radius `size[0]`.
    Sphere,
    /// Open cylinder, radius `size[0]`, height `size[1]`, along Z.
    Cylinder,
    /// Open cone frustum, radii `size[0]` → `size[1]`, height `size[2]`, along Z.
    Cone,
    /// Torus, radii `size[0]` (major) and `size[1]` (minor), around Z.
    Torus,
    /// Closed box `size`.
    Box,
    /// The alignment test part (block with a boss and a dome).
    Bracket,
    /// Freeform patch `size[0] × size[1]`, amplitude `size[2]` (z = a·sin(x/12)·cos(y/9)).
    Wave,
}

/// Synthetic scan parameters (ground truth known exactly; design doc §11.1 corpus level 1).
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SynthIn {
    /// Shape.
    pub shape: SynthShape,
    /// Dimensions (meaning per shape; defaults when omitted).
    #[serde(default)]
    pub size: Option<Vec<f64>>,
    /// Position of the shape's reference point.
    #[serde(default)]
    pub center: Option<[f64; 3]>,
    /// Grid resolution (default 64).
    #[serde(default)]
    pub resolution: Option<usize>,
    /// Gaussian noise σ along the normal, mm.
    #[serde(default)]
    pub noise: Option<f64>,
    /// Fraction of vertices turned into outliers.
    #[serde(default)]
    pub outliers: Option<f64>,
    /// Random seed.
    #[serde(default)]
    pub seed: Option<u64>,
    /// Object name.
    #[serde(default)]
    pub name: Option<String>,
}

fn synth_mesh(i: &SynthIn) -> CmdResult<Mesh> {
    let sz = i.size.clone().unwrap_or_default();
    let d = |k: usize, def: f64| sz.get(k).copied().unwrap_or(def);
    if sz.iter().any(|v| !v.is_finite() || *v <= 0.0) {
        return Err(CmdError::Params("sizes must be positive".into()));
    }
    let c = Point3::from(i.center.unwrap_or([0.0; 3]));
    if !surfacecraft_geom::finite(&c) {
        return Err(CmdError::Params("center must be finite".into()));
    }
    let n = i.resolution.unwrap_or(64).clamp(4, 2048);
    Ok(match i.shape {
        SynthShape::Plane => synth::plane(c, Vec3::x(), Vec3::y(), d(0, 50.0), d(1, 50.0), n, n),
        SynthShape::Sphere => synth::sphere(c, d(0, 20.0), n, n / 2),
        SynthShape::Cylinder => synth::cylinder(c, Vec3::z(), d(0, 10.0), d(1, 30.0), n, n / 2),
        SynthShape::Cone => synth::cone(c, Vec3::z(), d(0, 15.0), d(1, 5.0), d(2, 20.0), n, n / 2),
        SynthShape::Torus => synth::torus(c, Vec3::z(), d(0, 20.0), d(1, 5.0), n, n / 2),
        SynthShape::Box => synth::cuboid(c, Vec3::new(d(0, 40.0), d(1, 30.0), d(2, 20.0)), n / 4),
        SynthShape::Bracket => synth::bracket().transformed(&Transform::translation(c.coords)),
        SynthShape::Wave => synth::wave(c, d(0, 60.0), d(1, 40.0), d(2, 3.0), n, n * 2 / 3),
    })
}

fn synth_cmd(s: &mut Session, i: SynthIn) -> CmdResult<MeshInfo> {
    let mut m = synth_mesh(&i)?;
    let mut rng = Rng::new(i.seed.unwrap_or(1));
    if let Some(sigma) = i.noise {
        if !(sigma.is_finite() && sigma >= 0.0) {
            return Err(CmdError::Params("noise must be ≥ 0".into()));
        }
        synth::add_noise(&mut m, sigma, &mut rng);
    }
    if let Some(f) = i.outliers {
        let mag = m.bbox().diagonal() * 0.05;
        synth::add_outliers(&mut m, f, mag, &mut rng);
    }
    let default = format!("{:?}", i.shape).to_lowercase();
    let id = s.doc.add_mesh(i.name.as_deref().unwrap_or(&default), m);
    mesh_info(s, id)
}

/// `mesh.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "mesh.import",
        "Import mesh…",
        "Mesh",
        true,
        "Imports STL/OBJ/PLY from `path` or base64 `data` (+`format`), optional `scale`, `name`.",
        ImportIn,
        MeshInfo,
        import
    ),
    command!(
        "mesh.export",
        "Export mesh…",
        "Mesh",
        false,
        "Writes `object` as STL/OBJ/PLY to `path`, or returns base64 `data`.",
        ExportIn,
        ExportOut,
        export
    ),
    command!(
        "mesh.info",
        "Mesh info",
        "Mesh",
        false,
        "Counts, bounding box and area of `object`.",
        MeshIn,
        MeshInfo,
        info
    ),
    command!(
        "mesh.transform",
        "Transform mesh",
        "Mesh",
        true,
        "Moves `object` by `matrix`, or `rotate` {axis, degrees, center} then `translate`.",
        TransformIn,
        TransformOut,
        transform
    ),
    command!(
        "mesh.synth",
        "Synthetic scan",
        "Mesh",
        true,
        "Samples a known `shape` (plane/sphere/cylinder/cone/torus/box/bracket) with optional `noise` and `outliers`.",
        SynthIn,
        MeshInfo,
        synth_cmd
    ),
];

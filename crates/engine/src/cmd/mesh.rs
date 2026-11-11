//! `mesh.*`: scans and other triangle meshes.
use super::{CmdError, CmdResult, CommandSpec, ObjRef, read_file, write_file};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind, base64};
use surfacecraft_geom::reduce::{self, DeviationStats, ToleranceOptions, ToleranceReport};
use surfacecraft_geom::{Mesh, Point3, PointCloud, ReconParams, Rng, Transform, Vec3, edit, reconstruct, synth};
use surfacecraft_io::{Limits, MeshFormat, Scan, read_points, read_scan, write_mesh};

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

/// Point-cloud reconstruction settings (every field optional).
#[derive(Clone, Debug, Default, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ReconstructOpts {
    /// First ball radius as a multiple of the mean point spacing (0.5 - 50, default 2.5); two
    /// more passes double it each time. Larger closes bigger gaps, smaller keeps thin features.
    #[serde(default)]
    pub radius_scale: Option<f64>,
    /// Neighbours used for normal estimation (3 - 64, default 12).
    #[serde(default)]
    pub normals_k: Option<usize>,
    /// Ball-pivoting passes (1 - 6, default 3).
    #[serde(default)]
    pub passes: Option<usize>,
    /// Holes up to this many boundary edges are filled afterwards (default 24, 0 = none).
    #[serde(default)]
    pub fill_holes_max_edges: Option<usize>,
}

impl ReconstructOpts {
    fn params(&self) -> ReconParams {
        let d = ReconParams::default();
        let max_points = if cfg!(target_arch = "wasm32") { 5_000_000 } else { d.max_points };
        ReconParams {
            normals_k: self.normals_k.unwrap_or(d.normals_k),
            radius_scale: self.radius_scale.unwrap_or(d.radius_scale),
            passes: self.passes.unwrap_or(d.passes),
            fill_holes_max_edges: self.fill_holes_max_edges.unwrap_or(d.fill_holes_max_edges),
            max_points,
        }
    }
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
    /// `stl`, `obj`, `ply` or `xyz` (default: from the extension, else sniffed).
    #[serde(default)]
    pub format: Option<MeshFormat>,
    /// Scale applied on import (e.g. 25.4 for inches to mm). Default 1.
    #[serde(default)]
    pub scale: Option<f64>,
    /// Object name (default: the file name).
    #[serde(default)]
    pub name: Option<String>,
    /// Settings used when the file is a point cloud (PLY/OBJ without faces, XYZ): it is
    /// reconstructed into a mesh on import.
    #[serde(default)]
    pub reconstruct: Option<ReconstructOpts>,
}

/// What reconstructing a point cloud did.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct CloudInfo {
    /// Points in the file.
    pub points: usize,
    /// Exact duplicate points dropped.
    pub duplicates_removed: usize,
    /// Mean nearest-neighbour spacing, mm.
    pub spacing: f64,
    /// Ball radii used, mm.
    pub radii: Vec<f64>,
    /// `given` (normals from the file) or `estimated`.
    pub normals: String,
    /// Triangles built by ball pivoting.
    pub pivoted_triangles: usize,
    /// Points no ball rested on (left out).
    pub unused_points: usize,
    /// Isolated specks removed, triangles.
    pub specks_removed: usize,
    /// Holes filled.
    pub holes_filled: usize,
    /// Boundary loops left.
    pub holes_left: usize,
}

/// Result of `mesh.import` / `mesh.reconstruct`: the mesh, plus how it came from points.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ImportOut {
    /// The mesh object.
    #[serde(flatten)]
    pub mesh: MeshInfo,
    /// Present when the input was a point cloud.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub point_cloud: Option<CloudInfo>,
}

type Loaded = (Vec<u8>, String, MeshFormat);

fn load(s: &Session, path: &Option<String>, data: &Option<String>, format: Option<MeshFormat>) -> CmdResult<Loaded> {
    let (bytes, default_name, ext_fmt) = match (path, data) {
        (Some(p), None) => {
            let path = std::path::Path::new(p);
            let ext = path.extension().and_then(|e| e.to_str()).and_then(MeshFormat::from_extension);
            let stem = path.file_stem().and_then(|n| n.to_str()).unwrap_or("mesh").to_string();
            (read_file(s, p)?, stem, ext)
        }
        (None, Some(d)) => (base64::decode(d).map_err(CmdError::Params)?, "mesh".to_string(), None),
        _ => return Err(CmdError::Params("give exactly one of `path` or `data`".into())),
    };
    let fmt = format
        .or(ext_fmt)
        .or_else(|| MeshFormat::sniff(&bytes))
        .ok_or_else(|| CmdError::Params("cannot tell the mesh format; pass `format`".into()))?;
    Ok((bytes, default_name, fmt))
}

/// The browser has far less memory (design doc section 11): smaller meshes on the web.
fn read_limits() -> Limits {
    if cfg!(target_arch = "wasm32") {
        Limits { max_triangles: 5_000_000, max_vertices: 5_000_000, ..Limits::default() }
    } else {
        Limits::default()
    }
}

fn scaled(mut mesh: Mesh, scale: Option<f64>) -> CmdResult<Mesh> {
    if let Some(sc) = scale {
        if !(sc.is_finite() && sc > 0.0) {
            return Err(CmdError::Params("scale must be positive".into()));
        }
        mesh.scale(sc);
    }
    Ok(mesh)
}

fn from_cloud(mut cloud: PointCloud, scale: Option<f64>, opts: &ReconstructOpts) -> CmdResult<(Mesh, CloudInfo)> {
    if let Some(sc) = scale {
        if !(sc.is_finite() && sc > 0.0) {
            return Err(CmdError::Params("scale must be positive".into()));
        }
        for p in &mut cloud.positions {
            *p = Point3::from(p.coords * sc);
        }
    }
    let (mesh, r) = reconstruct(&cloud, &opts.params()).map_err(|e| CmdError::Failed(format!("point cloud: {e}")))?;
    let info = CloudInfo {
        points: r.points,
        duplicates_removed: r.duplicates_removed,
        spacing: r.spacing,
        radii: r.radii,
        normals: if r.normals_given { "given" } else { "estimated" }.to_string(),
        pivoted_triangles: r.pivoted_triangles,
        unused_points: r.unused_points,
        specks_removed: r.specks_removed,
        holes_filled: r.holes_filled,
        holes_left: r.holes_left,
    };
    Ok((mesh, info))
}

fn import(s: &mut Session, i: ImportIn) -> CmdResult<ImportOut> {
    let (bytes, default_name, fmt) = load(s, &i.path, &i.data, i.format)?;
    let name = i.name.clone().unwrap_or(default_name);
    let (mesh, cloud) = match read_scan(&bytes, fmt, &read_limits())? {
        Scan::Mesh(m) => (scaled(m, i.scale)?, None),
        Scan::Points(c) => {
            let (m, info) = from_cloud(c, i.scale, &i.reconstruct.clone().unwrap_or_default())?;
            (m, Some(info))
        }
    };
    let id = s.doc.add_mesh(&name, mesh);
    Ok(ImportOut { mesh: mesh_info(s, id)?, point_cloud: cloud })
}

/// Reconstruction parameters: a file as for `mesh.import`, whose points (vertices, when it has
/// faces) become a new mesh.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ReconstructIn {
    /// File path (format from the extension unless `format` is given).
    #[serde(default)]
    pub path: Option<String>,
    /// File contents, base64.
    #[serde(default)]
    pub data: Option<String>,
    /// `stl`, `obj`, `ply` or `xyz` (default: from the extension, else sniffed).
    #[serde(default)]
    pub format: Option<MeshFormat>,
    /// Scale applied to the points. Default 1.
    #[serde(default)]
    pub scale: Option<f64>,
    /// Object name (default: the file name).
    #[serde(default)]
    pub name: Option<String>,
    /// First ball radius as a multiple of the mean point spacing (0.5 - 50, default 2.5).
    #[serde(default)]
    pub radius_scale: Option<f64>,
    /// Neighbours used for normal estimation (3 - 64, default 12).
    #[serde(default)]
    pub normals_k: Option<usize>,
    /// Ball-pivoting passes (1 - 6, default 3).
    #[serde(default)]
    pub passes: Option<usize>,
    /// Holes up to this many boundary edges are filled afterwards (default 24, 0 = none).
    #[serde(default)]
    pub fill_holes_max_edges: Option<usize>,
}

fn reconstruct_cmd(s: &mut Session, i: ReconstructIn) -> CmdResult<ImportOut> {
    let (bytes, default_name, fmt) = load(s, &i.path, &i.data, i.format)?;
    let cloud = read_points(&bytes, fmt, &read_limits())?;
    let opts = ReconstructOpts {
        radius_scale: i.radius_scale,
        normals_k: i.normals_k,
        passes: i.passes,
        fill_holes_max_edges: i.fill_holes_max_edges,
    };
    let (mesh, info) = from_cloud(cloud, i.scale, &opts)?;
    let id = s.doc.add_mesh(i.name.as_deref().unwrap_or(&default_name), mesh);
    Ok(ImportOut { mesh: mesh_info(s, id)?, point_cloud: Some(info) })
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

/// Result of a mesh edit.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct EditOut {
    /// Object id.
    pub id: ObjectId,
    /// Triangles before.
    pub triangles_before: usize,
    /// Triangles after.
    pub triangles: usize,
    /// Triangles removed.
    pub removed_triangles: usize,
    /// Triangles added.
    pub added_triangles: usize,
    /// Holes filled.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub holes_filled: Option<usize>,
    /// Holes left open (above the size limit).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub holes_left: Option<usize>,
    /// Largest vertex displacement (smoothing), mm.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_displacement: Option<f64>,
    /// Regions of the mesh dropped because its triangles changed.
    pub regions_dropped: usize,
    /// Measured deviation of a tolerance-driven reduction (`mesh.decimate` with `tolerance`).
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub tolerance_report: Option<ToleranceOut>,
}

/// One direction of the measured deviation, mm.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct DeviationOut {
    /// Samples measured.
    pub samples: usize,
    /// Largest distance.
    pub max: f64,
    /// 99th percentile distance.
    pub p99: f64,
    /// Share of the samples within the tolerance, in [0, 1].
    pub share_within: f64,
}

impl From<DeviationStats> for DeviationOut {
    fn from(d: DeviationStats) -> Self {
        Self { samples: d.samples, max: d.max, p99: d.p99, share_within: d.share_within }
    }
}

/// What a tolerance-driven reduction measured, both ways.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ToleranceOut {
    /// Requested tolerance, mm.
    pub tolerance: f64,
    /// Requested share of the samples within the tolerance.
    pub share: f64,
    /// Original vertices to the reduced surface.
    pub original_to_reduced: DeviationOut,
    /// Reduced vertices and triangle centroids to the original surface.
    pub reduced_to_original: DeviationOut,
    /// Triangles before.
    pub triangles_before: usize,
    /// Triangles after.
    pub triangles_after: usize,
    /// Reduction rounds run (1 = the first reduction already met the tolerance).
    pub iterations: usize,
    /// Did both directions reach `share` with no sample beyond twice the tolerance?
    pub met: bool,
}

impl ToleranceOut {
    fn new(r: ToleranceReport, tolerance: f64, share: f64) -> Self {
        Self {
            tolerance,
            share,
            original_to_reduced: r.original_to_reduced.into(),
            reduced_to_original: r.reduced_to_original.into(),
            triangles_before: r.triangles_before,
            triangles_after: r.triangles_after,
            iterations: r.iterations,
            met: r.met,
        }
    }
}

/// Runs `f` on a copy of the mesh and stores the result (regions dropped when triangles changed).
fn edit_mesh(
    s: &mut Session,
    object: &ObjRef,
    f: impl FnOnce(&mut Mesh, &mut EditOut) -> CmdResult<()>,
) -> CmdResult<EditOut> {
    let id = object.resolve(s, ObjectKind::Mesh)?;
    let obj = s.doc.mesh(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let mut m: Mesh = (*obj.mesh).clone();
    let mut o = EditOut {
        id,
        triangles_before: m.triangle_count(),
        triangles: 0,
        removed_triangles: 0,
        added_triangles: 0,
        holes_filled: None,
        holes_left: None,
        max_displacement: None,
        regions_dropped: 0,
        tolerance_report: None,
    };
    let old_tris = m.triangles.clone();
    f(&mut m, &mut o)?;
    o.triangles = m.triangle_count();
    if m.triangles != old_tris {
        o.regions_dropped = s.doc.replace_mesh_topology(id, m).unwrap_or(0);
    } else if let Some(obj) = s.doc.mesh_mut(id) {
        obj.mesh = Arc::new(m);
    }
    s.overlays.remove(&id);
    Ok(o)
}

fn positive(v: Option<f64>, name: &str, default: f64) -> CmdResult<f64> {
    match v {
        None => Ok(default),
        Some(x) if x.is_finite() && x > 0.0 => Ok(x),
        Some(_) => Err(CmdError::Params(format!("{name} must be a positive number"))),
    }
}

fn fraction(v: Option<f64>, name: &str, default: f64) -> CmdResult<f64> {
    match v {
        None => Ok(default),
        Some(x) if x.is_finite() && (0.0..=1.0).contains(&x) => Ok(x),
        Some(_) => Err(CmdError::Params(format!("{name} must be in [0, 1]"))),
    }
}

/// Clean-up parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CleanIn {
    /// Mesh id or name.
    pub object: ObjRef,
    /// Components smaller than this fraction of the largest are removed (default 0.01).
    #[serde(default)]
    pub min_component_fraction: Option<f64>,
    /// Components with fewer triangles are removed (default 20).
    #[serde(default)]
    pub min_component_triangles: Option<usize>,
    /// Spike threshold: distance off the one-ring plane over the ring radius (default 1.5;
    /// `0` skips outlier removal).
    #[serde(default)]
    pub outlier_ratio: Option<f64>,
    /// Also fill holes up to this many edges (default: leave holes).
    #[serde(default)]
    pub fill_holes_max_edges: Option<usize>,
}

fn clean(s: &mut Session, i: CleanIn) -> CmdResult<EditOut> {
    let frac = fraction(i.min_component_fraction, "min_component_fraction", 0.01)?;
    let ratio = i.outlier_ratio.unwrap_or(1.5);
    if !(ratio.is_finite() && ratio >= 0.0) {
        return Err(CmdError::Params("outlier_ratio must be ≥ 0".into()));
    }
    edit_mesh(s, &i.object, |m, o| {
        let mut removed = m.remove_degenerate(0.0);
        if ratio > 0.0 {
            removed += edit::remove_outliers(m, ratio);
        }
        removed += edit::remove_small_components(m, frac, i.min_component_triangles.unwrap_or(20));
        o.removed_triangles = removed;
        if let Some(n) = i.fill_holes_max_edges {
            let before = m.triangle_count();
            let r = edit::fill_holes(m, n, true);
            o.added_triangles = m.triangle_count() - before;
            o.holes_filled = Some(r.filled);
            o.holes_left = Some(r.skipped);
        }
        edit::compact(m);
        Ok(())
    })
}

/// Outlier removal parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct OutliersIn {
    /// Mesh id or name.
    pub object: ObjRef,
    /// Distance off the one-ring plane over the ring radius above which a vertex is a spike
    /// (default 1.5).
    #[serde(default)]
    pub ratio: Option<f64>,
}

fn outliers(s: &mut Session, i: OutliersIn) -> CmdResult<EditOut> {
    let ratio = positive(i.ratio, "ratio", 1.5)?;
    edit_mesh(s, &i.object, |m, o| {
        o.removed_triangles = edit::remove_outliers(m, ratio);
        Ok(())
    })
}

/// Small component removal parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SmallIn {
    /// Mesh id or name.
    pub object: ObjRef,
    /// Fraction of the largest component below which a component goes (default 0.01).
    #[serde(default)]
    pub min_fraction: Option<f64>,
    /// Triangle count below which a component goes (default 20).
    #[serde(default)]
    pub min_triangles: Option<usize>,
}

fn small(s: &mut Session, i: SmallIn) -> CmdResult<EditOut> {
    let frac = fraction(i.min_fraction, "min_fraction", 0.01)?;
    edit_mesh(s, &i.object, |m, o| {
        o.removed_triangles = edit::remove_small_components(m, frac, i.min_triangles.unwrap_or(20));
        Ok(())
    })
}

/// Hole filling parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FillIn {
    /// Mesh id or name.
    pub object: ObjRef,
    /// Largest hole filled, in boundary edges (default 300, at most 4000).
    #[serde(default)]
    pub max_edges: Option<usize>,
    /// Curvature-following fill (refined, thin-plate faired); `false` = flat (default true).
    #[serde(default)]
    pub smooth: Option<bool>,
}

fn fill(s: &mut Session, i: FillIn) -> CmdResult<EditOut> {
    edit_mesh(s, &i.object, |m, o| {
        let before = m.triangle_count();
        let r = edit::fill_holes(m, i.max_edges.unwrap_or(300), i.smooth.unwrap_or(true));
        o.added_triangles = m.triangle_count() - before;
        o.holes_filled = Some(r.filled);
        o.holes_left = Some(r.skipped);
        Ok(())
    })
}

/// Decimation parameters: a `target` triangle count or a `ratio` of the current count.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DecimateIn {
    /// Mesh id or name.
    pub object: ObjRef,
    /// Triangles to keep.
    #[serde(default)]
    pub target: Option<usize>,
    /// Fraction of triangles to keep, (0, 1] (default 0.5 when `target` is not given).
    #[serde(default)]
    pub ratio: Option<f64>,
    /// Never move the surface more than this (mm, approximate).
    #[serde(default)]
    pub max_error: Option<f64>,
    /// Edges sharper than this stay in place (degrees, default 40).
    #[serde(default)]
    pub feature_angle_deg: Option<f64>,
    /// Keep the open boundary in place (default true).
    #[serde(default)]
    pub preserve_boundary: Option<bool>,
    /// Verified tolerance, mm: reduce, then measure original vertices to the reduced surface and
    /// reduced vertices and triangle centroids to the original surface, and keep detail where the
    /// tolerance is exceeded until `share` of the samples is within it in both directions (and none
    /// is beyond twice the tolerance). Without `target` and `ratio` the mesh is reduced as far as the
    /// tolerance allows; with one of them it is a floor. The measured numbers are returned in
    /// `tolerance_report`.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Share of the samples that must be within `tolerance`, (0, 1] (default 0.999).
    #[serde(default)]
    pub share: Option<f64>,
}

fn decimate(s: &mut Session, i: DecimateIn) -> CmdResult<EditOut> {
    let ratio = i.ratio.unwrap_or(0.5);
    if !(ratio.is_finite() && ratio > 0.0 && ratio <= 1.0) {
        return Err(CmdError::Params("ratio must be in (0, 1]".into()));
    }
    if i.target == Some(0) {
        // like a ratio of zero: it would erase the scan
        return Err(CmdError::Params("target must be at least one triangle".into()));
    }
    if let Some(e) = i.max_error
        && !(e.is_finite() && e > 0.0)
    {
        return Err(CmdError::Params("max_error must be positive".into()));
    }
    let angle = positive(i.feature_angle_deg, "feature_angle_deg", 40.0)?;
    let tolerance = match i.tolerance {
        None => None,
        Some(t) if t.is_finite() && t > 0.0 => Some(t),
        Some(_) => return Err(CmdError::Params("tolerance must be a positive number".into())),
    };
    let share = match (i.share, tolerance) {
        (None, _) => 0.999,
        (Some(x), Some(_)) if x.is_finite() && x > 0.0 && x <= 1.0 => x,
        (Some(_), Some(_)) => return Err(CmdError::Params("share must be in (0, 1]".into())),
        (Some(_), None) => return Err(CmdError::Params("share needs a tolerance".into())),
    };
    edit_mesh(s, &i.object, |m, o| {
        let before = m.triangle_count();
        let target = match (i.target, i.ratio, tolerance) {
            (Some(t), _, _) => t,
            (None, Some(r), _) => (before as f64 * r) as usize,
            (None, None, Some(_)) => 0,
            (None, None, None) => (before as f64 * ratio) as usize,
        };
        let opts = edit::DecimateOptions {
            target_triangles: target,
            max_error: i.max_error,
            feature_angle_deg: angle,
            preserve_boundary: i.preserve_boundary.unwrap_or(true),
        };
        if let Some(tol) = tolerance {
            let want = ToleranceOptions { share, ..ToleranceOptions::new(tol) };
            let report = reduce::reduce_to_tolerance(m, &opts, &want)
                .ok_or_else(|| CmdError::Params("the mesh is not valid (indices or coordinates)".into()))?;
            o.removed_triangles = before.saturating_sub(m.triangle_count());
            o.tolerance_report = Some(ToleranceOut::new(report, tol, share));
        } else {
            o.removed_triangles = edit::decimate(m, &opts);
        }
        Ok(())
    })
}

/// Smoothing parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SmoothIn {
    /// Mesh id or name.
    pub object: ObjRef,
    /// Taubin iterations (default 5, at most 1000).
    #[serde(default)]
    pub iterations: Option<usize>,
    /// Keep boundary vertices in place (default true).
    #[serde(default)]
    pub fix_boundary: Option<bool>,
    /// Vertices on edges sharper than this stay (degrees, default 40; 180 smooths everything).
    #[serde(default)]
    pub feature_angle_deg: Option<f64>,
}

fn smooth(s: &mut Session, i: SmoothIn) -> CmdResult<EditOut> {
    let angle = positive(i.feature_angle_deg, "feature_angle_deg", 40.0)?;
    edit_mesh(s, &i.object, |m, o| {
        let feat = if angle >= 180.0 { None } else { Some(angle) };
        o.max_displacement = Some(edit::smooth(m, i.iterations.unwrap_or(5), i.fix_boundary.unwrap_or(true), feat));
        Ok(())
    })
}

/// `mesh.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "mesh.import",
        "Import mesh…",
        "Mesh",
        true,
        "Imports STL/OBJ/PLY/XYZ from `path` or base64 `data` (+`format`), optional `scale`, `name`; a point cloud (no faces) is reconstructed into a mesh (`reconstruct` settings).",
        ImportIn,
        ImportOut,
        import
    ),
    command!(
        "mesh.reconstruct",
        "Reconstruct from points...",
        "Mesh",
        true,
        "Builds a mesh from the points of a file (PLY/OBJ/XYZ point cloud, or a mesh's vertices): normals by PCA, ball pivoting with `radius_scale` x point spacing, small holes filled.",
        ReconstructIn,
        ImportOut,
        reconstruct_cmd
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
    command!(
        "mesh.clean",
        "Clean scan",
        "Mesh",
        true,
        "Removes degenerate triangles, spikes (`outlier_ratio`) and small components; optionally fills holes up to `fill_holes_max_edges`.",
        CleanIn,
        EditOut,
        clean
    ),
    command!(
        "mesh.remove_outliers",
        "Remove spikes",
        "Mesh",
        true,
        "Removes triangles at vertices farther than `ratio` × ring radius off their one-ring plane.",
        OutliersIn,
        EditOut,
        outliers
    ),
    command!(
        "mesh.remove_small",
        "Remove small pieces",
        "Mesh",
        true,
        "Removes components below `min_fraction` of the largest or `min_triangles`.",
        SmallIn,
        EditOut,
        small
    ),
    command!(
        "mesh.fill_holes",
        "Fill holes",
        "Mesh",
        true,
        "Fills boundary loops up to `max_edges`; `smooth` follows the surrounding curvature, else flat.",
        FillIn,
        EditOut,
        fill
    ),
    command!(
        "mesh.decimate",
        "Decimate",
        "Mesh",
        true,
        "Reduces to `target` triangles or `ratio`, within `max_error` mm, keeping edges sharper than `feature_angle_deg`. With `tolerance` (mm) the reduction is measured both ways and refined locally until `share` (default 0.999) of the samples is within it; `tolerance_report` has max, p99 and share per direction, triangle counts and rounds.",
        DecimateIn,
        EditOut,
        decimate
    ),
    command!(
        "mesh.smooth",
        "Smooth",
        "Mesh",
        true,
        "Taubin smoothing, `iterations`, keeping the boundary and edges sharper than `feature_angle_deg`.",
        SmoothIn,
        EditOut,
        smooth
    ),
];

//! `scan.qa` and `body.compare`, the two reports read before modelling. `scan.qa` says what to clean
//! in a scan (holes, debris, bad triangles, noise); `body.compare` says where a body departs from the
//! scan, face by face.
use super::deviation::{CHORD_BOUNDS, NOISE_PRIMITIVE_MM, NOISE_WARNING_MM, REACH, TOLERANCE, positive, yes};
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_deviation::faces::{FaceDeviation, per_face, worst_first};
use surfacecraft_deviation::{DeviationStats, Sample, noise_estimate, samples_mesh_tagged, stats_of};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::edit::{boundary_loops, components};
use surfacecraft_geom::quality::{LengthStats, degenerate_count, duplicate_count, edge_census};
use surfacecraft_geom::{Aabb, Bvh, Mesh};
use surfacecraft_kernel::tess::{TopoMesh, tessellate_topo};

/// Holes listed by `scan.qa`, largest first (the count is always complete).
const MAX_LISTED_HOLES: usize = 50;
/// Default chord tolerance of a body reference, as a fraction of the colour-map tolerance (as in
/// `deviation.compute`).
const CHORD_FRACTION: f64 = 0.05;

/// One scan to check.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct QaIn {
    /// Scan mesh id or name.
    pub object: ObjRef,
}

/// How serious a finding is; the variants are in increasing order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Severity {
    /// Worth knowing, not a defect.
    Info,
    /// A defect to clean before modelling.
    Warning,
    /// Blocks modelling until it is fixed.
    Error,
}

/// One finding of the quality report.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct Finding {
    /// How serious it is.
    pub severity: Severity,
    /// Short identifier, stable across versions.
    pub code: String,
    /// What was found and what to do about it.
    pub message: String,
}

/// One open boundary loop.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct HoleOut {
    /// Vertices on the loop.
    pub vertices: usize,
    /// Diagonal of the loop's bounding box (mm): its size.
    pub diameter: f64,
}

/// Lengths of the distinct edges (mm).
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct EdgeLengthOut {
    /// Shortest.
    pub min: f64,
    /// Median.
    pub median: f64,
    /// Mean.
    pub mean: f64,
    /// Longest.
    pub max: f64,
}

impl From<LengthStats> for EdgeLengthOut {
    fn from(l: LengthStats) -> Self {
        EdgeLengthOut { min: l.min, median: l.median, mean: l.mean, max: l.max }
    }
}

/// Quality report of a scan mesh.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct QaOut {
    /// Scan mesh.
    pub id: ObjectId,
    /// Vertices.
    pub vertices: usize,
    /// Triangles.
    pub triangles: usize,
    /// Lower corner of the bounding box (mm).
    pub bbox_min: [f64; 3],
    /// Upper corner of the bounding box (mm).
    pub bbox_max: [f64; 3],
    /// Surface area (mm²).
    pub area: f64,
    /// Vertices per mm² of surface.
    pub density: f64,
    /// Distinct edges.
    pub edges: usize,
    /// Edges used by one triangle.
    pub boundary_edges: usize,
    /// Edges used by three or more triangles.
    pub non_manifold_edges: usize,
    /// Connected pieces (through shared vertices), the largest included.
    pub components: usize,
    /// Triangles outside the largest piece.
    pub debris_triangles: usize,
    /// Open boundary loops (holes).
    pub holes: usize,
    /// The holes, largest first (at most 50).
    pub hole_list: Vec<HoleOut>,
    /// Flat triangles or triangles with a repeated corner.
    pub degenerate_triangles: usize,
    /// Triangles that repeat the corners of an earlier triangle.
    pub duplicate_triangles: usize,
    /// Estimated scanner noise σ (mm); null when the estimate rests on too few smooth samples (sharp
    /// edges or a coarse mesh), as the number would then be inflated.
    pub noise_sigma: Option<f64>,
    /// Edge lengths (mm): the sampling density of the scan.
    pub edge_length: EdgeLengthOut,
    /// Findings, most serious first; empty when the scan has nothing to clean.
    pub findings: Vec<Finding>,
}

/// Body and scan to compare.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CompareIn {
    /// Body id or name.
    pub body: ObjRef,
    /// Scan mesh id or name.
    pub scan: ObjRef,
    /// Tolerance (mm, default 0.1): the share of points within it.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Scan points farther than this from the body are not measured (mm, default 5).
    #[serde(default)]
    pub max_distance: Option<f64>,
    /// Chord tolerance of the body's tessellation (mm, default 0.005, which is 0.05 × tolerance as in
    /// `deviation.compute`).
    #[serde(default)]
    pub chord: Option<f64>,
    /// Leave out points whose normal points away from the face normal (default true).
    #[serde(default = "yes")]
    pub facing: bool,
}

/// Body against scan, per B-rep face.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct CompareOut {
    /// Body.
    pub body: ObjectId,
    /// Scan mesh.
    pub scan: ObjectId,
    /// Tolerance used (mm).
    pub tolerance: f64,
    /// Chord tolerance used for the body's tessellation (mm).
    pub chord: f64,
    /// Faces of the body.
    pub faces: usize,
    /// Scan points farther than `max_distance` from the body.
    pub unmeasured: usize,
    /// Measured points whose nearest triangle has no face (the tessellation gave no face map).
    pub unattributed: usize,
    /// Statistics over every counted point.
    pub totals: DeviationStats,
    /// One row per face, worst first (largest RMS first; faces with no points last).
    pub per_face: Vec<FaceDeviation>,
}

/// The numbers the findings are judged by.
struct Facts {
    valid: bool,
    triangles: usize,
    non_manifold: usize,
    holes: usize,
    debris_pieces: usize,
    debris_triangles: usize,
    degenerate: usize,
    duplicates: usize,
    noise: Option<f64>,
}

fn finding(severity: Severity, code: &str, message: String) -> Finding {
    Finding { severity, code: code.to_string(), message }
}

/// Finding on the scanner noise; none for an empty or invalid mesh, which has no noise to judge.
fn noise_finding(f: &Facts) -> Option<Finding> {
    if !f.valid || f.triangles == 0 {
        return None;
    }
    let Some(sigma) = f.noise else {
        let msg = "the scanner noise cannot be estimated: too few flat-sampled points (sharp edges or a coarse mesh)";
        return Some(finding(Severity::Info, "noise_unknown", msg.into()));
    };
    if sigma > NOISE_PRIMITIVE_MM {
        let msg = format!("estimated noise σ = {sigma:.3} mm is above the {NOISE_PRIMITIVE_MM} mm primitive tolerance");
        Some(finding(Severity::Warning, "noise_above_primitive_tolerance", msg))
    } else if sigma > NOISE_WARNING_MM {
        let msg = format!("estimated noise σ = {sigma:.3} mm is above the {NOISE_WARNING_MM} mm warning level");
        Some(finding(Severity::Info, "noise_above_warning_level", msg))
    } else {
        None
    }
}

/// The findings for `f`, most serious first (stable within a severity).
fn findings(f: &Facts) -> Vec<Finding> {
    let checks = [
        (!f.valid).then(|| {
            finding(
                Severity::Error,
                "invalid_mesh",
                "indices or coordinates are out of range; the counts are not reliable".into(),
            )
        }),
        (f.valid && f.triangles == 0)
            .then(|| finding(Severity::Error, "empty_mesh", "the mesh has no triangles".into())),
        (f.non_manifold > 0).then(|| {
            let msg =
                format!("{} edges are shared by three or more triangles; remove the extra triangles", f.non_manifold);
            finding(Severity::Error, "non_manifold_edges", msg)
        }),
        (f.holes > 0).then(|| {
            let msg = format!("{} open boundary loops (holes); fill them or trim the rim", f.holes);
            finding(Severity::Warning, "holes", msg)
        }),
        (f.debris_pieces > 0).then(|| {
            let msg = format!("{} pieces outside the main body hold {} triangles", f.debris_pieces, f.debris_triangles);
            finding(Severity::Warning, "debris", msg)
        }),
        (f.degenerate > 0).then(|| {
            finding(
                Severity::Warning,
                "degenerate_triangles",
                format!("{} flat or repeated-corner triangles", f.degenerate),
            )
        }),
        (f.duplicates > 0).then(|| {
            finding(
                Severity::Warning,
                "duplicate_triangles",
                format!("{} triangles repeat an earlier triangle", f.duplicates),
            )
        }),
        noise_finding(f),
    ];
    let mut out: Vec<Finding> = checks.into_iter().flatten().collect();
    out.sort_by_key(|f| std::cmp::Reverse(f.severity));
    out
}

/// Bounding box corners of `mesh` (zeros when it has no vertices).
fn bbox_corners(mesh: &Mesh) -> ([f64; 3], [f64; 3]) {
    let bb = mesh.bbox();
    if bb.is_empty() {
        return ([0.0; 3], [0.0; 3]);
    }
    ([bb.min.x, bb.min.y, bb.min.z], [bb.max.x, bb.max.y, bb.max.z])
}

/// Diagonal of the bounding box of the points of a loop (mm).
fn loop_diameter(mesh: &Mesh, lp: &[u32]) -> f64 {
    let bb = Aabb::from_points(lp.iter().filter_map(|&v| mesh.positions.get(v as usize)));
    if bb.is_empty() { 0.0 } else { bb.diagonal() }
}

/// The holes, largest first, at most [`MAX_LISTED_HOLES`].
fn hole_list(mesh: &Mesh, loops: &[Vec<u32>]) -> Vec<HoleOut> {
    let mut holes: Vec<HoleOut> =
        loops.iter().map(|lp| HoleOut { vertices: lp.len(), diameter: loop_diameter(mesh, lp) }).collect();
    holes.sort_by(|a, b| b.vertices.cmp(&a.vertices).then(b.diameter.total_cmp(&a.diameter)));
    holes.truncate(MAX_LISTED_HOLES);
    holes
}

pub(crate) fn scan_qa(s: &Session, i: QaIn) -> CmdResult<QaOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let mesh = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let valid = mesh.validate().is_ok();
    let census = edge_census(&mesh);
    let pieces = components(&mesh);
    let loops = boundary_loops(&mesh);
    let debris_triangles: usize = pieces.iter().skip(1).map(Vec::len).sum();
    let degenerate = degenerate_count(&mesh);
    let duplicates = duplicate_count(&mesh);
    let estimate = valid.then(|| noise_estimate(&mesh)).filter(|e| e.smooth);
    let noise = estimate.map(|e| e.sigma);
    let facts = Facts {
        valid,
        triangles: mesh.triangle_count(),
        non_manifold: census.non_manifold,
        holes: loops.len(),
        debris_pieces: pieces.len().saturating_sub(1),
        debris_triangles,
        degenerate,
        duplicates,
        noise,
    };
    let (bbox_min, bbox_max) = bbox_corners(&mesh);
    let area = mesh.area();
    let vertices = mesh.vertex_count();
    Ok(QaOut {
        id,
        vertices,
        triangles: mesh.triangle_count(),
        bbox_min,
        bbox_max,
        area,
        density: if area > 0.0 { vertices as f64 / area } else { 0.0 },
        edges: census.edges,
        boundary_edges: census.boundary,
        non_manifold_edges: census.non_manifold,
        components: pieces.len(),
        debris_triangles,
        holes: loops.len(),
        hole_list: hole_list(&mesh, &loops),
        degenerate_triangles: degenerate,
        duplicate_triangles: duplicates,
        noise_sigma: noise,
        edge_length: census.lengths.into(),
        findings: findings(&facts),
    })
}

/// Chord tolerance (mm) of the body tessellation: the request, or the default, within bounds.
fn chord_of(chord: Option<f64>, tol: f64) -> CmdResult<f64> {
    let c = match chord {
        Some(c) => positive(c, "chord")?,
        None => tol * CHORD_FRACTION,
    };
    Ok(c.clamp(CHORD_BOUNDS.0, CHORD_BOUNDS.1))
}

/// The B-rep face of the triangle `t` of the tessellation, when the tessellation has a face map.
fn face_of(topo: &TopoMesh, t: Option<usize>) -> Option<usize> {
    t.and_then(|t| topo.tri_face.get(t)).map(|&f| f as usize)
}

pub(crate) fn compare(s: &Session, i: CompareIn) -> CmdResult<CompareOut> {
    let tol = positive(i.tolerance.unwrap_or(TOLERANCE), "tolerance")?;
    let maxd = positive(i.max_distance.unwrap_or(REACH), "max_distance")?;
    let chord = chord_of(i.chord, tol)?;
    let body_id = i.body.resolve(s, ObjectKind::Body)?;
    let scan_id = i.scan.resolve(s, ObjectKind::Mesh)?;
    let b = s.doc.body(body_id).ok_or_else(|| CmdError::Params(format!("no body {body_id}")))?;
    let scan =
        s.doc.mesh(scan_id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {scan_id}")))?;
    let topo = tessellate_topo(&b.brep, Some(chord))?;
    // the face count only: `body_out` would integrate the volume of every patch (0.7 ms each)
    let faces = surfacecraft_kernel::solid::counts(&b.brep).faces;
    let normals = if i.facing { scan.vertex_normals() } else { Vec::new() };
    let bvh = Bvh::build(&topo.mesh);
    let tagged: Vec<(Sample, Option<usize>)> = samples_mesh_tagged(&scan.positions, &normals, &bvh, maxd)
        .into_iter()
        .map(|(sample, tri)| (sample, face_of(&topo, tri)))
        .collect();
    let samples: Vec<Sample> = tagged.iter().map(|(sample, _)| *sample).collect();
    let mut rows = per_face(&tagged, faces, tol);
    worst_first(&mut rows);
    Ok(CompareOut {
        body: body_id,
        scan: scan_id,
        tolerance: tol,
        chord,
        faces,
        unmeasured: samples.iter().filter(|x| !x.d.is_finite()).count(),
        unattributed: tagged.iter().filter(|(x, f)| x.d.is_finite() && f.is_none()).count(),
        totals: stats_of(&samples, tol),
        per_face: rows,
    })
}

/// `scan.qa` and `body.compare` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "body.compare",
        "Body vs scan",
        "Analyze",
        false,
        "Compares scan `scan` with body `body` per B-rep face: the scan points nearest to each face, their max |deviation| (`max_abs`), RMS, signed mean and share within `tolerance`; totals, and the faces worst first.",
        CompareIn,
        CompareOut,
        compare
    ),
    command!(
        "scan.qa",
        "Scan quality",
        "Analyze",
        false,
        "Quality report of scan `object` before modelling: counts, bounding box, area, holes with sizes, pieces and debris, degenerate, duplicate and non-manifold triangles, noise σ, edge lengths, and findings with a severity each.",
        QaIn,
        QaOut,
        scan_qa
    ),
];

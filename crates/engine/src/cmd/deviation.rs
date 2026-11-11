//! `deviation.*`: how far a scan is from a reference, as a colour map, statistics, hotspots and
//! the deviation from one fitted primitive.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command, read_command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_deviation::{
    DeviationStats, Distribution, Hotspot, Report, ReportOptions, Sample, Volume, colorize, counted_values,
    distribution, estimate_noise, hotspots, oriented_plane, report, samples_mesh, samples_primitive, stats_of,
};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::Primitive;
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3};

/// Default colour-map tolerance and hotspot tolerance (mm).
pub(crate) const TOLERANCE: f64 = 0.1;
/// Default reach of a mesh or body reference: points farther away are not measured (mm).
pub(crate) const REACH: f64 = 5.0;
/// Most hotspot groups one answer lists.
const MAX_LIMIT: usize = 10_000;
/// Multiples of the map's tolerance at which the report's cumulative bands are taken.
const BAND_SCALES: [f64; 4] = [0.5, 1.0, 2.0, 3.0];
/// Scanner noise σ (mm) above which a warning is due, and above which the primitive tolerance is
/// not met (design doc §10).
pub(crate) const NOISE_WARNING_MM: f64 = 0.05;
pub(crate) const NOISE_PRIMITIVE_MM: f64 = 0.1;
/// Chord tolerance bounds (mm) of a body reference: a hostile tolerance cannot ask for an enormous
/// tessellation.
pub(crate) const CHORD_BOUNDS: (f64, f64) = (1e-4, 10.0);

pub(crate) fn yes() -> bool {
    true
}

fn default_bins() -> usize {
    20
}

fn default_bands() -> Vec<f64> {
    vec![0.05, 0.1, 0.2, 0.5]
}

/// Deviation parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ComputeIn {
    /// Scan mesh.
    pub object: ObjRef,
    /// Reference: a primitive, a mesh or a body.
    pub against: ObjRef,
    /// Only the vertices of this region.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Colour-map tolerance (mm, default 0.1).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Points farther than this from a mesh/body reference are not measured (mm, default 5).
    #[serde(default)]
    pub max_distance: Option<f64>,
    /// Leave out points whose normal points away from the reference normal (default true).
    #[serde(default = "yes")]
    pub facing: bool,
    /// Show the colour map on the mesh (default true).
    #[serde(default = "yes")]
    pub show: bool,
}

/// Deviation result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ComputeOut {
    /// Scan mesh.
    pub id: ObjectId,
    /// Statistics.
    pub stats: DeviationStats,
    /// Histogram over ±3 tolerances and the cumulative shares within ½, 1, 2 and 3 tolerances, in
    /// one pass. An output logged without it (before it existed) reads as empty.
    #[serde(default)]
    pub report: Distribution,
    /// Warning when the tolerance is below the estimated scan noise.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warning: Option<String>,
}

/// One measurement of a scan against a reference.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StatsIn {
    /// Scan mesh.
    pub object: ObjRef,
    /// Reference: a primitive, a mesh, a body or a surface.
    pub against: ObjRef,
    /// Only the vertices of this region.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Points farther than this from a mesh/body reference are not measured (mm, default 5).
    #[serde(default)]
    pub max_distance: Option<f64>,
    /// Leave out points whose normal points away from the reference normal (default true).
    #[serde(default = "yes")]
    pub facing: bool,
    /// Histogram bins, 1 to 1000 (default 20).
    #[serde(default = "default_bins")]
    pub bins: usize,
    /// Histogram range [lo, hi] in mm (default ±p99 of |deviation|).
    #[serde(default)]
    pub range: Option<[f64; 2]>,
    /// Tolerances (mm) of the bands: the share of points within each (default 0.05, 0.1, 0.2, 0.5).
    #[serde(default = "default_bands")]
    pub bands: Vec<f64>,
}

/// Statistics of the signed deviation.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct StatsOut {
    /// Scan mesh.
    pub id: ObjectId,
    /// Statistics: counts, extremes, mean, RMS, percentiles, histogram and bands.
    pub stats: Report,
}

/// Hotspot parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct HotspotsIn {
    /// Scan mesh.
    pub object: ObjRef,
    /// Reference: a primitive, a mesh, a body or a surface.
    pub against: ObjRef,
    /// Only the triangles of this region.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Points farther than this from a mesh/body reference are not measured (mm, default 5).
    #[serde(default)]
    pub max_distance: Option<f64>,
    /// Leave out points whose normal points away from the reference normal (default true).
    #[serde(default = "yes")]
    pub facing: bool,
    /// Triangles with |deviation| above this form hotspots (mm, default 0.1).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Smallest group area to list (mm², default 0).
    #[serde(default)]
    pub min_area: Option<f64>,
    /// Groups listed, largest first (default 50, at most 10000).
    #[serde(default)]
    pub limit: Option<usize>,
}

/// Hotspots and the volume error.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct HotspotsOut {
    /// Scan mesh.
    pub id: ObjectId,
    /// Tolerance used (mm).
    pub tolerance: f64,
    /// Groups found with area at least `min_area`, before `limit`.
    pub total: usize,
    /// Points left out because their normal points away from the reference normal.
    pub not_facing: usize,
    /// Groups, largest area first.
    pub hotspots: Vec<Hotspot>,
    /// Volume error over every measured triangle, split into excess and missing material (mm³).
    pub volume: Volume,
}

/// Deviation of a region from one fitted primitive.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ToPrimitiveIn {
    /// Region of a scan mesh.
    pub region: ObjRef,
    /// A fitted primitive: plane, sphere, cylinder, cone or torus.
    pub primitive: ObjRef,
    /// Leave out points whose normal points away from the primitive's normal (default true).
    #[serde(default = "yes")]
    pub facing: bool,
    /// Histogram bins, 1 to 1000 (default 20).
    #[serde(default = "default_bins")]
    pub bins: usize,
    /// Tolerances (mm) of the bands (default 0.05, 0.1, 0.2, 0.5).
    #[serde(default = "default_bands")]
    pub bands: Vec<f64>,
}

/// Signed deviation of a region from a primitive.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ToPrimitiveOut {
    /// Region.
    pub region: ObjectId,
    /// Primitive.
    pub primitive: ObjectId,
    /// Kind of the primitive: plane, sphere, cylinder, cone or torus.
    pub kind: String,
    /// Statistics of the signed deviation (positive: the scan lies outside the primitive).
    pub stats: Report,
}

/// One mesh.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshIn {
    /// Mesh id or name.
    pub object: ObjRef,
}

/// One triangle of a scan that has a deviation map.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AtIn {
    /// Scan mesh with a deviation map (`deviation.compute` with `show`).
    pub object: ObjRef,
    /// Triangle of the scan mesh (the triangle a pick reports).
    pub triangle: u32,
}

/// The deviation at one triangle.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct AtOut {
    /// Scan mesh.
    pub id: ObjectId,
    /// Triangle.
    pub triangle: u32,
    /// Mean signed deviation of its counted corners (mm); null when no corner is counted.
    pub deviation: Option<f64>,
    /// Corners counted (0 to 3); the others are not measured or not facing the reference.
    pub counted_corners: usize,
}

/// Cleared.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ClearOut {
    /// Mesh.
    pub id: ObjectId,
    /// Was a map shown?
    pub cleared: bool,
}

/// Noise estimate.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct NoiseOut {
    /// Mesh.
    pub id: ObjectId,
    /// Estimated σ (mm).
    pub sigma: f64,
    /// Advice relative to the default tolerances (design doc §10).
    pub advice: String,
}

pub(crate) fn positive(v: f64, what: &str) -> CmdResult<f64> {
    if v.is_finite() && v > 0.0 { Ok(v) } else { Err(CmdError::Params(format!("{what} must be positive"))) }
}

fn region_of(s: &Session, r: &Option<ObjRef>) -> CmdResult<Option<ObjectId>> {
    r.as_ref().map(|r| r.resolve(s, ObjectKind::Region)).transpose()
}

fn kind_name(p: &Primitive) -> &'static str {
    match p {
        Primitive::Plane { .. } => "plane",
        Primitive::Sphere { .. } => "sphere",
        Primitive::Cylinder { .. } => "cylinder",
        Primitive::Cone { .. } => "cone",
        Primitive::Torus { .. } => "torus",
    }
}

/// Scan normals of the vertices of `triangles` (the normals a plane's side is judged by).
fn region_normals(mesh: &Mesh, normals: &[Vec3], triangles: &[u32]) -> Vec<Vec3> {
    triangles
        .iter()
        .filter_map(|&t| mesh.triangles.get(t as usize))
        .flatten()
        .filter_map(|&vi| normals.get(vi as usize).copied())
        .collect()
}

/// One measurement: the scan mesh, the triangles in scope and one sample per vertex.
struct Measured {
    mesh: Arc<Mesh>,
    in_scope: Vec<bool>,
    samples: Vec<Sample>,
}

/// Measures the vertices of mesh `mesh_id` (those of `region`, or all) against `against`.
///
/// `tess` is the chord tolerance of a body reference (mm); `max_distance` bounds mesh and body
/// references. With `facing`, a point whose normal points away from the reference normal is
/// marked as not facing (see [`Sample`]).
fn measure(
    s: &Session,
    mesh_id: ObjectId,
    region: Option<ObjectId>,
    against: ObjectId,
    max_distance: f64,
    tess: f64,
    facing: bool,
) -> CmdResult<Measured> {
    let mesh =
        s.doc.mesh(mesh_id).map(|o| o.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh_id}")))?;
    let nv = mesh.vertex_count();
    let mut in_scope = vec![true; mesh.triangle_count()];
    let mut vert_in = vec![true; nv];
    if let Some(rid) = region {
        let reg = s.doc.region(rid).ok_or_else(|| CmdError::Params(format!("no region {rid}")))?;
        if reg.mesh != mesh_id {
            return Err(CmdError::Params("the region belongs to another mesh".into()));
        }
        in_scope.fill(false);
        vert_in.fill(false);
        for &t in reg.triangles.iter() {
            if let Some(x) = in_scope.get_mut(t as usize) {
                *x = true;
            }
            for &vi in mesh.triangles.get(t as usize).into_iter().flatten() {
                if let Some(x) = vert_in.get_mut(vi as usize) {
                    *x = true;
                }
            }
        }
    }
    let normals = mesh.vertex_normals();
    let idx: Vec<usize> = (0..nv).filter(|&i| vert_in.get(i).copied().unwrap_or(false)).collect();
    let pts: Vec<Point3> = idx.iter().filter_map(|&i| mesh.positions.get(i).copied()).collect();
    let scope_normals: Vec<Vec3> = idx.iter().filter_map(|&i| normals.get(i).copied()).collect();
    // no scan normals means every point counts as facing
    let ns: &[Vec3] = if facing { &scope_normals } else { &[] };
    let values: Vec<Sample> = match s.doc.kind_of(against) {
        Some(ObjectKind::Primitive) => {
            let po = s.doc.primitive(against).ok_or_else(|| CmdError::Params(format!("no primitive {against}")))?;
            // a plane has no inherent side: it is judged by the scan normals of its fit region
            let fit_vote = po
                .region
                .and_then(|r| s.doc.region(r))
                .filter(|r| r.mesh == mesh_id)
                .map(|r| region_normals(&mesh, &normals, &r.triangles));
            let vote = fit_vote.as_deref().unwrap_or(scope_normals.as_slice());
            let prim = oriented_plane(&po.primitive, vote);
            samples_primitive(&pts, ns, &prim)
        }
        Some(ObjectKind::Mesh) => {
            let t = s.doc.mesh(against).ok_or_else(|| CmdError::Params(format!("no mesh {against}")))?;
            samples_mesh(&pts, ns, &Bvh::build(&t.mesh), max_distance)
        }
        Some(ObjectKind::Body) => {
            let b = s.doc.body(against).ok_or_else(|| CmdError::Params(format!("no body {against}")))?;
            // bounded, so that a hostile tolerance cannot ask for an enormous tessellation
            let chord = tess.clamp(CHORD_BOUNDS.0, CHORD_BOUNDS.1);
            let t = surfacecraft_kernel::tess::tessellate(&b.brep, Some(chord))?;
            samples_mesh(&pts, ns, &Bvh::build(&t), max_distance)
        }
        Some(ObjectKind::Surface) => {
            // the distance to a freeform surface is unsigned and has no normal: every point counts
            // as outside, facing, and its areas are surface areas
            let o = s.doc.surface(against).ok_or_else(|| CmdError::Params(format!("no surface {against}")))?;
            let face = o.sheet.faces.first().ok_or_else(|| CmdError::Failed("the surface has no face".into()))?;
            let probe = surfacecraft_kernel::nurbs::SurfaceProbe::new(&face.surface)?;
            pts.iter()
                .map(|p| Sample {
                    d: probe.closest([p.x, p.y, p.z]).map_or(f64::NAN, |r| r.0),
                    facing: true,
                    reference: [0.0; 3],
                })
                .collect()
        }
        _ => return Err(CmdError::Params("compare against a primitive, mesh, body or surface".into())),
    };
    let mut samples = vec![Sample::UNMEASURED; nv];
    for (&i, v) in idx.iter().zip(values) {
        if let Some(slot) = samples.get_mut(i) {
            *slot = v;
        }
    }
    Ok(Measured { mesh, in_scope, samples })
}

fn compute(s: &mut Session, i: ComputeIn) -> CmdResult<ComputeOut> {
    let tol = positive(i.tolerance.unwrap_or(TOLERANCE), "tolerance")?;
    let maxd = positive(i.max_distance.unwrap_or(REACH), "max_distance")?;
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let region = region_of(s, &i.region)?;
    let (against, _) = i.against.resolve_any(s)?;
    let m = measure(s, id, region, against, maxd, tol * 0.05, i.facing)?;
    let stats = stats_of(&m.samples, tol);
    let band_tolerances: Vec<f64> = BAND_SCALES.iter().map(|k| k * tol).collect();
    let rep = distribution(&m.samples, tol, &band_tolerances);
    let values = counted_values(&m.samples);
    if i.show {
        let colours = colorize(&values, tol);
        let stored: Vec<f32> = values.iter().map(|&d| d as f32).collect();
        if !s.insert_map(id, colours, stored) {
            return Err(CmdError::Params(format!("no mesh {id}")));
        }
    }
    let noise = estimate_noise(&m.mesh);
    let warning =
        (noise > tol).then(|| format!("tolerance {tol} mm is below the estimated scan noise σ ≈ {noise:.3} mm"));
    Ok(ComputeOut { id, stats, report: rep, warning })
}

fn stats_cmd(s: &Session, i: StatsIn) -> CmdResult<StatsOut> {
    let maxd = positive(i.max_distance.unwrap_or(REACH), "max_distance")?;
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let region = region_of(s, &i.region)?;
    let (against, _) = i.against.resolve_any(s)?;
    let m = measure(s, id, region, against, maxd, TOLERANCE * 0.05, i.facing)?;
    let opts = ReportOptions { bins: i.bins, range: i.range, bands: i.bands };
    let stats = report(&m.samples, &opts).map_err(CmdError::Params)?;
    Ok(StatsOut { id, stats })
}

fn hotspots_cmd(s: &Session, i: HotspotsIn) -> CmdResult<HotspotsOut> {
    let tol = positive(i.tolerance.unwrap_or(TOLERANCE), "tolerance")?;
    let maxd = positive(i.max_distance.unwrap_or(REACH), "max_distance")?;
    let min_area = i.min_area.unwrap_or(0.0);
    if !(min_area.is_finite() && min_area >= 0.0) {
        return Err(CmdError::Params("min_area must be zero or positive".into()));
    }
    let limit = i.limit.unwrap_or(50);
    if limit > MAX_LIMIT {
        return Err(CmdError::Params(format!("limit must be at most {MAX_LIMIT}")));
    }
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let region = region_of(s, &i.region)?;
    let (against, _) = i.against.resolve_any(s)?;
    let m = measure(s, id, region, against, maxd, tol * 0.05, i.facing)?;
    let found = hotspots(&m.mesh, &m.samples, &m.in_scope, tol, min_area);
    let not_facing = m.samples.iter().filter(|x| x.not_facing()).count();
    let total = found.groups.len();
    let mut groups = found.groups;
    groups.truncate(limit);
    Ok(HotspotsOut { id, tolerance: tol, total, not_facing, hotspots: groups, volume: found.volume })
}

fn to_primitive_cmd(s: &Session, i: ToPrimitiveIn) -> CmdResult<ToPrimitiveOut> {
    let region = i.region.resolve(s, ObjectKind::Region)?;
    let mesh = s.doc.region(region).map(|r| r.mesh).ok_or_else(|| CmdError::Params(format!("no region {region}")))?;
    let primitive = i.primitive.resolve(s, ObjectKind::Primitive)?;
    let kind = s.doc.primitive(primitive).map_or("unknown", |p| kind_name(&p.primitive));
    // a primitive has no reach: every facing point of the region is measured
    let m = measure(s, mesh, Some(region), primitive, f64::INFINITY, TOLERANCE * 0.05, i.facing)?;
    let opts = ReportOptions { bins: i.bins, range: None, bands: i.bands };
    let stats = report(&m.samples, &opts).map_err(CmdError::Params)?;
    Ok(ToPrimitiveOut { region, primitive, kind: kind.to_string(), stats })
}

/// One mesh: its colour map is removed.
fn clear(s: &mut Session, i: MeshIn) -> CmdResult<ClearOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    Ok(ClearOut { id, cleared: s.drop_map(id) })
}

/// The mean signed deviation of the counted corners of one triangle of a mapped scan. The map must
/// still be valid for the scan's geometry (see `Session::live_map`).
fn at_cmd(s: &Session, i: AtIn) -> CmdResult<AtOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let map = s.live_map(id).ok_or_else(|| {
        CmdError::Failed(format!("{id} has no current deviation map: run deviation.compute with show"))
    })?;
    let values = &map.values;
    let mesh = s.doc.mesh(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    if values.len() != mesh.mesh.vertex_count() {
        return Err(CmdError::Failed(format!("the deviation map of {id} does not fit its mesh")));
    }
    let corners = mesh
        .mesh
        .triangles
        .get(i.triangle as usize)
        .ok_or_else(|| CmdError::Params(format!("triangle {} is not on {id}", i.triangle)))?;
    let counted: Vec<f64> = corners
        .iter()
        .filter_map(|&vi| values.get(vi as usize))
        .map(|&d| f64::from(d))
        .filter(|d| d.is_finite())
        .collect();
    let deviation = (!counted.is_empty()).then(|| counted.iter().sum::<f64>() / counted.len() as f64);
    Ok(AtOut { id, triangle: i.triangle, deviation, counted_corners: counted.len() })
}

fn noise(s: &mut Session, i: MeshIn) -> CmdResult<NoiseOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let m = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let sigma = estimate_noise(&m);
    let advice = if sigma > NOISE_PRIMITIVE_MM {
        format!("noise is above the {NOISE_PRIMITIVE_MM} mm primitive tolerance: expect RMS above it")
    } else if sigma > NOISE_WARNING_MM {
        format!("noise is above the {NOISE_WARNING_MM} mm warning level")
    } else {
        "noise is below the default tolerances".to_string()
    };
    Ok(NoiseOut { id, sigma, advice })
}

/// `deviation.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "deviation.compute",
        "Deviation map",
        "Analyze",
        false,
        "Signed deviation of `object` (or its `region`) from `against` (primitive/mesh/body); colour map ±`tolerance`. Points whose normal points away from the reference normal are left out.",
        ComputeIn,
        ComputeOut,
        compute
    ),
    command!(
        "deviation.clear",
        "Clear deviation map",
        "Analyze",
        false,
        "Removes the colour map of `object`.",
        MeshIn,
        ClearOut,
        clear
    ),
    read_command!(
        "deviation.at",
        "Deviation at a triangle",
        "Analyze",
        "Signed deviation of triangle `triangle` of the scan `object` from its deviation map: the mean of its counted corners in mm, null when none is counted. Needs a map made with `show`.",
        AtIn,
        AtOut,
        at_cmd
    ),
    command!(
        "deviation.noise",
        "Estimate noise",
        "Analyze",
        false,
        "Estimates the scan noise σ of `object`.",
        MeshIn,
        NoiseOut,
        noise
    ),
    command!(
        "deviation.stats",
        "Deviation statistics",
        "Analyze",
        false,
        "Signed deviation statistics of `object` (or its `region`) against `against`: min, max, mean, RMS, percentiles, a histogram of `bins` over `range` (default ±p99 of |d|) and the share within each tolerance of `bands`.",
        StatsIn,
        StatsOut,
        stats_cmd
    ),
    command!(
        "deviation.hotspots",
        "Deviation hotspots",
        "Analyze",
        false,
        "Connected groups of scan triangles with |deviation| above `tolerance`, sorted by footprint area on the reference (`min_area` filter, `limit` count), and the excess and missing volume in mm³.",
        HotspotsIn,
        HotspotsOut,
        hotspots_cmd
    ),
    command!(
        "deviation.to_primitive",
        "Deviation from primitive",
        "Analyze",
        false,
        "Signed deviation of a `region` from one fitted `primitive` (plane, sphere, cylinder, cone or torus): statistics as `deviation.stats`.",
        ToPrimitiveIn,
        ToPrimitiveOut,
        to_primitive_cmd
    ),
];

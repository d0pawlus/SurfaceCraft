//! `surface.wrap`: a watertight network of bicubic patches over a whole scan, for parts that are
//! not made of primitives (castings, body panels, organic shapes).
//!
//! The scan is reduced to a coarse triangle mesh that stays within a budget of the scan
//! (`geom::reduce`: small triangles where the shape changes, large ones where it is flat), every
//! triangle becomes three patches fitted to the scan (`fit::patch::wrap_patches`), and the
//! network is measured against the scan. A budget that leaves too many points outside the
//! tolerance is halved and the wrap repeated. Nothing is assumed about smoothness, so sharp edges
//! and thin walls survive; the price is that the patches meet G0.
use super::body::{BodyOut, body_out};
use super::deviation::positive;
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::ObjectKind;
use surfacecraft_fit::patch::{PatchSet, edge_normal_jumps, tessellate, wrap_patches};
use surfacecraft_geom::edit::{DecimateOptions, decimate, fill_holes, remove_non_manifold};
use surfacecraft_geom::reduce::{ToleranceOptions, reduce_to_tolerance};
use surfacecraft_geom::{Bvh, Mesh, Point3};
use surfacecraft_kernel::nurbs::{is_closed, sheet_from_bezier_network_as_is};

/// Default tolerance of the network against the scan (mm).
const TOLERANCE: f64 = 0.2;
/// Default share of the scan points that must lie within the tolerance.
const MIN_WITHIN: f64 = 0.99;
/// Default and largest number of patches.
const MAX_PATCHES: usize = 12_000;
pub(crate) const PATCH_LIMIT: usize = 60_000;
/// Patches per coarse triangle.
const PATCHES_PER_TRIANGLE: usize = 3;
/// The coarse mesh may leave the scan by this many tolerances in the first round.
const FIRST_BUDGET: f64 = 4.0;
/// Rounds at most; each one halves the budget.
const MAX_ROUNDS: usize = 3;
/// A patch point is looked for within this many budgets of its first guess.
const REACH_BUDGETS: f64 = 6.0;
/// Scan points measured at most (evenly strided).
const MAX_SAMPLES: usize = 200_000;
/// Times the budget is doubled to bring the coarse mesh under the patch limit.
const CAP_DOUBLINGS: usize = 4;
/// Holes left by removing non-manifold triangles are filled up to this many edges.
const REPAIR_HOLE_EDGES: usize = 60;
/// Tessellation of a patch for the measurement (quads per side).
const MEASURE_GRID: usize = 6;

/// Parameters of `surface.wrap`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct WrapIn {
    /// Scan mesh id or name.
    pub mesh: ObjRef,
    /// Tolerance the patches are held to against the scan (mm, default 0.2).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Share of the scan points that must lie within `tolerance` (0..1, default 0.99).
    #[serde(default)]
    pub min_within: Option<f64>,
    /// Most patches the network may have (default 12000, at most 60000).
    #[serde(default)]
    pub max_patches: Option<usize>,
    /// Name of the body (default "wrap").
    #[serde(default)]
    pub name: Option<String>,
}

/// The network against the scan.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, JsonSchema)]
pub struct WrapDeviation {
    /// Scan points measured.
    pub count: usize,
    /// Share of them within the tolerance (0..1).
    pub within: f64,
    /// RMS distance (mm).
    pub rms: f64,
    /// 99th percentile distance (mm).
    pub p99: f64,
    /// Largest distance (mm).
    pub max: f64,
}

/// Result of `surface.wrap`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct WrapOut {
    /// The body: a solid when the network is closed, else a sheet.
    pub body: BodyOut,
    /// Is the network a closed shell?
    pub closed: bool,
    /// Patches.
    pub patches: usize,
    /// Triangles of the coarse mesh the patches were laid on.
    pub coarse_triangles: usize,
    /// Distance the coarse mesh was allowed from the scan in the last round (mm).
    pub budget: f64,
    /// Rounds run.
    pub rounds: usize,
    /// True when `max_patches` stopped the refinement.
    pub capped: bool,
    /// Tolerance used (mm).
    pub tolerance: f64,
    /// The patches against the scan.
    pub deviation: WrapDeviation,
    /// Does at least `min_within` of the scan lie within the tolerance?
    pub within_tolerance: bool,
    /// Mean angle between the normals of neighbouring patches across their edge (degrees).
    pub normal_jump_mean_deg: f64,
    /// Advice when the tolerance was not met.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warning: Option<String>,
}

/// One round: the coarse mesh, its patches and their deviation.
struct Round {
    set: PatchSet,
    coarse_triangles: usize,
    budget: f64,
    capped: bool,
    deviation: WrapDeviation,
}

/// The coarse mesh within `budget` of `scan`. When it has more than `max_triangles`, it is reduced
/// further with a doubled budget (still measured against the surface, so the shape survives) and
/// only as a last resort cut to the count. The flag says whether the limit was hit.
fn coarse_mesh(scan: &Mesh, budget: f64, max_triangles: usize) -> (Mesh, bool) {
    let mut coarse = scan.clone();
    let opts =
        DecimateOptions { target_triangles: 0, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true };
    // an invalid mesh is left as it is; the caller has validated the scan
    let _report = reduce_to_tolerance(&mut coarse, &opts, &ToleranceOptions::new(budget));
    let capped = coarse.triangle_count() > max_triangles;
    let mut wider = budget;
    for _ in 0..CAP_DOUBLINGS {
        if coarse.triangle_count() <= max_triangles {
            break;
        }
        wider *= 2.0;
        let _report = reduce_to_tolerance(&mut coarse, &opts, &ToleranceOptions::new(wider));
    }
    if coarse.triangle_count() > max_triangles {
        decimate(&mut coarse, &DecimateOptions { target_triangles: max_triangles, ..opts });
    }
    // a reduction can pinch the surface; the network needs at most two triangles on an edge
    remove_non_manifold(&mut coarse);
    (coarse, capped)
}

/// Distances of the sample points from the patches.
fn measure(set: &PatchSet, samples: &[Point3], tolerance: f64) -> WrapDeviation {
    let bvh = Bvh::build(&tessellate(set, MEASURE_GRID));
    let mut d: Vec<f64> = samples.iter().filter_map(|p| bvh.closest(p, f64::INFINITY)).map(|h| h.distance).collect();
    if d.is_empty() {
        return WrapDeviation::default();
    }
    d.sort_by(f64::total_cmp);
    let count = d.len();
    let within = d.iter().filter(|x| **x <= tolerance).count() as f64 / count as f64;
    let rms = (d.iter().map(|x| x * x).sum::<f64>() / count as f64).sqrt();
    let p99 = d.get(((count - 1) as f64 * 0.99) as usize).copied().unwrap_or(0.0);
    WrapDeviation { count, within, rms, p99, max: d.last().copied().unwrap_or(0.0) }
}

/// Wraps `scan`, halving the budget until `min_within` of the samples are within `tolerance`.
fn wrap_rounds(scan: &Mesh, tolerance: f64, min_within: f64, max_patches: usize) -> CmdResult<(Round, usize)> {
    let bvh = Bvh::build(scan);
    let stride = scan.positions.len().div_ceil(MAX_SAMPLES).max(1);
    let samples: Vec<Point3> = scan.positions.iter().step_by(stride).copied().collect();
    let max_triangles = (max_patches / PATCHES_PER_TRIANGLE).max(4);
    let mut best: Option<Round> = None;
    let mut budget = FIRST_BUDGET * tolerance;
    let mut rounds = 0;
    while rounds < MAX_ROUNDS {
        rounds += 1;
        let (coarse, capped) = coarse_mesh(scan, budget, max_triangles);
        let set = wrap_patches(&coarse, &bvh, REACH_BUDGETS * budget);
        let deviation = measure(&set, &samples, tolerance);
        let round = Round { set, coarse_triangles: coarse.triangle_count(), budget, capped, deviation };
        let done = capped || deviation.within >= min_within;
        if best.as_ref().is_none_or(|b| deviation.within > b.deviation.within) {
            best = Some(round);
        }
        if done {
            break;
        }
        budget *= 0.5;
    }
    best.map(|b| (b, rounds)).ok_or_else(|| CmdError::Failed("the scan could not be wrapped".into()))
}

pub(crate) fn wrap(s: &mut Session, i: WrapIn) -> CmdResult<WrapOut> {
    let id = i.mesh.resolve(s, ObjectKind::Mesh)?;
    let tolerance = positive(i.tolerance.unwrap_or(TOLERANCE), "tolerance")?;
    let min_within = i.min_within.unwrap_or(MIN_WITHIN);
    if !(min_within.is_finite() && (0.0..=1.0).contains(&min_within)) {
        return Err(CmdError::Params("min_within must be within 0..=1".into()));
    }
    let max_patches = i.max_patches.unwrap_or(MAX_PATCHES);
    if !(12..=PATCH_LIMIT).contains(&max_patches) {
        return Err(CmdError::Params(format!("max_patches must be 12..={PATCH_LIMIT}")));
    }
    let mut scan =
        s.doc.mesh(id).map(|m| (*m.mesh).clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    scan.weld(1e-9);
    // a patch network needs a surface whose edges have at most two triangles
    if remove_non_manifold(&mut scan) > 0 {
        fill_holes(&mut scan, REPAIR_HOLE_EDGES, true);
        remove_non_manifold(&mut scan);
    }
    if scan.validate().is_err() || scan.triangle_count() < 4 {
        return Err(CmdError::Failed("the mesh is empty or invalid".into()));
    }
    let (round, rounds) = wrap_rounds(&scan, tolerance, min_within, max_patches)?;
    if round.set.nets.is_empty() {
        return Err(CmdError::Failed("the scan gave no patches".into()));
    }
    let nets: Vec<Vec<Vec<[f64; 3]>>> = round.set.nets.iter().map(|n| n.iter().map(|r| r.to_vec()).collect()).collect();
    let sheet = sheet_from_bezier_network_as_is(&nets, &round.set.corners)?;
    let closed = is_closed(&sheet);
    let jumps = edge_normal_jumps(&round.set);
    let jump_mean = (jumps.iter().sum::<f64>() / jumps.len().max(1) as f64).to_degrees();
    let body = s.doc.add_body(i.name.as_deref().unwrap_or("wrap"), sheet);
    let within_tolerance = round.deviation.within >= min_within;
    let warning = (!within_tolerance).then(|| {
        let d = &round.deviation;
        let why = if round.capped {
            "max_patches stopped the refinement: raise it"
        } else {
            "raise max_patches or the tolerance"
        };
        format!("{:.1} % of the scan is within {tolerance} mm (p99 {:.3} mm); {why}", 100.0 * d.within, d.p99)
    });
    Ok(WrapOut {
        body: body_out(s, body)?,
        closed,
        patches: round.set.nets.len(),
        coarse_triangles: round.coarse_triangles,
        budget: round.budget,
        rounds,
        capped: round.capped,
        tolerance,
        deviation: round.deviation,
        within_tolerance,
        normal_jump_mean_deg: jump_mean,
        warning,
    })
}

/// The `surface.wrap` command.
pub static COMMANDS: &[CommandSpec] = &[command!(
    "surface.wrap",
    "Wrap scan in patches",
    "Freeform",
    true,
    "A watertight network of bicubic patches over the whole scan `mesh`, for parts that are not made of primitives (castings, body panels, organic shapes): the scan is reduced to a coarse mesh within a budget, each triangle becomes three patches fitted to the scan, and the budget is halved until `min_within` of the scan lies within `tolerance` or `max_patches` is reached. Sharp edges and thin walls are kept (patches meet G0). The result is a solid when the network is closed, else a sheet.",
    WrapIn,
    WrapOut,
    wrap
)];

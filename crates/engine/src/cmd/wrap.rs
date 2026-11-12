//! `surface.wrap`: a watertight network of bicubic patches over a whole scan, for parts that are
//! not made of primitives (castings, body panels, organic shapes).
//!
//! The scan is reduced to a coarse triangle mesh that stays within a budget of the scan
//! (`geom::reduce`: small triangles where the shape changes, large ones where it is flat), every
//! triangle becomes three patches fitted to the scan (`fit::patch::wrap_patches`), and the
//! network is measured against the scan. Where the patches miss the tolerance the budget is halved
//! around the missed points only (`refine: adaptive`, default) or everywhere (`uniform`), and the
//! wrap repeated; see `wrap_rounds`. The scan tells creases from smooth edges (`crease_deg`): patches
//! are made tangent across the smooth ones (`continuity: g1`, experimental, opt-in) and meet G0 across the creases,
//! so sharp edges and thin walls survive.
use super::body::{BodyOut, body_out};
use super::deviation::positive;
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::patch::edge_normal_jumps;
use surfacecraft_fit::patch_g1::{G1Options, JumpStats, jump_stats};
use surfacecraft_geom::Mesh;
use surfacecraft_geom::edit::{fill_holes, remove_non_manifold};
use surfacecraft_kernel::nurbs::{is_closed, sheet_from_bezier_network_as_is};

use super::wrap_rounds::{Refine, Round, Target, WrapLevel, wrap_rounds};

/// Default tolerance of the network against the scan (mm).
const TOLERANCE: f64 = 0.2;
/// Default share of the scan points that must lie within the tolerance.
const MIN_WITHIN: f64 = 0.99;
/// Default and largest number of patches.
const MAX_PATCHES: usize = 12_000;
pub(crate) const PATCH_LIMIT: usize = 60_000;
/// Default angle (degrees) between the two sides of an edge above which it is a crease.
const CREASE_DEG: f64 = 35.0;
/// Creases are told from smooth edges up to this angle (the scan normals of both sides must still agree
/// within the support of the estimate).
const MAX_CREASE_DEG: f64 = 60.0;
/// Holes left by removing non-manifold triangles are filled up to this many edges.
const REPAIR_HOLE_EDGES: usize = 60;

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
    /// How the layout is refined where the tolerance is missed: `adaptive` (default, only around the missed scan points) or `uniform` (the whole scan).
    #[serde(default)]
    pub refine: Option<String>,
    /// Edges whose two sides (taken from the scan) differ by more than this angle in degrees are creases and stay sharp; the others are smooth (default 35, 1..60).
    #[serde(default)]
    pub crease_deg: Option<f64>,
    /// Continuity across the smooth edges: `g0` (default, positions only) or `g1` (experimental: tangent patches; costs patches and time and can miss the tolerance on real scans).
    #[serde(default)]
    pub continuity: Option<String>,
    /// Name of the body (default "wrap").
    #[serde(default)]
    pub name: Option<String>,
}

/// Normal jumps across patch edges.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, JsonSchema)]
pub struct WrapJump {
    /// Edges measured.
    pub count: usize,
    /// Mean (degrees).
    pub mean_deg: f64,
    /// 99th percentile (degrees).
    pub p99_deg: f64,
    /// Largest (degrees).
    pub max_deg: f64,
    /// Samples whose patch normal is undefined (collapsed patch): not in the figures above, so any means the continuity is not shown.
    pub invalid_samples: usize,
    /// Edges with at least one such sample.
    pub invalid_edges: usize,
}

impl From<JumpStats> for WrapJump {
    fn from(j: JumpStats) -> Self {
        Self {
            count: j.count,
            mean_deg: j.mean_deg,
            p99_deg: j.p99_deg,
            max_deg: j.max_deg,
            invalid_samples: j.invalid_samples,
            invalid_edges: j.invalid_edges,
        }
    }
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
    /// Layout used: `adaptive` or `uniform`.
    pub refine: String,
    /// Patches and fit of every round, in order (the result is the best of them).
    pub levels: Vec<WrapLevel>,
    /// True when `max_patches` cut or stopped the refinement and the tolerance was still missed.
    pub capped: bool,
    /// `max_patches` in force (the bound of every round; see `levels[].partial` for the rounds it cut).
    pub cap_used: usize,
    /// Tolerance used (mm).
    pub tolerance: f64,
    /// The patches against the scan.
    pub deviation: WrapDeviation,
    /// Does at least `min_within` of the scan lie within the tolerance?
    pub within_tolerance: bool,
    /// Mean angle between the normals of neighbouring patches across their edge (degrees), over all edges.
    pub normal_jump_mean_deg: f64,
    /// Continuity used across the smooth edges: `g1` or `g0`.
    pub continuity: String,
    /// Crease angle used (degrees).
    pub crease_deg: f64,
    /// Edges of the coarse mesh found to be creases (kept sharp).
    pub crease_edges: usize,
    /// Edges of the coarse mesh found to be smooth.
    pub smooth_edges: usize,
    /// Patches the tangent construction could not fit (a fold at a degenerate coarse triangle): built meeting their neighbours G0 only.
    pub g0_patches: usize,
    /// Worst normal jump along each smooth patch edge: the tangent continuity reached.
    pub smooth_jump: WrapJump,
    /// Mean angle between the normals of the patches across the crease edges (degrees; none without creases).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub crease_angle_mean_deg: Option<f64>,
    /// Advice when the tolerance was not met.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warning: Option<String>,
}

/// The validated parameters.
struct Settings {
    tolerance: f64,
    min_within: f64,
    refine: Refine,
    max_patches: usize,
    crease_deg: f64,
    g1: bool,
}

fn settings(i: &WrapIn) -> CmdResult<Settings> {
    let tolerance = positive(i.tolerance.unwrap_or(TOLERANCE), "tolerance")?;
    let min_within = i.min_within.unwrap_or(MIN_WITHIN);
    if !(min_within.is_finite() && (0.0..=1.0).contains(&min_within)) {
        return Err(CmdError::Params("min_within must be within 0..=1".into()));
    }
    let crease_deg = i.crease_deg.unwrap_or(CREASE_DEG);
    if !(crease_deg.is_finite() && (1.0..=MAX_CREASE_DEG).contains(&crease_deg)) {
        return Err(CmdError::Params(format!("crease_deg must be within 1..={MAX_CREASE_DEG}")));
    }
    let g1 = match i.continuity.as_deref() {
        Some("g1") => true,
        None | Some("g0") => false,
        Some(other) => return Err(CmdError::Params(format!("continuity must be \"g1\" or \"g0\", not \"{other}\""))),
    };
    let max_patches = i.max_patches.unwrap_or(MAX_PATCHES);
    if !(12..=PATCH_LIMIT).contains(&max_patches) {
        return Err(CmdError::Params(format!("max_patches must be 12..={PATCH_LIMIT}")));
    }
    Ok(Settings { tolerance, min_within, refine: Refine::parse(i.refine.as_deref())?, max_patches, crease_deg, g1 })
}

/// The scan as a surface whose edges have at most two triangles (a patch network needs that).
fn prepared_scan(s: &Session, id: ObjectId) -> CmdResult<Mesh> {
    let mut scan =
        s.doc.mesh(id).map(|m| (*m.mesh).clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    scan.weld(1e-9);
    if remove_non_manifold(&mut scan) > 0 {
        fill_holes(&mut scan, REPAIR_HOLE_EDGES, true);
        remove_non_manifold(&mut scan);
    }
    if scan.validate().is_err() || scan.triangle_count() < 4 {
        return Err(CmdError::Failed("the mesh is empty or invalid".into()));
    }
    Ok(scan)
}

/// Advice when the tolerance was not met.
fn advice(round: &Round, tolerance: f64) -> String {
    let d = &round.deviation;
    let why = if round.capped {
        "max_patches stopped the refinement: raise it"
    } else {
        "raise max_patches or the tolerance"
    };
    format!("{:.1} % of the scan is within {tolerance} mm (p99 {:.3} mm); {why}", 100.0 * d.within, d.p99)
}

pub(crate) fn wrap(s: &mut Session, i: WrapIn) -> CmdResult<WrapOut> {
    let id = i.mesh.resolve(s, ObjectKind::Mesh)?;
    let set = settings(&i)?;
    let scan = prepared_scan(s, id)?;
    let options = G1Options { g1: set.g1, crease_deg: set.crease_deg };
    let target = Target::new(&scan, set.tolerance, set.min_within, set.max_patches, options);
    let wrapped = wrap_rounds(&target, set.refine)?;
    let round = wrapped.best;
    if round.set.nets.is_empty() {
        return Err(CmdError::Failed("the scan gave no patches".into()));
    }
    let nets: Vec<Vec<Vec<[f64; 3]>>> = round.set.nets.iter().map(|n| n.iter().map(|r| r.to_vec()).collect()).collect();
    let sheet = sheet_from_bezier_network_as_is(&nets, &round.set.corners)?;
    let closed = is_closed(&sheet);
    let jumps = edge_normal_jumps(&round.set);
    let jump_mean = (jumps.iter().sum::<f64>() / jumps.len().max(1) as f64).to_degrees();
    let (smooth, sharp) = jump_stats(&round.set, &round.creases);
    let body = s.doc.add_body(i.name.as_deref().unwrap_or("wrap"), sheet);
    let within_tolerance = round.deviation.within >= set.min_within;
    Ok(WrapOut {
        body: body_out(s, body)?,
        closed,
        patches: round.set.nets.len(),
        coarse_triangles: round.coarse_triangles,
        budget: round.budget,
        rounds: wrapped.rounds,
        refine: set.refine.name().to_string(),
        levels: wrapped.levels,
        capped: round.capped,
        cap_used: set.max_patches,
        tolerance: set.tolerance,
        deviation: round.deviation,
        within_tolerance,
        normal_jump_mean_deg: jump_mean,
        continuity: if set.g1 { "g1" } else { "g0" }.to_string(),
        crease_deg: set.crease_deg,
        crease_edges: round.crease_edges,
        smooth_edges: round.smooth_edges,
        g0_patches: round.g0_patches,
        smooth_jump: smooth.into(),
        crease_angle_mean_deg: (sharp.count > 0).then_some(sharp.mean_deg),
        warning: (!within_tolerance).then(|| advice(&round, set.tolerance)),
    })
}

/// The `surface.wrap` command.
pub static COMMANDS: &[CommandSpec] = &[command!(
    "surface.wrap",
    "Wrap scan in patches",
    "Freeform",
    true,
    "A watertight network of bicubic patches over the whole scan `mesh`, for parts that are not made of primitives (castings, body panels, organic shapes): the scan is reduced to a coarse mesh within a budget, each triangle becomes three patches fitted to the scan, and the budget is halved (`refine` adaptive: only around the scan points the patches miss, uniform: everywhere) until `min_within` of the scan lies within `tolerance` or `max_patches` is reached. Every round is a conforming mesh, so neighbouring patches always share their boundary. The scan tells creases (edges whose two sides differ by more than `crease_deg`) from smooth edges: with `continuity` g1 (experimental, opt-in; default g0) patches are made tangent across the smooth ones (`smooth_jump` reports the normal jump that is left) and meet G0 across the creases, so sharp edges and thin walls are kept. The result is a solid when the network is closed, else a sheet.",
    WrapIn,
    WrapOut,
    wrap
)];

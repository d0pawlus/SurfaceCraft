//! `surface.wrap`: a watertight network of bicubic patches over a whole scan, for parts that are
//! not made of primitives (castings, body panels, organic shapes).
//!
//! The scan is reduced to a coarse triangle mesh that stays within a budget of the scan
//! (`geom::reduce`: small triangles where the shape changes, large ones where it is flat), every
//! triangle becomes three patches fitted to the scan (`fit::patch::wrap_patches`), and the
//! network is measured against the scan. Where the patches miss the tolerance the budget is halved
//! around the missed points only (`refine: adaptive`, default) or everywhere (`uniform`), and the
//! wrap repeated; see `wrap_rounds`. Nothing is assumed about smoothness, so sharp edges
//! and thin walls survive; the price is that the patches meet G0.
use super::body::{BodyOut, body_out};
use super::deviation::positive;
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::ObjectKind;
use surfacecraft_fit::patch::edge_normal_jumps;
use surfacecraft_geom::edit::{fill_holes, remove_non_manifold};
use surfacecraft_kernel::nurbs::{is_closed, sheet_from_bezier_network_as_is};

use super::wrap_rounds::{Refine, Target, WrapLevel, wrap_rounds};

/// Default tolerance of the network against the scan (mm).
const TOLERANCE: f64 = 0.2;
/// Default share of the scan points that must lie within the tolerance.
const MIN_WITHIN: f64 = 0.99;
/// Default and largest number of patches.
const MAX_PATCHES: usize = 12_000;
pub(crate) const PATCH_LIMIT: usize = 60_000;
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
    /// Layout used: `adaptive` or `uniform`.
    pub refine: String,
    /// Patches and fit of every round, in order (the result is the best of them).
    pub levels: Vec<WrapLevel>,
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

pub(crate) fn wrap(s: &mut Session, i: WrapIn) -> CmdResult<WrapOut> {
    let id = i.mesh.resolve(s, ObjectKind::Mesh)?;
    let tolerance = positive(i.tolerance.unwrap_or(TOLERANCE), "tolerance")?;
    let min_within = i.min_within.unwrap_or(MIN_WITHIN);
    if !(min_within.is_finite() && (0.0..=1.0).contains(&min_within)) {
        return Err(CmdError::Params("min_within must be within 0..=1".into()));
    }
    let refine = Refine::parse(i.refine.as_deref())?;
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
    let target = Target::new(&scan, tolerance, min_within, max_patches);
    let wrapped = wrap_rounds(&target, refine)?;
    let round = wrapped.best;
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
        rounds: wrapped.rounds,
        refine: refine.name().to_string(),
        levels: wrapped.levels,
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
    "A watertight network of bicubic patches over the whole scan `mesh`, for parts that are not made of primitives (castings, body panels, organic shapes): the scan is reduced to a coarse mesh within a budget, each triangle becomes three patches fitted to the scan, and the budget is halved (`refine` adaptive: only around the scan points the patches miss, uniform: everywhere) until `min_within` of the scan lies within `tolerance` or `max_patches` is reached. Every round is a conforming mesh, so neighbouring patches always share their boundary. Sharp edges and thin walls are kept (patches meet G0). The result is a solid when the network is closed, else a sheet.",
    WrapIn,
    WrapOut,
    wrap
)];

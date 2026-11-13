//! `mesh.*` edits of a triangle mesh: clean-up, decimation, smoothing and hole filling. Each edit
//! acts on the working copy while the mesh has one (see `mesh.working_*`), else on the original.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::reduce::{self, DeviationStats, ToleranceOptions, ToleranceReport};
use surfacecraft_geom::{Mesh, edit};

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
    /// Holes left open (above the size limit, not triangulable, or whose fill would put three triangles on one edge).
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
    /// The edit changed the working copy, and the original is untouched.
    #[serde(skip_serializing_if = "is_false", default)]
    pub working: bool,
    /// Why a `mesh.decimate` with a `target` stopped above it (`triangles` is what was reached).
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub target_not_reached: Option<String>,
}

fn is_false(v: &bool) -> bool {
    !*v
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

/// Runs `f` on a copy of the geometry edits act on. While the mesh has a working copy the edit
/// changes the copy and leaves the original (and its regions and overlay) alone; otherwise it
/// changes the original, dropping the regions whose triangles changed.
fn edit_mesh(
    s: &mut Session,
    object: &ObjRef,
    f: impl FnOnce(&mut Mesh, &mut EditOut) -> CmdResult<()>,
) -> CmdResult<EditOut> {
    let id = object.resolve(s, ObjectKind::Mesh)?;
    let obj = s.doc.mesh(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let working = obj.is_working();
    let mut m: Mesh = (**obj.editable()).clone();
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
        working,
        target_not_reached: None,
    };
    let old_tris = m.triangles.clone();
    f(&mut m, &mut o)?;
    o.triangles = m.triangle_count();
    if !working && m.triangles != old_tris {
        o.regions_dropped = s.doc.replace_mesh_topology(id, m).unwrap_or(0);
    } else {
        store_editable(s, id, m)?;
    }
    Ok(o)
}

/// Stores `m` as the geometry edits act on (see [`edit_mesh`]).
fn store_editable(s: &mut Session, id: ObjectId, m: Mesh) -> CmdResult<()> {
    let obj = s.doc.mesh_mut(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    *obj.editable_mut() = Arc::new(m);
    Ok(())
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

pub(crate) fn clean(s: &mut Session, i: CleanIn) -> CmdResult<EditOut> {
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
    /// Largest hole filled, in boundary edges (default 300, at most 20000; a smooth fill of a hole of thousands of edges takes tens of seconds).
    #[serde(default)]
    pub max_edges: Option<usize>,
    /// Curvature-following fill (refined, thin-plate faired); `false` = flat (default true).
    #[serde(default)]
    pub smooth: Option<bool>,
}

pub(crate) fn fill(s: &mut Session, i: FillIn) -> CmdResult<EditOut> {
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

/// Why a decimation stopped above its target, `None` when it did not.
fn not_reached(reached: usize, target: usize, report: &edit::DecimateReport, max_error: Option<f64>) -> Option<String> {
    if reached <= target.max(4) {
        return None;
    }
    Some(if report.turns_refused > 0 {
        format!(
            "stopped at {reached} triangles: {} collapses were refused because they would have folded a triangle over the surface (a thin wall or a sharp ridge)",
            report.turns_refused
        )
    } else if max_error.is_some() {
        format!("stopped at {reached} triangles: no collapse left within `max_error`")
    } else {
        format!(
            "stopped at {reached} triangles: no collapse left that keeps the surface a manifold without sharp edges or boundary moving"
        )
    })
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
            let report = edit::decimate_with_report(m, &opts);
            o.removed_triangles = report.removed;
            o.target_not_reached = not_reached(m.triangle_count(), target, &report, i.max_error);
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
    let cancel = s.cancel.clone();
    edit_mesh(s, &i.object, |m, o| {
        let feat = if angle >= 180.0 { None } else { Some(angle) };
        let moved =
            edit::smooth_cancellable(m, i.iterations.unwrap_or(5), i.fix_boundary.unwrap_or(true), feat, &cancel);
        o.max_displacement = Some(moved.ok_or_else(super::cancelled)?);
        Ok(())
    })
}

/// `mesh.*` edit commands.
pub static COMMANDS: &[CommandSpec] = &[
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
        "Reduces to `target` triangles or `ratio`, within `max_error` mm, keeping edges sharper than `feature_angle_deg`. A collapse that would fold a triangle over the surface is never made, so on a thin wall or a sharp ridge the result can stay above the target: `triangles` is what was reached and `target_not_reached` says why. With `tolerance` (mm) the reduction is measured both ways and refined locally until `share` (default 0.999) of the samples is within it; `tolerance_report` has max, p99 and share per direction, triangle counts and rounds.",
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

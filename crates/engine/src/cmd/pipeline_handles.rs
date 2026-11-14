//! `close_handles` of `scan.to_solid` (method `freeform`): the small handles of the cleaned scan are
//! closed on a copy (`mesh.close_handles`), the copy is wrapped and the solid is measured against the
//! scan as it was. Off by default; a freeform result that is not a pass on a scan with many handles
//! offers it as a next step.
use super::freeform::freeform_on;
use super::prepare::check_cancel;
use super::rebuild::Stopwatch;
use super::{CmdResult, Issue, NextStep, PrepStep, ScanSummary, Settings, ToSolidOut, Verdict, issue, step};
use crate::Session;
use crate::cmd::mesh_handles::{DEFAULT_SIZE_EDGES, by_scale, close, handle_estimate, size_of};
use serde_json::json;
use surfacecraft_geom::Mesh;
use surfacecraft_geom::handles::Report;
use surfacecraft_geom::quality::edge_census;

/// A scan with at least this many handles (estimated from its Euler characteristic) is offered the
/// step when a freeform result is not a pass.
pub(super) const HANDLE_HINT: i64 = 64;
/// Cutting out more than this share of the area of the scan is not small: the verdict is at most `review`.
const SMALL_AREA_SHARE: f64 = 0.002;

fn median_edge(mesh: &Mesh) -> f64 {
    edge_census(mesh).lengths.median
}

/// The preparation step of the closing.
fn prep_step(rep: &Report, size: f64, (before, after): (i64, i64), ms: u64) -> PrepStep {
    let reason = if rep.changed {
        format!(
            "close_handles: {} handles closed in {} parts up to {size:.2} mm; the copy with them closed was wrapped and the solid measured against the scan as it was",
            rep.handles_closed,
            rep.closed.len()
        )
    } else {
        format!("close_handles: no handle up to {size:.2} mm could be closed; the scan was wrapped as it is")
    };
    let numbers = json!({
        "max_size": size, "median_edge": rep.median_edge, "handles_before": before, "handles_after": after,
        "handles_closed": rep.handles_closed, "parts": rep.closed.len(), "by_scale": by_scale(rep),
        "refused": {"splits": rep.refused.splits, "unfillable": rep.refused.unfillable, "fill_far": rep.refused.fill_far, "uncovered": rep.refused.uncovered},
        "loops_before": rep.loops_before, "loops_after": rep.loops_after,
        "pieces_before": rep.pieces_before, "pieces_after": rep.pieces_after,
        "removed_triangles": rep.removed_triangles, "removed_area": rep.removed_area, "removed_area_share": rep.removed_area_share,
        "added_triangles": rep.added_triangles, "removed_vertices": rep.removed_vertices, "removed_within": rep.removed_within,
        "removed_max_distance": rep.removed_max_distance, "ms": ms,
    });
    PrepStep { step: "close_handles".into(), reason, numbers }
}

/// The issue of a closing that was not small against the tolerance, if it was not.
fn removal_issue(rep: &Report, tolerance: f64) -> Option<Issue> {
    let far = rep.removed_max_distance > tolerance;
    let large = rep.removed_area_share > SMALL_AREA_SHARE;
    (rep.changed && (far || large)).then(|| {
        issue(
            "scan_handles_closed",
            format!(
                "{} handles of the scan were closed before it was wrapped: {} parts cut out ({:.2} % of the area, {} vertices, of which {} lie farther than {tolerance} mm from the repaired scan, up to {:.3} mm) and filled; the solid follows the repaired scan and is measured against the scan as it was, so what was cut out counts against it",
                rep.handles_closed,
                rep.closed.len(),
                100.0 * rep.removed_area_share,
                rep.removed_vertices,
                rep.removed_vertices.saturating_sub(rep.removed_within),
                rep.removed_max_distance
            ),
        )
    })
}

/// Closes the handles of the scan on a copy, wraps the copy and measures the solid against the scan.
/// When nothing could be closed, the step that says so is the `Err`: the scan is modelled as it is.
pub(super) fn wrap_closed(
    s: &mut Session,
    set: &Settings,
    report: &ScanSummary,
) -> CmdResult<Result<ToSolidOut, PrepStep>> {
    check_cancel(s)?;
    let watch = Stopwatch::start();
    let Some(mesh) = s.doc.mesh(set.scan).map(|m| m.mesh.clone()) else {
        return Err(crate::cmd::CmdError::Params(format!("no mesh {}", set.scan)));
    };
    let size = size_of(set.handle_size, median_edge(&mesh))?;
    let (before, _) = handle_estimate(&mesh);
    let (closed, rep) = close(&mesh, size, set.tolerance, s)?;
    if !rep.changed {
        return Ok(Err(prep_step(&rep, size, (before, before), watch.ms())));
    }
    let (after, _) = handle_estimate(&closed);
    let step = prep_step(&rep, size, (before, after), watch.ms());
    let id = s.doc.add_mesh(&format!("{}-handles", set.name), closed);
    let result = freeform_on(s, set, id, report.clone());
    s.doc.remove(id);
    let mut out = result?;
    out.prep.preparation.push(step);
    if let Some(i) = removal_issue(&rep, set.tolerance) {
        out.issues.push(i);
        if out.verdict == Verdict::Pass {
            out.verdict = Verdict::Review;
        }
    }
    Ok(Ok(out))
}

/// The next step that closes the handles, for a freeform result that is not a pass on a scan with
/// at least [`HANDLE_HINT`] handles, when the call did not close them already.
pub(super) fn offer(s: &Session, set: &Settings, issues: &[Issue]) -> Option<NextStep> {
    if set.close_handles || issues.is_empty() {
        return None;
    }
    let mesh = s.doc.mesh(set.scan)?;
    let (handles, loops) = handle_estimate(&mesh.mesh);
    if handles < HANDLE_HINT {
        return None;
    }
    let size = DEFAULT_SIZE_EDGES * median_edge(&mesh.mesh);
    let why = format!(
        "the scan has about {handles} handles ({loops} boundary loops): small tunnels and bridges, typically where the two sides of a thin wall come closer than the scanner resolves; they stall the reduction and spend the patch budget. `close_handles` closes those in neighbourhoods up to {size:.1} mm wide on a copy (`mesh.close_handles` alone shows what it would cut out), keeps the rim and every vertex away from them, and measures the solid against the scan as it was"
    );
    let params = json!({"scan": set.scan, "method": "freeform", "tolerance": set.tolerance, "close_handles": true});
    Some(step("scan.to_solid", params, &why))
}

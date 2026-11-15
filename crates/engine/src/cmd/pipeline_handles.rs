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
        "refused": {"splits": rep.refused.splits, "unfillable": rep.refused.unfillable, "fill_far": rep.refused.fill_far, "uncovered": rep.refused.uncovered, "whole_pieces": rep.refused.whole_pieces},
        "loops_before": rep.loops_before, "loops_after": rep.loops_after,
        "pieces_before": rep.pieces_before, "pieces_after": rep.pieces_after,
        "removed_triangles": rep.removed_triangles, "removed_area": rep.removed_area, "removed_area_share": rep.removed_area_share,
        "added_triangles": rep.added_triangles, "removed_samples": rep.removed_samples, "removed_within": rep.removed_within,
        "removed_max_distance": rep.removed_max_distance, "ms": ms,
    });
    PrepStep { step: "close_handles".into(), reason, numbers }
}

/// The issue of a closing: raised whenever anything was cut, so the verdict is at most `review`. The
/// option changes the topology of the scan, and the deviation is measured from scan vertices, which
/// cannot see material added over a real opening.
fn removal_issue(rep: &Report, tolerance: f64) -> Option<Issue> {
    rep.changed.then(|| {
        issue(
            "scan_handles_closed",
            format!(
                "{} handles of the scan were closed before it was wrapped: {} parts cut out ({:.2} % of the area) and their holes filled; of {} points sampled over what was cut out, {} lie farther than {tolerance} mm from the repaired scan, the farthest {:.3} mm; the topology of the solid is not that of the scan, and material added over an opening is not seen by the deviation, so look at the parts closed before relying on it",
                rep.handles_closed,
                rep.closed.len(),
                100.0 * rep.removed_area_share,
                rep.removed_samples,
                rep.removed_samples.saturating_sub(rep.removed_within),
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
    let (before, _) = handle_estimate(&mesh);
    let Some(size) = size_of(set.handle_size, median_edge(&mesh))? else {
        // no edge length to take a size from: nothing can be closed, the scan is wrapped as it is
        let nothing = Report::default();
        return Ok(Err(prep_step(&nothing, 0.0, (before, before), watch.ms())));
    };
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

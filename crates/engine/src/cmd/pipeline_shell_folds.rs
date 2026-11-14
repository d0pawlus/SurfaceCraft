//! The fold removal inside `scan.to_solid` with a `thickness`, seen from the verdict: the scan is judged as
//! it was before the folds were taken out (the removed triangles are measured against the body too, where
//! the shell is wrapped), and a removal that is large (a fold taller than the tolerance, a fill that differs
//! from the clean surface by more than it, or more than [`MAX_REMOVED_AREA_SHARE`] of the area) raises the
//! issue `scan_folds_removed`, so such a result is never a `pass`.
use super::{DeviationSummary, Issue, Settings};
use crate::Session;
use crate::cmd::mesh_folds::FoldsOut;
use crate::cmd::qa::{CompareIn, compare_points};
use crate::cmd::{CmdResult, ObjRef};
use surfacecraft_geom::{Bvh, Mesh};

/// A removal of more than this share of the area of the scan (0.2 %) raises `scan_folds_removed`. (Measured
/// on the real scans of the corpus: 0.003 % to 0.09 %, and 0.5 % on one scan full of crossing flaps.)
pub(super) const MAX_REMOVED_AREA_SHARE: f64 = 0.002;

/// What the fold removal of a run did: the report and the scan as it was where it removed.
pub(super) struct FoldRun {
    pub out: Option<FoldsOut>,
    pub removed: Mesh,
}

/// The issue of a removal that is not small against the tolerance; none when nothing was removed or it was.
pub(super) fn fold_issue(f: &FoldsOut, tolerance: f64) -> Option<Issue> {
    let large = f.largest_removed_height > tolerance
        || f.largest_fill_deviation > tolerance
        || f.removed_area_share > MAX_REMOVED_AREA_SHARE;
    if f.removed == 0 || !large {
        return None;
    }
    let message = format!(
        "cleaning removed {} folds of the scan (triangles that faced against the surface around them and passed through it; {} triangles, {:.4} % of the area): the largest stood {:.3} mm out of the surface it was replaced by and the patch differs from the clean surface by {:.3} mm at most, against a tolerance of {tolerance} mm. The scan is judged as it was, the solid is built on the repaired scan: check `thickened.folds` and the scan before relying on it",
        f.removed,
        f.removed_triangles,
        100.0 * f.removed_area_share,
        f.largest_removed_height,
        f.largest_fill_deviation
    );
    Some(Issue { code: "scan_folds_removed".to_string(), message })
}

/// The vertices of the removed triangles, where the shell's scan layer puts them: the scan itself, or with
/// `symmetric` half the thickness along the normal of the repaired scan at the nearest point (the normals of
/// the folded triangles themselves mean nothing).
fn original_points(removed: &Mesh, repaired: &Mesh, thickness: f64, symmetric: bool) -> Mesh {
    let mut positions = removed.positions.clone();
    if symmetric {
        let bvh = Bvh::build(repaired);
        for p in &mut positions {
            let normal = bvh.closest(p, f64::MAX).and_then(|h| repaired.face_normal(h.triangle));
            if let Some(n) = normal {
                *p += n * (0.5 * thickness.abs());
            }
        }
    }
    Mesh { positions, triangles: Vec::new() }
}

/// `main` (the repaired scan, `vertices` points) and the removed points (`extra`, `removed` points, measured
/// with the facing test off: a folded triangle faces against the surface by definition) as one summary.
fn merge(
    main: &DeviationSummary,
    vertices: usize,
    extra: &surfacecraft_deviation::DeviationStats,
    removed: usize,
) -> DeviationSummary {
    let (n1, n2) = (main.count as f64, extra.count as f64);
    let n = (n1 + n2).max(1.0);
    let total = (vertices + removed).max(1) as f64;
    DeviationSummary {
        count: main.count + extra.count,
        coverage: (n1 + n2) / total,
        not_facing: main.not_facing * vertices as f64 / total,
        within: (main.within * n1 + extra.within * n2) / n,
        rms: ((main.rms.powi(2) * n1 + extra.rms.powi(2) * n2) / n).sqrt(),
        max_abs: main.max_abs.max(extra.max_abs),
        mean: (main.mean * n1 + extra.mean * n2) / n,
    }
}

/// The deviation of the body from the scan as it was: `main` is the repaired scan's, the removed triangles'
/// vertices are added when folds were taken out.
pub(super) fn judged(
    s: &Session,
    set: &Settings,
    body: surfacecraft_doc::ObjectId,
    (main, vertices): (DeviationSummary, usize),
    folds: &FoldRun,
    shell: (&Mesh, f64, bool),
) -> CmdResult<DeviationSummary> {
    if folds.removed.positions.is_empty() {
        return Ok(main);
    }
    let points = original_points(&folds.removed, shell.0, shell.1, shell.2);
    let cmp = CompareIn {
        body: ObjRef::Id(body.0),
        scan: ObjRef::Id(set.scan.0),
        tolerance: Some(set.tolerance),
        max_distance: None,
        chord: None,
        facing: false,
    };
    let extra = compare_points(s, body, set.scan, &points, &cmp)?;
    Ok(merge(&main, vertices, &extra.totals, points.positions.len()))
}

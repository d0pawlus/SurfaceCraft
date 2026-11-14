//! The fold removal inside `scan.to_solid` with a `thickness`, seen from the verdict: the scan is judged as
//! it was before the folds were taken out. The removed triangles' vertices are measured against the scan
//! layer of the body only, exactly as the points of the scan are (facing test on, with the normals of the
//! surface that replaced them: a folded triangle's own normal means nothing), and a removal that is large (a
//! fold taller than the tolerance, material that stands further than it from the body, a fill that differs
//! from the clean surface by more than it, or more than [`MAX_REMOVED_AREA_SHARE`] of the area) raises the
//! issue `scan_folds_removed`, so such a result is never a `pass`.
use super::{DeviationSummary, Issue, NextStep, Settings, ToSolidIn, step};
use crate::Session;
use crate::cmd::mesh_folds::FoldsOut;
use crate::cmd::qa::{CompareIn, compare_points_with};
use crate::cmd::{CmdResult, ObjRef};
use serde_json::json;
use std::cell::Cell;
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3, folds};

/// A removal of more than this share of the area of the scan (0.2 %) raises `scan_folds_removed`. (Measured
/// on the real scans of the corpus: 0.001 % to 0.03 %.)
pub(super) const MAX_REMOVED_AREA_SHARE: f64 = 0.002;

/// How the removed points stand against the body.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) struct RemovedStats {
    /// Points measured (facing the scan layer).
    pub count: usize,
    /// Points not measured because the nearest face of the body faces the other way (the point lies deeper in
    /// the wall than its middle, or inside out): a fold standing that deep into the wall.
    pub not_facing: usize,
    /// Their share within the tolerance.
    pub within: f64,
    /// The largest |deviation| (mm).
    pub max_abs: f64,
}

/// What the fold removal of a run did: the report, the scan as it was where it removed, and (once the body is
/// there) how the removed points stand against it.
pub(super) struct FoldRun {
    pub out: Option<FoldsOut>,
    pub removed: Mesh,
    pub stats: Cell<Option<RemovedStats>>,
}

impl FoldRun {
    pub(super) fn new(out: Option<FoldsOut>, removed: Mesh) -> Self {
        Self { out, removed, stats: Cell::new(None) }
    }

    /// The report with the numbers of the judge added.
    pub(super) fn report(self) -> Option<FoldsOut> {
        let stats = self.stats.get();
        self.out.map(|mut f| {
            if let Some(s) = stats {
                (f.removed_points, f.removed_points_not_facing) = (s.count, s.not_facing);
                (f.removed_points_within, f.removed_points_max_deviation) = (s.within, s.max_abs);
            }
            f
        })
    }
}

/// The issue of a removal that is not small against the tolerance; none when nothing was removed or it was.
pub(super) fn fold_issue(f: &FoldsOut, tolerance: f64, stats: Option<RemovedStats>) -> Option<Issue> {
    let stands = stats.map_or(0.0, |s| s.max_abs);
    let lost = stats.map_or(0, |s| s.not_facing);
    let large = f.largest_removed_height > tolerance
        || f.largest_fill_deviation > tolerance
        || stands > tolerance
        || lost > 0
        || f.removed_area_share > MAX_REMOVED_AREA_SHARE;
    if f.removed == 0 || !large {
        return None;
    }
    let message = format!(
        "cleaning removed {} folds of the scan (triangles that faced against the surface around them and passed through it; {} triangles, {:.4} % of the area): the largest stood {:.3} mm out of the surface that replaced it, the removed material is {:.3} mm from the surface that is left at most and {stands:.3} mm from the body ({lost} of its points lie deeper in the wall than its middle and are not measured), against a tolerance of {tolerance} mm. The scan is judged as it was, the solid is built on the repaired scan: check `thickened.folds` and the scan before relying on it",
        f.removed,
        f.removed_triangles,
        100.0 * f.removed_area_share,
        f.largest_removed_height,
        f.largest_fill_deviation
    );
    Some(Issue { code: "scan_folds_removed".to_string(), message })
}

/// The next step that offers the fold removal, with what it would remove, when the scan has folds that could be
/// removed (detection only: it finds and measures the groups, about as long as it takes to read the mesh a few
/// times, seconds on a million triangles, a fraction of the shell). `None` when the option is on already or the
/// scan was not cleaned, or there is nothing to remove. The removal itself may refuse some of them (beside a
/// crease, a patch that would stray), so the numbers are an upper bound.
pub(super) fn fold_offer(repaired: &Mesh, set: &Settings, i: &ToSolidIn) -> Option<NextStep> {
    if i.remove_folds || !i.clean {
        return None;
    }
    let found = folds::find_folds(repaired);
    let removable: Vec<_> = found.iter().filter(|f| f.small).collect();
    if removable.is_empty() {
        return None;
    }
    let tallest = removable.iter().map(|f| f.height).fold(0.0, f64::max);
    let triangles: usize = removable.iter().map(|f| f.triangles).sum();
    let outcome = if tallest > set.tolerance {
        format!(
            "the tallest stands {tallest:.2} mm above the surface around it, more than the tolerance of {} mm: the result will be a `review` with `scan_folds_removed`",
            set.tolerance
        )
    } else {
        format!(
            "the tallest stands {tallest:.2} mm above the surface around it, within the tolerance of {} mm",
            set.tolerance
        )
    };
    let why = format!(
        "would remove up to {} folds of the scan ({triangles} triangles, with a margin around each) that cross the sheet and may be what the shell cannot clear; {outcome}. Some may be refused (beside a crease, a patch that would stray)",
        removable.len()
    );
    let params = json!({
        "scan": set.scan, "thickness": i.thickness, "symmetric": i.symmetric,
        "tolerance": set.tolerance, "remove_folds": true
    });
    Some(step("scan.to_solid", params, &why))
}

/// The vertices of the removed triangles as the scan layer of the shell has them, with the normals of that
/// layer: the scan itself, or with `symmetric` half the thickness along the normal of the repaired scan at
/// the nearest point; the layer of a negative thickness faces against the scan's normals.
fn original_points(removed: &Mesh, repaired: &Mesh, thickness: f64, symmetric: bool) -> (Vec<Point3>, Vec<Vec3>) {
    let bvh = Bvh::build(repaired);
    let sign = if thickness < 0.0 && !symmetric { -1.0 } else { 1.0 };
    let (mut points, mut normals) = (Vec::new(), Vec::new());
    for p in &removed.positions {
        let Some(n) = bvh.closest(p, f64::MAX).and_then(|h| repaired.face_normal(h.triangle)) else { continue };
        points.push(if symmetric { p + n * (0.5 * thickness.abs()) } else { *p });
        normals.push(n * sign);
    }
    (points, normals)
}

/// `main` (the repaired scan, `vertices` points) and the removed points (`extra`, `removed` of them) as one summary.
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
        not_facing: (main.not_facing * vertices as f64 + extra.not_facing as f64) / total,
        within: (main.within * n1 + extra.within * n2) / n,
        rms: ((main.rms.powi(2) * n1 + extra.rms.powi(2) * n2) / n).sqrt(),
        max_abs: main.max_abs.max(extra.max_abs),
        mean: (main.mean * n1 + extra.mean * n2) / n,
    }
}

/// The deviation of the body from the scan as it was: `main` is the repaired scan's, the removed triangles'
/// vertices are added when folds were taken out. `shell` is the repaired scan, the thickness and `symmetric`.
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
    let (points, normals) = original_points(&folds.removed, shell.0, shell.1, shell.2);
    let cmp = CompareIn {
        body: ObjRef::Id(body.0),
        scan: ObjRef::Id(set.scan.0),
        tolerance: Some(set.tolerance),
        max_distance: None,
        chord: None,
        facing: true,
    };
    let extra = compare_points_with(s, body, set.scan, (&points, &normals), &cmp)?;
    let t = &extra.totals;
    folds.stats.set(Some(RemovedStats {
        count: t.count,
        not_facing: t.not_facing,
        within: t.within,
        max_abs: t.max_abs,
    }));
    Ok(merge(&main, vertices, t, points.len()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::synth;

    fn flat() -> Mesh {
        synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 10.0, 10.0, 5, 5)
    }

    fn on_the_surface() -> Mesh {
        Mesh { positions: vec![Point3::new(3.0, 3.0, 0.0)], triangles: Vec::new() }
    }

    #[test]
    fn the_symmetric_layer_is_half_the_wall_out_along_the_normal_of_the_repaired_scan() {
        let (points, normals) = original_points(&on_the_surface(), &flat(), 1.0, true);
        assert!((points[0].z.abs() - 0.5).abs() < 1e-12, "{points:?}");
        assert!(points[0].z * normals[0].z > 0.0, "the shift goes the way the normal points");
        // the sign of the thickness means nothing for a symmetric wall
        let (negative, _) = original_points(&on_the_surface(), &flat(), -1.0, true);
        assert_eq!(negative, points);
    }

    #[test]
    fn the_scan_itself_is_the_layer_for_a_one_sided_wall_and_a_negative_wall_turns_the_normals() {
        let (points, normals) = original_points(&on_the_surface(), &flat(), 1.0, false);
        assert_eq!(points[0], Point3::new(3.0, 3.0, 0.0));
        let (inner, turned) = original_points(&on_the_surface(), &flat(), -1.0, false);
        assert_eq!(inner, points);
        assert!((turned[0] + normals[0]).norm() < 1e-12);
    }
}

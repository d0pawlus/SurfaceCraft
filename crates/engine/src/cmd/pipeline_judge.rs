//! The judgement of `scan.to_solid` on the primitives method: the outcome of a run, the issues that
//! keep it from a pass, the next steps worth running and the verdict.
use super::{
    DeviationSummary, Issue, LeftOut, MAX_FILL_EDGES, MAX_SURFACE_HINTS, MIN_COVERAGE, MIN_MODELLED_SHARE, Modelled,
    NextStep, Settings, Verdict, coverage_issue, issue, step,
};
use crate::cmd::body::BodyOut;
use serde_json::json;
use surfacecraft_deviation::faces::FaceDeviation;
use surfacecraft_doc::ObjectId;

/// The run so far: what the issues and the next steps are read from.
pub(super) struct Outcome {
    pub(super) scan: ObjectId,
    pub(super) body: Option<BodyOut>,
    pub(super) build_error: Option<String>,
    pub(super) deviation: Option<DeviationSummary>,
    pub(super) worst_faces: Vec<FaceDeviation>,
    pub(super) modelled: Vec<Modelled>,
    pub(super) left_out: Vec<LeftOut>,
    pub(super) unassigned: usize,
    pub(super) holes_left: usize,
    pub(super) non_manifold: usize,
}

impl Outcome {
    pub(super) fn closed(&self) -> bool {
        self.body.as_ref().is_some_and(|b| b.volume.is_some_and(|v| v.is_finite() && v > 0.0))
    }

    pub(super) fn skipped(&self) -> usize {
        self.modelled.iter().filter(|m| m.skipped.is_some()).count()
    }

    pub(super) fn covered(&self) -> bool {
        self.deviation.as_ref().is_some_and(|d| d.coverage >= MIN_COVERAGE)
    }

    /// Share of the scan's triangles in the regions that became primitives.
    pub(super) fn modelled_share(&self, total_triangles: usize) -> f64 {
        if total_triangles == 0 {
            return 0.0;
        }
        self.modelled.iter().map(|m| m.triangles).sum::<usize>() as f64 / total_triangles as f64
    }
}

/// What keeps the outcome from a pass; empty when nothing does.
pub(super) fn issues_of(o: &Outcome, set: &Settings, total_triangles: usize) -> Vec<Issue> {
    let mut list = blocking_issues(o, set);
    if !list.is_empty() {
        list.extend(context_issues(o, total_triangles));
    }
    list
}

/// The issues that decide the verdict: no solid, skipped inputs, coverage, tolerance.
fn blocking_issues(o: &Outcome, set: &Settings) -> Vec<Issue> {
    let mut list = Vec::new();
    if let Some(e) = &o.build_error {
        list.push(issue("no_solid", format!("no solid was built: {e}")));
    } else if o.modelled.is_empty() {
        list.push(issue("no_solid", "no region of the scan could be modelled as a primitive".to_string()));
    } else if !o.closed() {
        list.push(issue("no_solid", "the result is not a closed solid (it has no volume)".to_string()));
    }
    if o.skipped() > 0 {
        let msg = format!("{} of {} primitives could not be built into the solid", o.skipped(), o.modelled.len());
        list.push(issue("skipped_inputs", msg));
    }
    if let Some(d) = &o.deviation {
        list.extend(coverage_issue(d, "the solid"));
        if d.count > 0 && d.within < set.min_within {
            let msg = format!(
                "{:.2} % of the measured points are within {} mm ({:.0} % required); max {:.3} mm, RMS {:.3} mm",
                100.0 * d.within,
                set.tolerance,
                100.0 * set.min_within,
                d.max_abs,
                d.rms
            );
            list.push(issue("out_of_tolerance", msg));
        }
    }
    list
}

/// Context for a run that did not pass: what the solid was not built from.
fn context_issues(o: &Outcome, total_triangles: usize) -> Vec<Issue> {
    let mut list = Vec::new();
    let share = o.modelled_share(total_triangles);
    if share < MIN_MODELLED_SHARE {
        let msg = format!(
            "only {:.0} % of the scan lies in regions a primitive fits: the part is mostly freeform, and this command models planes, cylinders, cones, spheres and tori",
            100.0 * share
        );
        list.push(issue("mostly_freeform", msg));
    }
    let left: usize = o.left_out.iter().map(|l| l.triangles).sum();
    if left > 0 {
        let msg = format!("{} regions ({left} triangles) were not modelled", o.left_out.len());
        list.push(issue("left_out_regions", msg));
    }
    if o.unassigned > 0 && total_triangles > 0 {
        let share = 100.0 * o.unassigned as f64 / total_triangles as f64;
        list.push(issue("unassigned_triangles", format!("{} triangles ({share:.1} %) are in no region", o.unassigned)));
    }
    if o.holes_left > 0 {
        list.push(issue("scan_defects", format!("{} holes of the scan were too large to fill", o.holes_left)));
    }
    if o.non_manifold > 0 {
        let msg = format!(
            "{} edges are shared by three or more triangles; regions and holes next to them are unreliable",
            o.non_manifold
        );
        list.push(issue("non_manifold_edges", msg));
    }
    list
}

/// Commands worth running next, most useful first.
pub(super) fn next_steps(o: &Outcome, issues: &[Issue], set: &Settings) -> Vec<NextStep> {
    let has = |code: &str| issues.iter().any(|i| i.code == code);
    let mut steps = Vec::new();
    if issues.is_empty() {
        if let Some(b) = &o.body {
            let why = "the solid passed: export it (pass `path` when files are allowed, else read `data`)";
            steps.push(step("export.step", json!({"objects": [b.id]}), why));
        }
        return steps;
    }
    let primitives: Vec<ObjectId> = o.modelled.iter().map(|m| m.primitive).collect();
    if (has("skipped_inputs") || has("no_solid")) && !primitives.is_empty() {
        let why = "reports the role of each primitive and why an input is skipped, before any cut";
        steps.push(step("body.plan_regions", json!({"primitives": primitives}), why));
    }
    if has("mostly_freeform") {
        let why = "the part is freeform: wrap the whole scan in a patch network instead";
        steps.push(step(
            "scan.to_solid",
            json!({"scan": o.scan, "method": "freeform", "tolerance": set.tolerance}),
            why,
        ));
    }
    if has("out_of_tolerance")
        && o.covered()
        && let Some(b) = &o.body
    {
        let why = "moves the analytic faces of the solid onto the scan (planes shift, radii change)";
        steps.push(step("body.refit_faces", json!({"body": b.id, "mesh": o.scan}), why));
    }
    let freeform = o.left_out.iter().filter(|l| l.reason.starts_with("RMS")).take(MAX_SURFACE_HINTS);
    for l in freeform {
        let why = "a large region that is no primitive: fit a freeform surface, then pass it in `surfaces` of body.from_regions";
        steps.push(step("surface.fit", json!({"region": l.region, "tolerance": set.tolerance}), why));
    }
    if has("scan_defects") {
        let why = "fills the holes that were too large (this drops the regions), then model the scan again";
        steps.push(step("mesh.fill_holes", json!({"object": o.scan, "max_edges": MAX_FILL_EDGES}), why));
        steps.push(step("scan.to_solid", json!({"scan": o.scan}), "segments and models the repaired scan"));
    } else if !has("mostly_freeform") {
        let why = "after correcting the regions (region.merge, region.split, region.grow, region.resegment), model them again";
        steps.push(step("scan.to_solid", json!({"scan": o.scan, "reuse_regions": true}), why));
    }
    steps.push(step(
        "guide.read",
        json!({"topic": "scan-to-solid"}),
        "what each verdict and issue means and how to go on by hand",
    ));
    steps
}

pub(super) fn verdict_of(o: &Outcome, issues: &[Issue]) -> Verdict {
    if issues.is_empty() {
        Verdict::Pass
    } else if o.closed() && o.covered() {
        Verdict::Review
    } else {
        Verdict::Fail
    }
}

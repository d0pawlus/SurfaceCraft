//! `scan.to_solid`: the whole path from a raw scan to a solid, judged against the scan, in one
//! undoable command. It runs the handlers of `scan.qa`, `mesh.clean`, `region.auto_segment`,
//! `primitive.fit`, `primitive.auto_constrain`, `body.from_regions` and `body.compare` in order
//! and adds no geometry of its own; what it adds is the choice of regions, the verdict and the
//! next steps, so that an agent needs one call instead of a dozen and cannot miss a poor result.
use super::deviation::{NOISE_PRIMITIVE_MM, positive, yes};
use super::hybrid::{FaceOf, FromRegionsIn, from_regions};
use super::mesh_edit::{CleanIn, clean};
use super::primitive::{AutoConstrainIn, auto_constrain};
use super::qa::{CompareIn, CompareOut, QaIn, compare, scan_qa};
use super::region::{AutoIn, RegionsOut, auto_segment};
use super::wrap::PATCH_LIMIT;
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use surfacecraft_deviation::noise_estimate;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::segment::SegmentOptions;

#[path = "pipeline_auto.rs"]
mod auto;
#[path = "pipeline_budget.rs"]
mod budget;
#[path = "pipeline_freeform.rs"]
mod freeform;
#[path = "pipeline_hybrid.rs"]
mod hybrid_method;
#[path = "pipeline_judge.rs"]
mod judge;
#[path = "pipeline_prepare.rs"]
mod prepare;
#[path = "pipeline_shell.rs"]
mod shell;
pub use prepare::{ComponentOut, PrepOut, PrepStep};
#[path = "pipeline_rebuild.rs"]
mod rebuild;
#[path = "pipeline_types.rs"]
mod types;
pub use hybrid_method::AnalyticSummary;
pub use types::{DeviationSummary, Issue, LeftOut, Modelled, NextStep, ScanSummary, ToSolidOut};
#[path = "pipeline_regions.rs"]
mod regions;
use judge::{Outcome, issues_of, next_steps, verdict_of};
use regions::{candidates, fit_regions, primitive_share};

/// Default tolerance the solid is held to against the scan (mm).
const TOLERANCE: f64 = 0.2;
/// Default share of the measured scan points that must lie within the tolerance for a pass.
const MIN_WITHIN: f64 = 0.99;
/// A solid that measures less than this share of the scan points does not represent the scan.
const MIN_COVERAGE: f64 = 0.9;
/// Default size limit of the holes `mesh.clean` fills (edges of the boundary loop).
const FILL_HOLES_MAX_EDGES: usize = 400;
/// Largest hole `mesh.fill_holes` accepts (edges), offered when holes were left open.
const MAX_FILL_EDGES: usize = surfacecraft_geom::edit::MAX_FILL_EDGES;
/// A region is modelled when it holds at least this share of the scan's triangles...
const MIN_REGION_SHARE: f64 = 0.01;
/// ...and never fewer triangles than this.
const MIN_REGION_TRIANGLES: usize = 30;
/// Most inputs `body.from_regions` takes.
const MAX_INPUTS: usize = 256;
/// Regions listed in `left_out` (the largest; the counts are always complete).
const MAX_LISTED_LEFT_OUT: usize = 20;
/// Faces listed in `worst_faces`.
const MAX_LISTED_FACES: usize = 5;
/// Left-out regions offered to `surface.fit` in the next steps.
const MAX_SURFACE_HINTS: usize = 3;
/// A scan with less than this share of its triangles in modelled regions is mostly freeform.
const MIN_MODELLED_SHARE: f64 = 0.5;
/// Findings of the quality check that stop the run: nothing can be computed on such a mesh.
const BLOCKING_FINDINGS: [&str; 2] = ["invalid_mesh", "empty_mesh"];

/// How the solid is made.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Method {
    /// Primitives when at least half of the scan lies in regions a primitive fits, else freeform.
    #[default]
    Auto,
    /// Planes, cylinders, cones, spheres and tori fitted to regions (machined, prismatic parts).
    Primitives,
    /// A network of patches over the whole scan (castings, body panels, organic shapes).
    Freeform,
    /// One closed body: the regions a primitive fits are exact analytic faces, the rest of the scan
    /// is covered by patches (parts that are partly machined and partly organic).
    Hybrid,
}

/// Parameters of `scan.to_solid`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ToSolidIn {
    /// Scan mesh id or name.
    pub scan: ObjRef,
    /// Tolerance the solid is held to against the scan (mm, default 0.2).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Share of the measured scan points that must lie within `tolerance` for a pass (0..1,
    /// default 0.99).
    #[serde(default)]
    pub min_within: Option<f64>,
    /// `auto` (default), `primitives`, `freeform` or `hybrid`. `auto` segments the scan and takes
    /// `primitives` when at least half of its triangles lie in regions a primitive fits, else
    /// `freeform`: a watertight network of patches over the whole scan; when the primitives cover a
    /// quarter of the scan or more it also tries `hybrid` and returns it when it is not worse.
    /// `hybrid`: one closed body whose regions that fit a primitive are exact faces of it and the
    /// rest patches (the result reports `analytic`); a closed scan is needed.
    #[serde(default)]
    pub method: Method,
    /// Clean the scan first: debris, spikes and holes (default true). Not done with
    /// `reuse_regions`: cleaning changes the triangles and would drop the regions.
    #[serde(default = "yes")]
    pub clean: bool,
    /// Holes up to this many edges are filled when cleaning (default 400).
    #[serde(default)]
    pub fill_holes_max_edges: Option<usize>,
    /// Model the regions the scan already has instead of segmenting it (default false). For a
    /// second run after the regions were corrected by hand.
    #[serde(default)]
    pub reuse_regions: bool,
    /// Regions with fewer triangles are not modelled (default: 1 % of the scan, at least 30).
    #[serde(default)]
    pub min_region_triangles: Option<usize>,
    /// Snap the primitives to the relations found between them: parallel, perpendicular,
    /// coaxial, equal radii (default true).
    #[serde(default = "yes")]
    pub constrain: bool,
    /// `auto` only: also try the other method when the first does not pass (default true).
    #[serde(default)]
    pub retry: Option<bool>,
    /// Most patches of the network of method `freeform` (default 12000, at most 60000). An explicit
    /// value is a promise and is never exceeded; without it, a network that stops at the default limit
    /// short of the tolerance is built once more with a larger limit when that is likely to help
    /// (`allow_retry`, `retry` of the result).
    #[serde(default)]
    pub max_patches: Option<usize>,
    /// Let method `freeform` build the network once more with a larger patch limit when the first
    /// one stopped at the default limit short of the tolerance (default true; never done when
    /// `max_patches` is given).
    #[serde(default = "yes")]
    pub allow_retry: bool,
    /// Rebuild a closed scan with many handles or non-manifold edges, or that wrapping did not pass on, with `mesh.remesh` and wrap it (again) (default true; for at most the 4 largest parts). The wrap again, the larger-cap retry (`allow_retry`, for the first part in area order that qualifies for it) and the other method of `auto` are extra attempts: one budget of 2 per call (a count, no clock). At most 26 wraps per call in all (a wrap for each of 24 parts, and the 2 extra attempts). `false`: the scan is wrapped as it is.
    #[serde(default = "yes")]
    pub rebuild: bool,
    /// A part of a scan with several pieces is co-equal, and modelled, when its area is at least this share of the largest piece's (0..1, default 0.1). Smaller pieces and debris are reported, not modelled.
    #[serde(default)]
    pub part_share: Option<f64>,
    /// Wall thickness in mm of the solid made from an open scan (no default; not 0, at most half the scan's size): the cleaned scan is thickened and the shell wrapped (method `freeform`); deviation and verdict are measured against the scan. Positive puts the material behind the scanned face, negative in front of it; an error when the scan is closed, or has several co-equal parts (`scan.split` first).
    #[serde(default)]
    pub thickness: Option<f64>,
    /// Put half of `thickness` on each side of the scan (default false; needs `thickness`); the scan is then measured against the face moved half the thickness.
    #[serde(default)]
    pub symmetric: bool,
    /// Keep the thickened mesh in the document (default false: it is removed; needs `thickness`).
    #[serde(default)]
    pub keep_thickened: bool,
    /// Name of the solid (default "solid").
    #[serde(default)]
    pub name: Option<String>,
    /// Which co-equal parts of a scan with several parts are modelled: `all` (default, the largest 24, one solid each), `largest`, or a number of the largest parts.
    #[serde(default)]
    pub components: Option<prepare::Components>,
}

/// How the run ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Verdict {
    /// A closed solid that matches the scan within the tolerance; nothing was skipped.
    Pass,
    /// A closed solid with issues to look at.
    Review,
    /// No solid, or a solid that does not represent the scan.
    Fail,
}

/// The parameters after validation.
struct Settings {
    /// Extra attempts of this call draw from it (see `pipeline_budget`).
    budget: budget::CallBudget,
    scan: ObjectId,
    tolerance: f64,
    min_within: f64,
    min_triangles: Option<usize>,
    max_patches: Option<usize>,
    allow_retry: bool,
    rebuild: bool,
    name: String,
}

/// `inconsistent_orientation` needs at least this share of the missing coverage to lie on the solid
/// but face away from it; noise at sharp edges alone gives a few percent that do.
const ORIENTATION_SHARE_OF_MISSING: f64 = 0.5;
/// Above this share of the scan facing away, it is the scan that is inside out, not the solid.
const INSIDE_OUT_SHARE: f64 = 0.9;

/// The issue of a coverage below [`MIN_COVERAGE`], told by what is wrong. `scan_inside_out`: nearly
/// every point faces away (the kernel turns a solid outward, so the scan's normals point inward).
/// `inconsistent_orientation`: much of what is missing lies on `what` but faces away from it (its
/// faces are turned the wrong way). Otherwise `low_coverage`: the scan is not near `what`.
pub(super) fn coverage_issue(d: &DeviationSummary, what: &str) -> Option<Issue> {
    if d.coverage >= MIN_COVERAGE {
        return None;
    }
    let missing = 1.0 - d.coverage;
    if d.not_facing >= INSIDE_OUT_SHARE {
        let msg = format!(
            "{:.1} % of the scan points face away from {what}: the scan is inside out (its normals point inward); flip the normals of the scan and run again",
            100.0 * d.not_facing
        );
        return Some(issue("scan_inside_out", msg));
    }
    if d.not_facing >= ORIENTATION_SHARE_OF_MISSING * missing {
        let msg = format!(
            "{:.1} % of the scan points lie on {what} but face away from it (only {:.1} % face it): its faces are turned the wrong way",
            100.0 * d.not_facing,
            100.0 * d.coverage
        );
        return Some(issue("inconsistent_orientation", msg));
    }
    let far = (missing - d.not_facing).max(0.0);
    let msg = format!(
        "only {:.1} % of the scan points lie on {what} and face it (at least {:.0} % expected): {:.1} % lie on it but face away, {:.1} % are farther away",
        100.0 * d.coverage,
        100.0 * MIN_COVERAGE,
        100.0 * d.not_facing,
        100.0 * far
    );
    Some(issue("low_coverage", msg))
}

fn issue(code: &str, message: String) -> Issue {
    Issue { code: code.to_string(), message }
}

fn step(command: &str, params: Value, why: &str) -> NextStep {
    NextStep { command: command.to_string(), params, why: why.to_string() }
}

fn settings(s: &Session, i: &ToSolidIn) -> CmdResult<Settings> {
    let scan = i.scan.resolve(s, ObjectKind::Mesh)?;
    let tolerance = positive(i.tolerance.unwrap_or(TOLERANCE), "tolerance")?;
    let min_within = i.min_within.unwrap_or(MIN_WITHIN);
    if !(min_within.is_finite() && (0.0..=1.0).contains(&min_within)) {
        return Err(CmdError::Params("min_within must be within 0..=1".into()));
    }
    if i.thickness.is_some() && i.method == Method::Hybrid {
        return Err(CmdError::Params(
            "thickness makes a shell from an open scan with method `freeform` (or `auto`): it has no meaning with method `hybrid`".into(),
        ));
    }
    if i.reuse_regions && i.method == Method::Freeform {
        return Err(CmdError::Params("reuse_regions has no meaning with method `freeform`".into()));
    }
    if i.reuse_regions && s.doc.regions_of(scan).next().is_none() {
        return Err(CmdError::Params("reuse_regions: the scan has no regions (run without it to segment)".into()));
    }
    if let Some(m) = i.max_patches
        && !(freeform::MIN_PATCHES..=PATCH_LIMIT).contains(&m)
    {
        return Err(CmdError::Params(format!("max_patches must be {}..={PATCH_LIMIT}", freeform::MIN_PATCHES)));
    }
    Ok(Settings {
        budget: budget::CallBudget::default(),
        scan,
        tolerance,
        min_within,
        min_triangles: i.min_region_triangles,
        max_patches: i.max_patches,
        allow_retry: i.allow_retry,
        rebuild: i.rebuild,
        name: i.name.clone().unwrap_or_else(|| "solid".to_string()),
    })
}

fn triangle_count(s: &Session, scan: ObjectId) -> usize {
    s.doc.mesh(scan).map_or(0, |m| m.mesh.triangle_count())
}

fn message_of(e: &CmdError) -> &str {
    match e {
        CmdError::Params(m) | CmdError::Failed(m) | CmdError::Unavailable(m) => m,
    }
}

/// Copies the role and faces `body.from_regions` reports into the modelled list.
fn apply_roles(modelled: &mut [Modelled], faces_of: &[FaceOf]) {
    for m in modelled.iter_mut() {
        let Some(f) = faces_of.iter().find(|f| f.object == m.primitive) else { continue };
        m.role.clone_from(&f.role);
        m.faces.clone_from(&f.faces);
        m.skipped = f.how.strip_prefix("skipped: ").map(str::to_string);
    }
}

fn summary(c: &CompareOut, scan_vertices: usize) -> DeviationSummary {
    let t = &c.totals;
    let share = |n: usize| if scan_vertices > 0 { (n as f64 / scan_vertices as f64).min(1.0) } else { 0.0 };
    DeviationSummary {
        count: t.count,
        coverage: share(t.count),
        not_facing: share(t.not_facing),
        within: t.within,
        rms: t.rms,
        max_abs: t.max_abs,
        mean: t.mean,
    }
}

/// Cleans the scan when asked and reports what the scan was and is.
fn prepare(s: &mut Session, i: &ToSolidIn, scan: ObjectId) -> CmdResult<ScanSummary> {
    let qa = scan_qa(s, QaIn { object: ObjRef::Id(scan.0) })?;
    if let Some(f) = qa.findings.iter().find(|f| BLOCKING_FINDINGS.contains(&f.code.as_str())) {
        return Err(CmdError::Failed(format!("the scan cannot be modelled: {}", f.message)));
    }
    let (mut filled, mut left, mut non_manifold) = (0, qa.holes, qa.non_manifold_edges);
    if i.clean && !i.reuse_regions {
        let edit = clean(
            s,
            CleanIn {
                object: ObjRef::Id(scan.0),
                min_component_fraction: None,
                min_component_triangles: None,
                outlier_ratio: None,
                fill_holes_max_edges: Some(i.fill_holes_max_edges.unwrap_or(FILL_HOLES_MAX_EDGES)),
            },
        )?;
        filled = edit.holes_filled.unwrap_or(0);
        left = edit.holes_left.unwrap_or(0);
        if non_manifold > 0 {
            // cleaning removes debris and spikes, which is where most such edges are
            non_manifold = scan_qa(s, QaIn { object: ObjRef::Id(scan.0) })?.non_manifold_edges;
        }
    }
    let noise = s.doc.mesh(scan).map(|m| noise_estimate(&m.mesh)).filter(|e| e.smooth).map(|e| e.sigma);
    Ok(ScanSummary {
        triangles_before: qa.triangles,
        triangles: triangle_count(s, scan),
        holes_before: qa.holes,
        holes_filled: filled,
        holes_left: left,
        non_manifold_edges: non_manifold,
        noise_sigma: noise,
        noise_used: None,
        noise_source: None,
        noise_spread: None,
        noise_above_tolerance: false,
        findings: qa.findings,
    })
}

/// Builds the solid from the modelled regions and measures it against the scan.
fn build(s: &mut Session, set: &Settings, o: &mut Outcome) -> CmdResult<()> {
    if o.modelled.is_empty() {
        return Ok(());
    }
    let primitives: Vec<ObjRef> = o.modelled.iter().map(|m| ObjRef::Id(m.primitive.0)).collect();
    let params = FromRegionsIn {
        primitives,
        surfaces: Vec::new(),
        margin: None,
        name: Some(set.name.clone()),
        square_planes_deg: None,
    };
    let built = match from_regions(s, params) {
        Ok(b) => b,
        Err(e) => {
            o.build_error = Some(message_of(&e).to_string());
            return Ok(());
        }
    };
    apply_roles(&mut o.modelled, &built.faces_of);
    let cmp = compare(
        s,
        CompareIn {
            body: ObjRef::Id(built.body.id.0),
            scan: ObjRef::Id(set.scan.0),
            tolerance: Some(set.tolerance),
            max_distance: None,
            chord: None,
            facing: true,
        },
    )?;
    let vertices = s.doc.mesh(set.scan).map_or(0, |m| m.mesh.vertex_count());
    o.deviation = Some(summary(&cmp, vertices));
    o.worst_faces = cmp.per_face.into_iter().filter(|f| f.count > 0 && f.within < 1.0).take(MAX_LISTED_FACES).collect();
    o.body = Some(built.body);
    Ok(())
}

/// Snaps the primitives to the relations between them; returns how many were imposed. Relations
/// are an improvement, not a requirement: a refusal leaves the fits as they are.
fn relate(s: &mut Session, modelled: &[Modelled], sigma: Option<f64>) -> usize {
    if modelled.len() < 2 {
        return 0;
    }
    let params = AutoConstrainIn {
        objects: Some(modelled.iter().map(|m| ObjRef::Id(m.primitive.0)).collect()),
        angle_tol_deg: None,
        length_tol: None,
        apply: None,
        symmetry_planes: None,
        detect_symmetry: None,
        noise_sigma: sigma,
    };
    auto_constrain(s, params).map_or(0, |r| if r.applied { r.relations.len() } else { 0 })
}

pub(super) fn to_solid(s: &mut Session, i: ToSolidIn) -> CmdResult<ToSolidOut> {
    shell::check(&i)?;
    let set = settings(s, &i)?;
    if i.thickness.is_some() {
        return shell::run(s, &set, &i);
    }
    let scan_report = prepare(s, &i, set.scan)?;
    if let Some(done) = prepare::split_parts(s, &i, &set, &scan_report)? {
        return Ok(done);
    }
    solve(s, &i, set, scan_report)
}

/// Models one scan (a part, or the whole scan) once it is cleaned: the given method, or `auto`'s
/// ordered attempts (see `pipeline_auto`); the attempts are listed in the result.
fn solve(s: &mut Session, i: &ToSolidIn, set: Settings, scan_report: ScanSummary) -> CmdResult<ToSolidOut> {
    let report = auto::run(s, i, &set, scan_report)?;
    Ok(ToSolidOut { attempts: report.attempts, ..report.result })
}

/// Segments the scan into regions at the caller's tolerance (unless the regions are reused) and
/// returns what the segmentation reports, the noise it used among it.
fn segment_scan(s: &mut Session, i: &ToSolidIn, set: &Settings) -> CmdResult<Option<RegionsOut>> {
    if i.reuse_regions {
        return Ok(None);
    }
    let options = SegmentOptions { tolerance: set.tolerance, ..SegmentOptions::default() };
    auto_segment(s, AutoIn { mesh: ObjRef::Id(set.scan.0), options, replace: true }).map(Some)
}

/// The smallest region the primitives method models, and the RMS above which a fit is no primitive.
fn region_limits(set: &Settings, total: usize, scan_report: &ScanSummary) -> (usize, f64) {
    let min_triangles = set
        .min_triangles
        .unwrap_or_else(|| ((total as f64 * MIN_REGION_SHARE).ceil() as usize).max(MIN_REGION_TRIANGLES));
    (min_triangles, NOISE_PRIMITIVE_MM.max(2.0 * scan_report.noise_sigma.unwrap_or(0.0)))
}

/// Share of the scan's triangles in the regions a primitive fits (freeform regions have no kind and
/// never count), and how many regions that is: the heuristic that orders the attempts of `method: auto`.
fn scan_primitive_share(s: &Session, set: &Settings, scan_report: &ScanSummary) -> (f64, usize) {
    let (list, _) = candidates(s, set.scan);
    let total = triangle_count(s, set.scan);
    let (min_triangles, rms_limit) = region_limits(set, total, scan_report);
    let regions = list.iter().filter(|c| regions::refusal(c, min_triangles, rms_limit, 0).is_none()).count();
    (primitive_share(&list, min_triangles, rms_limit, total), regions)
}

/// The primitives method on the regions the scan has: fits, relates and builds.
fn primitives_solid(s: &mut Session, i: &ToSolidIn, set: &Settings, scan_report: ScanSummary) -> CmdResult<ToSolidOut> {
    let (list, unassigned) = candidates(s, set.scan);
    let total = triangle_count(s, set.scan);
    let (min_triangles, rms_limit) = region_limits(set, total, &scan_report);
    let (modelled, left) = fit_regions(s, &list, min_triangles, rms_limit);
    let relations = if i.constrain { relate(s, &modelled, scan_report.noise_sigma) } else { 0 };
    let mut o = Outcome {
        scan: set.scan,
        body: None,
        build_error: None,
        deviation: None,
        worst_faces: Vec::new(),
        modelled,
        left_out: left,
        unassigned,
        holes_left: scan_report.holes_left,
        non_manifold: scan_report.non_manifold_edges,
    };
    build(s, set, &mut o)?;
    let issues = issues_of(&o, set, total);
    let next = next_steps(&o, &issues, set);
    let verdict = verdict_of(&o, &issues);
    let left_out_regions = o.left_out.len();
    let left_out_triangles = o.left_out.iter().map(|l| l.triangles).sum();
    let mut listed = o.left_out;
    listed.truncate(MAX_LISTED_LEFT_OUT);
    Ok(ToSolidOut {
        verdict,
        method: Method::Primitives,
        scan: set.scan,
        body: o.body,
        patches: None,
        retry: None,
        tolerance: set.tolerance,
        deviation: o.deviation,
        worst_faces: o.worst_faces,
        scan_report,
        modelled: o.modelled,
        left_out: listed,
        left_out_regions,
        left_out_triangles,
        unassigned_triangles: o.unassigned,
        relations,
        issues,
        next_steps: next,
        attempts: Vec::new(),
        prep: PrepOut::default(),
        thickened: None,
        analytic: None,
    })
}

/// The `scan.to_solid` command.
pub static COMMANDS: &[CommandSpec] = &[command!(
    "scan.to_solid",
    "Scan to solid",
    "Solid",
    true,
    "From raw scan `scan` to a closed solid in one step, measured against the scan. Prismatic parts (`method` primitives): cleans the scan, segments it, fits a primitive to each sizeable region, relates the primitives and builds the solid. Organic parts, castings and body panels (`method` freeform): wraps the whole scan in a watertight network of patches. `method` hybrid builds one closed body with exact analytic faces on the regions a primitive fits and patches on the rest (`analytic`). `method` auto (default) tries the suggested method, then the other if it fails, and when the primitives cover a quarter of the scan also a hybrid body; it returns the best (`attempts`). Returns `verdict` (pass, review or fail), the deviation, `issues`, and `next_steps` with ready parameters. A poor model is a verdict, not an error; regions stay for manual work. `reuse_regions` models corrected regions again (without cleaning).",
    ToSolidIn,
    ToSolidOut,
    super::mesh::scan_size::to_solid_checked
)];

#[cfg(test)]
mod coverage_tests {
    use super::*;

    fn summary_of(coverage: f64, not_facing: f64) -> DeviationSummary {
        DeviationSummary { count: 100, coverage, not_facing, within: 1.0, rms: 0.0, max_abs: 0.0, mean: 0.0 }
    }

    #[test]
    fn a_solid_that_covers_the_scan_has_no_coverage_issue() {
        assert!(coverage_issue(&summary_of(0.95, 0.0), "the solid").is_none());
    }

    fn code(coverage: f64, not_facing: f64) -> String {
        coverage_issue(&summary_of(coverage, not_facing), "the solid").expect("an issue").code
    }

    #[test]
    fn points_that_lie_on_the_solid_but_face_away_are_an_orientation_issue_not_missing_scan() {
        assert_eq!(code(0.71, 0.29), "inconsistent_orientation");
    }

    #[test]
    fn a_few_percent_facing_away_beside_a_real_gap_is_low_coverage_with_both_figures() {
        // 85 % cover, 6 % face away, 9 % are far: the gap is the problem
        let issue = coverage_issue(&summary_of(0.85, 0.06), "the solid").expect("an issue");
        assert_eq!(issue.code, "low_coverage");
        assert!(issue.message.contains("85.0 %") && issue.message.contains("6.0 %") && issue.message.contains("9.0 %"));
    }

    #[test]
    fn the_orientation_issue_starts_at_half_of_the_missing_share() {
        // missing 0.20: 0.10 facing away is the boundary
        assert_eq!(code(0.80, 0.10), "inconsistent_orientation");
        assert_eq!(code(0.80, 0.0999), "low_coverage");
    }

    #[test]
    fn a_scan_whose_points_nearly_all_face_away_is_inside_out() {
        assert_eq!(code(0.05, 0.95), "scan_inside_out");
        assert_eq!(code(0.10, 0.90), "scan_inside_out");
        assert_eq!(code(0.20, 0.80), "inconsistent_orientation");
    }

    #[test]
    fn points_far_from_the_solid_are_low_coverage() {
        assert_eq!(code(0.5, 0.05), "low_coverage");
    }
}

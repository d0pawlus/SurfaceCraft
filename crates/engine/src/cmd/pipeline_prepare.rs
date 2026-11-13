//! Preparation of messy scans for `scan.to_solid` (P9-O9): a scan that holds several significant
//! parts is split and every part modelled on its own (`scan.split`), and a closed scan that wrapping
//! does not reach a pass on is rebuilt first (`mesh.remesh`) and wrapped again. A scan that already
//! works takes exactly the path it took before: no preparation, an empty `preparation` list.
use super::freeform::freeform_on;
use super::{
    CmdError, CmdResult, DeviationSummary, Issue, Method, NextStep, ScanSummary, Settings, ToSolidIn, ToSolidOut,
    Verdict, issue, message_of, prepare, settings, solve,
};
use crate::Session;
use crate::cmd::ObjRef;
use crate::cmd::body::BodyOut;
use crate::cmd::mesh_remesh::{RemeshIn, remesh_cmd};
use crate::cmd::scan_split::{self, MIN_SHARE};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use surfacecraft_doc::ObjectId;
use surfacecraft_geom::Mesh;
use surfacecraft_geom::pieces::pieces;

/// Parts modelled in one run at most (the largest); the rest are reported.
pub(super) const MAX_COMPONENTS: usize = 24;
/// A closed scan with at least this genus is rebuilt before it is wrapped: thousands of handles
/// stall the mesh reduction, and the remesh keeps the genuine ones.
const HANDLE_TRIGGER: usize = 64;
/// ... or with at least this many non-manifold edges.
const NON_MANIFOLD_TRIGGER: usize = 20;
/// The surface of a remeshed scan has about this many quads at most: the voxel grows with the area.
const REMESH_QUAD_BUDGET: f64 = 1_500_000.0;

/// Which parts of a scan with several significant parts are modelled.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(untagged)]
pub enum Components {
    /// `all` (the largest 24) or `largest`.
    Word(String),
    /// The number of largest parts.
    Count(usize),
}

impl Components {
    fn limit(c: &Option<Components>) -> CmdResult<usize> {
        match c {
            None => Ok(MAX_COMPONENTS),
            Some(Components::Word(w)) if w == "all" => Ok(MAX_COMPONENTS),
            Some(Components::Word(w)) if w == "largest" => Ok(1),
            Some(Components::Count(n)) if *n >= 1 => Ok((*n).min(MAX_COMPONENTS)),
            Some(_) => Err(CmdError::Params(
                "components must be \"largest\", \"all\" or a number of parts (at least 1)".into(),
            )),
        }
    }
}

/// One preparation step.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct PrepStep {
    /// `split` or `remesh`.
    pub step: String,
    /// Why it was done (or not possible).
    pub reason: String,
    /// The figures behind it.
    pub numbers: Value,
}

/// The result for one part of a scan.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct ComponentOut {
    /// The mesh of the part (made by the split).
    pub mesh: ObjectId,
    /// Its name.
    pub name: String,
    /// Its triangles.
    pub triangles: usize,
    /// How its run ended.
    pub verdict: Verdict,
    /// The method used.
    pub method: Method,
    /// Its solid.
    pub body: Option<BodyOut>,
    /// Patches (freeform).
    pub patches: Option<usize>,
    /// The solid against the part.
    pub deviation: Option<DeviationSummary>,
    /// What kept it from a pass.
    pub issues: Vec<Issue>,
    /// What was done to the part first.
    pub preparation: Vec<PrepStep>,
}

fn rank(v: Verdict) -> u8 {
    match v {
        Verdict::Pass => 2,
        Verdict::Review => 1,
        Verdict::Fail => 0,
    }
}

fn worst(a: Verdict, b: Verdict) -> Verdict {
    if rank(a) <= rank(b) { a } else { b }
}

fn mesh_of(s: &Session, id: ObjectId) -> Option<std::sync::Arc<Mesh>> {
    s.doc.mesh(id).map(|m| m.mesh.clone())
}

/// Splits a scan with several significant parts and models each. `None` for a scan with one part
/// (the usual case: nothing changes).
pub(super) fn split_parts(
    s: &mut Session,
    i: &ToSolidIn,
    set: &Settings,
    report: &ScanSummary,
) -> CmdResult<Option<ToSolidOut>> {
    let limit = Components::limit(&i.components)?;
    if i.reuse_regions {
        return Ok(None);
    }
    let Some(mesh) = mesh_of(s, set.scan) else { return Ok(None) };
    let a = scan_split::analyse(&mesh, MIN_SHARE, true);
    if a.parts.len() < 2 {
        return Ok(None);
    }
    let distinct = a.distinct();
    if distinct.len() < 2 {
        return Ok(None);
    }
    let base = s.doc.mesh(set.scan).map_or_else(|| "scan".to_string(), |m| m.name.clone());
    let total = mesh.triangle_count() as f64;
    let chosen: Vec<usize> = distinct.iter().copied().take(limit).collect();
    let mut comps = Vec::new();
    for &k in &chosen {
        let part = scan_split::add_part(s, &base, k, &a.parts[k], &mesh, total);
        comps.push(run_part(s, i, set, &part.name, part.id, comps.len() + 1));
    }
    let skipped_cavities = a.parts.len() - distinct.len();
    let beyond_limit = distinct.len() - chosen.len() + a.beyond;
    let debris: usize = a.rest.iter().map(|p| p.triangles.len()).sum();
    let step = PrepStep {
        step: "split".into(),
        reason: format!(
            "the scan holds {} significant parts (each at least {:.0} % of its triangles or area), modelled one by one",
            distinct.len(),
            100.0 * MIN_SHARE
        ),
        numbers: json!({
            "parts": distinct.len(), "modelled": chosen.len(), "beyond_limit": beyond_limit, "cavity_shells_skipped": skipped_cavities,
            "debris_pieces": a.rest.len(), "debris_triangles": debris, "limit": MAX_COMPONENTS,
        }),
    };
    Ok(Some(combine(set, report.clone(), comps, step, beyond_limit, skipped_cavities)))
}

/// Models one part on its own.
fn run_part(
    s: &mut Session,
    i: &ToSolidIn,
    set: &Settings,
    name: &str,
    id: ObjectId,
    n: usize,
) -> (ComponentOut, ToSolidOut) {
    let part_in = ToSolidIn {
        scan: ObjRef::Id(id.0),
        clean: false,
        name: Some(format!("{}-{n}", set.name)),
        components: None,
        ..i.clone()
    };
    let triangles = mesh_of(s, id).map_or(0, |m| m.triangle_count());
    let mut out = model_part(s, &part_in, set, id);
    if i.method == Method::Auto && out.verdict == Verdict::Fail && out.body.is_none() {
        // the primitives found no closed solid in this part: model it as freeform instead
        let again = model_part(s, &ToSolidIn { method: Method::Freeform, ..part_in.clone() }, set, id);
        if again.body.is_some() {
            let step = PrepStep {
                step: "freeform_fallback".into(),
                reason: "the primitives built no solid for this part, so it was wrapped in patches".into(),
                numbers: json!({"primitive_issues": out.issues.iter().map(|x| x.code.clone()).collect::<Vec<_>>()}),
            };
            out = again;
            out.preparation.insert(0, step);
        }
    }
    let comp = ComponentOut {
        mesh: id,
        name: name.to_string(),
        triangles,
        verdict: out.verdict,
        method: out.method,
        body: out.body.clone(),
        patches: out.patches,
        deviation: out.deviation.clone(),
        issues: out.issues.clone(),
        preparation: out.preparation.clone(),
    };
    (comp, out)
}

/// One run of the single-scan pipeline on a part; an error becomes a failed result.
fn model_part(s: &mut Session, part_in: &ToSolidIn, set: &Settings, id: ObjectId) -> ToSolidOut {
    let run =
        settings(s, part_in).and_then(|pset| prepare(s, part_in, pset.scan).and_then(|r| solve(s, part_in, pset, r)));
    run.unwrap_or_else(|e| failure(set, id, message_of(&e)))
}

fn failure(set: &Settings, id: ObjectId, why: &str) -> ToSolidOut {
    let report = ScanSummary {
        triangles_before: 0,
        triangles: 0,
        holes_before: 0,
        holes_filled: 0,
        holes_left: 0,
        non_manifold_edges: 0,
        noise_sigma: None,
        findings: Vec::new(),
    };
    ToSolidOut {
        verdict: Verdict::Fail,
        method: Method::Freeform,
        scan: id,
        body: None,
        patches: None,
        tolerance: set.tolerance,
        deviation: None,
        worst_faces: Vec::new(),
        scan_report: report,
        modelled: Vec::new(),
        left_out: Vec::new(),
        left_out_regions: 0,
        left_out_triangles: 0,
        unassigned_triangles: 0,
        relations: 0,
        issues: vec![issue("no_solid", format!("the part could not be modelled: {why}"))],
        next_steps: Vec::new(),
        preparation: Vec::new(),
        components: Vec::new(),
    }
}

/// One result for the whole scan: the worst verdict, the issues of every part, the largest part's figures.
fn combine(
    set: &Settings,
    report: ScanSummary,
    comps: Vec<(ComponentOut, ToSolidOut)>,
    step: PrepStep,
    beyond_limit: usize,
    cavities: usize,
) -> ToSolidOut {
    let mut verdict = comps.iter().fold(Verdict::Pass, |v, c| worst(v, c.0.verdict));
    let mut issues: Vec<Issue> = Vec::new();
    for (n, (c, _)) in comps.iter().enumerate() {
        issues.extend(
            c.issues.iter().map(|x| Issue {
                code: x.code.clone(),
                message: format!("part {} ({}): {}", n + 1, c.name, x.message),
            }),
        );
    }
    if beyond_limit > 0 {
        let msg = format!(
            "{beyond_limit} more significant parts were not modelled (at most {MAX_COMPONENTS} are, the largest first); run `scan.split` and model them by hand"
        );
        issues.push(issue("parts_beyond_limit", msg));
    }
    if cavities > 0 {
        let msg = format!(
            "{cavities} inner shells (cavity skins of hollow parts) were not modelled: the solids are the outer shells"
        );
        issues.push(issue("nested_shell_skipped", msg));
    }
    if (beyond_limit > 0 || cavities > 0) && verdict == Verdict::Pass {
        verdict = Verdict::Review;
    }
    let mut next: Vec<NextStep> = Vec::new();
    for (_, o) in &comps {
        for st in &o.next_steps {
            if next.len() < 6 && !next.iter().any(|x| x.command == st.command && x.params == st.params) {
                next.push(st.clone());
            }
        }
    }
    let first = comps.first().map(|c| c.1.clone());
    let (components, outs): (Vec<ComponentOut>, Vec<ToSolidOut>) = comps.into_iter().unzip();
    let mut out = first.unwrap_or_else(|| failure(set, set.scan, "no part"));
    drop(outs);
    out.verdict = verdict;
    out.scan = set.scan;
    out.scan_report = report;
    out.issues = issues;
    out.next_steps = next;
    out.preparation = vec![step];
    out.components = components;
    out
}

/// Evidence from the cleaned scan that rebuilding it first is worth it: it is closed and has many
/// handles or non-manifold edges. Returns the figures.
fn remesh_evidence(mesh: &Mesh) -> Option<(bool, Value)> {
    let ps = pieces(mesh);
    let big: Vec<_> = ps.iter().filter(|p| p.triangles.len() * 100 >= mesh.triangle_count()).collect();
    // flaps and slivers leave tiny loops; an opening wider than `mesh.remesh` fills makes the scan open
    let wide = surfacecraft_geom::remesh::Params::default().max_hole * std::f64::consts::PI;
    let closed = !big.is_empty() && big.iter().all(|p| p.loops.iter().all(|l| l.length <= wide));
    let genus: usize = big.iter().filter_map(|p| p.genus()).sum();
    let non_manifold: usize = ps.iter().map(|p| p.non_manifold_edges).sum();
    let strong = genus >= HANDLE_TRIGGER || non_manifold >= NON_MANIFOLD_TRIGGER;
    closed.then(|| (strong, json!({"genus": genus, "non_manifold_edges": non_manifold})))
}

/// The voxel for the scan: the default of `mesh.remesh`, or coarser when the area would give more quads than wrapping can use.
fn remesh_voxel(mesh: &Mesh, tolerance: f64) -> f64 {
    (1.5 * tolerance).max((mesh.area() / REMESH_QUAD_BUDGET).sqrt())
}

/// Remeshes the scan; returns the new mesh and the step, or the step that says why it was not done.
fn remesh_scan(
    s: &mut Session,
    set: &Settings,
    reason: &str,
    figures: Value,
) -> Result<(ObjectId, PrepStep), PrepStep> {
    let Some(mesh) = mesh_of(s, set.scan) else { return Err(refused(reason, "no mesh", figures)) };
    let voxel = remesh_voxel(&mesh, set.tolerance);
    let input = RemeshIn {
        object: ObjRef::Id(set.scan.0),
        tolerance: Some(set.tolerance),
        voxel: None,
        min_feature: None,
        max_hole: None,
        min_component_fraction: None,
        name: Some(format!("{}-remesh", set.name)),
    };
    let input = RemeshIn { voxel: (voxel > 1.5 * set.tolerance).then_some(voxel), ..input };
    match remesh_cmd(s, input) {
        Ok(r) => {
            let numbers = json!({
                "voxel": r.voxel, "triangles": r.triangles, "genus_before": r.genus_before, "genus_after": r.genus_after,
                "non_manifold_before": r.non_manifold_before, "feature_removed": r.feature_removed,
                "max_deviation_to_scan": r.original_to_result.max, "p99_deviation_to_scan": r.original_to_result.p99,
                "evidence": figures,
            });
            Ok((r.id, PrepStep { step: "remesh".into(), reason: reason.into(), numbers }))
        }
        Err(e) => Err(refused(reason, message_of(&e), figures)),
    }
}

fn refused(reason: &str, why: &str, figures: Value) -> PrepStep {
    PrepStep {
        step: "remesh".into(),
        reason: format!("{reason}; not done: {why}"),
        numbers: json!({"refused": why, "evidence": figures}),
    }
}

fn better(a: &ToSolidOut, b: &ToSolidOut) -> bool {
    let within = |o: &ToSolidOut| o.deviation.as_ref().map_or(0.0, |d| d.within);
    (rank(a.verdict), within(a)) > (rank(b.verdict), within(b))
}

/// Removes what a rejected attempt left in the document.
fn discard(s: &mut Session, out: &ToSolidOut, remeshed: Option<ObjectId>) {
    if let Some(b) = &out.body {
        s.doc.remove(b.id);
    }
    if let Some(m) = remeshed {
        s.doc.remove(m);
    }
}

/// The freeform method with its preparation: the scan is wrapped as it is; when that does not pass
/// and the scan is closed, it is rebuilt and wrapped again, and the better result is kept. A scan
/// with strong evidence (many handles or non-manifold edges) is rebuilt first.
pub(super) fn freeform_prepared(s: &mut Session, set: &Settings, report: ScanSummary) -> CmdResult<ToSolidOut> {
    let evidence = mesh_of(s, set.scan).and_then(|m| remesh_evidence(&m));
    let rebuild_first = evidence.as_ref().is_some_and(|e| e.0);
    if rebuild_first && let Some((_, figures)) = &evidence {
        let reason = "the scan has many handles or non-manifold edges that stall the reduction";
        if let Ok((id, step)) = remesh_scan(s, set, reason, figures.clone()) {
            let mut out = freeform_on(s, set, id, report.clone())?;
            if out.verdict == Verdict::Pass {
                out.preparation.push(step);
                return Ok(out);
            }
            let base = freeform_on(s, set, set.scan, report)?;
            return Ok(choose(s, base, out, step, id));
        }
    }
    let base = freeform_on(s, set, set.scan, report.clone())?;
    let Some((_, figures)) = evidence.filter(|_| !rebuild_first) else { return Ok(base) };
    if base.verdict == Verdict::Pass {
        return Ok(base);
    }
    let reason = "wrapping the scan as it is did not pass; the scan is closed and is rebuilt as a clean surface";
    match remesh_scan(s, set, reason, figures) {
        Ok((id, step)) => {
            let rebuilt = freeform_on(s, set, id, report)?;
            Ok(choose(s, base, rebuilt, step, id))
        }
        Err(step) => {
            let mut out = base;
            out.preparation.push(step);
            Ok(out)
        }
    }
}

/// Keeps the better of the plain and the rebuilt result and removes the other.
fn choose(s: &mut Session, mut base: ToSolidOut, mut rebuilt: ToSolidOut, step: PrepStep, id: ObjectId) -> ToSolidOut {
    if better(&rebuilt, &base) {
        discard(s, &base, None);
        rebuilt.preparation.push(step);
        rebuilt
    } else {
        discard(s, &rebuilt, Some(id));
        let note = PrepStep {
            step: step.step,
            reason: format!("{}; the rebuilt scan did not model better and was discarded", step.reason),
            numbers: step.numbers,
        };
        base.preparation.push(note);
        base
    }
}

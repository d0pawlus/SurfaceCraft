//! Preparation of messy scans for `scan.to_solid` (P9-O9), part two: a closed scan with topological
//! noise is rebuilt (`mesh.remesh`) and wrapped, and a closed scan that wrapping does not pass on is
//! rebuilt and wrapped again, the better result kept by the rule of the larger-cap retry (`judge`).
//! `rebuild: false` turns all of it off. At most one rebuild-and-wrap-again per part, and per call
//! only for the largest [`super::prepare`] parts.
use super::freeform::{Attempt, RetryReason, freeform_on, judge};
use super::prepare::check_cancel;
use super::{CmdResult, PrepStep, ScanSummary, Settings, ToSolidOut, Verdict, message_of};
use crate::Session;
use crate::cmd::mesh_remesh::{RemeshIn, remesh_cmd};
use crate::cmd::{ObjRef, is_cancelled};
use serde_json::{Value, json};
use surfacecraft_doc::ObjectId;
use surfacecraft_geom::Mesh;
use surfacecraft_geom::pieces::pieces;
use surfacecraft_geom::remesh::MAX_FLAP_EDGES;

/// A closed scan with at least this genus is rebuilt before it is wrapped: thousands of handles stall
/// the mesh reduction, and the remesh keeps the genuine ones.
const HANDLE_TRIGGER: usize = 64;
/// ... or with at least this many non-manifold edges.
const NON_MANIFOLD_TRIGGER: usize = 20;
/// Topology evidence lower than the triggers above: with it a closed result that is short of the tolerance
/// may still be a victim of the topology, so the rebuild-and-wrap-again is tried; without it a closed
/// result short of the tolerance is limited by the patch budget, which a rebuild does not change.
const SOFT_HANDLES: usize = 16;
const SOFT_NON_MANIFOLD: usize = 5;
/// The surface of a rebuilt scan has about this many quads at most: the voxel grows with the area.
const REMESH_QUAD_BUDGET: f64 = 1_500_000.0;

/// Wall-clock time of a step; zero where the clock is not available (the browser build).
pub(super) struct Stopwatch(#[cfg(not(target_arch = "wasm32"))] std::time::Instant);

impl Stopwatch {
    pub(super) fn start() -> Self {
        #[cfg(not(target_arch = "wasm32"))]
        {
            Self(std::time::Instant::now())
        }
        #[cfg(target_arch = "wasm32")]
        {
            Self()
        }
    }

    pub(super) fn ms(&self) -> u64 {
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.0.elapsed().as_millis() as u64
        }
        #[cfg(target_arch = "wasm32")]
        {
            0
        }
    }
}

fn mesh_of(s: &Session, id: ObjectId) -> Option<std::sync::Arc<Mesh>> {
    s.doc.mesh(id).map(|m| m.mesh.clone())
}

/// Evidence from the cleaned scan that rebuilding it is possible and whether it is worth doing first:
/// every significant piece is closed (flaps aside); many handles or non-manifold edges is strong.
fn evidence(mesh: &Mesh) -> Option<(bool, bool, Value)> {
    let ps = pieces(mesh);
    let big: Vec<_> = ps.iter().filter(|p| p.triangles.len() * 100 >= mesh.triangle_count()).collect();
    let closed = !big.is_empty() && big.iter().all(|p| p.loops.iter().all(|l| l.edges <= MAX_FLAP_EDGES));
    let genus: usize = big.iter().filter_map(|p| p.genus()).sum();
    let non_manifold: usize = ps.iter().map(|p| p.non_manifold_edges).sum();
    let strong = genus >= HANDLE_TRIGGER || non_manifold >= NON_MANIFOLD_TRIGGER;
    let soft = genus >= SOFT_HANDLES || non_manifold >= SOFT_NON_MANIFOLD;
    closed.then(|| (strong, soft, json!({"genus": genus, "non_manifold_edges": non_manifold})))
}

/// What a remesh attempt came to: the new mesh and the step, or the step saying why it was not done.
type Rebuilt = Result<(ObjectId, PrepStep), PrepStep>;

/// Remeshes the scan. A cancellation is an error; any other refusal is the `Err` step.
fn remesh_scan(s: &mut Session, set: &Settings, reason: &str, figures: Value) -> CmdResult<Rebuilt> {
    check_cancel(s)?;
    let watch = Stopwatch::start();
    let Some(mesh) = mesh_of(s, set.scan) else { return Ok(Err(refused(reason, "no mesh", figures, 0))) };
    let voxel = (1.5 * set.tolerance).max((mesh.area() / REMESH_QUAD_BUDGET).sqrt());
    let input = RemeshIn {
        object: ObjRef::Id(set.scan.0),
        tolerance: Some(set.tolerance),
        voxel: (voxel > 1.5 * set.tolerance).then_some(voxel),
        min_feature: None,
        max_hole: None,
        min_component_fraction: None,
        name: Some(format!("{}-remesh", set.name)),
    };
    match remesh_cmd(s, input) {
        Ok(r) => {
            let numbers = json!({
                "voxel": r.voxel, "triangles": r.triangles, "genus_before": r.genus_before, "genus_after": r.genus_after,
                "non_manifold_before": r.non_manifold_before, "feature_removed": r.feature_removed,
                "max_deviation_to_scan": r.original_to_result.max, "p99_deviation_to_scan": r.original_to_result.p99,
                "removed_share": r.removed_share, "warnings": r.warnings, "evidence": figures, "ms": watch.ms(),
            });
            Ok(Ok((r.id, PrepStep { step: "remesh".into(), reason: reason.into(), numbers })))
        }
        Err(e) if is_cancelled(&e) => Err(e),
        Err(e) => Ok(Err(refused(reason, message_of(&e), figures, watch.ms()))),
    }
}

fn refused(reason: &str, why: &str, figures: Value, ms: u64) -> PrepStep {
    PrepStep {
        step: "remesh".into(),
        reason: format!("{reason}; not done: {why}"),
        numbers: json!({"refused": why, "evidence": figures, "ms": ms}),
    }
}

/// Can a rebuild of the scan improve this plain result? The rebuild repairs topology. A closed result
/// whose only issue is `out_of_tolerance` is limited by the patch budget (the cap was reached, or the
/// larger-cap retry already ran), not by topology: unless the scan shows some topology evidence
/// (`soft`), rebuilding it only costs time. An open or failed result, or one with other issues, may
/// be a victim of the topology.
fn rebuild_can_help(base: &ToSolidOut, soft: bool) -> bool {
    let closed_and_short = base.body.is_some() && base.issues.iter().all(|i| i.code == "out_of_tolerance");
    !closed_and_short || soft
}

/// What the rule of the retry sees of a result: whether the solid is closed and the share within the tolerance.
fn attempt(o: &ToSolidOut) -> Attempt {
    Attempt {
        closed: o.body.is_some() && !o.issues.iter().any(|i| i.code == "open_surface"),
        within: o.deviation.as_ref().map_or(0.0, |d| d.within),
    }
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

/// Keeps the better of the plain and the rebuilt result by the rule of the retry (the rebuilt one
/// must win, a tie keeps the plain one), removes the other and gives the winner the requested name.
fn choose(
    s: &mut Session,
    set: &Settings,
    (mut base, mut rebuilt): (ToSolidOut, ToSolidOut),
    step: PrepStep,
    id: ObjectId,
) -> ToSolidOut {
    let reason = judge(attempt(&base), attempt(&rebuilt));
    let out = if reason == RetryReason::Better {
        discard(s, &base, None);
        rebuilt.prep.preparation.push(step);
        rebuilt
    } else {
        discard(s, &rebuilt, Some(id));
        let note = PrepStep {
            step: step.step,
            reason: format!("{}; the rebuilt scan did not model better ({reason:?}) and was discarded", step.reason),
            numbers: step.numbers,
        };
        base.prep.preparation.push(note);
        base
    };
    restore_name(s, set, &out);
    out
}

/// Gives the solid of `out` the requested name when another attempt held it while it was made.
fn restore_name(s: &mut Session, set: &Settings, out: &ToSolidOut) {
    if let Some(b) = &out.body
        && s.doc.body(b.id).is_some_and(|o| o.name != set.name)
    {
        s.doc.rename(b.id, &set.name);
    }
}

/// The freeform method with its preparation: the scan is wrapped as it is; when that does not pass
/// and the scan is closed, it is rebuilt and wrapped again. A scan with strong evidence (many
/// handles or non-manifold edges) is rebuilt first. With `rebuild` off, only the plain wrap.
pub(super) fn freeform_prepared(s: &mut Session, set: &Settings, report: ScanSummary) -> CmdResult<ToSolidOut> {
    let found = if set.rebuild { mesh_of(s, set.scan).and_then(|m| evidence(&m)) } else { None };
    let Some((strong, soft, figures)) = found else { return freeform_on(s, set, set.scan, report) };
    // the larger-cap retry belongs to the first attempt only: a second attempt never makes a third wrap
    let second = Settings { allow_retry: false, name: set.name.clone(), budget: set.budget.clone(), ..*set };
    let mut refusal = None;
    if strong {
        let reason = "the scan has many handles or non-manifold edges that stall the reduction";
        match remesh_scan(s, set, reason, figures.clone())? {
            Ok((id, step)) => {
                check_cancel(s)?;
                let rebuilt = freeform_on(s, set, id, report.clone())?;
                if rebuilt.verdict == Verdict::Pass {
                    return Ok(finish_rebuilt(s, set, rebuilt, step));
                }
                // wrapping the scan as it is as well is an extra attempt of the call's budget
                if !set.budget.draw() {
                    return Ok(finish_rebuilt(s, set, rebuilt, step));
                }
                check_cancel(s)?;
                let base = freeform_on(s, &second, set.scan, report)?;
                return Ok(choose(s, set, (base, rebuilt), step, id));
            }
            Err(step) => refusal = Some(step),
        }
    }
    let mut base = freeform_on(s, set, set.scan, report.clone())?;
    if base.verdict == Verdict::Pass || strong || !rebuild_can_help(&base, soft) {
        base.prep.preparation.extend(refusal);
        return Ok(base);
    }
    if !set.budget.draw() {
        return Ok(base);
    }
    check_cancel(s)?;
    let reason = "wrapping the scan as it is did not pass; the scan is closed and is rebuilt as a clean surface";
    match remesh_scan(s, set, reason, figures)? {
        Ok((id, step)) => {
            check_cancel(s)?;
            let rebuilt = freeform_on(s, &second, id, report)?;
            Ok(choose(s, set, (base, rebuilt), step, id))
        }
        Err(step) => {
            base.prep.preparation.push(step);
            Ok(base)
        }
    }
}

/// The rebuilt result passed: it is the result, under the requested name.
fn finish_rebuilt(s: &mut Session, set: &Settings, mut rebuilt: ToSolidOut, step: PrepStep) -> ToSolidOut {
    rebuilt.prep.preparation.push(step);
    restore_name(s, set, &rebuilt);
    rebuilt
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cmd::pipeline::{Issue, Method};
    use crate::cmd::{ObjRef, body::BodyOut};
    use serde_json::json;

    fn box_body(s: &mut Session, name: &str) -> BodyOut {
        s.execute("body.box", json!({"size": [1.0, 1.0, 1.0], "name": name})).unwrap();
        let (id, _) = s.doc.find_by_name(name).unwrap();
        crate::cmd::body::body_out(s, id).unwrap()
    }

    fn result(body: BodyOut, within: f64, open: bool) -> ToSolidOut {
        let set = Settings {
            scan: ObjectId(0),
            tolerance: 0.2,
            min_within: 0.99,
            min_triangles: None,
            max_patches: None,
            allow_retry: true,
            budget: super::super::budget::CallBudget::default(),
            rebuild: true,
            name: "solid".into(),
        };
        let mut o = super::super::prepare::failure_for_tests(&set, body.id);
        o.body = Some(body);
        o.method = Method::Freeform;
        o.verdict = Verdict::Review;
        o.deviation = Some(super::super::DeviationSummary {
            count: 10,
            coverage: 1.0,
            not_facing: 0.0,
            within,
            rms: 0.0,
            max_abs: 0.0,
            mean: 0.0,
        });
        o.issues = if open { vec![Issue { code: "open_surface".into(), message: String::new() }] } else { Vec::new() };
        o
    }

    fn setting() -> Settings {
        Settings {
            scan: ObjectId(0),
            tolerance: 0.2,
            min_within: 0.99,
            min_triangles: None,
            max_patches: None,
            allow_retry: true,
            budget: super::super::budget::CallBudget::default(),
            rebuild: true,
            name: "solid".into(),
        }
    }

    fn step() -> PrepStep {
        PrepStep { step: "remesh".into(), reason: "test".into(), numbers: json!({}) }
    }

    #[test]
    fn a_tie_keeps_the_plain_result_and_its_name() {
        let mut s = Session::new();
        let plain = box_body(&mut s, "solid");
        let rebuilt = box_body(&mut s, "solid 2");
        let mesh =
            s.doc.add_mesh("rebuilt", surfacecraft_geom::synth::sphere(surfacecraft_geom::Point3::origin(), 1.0, 8, 4));
        let out = choose(
            &mut s,
            &setting(),
            (result(plain.clone(), 0.90, false), result(rebuilt.clone(), 0.904, false)),
            step(),
            mesh,
        );
        assert_eq!(out.body.unwrap().id, plain.id);
        assert!(s.doc.body(rebuilt.id).is_none() && s.doc.mesh(mesh).is_none(), "the loser and its mesh are gone");
        assert_eq!(s.doc.body(plain.id).unwrap().name, "solid");
        assert!(out.prep.preparation[0].reason.contains("discarded"));
    }

    #[test]
    fn a_clear_win_replaces_the_plain_result_and_takes_the_requested_name_but_an_open_one_never_wins() {
        let mut s = Session::new();
        let plain = box_body(&mut s, "solid");
        let rebuilt = box_body(&mut s, "solid 2");
        let mesh =
            s.doc.add_mesh("rebuilt", surfacecraft_geom::synth::sphere(surfacecraft_geom::Point3::origin(), 1.0, 8, 4));
        let out = choose(
            &mut s,
            &setting(),
            (result(plain.clone(), 0.80, false), result(rebuilt.clone(), 0.95, false)),
            step(),
            mesh,
        );
        assert_eq!(out.body.unwrap().id, rebuilt.id);
        assert!(s.doc.body(plain.id).is_none());
        assert_eq!(s.doc.body(rebuilt.id).unwrap().name, "solid");
        let mut s = Session::new();
        let (plain, rebuilt) = (box_body(&mut s, "solid"), box_body(&mut s, "solid 2"));
        let mesh =
            s.doc.add_mesh("rebuilt", surfacecraft_geom::synth::sphere(surfacecraft_geom::Point3::origin(), 1.0, 8, 4));
        let out =
            choose(&mut s, &setting(), (result(plain.clone(), 0.80, false), result(rebuilt, 0.99, true)), step(), mesh);
        assert_eq!(out.body.unwrap().id, plain.id);
        let _ = (ObjRef::Id(0), result as fn(BodyOut, f64, bool) -> ToSolidOut);
    }

    #[test]
    fn a_cancelled_remesh_is_a_cancelled_error_not_a_refused_step() {
        let mut s = Session::new();
        let id =
            s.doc.add_mesh("scan", surfacecraft_geom::synth::sphere(surfacecraft_geom::Point3::origin(), 5.0, 16, 8));
        let set = Settings { scan: id, ..setting() };
        s.cancel.cancel();
        let r = remesh_scan(&mut s, &set, "test", json!({}));
        assert!(matches!(r, Err(ref e) if is_cancelled(e)));
        let c = remesh_cmd(
            &mut s,
            RemeshIn {
                object: ObjRef::Id(id.0),
                tolerance: None,
                voxel: None,
                min_feature: None,
                max_hole: None,
                min_component_fraction: None,
                name: None,
            },
        );
        assert!(matches!(c, Err(ref e) if is_cancelled(e)), "the command itself checks too");
    }

    #[test]
    fn a_closed_result_short_only_of_the_tolerance_is_not_rebuilt_unless_the_topology_says_so() {
        let mut s = Session::new();
        let body = box_body(&mut s, "solid");
        let issue = |code: &str| Issue { code: code.into(), message: String::new() };
        let mut closed_short = result(body, 0.8, false);
        closed_short.issues = vec![issue("out_of_tolerance")];
        assert!(!rebuild_can_help(&closed_short, false), "limited by the patch budget");
        assert!(rebuild_can_help(&closed_short, true), "with topology evidence the rebuild may help");
        let mut open = closed_short.clone();
        open.issues = vec![issue("open_surface"), issue("out_of_tolerance")];
        assert!(rebuild_can_help(&open, false), "an open result may be the topology's");
        let mut other = closed_short.clone();
        other.issues = vec![issue("out_of_tolerance"), issue("non_manifold_edges")];
        assert!(rebuild_can_help(&other, false));
        let mut none = closed_short;
        none.body = None;
        assert!(rebuild_can_help(&none, false), "no solid at all");
    }
}

//! `body.refit_features` and `body.refit_faces`: update a CAD body so that it matches a new scan
//! of the changed part (as-built). The scan is already aligned to the body.
//!
//! * `body.refit_features` re-dimensions a body that has a feature history: sketch dimensions,
//!   extrude depths and blend sizes are optimised, within bounds, to minimise the scan-to-body
//!   distance (Huber least squares, finite-difference Jacobian over the replay).
//! * `body.refit_faces` edits a body without history: scan points are associated to faces
//!   (nearest face whose normal faces the point), and each analytic face is moved along its
//!   normal by a regularised amount; the kernel rebuilds the solid.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::Arc;
use surfacecraft_deviation::against_mesh;
use surfacecraft_doc::{Document, FeatureOp, ObjectId, ObjectKind};
use surfacecraft_fit::lm::{BoundedOptions, minimize_bounded, robust_residual};
use surfacecraft_geom::{Bvh, Point3, Vec3, par};
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_kernel::refit::{
    FaceMove, FaceShape, SAME_SURFACE_TOL, face_shapes, offset_face_groups, relation_counts,
};
use surfacecraft_sketch::{Constraint, Sketch, solve};

/// Scan points a refit measures with at most (more are thinned evenly).
const MAX_POINTS: usize = 200_000;

fn positive(v: Option<f64>, default: f64, what: &str) -> CmdResult<f64> {
    let v = v.unwrap_or(default);
    if v.is_finite() && v > 0.0 { Ok(v) } else { Err(CmdError::Params(format!("{what} must be a positive number"))) }
}

/// The scan's vertices and unit vertex normals (zero when unknown), thinned to `max_points`.
fn scan_points(s: &Session, mesh: &ObjRef, max_points: usize) -> CmdResult<(Vec<Point3>, Vec<Vec3>)> {
    let id = mesh.resolve(s, ObjectKind::Mesh)?;
    let m = s.doc.mesh(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let normals = m.mesh.vertex_normals();
    let n = m.mesh.positions.len();
    if n == 0 {
        return Err(CmdError::Params("the scan has no points".into()));
    }
    let stride = n.div_ceil(max_points.clamp(1, MAX_POINTS));
    let mut pts = Vec::new();
    let mut nrm = Vec::new();
    for i in (0..n).step_by(stride) {
        if let Some(p) = m.mesh.positions.get(i)
            && p.coords.iter().all(|c| c.is_finite())
        {
            pts.push(*p);
            nrm.push(normals.get(i).copied().unwrap_or_else(Vec3::zeros));
        }
    }
    if pts.is_empty() {
        return Err(CmdError::Params("the scan has no finite points".into()));
    }
    Ok((pts, nrm))
}

fn bbox_diagonal(b: &Brep) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for v in &b.vertices {
        for k in 0..3 {
            lo[k] = lo[k].min(v.point[k]);
            hi[k] = hi[k].max(v.point[k]);
        }
    }
    let d = (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt();
    if d.is_finite() { d } else { 1.0 }
}

fn rms(values: &[f64]) -> (f64, usize) {
    let v: Vec<f64> = values.iter().copied().filter(|x| x.is_finite()).collect();
    if v.is_empty() {
        return (f64::NAN, 0);
    }
    ((v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt(), v.len())
}

// ---------------------------------------------------------------------------------------------
// body.refit_features
// ---------------------------------------------------------------------------------------------

/// Parameters of `body.refit_features`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RefitFeaturesIn {
    /// Body with a feature history.
    pub body: ObjRef,
    /// The scan (a mesh, aligned to the body).
    pub mesh: ObjRef,
    /// Parameters to optimise: ids as listed in the result (`sketch/<id>/<constraint>`,
    /// `feature/<id>/<field>`), or `sketch/<id>` / `feature/<id>` for all dimensions of one
    /// object. Default: every dimension-like parameter (sketch lengths, distances and radii,
    /// extrude depths, fillet and chamfer sizes, shell thickness).
    #[serde(default)]
    pub params: Option<Vec<String>>,
    /// Largest change of a length parameter (mm, default 2).
    #[serde(default)]
    pub max_change: Option<f64>,
    /// Largest change of an angle parameter (degrees, default 5).
    #[serde(default)]
    pub max_angle_change: Option<f64>,
    /// Iterations at most (default 20, up to 100).
    #[serde(default)]
    pub max_iterations: Option<usize>,
    /// Scan points measured at most (default 20000).
    #[serde(default)]
    pub max_points: Option<usize>,
    /// Scan points farther than this from the body are ignored (mm, default `3 × max_change`).
    #[serde(default)]
    pub max_distance: Option<f64>,
    /// Residuals beyond this are down-weighted (Huber, mm; default a quarter of `max_change`).
    #[serde(default)]
    pub robust_scale: Option<f64>,
    /// Apply the result to the document (default true); false only reports it.
    #[serde(default)]
    pub apply: Option<bool>,
}

/// One refitted parameter.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ParamOut {
    /// Parameter id (usable in `params`).
    pub id: String,
    /// What it is.
    pub label: String,
    /// Value before.
    pub old: f64,
    /// Value after.
    pub new: f64,
    /// `new − old`.
    pub change: f64,
    /// Lower bound used.
    pub lower: f64,
    /// Upper bound used.
    pub upper: f64,
    /// The optimiser stopped against a bound.
    pub at_bound: bool,
    /// How strongly the scan-to-body distance responds to the parameter (RMS mm per unit).
    pub sensitivity: f64,
    /// A remark (e.g. the scan does not constrain it).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

/// Result of `body.refit_features`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct RefitFeaturesOut {
    /// The body.
    pub body: ObjectId,
    /// The parameters, old to new.
    pub params: Vec<ParamOut>,
    /// RMS scan-to-body distance before (mm).
    pub rms_before: f64,
    /// RMS scan-to-body distance after (mm).
    pub rms_after: f64,
    /// Scan points inside `max_distance` before and after (the RMS counts these).
    pub points_before: usize,
    /// Scan points inside `max_distance` after.
    pub points_after: usize,
    /// Optimiser iterations.
    pub iterations: usize,
    /// Replays of the history made.
    pub evaluations: usize,
    /// The optimiser met its tolerance (false: it ran out of iterations).
    pub converged: bool,
    /// Was the result applied?
    pub applied: bool,
    /// Features that failed in the final replay (0 when applied cleanly).
    pub failed_features: usize,
}

#[derive(Clone, Debug)]
enum Owner {
    Constraint { sketch: ObjectId, index: usize },
    Field { feature: ObjectId, name: &'static str },
}

#[derive(Clone, Debug)]
struct Param {
    id: String,
    label: String,
    owner: Owner,
    old: f64,
    lo: f64,
    hi: f64,
    step: f64,
    default: bool,
}

/// The features that make `body` (and the bodies they read) and the sketches they read.
fn scope(doc: &Document, body: ObjectId) -> (Vec<ObjectId>, Vec<ObjectId>) {
    let mut bodies = vec![body];
    let mut features: Vec<ObjectId> = Vec::new();
    let mut sketches: Vec<ObjectId> = Vec::new();
    loop {
        let mut changed = false;
        for f in &doc.features {
            if bodies.contains(&f.body) && !features.contains(&f.id) {
                features.push(f.id);
                changed = true;
                for input in f.op.inputs() {
                    match doc.kind_of(input) {
                        Some(ObjectKind::Body) if !bodies.contains(&input) => bodies.push(input),
                        Some(ObjectKind::Sketch) if !sketches.contains(&input) => sketches.push(input),
                        _ => {}
                    }
                }
            }
        }
        if !changed || features.len() >= doc.features.len() {
            break;
        }
    }
    // history order
    let ordered = doc.features.iter().map(|f| f.id).filter(|id| features.contains(id)).collect();
    (ordered, sketches)
}

/// The numeric fields of a feature: `(name, dimension-like by default, value)`.
fn feature_fields(op: &FeatureOp) -> Vec<(&'static str, bool, f64)> {
    match op {
        FeatureOp::Extrude { distance, draft_deg, .. } => {
            vec![("distance", true, *distance), ("draft_deg", false, *draft_deg)]
        }
        FeatureOp::Revolve { angle_deg, .. } => vec![("angle_deg", false, *angle_deg)],
        FeatureOp::Fillet { radius, .. } => vec![("radius", true, *radius)],
        FeatureOp::Chamfer { distance, .. } => vec![("distance", true, *distance)],
        FeatureOp::Shell { thickness, .. } => vec![("thickness", true, *thickness)],
        FeatureOp::Draft { angle_deg, .. } => vec![("angle_deg", false, *angle_deg)],
        _ => Vec::new(),
    }
}

fn set_field(op: &mut FeatureOp, name: &str, v: f64) -> bool {
    match (op, name) {
        (FeatureOp::Extrude { distance, .. }, "distance") => *distance = v,
        (FeatureOp::Extrude { draft_deg, .. }, "draft_deg") => *draft_deg = v,
        (FeatureOp::Revolve { angle_deg, .. }, "angle_deg") | (FeatureOp::Draft { angle_deg, .. }, "angle_deg") => {
            *angle_deg = v;
        }
        (FeatureOp::Fillet { radius, .. }, "radius") => *radius = v,
        (FeatureOp::Chamfer { distance, .. }, "distance") => *distance = v,
        (FeatureOp::Shell { thickness, .. }, "thickness") => *thickness = v,
        _ => return false,
    }
    true
}

/// A sketch dimension: `(kind, dimension-like by default, value)`.
fn constraint_dim(c: &Constraint) -> Option<(&'static str, bool, f64)> {
    match c {
        Constraint::Length { value, .. } => Some(("length", true, *value)),
        Constraint::Distance { value, .. } => Some(("distance", true, *value)),
        Constraint::Radius { value, .. } => Some(("radius", true, *value)),
        Constraint::Angle { degrees, .. } => Some(("angle", false, *degrees)),
        _ => None,
    }
}

fn set_constraint(c: &mut Constraint, v: f64) -> bool {
    match c {
        Constraint::Length { value, .. } | Constraint::Distance { value, .. } | Constraint::Radius { value, .. } => {
            *value = v;
        }
        Constraint::Angle { degrees, .. } => *degrees = v,
        _ => return false,
    }
    true
}

fn bounds(old: f64, angle: bool, mc: f64, mca: f64) -> (f64, f64) {
    if angle {
        return (old - mca, old + mca);
    }
    // a length keeps its sign and never shrinks below a twentieth of itself
    let floor = (0.05 * old.abs()).max(1e-3);
    if old > 0.0 {
        ((old - mc).max(floor), old + mc)
    } else if old < 0.0 {
        (old - mc, (old + mc).min(-floor))
    } else {
        (-mc, mc)
    }
}

fn discover(doc: &Document, features: &[ObjectId], mc: f64, mca: f64) -> Vec<Param> {
    let mut out = Vec::new();
    let mut seen: Vec<ObjectId> = Vec::new();
    for fid in features {
        let Some(f) = doc.feature(*fid) else { continue };
        for input in f.op.inputs() {
            if doc.kind_of(input) == Some(ObjectKind::Sketch)
                && !seen.contains(&input)
                && let Some(sk) = doc.sketch(input)
            {
                seen.push(input);
                for (i, c) in sk.sketch.constraints.iter().enumerate() {
                    if let Some((kind, default, v)) = constraint_dim(c)
                        && v.is_finite()
                    {
                        let angle = kind == "angle";
                        let (lo, hi) = bounds(v, angle, mc, mca);
                        out.push(Param {
                            id: format!("sketch/{}/{i}", input.0),
                            label: format!("sketch '{}': {kind} (constraint {i})", sk.name),
                            owner: Owner::Constraint { sketch: input, index: i },
                            old: v,
                            lo,
                            hi,
                            step: if angle { 0.05 } else { 0.02 },
                            default,
                        });
                    }
                }
            }
        }
        for (name, default, v) in feature_fields(&f.op) {
            if !v.is_finite() {
                continue;
            }
            let angle = name.ends_with("_deg");
            let (lo, hi) = bounds(v, angle, mc, mca);
            out.push(Param {
                id: format!("feature/{}/{name}", fid.0),
                label: format!("feature {} '{}' ({}): {name}", fid.0, f.name, f.op.label()),
                owner: Owner::Field { feature: *fid, name },
                old: v,
                lo,
                hi,
                step: if angle { 0.05 } else { 0.02 },
                default,
            });
        }
    }
    out
}

fn select(all: Vec<Param>, wanted: Option<&[String]>) -> CmdResult<Vec<Param>> {
    let Some(list) = wanted else {
        return Ok(all.into_iter().filter(|p| p.default).collect());
    };
    if list.is_empty() {
        return Err(CmdError::Params("`params` is empty: leave it out to refit every dimension".into()));
    }
    let mut picked: Vec<Param> = Vec::new();
    for w in list {
        let w = w.trim();
        let exact: Vec<&Param> = all.iter().filter(|p| p.id == w).collect();
        let by_owner: Vec<&Param> = if exact.is_empty() {
            let prefix = format!("{}/", w.trim_end_matches('/'));
            all.iter().filter(|p| p.default && p.id.starts_with(&prefix)).collect()
        } else {
            exact
        };
        if by_owner.is_empty() {
            let known: Vec<&str> = all.iter().take(24).map(|p| p.id.as_str()).collect();
            return Err(CmdError::Params(format!("no parameter `{w}`; available: {}", known.join(", "))));
        }
        for p in by_owner {
            if !picked.iter().any(|q| q.id == p.id) {
                picked.push(p.clone());
            }
        }
    }
    Ok(picked)
}

/// Writes parameter `values` into `doc` (from the pristine `sketches` and `ops`) and re-solves
/// the sketches they touch.
fn write_values(
    doc: &mut Document,
    params: &[Param],
    sketches: &[(ObjectId, Sketch)],
    ops: &[(ObjectId, FeatureOp)],
    values: &[f64],
    keep_report: bool,
) -> Result<(), String> {
    for (id, sk) in sketches {
        if let Some(o) = doc.sketch_mut(*id) {
            o.sketch = sk.clone();
        }
    }
    for (id, op) in ops {
        if let Some(f) = doc.feature_mut(*id) {
            f.op = op.clone();
        }
    }
    let mut touched: Vec<ObjectId> = Vec::new();
    for (p, v) in params.iter().zip(values) {
        match &p.owner {
            Owner::Constraint { sketch, index } => {
                let o = doc.sketch_mut(*sketch).ok_or_else(|| format!("{}: the constraint is gone", p.id))?;
                let c =
                    o.sketch.constraints.get_mut(*index).ok_or_else(|| format!("{}: the constraint is gone", p.id))?;
                if !set_constraint(c, *v) {
                    return Err(format!("{}: not a dimension", p.id));
                }
                // the fitted value is a plain number: it no longer follows an expression
                o.sketch.dim_exprs.remove(index);
                if !touched.contains(sketch) {
                    touched.push(*sketch);
                }
            }
            Owner::Field { feature, name } => {
                let f = doc.feature_mut(*feature).ok_or_else(|| format!("{}: the feature is gone", p.id))?;
                if !set_field(&mut f.op, name, *v) {
                    return Err(format!("{}: not a parameter", p.id));
                }
                f.exprs.remove(*name);
            }
        }
    }
    for id in touched {
        let o = doc.sketch_mut(id).ok_or("a sketch is gone")?;
        let mut sk = o.sketch.clone();
        let rep = solve(&mut sk).map_err(|e| e.to_string())?;
        if !(rep.residual < 1e-6) {
            return Err(format!("sketch {id} cannot satisfy these dimensions ({})", rep.message));
        }
        o.sketch = sk;
        if keep_report {
            o.report = Some(rep);
        }
    }
    Ok(())
}

/// Replays trial parameters on a scratch document and measures the scan against the body.
struct Evaluator {
    scratch: Document,
    params: Vec<Param>,
    sketches: Vec<(ObjectId, Sketch)>,
    ops: Vec<(ObjectId, FeatureOp)>,
    features: Vec<ObjectId>,
    target: ObjectId,
    points: Vec<Point3>,
    chord: f64,
    max_distance: f64,
}

impl Evaluator {
    fn body(&mut self, values: &[f64]) -> Result<Arc<Brep>, String> {
        write_values(&mut self.scratch, &self.params, &self.sketches, &self.ops, values, false)?;
        crate::replay::replay_doc(&mut self.scratch);
        for fid in &self.features {
            if let Some(e) = self.scratch.feature(*fid).and_then(|f| f.error.as_ref()) {
                return Err(e.clone());
            }
        }
        self.scratch.body(self.target).map(|b| b.brep.clone()).ok_or_else(|| "the body is gone".to_string())
    }

    /// Signed distance of every scan point from the body (NaN beyond `max_distance`); `None`
    /// when the model cannot be built or meshed at `values`.
    fn distances(&mut self, values: &[f64]) -> Option<Vec<f64>> {
        let brep = self.body(values).ok()?;
        let mesh = surfacecraft_kernel::tess::tessellate(&brep, Some(self.chord)).ok()?;
        if mesh.triangle_count() == 0 {
            return None;
        }
        Some(against_mesh(&self.points, &Bvh::build(&mesh), self.max_distance))
    }
}

fn refit_features(s: &mut Session, i: RefitFeaturesIn) -> CmdResult<RefitFeaturesOut> {
    let body = i.body.resolve(s, ObjectKind::Body)?;
    let mc = positive(i.max_change, 2.0, "max_change")?;
    let mca = positive(i.max_angle_change, 5.0, "max_angle_change")?;
    let max_distance = positive(i.max_distance, 3.0 * mc, "max_distance")?;
    let robust = positive(i.robust_scale, (mc / 4.0).max(0.05), "robust_scale")?;
    let max_iter = i.max_iterations.unwrap_or(20).clamp(1, 100);
    let (points, _) = scan_points(s, &i.mesh, i.max_points.unwrap_or(20_000))?;
    let (features, sketch_ids) = scope(&s.doc, body);
    let has_history =
        features.iter().any(|f| s.doc.feature(*f).is_some_and(|f| !matches!(f.op, FeatureOp::Base { .. })));
    if !has_history {
        return Err(CmdError::Failed(
            "this body has no feature history to refit; use body.refit_faces to move its faces".into(),
        ));
    }
    let all = discover(&s.doc, &features, mc, mca);
    let params = select(all, i.params.as_deref())?;
    if params.is_empty() {
        return Err(CmdError::Failed("the history has no numeric parameters to refit".into()));
    }
    if params.len() > 64 {
        return Err(CmdError::Params(format!("{} parameters: name at most 64 in `params`", params.len())));
    }
    let b0 = s.doc.body(body).ok_or_else(|| CmdError::Params(format!("no body {body}")))?;
    let chord = (bbox_diagonal(&b0.brep) * 1e-4).clamp(0.002, 0.02);
    let sketches: Vec<(ObjectId, Sketch)> =
        sketch_ids.iter().filter_map(|id| s.doc.sketch(*id).map(|o| (*id, o.sketch.clone()))).collect();
    let ops: Vec<(ObjectId, FeatureOp)> =
        features.iter().filter_map(|id| s.doc.feature(*id).map(|f| (*id, f.op.clone()))).collect();
    let mut ev = Evaluator {
        scratch: s.doc.clone(),
        params: params.clone(),
        sketches: sketches.clone(),
        ops: ops.clone(),
        features: features.clone(),
        target: body,
        points,
        chord,
        max_distance,
    };
    let p0: Vec<f64> = params.iter().map(|p| p.old).collect();
    let d0 = ev
        .distances(&p0)
        .ok_or_else(|| CmdError::Failed("the history does not replay into a body before the refit".into()))?;
    let (rms_before, points_before) = rms(&d0);
    if points_before == 0 {
        return Err(CmdError::Failed(
            "no scan point lies within `max_distance` of the body: is the scan aligned to it?".into(),
        ));
    }
    let m = ev.points.len();
    let opts = BoundedOptions {
        lower: params.iter().map(|p| p.lo).collect(),
        upper: params.iter().map(|p| p.hi).collect(),
        step: params.iter().map(|p| p.step).collect(),
        max_iter,
        max_trials: 10,
        central: false,
        tol_step: 1e-5,
        tol_cost: 1e-7,
    };
    let result = minimize_bounded(&p0, m, &opts, |p, out| match ev.distances(p) {
        Some(d) => {
            for (o, v) in out.iter_mut().zip(&d) {
                *o = if v.is_finite() { robust_residual(*v, robust) } else { 0.0 };
            }
        }
        None => out.fill(f64::NAN),
    })
    .ok_or_else(|| CmdError::Failed("the optimiser could not start".into()))?;
    let mut values = result.params.clone();
    let d1 = ev.distances(&values).ok_or_else(|| CmdError::Failed("the refitted history does not replay".into()))?;
    let (mut rms_after, mut points_after) = rms(&d1);
    // keep the better of the two (the start is always a candidate)
    if rms_after > rms_before {
        values = p0.clone();
        rms_after = rms_before;
        points_after = points_before;
    }
    // a parameter whose change the scan cannot see (a cut deeper than the part is thick) is put
    // back: it only moved on the noise of the finite differences
    let mut reverted: Vec<bool> = vec![false; params.len()];
    for k in 0..params.len() {
        let (Some(v), Some(o)) = (values.get(k).copied(), p0.get(k).copied()) else { continue };
        if (v - o).abs() < 1e-12 {
            continue;
        }
        let mut trial = values.clone();
        if let Some(t) = trial.get_mut(k) {
            *t = o;
        }
        if let Some(d) = ev.distances(&trial) {
            let (r, n) = rms(&d);
            if r - rms_after < 2e-4 {
                values = trial;
                rms_after = r;
                points_after = n;
                if let Some(x) = reverted.get_mut(k) {
                    *x = true;
                }
            }
        }
    }
    let outs: Vec<ParamOut> = params
        .iter()
        .zip(&values)
        .zip(&result.sensitivity)
        .zip(&reverted)
        .map(|(((p, v), sens), rev)| {
            let at_bound = (*v - p.lo).abs() < 1e-9 || (*v - p.hi).abs() < 1e-9;
            let note = if *rev {
                Some("the scan shows no change worth making here (below its resolution); left as it was".to_string())
            } else if *sens < 1e-6 {
                Some("the scan does not constrain this parameter; left as it was".to_string())
            } else if at_bound {
                Some("stopped at its bound: raise max_change if the part changed more".to_string())
            } else {
                None
            };
            ParamOut {
                id: p.id.clone(),
                label: p.label.clone(),
                old: p.old,
                new: *v,
                change: v - p.old,
                lower: p.lo,
                upper: p.hi,
                at_bound,
                sensitivity: *sens,
                note,
            }
        })
        .collect();
    let apply = i.apply.unwrap_or(true);
    let mut failed = 0;
    if apply {
        write_values(&mut s.doc, &params, &sketches, &ops, &values, true).map_err(CmdError::Failed)?;
        failed = crate::replay::replay(s);
        for fid in &features {
            if let Some(e) = s.doc.feature(*fid).and_then(|f| f.error.clone()) {
                return Err(CmdError::Failed(format!("the refitted history fails: {e}")));
            }
        }
    }
    Ok(RefitFeaturesOut {
        body,
        params: outs,
        rms_before,
        rms_after,
        points_before,
        points_after,
        iterations: result.iterations,
        evaluations: result.evaluations,
        converged: result.converged,
        applied: apply,
        failed_features: failed,
    })
}

// ---------------------------------------------------------------------------------------------
// body.refit_faces
// ---------------------------------------------------------------------------------------------

/// How firmly a face is held to its original place.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Stiffness {
    /// Never moves.
    Fixed,
    /// The original counts twice as much as the scan evidence: a third of the measured offset.
    Tight,
    /// The original counts a twentieth as much as the scan: almost all of the measured offset.
    Loose,
    /// Follows the scan.
    Free,
}

impl Stiffness {
    /// Weight of the original position against the scan evidence of the face.
    fn prior(self) -> f64 {
        match self {
            Stiffness::Fixed => f64::INFINITY,
            Stiffness::Tight => 2.0,
            Stiffness::Loose => 0.05,
            Stiffness::Free => 0.0,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Stiffness::Fixed => "fixed",
            Stiffness::Tight => "tight",
            Stiffness::Loose => "loose",
            Stiffness::Free => "free",
        }
    }
}

/// Stiffness of one face.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FaceStiffness {
    /// Face index (record order of the body).
    pub face: usize,
    /// Its stiffness.
    pub stiffness: Stiffness,
}

/// Parameters of `body.refit_faces`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RefitFacesIn {
    /// Body without feature history (e.g. imported from STEP).
    pub body: ObjRef,
    /// The scan (a mesh, aligned to the body).
    pub mesh: ObjRef,
    /// Stiffness of every face not listed in `faces` (default `loose`).
    #[serde(default)]
    pub stiffness: Option<Stiffness>,
    /// Per-face stiffness.
    #[serde(default)]
    pub faces: Vec<FaceStiffness>,
    /// Largest move of a face (mm, default 1).
    #[serde(default)]
    pub max_change: Option<f64>,
    /// Scan points farther than this from every face are ignored (mm, default `max_change`).
    #[serde(default)]
    pub max_distance: Option<f64>,
    /// A scan point belongs to a face only when its normal is within this angle of the face's
    /// (degrees, default 60).
    #[serde(default)]
    pub max_angle: Option<f64>,
    /// Faces with fewer scan points are not moved (default 8).
    #[serde(default)]
    pub min_points: Option<usize>,
    /// Moves smaller than this are not made (mm, default 0.002).
    #[serde(default)]
    pub min_move: Option<f64>,
    /// Scan points measured at most (default 50000).
    #[serde(default)]
    pub max_points: Option<usize>,
    /// Keep equal radii equal: cylinders, spheres and torus tubes of one radius change together
    /// (default true).
    #[serde(default)]
    pub keep_relations: Option<bool>,
    /// Apply the result to the document (default true); false only reports it.
    #[serde(default)]
    pub apply: Option<bool>,
}

/// One face in the result of `body.refit_faces`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FaceOut {
    /// Face index in the original body.
    pub face: usize,
    /// `plane`, `cylinder`, `cone`, `sphere`, `torus` or `freeform`.
    pub kind: String,
    /// The stiffness it was given.
    pub stiffness: String,
    /// `moved`, `unchanged`, `fixed`, `no_data`, `refused` or `failed`.
    pub status: String,
    /// Scan points associated to the face.
    pub points: usize,
    /// Applied move along the outward normal (mm; for a hole a positive move shrinks it).
    pub offset: f64,
    /// RMS distance of its scan points from the face before (mm).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rms_before: Option<f64>,
    /// RMS distance of the same points from the rebuilt face (mm).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rms_after: Option<f64>,
    /// Surface parameters before.
    pub old: BTreeMap<String, f64>,
    /// Surface parameters after (read back from the rebuilt body).
    pub new: BTreeMap<String, f64>,
    /// Why it did not move, or what went wrong.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

/// Relation counts of a body.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct RelationsOut {
    /// Pairs of parallel normals or axes.
    pub parallel: usize,
    /// Pairs of perpendicular normals or axes.
    pub perpendicular: usize,
    /// Pairs of faces on one axis line.
    pub coaxial: usize,
    /// Pairs of equal radius.
    pub equal_radius: usize,
}

/// Result of `body.refit_faces`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct RefitFacesOut {
    /// The body.
    pub body: ObjectId,
    /// Per face of the original body.
    pub faces: Vec<FaceOut>,
    /// RMS distance of the associated scan points before (mm).
    pub rms_before: f64,
    /// RMS distance of the same points from the rebuilt faces (mm).
    pub rms_after: f64,
    /// Scan points associated to an analytic face.
    pub points: usize,
    /// Scan points that belong to no face (too far or facing away) or to a refused face.
    pub unassigned: usize,
    /// Faces moved.
    pub moved: usize,
    /// Faces of the body before.
    pub faces_before: usize,
    /// Faces of the body after.
    pub faces_after: usize,
    /// Relations among the faces before.
    pub relations_before: RelationsOut,
    /// Relations among the faces after (equal to `relations_before` when they are all kept).
    pub relations_after: RelationsOut,
    /// Was the result applied?
    pub applied: bool,
}

fn relations_of(shapes: &[Result<FaceShape, String>]) -> RelationsOut {
    let list: Vec<Option<&FaceShape>> = shapes.iter().map(|s| s.as_ref().ok()).collect();
    let r = relation_counts(&list);
    RelationsOut {
        parallel: r.parallel,
        perpendicular: r.perpendicular,
        coaxial: r.coaxial,
        equal_radius: r.equal_radius,
    }
}

fn shape_params(s: &FaceShape) -> BTreeMap<String, f64> {
    s.params().into_iter().map(|(k, v)| (k.to_string(), v)).collect()
}

struct UnionFind(Vec<usize>);

impl UnionFind {
    fn new(n: usize) -> Self {
        UnionFind((0..n).collect())
    }
    fn find(&mut self, mut x: usize) -> usize {
        while let Some(&p) = self.0.get(x) {
            if p == x {
                break;
            }
            let gp = self.0.get(p).copied().unwrap_or(p);
            if let Some(slot) = self.0.get_mut(x) {
                *slot = gp;
            }
            x = gp;
        }
        x
    }
    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb
            && let Some(slot) = self.0.get_mut(ra)
        {
            *slot = rb;
        }
    }
}

fn median(v: &mut [f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(|a, b| a.total_cmp(b));
    let n = v.len();
    let hi = v.get(n / 2).copied().unwrap_or(0.0);
    if n % 2 == 1 { hi } else { (hi + v.get(n / 2 - 1).copied().unwrap_or(hi)) / 2.0 }
}

/// One group of faces that move together: on the same surface or tied by an equal radius. Member
/// `k` moves by `signs[k]·θ`.
struct Group {
    faces: Vec<usize>,
}

/// The shared move θ of a group: a Huber-weighted mean of the members' signed offsets, held
/// towards zero by the members' priors; zero when a member is fixed.
fn solve_group(members: &[(f64, &[f64], f64)], max_change: f64) -> f64 {
    // member = (sign, offsets d of its scan points, prior weight ω)
    let mut prior_total = 0.0;
    let mut signed: Vec<f64> = Vec::new();
    for (sign, d, omega) in members {
        if !omega.is_finite() {
            return 0.0;
        }
        prior_total += omega * d.len() as f64;
        signed.extend(d.iter().map(|x| sign * x));
    }
    if signed.is_empty() {
        return 0.0;
    }
    let mut theta = median(&mut signed.clone());
    for _ in 0..12 {
        let mut resid: Vec<f64> = signed.iter().map(|x| (x - theta).abs()).collect();
        let mad = median(&mut resid);
        let k = (1.4826 * 1.5 * mad).max(0.01);
        let (mut num, mut den) = (0.0, 0.0);
        for x in &signed {
            let r = (x - theta).abs();
            let w = if r <= k { 1.0 } else { k / r };
            num += w * x;
            den += w;
        }
        let next = num / (den + prior_total);
        let done = (next - theta).abs() < 1e-9;
        theta = next;
        if done {
            break;
        }
    }
    theta.clamp(-max_change, max_change)
}

fn refit_faces(s: &mut Session, i: RefitFacesIn) -> CmdResult<RefitFacesOut> {
    let body = i.body.resolve(s, ObjectKind::Body)?;
    if s.doc.features.iter().any(|f| f.body == body) {
        return Err(CmdError::Failed(
            "this body is made by a feature history (a replay would undo the edit): use body.refit_features".into(),
        ));
    }
    let mc = positive(i.max_change, 1.0, "max_change")?;
    let max_distance = positive(i.max_distance, mc, "max_distance")?;
    let max_angle = positive(i.max_angle, 60.0, "max_angle")?;
    if max_angle >= 180.0 {
        return Err(CmdError::Params("max_angle must be below 180 degrees".into()));
    }
    let min_move = positive(i.min_move, 0.002, "min_move")?;
    let min_points = i.min_points.unwrap_or(8).max(1);
    let keep_relations = i.keep_relations.unwrap_or(true);
    let default_stiffness = i.stiffness.unwrap_or(Stiffness::Loose);
    let (points, normals) = scan_points(s, &i.mesh, i.max_points.unwrap_or(50_000))?;
    let brep = s.doc.body(body).map(|b| b.brep.clone()).ok_or_else(|| CmdError::Params(format!("no body {body}")))?;
    let closed_shells =
        brep.shells.iter().filter(|sh| matches!(sh, surfacecraft_kernel::record::brep::Shell::Faces { .. })).count();
    if closed_shells != 1 {
        return Err(CmdError::Failed("only a body of one shell is refitted (not several solids or cavities)".into()));
    }
    let shapes = face_shapes(&brep)?;
    let nf = shapes.len();
    // stiffness per face
    let mut stiff = vec![default_stiffness; nf];
    let mut explicit = vec![false; nf];
    for fs in &i.faces {
        let Some(slot) = stiff.get_mut(fs.face) else {
            return Err(CmdError::Params(format!("face {} does not exist ({nf} faces)", fs.face)));
        };
        if explicit.get(fs.face).copied().unwrap_or(false) {
            return Err(CmdError::Params(format!("face {} is listed twice", fs.face)));
        }
        if let Some(Err(reason)) = shapes.get(fs.face)
            && fs.stiffness != Stiffness::Fixed
        {
            return Err(CmdError::Params(format!("face {} cannot be refitted: {reason}", fs.face)));
        }
        *slot = fs.stiffness;
        if let Some(e) = explicit.get_mut(fs.face) {
            *e = true;
        }
    }
    // faces with their triangles
    let chord = (bbox_diagonal(&brep) * 2e-4).clamp(0.005, 0.05);
    let topo = surfacecraft_kernel::tess::tessellate_topo(&brep, Some(chord))?;
    if topo.tri_face.is_empty() || topo.mesh.triangle_count() == 0 {
        return Err(CmdError::Failed("the body could not be meshed to associate the scan to its faces".into()));
    }
    let mut tris: Vec<Vec<u32>> = vec![Vec::new(); nf];
    for (t, f) in topo.tri_face.iter().enumerate() {
        if let Some(list) = tris.get_mut(*f as usize) {
            list.push(t as u32);
        }
    }
    let bvhs: Vec<Option<Bvh>> =
        tris.iter().map(|t| (!t.is_empty()).then(|| Bvh::build(&topo.mesh.submesh(t)))).collect();
    // association: the nearest face (within max_distance) whose normal faces the scan point's
    let cos_min = max_angle.to_radians().cos();
    let assigned: Vec<Option<(usize, f64)>> = par::map(points.len(), |k| {
        let (Some(p), Some(n)) = (points.get(k), normals.get(k)) else { return None };
        let has_normal = n.norm() > 0.5;
        let mut best: Option<(usize, f64)> = None;
        let mut reach = max_distance;
        for (f, bvh) in bvhs.iter().enumerate() {
            let Some(bvh) = bvh else { continue };
            let Some(hit) = bvh.closest(p, reach) else { continue };
            let facing = match shapes.get(f) {
                Some(Ok(shape)) => {
                    let fnm = shape.outward_normal([hit.point.x, hit.point.y, hit.point.z]);
                    !has_normal || n.dot(&Vec3::from(fnm)) >= cos_min
                }
                // a refused face: only its triangle's direction, whichever way the mesh winds
                _ => !has_normal || bvh.triangle_normal(hit.triangle).is_none_or(|t| n.dot(&t).abs() >= cos_min),
            };
            if facing {
                reach = hit.distance;
                best = Some((f, hit.distance));
            }
        }
        best
    });
    // signed offsets of the scan points from the analytic face they belong to
    let mut offsets: Vec<Vec<f64>> = vec![Vec::new(); nf];
    let mut unassigned = 0;
    for (p, a) in points.iter().zip(&assigned) {
        match a {
            Some((f, _)) => match shapes.get(*f) {
                Some(Ok(shape)) => {
                    let d = shape.signed_distance([p.x, p.y, p.z]);
                    if d.is_finite()
                        && let Some(list) = offsets.get_mut(*f)
                    {
                        list.push(d);
                    }
                }
                _ => unassigned += 1,
            },
            None => unassigned += 1,
        }
    }
    // groups of faces that must move together
    let mut uf = UnionFind::new(nf);
    for a in 0..nf {
        let Some(Ok(sa)) = shapes.get(a) else { continue };
        for b in (a + 1)..nf {
            let Some(Ok(sb)) = shapes.get(b) else { continue };
            let same = sa.same_surface(sb, SAME_SURFACE_TOL);
            let equal = keep_relations
                && matches!((sa.radius(), sb.radius()), (Some(ra), Some(rb)) if (ra - rb).abs() < 1e-5 * (1.0 + ra.abs()));
            if same || equal {
                uf.union(a, b);
            }
        }
    }
    let mut groups: Vec<Group> = Vec::new();
    let mut slot_of: BTreeMap<usize, usize> = BTreeMap::new();
    for f in 0..nf {
        if !matches!(shapes.get(f), Some(Ok(_))) {
            continue;
        }
        let root = uf.find(f);
        let g = *slot_of.entry(root).or_insert_with(|| {
            groups.push(Group { faces: Vec::new() });
            groups.len() - 1
        });
        if let Some(group) = groups.get_mut(g) {
            group.faces.push(f);
        }
    }
    // θ per group, then the move of every face: δ = sign·θ
    let mut delta = vec![0.0; nf];
    let mut reason: Vec<Option<String>> = vec![None; nf];
    for g in &groups {
        let members: Vec<(f64, &[f64], f64)> = g
            .faces
            .iter()
            .map(|&f| {
                let sign = shapes.get(f).and_then(|s| s.as_ref().ok()).map_or(1.0, FaceShape::radius_sign);
                let d: &[f64] = offsets.get(f).map_or(&[], Vec::as_slice);
                (sign, d, stiff.get(f).copied().unwrap_or(default_stiffness).prior())
            })
            .collect();
        let total: usize = members.iter().map(|m| m.1.len()).sum();
        let fixed_member = g.faces.iter().copied().find(|&f| stiff.get(f) == Some(&Stiffness::Fixed));
        let theta = solve_group(&members, mc);
        for &f in &g.faces {
            let sign = shapes.get(f).and_then(|s| s.as_ref().ok()).map_or(1.0, FaceShape::radius_sign);
            let own = offsets.get(f).map_or(0, Vec::len);
            if let Some(fx) = fixed_member
                && stiff.get(f) != Some(&Stiffness::Fixed)
            {
                if let Some(r) = reason.get_mut(f) {
                    *r = Some(format!("tied to the fixed face {fx} (same surface or equal radius)"));
                }
                continue;
            }
            if total < min_points {
                if let Some(r) = reason.get_mut(f) {
                    *r = Some(format!("too few scan points ({total}, at least {min_points})"));
                }
                continue;
            }
            if own < min_points && g.faces.len() == 1 {
                if let Some(r) = reason.get_mut(f) {
                    *r = Some(format!("too few scan points ({own}, at least {min_points})"));
                }
                continue;
            }
            if let Some(d) = delta.get_mut(f) {
                *d = sign * theta;
            }
        }
    }
    // kernel moves: one per surface (twins together); curved faces before planes
    let mut by_surface: Vec<(Vec<usize>, f64)> = Vec::new();
    let mut taken = vec![false; nf];
    for f in 0..nf {
        if taken.get(f).copied().unwrap_or(true) {
            continue;
        }
        let d = delta.get(f).copied().unwrap_or(0.0);
        let Some(Ok(sf)) = shapes.get(f) else { continue };
        let mut faces = vec![f];
        for h in (f + 1)..nf {
            if let Some(Ok(sh)) = shapes.get(h)
                && !taken.get(h).copied().unwrap_or(true)
                && sf.same_surface(sh, SAME_SURFACE_TOL)
            {
                faces.push(h);
            }
        }
        for &h in &faces {
            if let Some(t) = taken.get_mut(h) {
                *t = true;
            }
        }
        if d.abs() >= min_move {
            by_surface.push((faces, d));
        }
    }
    by_surface.sort_by_key(|(faces, _)| {
        matches!(faces.first().and_then(|f| shapes.get(*f)), Some(Ok(FaceShape::Plane { .. })))
    });
    let moves: Vec<FaceMove> =
        by_surface.iter().map(|(faces, d)| FaceMove { faces: faces.clone(), distance: *d }).collect();
    let mut applied_offset = vec![0.0; nf];
    let mut failed_reason: Vec<Option<String>> = vec![None; nf];
    let new_brep = if moves.is_empty() {
        (*brep).clone()
    } else {
        let (nb, outcomes) = offset_face_groups(&brep, &moves)?;
        if outcomes.iter().all(|o| !o.applied) {
            let why = outcomes.iter().filter_map(|o| o.message.clone()).next().unwrap_or_default();
            return Err(CmdError::Failed(format!("the kernel could not move any face: {why}")));
        }
        for (m, o) in moves.iter().zip(&outcomes) {
            for &f in &m.faces {
                if o.applied {
                    if let Some(a) = applied_offset.get_mut(f) {
                        *a = m.distance;
                    }
                } else if let Some(r) = failed_reason.get_mut(f) {
                    *r = o.message.clone();
                }
            }
        }
        nb
    };
    let volume = surfacecraft_kernel::solid::volume(&new_brep).map_err(|e| CmdError::Failed(e.to_string()))?;
    if !(volume.is_finite() && volume > 0.0) {
        return Err(CmdError::Failed("the rebuilt body has no volume".into()));
    }
    let new_shapes = face_shapes(&new_brep)?;
    // report
    let mut faces_out = Vec::with_capacity(nf);
    let (mut sq_before, mut sq_after, mut n_pts) = (0.0, 0.0, 0usize);
    let mut moved = 0;
    for f in 0..nf {
        let st = stiff.get(f).copied().unwrap_or(default_stiffness);
        let own = offsets.get(f).map_or(&[][..], Vec::as_slice);
        match shapes.get(f) {
            Some(Err(why)) => faces_out.push(FaceOut {
                face: f,
                kind: "freeform".into(),
                stiffness: st.name().into(),
                status: "refused".into(),
                points: 0,
                offset: 0.0,
                rms_before: None,
                rms_after: None,
                old: BTreeMap::new(),
                new: BTreeMap::new(),
                reason: Some(why.clone()),
            }),
            Some(Ok(shape)) => {
                let off = applied_offset.get(f).copied().unwrap_or(0.0);
                let predicted = if off != 0.0 { shape.offset(off) } else { Some(shape.clone()) };
                let found = predicted
                    .as_ref()
                    .and_then(|p| new_shapes.iter().flatten().find(|n| n.same_surface(p, 1e-4)))
                    .cloned();
                let after_shape = found.clone().or(predicted.clone());
                let (b2, a2): (f64, f64) = (
                    own.iter().map(|d| d * d).sum(),
                    after_shape.as_ref().map_or(f64::NAN, |ns| {
                        // the same scan points measured against the rebuilt face
                        let mut acc = 0.0;
                        let mut cnt = 0;
                        for (p, a) in points.iter().zip(&assigned) {
                            if matches!(a, Some((g, _)) if *g == f) {
                                let d = ns.signed_distance([p.x, p.y, p.z]);
                                if d.is_finite() {
                                    acc += d * d;
                                    cnt += 1;
                                }
                            }
                        }
                        if cnt == 0 { 0.0 } else { acc }
                    }),
                );
                sq_before += b2;
                if a2.is_finite() {
                    sq_after += a2;
                }
                n_pts += own.len();
                let status = if let Some(m) = failed_reason.get(f).and_then(Clone::clone) {
                    (m, "failed")
                } else if st == Stiffness::Fixed {
                    ("fixed by request".to_string(), "fixed")
                } else if off != 0.0 {
                    (String::new(), "moved")
                } else if let Some(r) = reason.get(f).and_then(Clone::clone) {
                    (r, if own.len() < min_points { "no_data" } else { "unchanged" })
                } else {
                    ("the move is below min_move".to_string(), "unchanged")
                };
                if status.1 == "moved" {
                    moved += 1;
                }
                let n = own.len().max(1) as f64;
                faces_out.push(FaceOut {
                    face: f,
                    kind: shape.kind().into(),
                    stiffness: st.name().into(),
                    status: status.1.into(),
                    points: own.len(),
                    offset: off,
                    rms_before: (!own.is_empty()).then(|| (b2 / n).sqrt()),
                    rms_after: (!own.is_empty() && a2.is_finite()).then(|| (a2 / n).sqrt()),
                    old: shape_params(shape),
                    new: after_shape.as_ref().map(shape_params).unwrap_or_default(),
                    reason: (!status.0.is_empty()).then_some(status.0),
                });
            }
            None => {}
        }
    }
    let denom = n_pts.max(1) as f64;
    let apply = i.apply.unwrap_or(true);
    let out = RefitFacesOut {
        body,
        faces: faces_out,
        rms_before: (sq_before / denom).sqrt(),
        rms_after: (sq_after / denom).sqrt(),
        points: n_pts,
        unassigned,
        moved,
        faces_before: nf,
        faces_after: new_brep.faces.len(),
        relations_before: relations_of(&shapes),
        relations_after: relations_of(&new_shapes),
        applied: apply,
    };
    if apply
        && moved > 0
        && let Some(b) = s.doc.body_mut(body)
    {
        b.brep = Arc::new(new_brep);
    }
    Ok(out)
}

/// `body.refit_*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "body.refit_features",
        "Refit features to scan",
        "Solid",
        true,
        "Optimises dimensions of the feature history of `body` (sketch lengths/radii, extrude depths, blend sizes; `params` or all by default) within ±`max_change` to minimise the deviation of the aligned scan `mesh`; reports old and new values and the RMS before and after.",
        RefitFeaturesIn,
        RefitFeaturesOut,
        refit_features
    ),
    command!(
        "body.refit_faces",
        "Refit faces to scan",
        "Solid",
        true,
        "Moves the analytic faces of a history-free `body` along their normals to match the aligned scan `mesh`, with per-face stiffness (fixed, tight, loose, free), equal radii kept equal, and rebuilds the solid in the kernel; freeform faces are refused.",
        RefitFacesIn,
        RefitFacesOut,
        refit_faces
    ),
];

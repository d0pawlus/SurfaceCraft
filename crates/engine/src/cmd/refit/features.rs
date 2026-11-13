//! The `body.refit_features` command: optimise the dimensions of a body's feature history against a scan.

use super::*;

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

pub(super) fn bounds(old: f64, angle: bool, mc: f64, mca: f64) -> (f64, f64) {
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
    pub(super) fn body(&mut self, values: &[f64]) -> Result<Arc<Brep>, String> {
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

pub(super) fn refit_features(s: &mut Session, i: RefitFeaturesIn) -> CmdResult<RefitFeaturesOut> {
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

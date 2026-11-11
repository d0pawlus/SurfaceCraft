//! `feature.*`: parametric features and the history DAG (design doc §6.4, milestone M5).
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use surfacecraft_doc::{BoolOp, FeatureOp, Mode, ObjectId, ObjectKind, Side};
use surfacecraft_geom::{Point3, Vec3};

/// Result of a feature command.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FeatureOut {
    /// Feature id.
    pub id: ObjectId,
    /// Body it writes.
    pub body: ObjectId,
    /// Body volume after the replay (mm³).
    pub volume: Option<f64>,
    /// Faces of the body.
    pub faces: usize,
    /// Edges of the body.
    pub edges: usize,
    /// This feature's error, if it failed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// Features that failed in the replay.
    pub failed_features: usize,
}

fn report(s: &Session, id: ObjectId, failed: usize) -> CmdResult<FeatureOut> {
    let f = s.doc.feature(id).ok_or_else(|| CmdError::Params(format!("no feature {id}")))?;
    let body = f.body;
    let (volume, faces, edges) = s.doc.body(body).map_or((None, 0, 0), |b| {
        (surfacecraft_kernel::solid::volume(&b.brep).ok(), b.brep.faces.len(), b.brep.edges.len())
    });
    Ok(FeatureOut { id, body, volume, faces, edges, error: f.error.clone(), failed_features: failed })
}

/// Makes sure a body that did not come from a feature enters the history as a base feature.
fn ensure_history(s: &mut Session, body: ObjectId) -> CmdResult<()> {
    if s.doc.features.iter().any(|f| f.body == body) {
        return Ok(());
    }
    let b = s.doc.body(body).ok_or_else(|| CmdError::Params(format!("no body {body}")))?;
    let (name, brep) = (format!("base {}", b.name), b.brep.clone());
    s.doc.add_feature(&name, FeatureOp::Base { brep }, body);
    Ok(())
}

/// Appends a feature and replays; a feature that fails on creation is refused (rolled back).
fn push(
    s: &mut Session,
    name: &str,
    op: FeatureOp,
    body: Option<ObjectId>,
    body_name: Option<&str>,
) -> CmdResult<FeatureOut> {
    for input in op.inputs() {
        if s.doc.kind_of(input) == Some(ObjectKind::Body) {
            ensure_history(s, input)?;
        }
    }
    let body = match body {
        Some(b) => b,
        None => s.doc.fresh_id(),
    };
    let fid = s.doc.add_feature(name, op, body);
    let failed = crate::replay::replay(s);
    if let Some(e) = s.doc.feature(fid).and_then(|f| f.error.clone()) {
        return Err(CmdError::Failed(format!("{name} failed: {e}")));
    }
    if let Some(n) = body_name
        && s.doc.body(body).is_some()
    {
        s.doc.rename(body, n);
    }
    report(s, fid, failed)
}

fn resolve_body(s: &Session, r: &ObjRef) -> CmdResult<ObjectId> {
    r.resolve(s, ObjectKind::Body)
}

/// Extrude parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ExtrudeIn {
    /// Sketch with closed profiles.
    pub sketch: ObjRef,
    /// Distance along the sketch normal (negative = other side).
    pub distance: f64,
    /// Draft angle, degrees.
    #[serde(default)]
    pub draft_deg: f64,
    /// `new` body, `add` to or `cut` from `target`.
    #[serde(default)]
    pub mode: Mode,
    /// Target body for add/cut.
    #[serde(default)]
    pub target: Option<ObjRef>,
    /// Name of a new body.
    #[serde(default)]
    pub name: Option<String>,
}

fn extrude(s: &mut Session, i: ExtrudeIn) -> CmdResult<FeatureOut> {
    let sketch = i.sketch.resolve(s, ObjectKind::Sketch)?;
    let target = i.target.as_ref().map(|t| resolve_body(s, t)).transpose()?;
    if i.mode != Mode::New && target.is_none() {
        return Err(CmdError::Params("add/cut needs `target`".into()));
    }
    let op = FeatureOp::Extrude { sketch, distance: i.distance, draft_deg: i.draft_deg, mode: i.mode, target };
    push(s, "extrude", op, target, i.name.as_deref().or(Some("part")).filter(|_| target.is_none()))
}

/// Revolve parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RevolveIn {
    /// Sketch with a closed profile.
    pub sketch: ObjRef,
    /// A point of the axis (model space).
    pub axis_origin: [f64; 3],
    /// Axis direction.
    pub axis_direction: [f64; 3],
    /// Angle, degrees (default 360).
    #[serde(default)]
    pub angle_deg: Option<f64>,
    /// `new`, `add` or `cut`.
    #[serde(default)]
    pub mode: Mode,
    /// Target body for add/cut.
    #[serde(default)]
    pub target: Option<ObjRef>,
    /// Name of a new body.
    #[serde(default)]
    pub name: Option<String>,
}

fn revolve(s: &mut Session, i: RevolveIn) -> CmdResult<FeatureOut> {
    let sketch = i.sketch.resolve(s, ObjectKind::Sketch)?;
    let target = i.target.as_ref().map(|t| resolve_body(s, t)).transpose()?;
    if i.mode != Mode::New && target.is_none() {
        return Err(CmdError::Params("add/cut needs `target`".into()));
    }
    let op = FeatureOp::Revolve {
        sketch,
        axis_origin: i.axis_origin,
        axis_direction: i.axis_direction,
        angle_deg: i.angle_deg.unwrap_or(360.0),
        mode: i.mode,
        target,
    };
    push(s, "revolve", op, target, i.name.as_deref().or(Some("part")).filter(|_| target.is_none()))
}

/// Edge-blend parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BlendIn {
    /// Body.
    #[schemars(extend("x-pick-body" = true))]
    pub body: ObjRef,
    /// Edge indices.
    #[schemars(extend("x-pick" = "edges"))]
    pub edges: Vec<usize>,
    /// Radius (fillet) or distance (chamfer).
    pub size: f64,
}

fn fillet(s: &mut Session, i: BlendIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    push(
        s,
        "fillet",
        FeatureOp::Fillet { target, edges: i.edges, radius: i.size, edge_refs: Vec::new() },
        Some(target),
        None,
    )
}

fn chamfer(s: &mut Session, i: BlendIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    push(
        s,
        "chamfer",
        FeatureOp::Chamfer { target, edges: i.edges, distance: i.size, edge_refs: Vec::new() },
        Some(target),
        None,
    )
}

/// Shell parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ShellIn {
    /// Body.
    pub body: ObjRef,
    /// Faces removed.
    pub faces: Vec<usize>,
    /// Wall thickness (positive outward, negative inward).
    pub thickness: f64,
}

fn shell(s: &mut Session, i: ShellIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    push(
        s,
        "shell",
        FeatureOp::Shell { target, faces: i.faces, thickness: i.thickness, face_refs: Vec::new() },
        Some(target),
        None,
    )
}

/// Pattern parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PatternIn {
    /// Body.
    pub body: ObjRef,
    /// Instances including the original.
    pub count: usize,
    /// Translation between instances.
    #[serde(default)]
    pub translate: [f64; 3],
    /// Rotation axis between instances (circular pattern).
    #[serde(default)]
    pub rotate_axis: Option<[f64; 3]>,
    /// Point of the rotation axis.
    #[serde(default)]
    pub rotate_origin: [f64; 3],
    /// Rotation angle between instances, degrees.
    #[serde(default)]
    pub rotate_deg: f64,
}

fn pattern(s: &mut Session, i: PatternIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let op = FeatureOp::Pattern {
        target,
        count: i.count,
        translate: i.translate,
        rotate_axis: i.rotate_axis,
        rotate_origin: i.rotate_origin,
        rotate_deg: i.rotate_deg,
    };
    push(s, "pattern", op, Some(target), None)
}

/// Boolean parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BooleanIn {
    /// Body changed.
    pub body: ObjRef,
    /// Tool body (hidden afterwards).
    pub tool: ObjRef,
    /// `union`, `subtract` or `intersect`.
    pub op: BoolOp,
}

fn boolean(s: &mut Session, i: BooleanIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let tool = resolve_body(s, &i.tool)?;
    if target == tool {
        return Err(CmdError::Params("a body cannot be combined with itself".into()));
    }
    push(s, "boolean", FeatureOp::Boolean { target, tool, op: i.op }, Some(target), None)
}

/// Edit parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct EditIn {
    /// Feature id or name.
    pub feature: ObjRef,
    /// Fields of the operation to change (e.g. `{"distance": 12}`, `{"radius": 1.5}`).
    pub set: Value,
}

fn edit(s: &mut Session, i: EditIn) -> CmdResult<FeatureOut> {
    let id = i.feature.resolve(s, ObjectKind::Feature)?;
    let f = s.doc.feature(id).ok_or_else(|| CmdError::Params(format!("no feature {id}")))?;
    if matches!(f.op, FeatureOp::Base { .. }) {
        return Err(CmdError::Params("a base feature has no parameters".into()));
    }
    let mut v = serde_json::to_value(&f.op).map_err(|e| CmdError::Failed(e.to_string()))?;
    let Some(set) = i.set.as_object() else {
        return Err(CmdError::Params("`set` is an object of fields".into()));
    };
    if let Some(obj) = v.as_object_mut() {
        for (k, val) in set {
            if k == "kind" || !obj.contains_key(k) {
                return Err(CmdError::Params(format!("`{k}` is not a parameter of this feature")));
            }
            obj.insert(k.clone(), val.clone());
        }
    }
    // new indices name new references: the stored signatures are recaptured from them
    if let Some(obj) = v.as_object_mut() {
        if set.contains_key("edges") {
            obj.remove("edge_refs");
        }
        if set.contains_key("faces") {
            obj.remove("face_refs");
        }
    }
    let op: FeatureOp = serde_json::from_value(v).map_err(|e| CmdError::Params(e.to_string()))?;
    if let Some(f) = s.doc.feature_mut(id) {
        f.op = op;
    }
    let failed = crate::replay::replay(s);
    if let Some(e) = s.doc.feature(id).and_then(|f| f.error.clone()) {
        return Err(CmdError::Failed(format!("the edited feature fails: {e}")));
    }
    report(s, id, failed)
}

/// Suppress parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SuppressIn {
    /// Feature id or name.
    pub feature: ObjRef,
    /// Suppress (true) or restore (false).
    pub suppressed: bool,
}

fn suppress(s: &mut Session, i: SuppressIn) -> CmdResult<FeatureOut> {
    let id = i.feature.resolve(s, ObjectKind::Feature)?;
    if let Some(f) = s.doc.feature_mut(id) {
        f.suppressed = i.suppressed;
    }
    let failed = crate::replay::replay(s);
    report(s, id, failed)
}

/// One node of the history.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct TreeNode {
    /// Feature id.
    pub id: ObjectId,
    /// Name.
    pub name: String,
    /// Operation.
    pub op: String,
    /// Body written.
    pub body: ObjectId,
    /// Earlier features this one depends on (through its sketch's or bodies' writers).
    pub depends_on: Vec<ObjectId>,
    /// Sketch read, if any.
    pub sketch: Option<ObjectId>,
    /// Suppressed?
    pub suppressed: bool,
    /// Error of the last replay.
    pub error: Option<String>,
}

/// The history.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct TreeOut {
    /// Features in order.
    pub features: Vec<TreeNode>,
    /// Features that failed.
    pub failed_features: usize,
}

fn tree_of(s: &Session, failed: usize) -> TreeOut {
    let mut nodes = Vec::new();
    for (k, f) in s.doc.features.iter().enumerate() {
        let mut deps = Vec::new();
        let mut sketch = None;
        for input in f.op.inputs() {
            if s.doc.kind_of(input) == Some(ObjectKind::Sketch) {
                sketch = Some(input);
            }
            // the last earlier feature writing that body
            if let Some(w) = s.doc.features.iter().take(k).rev().find(|g| g.body == input) {
                deps.push(w.id);
            }
        }
        // a feature also follows the previous writer of its own body
        if let Some(w) = s.doc.features.iter().take(k).rev().find(|g| g.body == f.body)
            && !deps.contains(&w.id)
        {
            deps.push(w.id);
        }
        nodes.push(TreeNode {
            id: f.id,
            name: f.name.clone(),
            op: f.op.label().into(),
            body: f.body,
            depends_on: deps,
            sketch,
            suppressed: f.suppressed,
            error: f.error.clone(),
        });
    }
    TreeOut { features: nodes, failed_features: failed }
}

fn tree(s: &mut Session, _: super::NoParams) -> CmdResult<TreeOut> {
    let failed = s.doc.features.iter().filter(|f| f.error.is_some()).count();
    Ok(tree_of(s, failed))
}

fn replay_cmd(s: &mut Session, _: super::NoParams) -> CmdResult<TreeOut> {
    let failed = crate::replay::replay(s);
    Ok(tree_of(s, failed))
}

/// Suggestion parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SuggestIn {
    /// Body whose edge would be filleted.
    pub body: ObjRef,
    /// Edge index.
    pub edge: usize,
    /// Scan mesh to measure on.
    pub mesh: ObjRef,
    /// Search radius around the edge (mm, default 10).
    #[serde(default)]
    pub search: Option<f64>,
}

/// A suggested parameter.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SuggestOut {
    /// Suggested fillet radius (mm).
    pub radius: f64,
    /// RMS of the cylinder fitted to the scan around the edge (mm).
    pub rms: f64,
    /// Scan points used.
    pub points: usize,
}

/// Redesign-assistant style suggestion (design doc §6.4): the radius of the rounded scan around a
/// body edge, from a cylinder fitted to the scan points near the edge's middle.
fn suggest_fillet(s: &mut Session, i: SuggestIn) -> CmdResult<SuggestOut> {
    let body = resolve_body(s, &i.body)?;
    let mesh = i.mesh.resolve(s, ObjectKind::Mesh)?;
    let b = s.doc.body(body).ok_or_else(|| CmdError::Params(format!("no body {body}")))?;
    let e = b.brep.edges.get(i.edge).ok_or_else(|| CmdError::Params(format!("edge {} does not exist", i.edge)))?;
    let v = |k: u32| b.brep.vertices.get(k as usize).map(|v| Point3::from(v.point));
    let (Some(a), Some(c)) = (v(e.vertices[0]), v(e.vertices[1])) else {
        return Err(CmdError::Failed("edge without vertices".into()));
    };
    let mid = Point3::from((a.coords + c.coords) / 2.0);
    let along = surfacecraft_geom::normalize(c - a).unwrap_or_else(Vec3::z);
    let half = ((c - a).norm() / 2.0) * 0.6;
    let r = i.search.unwrap_or(10.0);
    if !(r.is_finite() && r > 0.0) {
        return Err(CmdError::Params("search must be positive".into()));
    }
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let normals = m.vertex_normals();
    let mut pts = Vec::new();
    let mut nrm = Vec::new();
    for (p, n) in m.positions.iter().zip(&normals) {
        let w = p - mid;
        let t = w.dot(&along);
        if t.abs() <= half && (w - along * t).norm() <= r {
            // only the rounded part: normals turning across the edge
            pts.push(*p);
            nrm.push(*n);
        }
    }
    let f = surfacecraft_fit::primitive::fit_cylinder(&pts, &nrm)
        .ok_or_else(|| CmdError::Failed("no rounded scan near this edge".into()))?;
    let surfacecraft_fit::Primitive::Cylinder { radius, .. } = f.primitive else {
        return Err(CmdError::Failed("no cylinder".into()));
    };
    Ok(SuggestOut { radius, rms: f.rms, points: pts.len() })
}

fn new_body_name(i: &Option<String>, target: Option<ObjectId>) -> Option<String> {
    target.is_none().then(|| i.clone().unwrap_or_else(|| "part".to_string()))
}

/// Loft parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LoftIn {
    /// Section sketches in order (each contributes its outer closed profile; at least two).
    pub sketches: Vec<ObjRef>,
    /// Degree across the sections (1 = ruled, up to 3; default: the highest the count allows).
    #[serde(default)]
    pub degree: Option<u32>,
    /// `new`, `add` or `cut`.
    #[serde(default)]
    pub mode: Mode,
    /// Target body for add/cut.
    #[serde(default)]
    pub target: Option<ObjRef>,
    /// Name of a new body.
    #[serde(default)]
    pub name: Option<String>,
}

fn loft(s: &mut Session, i: LoftIn) -> CmdResult<FeatureOut> {
    if i.sketches.len() < 2 {
        return Err(CmdError::Params("a loft needs at least two section sketches".into()));
    }
    let sketches = i.sketches.iter().map(|r| r.resolve(s, ObjectKind::Sketch)).collect::<CmdResult<Vec<_>>>()?;
    let target = i.target.as_ref().map(|t| resolve_body(s, t)).transpose()?;
    if i.mode != Mode::New && target.is_none() {
        return Err(CmdError::Params("add/cut needs `target`".into()));
    }
    let op = FeatureOp::Loft { sketches, degree: i.degree, mode: i.mode, target };
    push(s, "loft", op, target, new_body_name(&i.name, target).as_deref())
}

/// Sweep parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SweepIn {
    /// Sketch with the closed profile, placed at the start of the path.
    pub profile: ObjRef,
    /// Sketch with the open path (a chain of lines and arcs).
    pub path: ObjRef,
    /// The profile turns with the path (default true) or only moves along it.
    #[serde(default)]
    pub follow: Option<bool>,
    /// `new`, `add` or `cut`.
    #[serde(default)]
    pub mode: Mode,
    /// Target body for add/cut.
    #[serde(default)]
    pub target: Option<ObjRef>,
    /// Name of a new body.
    #[serde(default)]
    pub name: Option<String>,
}

fn sweep(s: &mut Session, i: SweepIn) -> CmdResult<FeatureOut> {
    let profile = i.profile.resolve(s, ObjectKind::Sketch)?;
    let path = i.path.resolve(s, ObjectKind::Sketch)?;
    let target = i.target.as_ref().map(|t| resolve_body(s, t)).transpose()?;
    if i.mode != Mode::New && target.is_none() {
        return Err(CmdError::Params("add/cut needs `target`".into()));
    }
    let op = FeatureOp::Sweep { profile, path, follow: i.follow.unwrap_or(true), mode: i.mode, target };
    push(s, "sweep", op, target, new_body_name(&i.name, target).as_deref())
}

/// Draft parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DraftIn {
    /// Body.
    pub body: ObjRef,
    /// Faces drafted (record indices).
    pub faces: Vec<usize>,
    /// Angle, degrees (positive leans the faces in going along `pull`).
    pub angle_deg: f64,
    /// A point of the neutral plane (it does not move).
    pub neutral_origin: [f64; 3],
    /// Normal of the neutral plane.
    pub neutral_normal: [f64; 3],
    /// Pull direction (default: `neutral_normal`).
    #[serde(default)]
    pub pull: Option<[f64; 3]>,
}

fn draft(s: &mut Session, i: DraftIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let op = FeatureOp::Draft {
        target,
        faces: i.faces,
        angle_deg: i.angle_deg,
        neutral_origin: i.neutral_origin,
        neutral_normal: i.neutral_normal,
        pull: i.pull,
        face_refs: Vec::new(),
    };
    push(s, "draft", op, Some(target), None)
}

/// Mirror parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MirrorIn {
    /// Body.
    pub body: ObjRef,
    /// A point of the mirror plane.
    pub plane_origin: [f64; 3],
    /// Normal of the mirror plane.
    pub plane_normal: [f64; 3],
    /// Unite the reflection with the original (default true) or keep only the reflection.
    #[serde(default)]
    pub merge: Option<bool>,
}

fn mirror(s: &mut Session, i: MirrorIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let op = FeatureOp::Mirror {
        target,
        plane_origin: i.plane_origin,
        plane_normal: i.plane_normal,
        merge: i.merge.unwrap_or(true),
    };
    push(s, "mirror", op, Some(target), None)
}

/// Split parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SplitIn {
    /// Body.
    pub body: ObjRef,
    /// A point of the cutting plane.
    pub plane_origin: [f64; 3],
    /// Normal of the cutting plane.
    pub plane_normal: [f64; 3],
    /// The side kept: `front` (the normal side, default) or `back`.
    #[serde(default)]
    pub keep: Side,
}

fn split(s: &mut Session, i: SplitIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let op = FeatureOp::Split { target, plane_origin: i.plane_origin, plane_normal: i.plane_normal, keep: i.keep };
    push(s, "split", op, Some(target), None)
}

/// Pattern-along-a-path parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PathPatternIn {
    /// Body.
    pub body: ObjRef,
    /// Sketch with the open path (a chain of lines and arcs).
    pub path: ObjRef,
    /// Instances including the original (at the start of the path), evenly spaced by arc length.
    pub count: usize,
    /// Each copy turns with the path tangent (default false: translated only).
    #[serde(default)]
    pub follow: bool,
}

fn path_pattern(s: &mut Session, i: PathPatternIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let path = i.path.resolve(s, ObjectKind::Sketch)?;
    push(
        s,
        "path_pattern",
        FeatureOp::PathPattern { target, path, count: i.count, follow: i.follow },
        Some(target),
        None,
    )
}

/// The scan points (and normals) a suggestion measures on: a region, or a mesh and triangles.
fn selection_points(
    s: &Session,
    region: &Option<ObjRef>,
    mesh: &Option<ObjRef>,
    triangles: &Option<Vec<u32>>,
) -> CmdResult<(Vec<Point3>, Vec<Vec3>)> {
    let (mesh, tris) = match (region, mesh, triangles) {
        (Some(r), None, None) => {
            let rid = r.resolve(s, ObjectKind::Region)?;
            let reg = s.doc.region(rid).ok_or_else(|| CmdError::Params(format!("no region {rid}")))?;
            (reg.mesh, reg.triangles.to_vec())
        }
        (None, Some(m), Some(t)) => (m.resolve(s, ObjectKind::Mesh)?, t.clone()),
        _ => return Err(CmdError::Params("give `region`, or `mesh` and `triangles`".into())),
    };
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    if let Some(t) = tris.iter().find(|&&t| t as usize >= m.triangle_count()) {
        return Err(CmdError::Params(format!("triangle {t} out of range")));
    }
    let normals = m.vertex_normals();
    let (p, n) = surfacecraft_fit::segment::region_points(&m, &normals, &tris);
    if p.len() < 6 {
        return Err(CmdError::Params("too few scan points to measure".into()));
    }
    Ok((p, n))
}

/// Axis suggestion parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SuggestAxisIn {
    /// Region of a turned (revolved) surface of the scan.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Mesh (with `triangles`) when no region is given.
    #[serde(default)]
    pub mesh: Option<ObjRef>,
    /// Triangles of `mesh`.
    #[serde(default)]
    pub triangles: Option<Vec<u32>>,
}

/// A suggested revolve axis.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SuggestAxisOut {
    /// A point of the axis (`axis_origin` of `feature.revolve`).
    pub axis_origin: [f64; 3],
    /// Unit axis direction (`axis_direction` of `feature.revolve`).
    pub axis_direction: [f64; 3],
    /// What the scan is: `cylinder`, `cone`, `torus` or `sphere`.
    pub surface: String,
    /// Radius (cylinder, sphere) or major radius (torus); 0 for a cone.
    pub radius: f64,
    /// RMS of the fit (mm).
    pub rms: f64,
    /// Scan points used.
    pub points: usize,
}

/// Redesign assistant (design doc §6.4): the revolve axis of a turned scan surface, from the best
/// cylinder, cone, torus or sphere fitted to it.
fn suggest_axis(s: &mut Session, i: SuggestAxisIn) -> CmdResult<SuggestAxisOut> {
    use surfacecraft_fit::{Primitive, PrimitiveKind};
    let (p, n) = selection_points(s, &i.region, &i.mesh, &i.triangles)?;
    let fits: Vec<_> = [PrimitiveKind::Sphere, PrimitiveKind::Cylinder, PrimitiveKind::Cone, PrimitiveKind::Torus]
        .into_iter()
        .filter_map(|k| surfacecraft_fit::primitive::fit(k, &p, &n))
        .filter(|f| f.rms.is_finite())
        .collect();
    let floor = fits.iter().map(|f| f.rms).fold(f64::INFINITY, f64::min);
    // a plane that fits as well as any turned surface means there is no axis to find
    if let Some(pl) = surfacecraft_fit::primitive::fit(PrimitiveKind::Plane, &p, &n)
        && pl.rms <= floor * 1.2 + 1e-9
    {
        return Err(CmdError::Failed("the scan is planar: it has no axis".into()));
    }
    // the simplest surface within 20 % of the best residual wins (a barely tapered cone is a cylinder)
    let best = fits
        .into_iter()
        .find(|f| f.rms <= floor * 1.2 + 1e-9)
        .ok_or_else(|| CmdError::Failed("no surface of revolution fits these scan points".into()))?;
    let (origin, dir, surface, radius) = match best.primitive {
        Primitive::Cylinder { point, axis, radius } => (point, axis, "cylinder", radius),
        Primitive::Cone { apex, axis, .. } => (apex, axis, "cone", 0.0),
        Primitive::Torus { center, axis, major_radius, .. } => (center, axis, "torus", major_radius),
        Primitive::Sphere { center, radius } => (center, [0.0, 0.0, 1.0], "sphere", radius),
        Primitive::Plane { .. } => return Err(CmdError::Failed("the scan is planar: it has no axis".into())),
    };
    Ok(SuggestAxisOut {
        axis_origin: origin,
        axis_direction: dir,
        surface: surface.into(),
        radius,
        rms: best.rms,
        points: best.count,
    })
}

/// Draft suggestion parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SuggestDraftIn {
    /// Region of the side face (a plane or a cone) of the scan.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Mesh (with `triangles`) when no region is given.
    #[serde(default)]
    pub mesh: Option<ObjRef>,
    /// Triangles of `mesh`.
    #[serde(default)]
    pub triangles: Option<Vec<u32>>,
    /// Pull (mould opening) direction.
    pub pull: [f64; 3],
}

/// A suggested draft.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SuggestDraftOut {
    /// Draft angle, degrees (sign as `feature.draft`: positive leans the face in going along `pull`).
    pub angle_deg: f64,
    /// `plane` or `cone`.
    pub surface: String,
    /// RMS of the fit (mm).
    pub rms: f64,
    /// Scan points used.
    pub points: usize,
}

/// Redesign assistant: the draft angle of a scanned side face against the pull direction, from a
/// plane (angle between the face and the pull) or a cone (half angle) fitted to it.
fn suggest_draft(s: &mut Session, i: SuggestDraftIn) -> CmdResult<SuggestDraftOut> {
    use surfacecraft_fit::{Primitive, PrimitiveKind};
    let pull = surfacecraft_geom::normalize(Vec3::from(i.pull))
        .filter(|v| v.iter().all(|c| c.is_finite()))
        .ok_or_else(|| CmdError::Params("`pull` must be a finite non-zero direction".into()))?;
    let (p, n) = selection_points(s, &i.region, &i.mesh, &i.triangles)?;
    let plane = surfacecraft_fit::primitive::fit(PrimitiveKind::Plane, &p, &n);
    let cone = surfacecraft_fit::primitive::fit(PrimitiveKind::Cone, &p, &n);
    // the simpler surface wins unless the cone fits clearly better (as the auto-typing rule)
    let use_cone = match (&plane, &cone) {
        (Some(pl), Some(c)) => c.rms * 1.2 < pl.rms,
        (None, Some(_)) => true,
        _ => false,
    };
    if use_cone && let Some(c) = cone {
        let Primitive::Cone { axis, half_angle, .. } = c.primitive else {
            return Err(CmdError::Failed("no cone".into()));
        };
        // the cone must be turned about the pull for a draft in the mould sense
        if Vec3::from(axis).dot(&pull).abs() < 0.9 {
            return Err(CmdError::Failed("the conical face is not turned about the pull direction".into()));
        }
        // a cone opening along the pull widens along it: its walls lean out, going along the pull
        let sign = if Vec3::from(axis).dot(&pull) > 0.0 { -1.0 } else { 1.0 };
        return Ok(SuggestDraftOut {
            angle_deg: sign * half_angle.to_degrees(),
            surface: "cone".into(),
            rms: c.rms,
            points: c.count,
        });
    }
    let pl = plane.ok_or_else(|| CmdError::Failed("no plane or cone fits these scan points".into()))?;
    let Primitive::Plane { normal, .. } = pl.primitive else {
        return Err(CmdError::Failed("no plane".into()));
    };
    // outward normal of the scanned face: the fitted normal turned to agree with the mesh normals
    let mean = n.iter().fold(Vec3::zeros(), |a, v| a + v);
    let mut nv = Vec3::from(normal);
    if nv.dot(&mean) < 0.0 {
        nv = -nv;
    }
    // positive: the outward normal leans along the pull, i.e. the face leans in going along it
    let angle = nv.dot(&pull).clamp(-1.0, 1.0).asin().to_degrees();
    Ok(SuggestDraftOut { angle_deg: angle, surface: "plane".into(), rms: pl.rms, points: pl.count })
}

/// `feature.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "feature.loft",
        "Loft",
        "Feature",
        true,
        "Skins a solid through the outer profiles of `sketches` (degree 1 = ruled), new/add/cut.",
        LoftIn,
        FeatureOut,
        loft
    ),
    command!(
        "feature.sweep",
        "Sweep",
        "Feature",
        true,
        "Sweeps the profile of `profile` along the open path of `path` (follow = turns with the path), new/add/cut.",
        SweepIn,
        FeatureOut,
        sweep
    ),
    command!(
        "feature.draft",
        "Draft",
        "Feature",
        true,
        "Tilts `faces` of `body` by `angle_deg` about the neutral plane, leaning in along `pull` (parametric, faces tracked by signature).",
        DraftIn,
        FeatureOut,
        draft
    ),
    command!(
        "feature.mirror",
        "Mirror",
        "Feature",
        true,
        "Reflects `body` in a plane and unites the copy with it (or keeps only the reflection).",
        MirrorIn,
        FeatureOut,
        mirror
    ),
    command!(
        "feature.split",
        "Split / trim",
        "Feature",
        true,
        "Cuts `body` by a plane and keeps the `front` or `back` side.",
        SplitIn,
        FeatureOut,
        split
    ),
    command!(
        "feature.path_pattern",
        "Pattern along path",
        "Feature",
        true,
        "`count` instances of `body` evenly spaced along the open path sketch `path` (optionally turning with it).",
        PathPatternIn,
        FeatureOut,
        path_pattern
    ),
    command!(
        "feature.suggest_axis",
        "Suggest revolve axis",
        "Feature",
        false,
        "Axis (origin and direction) of the turned scan surface of `region`, for `feature.revolve` (redesign assistant).",
        SuggestAxisIn,
        SuggestAxisOut,
        suggest_axis
    ),
    command!(
        "feature.suggest_draft",
        "Suggest draft angle",
        "Feature",
        false,
        "Draft angle of the scanned side face of `region` against the `pull` direction, for `feature.draft` (redesign assistant).",
        SuggestDraftIn,
        SuggestDraftOut,
        suggest_draft
    ),
    command!(
        "feature.extrude",
        "Extrude",
        "Feature",
        true,
        "Extrudes the closed profiles of `sketch` by `distance` (draft, new/add/cut on `target`).",
        ExtrudeIn,
        FeatureOut,
        extrude
    ),
    command!(
        "feature.revolve",
        "Revolve",
        "Feature",
        true,
        "Revolves the first profile of `sketch` about an axis by `angle_deg` (new/add/cut).",
        RevolveIn,
        FeatureOut,
        revolve
    ),
    command!(
        "feature.fillet",
        "Fillet",
        "Feature",
        true,
        "Rounds `edges` of `body` with radius `size` (parametric).",
        BlendIn,
        FeatureOut,
        fillet
    ),
    command!(
        "feature.chamfer",
        "Chamfer",
        "Feature",
        true,
        "Bevels `edges` of `body` by `size` (parametric).",
        BlendIn,
        FeatureOut,
        chamfer
    ),
    command!(
        "feature.shell",
        "Shell",
        "Feature",
        true,
        "Hollows `body`, removing `faces`, walls of `thickness`.",
        ShellIn,
        FeatureOut,
        shell
    ),
    command!(
        "feature.pattern",
        "Pattern",
        "Feature",
        true,
        "`count` instances of `body`, each moved by `translate` and/or rotated by `rotate_deg` about `rotate_axis`.",
        PatternIn,
        FeatureOut,
        pattern
    ),
    command!(
        "feature.boolean",
        "Boolean",
        "Feature",
        true,
        "Union/subtract/intersect `tool` with `body`.",
        BooleanIn,
        FeatureOut,
        boolean
    ),
    command!(
        "feature.edit",
        "Edit feature",
        "Feature",
        true,
        "Changes parameters `set` of `feature` and replays the history.",
        EditIn,
        FeatureOut,
        edit
    ),
    command!(
        "feature.suppress",
        "Suppress",
        "Feature",
        true,
        "Suppresses or restores `feature` and replays.",
        SuppressIn,
        FeatureOut,
        suppress
    ),
    command!(
        "feature.tree",
        "History",
        "Feature",
        false,
        "The feature history with dependencies and errors.",
        super::NoParams,
        TreeOut,
        tree
    ),
    command!(
        "feature.replay",
        "Replay history",
        "Feature",
        true,
        "Rebuilds every feature-made body from the history.",
        super::NoParams,
        TreeOut,
        replay_cmd
    ),
    command!(
        "feature.suggest_fillet",
        "Suggest fillet radius",
        "Feature",
        false,
        "Radius of the rounded scan around `edge` of `body`, measured on `mesh` (redesign assistant).",
        SuggestIn,
        SuggestOut,
        suggest_fillet
    ),
];

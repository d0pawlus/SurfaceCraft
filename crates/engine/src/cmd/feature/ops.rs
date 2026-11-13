//! Creating and changing features: extrude, revolve, blends, shell, pattern, boolean, edit and suppress.

use super::*;

/// Extrude parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ExtrudeIn {
    /// Sketch with closed profiles.
    pub sketch: ObjRef,
    /// Distance along the sketch normal (negative = other side).
    pub distance: Num,
    /// Draft angle, degrees.
    #[serde(default)]
    pub draft_deg: Num,
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

pub(super) fn extrude(s: &mut Session, i: ExtrudeIn) -> CmdResult<FeatureOut> {
    let sketch = i.sketch.resolve(s, ObjectKind::Sketch)?;
    let target = i.target.as_ref().map(|t| resolve_body(s, t)).transpose()?;
    if i.mode != Mode::New && target.is_none() {
        return Err(CmdError::Params("add/cut needs `target`".into()));
    }
    let mut x = ExprSet::new(&s.doc.params);
    let (distance, draft_deg) = (x.num("distance", &i.distance)?, x.num("draft_deg", &i.draft_deg)?);
    let op = FeatureOp::Extrude { sketch, distance, draft_deg, mode: i.mode, target };
    push(s, "extrude", (op, x.finish()), target, i.name.as_deref().or(Some("part")).filter(|_| target.is_none()))
}

/// Revolve parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RevolveIn {
    /// Sketch with a closed profile.
    pub sketch: ObjRef,
    /// A point of the axis (model space).
    pub axis_origin: [Num; 3],
    /// Axis direction.
    pub axis_direction: [Num; 3],
    /// Angle, degrees (default 360).
    #[serde(default)]
    pub angle_deg: Option<Num>,
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

pub(super) fn revolve(s: &mut Session, i: RevolveIn) -> CmdResult<FeatureOut> {
    let sketch = i.sketch.resolve(s, ObjectKind::Sketch)?;
    let target = i.target.as_ref().map(|t| resolve_body(s, t)).transpose()?;
    if i.mode != Mode::New && target.is_none() {
        return Err(CmdError::Params("add/cut needs `target`".into()));
    }
    let mut x = ExprSet::new(&s.doc.params);
    let op = FeatureOp::Revolve {
        sketch,
        axis_origin: x.vec3("axis_origin", &i.axis_origin)?,
        axis_direction: x.vec3("axis_direction", &i.axis_direction)?,
        angle_deg: x.opt("angle_deg", &i.angle_deg)?.unwrap_or(360.0),
        mode: i.mode,
        target,
    };
    push(s, "revolve", (op, x.finish()), target, i.name.as_deref().or(Some("part")).filter(|_| target.is_none()))
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
    pub size: Num,
}

pub(super) fn fillet(s: &mut Session, i: BlendIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let mut x = ExprSet::new(&s.doc.params);
    let radius = x.num("radius", &i.size)?;
    let op = FeatureOp::Fillet { target, edges: i.edges, radius, edge_refs: Vec::new() };
    push(s, "fillet", (op, x.finish()), Some(target), None)
}

pub(super) fn chamfer(s: &mut Session, i: BlendIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let mut x = ExprSet::new(&s.doc.params);
    let distance = x.num("distance", &i.size)?;
    let op = FeatureOp::Chamfer { target, edges: i.edges, distance, edge_refs: Vec::new() };
    push(s, "chamfer", (op, x.finish()), Some(target), None)
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
    pub thickness: Num,
}

pub(super) fn shell(s: &mut Session, i: ShellIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let mut x = ExprSet::new(&s.doc.params);
    let thickness = x.num("thickness", &i.thickness)?;
    let op = FeatureOp::Shell { target, faces: i.faces, thickness, face_refs: Vec::new() };
    push(s, "shell", (op, x.finish()), Some(target), None)
}

/// Pattern parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PatternIn {
    /// Body.
    pub body: ObjRef,
    /// Instances including the original.
    pub count: Count,
    /// Translation between instances.
    #[serde(default = "zero3")]
    pub translate: [Num; 3],
    /// Rotation axis between instances (circular pattern).
    #[serde(default)]
    pub rotate_axis: Option<[Num; 3]>,
    /// Point of the rotation axis.
    #[serde(default = "zero3")]
    pub rotate_origin: [Num; 3],
    /// Rotation angle between instances, degrees.
    #[serde(default)]
    pub rotate_deg: Num,
}

pub(super) fn pattern(s: &mut Session, i: PatternIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let mut x = ExprSet::new(&s.doc.params);
    let op = FeatureOp::Pattern {
        target,
        count: x.count("count", &i.count)?,
        translate: x.vec3("translate", &i.translate)?,
        rotate_axis: x.opt_vec3("rotate_axis", &i.rotate_axis)?,
        rotate_origin: x.vec3("rotate_origin", &i.rotate_origin)?,
        rotate_deg: x.num("rotate_deg", &i.rotate_deg)?,
    };
    push(s, "pattern", (op, x.finish()), Some(target), None)
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

pub(super) fn boolean(s: &mut Session, i: BooleanIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let tool = resolve_body(s, &i.tool)?;
    if target == tool {
        return Err(CmdError::Params("a body cannot be combined with itself".into()));
    }
    push(s, "boolean", (FeatureOp::Boolean { target, tool, op: i.op }, BTreeMap::new()), Some(target), None)
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

pub(super) fn edit(s: &mut Session, i: EditIn) -> CmdResult<FeatureOut> {
    let id = i.feature.resolve(s, ObjectKind::Feature)?;
    let f = s.doc.feature(id).ok_or_else(|| CmdError::Params(format!("no feature {id}")))?;
    if matches!(f.op, FeatureOp::Base { .. }) {
        return Err(CmdError::Params("a base feature has no parameters".into()));
    }
    let Some(set) = i.set.as_object() else {
        return Err(CmdError::Params("`set` is an object of fields".into()));
    };
    let (op, exprs, cleared) = exprs::edited(f, set, &s.doc.params)?;
    if let Some(f) = s.doc.feature_mut(id) {
        f.op = op;
        f.exprs = exprs;
    }
    let failed = crate::replay::replay(s);
    if let Some(e) = s.doc.feature(id).and_then(|f| f.error.clone()) {
        return Err(CmdError::Failed(format!("the edited feature fails: {e}")));
    }
    let expressions_cleared =
        cleared.into_iter().map(|(field, expression)| ClearedExpr { field, expression }).collect();
    Ok(FeatureOut { expressions_cleared, ..report(s, id, failed)? })
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

pub(super) fn suppress(s: &mut Session, i: SuppressIn) -> CmdResult<FeatureOut> {
    let id = i.feature.resolve(s, ObjectKind::Feature)?;
    if let Some(f) = s.doc.feature_mut(id) {
        f.suppressed = i.suppressed;
    }
    let failed = crate::replay::replay(s);
    report(s, id, failed)
}

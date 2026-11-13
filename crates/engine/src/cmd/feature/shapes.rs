//! Loft, sweep, draft, mirror, split and path pattern features.

use super::suggest::*;
use super::*;

/// Loft parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LoftIn {
    /// Section sketches in order (each contributes its outer closed profile; at least two).
    pub sketches: Vec<ObjRef>,
    /// Degree across the sections (1 = ruled, up to 3; default: the highest the count allows).
    #[serde(default)]
    pub degree: Option<Count>,
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

pub(super) fn loft(s: &mut Session, i: LoftIn) -> CmdResult<FeatureOut> {
    if i.sketches.len() < 2 {
        return Err(CmdError::Params("a loft needs at least two section sketches".into()));
    }
    let sketches = i.sketches.iter().map(|r| r.resolve(s, ObjectKind::Sketch)).collect::<CmdResult<Vec<_>>>()?;
    let target = i.target.as_ref().map(|t| resolve_body(s, t)).transpose()?;
    if i.mode != Mode::New && target.is_none() {
        return Err(CmdError::Params("add/cut needs `target`".into()));
    }
    let mut x = ExprSet::new(&s.doc.params);
    let op = FeatureOp::Loft { sketches, degree: x.opt_count("degree", &i.degree)?, mode: i.mode, target };
    push(s, "loft", (op, x.finish()), target, new_body_name(&i.name, target).as_deref())
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

pub(super) fn sweep(s: &mut Session, i: SweepIn) -> CmdResult<FeatureOut> {
    let profile = i.profile.resolve(s, ObjectKind::Sketch)?;
    let path = i.path.resolve(s, ObjectKind::Sketch)?;
    let target = i.target.as_ref().map(|t| resolve_body(s, t)).transpose()?;
    if i.mode != Mode::New && target.is_none() {
        return Err(CmdError::Params("add/cut needs `target`".into()));
    }
    let op = FeatureOp::Sweep { profile, path, follow: i.follow.unwrap_or(true), mode: i.mode, target };
    push(s, "sweep", (op, BTreeMap::new()), target, new_body_name(&i.name, target).as_deref())
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
    pub angle_deg: Num,
    /// A point of the neutral plane (it does not move).
    pub neutral_origin: [Num; 3],
    /// Normal of the neutral plane.
    pub neutral_normal: [Num; 3],
    /// Pull direction (default: `neutral_normal`).
    #[serde(default)]
    pub pull: Option<[Num; 3]>,
}

pub(super) fn draft(s: &mut Session, i: DraftIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let mut x = ExprSet::new(&s.doc.params);
    let op = FeatureOp::Draft {
        target,
        faces: i.faces,
        angle_deg: x.num("angle_deg", &i.angle_deg)?,
        neutral_origin: x.vec3("neutral_origin", &i.neutral_origin)?,
        neutral_normal: x.vec3("neutral_normal", &i.neutral_normal)?,
        pull: x.opt_vec3("pull", &i.pull)?,
        face_refs: Vec::new(),
    };
    push(s, "draft", (op, x.finish()), Some(target), None)
}

/// Mirror parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MirrorIn {
    /// Body.
    pub body: ObjRef,
    /// A point of the mirror plane.
    pub plane_origin: [Num; 3],
    /// Normal of the mirror plane.
    pub plane_normal: [Num; 3],
    /// Unite the reflection with the original (default true) or keep only the reflection.
    #[serde(default)]
    pub merge: Option<bool>,
}

pub(super) fn mirror(s: &mut Session, i: MirrorIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let mut x = ExprSet::new(&s.doc.params);
    let op = FeatureOp::Mirror {
        target,
        plane_origin: x.vec3("plane_origin", &i.plane_origin)?,
        plane_normal: x.vec3("plane_normal", &i.plane_normal)?,
        merge: i.merge.unwrap_or(true),
    };
    push(s, "mirror", (op, x.finish()), Some(target), None)
}

/// Split parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SplitIn {
    /// Body.
    pub body: ObjRef,
    /// A point of the cutting plane.
    pub plane_origin: [Num; 3],
    /// Normal of the cutting plane.
    pub plane_normal: [Num; 3],
    /// The side kept: `front` (the normal side, default) or `back`.
    #[serde(default)]
    pub keep: Side,
}

pub(super) fn split(s: &mut Session, i: SplitIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let mut x = ExprSet::new(&s.doc.params);
    let op = FeatureOp::Split {
        target,
        plane_origin: x.vec3("plane_origin", &i.plane_origin)?,
        plane_normal: x.vec3("plane_normal", &i.plane_normal)?,
        keep: i.keep,
    };
    push(s, "split", (op, x.finish()), Some(target), None)
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
    pub count: Count,
    /// Each copy turns with the path tangent (default false: translated only).
    #[serde(default)]
    pub follow: bool,
}

pub(super) fn path_pattern(s: &mut Session, i: PathPatternIn) -> CmdResult<FeatureOut> {
    let target = resolve_body(s, &i.body)?;
    let path = i.path.resolve(s, ObjectKind::Sketch)?;
    let mut x = ExprSet::new(&s.doc.params);
    let op = FeatureOp::PathPattern { target, path, count: x.count("count", &i.count)?, follow: i.follow };
    push(s, "path_pattern", (op, x.finish()), Some(target), None)
}

/// The scan points (and normals) a suggestion measures on: a region, or a mesh and triangles.
pub(super) fn selection_points(
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

//! `primitive.*`: analytic primitives fitted to regions or triangles, and the symmetry plane.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::segment::region_points;
use surfacecraft_fit::{Primitive, PrimitiveKind, constrain, fit_auto, primitive::fit};
use surfacecraft_geom::Point3;

/// Which kind to fit.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, JsonSchema, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum KindChoice {
    /// Fit every kind and choose by residual.
    #[default]
    Auto,
    /// Plane.
    Plane,
    /// Sphere.
    Sphere,
    /// Cylinder.
    Cylinder,
    /// Cone.
    Cone,
    /// Torus.
    Torus,
}

/// Fit parameters: a region, or a mesh and triangles.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FitIn {
    /// Region to fit.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Mesh (with `triangles`) when no region is given.
    #[serde(default)]
    pub mesh: Option<ObjRef>,
    /// Triangles of `mesh`.
    #[serde(default)]
    pub triangles: Option<Vec<u32>>,
    /// Kind (default auto).
    #[serde(default)]
    pub kind: KindChoice,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

/// A fitted primitive.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FitOut {
    /// New primitive object.
    pub id: ObjectId,
    /// The surface.
    pub primitive: Primitive,
    /// RMS (mm).
    pub rms: f64,
    /// Largest deviation (mm).
    pub max: f64,
    /// Points fitted.
    pub count: usize,
    /// Kinds tried with their RMS (auto only).
    pub candidates: Vec<(PrimitiveKind, f64)>,
    /// Advice, e.g. "more suitable as a cylinder".
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
    /// Warning when the RMS exceeds the default tolerance (design doc §11.1: 0.1 mm; warn above 0.05).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warning: Option<String>,
}

pub(crate) fn fit_cmd(s: &mut Session, i: FitIn) -> CmdResult<FitOut> {
    let (mesh, tris, region) = match (&i.region, &i.mesh, &i.triangles) {
        (Some(r), None, None) => {
            let rid = r.resolve(s, ObjectKind::Region)?;
            let reg = s.doc.region(rid).ok_or_else(|| CmdError::Params(format!("no region {rid}")))?;
            (reg.mesh, reg.triangles.to_vec(), Some(rid))
        }
        (None, Some(m), Some(t)) => (m.resolve(s, ObjectKind::Mesh)?, t.clone(), None),
        _ => return Err(CmdError::Params("give `region`, or `mesh` and `triangles`".into())),
    };
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    if let Some(t) = tris.iter().find(|&&t| t as usize >= m.triangle_count()) {
        return Err(CmdError::Params(format!("triangle {t} out of range")));
    }
    let normals = m.vertex_normals();
    let (p, n) = region_points(&m, &normals, &tris);
    if p.len() < 3 {
        return Err(CmdError::Params("too few points to fit".into()));
    }
    let kind = match i.kind {
        KindChoice::Auto => None,
        KindChoice::Plane => Some(PrimitiveKind::Plane),
        KindChoice::Sphere => Some(PrimitiveKind::Sphere),
        KindChoice::Cylinder => Some(PrimitiveKind::Cylinder),
        KindChoice::Cone => Some(PrimitiveKind::Cone),
        KindChoice::Torus => Some(PrimitiveKind::Torus),
    };
    let (best, candidates, note) = match kind {
        None => {
            let a = fit_auto(&p, &n).ok_or_else(|| CmdError::Failed("no primitive could be fitted".into()))?;
            (a.best, a.candidates, a.note)
        }
        Some(k) => {
            let f = fit(k, &p, &n).ok_or_else(|| CmdError::Failed(format!("failed to fit a {k:?} to these points")))?;
            (f, Vec::new(), None)
        }
    };
    let warning = (best.rms > 0.05).then(|| {
        format!(
            "RMS {:.4} mm is above {} mm",
            best.rms,
            if best.rms > 0.1 { "the 0.1 mm tolerance" } else { "the 0.05 mm warning level" }
        )
    });
    let patch = best.primitive.patch_mesh(&p, 32);
    let name = i.name.unwrap_or_else(|| format!("{:?}", best.primitive.kind()).to_lowercase());
    let id = s.doc.add_primitive(&name, best.primitive.clone(), region, best.rms, best.max, patch);
    if let Some(rid) = region
        && let Some(r) = s.doc.region_mut(rid)
    {
        // a region a primitive was fitted to is a primitive region, whatever segmentation said
        (r.kind, r.rms, r.freeform) = (Some(best.primitive.kind()), Some(best.rms), false);
    }
    Ok(FitOut {
        id,
        primitive: best.primitive,
        rms: best.rms,
        max: best.max,
        count: best.count,
        candidates,
        note,
        warning,
    })
}

/// One primitive.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PrimIn {
    /// Primitive id or name.
    pub object: ObjRef,
}

/// Primitive facts.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PrimOut {
    /// Id.
    pub id: ObjectId,
    /// The surface.
    pub primitive: Primitive,
    /// RMS (mm).
    pub rms: f64,
    /// Largest deviation (mm).
    pub max: f64,
    /// Region it came from.
    pub region: Option<ObjectId>,
}

fn info(s: &mut Session, i: PrimIn) -> CmdResult<PrimOut> {
    let id = i.object.resolve(s, ObjectKind::Primitive)?;
    let p = s.doc.primitive(id).ok_or_else(|| CmdError::Params(format!("no primitive {id}")))?;
    Ok(PrimOut { id, primitive: p.primitive.clone(), rms: p.rms, max: p.max, region: p.region })
}

/// Symmetry parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SymIn {
    /// Mesh id or name.
    pub mesh: ObjRef,
}

fn symmetry(s: &mut Session, i: SymIn) -> CmdResult<FitOut> {
    let mesh = i.mesh.resolve(s, ObjectKind::Mesh)?;
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let sym = surfacecraft_align::symmetry_plane(&m)?;
    let prim = Primitive::Plane { point: sym.point, normal: sym.normal };
    // display: the plane clipped to the projection of the mesh's bounding box
    let b = m.bbox();
    let corners: Vec<Point3> = (0..8)
        .map(|k| {
            Point3::new(
                if k & 1 == 0 { b.min.x } else { b.max.x },
                if k & 2 == 0 { b.min.y } else { b.max.y },
                if k & 4 == 0 { b.min.z } else { b.max.z },
            )
        })
        .map(|c| prim.project(&c))
        .collect();
    let patch = prim.patch_mesh(&corners, 4);
    let id = s.doc.add_primitive("symmetry plane", prim.clone(), None, sym.rms, sym.rms, patch);
    Ok(FitOut {
        id,
        primitive: prim,
        rms: sym.rms,
        max: sym.rms,
        count: m.vertex_count(),
        candidates: Vec::new(),
        note: None,
        warning: None,
    })
}

/// A geometric relation between two fitted primitives (design doc §6.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum RelationKind {
    /// Normals / axes parallel.
    Parallel,
    /// Normals / axes perpendicular.
    Perpendicular,
    /// Parallel axes on one line.
    Coaxial,
    /// Equal radii (cylinders, spheres).
    EqualRadius,
    /// A plane touches a sphere, a cylinder or a torus tube; two parallel cylinders touch;
    /// a cylinder touches a coaxial torus.
    Tangent,
    /// Two primitives are mirror images of each other about a symmetry plane.
    Symmetric,
    /// Equal distances: three parallel planes, or three parallel axes, equally spaced
    /// (`b` is the middle one, `c` the third).
    EqualDistance,
}

/// One relation found between two primitives.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct Relation {
    /// Kind.
    pub kind: RelationKind,
    /// First primitive.
    pub a: ObjectId,
    /// Second primitive (the middle one of an equal distance).
    pub b: ObjectId,
    /// Third primitive (equal distance only).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub c: Option<ObjectId>,
    /// Deviation before snapping, measured on the primitives as fitted: degrees (parallel,
    /// perpendicular), mm (coaxial, equal radius, tangent: gap between the surfaces, symmetric:
    /// largest position or size mismatch of the mirror image, equal distance: difference of the
    /// two distances).
    pub error: f64,
}

/// A relation that was found but not imposed.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct RejectedRelation {
    /// The relation, with its error before snapping.
    pub relation: Relation,
    /// Why it was not imposed.
    pub reason: String,
}

/// The sigma of one primitive: the RMS of the points of its region about it.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PrimitiveSigma {
    /// Primitive.
    pub id: ObjectId,
    /// Its name.
    pub name: String,
    /// Points of its region used for the RMS (at most 4000, evenly thinned); 0 without a region.
    pub points: usize,
    /// RMS before snapping (mm); absent for a primitive without a region.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rms_before: Option<f64>,
    /// RMS after snapping (mm); with `apply` false the value snapping would give.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rms_after: Option<f64>,
}

/// Auto-constraint parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AutoConstrainIn {
    /// Primitives to relate (default: all).
    #[serde(default)]
    pub objects: Option<Vec<ObjRef>>,
    /// Angular tolerance, degrees (default 3, at most 20).
    #[serde(default)]
    pub angle_tol_deg: Option<f64>,
    /// Length tolerance, mm, for coaxial distance and equal radii (default 0.2).
    #[serde(default)]
    pub length_tol: Option<f64>,
    /// Snap the primitives to satisfy the relations (default true); false only reports them.
    #[serde(default)]
    pub apply: Option<bool>,
    /// Plane primitives (e.g. from `primitive.symmetry`) to take as mirror planes for the
    /// symmetry relations. They are references: never snapped, never related to the others.
    #[serde(default)]
    pub symmetry_planes: Option<Vec<ObjRef>>,
    /// Without `symmetry_planes`: look for mirror planes among the principal planes of the model
    /// (default true). A plane counts only when the whole model is symmetric about it.
    #[serde(default)]
    pub detect_symmetry: Option<bool>,
    /// Sigma of the scan noise, mm (default: the median RMS of the fitted regions). A relation
    /// is rejected when snapping raises the RMS of a primitive above max(0.1 mm, 2 sigma).
    #[serde(default)]
    pub noise_sigma: Option<f64>,
}

/// Auto-constraint result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct AutoConstrainOut {
    /// Relations found within the tolerances and imposed (or imposable, with `apply` false);
    /// `error` is the deviation before snapping.
    pub relations: Vec<Relation>,
    /// Relations found but not imposed, with the reason (typically: snapping would raise the RMS
    /// of a primitive above the limit).
    pub rejected: Vec<RejectedRelation>,
    /// Per primitive: the RMS of its region's points before and after snapping.
    pub primitives: Vec<PrimitiveSigma>,
    /// Sigma of the scan noise used (mm).
    pub sigma: f64,
    /// The RMS no primitive may be raised above by snapping: max(0.1 mm, 2 sigma).
    pub rms_limit: f64,
    /// Were the primitives snapped?
    pub applied: bool,
    /// Largest angular error of the imposed relations (degrees): after snapping, or the current
    /// one when nothing was applied.
    pub max_angle_error_deg: f64,
    /// Largest length error of the imposed coaxial, equal-radius, tangent, symmetric and
    /// equal-distance relations (mm): after snapping, or the current one when nothing was applied.
    pub max_length_error: f64,
}

type V3 = surfacecraft_geom::Vec3;

/// Unit direction of a primitive (normal or axis), if it has one.
fn direction(p: &Primitive) -> Option<V3> {
    match p {
        Primitive::Plane { normal, .. } => Some(V3::from(*normal)),
        Primitive::Cylinder { axis, .. } | Primitive::Cone { axis, .. } | Primitive::Torus { axis, .. } => {
            Some(V3::from(*axis))
        }
        Primitive::Sphere { .. } => None,
    }
    .and_then(surfacecraft_geom::normalize)
}

/// A point on the axis of an axial primitive.
fn axis_point(p: &Primitive) -> Option<V3> {
    match p {
        Primitive::Cylinder { point, .. } => Some(V3::from(*point)),
        Primitive::Cone { apex, .. } => Some(V3::from(*apex)),
        Primitive::Torus { center, .. } => Some(V3::from(*center)),
        _ => None,
    }
}

fn radius_of(p: &Primitive) -> Option<(u8, f64)> {
    match p {
        Primitive::Cylinder { radius, .. } => Some((0, *radius)),
        Primitive::Sphere { radius, .. } => Some((1, *radius)),
        _ => None,
    }
}

fn set_direction(p: &mut Primitive, d: V3) {
    match p {
        Primitive::Plane { normal, .. } => *normal = d.into(),
        Primitive::Cylinder { axis, .. } | Primitive::Cone { axis, .. } | Primitive::Torus { axis, .. } => {
            *axis = d.into();
        }
        Primitive::Sphere { .. } => {}
    }
}

fn set_axis_point(p: &mut Primitive, q: V3) {
    match p {
        Primitive::Cylinder { point, .. } => *point = q.into(),
        Primitive::Cone { apex, .. } => *apex = q.into(),
        Primitive::Torus { center, .. } => *center = q.into(),
        _ => {}
    }
}

fn set_radius(p: &mut Primitive, r: f64) {
    match p {
        Primitive::Cylinder { radius, .. } | Primitive::Sphere { radius, .. } => *radius = r,
        _ => {}
    }
}

fn find(parent: &mut [usize], i: usize) -> usize {
    let mut r = i;
    while parent.get(r).copied().unwrap_or(r) != r {
        r = parent.get(r).copied().unwrap_or(r);
    }
    // path compression
    let mut c = i;
    while parent.get(c).copied().unwrap_or(c) != c {
        let next = parent.get(c).copied().unwrap_or(c);
        if let Some(p) = parent.get_mut(c) {
            *p = r;
        }
        c = next;
    }
    r
}

fn union(parent: &mut [usize], a: usize, b: usize) {
    let (ra, rb) = (find(parent, a), find(parent, b));
    if ra != rb {
        let (lo, hi) = (ra.min(rb), ra.max(rb));
        if let Some(p) = parent.get_mut(hi) {
            *p = lo;
        }
    }
}

/// Relations of a primitive list within the tolerances (`angle_tol` in degrees).
fn find_relations(prims: &[(ObjectId, Primitive)], angle_tol: f64, length_tol: f64) -> Vec<(usize, usize, Relation)> {
    let mut out = Vec::new();
    for (i, (ida, pa)) in prims.iter().enumerate() {
        for (j, (idb, pb)) in prims.iter().enumerate().skip(i + 1) {
            let rel = |kind, error| Relation { kind, a: *ida, b: *idb, c: None, error };
            if let (Some(da), Some(db)) = (direction(pa), direction(pb)) {
                let theta = da.dot(&db).abs().clamp(0.0, 1.0).acos().to_degrees();
                if theta <= angle_tol {
                    out.push((i, j, rel(RelationKind::Parallel, theta)));
                    if let (Some(oa), Some(ob)) = (axis_point(pa), axis_point(pb)) {
                        let w = ob - oa;
                        let off = (w - da * w.dot(&da)).norm();
                        if off <= length_tol {
                            out.push((i, j, rel(RelationKind::Coaxial, off)));
                        }
                    }
                } else if 90.0 - theta <= angle_tol {
                    out.push((i, j, rel(RelationKind::Perpendicular, 90.0 - theta)));
                }
            }
            if let (Some((ka, ra)), Some((kb, rb))) = (radius_of(pa), radius_of(pb))
                && ka == kb
                && (ra - rb).abs() <= length_tol
            {
                out.push((i, j, rel(RelationKind::EqualRadius, (ra - rb).abs())));
            }
        }
    }
    out
}

/// Snaps the primitives so that every found relation holds exactly.
fn snap(prims: &mut [(ObjectId, Primitive)], rels: &[(usize, usize, Relation)]) {
    let n = prims.len();
    // 1. directions: parallel classes get their mean, perpendicular classes are orthogonalised
    let mut parent: Vec<usize> = (0..n).collect();
    for (i, j, r) in rels {
        if r.kind == RelationKind::Parallel {
            union(&mut parent, *i, *j);
        }
    }
    let mut classes: Vec<(usize, Vec<usize>)> = Vec::new();
    for i in 0..n {
        if prims.get(i).and_then(|p| direction(&p.1)).is_none() {
            continue;
        }
        let r = find(&mut parent, i);
        match classes.iter_mut().find(|c| c.0 == r) {
            Some(c) => c.1.push(i),
            None => classes.push((r, vec![i])),
        }
    }
    // largest class first, ties by lowest member
    classes.sort_by(|a, b| b.1.len().cmp(&a.1.len()).then(a.1.first().cmp(&b.1.first())));
    let class_of = |i: usize, classes: &[(usize, Vec<usize>)]| classes.iter().position(|c| c.1.contains(&i));
    let mut fixed: Vec<(usize, V3)> = Vec::new(); // class index → direction
    for (ci, (_, members)) in classes.iter().enumerate() {
        let Some(first) = members.first().and_then(|&m| prims.get(m)).and_then(|p| direction(&p.1)) else { continue };
        let mut mean = V3::zeros();
        for &m in members {
            if let Some(d) = prims.get(m).and_then(|p| direction(&p.1)) {
                mean += if d.dot(&first) < 0.0 { -d } else { d };
            }
        }
        let mut d = surfacecraft_geom::normalize(mean).unwrap_or(first);
        for (fc, fd) in &fixed {
            let perpendicular = rels.iter().any(|(i, j, r)| {
                r.kind == RelationKind::Perpendicular
                    && ((class_of(*i, &classes) == Some(ci) && class_of(*j, &classes) == Some(*fc))
                        || (class_of(*j, &classes) == Some(ci) && class_of(*i, &classes) == Some(*fc)))
            });
            if perpendicular {
                d = surfacecraft_geom::normalize(d - fd * d.dot(fd)).unwrap_or(d);
            }
        }
        fixed.push((ci, d));
        for &m in members {
            if let Some(p) = prims.get_mut(m)
                && let Some(orig) = direction(&p.1)
            {
                set_direction(&mut p.1, if orig.dot(&d) < 0.0 { -d } else { d });
            }
        }
    }
    // 2. coaxial groups share one axis line (the mean of their perpendicular offsets)
    let mut parent: Vec<usize> = (0..n).collect();
    for (i, j, r) in rels {
        if r.kind == RelationKind::Coaxial {
            union(&mut parent, *i, *j);
        }
    }
    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for i in 0..n {
        if prims.get(i).and_then(|p| axis_point(&p.1)).is_none() {
            continue;
        }
        let r = find(&mut parent, i);
        match groups.iter_mut().find(|g| g.0 == r) {
            Some(g) => g.1.push(i),
            None => groups.push((r, vec![i])),
        }
    }
    for (_, members) in groups.iter().filter(|g| g.1.len() > 1) {
        let Some((o, d)) =
            members.first().and_then(|&m| prims.get(m)).and_then(|p| axis_point(&p.1).zip(direction(&p.1)))
        else {
            continue;
        };
        let offs: Vec<(usize, V3)> = members
            .iter()
            .filter_map(|&m| {
                let q = prims.get(m).and_then(|p| axis_point(&p.1))?;
                let w = q - o;
                Some((m, w - d * w.dot(&d)))
            })
            .collect();
        let mean = offs.iter().fold(V3::zeros(), |a, (_, w)| a + w) / offs.len().max(1) as f64;
        for (m, w) in offs {
            if let Some(p) = prims.get_mut(m)
                && let Some(q) = axis_point(&p.1)
            {
                set_axis_point(&mut p.1, q - w + mean);
            }
        }
    }
    // 3. equal radii share their mean
    let mut parent: Vec<usize> = (0..n).collect();
    for (i, j, r) in rels {
        if r.kind == RelationKind::EqualRadius {
            union(&mut parent, *i, *j);
        }
    }
    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for i in 0..n {
        if prims.get(i).and_then(|p| radius_of(&p.1)).is_none() {
            continue;
        }
        let r = find(&mut parent, i);
        match groups.iter_mut().find(|g| g.0 == r) {
            Some(g) => g.1.push(i),
            None => groups.push((r, vec![i])),
        }
    }
    for (_, members) in groups.iter().filter(|g| g.1.len() > 1) {
        let radii: Vec<f64> =
            members.iter().filter_map(|&m| prims.get(m).and_then(|p| radius_of(&p.1)).map(|r| r.1)).collect();
        let mean = radii.iter().sum::<f64>() / radii.len().max(1) as f64;
        for &m in members {
            if let Some(p) = prims.get_mut(m) {
                set_radius(&mut p.1, mean);
            }
        }
    }
}

/// Residual of an angular / coaxial / equal-radius relation on two primitives: (degrees, mm).
fn old_residual(kind: RelationKind, pa: &Primitive, pb: &Primitive) -> (f64, f64) {
    let (mut angle, mut len) = (0.0_f64, 0.0_f64);
    match kind {
        RelationKind::Parallel | RelationKind::Perpendicular | RelationKind::Coaxial => {
            if let (Some(da), Some(db)) = (direction(pa), direction(pb)) {
                let theta = da.dot(&db).abs().clamp(0.0, 1.0).acos().to_degrees();
                angle = if kind == RelationKind::Perpendicular { (90.0 - theta).abs() } else { theta };
                if kind == RelationKind::Coaxial
                    && let (Some(oa), Some(ob)) = (axis_point(pa), axis_point(pb))
                {
                    let w = ob - oa;
                    len = (w - da * w.dot(&da)).norm();
                }
            }
        }
        RelationKind::EqualRadius => {
            if let (Some((_, ra)), Some((_, rb))) = (radius_of(pa), radius_of(pb)) {
                len = (ra - rb).abs();
            }
        }
        RelationKind::Tangent | RelationKind::Symmetric | RelationKind::EqualDistance => {}
    }
    (angle, len)
}

/// The most points of a region used for an RMS.
const MAX_RMS_POINTS: usize = 4000;
/// The RMS no primitive may be raised above by snapping is at least this (mm, design doc §11.1).
const RMS_FLOOR: f64 = 0.1;

/// Vertices of the region a primitive was fitted to, evenly thinned to `MAX_RMS_POINTS`.
fn region_sample(s: &Session, id: ObjectId) -> Vec<Point3> {
    let Some(reg) = s.doc.primitive(id).and_then(|p| p.region).and_then(|r| s.doc.region(r)) else {
        return Vec::new();
    };
    let Some(mesh) = s.doc.mesh(reg.mesh) else { return Vec::new() };
    let mut ids: Vec<u32> = reg
        .triangles
        .iter()
        .filter_map(|&t| mesh.mesh.triangles.get(t as usize))
        .flat_map(|t| t.iter().copied())
        .collect();
    ids.sort_unstable();
    ids.dedup();
    let step = ids.len() / MAX_RMS_POINTS + 1;
    ids.iter().step_by(step).filter_map(|&v| mesh.mesh.positions.get(v as usize).copied()).collect()
}

/// RMS and largest deviation of `pts` from the primitive; none without points.
fn rms_of(p: &Primitive, pts: &[Point3]) -> Option<(f64, f64)> {
    if pts.is_empty() {
        return None;
    }
    let (mut sum, mut max) = (0.0_f64, 0.0_f64);
    for q in pts {
        let d = p.distance(q);
        sum += d * d;
        max = max.max(d.abs());
    }
    let rms = (sum / pts.len() as f64).sqrt();
    (rms.is_finite() && max.is_finite()).then_some((rms, max))
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(|a, b| a.total_cmp(b));
    v.get(v.len() / 2).copied().unwrap_or(0.0)
}

/// The RMS check: snapping must not raise the RMS of a primitive above `limit`.
struct Gate<'a> {
    pts: &'a [Vec<Point3>],
    before: &'a [Option<f64>],
    names: &'a [String],
    limit: f64,
}

impl Gate<'_> {
    fn after(&self, idx: usize, p: &Primitive) -> Option<f64> {
        rms_of(p, self.pts.get(idx)?).map(|v| v.0)
    }

    /// A reason when primitive `idx`, as `p`, has been pushed above the limit by snapping.
    fn broken(&self, idx: usize, p: &Primitive) -> Option<String> {
        let after = self.after(idx, p)?;
        let before = (*self.before.get(idx)?)?;
        (after > self.limit && after > before + 1e-9).then(|| {
            format!(
                "snapping would raise the RMS of `{}` from {before:.4} mm to {after:.4} mm, above the limit of {:.4} mm",
                self.names.get(idx).map_or("?", String::as_str),
                self.limit
            )
        })
    }
}

/// A relation under consideration: for the output, the snapping and the RMS check.
struct Cand {
    rel: Relation,
    i: usize,
    j: usize,
    k: Option<usize>,
    /// The relation for the position stage (tangent, symmetric and equal distance only).
    crel: Option<constrain::Rel>,
    /// Largest RMS of its primitives after snapping this relation alone (mm).
    effect: f64,
    /// Why it was rejected.
    verdict: Option<String>,
}

impl Cand {
    fn live(&self) -> bool {
        self.verdict.is_none()
    }

    fn is_new(&self) -> bool {
        self.crel.is_some()
    }

    fn touches(&self, p: usize) -> bool {
        self.i == p || self.j == p || self.k == Some(p)
    }
}

fn live_old(cands: &[Cand]) -> Vec<(usize, usize, Relation)> {
    cands.iter().filter(|c| c.live() && !c.is_new()).map(|c| (c.i, c.j, c.rel.clone())).collect()
}

/// The coaxial and equal-radius relations still standing, as chains from the first member of each
/// group, to keep them holding while the position stage runs.
fn carried(cands: &[Cand], n: usize) -> Vec<constrain::Rel> {
    let mut out = Vec::new();
    for (kind, ck) in [
        (RelationKind::Coaxial, constrain::RelKind::Coaxial),
        (RelationKind::EqualRadius, constrain::RelKind::EqualRadius),
    ] {
        let live: Vec<&Cand> = cands.iter().filter(|c| c.live() && c.rel.kind == kind).collect();
        let mut parent: Vec<usize> = (0..n).collect();
        for c in &live {
            union(&mut parent, c.i, c.j);
        }
        let mut members: Vec<usize> = live.iter().flat_map(|c| [c.i, c.j]).collect();
        members.sort_unstable();
        members.dedup();
        for m in members {
            let root = find(&mut parent, m);
            if root != m {
                out.push(constrain::Rel { kind: ck, a: root, b: m, c: None, mirror: None });
            }
        }
    }
    out
}

/// The primitives after snapping every live relation: directions and radii first (the angular,
/// coaxial and equal-radius relations), then the position stage for the new relations. An error
/// carries the message and the candidate to blame.
fn simulate(
    prims: &[(ObjectId, Primitive)],
    cands: &[Cand],
    mirrors: &[constrain::Mirror],
) -> Result<Vec<Primitive>, (String, Option<usize>)> {
    let mut out = prims.to_vec();
    snap(&mut out, &live_old(cands));
    let mut ps: Vec<Primitive> = out.into_iter().map(|p| p.1).collect();
    let mut rels: Vec<constrain::Rel> = Vec::new();
    let mut owner: Vec<Option<usize>> = Vec::new();
    for (ci, c) in cands.iter().enumerate().filter(|(_, c)| c.live()) {
        if let Some(r) = c.crel {
            rels.push(r);
            owner.push(Some(ci));
        }
    }
    if rels.is_empty() {
        return Ok(ps);
    }
    for r in carried(cands, ps.len()) {
        rels.push(r);
        owner.push(None);
    }
    constrain::apply(&mut ps, mirrors, &rels)
        .map_err(|f| (f.message, f.worst.and_then(|w| owner.get(w).copied().flatten())))?;
    Ok(ps)
}

pub(crate) fn auto_constrain(s: &mut Session, i: AutoConstrainIn) -> CmdResult<AutoConstrainOut> {
    let angle_tol = i.angle_tol_deg.unwrap_or(3.0);
    let length_tol = i.length_tol.unwrap_or(0.2);
    if !(angle_tol.is_finite() && (0.0..=20.0).contains(&angle_tol)) {
        return Err(CmdError::Params("angle_tol_deg must be within 0..=20".into()));
    }
    if !(length_tol.is_finite() && length_tol >= 0.0) {
        return Err(CmdError::Params("length_tol must be ≥ 0".into()));
    }
    if let Some(v) = i.noise_sigma
        && !(v.is_finite() && v >= 0.0)
    {
        return Err(CmdError::Params("noise_sigma must be ≥ 0".into()));
    }
    // mirror planes given by the caller: references, not parts
    let mut given: Vec<(ObjectId, constrain::Mirror)> = Vec::new();
    for r in i.symmetry_planes.iter().flatten() {
        let id = r.resolve(s, ObjectKind::Primitive)?;
        let p = s.doc.primitive(id).ok_or_else(|| CmdError::Params(format!("no primitive {id}")))?;
        let Primitive::Plane { point, normal } = &p.primitive else {
            return Err(CmdError::Params(format!("symmetry plane {id} is not a plane")));
        };
        let m = constrain::Mirror::new(V3::from(*point), V3::from(*normal))
            .ok_or_else(|| CmdError::Params(format!("symmetry plane {id} has no normal")))?;
        given.push((id, m));
    }
    let ids: Vec<ObjectId> = match &i.objects {
        Some(list) => list.iter().map(|r| r.resolve(s, ObjectKind::Primitive)).collect::<CmdResult<_>>()?,
        None => s.doc.primitives.iter().map(|p| p.id).collect(),
    };
    let mut prims: Vec<(ObjectId, Primitive)> = Vec::new();
    let mut names: Vec<String> = Vec::new();
    for id in ids {
        if prims.iter().any(|p| p.0 == id) || given.iter().any(|g| g.0 == id) {
            continue;
        }
        let p = s.doc.primitive(id).ok_or_else(|| CmdError::Params(format!("no primitive {id}")))?;
        prims.push((id, p.primitive.clone()));
        names.push(p.name.clone());
    }
    if prims.len() < 2 {
        return Err(CmdError::Params("auto-constrain needs at least two primitives".into()));
    }
    let apply = i.apply.unwrap_or(true);

    // sigma before: the RMS of each primitive's region, and the noise it implies
    let pts: Vec<Vec<Point3>> = prims.iter().map(|(id, _)| region_sample(s, *id)).collect();
    let before: Vec<Option<f64>> = prims.iter().zip(&pts).map(|((_, p), q)| rms_of(p, q).map(|v| v.0)).collect();
    let sigma = match i.noise_sigma {
        Some(v) => v,
        None => {
            let fitted: Vec<f64> = before.iter().flatten().copied().collect();
            if fitted.is_empty() {
                median(
                    prims
                        .iter()
                        .filter_map(|(id, _)| s.doc.primitive(*id).map(|p| p.rms))
                        .filter(|r| r.is_finite())
                        .collect(),
                )
            } else {
                median(fitted)
            }
        }
    };
    let limit = RMS_FLOOR.max(2.0 * sigma);
    let gate = Gate { pts: &pts, before: &before, names: &names, limit };

    // angular, coaxial and equal-radius relations, each checked alone against the RMS limit
    let mut cands: Vec<Cand> = find_relations(&prims, angle_tol, length_tol)
        .into_iter()
        .map(|(a, b, rel)| Cand { rel, i: a, j: b, k: None, crel: None, effect: 0.0, verdict: None })
        .collect();
    for c in &mut cands {
        let (Some(pa), Some(pb)) = (prims.get(c.i), prims.get(c.j)) else { continue };
        let mut local = vec![pa.clone(), pb.clone()];
        snap(&mut local, &[(0, 1, c.rel.clone())]);
        for (slot, idx) in [(0_usize, c.i), (1, c.j)] {
            if let Some((_, p)) = local.get(slot) {
                c.effect = c.effect.max(gate.after(idx, p).unwrap_or(0.0));
                if c.verdict.is_none() {
                    c.verdict = gate.broken(idx, p);
                }
            }
        }
    }

    // tangency, symmetry and equal distances, found on the primitives with their directions snapped
    let raw: Vec<Primitive> = prims.iter().map(|p| p.1.clone()).collect();
    let mut s1 = prims.clone();
    snap(&mut s1, &live_old(&cands));
    let s1p: Vec<Primitive> = s1.into_iter().map(|p| p.1).collect();
    let mut mirrors: Vec<constrain::Mirror> = given.iter().map(|g| g.1).collect();
    if given.is_empty() && i.detect_symmetry.unwrap_or(true) {
        let all: Vec<Point3> = pts.iter().flatten().copied().collect();
        mirrors.extend(constrain::detect_mirrors(&s1p, &all, angle_tol, length_tol));
    }
    let id_of = |k: usize| prims.get(k).map(|p| p.0);
    for r in constrain::find_relations(&s1p, &mirrors, angle_tol, length_tol) {
        let kind = match r.kind {
            constrain::RelKind::Tangent => RelationKind::Tangent,
            constrain::RelKind::Symmetric => RelationKind::Symmetric,
            constrain::RelKind::EqualDistance => RelationKind::EqualDistance,
            constrain::RelKind::Coaxial | constrain::RelKind::EqualRadius => continue,
        };
        let (Some(a), Some(b)) = (id_of(r.a), id_of(r.b)) else { continue };
        let error = constrain::measure(&raw, &mirrors, &r).map_or(0.0, |m| m.0);
        cands.push(Cand {
            rel: Relation { kind, a, b, c: r.c.and_then(id_of), error },
            i: r.a,
            j: r.b,
            k: r.c,
            crel: Some(r),
            effect: 0.0,
            verdict: None,
        });
    }
    for c in cands.iter_mut().filter(|c| c.is_new()) {
        let Some(r) = c.crel else { continue };
        let idxs: Vec<usize> = [c.i, c.j].into_iter().chain(c.k).collect();
        let mut local: Vec<Primitive> = idxs.iter().filter_map(|&x| s1p.get(x).cloned()).collect();
        let lone = constrain::Rel { a: 0, b: 1, c: c.k.map(|_| 2), ..r };
        match constrain::apply(&mut local, &mirrors, &[lone]) {
            Err(f) => c.verdict = Some(f.message),
            Ok(()) => {
                for (slot, &idx) in idxs.iter().enumerate() {
                    if let Some(p) = local.get(slot) {
                        c.effect = c.effect.max(gate.after(idx, p).unwrap_or(0.0));
                        if c.verdict.is_none() {
                            c.verdict = gate.broken(idx, p);
                        }
                    }
                }
            }
        }
    }

    // all together: while snapping everything raises a primitive above the limit (or the
    // relations conflict), drop the relation that matters most for it, and try again
    let mut snapped: Option<Vec<Primitive>> = None;
    for _ in 0..64 {
        match simulate(&prims, &cands, &mirrors) {
            Err((msg, culprit)) => {
                let Some(k) = culprit.or_else(|| cands.iter().rposition(|c| c.live() && c.is_new())) else {
                    break;
                };
                if let Some(c) = cands.get_mut(k) {
                    c.verdict = Some(msg);
                }
            }
            Ok(ps) => {
                let mut worst: Option<(usize, f64, String)> = None;
                for (x, p) in ps.iter().enumerate() {
                    if let Some(msg) = gate.broken(x, p) {
                        let a = gate.after(x, p).unwrap_or(0.0);
                        if worst.as_ref().is_none_or(|w| a > w.1) {
                            worst = Some((x, a, msg));
                        }
                    }
                }
                let Some((x, _, msg)) = worst else {
                    snapped = Some(ps);
                    break;
                };
                let blame = cands
                    .iter()
                    .enumerate()
                    .filter(|(_, c)| c.live() && c.touches(x))
                    .max_by(|a, b| a.1.effect.total_cmp(&b.1.effect))
                    .map(|(k, _)| k);
                let Some(c) = blame.and_then(|k| cands.get_mut(k)) else { break };
                c.verdict = Some(format!("together with the other relations: {msg}"));
            }
        }
    }
    let snapped = match snapped {
        Some(ps) => ps,
        None => {
            for c in cands.iter_mut().filter(|c| c.live()) {
                c.verdict = Some("the relations could not be imposed together within the RMS limit".into());
            }
            raw.clone()
        }
    };

    if apply {
        for ((id, _), new) in prims.iter().zip(&snapped) {
            if let Some(obj) = s.doc.primitives.iter_mut().find(|p| p.id == *id)
                && obj.primitive != *new
            {
                let mut patch = (*obj.patch).clone();
                for q in &mut patch.positions {
                    *q = new.project(q);
                }
                obj.patch = std::sync::Arc::new(patch);
                obj.primitive = new.clone();
            }
        }
    }

    // residuals of the relations that stand, on the primitives as they are now
    let shown: &[Primitive] = if apply { &snapped } else { &raw };
    let (mut max_angle, mut max_len) = (0.0_f64, 0.0_f64);
    for c in cands.iter().filter(|c| c.live()) {
        if let Some(r) = c.crel {
            if let Some((len, angle)) = constrain::measure(shown, &mirrors, &r) {
                max_len = max_len.max(len);
                max_angle = max_angle.max(angle);
            }
        } else if let (Some(pa), Some(pb)) = (shown.get(c.i), shown.get(c.j)) {
            let (angle, len) = old_residual(c.rel.kind, pa, pb);
            max_angle = max_angle.max(angle);
            max_len = max_len.max(len);
        }
    }

    let primitives: Vec<PrimitiveSigma> = prims
        .iter()
        .enumerate()
        .map(|(x, (id, _))| PrimitiveSigma {
            id: *id,
            name: names.get(x).cloned().unwrap_or_default(),
            points: pts.get(x).map_or(0, Vec::len),
            rms_before: before.get(x).copied().flatten(),
            rms_after: snapped.get(x).zip(pts.get(x)).and_then(|(p, q)| rms_of(p, q)).map(|v| v.0),
        })
        .collect();
    let mut relations: Vec<Relation> = cands.iter().filter(|c| c.live()).map(|c| c.rel.clone()).collect();
    relations.truncate(5000);
    let mut rejected: Vec<RejectedRelation> = cands
        .iter()
        .filter_map(|c| {
            c.verdict.as_ref().map(|reason| RejectedRelation { relation: c.rel.clone(), reason: reason.clone() })
        })
        .collect();
    rejected.truncate(5000);
    Ok(AutoConstrainOut {
        relations,
        rejected,
        primitives,
        sigma,
        rms_limit: limit,
        applied: apply,
        max_angle_error_deg: max_angle,
        max_length_error: max_len,
    })
}

/// `primitive.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "primitive.auto_constrain",
        "Auto-constrain",
        "Primitive",
        true,
        "Finds parallel/perpendicular/coaxial/equal-radius/tangent/symmetric/equal-distance relations between fitted primitives within tolerances and snaps them, unless snapping would raise a primitive RMS above max(0.1 mm, 2 sigma); reports sigma before and after (design doc §6.2).",
        AutoConstrainIn,
        AutoConstrainOut,
        auto_constrain
    ),
    command!(
        "primitive.fit",
        "Fit primitive",
        "Primitive",
        true,
        "Fits a plane/sphere/cylinder/cone/torus (`kind`, default auto) to `region` or `mesh`+`triangles`.",
        FitIn,
        FitOut,
        fit_cmd
    ),
    command!(
        "primitive.info",
        "Primitive info",
        "Primitive",
        false,
        "Parameters and fit quality of `object`.",
        PrimIn,
        PrimOut,
        info
    ),
    command!(
        "primitive.symmetry",
        "Symmetry plane",
        "Primitive",
        true,
        "Fits the plane minimising the distance between `mesh` and its mirror image.",
        SymIn,
        FitOut,
        symmetry
    ),
];

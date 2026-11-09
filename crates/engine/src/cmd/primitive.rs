//! `primitive.*`: analytic primitives fitted to regions or triangles, and the symmetry plane.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::segment::region_points;
use surfacecraft_fit::{Primitive, PrimitiveKind, fit_auto, primitive::fit};
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

fn fit_cmd(s: &mut Session, i: FitIn) -> CmdResult<FitOut> {
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
        r.kind = Some(best.primitive.kind());
        r.rms = Some(best.rms);
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
}

/// One relation found between two primitives.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct Relation {
    /// Kind.
    pub kind: RelationKind,
    /// First primitive.
    pub a: ObjectId,
    /// Second primitive.
    pub b: ObjectId,
    /// Deviation before snapping: degrees (parallel/perpendicular), mm (coaxial, equal radius).
    pub error: f64,
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
}

/// Auto-constraint result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct AutoConstrainOut {
    /// Relations found (before snapping).
    pub relations: Vec<Relation>,
    /// Were the primitives snapped?
    pub applied: bool,
    /// Largest angular error of the angular relations after snapping (degrees).
    pub max_angle_error_deg: f64,
    /// Largest length error of coaxial and equal-radius relations after snapping (mm).
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
            let rel = |kind, error| Relation { kind, a: *ida, b: *idb, error };
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

fn auto_constrain(s: &mut Session, i: AutoConstrainIn) -> CmdResult<AutoConstrainOut> {
    let angle_tol = i.angle_tol_deg.unwrap_or(3.0);
    let length_tol = i.length_tol.unwrap_or(0.2);
    if !(angle_tol.is_finite() && (0.0..=20.0).contains(&angle_tol)) {
        return Err(CmdError::Params("angle_tol_deg must be within 0..=20".into()));
    }
    if !(length_tol.is_finite() && length_tol >= 0.0) {
        return Err(CmdError::Params("length_tol must be ≥ 0".into()));
    }
    let ids: Vec<ObjectId> = match &i.objects {
        Some(list) => list.iter().map(|r| r.resolve(s, ObjectKind::Primitive)).collect::<CmdResult<_>>()?,
        None => s.doc.primitives.iter().map(|p| p.id).collect(),
    };
    let mut prims: Vec<(ObjectId, Primitive)> = Vec::new();
    for id in ids {
        if prims.iter().any(|p| p.0 == id) {
            continue;
        }
        let p = s.doc.primitive(id).ok_or_else(|| CmdError::Params(format!("no primitive {id}")))?;
        prims.push((id, p.primitive.clone()));
    }
    if prims.len() < 2 {
        return Err(CmdError::Params("auto-constrain needs at least two primitives".into()));
    }
    let rels = find_relations(&prims, angle_tol, length_tol);
    let apply = i.apply.unwrap_or(true);
    let mut after = prims.clone();
    if apply {
        snap(&mut after, &rels);
        for (id, new) in &after {
            if let Some(obj) = s.doc.primitives.iter_mut().find(|p| p.id == *id) {
                let mut patch = (*obj.patch).clone();
                for q in &mut patch.positions {
                    *q = new.project(q);
                }
                obj.patch = std::sync::Arc::new(patch);
                obj.primitive = new.clone();
            }
        }
    }
    // residuals of the relations that were found, measured on the primitives as they are now
    let (mut max_angle, mut max_len) = (0.0_f64, 0.0_f64);
    for (i, j, r) in &rels {
        let (Some((_, pa)), Some((_, pb))) = (after.get(*i), after.get(*j)) else { continue };
        match r.kind {
            RelationKind::Parallel | RelationKind::Perpendicular | RelationKind::Coaxial => {
                if let (Some(da), Some(db)) = (direction(pa), direction(pb)) {
                    let c = da.dot(&db).abs().clamp(0.0, 1.0);
                    let theta = c.acos().to_degrees();
                    let err = if r.kind == RelationKind::Perpendicular { (90.0 - theta).abs() } else { theta };
                    max_angle = max_angle.max(err);
                    if r.kind == RelationKind::Coaxial
                        && let (Some(oa), Some(ob)) = (axis_point(pa), axis_point(pb))
                    {
                        let w = ob - oa;
                        max_len = max_len.max((w - da * w.dot(&da)).norm());
                    }
                }
            }
            RelationKind::EqualRadius => {
                if let (Some((_, ra)), Some((_, rb))) = (radius_of(pa), radius_of(pb)) {
                    max_len = max_len.max((ra - rb).abs());
                }
            }
        }
    }
    let mut relations: Vec<Relation> = rels.into_iter().map(|r| r.2).collect();
    relations.truncate(5000);
    Ok(AutoConstrainOut { relations, applied: apply, max_angle_error_deg: max_angle, max_length_error: max_len })
}

/// `primitive.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "primitive.auto_constrain",
        "Auto-constrain",
        "Primitive",
        true,
        "Finds parallel/perpendicular/coaxial/equal-radius relations between fitted primitives within tolerances and snaps them (design doc §6.2).",
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

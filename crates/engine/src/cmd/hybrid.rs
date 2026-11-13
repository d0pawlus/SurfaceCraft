//! `body.from_regions`, `body.replace_face`, `body.extract_face`, `surface.trim_to_region`,
//! `primitive.trim_to_region`: the hybrid model (design doc §6.3, plan P3). Fitted primitives
//! and freeform patches become one closed solid by trimming a stock (the kernel does the
//! B-rep work, see `surfacecraft_kernel::hybrid`); the faces of that solid are the primitives
//! and patches trimmed to the contours where they meet.
use super::body::{BodyOut, body_out};
use super::plan::{self, Item, PlanOut, RoleKind};
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::Primitive;
use surfacecraft_fit::resample::HeightField;
use surfacecraft_fit::segment::region_points;
use surfacecraft_geom::{Point3, Vec3, normalize};
use surfacecraft_kernel::hybrid::{self, Cut, CutStatus, FreeCut, Shape};
use surfacecraft_kernel::local::{LocalAdd, LocalTool};
use surfacecraft_kernel::nurbs::fit_grid;
use surfacecraft_kernel::record::brep::Surface;

type Cloud = (Vec<Point3>, Vec<Vec3>);

/// Points and normals of the region `rid`, normals computed once per mesh.
fn region_cloud(s: &Session, rid: ObjectId, cache: &mut HashMap<ObjectId, Arc<Vec<Vec3>>>) -> CmdResult<Cloud> {
    let reg = s.doc.region(rid).ok_or_else(|| CmdError::Params(format!("no region {rid}")))?;
    let m = s.doc.mesh(reg.mesh).ok_or_else(|| CmdError::Params(format!("no mesh {}", reg.mesh)))?;
    let normals = cache.entry(reg.mesh).or_insert_with(|| Arc::new(m.mesh.vertex_normals())).clone();
    Ok(region_points(&m.mesh, &normals, &reg.triangles))
}

/// Largest gap between sorted angles around the circle (radians).
fn max_gap(mut angles: Vec<f64>) -> f64 {
    if angles.len() < 2 {
        return std::f64::consts::TAU;
    }
    angles.sort_by(f64::total_cmp);
    let mut gap = angles[0] + std::f64::consts::TAU - angles[angles.len() - 1];
    for w in angles.windows(2) {
        gap = gap.max(w[1] - w[0]);
    }
    gap
}

fn v3(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

fn arr(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

/// Angles of the points around `axis` (in the frame of any perpendicular).
fn angles_about(pts: &[Point3], origin: Vec3, axis: Vec3) -> Vec<f64> {
    let x = surfacecraft_geom::any_perpendicular(&axis);
    let y = axis.cross(&x);
    pts.iter()
        .map(|p| {
            let d = p.coords - origin;
            d.dot(&y).atan2(d.dot(&x))
        })
        .collect()
}

/// A [`Cut`] from a fitted primitive and its region's points: which side the material is
/// (from the scan normals) and whether the region goes all the way round.
pub(super) fn cut_of(p: &Primitive, pts: &[Point3], nrm: &[Vec3]) -> Result<Cut, String> {
    if pts.is_empty() || pts.len() != nrm.len() {
        return Err("the region has no points".into());
    }
    // mean of n · (outward direction of the shape at the point): > 0 when the normals point away
    // from the shape's axis or centre, i.e. the material is inside it
    let outwardness = |dir: &dyn Fn(&Point3) -> Option<Vec3>| -> f64 {
        let mut sum = 0.0;
        let mut n = 0.0;
        for (q, nn) in pts.iter().zip(nrm) {
            if let Some(d) = dir(q) {
                sum += nn.dot(&d);
                n += 1.0;
            }
        }
        if n > 0.0 { sum / n } else { 0.0 }
    };
    let radial = |origin: Vec3, axis: Vec3| {
        move |q: &Point3| {
            let d = q.coords - origin;
            normalize(d - axis * d.dot(&axis))
        }
    };
    match p {
        Primitive::Plane { point, normal } => {
            let n = normalize(v3(*normal)).ok_or("degenerate plane")?;
            let mean: Vec3 = nrm.iter().sum();
            let n = if mean.dot(&n) < 0.0 { -n } else { n };
            Ok(Cut { shape: Shape::Plane { point: *point, normal: arr(n) }, convex: true, full: false })
        }
        Primitive::Cylinder { point, axis, radius } => {
            let (o, a) = (v3(*point), normalize(v3(*axis)).ok_or("degenerate axis")?);
            let convex = outwardness(&radial(o, a)) > 0.0;
            let full = max_gap(angles_about(pts, o, a)) < 1.0;
            Ok(Cut { shape: Shape::Cylinder { point: *point, axis: arr(a), radius: *radius }, convex, full })
        }
        Primitive::Cone { apex, axis, half_angle } => {
            let (o, a) = (v3(*apex), normalize(v3(*axis)).ok_or("degenerate axis")?);
            let convex = outwardness(&radial(o, a)) > 0.0;
            let full = max_gap(angles_about(pts, o, a)) < 1.0;
            Ok(Cut { shape: Shape::Cone { apex: *apex, axis: arr(a), half_angle: *half_angle }, convex, full })
        }
        Primitive::Sphere { center, radius } => {
            let c = v3(*center);
            let dirs: Vec<Vec3> = pts.iter().filter_map(|q| normalize(q.coords - c)).collect();
            let mean: Vec3 = dirs.iter().sum::<Vec3>() / dirs.len().max(1) as f64;
            // a sphere that is not nearly closed would need a boundary we do not have
            let full = mean.norm() < 0.3;
            let convex = outwardness(&|q: &Point3| normalize(q.coords - c)) > 0.0;
            Ok(Cut { shape: Shape::Sphere { center: *center, radius: *radius }, convex, full })
        }
        Primitive::Torus { center, axis, major_radius, minor_radius } => {
            let (c, a) = (v3(*center), normalize(v3(*axis)).ok_or("degenerate axis")?);
            let tube = |q: &Point3| {
                let d = q.coords - c;
                let r = normalize(d - a * d.dot(&a))?;
                normalize(d - r * *major_radius)
            };
            let convex = outwardness(&tube) > 0.0;
            let full = max_gap(angles_about(pts, c, a)) < 1.0;
            Ok(Cut {
                shape: Shape::Torus {
                    center: *center,
                    axis: arr(a),
                    major_radius: *major_radius,
                    minor_radius: *minor_radius,
                },
                convex,
                full,
            })
        }
    }
}

/// Auto-constraint of axes: revolved shapes whose axes are within 3° of the first one's take
/// its direction, and those within 0.5 mm of its line also sit on it (coaxial). Fitted axes
/// differ by noise, and the kernel's intersection of two nearly coaxial surfaces is the
/// general (and fragile) case instead of the analytic one.
pub(crate) fn snap_axes(cuts: &mut [Cut]) {
    let axis_of = |c: &Cut| match &c.shape {
        Shape::Cylinder { point, axis, .. } => Some((*point, *axis)),
        Shape::Cone { apex, axis, .. } => Some((*apex, *axis)),
        Shape::Torus { center, axis, .. } => Some((*center, *axis)),
        _ => None,
    };
    let Some((rp, ra)) = cuts.iter().find_map(axis_of) else { return };
    let (rp, ra) = (v3(rp), v3(ra));
    for c in cuts.iter_mut() {
        let Some((p, a)) = axis_of(c) else { continue };
        let (p, a) = (v3(p), v3(a));
        if a.dot(&ra).abs() < 3.0_f64.to_radians().cos() {
            continue;
        }
        let new_axis = if a.dot(&ra) < 0.0 { -ra } else { ra };
        let d = p - rp;
        let off = d - ra * d.dot(&ra);
        let new_point = if off.norm() < 0.5 { rp + ra * d.dot(&ra) } else { p };
        match &mut c.shape {
            Shape::Cylinder { point, axis, .. } => {
                *point = arr(new_point);
                *axis = arr(new_axis);
            }
            Shape::Cone { apex, axis, .. } => {
                *apex = arr(new_point);
                *axis = arr(new_axis);
            }
            Shape::Torus { center, axis, .. } => {
                *center = arr(new_point);
                *axis = arr(new_axis);
            }
            _ => {}
        }
    }
}

/// The spline of a freeform surface refitted over its region's plane frame with `margin`
/// extra on every side (so the tool crosses the stock), with the mean outward direction of
/// the scan. The extension is the moving-least-squares field continued a few millimetres.
fn extended_spline(
    s: &Session,
    id: ObjectId,
    margin: f64,
    cache: &mut HashMap<ObjectId, Arc<Vec<Vec3>>>,
) -> CmdResult<(Surface, [f64; 3])> {
    let o = s.doc.surface(id).ok_or_else(|| CmdError::Params(format!("no surface {id}")))?;
    let rid = o.region.ok_or_else(|| {
        CmdError::Params(format!("surface {id} has no source region (only fitted regions can be extended)"))
    })?;
    let (pts, nrm) = region_cloud(s, rid, cache)?;
    let field = HeightField::new(&pts, &nrm)
        .ok_or_else(|| CmdError::Failed("the region is not a single patch over its plane".into()))?;
    let [[u0, u1], [v0, v1]] = field.domain;
    let d = [[u0 - margin, u1 + margin], [v0 - margin, v1 + margin]];
    let mean: Vec3 = nrm.iter().sum();
    let out = normalize(mean).unwrap_or(field.n);
    // the same density of control points over the larger domain, then more until the region's
    // points are as close as the original fit was (the extension must not coarsen the surface)
    let grow = |orig: f64, n: usize| ((n as f64) * (orig + 2.0 * margin) / orig.max(1e-9)).ceil() as usize + 1;
    let (cu, cv) = (grow(u1 - u0, o.control[0]), grow(v1 - v0, o.control[1]));
    let g = (cu.max(cv) * 2 + 8).clamp(30, 160);
    // beyond a thin inset from the data the field continues linearly (its tangent plane there):
    // a quadratic moving least squares extrapolates noise into a bump at the contour
    let inset = 0.6;
    let (iu0, iu1, iv0, iv1) = (u0 + inset, u1 - inset, v0 + inset, v1 - inset);
    let ext_fail = || CmdError::Failed("the height field could not be extended over the margin".into());
    let mut grid: Vec<Vec<[f64; 3]>> = Vec::with_capacity(g);
    for i in 0..g {
        let x = d[0][0] + (d[0][1] - d[0][0]) * i as f64 / (g - 1) as f64;
        let mut row = Vec::with_capacity(g);
        for j in 0..g {
            let y = d[1][0] + (d[1][1] - d[1][0]) * j as f64 / (g - 1) as f64;
            let (xc, yc) = (x.clamp(iu0.min(iu1), iu1.max(iu0)), y.clamp(iv0.min(iv1), iv1.max(iv0)));
            let h = 0.3;
            let z0 = field.height(xc, yc).ok_or_else(ext_fail)?;
            let z = if (xc, yc) == (x, y) {
                z0
            } else {
                let gx = (field.height(xc + h, yc).ok_or_else(ext_fail)?
                    - field.height(xc - h, yc).ok_or_else(ext_fail)?)
                    / (2.0 * h);
                let gy = (field.height(xc, yc + h).ok_or_else(ext_fail)?
                    - field.height(xc, yc - h).ok_or_else(ext_fail)?)
                    / (2.0 * h);
                z0 + gx * (x - xc) + gy * (y - yc)
            };
            let p = field.origin + field.u * x + field.v * y + field.n * z;
            row.push([p.x, p.y, p.z]);
        }
        grid.push(row);
    }
    let stride = pts.len().div_ceil(600).max(1);
    let probe_pts: Vec<Point3> = pts.iter().step_by(stride).copied().collect();
    let target = (o.max * 1.1).max(0.12);
    let mut best: Option<(Surface, f64)> = None;
    for extra in [0usize, 3, 6, 10] {
        let c = [
            (cu + extra).min(g).min(surfacecraft_kernel::nurbs::MAX_CONTROL),
            (cv + extra).min(g).min(surfacecraft_kernel::nurbs::MAX_CONTROL),
        ];
        let Ok(spline) = fit_grid(&grid, c[0], c[1], o.degree) else { continue };
        let probe = surfacecraft_kernel::nurbs::SurfaceProbe::new(&spline)?;
        let mut worst: f64 = 0.0;
        for p in &probe_pts {
            worst = worst.max(probe.closest([p.x, p.y, p.z])?.0);
        }
        let better = best.as_ref().is_none_or(|b| worst < b.1);
        if better {
            best = Some((spline, worst));
        }
        if worst <= target {
            break;
        }
    }
    let (spline, _) = best.ok_or_else(|| CmdError::Failed("no extended surface could be fitted".into()))?;
    Ok((spline, arr(out)))
}

/// Parameters of `body.from_regions`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FromRegionsIn {
    /// Fitted primitives (each needs its source region: its points say which side is material).
    #[serde(default)]
    pub primitives: Vec<ObjRef>,
    /// Fitted freeform surfaces (each needs its source region).
    #[serde(default)]
    pub surfaces: Vec<ObjRef>,
    /// Extra room around the points of all regions for the stock, mm (default 2 % of their
    /// diagonal + 3 mm).
    #[serde(default)]
    pub margin: Option<f64>,
    /// Name of the solid.
    #[serde(default)]
    pub name: Option<String>,
    /// Global planes within this many degrees of a model axis are squared to it unless that
    /// moves them over their region by more than max(2 sigma, 0.05 mm) (default 0.3, 0 disables).
    #[serde(default)]
    pub square_planes_deg: Option<f64>,
}

/// Which face of the solid an input became.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FaceOf {
    /// The input object (primitive or surface).
    pub object: ObjectId,
    /// Face indices of the solid on that surface (empty when it was left out).
    pub faces: Vec<usize>,
    /// How it was applied, or why it was left out.
    pub how: String,
    /// Its role in the plan: `global`, `blend`, `local-subtract`, `local-add` or `skipped`.
    #[serde(default)]
    pub role: String,
}

/// Result of `body.from_regions`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FromRegionsOut {
    /// The solid.
    #[serde(flatten)]
    pub body: BodyOut,
    /// Per input: its faces in the solid.
    pub faces_of: Vec<FaceOf>,
    /// Inputs the kernel could not honour (their reasons are in `faces_of`).
    pub skipped: usize,
}

fn describe(c: &CutStatus) -> (String, bool) {
    match c {
        CutStatus::Applied(h) => ((*h).to_string(), true),
        CutStatus::Skipped(why) => (format!("skipped: {why}"), false),
    }
}

/// The fitted primitives of `refs` as planner items, all from one mesh, with the scan's points;
/// `grow` sees the points of every region. Primitives whose cut cannot be made are returned as
/// skipped.
pub(crate) fn collect_items(
    s: &Session,
    refs: &[ObjRef],
    cache: &mut HashMap<ObjectId, Arc<Vec<Vec3>>>,
    mut grow: impl FnMut(&[Point3]),
) -> CmdResult<(Vec<Item>, Vec<FaceOf>, plan::Scan)> {
    let mut items: Vec<Item> = Vec::new();
    let mut early: Vec<FaceOf> = Vec::new();
    let mut mesh: Option<ObjectId> = None;
    for r in refs {
        let id = r.resolve(s, ObjectKind::Primitive)?;
        let o = s.doc.primitive(id).ok_or_else(|| CmdError::Params(format!("no primitive {id}")))?;
        let prim = o.primitive.clone();
        let rid = o.region.ok_or_else(|| {
            CmdError::Params(format!("primitive {id} has no source region (its points give the material side)"))
        })?;
        let reg = s.doc.region(rid).ok_or_else(|| CmdError::Params(format!("no region {rid}")))?;
        match mesh {
            None => mesh = Some(reg.mesh),
            Some(m) if m != reg.mesh => {
                return Err(CmdError::Params("all primitives must come from regions of one mesh".into()));
            }
            Some(_) => {}
        }
        let (pts, nrm) = region_cloud(s, rid, cache)?;
        grow(&pts);
        match cut_of(&prim, &pts, &nrm) {
            Ok(cut) => items.push(Item { id, kind: kind_name(&prim), cut, pts }),
            Err(why) => early.push(FaceOf {
                object: id,
                faces: Vec::new(),
                how: format!("skipped: {why}"),
                role: RoleKind::Skipped.name().to_string(),
            }),
        }
    }
    let scan = match mesh.and_then(|m| s.doc.mesh(m)) {
        Some(m) => plan::Scan::from_mesh(&m.mesh),
        None => plan::Scan::from_points(Vec::new(), 1.0),
    };
    Ok((items, early, scan))
}

fn kind_name(p: &Primitive) -> &'static str {
    match p {
        Primitive::Plane { .. } => "plane",
        Primitive::Sphere { .. } => "sphere",
        Primitive::Cylinder { .. } => "cylinder",
        Primitive::Cone { .. } => "cone",
        Primitive::Torus { .. } => "torus",
    }
}

/// How the cut, tool or addition of item `k` came out: the text, whether it was applied, and the
/// shape as planned.
fn item_outcome(k: usize, plan: &plan::Plan, cuts: &[Cut], built: &hybrid::Built) -> Option<(String, bool, Shape)> {
    let role = plan.roles.get(k)?;
    let of_tools = |sts: Vec<&CutStatus>, none: &str| match sts.iter().find(|st| matches!(st, CutStatus::Skipped(_))) {
        Some(bad) => (describe(bad).0, false, role.shape.clone()),
        None if sts.is_empty() => (none.to_string(), false, role.shape.clone()),
        None => (sts.first().map_or(String::new(), |st| describe(st).0), true, role.shape.clone()),
    };
    Some(match role.kind {
        RoleKind::Global | RoleKind::Blend => {
            let ci = plan.cut_members.iter().position(|m| m.contains(&k));
            match ci.and_then(|ci| built.primitives.get(ci).zip(cuts.get(ci))) {
                Some((st, cut)) => {
                    let (how, ok) = describe(st);
                    (how, ok, cut.shape.clone())
                }
                None => ("skipped: not passed to the kernel".to_string(), false, role.shape.clone()),
            }
        }
        RoleKind::LocalSubtract => {
            of_tools(role.tools.iter().filter_map(|&t| built.tools.get(t)).collect(), "skipped: no tool")
        }
        RoleKind::LocalAdd => {
            of_tools(role.adds.iter().filter_map(|&t| built.adds.get(t)).collect(), "skipped: no addition")
        }
        RoleKind::Skipped => (format!("skipped: {}", role.how), false, role.shape.clone()),
    })
}

/// Per planned item: its role, how it was applied (or why not) and its faces in the built solid.
pub(crate) fn item_faces(
    items: &[Item],
    plan: &plan::Plan,
    cuts: &[Cut],
    built: &hybrid::Built,
    tol: f64,
) -> Vec<FaceOf> {
    let mut out = Vec::with_capacity(items.len());
    for (k, it) in items.iter().enumerate() {
        let Some(role) = plan.roles.get(k) else { continue };
        let Some((how, applied, shape)) = item_outcome(k, plan, cuts, built) else { continue };
        let faces = if applied { hybrid::faces_on_shape(&built.brep, &shape, tol) } else { Vec::new() };
        out.push(FaceOf { object: it.id, faces, how, role: role.kind.name().to_string() });
    }
    out
}

pub(crate) fn from_regions(s: &mut Session, i: FromRegionsIn) -> CmdResult<FromRegionsOut> {
    if i.primitives.is_empty() && i.surfaces.is_empty() {
        return Err(CmdError::Params("give `primitives` and/or `surfaces`".into()));
    }
    if i.primitives.len() + i.surfaces.len() > 256 {
        return Err(CmdError::Params("at most 256 inputs".into()));
    }
    if let Some(m) = i.margin
        && !(m.is_finite() && (0.0..=1e6).contains(&m))
    {
        return Err(CmdError::Params("margin must be finite and non-negative".into()));
    }
    let mut cache = HashMap::new();
    let (mut lo, mut hi) = (Vec3::repeat(f64::INFINITY), Vec3::repeat(f64::NEG_INFINITY));
    let (items, skipped_early, scan) = collect_items(s, &i.primitives, &mut cache, |pts| {
        for p in pts {
            lo = lo.inf(&p.coords);
            hi = hi.sup(&p.coords);
        }
    })?;
    // the freeform regions bound the stock too
    let mut surf_regions: Vec<(ObjectId, ObjectId)> = Vec::new();
    for r in &i.surfaces {
        let id = r.resolve(s, ObjectKind::Surface)?;
        let o = s.doc.surface(id).ok_or_else(|| CmdError::Params(format!("no surface {id}")))?;
        let rid = o.region.ok_or_else(|| CmdError::Params(format!("surface {id} has no source region")))?;
        let (pts, _) = region_cloud(s, rid, &mut cache)?;
        for p in &pts {
            lo = lo.inf(&p.coords);
            hi = hi.sup(&p.coords);
        }
        surf_regions.push((id, rid));
    }
    if !(lo.iter().chain(hi.iter()).all(|v| v.is_finite())) {
        return Err(CmdError::Params("the regions have no points".into()));
    }
    let diag = (hi - lo).norm();
    let margin = i.margin.unwrap_or(0.02 * diag + 3.0);
    let (slo, shi) = (lo - Vec3::repeat(margin), hi + Vec3::repeat(margin));
    let mut free = Vec::new();
    for (id, _) in &surf_regions {
        let (spline, outward) = extended_spline(s, *id, margin + 1.0, &mut cache)?;
        free.push(FreeCut { surface: spline, outward });
    }
    // which regions are global cuts of the stock and which form local tools (blind holes,
    // pockets, slots)
    let square = square_deg(i.square_planes_deg)?;
    let plan = plan::plan(&items, &scan, square);
    let mut cuts = plan.cuts.clone();
    snap_axes(&mut cuts);
    let tools: Vec<LocalTool> = plan.tools.iter().map(|t| t.tool.clone()).collect();
    if cuts.is_empty() && free.is_empty() {
        return Err(CmdError::Failed(
            "no region is a global cut of the stock (see body.plan_regions for the role of each)".into(),
        ));
    }
    let adds: Vec<LocalAdd> = plan.adds.iter().map(|t| t.add.clone()).collect();
    let built = hybrid::solid_from_plan(arr(slo), arr(shi), &cuts, &free, &tools, &adds)?;
    let mut faces_of = skipped_early;
    let tol = 0.02 * diag.max(1.0) / 10.0 + 1e-3;
    let mut skipped = faces_of.len();
    let of_items = item_faces(&items, &plan, &cuts, &built, tol);
    skipped += of_items.iter().filter(|f| f.how.starts_with("skipped")).count();
    faces_of.extend(of_items);
    for (((id, _), f), st) in surf_regions.iter().zip(&free).zip(&built.freeform) {
        let (how, applied) = describe(st);
        let faces = if applied { hybrid::faces_on_spline(&built.brep, &f.surface, tol)? } else { Vec::new() };
        skipped += usize::from(!applied);
        faces_of.push(FaceOf { object: *id, faces, how, role: "global".to_string() });
    }
    let bid = s.doc.add_body(i.name.as_deref().unwrap_or("hybrid"), built.brep);
    Ok(FromRegionsOut { body: body_out(s, bid)?, faces_of, skipped })
}

/// Parameters of `body.plan_regions`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PlanRegionsIn {
    /// Fitted primitives (each needs its source region), the same list `body.from_regions` takes.
    pub primitives: Vec<ObjRef>,
    /// As in `body.from_regions` (default 0.3, 0 disables squaring of global planes).
    #[serde(default)]
    pub square_planes_deg: Option<f64>,
}

pub(crate) fn square_deg(v: Option<f64>) -> CmdResult<f64> {
    let d = v.unwrap_or(0.3);
    if d.is_finite() && (0.0..=5.0).contains(&d) {
        Ok(d)
    } else {
        Err(CmdError::Params("square_planes_deg must be between 0 and 5".into()))
    }
}

fn plan_regions(s: &mut Session, i: PlanRegionsIn) -> CmdResult<PlanOut> {
    if i.primitives.is_empty() {
        return Err(CmdError::Params("give `primitives`".into()));
    }
    if i.primitives.len() > 256 {
        return Err(CmdError::Params("at most 256 inputs".into()));
    }
    let (items, _, scan) = collect_items(s, &i.primitives, &mut HashMap::new(), |_| {})?;
    let p = plan::plan(&items, &scan, square_deg(i.square_planes_deg)?);
    Ok(plan::report(&items, &p))
}

/// Parameters of `body.replace_face`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ReplaceFaceIn {
    /// Solid body.
    pub object: ObjRef,
    /// Record index of a planar face.
    pub face: usize,
    /// Fitted freeform surface that takes its place (needs its source region). It must run
    /// across the whole face; where it lies above the face material is added (the face is
    /// extruded first), where it lies below material is cut away.
    pub surface: ObjRef,
    /// Extension of the patch past its region on every side, mm (default 3).
    #[serde(default)]
    pub margin: Option<f64>,
}

fn replace_face(s: &mut Session, i: ReplaceFaceIn) -> CmdResult<BodyOut> {
    let id = i.object.resolve(s, ObjectKind::Body)?;
    let sid = i.surface.resolve(s, ObjectKind::Surface)?;
    let margin = i.margin.unwrap_or(3.0);
    if !(margin.is_finite() && (0.0..=1e6).contains(&margin)) {
        return Err(CmdError::Params("margin must be finite and non-negative".into()));
    }
    let (spline, _) = extended_spline(s, sid, margin, &mut HashMap::new())?;
    let src = s.doc.body(id).ok_or_else(|| CmdError::Params(format!("no body {id}")))?;
    let out = hybrid::replace_face(&src.brep, i.face, &spline)?;
    if let Some(b) = s.doc.body_mut(id) {
        b.brep = Arc::new(out);
    }
    body_out(s, id)
}

/// Parameters of `body.extract_face`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ExtractFaceIn {
    /// Body.
    pub object: ObjRef,
    /// Record index of the face.
    pub face: usize,
    /// Name of the sheet.
    #[serde(default)]
    pub name: Option<String>,
}

fn extract_face(s: &mut Session, i: ExtractFaceIn) -> CmdResult<BodyOut> {
    let id = i.object.resolve(s, ObjectKind::Body)?;
    let src = s.doc.body(id).ok_or_else(|| CmdError::Params(format!("no body {id}")))?;
    let sheet = hybrid::extract_face(&src.brep, i.face)?;
    let bid = s.doc.add_body(i.name.as_deref().unwrap_or("face"), sheet);
    body_out(s, bid)
}

/// Parameters of the `*.trim_to_region` commands.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TrimIn {
    /// The primitive or freeform surface.
    pub object: ObjRef,
    /// The solid made by `body.from_regions` (or any body that has a face on the object).
    pub body: ObjRef,
    /// Name of the sheet.
    #[serde(default)]
    pub name: Option<String>,
}

fn trimmed(s: &mut Session, i: TrimIn, kind: ObjectKind) -> CmdResult<BodyOut> {
    let oid = i.object.resolve(s, kind)?;
    let bid = i.body.resolve(s, ObjectKind::Body)?;
    let brep = s.doc.body(bid).map(|b| b.brep.clone()).ok_or_else(|| CmdError::Params(format!("no body {bid}")))?;
    let b = brep.as_ref();
    let diag = b.vertices.iter().fold(0.0_f64, |m, v| m.max(v.point.iter().fold(0.0_f64, |a, x| a.max(x.abs()))));
    let tol = 0.002 * diag.max(1.0) + 1e-3;
    let faces = if kind == ObjectKind::Primitive {
        let o = s.doc.primitive(oid).ok_or_else(|| CmdError::Params(format!("no primitive {oid}")))?;
        let shape = shape_of(&o.primitive);
        hybrid::faces_on_shape(b, &shape, tol)
    } else {
        let o = s.doc.surface(oid).ok_or_else(|| CmdError::Params(format!("no surface {oid}")))?;
        let sheet = o.parent.clone().unwrap_or_else(|| o.sheet.clone());
        let surf = sheet
            .faces
            .first()
            .map(|f| f.surface.clone())
            .ok_or_else(|| CmdError::Params("the surface has no face".into()))?;
        // the fitted spline spans the region only; the trimmed face reaches a little past it
        hybrid::faces_on_spline(b, &surf, 1.0)?
    };
    let [face] = faces[..] else {
        return Err(CmdError::Failed(format!(
            "expected exactly one face of the body on the object, found {}",
            faces.len()
        )));
    };
    let sheet = hybrid::extract_face(b, face)?;
    let id = s.doc.add_body(i.name.as_deref().unwrap_or("trimmed face"), sheet);
    body_out(s, id)
}

fn shape_of(p: &Primitive) -> Shape {
    match p {
        Primitive::Plane { point, normal } => Shape::Plane { point: *point, normal: *normal },
        Primitive::Sphere { center, radius } => Shape::Sphere { center: *center, radius: *radius },
        Primitive::Cylinder { point, axis, radius } => Shape::Cylinder { point: *point, axis: *axis, radius: *radius },
        Primitive::Cone { apex, axis, half_angle } => Shape::Cone { apex: *apex, axis: *axis, half_angle: *half_angle },
        Primitive::Torus { center, axis, major_radius, minor_radius } => {
            Shape::Torus { center: *center, axis: *axis, major_radius: *major_radius, minor_radius: *minor_radius }
        }
    }
}

fn trim_surface(s: &mut Session, i: TrimIn) -> CmdResult<BodyOut> {
    trimmed(s, i, ObjectKind::Surface)
}

fn trim_primitive(s: &mut Session, i: TrimIn) -> CmdResult<BodyOut> {
    trimmed(s, i, ObjectKind::Primitive)
}

/// The hybrid-model commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "body.from_regions",
        "Solid from regions",
        "Solid",
        true,
        "Builds one closed solid by trimming a stock with fitted `primitives` and freeform `surfaces`; reports each input's faces.",
        FromRegionsIn,
        FromRegionsOut,
        from_regions
    ),
    command!(
        "body.plan_regions",
        "Plan regions",
        "Solid",
        false,
        "Reports the role of each fitted `primitives` region in `body.from_regions` (global cut, local tool part, fillet, skipped, with the reason), the local tools (blind holes, pockets, slots) and the convex/concave contacts between regions.",
        PlanRegionsIn,
        PlanOut,
        plan_regions
    ),
    command!(
        "body.replace_face",
        "Replace face",
        "Solid",
        true,
        "Replaces planar `face` of `object` by the freeform `surface` (cuts the material beyond the patch).",
        ReplaceFaceIn,
        BodyOut,
        replace_face
    ),
    command!(
        "body.extract_face",
        "Extract face",
        "Solid",
        true,
        "Copies `face` of `object` as a sheet body, trimmed to its contour.",
        ExtractFaceIn,
        BodyOut,
        extract_face
    ),
    command!(
        "surface.trim_to_region",
        "Trim surface to region",
        "Surface",
        true,
        "Sheet of the face of `body` that lies on freeform `object`: the patch trimmed to its region contour.",
        TrimIn,
        BodyOut,
        trim_surface
    ),
    command!(
        "primitive.trim_to_region",
        "Trim primitive to region",
        "Primitive",
        true,
        "Sheet of the face of `body` that lies on primitive `object`: bounded by its intersections with the neighbours.",
        TrimIn,
        BodyOut,
        trim_primitive
    ),
];

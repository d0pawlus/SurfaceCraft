//! The hybrid model (design doc §6.3, plan P3): one closed solid from scanned primitives and
//! freeform patches, built by trimming a stock.
//!
//! Every recognised surface is a *cut* of the stock. A plane keeps the material side of a
//! kernel split (`ops::split::split_body`, the plane as the tool); a convex cylinder bridging two
//! planes is a fillet of their edge (`ops::blend`); a full-revolution cylinder, cone, sphere or
//! torus is a solid of that shape intersected with (convex) or subtracted from (concave) the
//! stock; a freeform patch is a spline tool that splits the stock and keeps its material side.
//! The faces of the result are therefore exactly the primitives trimmed to the contours where
//! they meet their neighbours, which is the "primitive → trimmed B-rep face" step of the plan.
//! All B-rep construction is the kernel's; this file sequences the calls and chooses sides.
use crate::nurbs::{SurfaceProbe, canonicalize};
use crate::record::brep::{Brep, Frame, Loop, Region, Shell, Surface};
use crate::solid::{decode, encode};
use crate::{KernelError, Result, finite_all, op_err, solid};
use solidcore_ops as ops;

/// An analytic surface of a recognised region.
#[derive(Clone, Debug, PartialEq)]
pub enum Shape {
    /// Plane through `point` with unit `normal` pointing out of the material.
    Plane {
        /// A point of the plane.
        point: [f64; 3],
        /// Unit normal.
        normal: [f64; 3],
    },
    /// Cylinder about the axis through `point` along unit `axis`.
    Cylinder {
        /// A point of the axis.
        point: [f64; 3],
        /// Unit axis.
        axis: [f64; 3],
        /// Radius.
        radius: f64,
    },
    /// Cone with `apex`, unit `axis` pointing into its opening, `half_angle` in (0, π/2).
    Cone {
        /// Apex.
        apex: [f64; 3],
        /// Unit axis into the opening.
        axis: [f64; 3],
        /// Half angle, radians.
        half_angle: f64,
    },
    /// Sphere.
    Sphere {
        /// Centre.
        center: [f64; 3],
        /// Radius.
        radius: f64,
    },
    /// Ring torus about unit `axis`.
    Torus {
        /// Centre.
        center: [f64; 3],
        /// Unit axis.
        axis: [f64; 3],
        /// Major radius.
        major_radius: f64,
        /// Minor radius.
        minor_radius: f64,
    },
}

/// A primitive cut of the stock.
#[derive(Clone, Debug, PartialEq)]
pub struct Cut {
    /// The surface.
    pub shape: Shape,
    /// The material lies inside the shape (a boss, an outer wall, a fillet), not outside it
    /// (a hole, a countersink). Ignored for planes: their material is behind the normal.
    pub convex: bool,
    /// The region wraps all the way round the axis (cylinder, cone). A partial convex cylinder
    /// is a fillet and needs the two planes it is tangent to.
    pub full: bool,
}

/// A freeform cut: a spline tool over its parameter box.
#[derive(Clone, Debug, PartialEq)]
pub struct FreeCut {
    /// A record spline surface (see [`crate::nurbs::fit_grid`]).
    pub surface: Surface,
    /// Direction the region's outward normals point on average (picks the material side).
    pub outward: [f64; 3],
}

/// What [`solid_from_cuts`] did.
#[derive(Clone, Debug)]
pub struct Built {
    /// The closed solid.
    pub brep: Brep,
    /// Per primitive cut: how it was applied, or why it was left out.
    pub primitives: Vec<CutStatus>,
    /// Per freeform cut: applied?
    pub freeform: Vec<CutStatus>,
}

/// The outcome of one cut.
#[derive(Clone, Debug, PartialEq)]
pub enum CutStatus {
    /// Applied; the text says how (`halfspace`, `fillet`, `intersect`, `subtract`, `split`).
    Applied(&'static str),
    /// Left out, with the reason.
    Skipped(String),
}

type V = [f64; 3];

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn scale(a: V, s: f64) -> V {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn norm(a: V) -> f64 {
    dot(a, a).sqrt()
}
fn unit(a: V) -> Option<V> {
    let n = norm(a);
    (n.is_finite() && n > 1e-12).then(|| scale(a, 1.0 / n))
}
fn perpendicular(a: V) -> V {
    let t = if a[0].abs() < 0.6 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let p = sub(t, scale(a, dot(t, a)));
    unit(p).unwrap_or([0.0, 0.0, 1.0])
}

/// Column-vector rigid matrix taking local (x, y, z) to the frame at `origin` with z along `axis`.
fn frame_matrix(origin: V, axis: V) -> [[f64; 4]; 4] {
    let x = perpendicular(axis);
    let y = cross(axis, x);
    [
        [x[0], y[0], axis[0], origin[0]],
        [x[1], y[1], axis[1], origin[1]],
        [x[2], y[2], axis[2], origin[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn corners(lo: V, hi: V) -> Vec<V> {
    (0..8)
        .map(|k| {
            [
                if k & 1 == 0 { lo[0] } else { hi[0] },
                if k & 2 == 0 { lo[1] } else { hi[1] },
                if k & 4 == 0 { lo[2] } else { hi[2] },
            ]
        })
        .collect()
}

/// The material side of a split by an oriented tool: `outward` is where the tool normal should
/// point; the kernel returns (behind the normal, in front of it).
fn pick(parts: Option<(ops::Body, ops::Body)>, tool_normal: V, outward: V) -> Result<ops::Body> {
    let (back, front) = parts.ok_or_else(|| KernelError::Op { op: "split_body", message: "a side is empty".into() })?;
    Ok(if dot(tool_normal, outward) >= 0.0 { back } else { front })
}

fn plane_tool(point: V, normal: V, half: f64) -> Result<(ops::split::Tool, V)> {
    let n = unit(normal).ok_or_else(|| KernelError::Argument("plane normal is zero".into()))?;
    let u = perpendicular(n);
    let v = cross(n, u);
    let tool = ops::split::Tool {
        surface: Surface::Plane { origin: point, u_axis: u, v_axis: v },
        domain: [[-half, half], [-half, half]],
    };
    Ok((tool, n))
}

fn half_space(body: &ops::Body, point: V, normal: V, half: f64) -> Result<ops::Body> {
    let (tool, n) = plane_tool(point, normal, half)?;
    let parts = ops::split::split_body(body, &tool).map_err(|e| op_err("split_body", e))?;
    // the plane's own normal is u × v = n, so material behind it is the back part
    pick(parts, n, n)
}

fn shape_solid(shape: &Shape, lo: V, hi: V) -> Result<ops::Body> {
    let cs = corners(lo, hi);
    let diag = norm(sub(hi, lo)).max(1.0);
    match shape {
        Shape::Cylinder { point, axis, radius } => {
            let a = unit(*axis).ok_or_else(|| KernelError::Argument("axis is zero".into()))?;
            let (mut t0, mut t1) = (f64::INFINITY, f64::NEG_INFINITY);
            for c in &cs {
                let t = dot(sub(*c, *point), a);
                t0 = t0.min(t);
                t1 = t1.max(t);
            }
            let m = 0.1 * diag + 1.0;
            let b = ops::make_cylinder([0.0, 0.0, t0 - m], *radius, t1 - t0 + 2.0 * m)
                .map_err(|e| op_err("make_cylinder", e))?;
            b.transform(&crate::to_kernel_matrix(&frame_matrix(*point, a))).map_err(|e| op_err("transform", e))?;
            Ok(b)
        }
        Shape::Cone { apex, axis, half_angle } => {
            let a = unit(*axis).ok_or_else(|| KernelError::Argument("axis is zero".into()))?;
            // the cone from its apex out to past the farthest corner along the axis
            let mut depth: f64 = 0.0;
            for c in &cs {
                depth = depth.max(dot(sub(*c, *apex), a));
            }
            let depth = depth + 0.1 * diag + 1.0;
            let wide = depth * half_angle.tan();
            // the cone widens along +z (a record cone that narrows along z does not survive a
            // STEP round trip in solidcore), so it starts from a tiny disc of radius `tip`
            let tip = 0.01;
            let start = tip / half_angle.tan();
            let b = ops::make_cone([0.0; 3], tip, wide, depth - start).map_err(|e| op_err("make_cone", e))?;
            let origin = add(*apex, scale(a, start));
            b.transform(&crate::to_kernel_matrix(&frame_matrix(origin, a))).map_err(|e| op_err("transform", e))?;
            Ok(b)
        }
        Shape::Sphere { center, radius } => ops::make_sphere(*center, *radius).map_err(|e| op_err("make_sphere", e)),
        Shape::Torus { center, axis, major_radius, minor_radius } => {
            let a = unit(*axis).ok_or_else(|| KernelError::Argument("axis is zero".into()))?;
            let b = ops::make_torus([0.0; 3], *major_radius, *minor_radius).map_err(|e| op_err("make_torus", e))?;
            b.transform(&crate::to_kernel_matrix(&frame_matrix(*center, a))).map_err(|e| op_err("transform", e))?;
            Ok(b)
        }
        Shape::Plane { .. } => Err(KernelError::Argument("a plane is not a solid".into())),
    }
}

fn check_shape(s: &Shape) -> Result<()> {
    let ok = match s {
        Shape::Plane { point, normal } => finite_all(point) && finite_all(normal),
        Shape::Cylinder { point, axis, radius } => {
            finite_all(point) && finite_all(axis) && *radius > 0.0 && radius.is_finite()
        }
        Shape::Cone { apex, axis, half_angle } => {
            finite_all(apex)
                && finite_all(axis)
                && *half_angle > 1e-6
                && *half_angle < std::f64::consts::FRAC_PI_2 - 1e-6
        }
        Shape::Sphere { center, radius } => finite_all(center) && *radius > 0.0 && radius.is_finite(),
        Shape::Torus { center, axis, major_radius, minor_radius } => {
            finite_all(center)
                && finite_all(axis)
                && *minor_radius > 0.0
                && major_radius > minor_radius
                && major_radius.is_finite()
        }
    };
    if ok { Ok(()) } else { Err(KernelError::Argument("shape parameters must be finite and in range".into())) }
}

/// The record edge of `brep` whose both end vertices lie on both planes (a sharp edge of two
/// plane faces), if exactly one does.
fn edge_between(brep: &Brep, a: (V, V), b: (V, V), tol: f64) -> Option<usize> {
    let on = |p: V, pl: (V, V)| dot(sub(p, pl.0), pl.1).abs() < tol;
    let mut found = None;
    for (i, e) in brep.edges.iter().enumerate() {
        let ends =
            e.vertices.iter().filter_map(|&v| brep.vertices.get(v as usize)).map(|v| v.point).collect::<Vec<_>>();
        if ends.len() == 2 && ends.iter().all(|&p| on(p, a) && on(p, b)) {
            if found.is_some() {
                return None;
            }
            found = Some(i);
        }
    }
    found
}

/// Builds the solid. `lo`/`hi` bound the stock (it must contain the part, with room to spare);
/// planes and full-revolution shapes are applied in that order, then fillets, then freeform
/// cuts. Cuts the kernel cannot honour are reported as skipped; the solid is verified closed.
pub fn solid_from_cuts(lo: V, hi: V, cuts: &[Cut], free: &[FreeCut]) -> Result<Built> {
    if !finite_all(&lo) || !finite_all(&hi) || (0..3).any(|i| hi[i] <= lo[i]) {
        return Err(KernelError::Argument("stock bounds must be finite with hi > lo".into()));
    }
    for c in cuts {
        check_shape(&c.shape)?;
    }
    if cuts.is_empty() && free.is_empty() {
        return Err(KernelError::Argument("nothing to build from".into()));
    }
    let half = 4.0 * norm(sub(hi, lo)).max(1.0);
    let stock = ops::make_box(lo, sub(hi, lo)).map_err(|e| op_err("make_box", e))?;
    let mut body = stock;
    let mut status: Vec<CutStatus> = vec![CutStatus::Skipped("not applied".into()); cuts.len()];
    // 1. planes
    let mut planes: Vec<(usize, V, V)> = Vec::new();
    for (i, c) in cuts.iter().enumerate() {
        if let Shape::Plane { point, normal } = &c.shape {
            let n = unit(*normal).ok_or_else(|| KernelError::Argument("plane normal is zero".into()))?;
            body = half_space(&body, *point, n, half)?;
            planes.push((i, *point, n));
            if let Some(s) = status.get_mut(i) {
                *s = CutStatus::Applied("halfspace");
            }
        }
    }
    // 2. fillets: a partial convex cylinder tangent to two of the planes
    for (i, c) in cuts.iter().enumerate() {
        let Shape::Cylinder { point, axis, radius } = &c.shape else { continue };
        if !c.convex || c.full {
            continue;
        }
        let a = unit(*axis).ok_or_else(|| KernelError::Argument("axis is zero".into()))?;
        let tol = 0.05 * radius + 0.05;
        // planes parallel to the axis whose distance from it is the radius
        let tangent: Vec<(V, V)> = planes
            .iter()
            .filter(|(_, p, n)| dot(*n, a).abs() < 0.05 && (dot(sub(*point, *p), *n).abs() - radius).abs() < tol)
            .map(|(_, p, n)| (*p, *n))
            .collect();
        let mut done = false;
        'pairs: for x in 0..tangent.len() {
            for y in (x + 1)..tangent.len() {
                let (p1, p2) = (tangent[x], tangent[y]);
                // the centre lies behind both planes at distance r, and the pair is not parallel
                if dot(p1.1, p2.1).abs() > 0.95 {
                    continue;
                }
                // the cylinder's axis must be the offset intersection: check against both planes
                let inside = |p: (V, V)| dot(sub(*point, p.0), p.1) < 0.0;
                if !inside(p1) || !inside(p2) {
                    continue;
                }
                let rec = encode(&body)?;
                if let Some(e) = edge_between(&rec, p1, p2, 1e-4 * (1.0 + norm(sub(hi, lo)))) {
                    body = decode(&solid::fillet(&rec, &[e], *radius)?)?;
                    done = true;
                    break 'pairs;
                }
            }
        }
        if let Some(s) = status.get_mut(i) {
            *s = if done {
                CutStatus::Applied("fillet")
            } else {
                CutStatus::Skipped("no edge of two planes tangent to this cylinder was found".into())
            };
        }
    }
    // 3. full-revolution shapes
    for (i, c) in cuts.iter().enumerate() {
        let revolve = match &c.shape {
            Shape::Plane { .. } => continue,
            Shape::Cylinder { .. } | Shape::Cone { .. } if !c.full => {
                if c.convex {
                    continue; // handled as a fillet above
                }
                if let Some(s) = status.get_mut(i) {
                    *s = CutStatus::Skipped("a partial concave cylinder or cone is not supported yet".into());
                }
                continue;
            }
            _ => true,
        };
        if !revolve {
            continue;
        }
        let tool = shape_solid(&c.shape, lo, hi)?;
        let (op, how) =
            if c.convex { (ops::BooleanOp::Intersect, "intersect") } else { (ops::BooleanOp::Subtract, "subtract") };
        body = ops::boolean::boolean(body, tool, op)
            .map_err(|e| op_err("boolean", format!("{e} ({how} {:?})", c.shape)))?
            .ok_or_else(|| KernelError::Op { op: "boolean", message: "the cut removed everything".into() })?;
        if let Some(s) = status.get_mut(i) {
            *s = CutStatus::Applied(how);
        }
    }
    // 4. freeform tools
    let mut fstatus = Vec::with_capacity(free.len());
    for f in free {
        body = cut_by_spline_body(&body, &f.surface, f.outward)?;
        fstatus.push(CutStatus::Applied("split"));
    }
    if !body.is_manifold_solid().map_err(|e| op_err("is_manifold_solid", e))? {
        return Err(KernelError::Op {
            op: "solid_from_cuts",
            message: "the result is not a closed manifold solid".into(),
        });
    }
    let n = body.solid_count().map_err(|e| op_err("solid_count", e))?;
    if n != 1 {
        return Err(KernelError::Op {
            op: "solid_from_cuts",
            message: format!("the result has {n} solids, expected 1"),
        });
    }
    Ok(Built { brep: encode(&body)?, primitives: status, freeform: fstatus })
}

/// The normal of a spline at the middle of its domain, as finite differences of the surface.
fn spline_normal(surface: &Surface) -> Result<V> {
    let [[u0, u1], [v0, v1]] = crate::nurbs::domain(surface)?;
    let probe = SurfaceProbe::new(surface)?;
    let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let (du, dv) = (1e-3 * (u1 - u0), 1e-3 * (v1 - v0));
    let a = sub(probe.eval(um + du, vm)?, probe.eval(um - du, vm)?);
    let b = sub(probe.eval(um, vm + dv)?, probe.eval(um, vm - dv)?);
    unit(cross(a, b)).ok_or_else(|| KernelError::Argument("degenerate spline".into()))
}

fn cut_by_spline_body(body: &ops::Body, surface: &Surface, outward: V) -> Result<ops::Body> {
    if !finite_all(&outward) {
        return Err(KernelError::Argument("outward direction must be finite".into()));
    }
    let domain = crate::nurbs::domain(surface)?;
    let n = spline_normal(surface)?;
    let tool = ops::split::Tool { surface: surface.clone(), domain };
    let parts = ops::split::split_body(body, &tool).map_err(|e| op_err("split_body", e))?;
    pick(parts, n, outward)
}

/// Cuts `body` with a spline tool and keeps the side `outward` points away from (the tool's
/// own side that holds the material). The spline must cross the body completely.
pub fn cut_by_spline(body: &Brep, surface: &Surface, outward: [f64; 3]) -> Result<Brep> {
    let b = decode(body)?;
    let r = cut_by_spline_body(&b, surface, outward)?;
    encode(&r)
}

/// Replaces `face` (a record index of a planar face) of a solid by the spline tool `surface`:
/// the material beyond the patch (on the face's outward side) is cut away. The patch must run
/// across the whole footprint of the face and not above it everywhere (it only removes material).
pub fn replace_face(body: &Brep, face: usize, surface: &Surface) -> Result<Brep> {
    let outward = planar_face_outward(body, face)?;
    cut_by_spline(body, surface, outward)
}

/// The outward unit normal of a planar record face.
pub fn planar_face_outward(body: &Brep, face: usize) -> Result<V> {
    let f = body
        .faces
        .get(face)
        .ok_or_else(|| KernelError::Argument(format!("face indices must be in 0..{}", body.faces.len())))?;
    let n = match &f.surface {
        Surface::Plane { u_axis, v_axis, .. } => cross(*u_axis, *v_axis),
        Surface::PlaneThrough { origin, u_point, v_point } => cross(sub(*u_point, *origin), sub(*v_point, *origin)),
        _ => return Err(KernelError::Argument("only a planar face can be replaced".into())),
    };
    let n = unit(n).ok_or_else(|| KernelError::Argument("degenerate plane".into()))?;
    let forward = body
        .shells
        .iter()
        .find_map(|s| match s {
            Shell::Faces { faces, .. } => faces.iter().find(|o| o.face as usize == face).map(|o| o.forward),
            _ => None,
        })
        .unwrap_or(true);
    Ok(if forward { n } else { scale(n, -1.0) })
}

/// One face of a body as a sheet (the trimmed face on its own surface, bounded by the edges it
/// has in the body).
pub fn extract_face(body: &Brep, face: usize) -> Result<Brep> {
    let f = body
        .faces
        .get(face)
        .ok_or_else(|| KernelError::Argument(format!("face indices must be in 0..{}", body.faces.len())))?
        .clone();
    // the sheet uses the face on its natural side (the canonical walk of the record depends on it)
    if f.loops.iter().any(|l| matches!(l, Loop::Vertex { .. })) {
        return Err(KernelError::Argument("a face with a vertex loop cannot be extracted".into()));
    }
    let sheet = Brep {
        version: body.version,
        tolerance: body.tolerance,
        regions: vec![Region { shells: vec![0], void: false }],
        shells: vec![Shell::Faces {
            faces: vec![crate::record::brep::OrientedFace { face: 0, forward: true }],
            wires: vec![],
        }],
        faces: vec![f],
        edges: body.edges.clone(),
        vertices: body.vertices.clone(),
    };
    let sheet = canonicalize(sheet);
    sheet.validate().map_err(|e| op_err("record validate", e))?;
    let (b, _) = ops::Body::from_record(&sheet, false).map_err(|e| op_err("from_record", e))?;
    encode(&b)
}

fn face_vertices(body: &Brep, face: usize) -> Vec<V> {
    let mut out = Vec::new();
    if let Some(f) = body.faces.get(face) {
        for l in &f.loops {
            if let Loop::Edges { edges } = l {
                for oe in edges {
                    if let Some(e) = body.edges.get(oe.edge as usize) {
                        for &v in &e.vertices {
                            if let Some(v) = body.vertices.get(v as usize) {
                                out.push(v.point);
                            }
                        }
                    }
                }
            }
        }
    }
    out
}

fn frame_axis(f: &Frame) -> V {
    cross(f.x_axis, f.y_axis)
}

fn axis_distance(p: V, point: V, axis: V) -> f64 {
    let d = sub(p, point);
    norm(sub(d, scale(axis, dot(d, axis))))
}

/// The faces of `body` that lie on `shape` (same surface type, parameters within `tol`).
#[must_use]
pub fn faces_on_shape(body: &Brep, shape: &Shape, tol: f64) -> Vec<usize> {
    let mut out = Vec::new();
    for (i, f) in body.faces.iter().enumerate() {
        let hit = match (&f.surface, shape) {
            (Surface::Plane { origin, u_axis, v_axis }, Shape::Plane { point, normal }) => {
                unit(cross(*u_axis, *v_axis))
                    .is_some_and(|n| dot(n, *normal).abs() > 0.999 && dot(sub(*point, *origin), n).abs() < tol)
            }
            (Surface::Cylinder { frame, radius }, Shape::Cylinder { point, axis, radius: r }) => {
                unit(frame_axis(frame)).is_some_and(|a| dot(a, *axis).abs() > 0.999)
                    && (radius - r).abs() < tol
                    && axis_distance(frame.origin, *point, *axis) < tol
            }
            // a blend's cylinder comes back as a cone of half angle zero (SMLib's own form)
            (Surface::Cone { frame, radius_at_origin, half_angle }, Shape::Cylinder { point, axis, radius: r })
                if half_angle.abs() < 1e-9 =>
            {
                unit(frame_axis(frame)).is_some_and(|a| dot(a, *axis).abs() > 0.999)
                    && (radius_at_origin - r).abs() < tol
                    && axis_distance(frame.origin, *point, *axis) < tol
            }
            (Surface::Cone { frame, half_angle, .. }, Shape::Cone { apex, axis, half_angle: h }) => {
                unit(frame_axis(frame)).is_some_and(|a| dot(a, *axis).abs() > 0.999)
                    && (half_angle.abs() - h).abs() < 0.02
                    && axis_distance(frame.origin, *apex, *axis) < tol
            }
            (Surface::Sphere { frame, radius }, Shape::Sphere { center, radius: r }) => {
                (radius - r).abs() < tol && norm(sub(frame.origin, *center)) < tol
            }
            (
                Surface::Torus { frame, major_radius, minor_radius, .. },
                Shape::Torus { center, axis, major_radius: mr, minor_radius: nr },
            ) => {
                unit(frame_axis(frame)).is_some_and(|a| dot(a, *axis).abs() > 0.999)
                    && (major_radius - mr).abs() < tol
                    && (minor_radius - nr).abs() < tol
                    && norm(sub(frame.origin, *center)) < tol
            }
            _ => false,
        };
        if hit {
            out.push(i);
        }
    }
    out
}

/// The faces of `body` whose boundary vertices all lie within `tol` of the spline `surface`.
pub fn faces_on_spline(body: &Brep, surface: &Surface, tol: f64) -> Result<Vec<usize>> {
    let probe = SurfaceProbe::new(surface)?;
    let mut out = Vec::new();
    for (i, f) in body.faces.iter().enumerate() {
        if !matches!(f.surface, Surface::Spline { .. } | Surface::RationalSpline { .. }) {
            continue;
        }
        let vs = face_vertices(body, i);
        if !vs.is_empty() && vs.iter().all(|&p| probe.closest(p).map(|(d, _)| d < tol).unwrap_or(false)) {
            out.push(i);
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::{fit_grid, sheet_from_spline};

    fn wave_surface(x0: f64, x1: f64, y0: f64, y1: f64) -> Surface {
        let n = 24;
        let grid: Vec<Vec<[f64; 3]>> = (0..n)
            .map(|i| {
                (0..n)
                    .map(|j| {
                        let x = x0 + (x1 - x0) * i as f64 / (n - 1) as f64;
                        let y = y0 + (y1 - y0) * j as f64 / (n - 1) as f64;
                        [x, y, 12.0 + 2.0 * (x / 10.0).sin() * (y / 8.0).cos()]
                    })
                    .collect()
            })
            .collect();
        fit_grid(&grid, 10, 10, 3).unwrap()
    }

    #[test]
    fn plane_fillet_hole_and_freeform_make_one_solid() {
        // a 40 × 30 block (z 0..~12, wavy top) with filleted vertical edges r = 4 and a hole r = 3
        let mut cuts = vec![];
        for (p, n) in [
            ([0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ([40.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ([0.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
            ([0.0, 30.0, 0.0], [0.0, 1.0, 0.0]),
            ([0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
        ] {
            cuts.push(Cut { shape: Shape::Plane { point: p, normal: n }, convex: true, full: false });
        }
        for c in [[4.0, 4.0], [36.0, 4.0], [4.0, 26.0], [36.0, 26.0]] {
            cuts.push(Cut {
                shape: Shape::Cylinder { point: [c[0], c[1], 0.0], axis: [0.0, 0.0, 1.0], radius: 4.0 },
                convex: true,
                full: false,
            });
        }
        cuts.push(Cut {
            shape: Shape::Cylinder { point: [20.0, 15.0, 0.0], axis: [0.0, 0.0, 1.0], radius: 3.0 },
            convex: false,
            full: true,
        });
        let top = wave_surface(-3.0, 43.0, -3.0, 33.0);
        let free = [FreeCut { surface: top, outward: [0.0, 0.0, 1.0] }];
        let built = solid_from_cuts([-6.0, -6.0, -3.0], [46.0, 36.0, 20.0], &cuts, &free).unwrap();
        for s in built.primitives.iter().chain(&built.freeform) {
            assert!(matches!(s, CutStatus::Applied(_)), "{s:?}");
        }
        // planes + 4 fillets + hole wall + freeform top
        assert_eq!(built.brep.faces.len(), 5 + 4 + 1 + 1, "faces");
        let v = solid::volume(&built.brep).unwrap();
        // footprint (rounded rectangle) minus the hole, times the mean height ≈ 12
        let area = 40.0 * 30.0 - (4.0 - std::f64::consts::PI) * 16.0 - std::f64::consts::PI * 9.0;
        assert!(v > area * 11.0 && v < area * 13.0, "volume {v}");
        // the faces map back to their shapes
        for c in &cuts {
            assert_eq!(faces_on_shape(&built.brep, &c.shape, 1e-3).len(), 1, "{:?}", c.shape);
        }
        // extract the top as a sheet
        let top_face = faces_on_spline(&built.brep, &free[0].surface, 0.05).unwrap();
        assert_eq!(top_face.len(), 1);
        let sheet = extract_face(&built.brep, top_face[0]).unwrap();
        assert_eq!(sheet.faces.len(), 1);
        // replace the flat top of a block by the same patch gives the same volume as the CAD cut
        let block = solid::make_box([0.0, 0.0, 0.0], [40.0, 30.0, 20.0]).unwrap();
        let top_idx = faces_on_shape(&block, &Shape::Plane { point: [0.0, 0.0, 20.0], normal: [0.0, 0.0, 1.0] }, 1e-6);
        assert_eq!(top_idx.len(), 1);
        let patch = wave_surface(-3.0, 43.0, -3.0, 33.0);
        let r = replace_face(&block, top_idx[0], &patch).unwrap();
        assert_eq!(r.faces.len(), 6);
        let rv = solid::volume(&r).unwrap();
        assert!(rv > 40.0 * 30.0 * 11.0 && rv < 40.0 * 30.0 * 13.0, "replaced volume {rv}");
        let _ = sheet_from_spline;
    }

    #[test]
    fn revolved_part_cone_and_through_hole() {
        // cylinder r = 12, h = 30 with a 3 mm chamfer on the top edge and a through hole r = 4
        let cuts = vec![
            Cut { shape: Shape::Plane { point: [0.0; 3], normal: [0.0, 0.0, -1.0] }, convex: true, full: false },
            Cut { shape: Shape::Plane { point: [0.0, 0.0, 30.0], normal: [0.0, 0.0, 1.0] }, convex: true, full: false },
            Cut {
                shape: Shape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: 12.0 },
                convex: true,
                full: true,
            },
            Cut {
                shape: Shape::Cone {
                    apex: [0.0, 0.0, 39.0],
                    axis: [0.0, 0.0, -1.0],
                    half_angle: std::f64::consts::FRAC_PI_4,
                },
                convex: true,
                full: true,
            },
            Cut {
                shape: Shape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: 4.0 },
                convex: false,
                full: true,
            },
        ];
        let built = solid_from_cuts([-20.0, -20.0, -5.0], [20.0, 20.0, 40.0], &cuts, &[]).unwrap();
        assert_eq!(built.brep.faces.len(), 5);
        let text = crate::step::export_solids(&[("p", &built.brep)]).unwrap();
        let back = crate::step::import(text.as_bytes()).unwrap();
        // one body back (a cone record that narrows along z is split into pieces by solidcore's STEP)
        assert_eq!(back.len(), 1);
        assert_eq!(back[0].faces.len(), 5);
        let v = solid::volume(&built.brep).unwrap();
        let pi = std::f64::consts::PI;
        // cylinder minus the hole, minus the chamfer ring (a triangle 3 × 3 revolved at r = 11)
        let exact = pi * (144.0 - 16.0) * 30.0 - 2.0 * pi * 11.0 * 4.5;
        assert!((v - exact).abs() < 1e-3 * exact, "volume {v} vs {exact}");
    }

    #[test]
    fn bad_input_is_refused() {
        assert!(solid_from_cuts([0.0; 3], [1.0; 3], &[], &[]).is_err());
        assert!(solid_from_cuts([f64::NAN, 0.0, 0.0], [1.0; 3], &[], &[]).is_err());
        let bad = Cut { shape: Shape::Sphere { center: [0.0; 3], radius: -1.0 }, convex: true, full: true };
        assert!(solid_from_cuts([0.0; 3], [1.0; 3], &[bad], &[]).is_err());
        let b = solid::make_box([0.0; 3], [1.0; 3]).unwrap();
        assert!(extract_face(&b, 99).is_err());
        assert!(replace_face(&b, 99, &wave_surface(0.0, 1.0, 0.0, 1.0)).is_err());
    }
}

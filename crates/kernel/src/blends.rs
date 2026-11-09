//! Blends of the hybrid model (plan P3b, Z3): the partial cylinders, cones, tori and spheres of a
//! scan are the fillets, chamfers and corner patches of the part's edges. Each is found as an edge
//! of the solid built so far and given to the kernel's `blend` (fillets of one radius in one call,
//! so that the kernel makes the corner patch where three of them meet), `chamfer_sides` (a cone) or
//! left to the kernel's result (a sphere that is the corner patch of three equal rounds).
//!
//! The edge a surface rounds is identified from the solid's own topology: the two faces that
//! share it, their planes, cylinders and orientation, never from the scan.
use crate::Result;
use crate::hybrid::{
    Cut, CutStatus, Shape, V, add, axis_distance, dot, face_vertices, frame_axis, norm, planar_face_outward, scale,
    sub, unit,
};
use crate::record::brep::{Brep, Curve, Surface};
use crate::solid::{self, decode, encode};
use solidcore_ops as ops;

/// The faces on either side of an edge (record indices, each once).
fn edge_faces(rec: &Brep, e: usize) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::new();
    if let Some(ed) = rec.edges.get(e) {
        for u in &ed.uses {
            let f = u.face as usize;
            if !out.contains(&f) {
                out.push(f);
            }
        }
    }
    out
}

/// A planar face: a point on it and its outward unit normal.
fn face_plane(rec: &Brep, f: usize) -> Option<(V, V)> {
    let origin = match &rec.faces.get(f)?.surface {
        Surface::Plane { origin, .. } | Surface::PlaneThrough { origin, .. } => *origin,
        _ => return None,
    };
    Some((origin, planar_face_outward(rec, f).ok()?))
}

/// A cylindrical face (a point of its axis, unit axis, radius). A blend's cylinder comes back as a
/// cone of half angle zero.
fn face_cylinder(rec: &Brep, f: usize) -> Option<(V, V, f64)> {
    match &rec.faces.get(f)?.surface {
        Surface::Cylinder { frame, radius } => Some((frame.origin, unit(frame_axis(frame))?, *radius)),
        Surface::Cone { frame, radius_at_origin, half_angle } if half_angle.abs() < 1e-9 => {
            Some((frame.origin, unit(frame_axis(frame))?, *radius_at_origin))
        }
        _ => None,
    }
}

fn edge_ends(rec: &Brep, e: usize) -> Option<(V, V)> {
    let ed = rec.edges.get(e)?;
    let a = rec.vertices.get(ed.vertices[0] as usize)?.point;
    let b = rec.vertices.get(ed.vertices[1] as usize)?.point;
    Some((a, b))
}

/// Is the straight edge `e` between the planar faces `f1`, `f2` convex (the material lies behind
/// both planes at the edge)?
fn straight_edge_is_convex(rec: &Brep, e: usize, f1: usize, f2: usize) -> Option<bool> {
    let (a, b) = edge_ends(rec, e)?;
    let t = unit(sub(b, a))?;
    let (p1, n1) = face_plane(rec, f1)?;
    let (p2, n2) = face_plane(rec, f2)?;
    // the vertex of each face farthest from the edge's line: where that face's material is
    let far = |f: usize| -> Option<V> {
        face_vertices(rec, f).into_iter().max_by(|x, y| axis_distance(*x, a, t).total_cmp(&axis_distance(*y, a, t)))
    };
    let (w1, w2) = (far(f1)?, far(f2)?);
    // convex when each face's far point lies behind the other face's plane
    let s = dot(sub(w2, p1), n1) + dot(sub(w1, p2), n2);
    Some(s < 0.0)
}

/// The straight edge of two planar faces that a cylinder of `radius` about the axis through
/// `point` along `axis` rounds: both planes parallel to the axis at distance `radius` from it, the
/// axis behind both planes for a convex round (material inside the cylinder) and in front of both
/// for a concave one, and the edge convex or concave to match.
pub(crate) fn cylinder_fillet_edge(rec: &Brep, point: V, axis: V, radius: f64, convex: bool) -> Option<usize> {
    let a = unit(axis)?;
    let sgn = if convex { -1.0 } else { 1.0 };
    let tol = 0.08 * radius + 0.1;
    let mut best: Option<(f64, usize)> = None;
    for (i, e) in rec.edges.iter().enumerate() {
        if !matches!(e.curve, Curve::Line { .. }) {
            continue;
        }
        let faces = edge_faces(rec, i);
        let [f1, f2] = faces.as_slice() else { continue };
        let (Some((p1, n1)), Some((p2, n2))) = (face_plane(rec, *f1), face_plane(rec, *f2)) else { continue };
        if dot(n1, a).abs() > 0.05 || dot(n2, a).abs() > 0.05 || dot(n1, n2).abs() > 0.95 {
            continue;
        }
        if (dot(sub(point, p1), n1) - sgn * radius).abs() > tol || (dot(sub(point, p2), n2) - sgn * radius).abs() > tol
        {
            continue;
        }
        if straight_edge_is_convex(rec, i, *f1, *f2) != Some(convex) {
            continue;
        }
        let Some((p, q)) = edge_ends(rec, i) else { continue };
        let d = axis_distance(scale(add(p, q), 0.5), point, a);
        if best.is_none_or(|b| d < b.0) {
            best = Some((d, i));
        }
    }
    best.map(|b| b.1)
}

/// A circular edge between a plane face (normal along `axis`) and a cylindrical face about
/// `axis`: (edge, plane face, plane point, plane outward normal, circle centre, radius).
struct CircleEdge {
    edge: usize,
    plane_face: usize,
    plane: (V, V),
    center: V,
    radius: f64,
    cyl_radius: f64,
}

fn circle_edges(rec: &Brep, on: V, axis: V, axis_tol: f64) -> Vec<CircleEdge> {
    let mut out = Vec::new();
    for (i, e) in rec.edges.iter().enumerate() {
        let Curve::Circle { frame, radius } = &e.curve else { continue };
        let Some(ea) = unit(frame_axis(frame)) else { continue };
        if dot(ea, axis).abs() < 0.99 || axis_distance(frame.origin, on, axis) > axis_tol {
            continue;
        }
        let faces = edge_faces(rec, i);
        let [f1, f2] = faces.as_slice() else { continue };
        for (pf, cf) in [(*f1, *f2), (*f2, *f1)] {
            let (Some(pl), Some((co, ca, cr))) = (face_plane(rec, pf), face_cylinder(rec, cf)) else { continue };
            if dot(pl.1, axis).abs() < 0.99 || dot(ca, axis).abs() < 0.99 || axis_distance(co, on, axis) > axis_tol {
                continue;
            }
            out.push(CircleEdge {
                edge: i,
                plane_face: pf,
                plane: pl,
                center: frame.origin,
                radius: *radius,
                cyl_radius: cr,
            });
        }
    }
    out
}

/// The circular edge a torus rounds: between a plane tangent to the tube and a cylinder about the
/// torus's axis (the tube's centre circle lies `minor` outside the cylinder for a concave round at
/// the base of a boss, `minor` inside it for a convex round at the rim of a disc).
pub(crate) fn torus_edge(rec: &Brep, center: V, axis: V, major: f64, minor: f64, convex: bool) -> Option<usize> {
    let a = unit(axis)?;
    let sgn = if convex { -1.0 } else { 1.0 };
    let tol = 0.25 + 0.1 * minor;
    let want = if convex { major + minor } else { major - minor };
    let mut best: Option<(f64, usize)> = None;
    for c in circle_edges(rec, center, a, 0.3 + 0.1 * minor) {
        let off = (dot(sub(center, c.plane.0), c.plane.1) - sgn * minor).abs();
        let rad = (c.cyl_radius - want).abs();
        if off > tol || rad > tol || (c.radius - c.cyl_radius).abs() > 0.1 + 0.05 * minor {
            continue;
        }
        let score = off + rad;
        if best.is_none_or(|b| score < b.0) {
            best = Some((score, c.edge));
        }
    }
    best.map(|b| b.1)
}

/// The circular edge a cone chamfers, with the face to measure along and the two distances: the
/// cone meets the plane face at radius `rho` and the cylinder at radius `r` (`rho < r` for a
/// convex chamfer, `rho > r` for a countersink). `axis` points into the cone's opening.
pub(crate) fn cone_chamfer_edge(
    rec: &Brep,
    apex: V,
    axis: V,
    half: f64,
    convex: bool,
) -> Option<(usize, usize, f64, f64)> {
    let a = unit(axis)?;
    let t = half.tan();
    if !(t.is_finite() && t > 1e-3) {
        return None;
    }
    let mut best: Option<(f64, (usize, usize, f64, f64))> = None;
    for c in circle_edges(rec, apex, a, 0.5) {
        let s = dot(sub(c.center, apex), a);
        if s <= 1e-6 {
            continue;
        }
        let rho = s * t;
        if (convex && rho >= c.cyl_radius - 0.02) || (!convex && rho <= c.cyl_radius + 0.02) {
            continue;
        }
        let dp = (c.cyl_radius - rho).abs();
        let dc = dp / t;
        if dp < 0.02 || dc < 0.02 || dp > c.cyl_radius {
            continue;
        }
        if best.is_none_or(|b| dp < b.0) {
            best = Some((dp, (c.edge, c.plane_face, dp, dc)));
        }
    }
    best.map(|b| b.1)
}

/// Is there a face of a sphere of `radius` about `center` (a sphere, or the corner patch of three
/// equal rounds) in `rec`?
fn has_ball_face(rec: &Brep, center: V, radius: f64) -> bool {
    let tol = 0.15 * radius + 0.3;
    rec.faces.iter().any(|f| match &f.surface {
        Surface::Sphere { frame, radius: r } => (r - radius).abs() < tol && norm(sub(frame.origin, center)) < tol,
        Surface::BallPatch { center: c, radius: r } => (r - radius).abs() < tol && norm(sub(*c, center)) < tol,
        _ => false,
    })
}

fn pending(s: &CutStatus) -> bool {
    matches!(s, CutStatus::Skipped(_))
}

fn radius_of(c: &Cut) -> f64 {
    match &c.shape {
        Shape::Cylinder { radius, .. } => *radius,
        Shape::Torus { minor_radius, .. } => *minor_radius,
        Shape::Sphere { radius, .. } => *radius,
        _ => 0.0,
    }
}

/// Applies the blends among `cuts` (partial cylinders, cones, tori, spheres) that are still
/// pending in `status`. With `last` unset only the cylinders are tried and failures stay pending
/// (a later pass sees the solid with its bosses); with `last` set every blend is tried and the
/// ones that fail get their reason.
pub(crate) fn blend_pass(mut body: ops::Body, cuts: &[Cut], status: &mut [CutStatus], last: bool) -> Result<ops::Body> {
    fn set(status: &mut [CutStatus], i: usize, s: CutStatus) {
        if let Some(x) = status.get_mut(i) {
            *x = s;
        }
    }
    // --- cylinders, grouped by radius, largest first: one blend call per group, so that the
    // kernel makes the corner patch where three rounds of one radius meet
    let mut cyl: Vec<usize> = cuts
        .iter()
        .enumerate()
        .filter(|(i, c)| matches!(c.shape, Shape::Cylinder { .. }) && !c.full && status.get(*i).is_some_and(pending))
        .map(|(i, _)| i)
        .collect();
    cyl.sort_by(|&x, &y| {
        let (rx, ry) = (cuts.get(x).map_or(0.0, radius_of), cuts.get(y).map_or(0.0, radius_of));
        ry.total_cmp(&rx)
    });
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for i in cyl {
        let r = cuts.get(i).map_or(0.0, radius_of);
        match groups.last_mut() {
            Some(g) if g.first().is_some_and(|&f| cuts.get(f).map_or(0.0, radius_of) - r <= 0.08 * r + 0.1) => {
                g.push(i);
            }
            _ => groups.push(vec![i]),
        }
    }
    let find = |rec: &Brep, c: &Cut| match &c.shape {
        Shape::Cylinder { point, axis, radius } => cylinder_fillet_edge(rec, *point, *axis, *radius, c.convex),
        _ => None,
    };
    for g in groups {
        let rec = encode(&body)?;
        let mut found: Vec<(usize, usize)> = Vec::new();
        for &i in &g {
            match cuts.get(i).and_then(|c| find(&rec, c)) {
                Some(e) => found.push((i, e)),
                None if last => set(
                    status,
                    i,
                    CutStatus::Skipped("no edge of two planes tangent to this cylinder was found".into()),
                ),
                None => {}
            }
        }
        if found.is_empty() {
            continue;
        }
        let mut edges: Vec<usize> = found.iter().map(|f| f.1).collect();
        edges.sort_unstable();
        edges.dedup();
        let mean = found.iter().map(|f| cuts.get(f.0).map_or(0.0, radius_of)).sum::<f64>() / found.len() as f64;
        match solid::fillet(&rec, &edges, mean).and_then(|b| decode(&b)) {
            Ok(b) => {
                body = b;
                for &(i, _) in &found {
                    set(status, i, CutStatus::Applied("fillet"));
                }
            }
            Err(_) => {
                // one by one: the kernel may refuse the combination but take each round
                for &(i, _) in &found {
                    let Some(c) = cuts.get(i) else { continue };
                    let rec = encode(&body)?;
                    let Some(e) = find(&rec, c) else {
                        set(status, i, CutStatus::Skipped("its edge was consumed by an earlier blend".into()));
                        continue;
                    };
                    match solid::fillet(&rec, &[e], radius_of(c)).and_then(|b| decode(&b)) {
                        Ok(b) => {
                            body = b;
                            set(status, i, CutStatus::Applied("fillet"));
                        }
                        Err(err) => set(status, i, CutStatus::Skipped(format!("the kernel refused the fillet: {err}"))),
                    }
                }
            }
        }
    }
    if !last {
        return Ok(body);
    }
    // --- tori: the circular fillet of a boss's base or a disc's rim, largest first
    let mut tori: Vec<usize> = cuts
        .iter()
        .enumerate()
        .filter(|(i, c)| matches!(c.shape, Shape::Torus { .. }) && !c.full && status.get(*i).is_some_and(pending))
        .map(|(i, _)| i)
        .collect();
    tori.sort_by(|&x, &y| {
        let (rx, ry) = (cuts.get(x).map_or(0.0, radius_of), cuts.get(y).map_or(0.0, radius_of));
        ry.total_cmp(&rx)
    });
    for i in tori {
        let Some(c) = cuts.get(i) else { continue };
        let Shape::Torus { center, axis, major_radius, minor_radius } = &c.shape else { continue };
        let rec = encode(&body)?;
        let Some(e) = torus_edge(&rec, *center, *axis, *major_radius, *minor_radius, c.convex) else {
            set(
                status,
                i,
                CutStatus::Skipped(
                    "no circular edge of a plane and a coaxial cylinder is tangent to this torus".into(),
                ),
            );
            continue;
        };
        match solid::fillet(&rec, &[e], *minor_radius).and_then(|b| decode(&b)) {
            Ok(b) => {
                body = b;
                set(status, i, CutStatus::Applied("fillet (circular edge)"));
            }
            Err(err) => set(status, i, CutStatus::Skipped(format!("the kernel refused the circular fillet: {err}"))),
        }
    }
    // --- cones: a chamfer of a circular edge
    for (i, c) in cuts.iter().enumerate() {
        let Shape::Cone { apex, axis, half_angle } = &c.shape else { continue };
        if c.full || !status.get(i).is_some_and(pending) {
            continue;
        }
        let rec = encode(&body)?;
        let Some((e, along, dp, dc)) = cone_chamfer_edge(&rec, *apex, *axis, *half_angle, c.convex) else {
            set(
                status,
                i,
                CutStatus::Skipped("no circular edge of a plane and a coaxial cylinder meets this cone".into()),
            );
            continue;
        };
        match solid::chamfer_sides(&rec, &[e], &[along], dp, dc).and_then(|b| decode(&b)) {
            Ok(b) => {
                body = b;
                set(status, i, CutStatus::Applied("chamfer"));
            }
            Err(err) => set(status, i, CutStatus::Skipped(format!("the kernel refused the chamfer: {err}"))),
        }
    }
    // --- spheres: the corner patch the kernel made where equal rounds meet
    let rec = encode(&body)?;
    for (i, c) in cuts.iter().enumerate() {
        let Shape::Sphere { center, radius } = &c.shape else { continue };
        if c.full || !status.get(i).is_some_and(pending) {
            continue;
        }
        if has_ball_face(&rec, *center, *radius) {
            set(status, i, CutStatus::Applied("corner patch of equal rounds"));
        } else {
            set(status, i, CutStatus::Skipped("the solid has no corner patch of equal rounds at this sphere".into()));
        }
    }
    Ok(body)
}

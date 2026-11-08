// Adapted from SolidCraft (https://github.com/tangency-labs/solidcraft) at fcd3186,
// crates/kernel/src/iges_out.rs (IGES 5.3 B-rep export: manifold solids 186 and shells 514 of
// faces 510, loops 508, edge lists 504 and vertex lists 502; B-spline geometry 126/128, lines 110,
// arcs 100 and conics 104 under 124 frames; name properties 406; the fixed-column file layout).
// Copyright (c) 2026 SolidCraft contributors. Licensed under MIT OR Apache-2.0.
// Modified for SurfaceCraft (Apache-2.0 §4): works directly on solidcore-record without
// solidcraft-step's geometry layer; every face is written on the surface its (u, v) are on (its
// NURBS form when it has one, planes as exact bilinear 128 patches), so the record's loops keep
// their orientation; vertex loops; the outer loop found from the pcurves; no colours; a fixed
// time stamp (deterministic output, no clock on wasm); refusals as `IoError::Export`.
// Technical debt (design doc §10): replace with an IGES writer in solidcore if one is moved there.

//! IGES 5.3 export of record bodies, millimetres (design doc §10). Each bounded material region
//! becomes a manifold solid B-rep object (186) with its voids; every other face shell becomes an
//! open shell (514 form 2). Faces carry their exact geometry as B-spline surfaces (128) and
//! their edges as lines (110), arcs (100), conics (104) or B-spline curves (126). IGES loops run
//! counter-clockwise about their surface's normal, which is the record's convention for the
//! surface a face is parameterised on.

use std::collections::HashMap;
use std::fmt::Write as _;

use crate::IoError;
use surfacecraft_kernel::record::brep::{Brep, Curve, Curve2, Edge, Face, Loop, OrientedEdge, Shell, Surface};

type V3 = [f64; 3];

/// Largest directory or parameter sequence number the fixed columns hold.
const MAX_SEQUENCE: usize = 9_999_999;

fn bad(m: impl Into<String>) -> IoError {
    IoError::Export(format!("IGES: {}", m.into()))
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// A real in IGES free format (always with a decimal point; `{:?}` round-trips exactly).
fn real(x: f64) -> String {
    let x = if x.is_finite() { x } else { 0.0 };
    let s = format!("{x:?}").replace('e', "E");
    if s.contains('.') {
        s
    } else if let Some((m, e)) = s.split_once('E') {
        format!("{m}.E{e}")
    } else {
        format!("{s}.")
    }
}

/// A real from the input: refused unless finite.
fn num(x: f64) -> Result<String, IoError> {
    if x.is_finite() { Ok(real(x)) } else { Err(bad("non-finite number")) }
}

/// A Hollerith string (non-printable or non-ASCII characters replaced by `_`).
fn holl(s: &str) -> String {
    let clean: String = s.chars().map(|c| if c.is_ascii() && !c.is_ascii_control() { c } else { '_' }).collect();
    format!("{}H{clean}", clean.len())
}

struct Ent {
    typ: i64,
    form: i64,
    params: Vec<String>,
    xform: i64,
    label: String,
    dependent: bool,
}

struct W {
    ents: Vec<Ent>,
    /// The largest coordinate written (the global section's maximum coordinate).
    max_coord: f64,
}

impl W {
    /// Adds an entity; its directory-entry pointer.
    fn add(&mut self, typ: i64, form: i64, params: Vec<String>) -> i64 {
        self.ents.push(Ent { typ, form, params, xform: 0, label: String::new(), dependent: true });
        2 * self.ents.len() as i64 - 1
    }
    fn get_mut(&mut self, de: i64) -> Option<&mut Ent> {
        usize::try_from((de - 1) / 2).ok().and_then(|i| self.ents.get_mut(i))
    }
    /// Coordinates of a model point, noted for the maximum coordinate.
    fn coords(&mut self, p: V3) -> Result<[String; 3], IoError> {
        if !p.iter().all(|x| x.is_finite()) {
            return Err(bad("non-finite coordinate"));
        }
        self.max_coord = p.iter().fold(self.max_coord, |m, x| m.max(x.abs()));
        Ok(p.map(real))
    }
    /// Transformation (124) taking a frame's local coordinates to the model.
    fn frame(&mut self, o: V3, x: V3, y: V3) -> Result<i64, IoError> {
        let z = cross(x, y);
        let mut rows = Vec::with_capacity(12);
        for i in 0..3 {
            for v in [x[i], y[i], z[i], o[i]] {
                rows.push(num(v)?);
            }
        }
        Ok(self.add(124, 0, rows))
    }
}

/// Degree of a spline with `count` control points on `knots` (full, clamped or not).
fn degree(knots: &[f64], count: usize) -> Result<usize, IoError> {
    knots
        .len()
        .checked_sub(count.saturating_add(1))
        .filter(|d| *d >= 1 && count >= 2)
        .ok_or_else(|| bad(format!("{} knots for {count} control points", knots.len())))
}

/// Points and weights of homogeneous coefficients.
fn split_rational(h: &[[f64; 4]]) -> Result<(Vec<V3>, Vec<f64>), IoError> {
    let mut pts = Vec::with_capacity(h.len());
    let mut ws = Vec::with_capacity(h.len());
    for c in h {
        if !(c[3] > 0.0) || !c[3].is_finite() {
            return Err(bad("rational weight must be positive"));
        }
        pts.push([c[0] / c[3], c[1] / c[3], c[2] / c[3]]);
        ws.push(c[3]);
    }
    Ok((pts, ws))
}

/// A B-spline curve (126) on full `knots` over `[t0, t1]`; rational when `weights` is given.
fn bspline_curve(
    w: &mut W,
    knots: &[f64],
    weights: Option<&[f64]>,
    points: &[V3],
    [t0, t1]: [f64; 2],
) -> Result<i64, IoError> {
    let deg = degree(knots, points.len())?;
    let rational = weights.is_some_and(|ws| ws.iter().any(|x| (x - 1.0).abs() > 1e-12));
    let mut p = vec![
        (points.len() - 1).to_string(),
        deg.to_string(),
        "0".into(),
        "0".into(),
        if rational { "0" } else { "1" }.into(),
        "0".into(),
    ];
    for k in knots {
        p.push(num(*k)?);
    }
    match weights {
        Some(ws) if ws.len() == points.len() => {
            for x in ws {
                p.push(num(*x)?);
            }
        }
        Some(_) => return Err(bad("weights and control points differ in number")),
        None => p.extend(std::iter::repeat_n(real(1.0), points.len())),
    }
    for q in points {
        p.extend(w.coords(*q)?);
    }
    p.extend([num(t0)?, num(t1)?, real(0.0), real(0.0), real(0.0)]);
    Ok(w.add(126, 0, p))
}

fn curve_name(c: &Curve) -> &'static str {
    match c {
        Curve::Line { .. } => "line",
        Curve::Circle { .. } => "circle",
        Curve::Ellipse { .. } => "ellipse",
        Curve::Polyline { .. } => "polyline",
        Curve::Spline { .. } => "spline",
        Curve::RationalSpline { .. } => "rational spline",
        Curve::SurfaceIntersection { .. } => "surface intersection",
    }
}

fn surface_name(s: &Surface) -> &'static str {
    match s {
        Surface::Plane { .. } => "plane",
        Surface::PlaneThrough { .. } => "plane through points",
        Surface::Cylinder { .. } => "cylinder",
        Surface::Cone { .. } => "cone",
        Surface::Sphere { .. } => "sphere",
        Surface::Torus { .. } => "torus",
        Surface::BallPatch { .. } => "ball patch",
        Surface::Spline { .. } => "spline",
        Surface::RationalSpline { .. } => "rational spline",
        Surface::PlacedRevolution { .. } => "placed revolution",
    }
}

/// The edge `ei` as an IGES curve running from `a` to `b` (its vertices at interval min and
/// max); `closed` when they are one vertex. The DE pointer.
fn curve(w: &mut W, ei: u32, edge: &Edge, a: V3, b: V3, closed: bool) -> Result<i64, IoError> {
    let c = edge.parameters();
    match c {
        Curve::Line { .. } => {
            let (ca, cb) = (w.coords(a)?, w.coords(b)?);
            Ok(w.add(110, 0, ca.into_iter().chain(cb).collect()))
        }
        Curve::Circle { frame, .. } | Curve::Ellipse { frame, .. } => {
            let (o, x, y) = (frame.origin, frame.x_axis, frame.y_axis);
            let local = |q: V3| {
                let v = sub(q, o);
                [dot(v, x), dot(v, y)]
            };
            let (la, lb) = (local(a), if closed { local(a) } else { local(b) });
            let tr = w.frame(o, x, y)?;
            let de = match c {
                Curve::Ellipse { radii: [ra, rb], .. } => {
                    if !(*ra > 0.0 && *rb > 0.0) {
                        return Err(bad(format!("edge {ei}: ellipse radii must be positive")));
                    }
                    // A x² + C y² + F = 0 with F = -1: A = 1/ra², C = 1/rb².
                    let p = vec![
                        num(1.0 / (ra * ra))?,
                        real(0.0),
                        num(1.0 / (rb * rb))?,
                        real(0.0),
                        real(0.0),
                        real(-1.0),
                        real(0.0),
                        num(la[0])?,
                        num(la[1])?,
                        num(lb[0])?,
                        num(lb[1])?,
                    ];
                    w.add(104, 1, p)
                }
                _ => {
                    let p = vec![real(0.0), real(0.0), real(0.0), num(la[0])?, num(la[1])?, num(lb[0])?, num(lb[1])?];
                    w.add(100, 0, p)
                }
            };
            if let Some(ent) = w.get_mut(de) {
                ent.xform = tr;
            }
            Ok(de)
        }
        // Segment i on [i, i + 1]: a degree-1 B-spline on knots 0, 0, 1, …, n − 1, n − 1.
        Curve::Polyline { points } => {
            let n = points.len();
            let knots: Vec<f64> = std::iter::once(0.0)
                .chain((0..n).map(|i| i as f64))
                .chain(std::iter::once(n.saturating_sub(1) as f64))
                .collect();
            bspline_curve(w, &knots, None, points, edge.interval)
        }
        Curve::Spline { knots, points } => bspline_curve(w, knots, None, points, edge.interval),
        Curve::RationalSpline { knots, homogeneous } => {
            let (pts, ws) = split_rational(homogeneous)?;
            bspline_curve(w, knots, Some(&ws), &pts, edge.interval)
        }
        Curve::SurfaceIntersection { .. } => {
            Err(bad(format!("edge {ei}: a {} curve is not written (no exact IGES form)", curve_name(c))))
        }
    }
}

/// A rational or polynomial B-spline surface (128): `points[i][j]`, i along u.
fn bspline_surface(
    w: &mut W,
    u: &[f64],
    v: &[f64],
    points: &[Vec<V3>],
    weights: Option<&[Vec<f64>]>,
) -> Result<i64, IoError> {
    let nu = points.len();
    let nv = points.first().map_or(0, Vec::len);
    if points.iter().any(|r| r.len() != nv) {
        return Err(bad("control net is not rectangular"));
    }
    let (p, q) = (degree(u, nu)?, degree(v, nv)?);
    let wt = |i: usize, j: usize| weights.and_then(|w| w.get(i)?.get(j).copied());
    if weights.is_some() && (0..nu).any(|i| (0..nv).any(|j| wt(i, j).is_none())) {
        return Err(bad("weights and control points differ in number"));
    }
    let rational = (0..nu).any(|i| (0..nv).any(|j| wt(i, j).is_some_and(|x| (x - 1.0).abs() > 1e-12)));
    let mut par = vec![
        (nu - 1).to_string(),
        (nv - 1).to_string(),
        p.to_string(),
        q.to_string(),
        "0".into(),
        "0".into(),
        if rational { "0" } else { "1" }.into(),
        "0".into(),
        "0".into(),
    ];
    for k in u.iter().chain(v) {
        par.push(num(*k)?);
    }
    // The first index (u) runs fastest.
    for j in 0..nv {
        for i in 0..nu {
            par.push(num(wt(i, j).unwrap_or(1.0))?);
        }
    }
    for j in 0..nv {
        for row in points {
            let c = row.get(j).copied().ok_or_else(|| bad("control net is not rectangular"))?;
            par.extend(w.coords(c)?);
        }
    }
    let range = |k: &[f64], d: usize, n: usize| -> Result<(f64, f64), IoError> {
        match (k.get(d), k.get(n)) {
            (Some(a), Some(b)) if a < b => Ok((*a, *b)),
            _ => Err(bad("empty spline domain")),
        }
    };
    let ((u0, u1), (v0, v1)) = (range(u, p, nu)?, range(v, q, nv)?);
    par.extend([num(u0)?, num(u1)?, num(v0)?, num(v1)?]);
    Ok(w.add(128, 0, par))
}

/// The face's surface as the face's (u, v) are on it: its NURBS form when it has one, a plane as
/// the exact bilinear patch over the face's domain. The DE pointer.
fn surface(w: &mut W, fi: u32, face: &Face) -> Result<i64, IoError> {
    let s = face.parameters();
    let plane = |w: &mut W, o: V3, du: V3, dv: V3| -> Result<i64, IoError> {
        let [[u0, u1], [v0, v1]] = face.domain;
        if !(u0 < u1 && v0 < v1) {
            return Err(bad(format!("face {fi}: empty domain")));
        }
        let at = |u: f64, v: f64| [0, 1, 2].map(|k| o[k] + u * du[k] + v * dv[k]);
        let points = vec![vec![at(u0, v0), at(u0, v1)], vec![at(u1, v0), at(u1, v1)]];
        bspline_surface(w, &[u0, u0, u1, u1], &[v0, v0, v1, v1], &points, None)
    };
    match s {
        Surface::Spline { u_knots, v_knots, points } => bspline_surface(w, u_knots, v_knots, points, None),
        Surface::RationalSpline { u_knots, v_knots, homogeneous } => {
            let mut pts = Vec::with_capacity(homogeneous.len());
            let mut ws = Vec::with_capacity(homogeneous.len());
            for row in homogeneous {
                let (p, wr) = split_rational(row)?;
                pts.push(p);
                ws.push(wr);
            }
            bspline_surface(w, u_knots, v_knots, &pts, Some(&ws))
        }
        Surface::Plane { origin, u_axis, v_axis } => plane(w, *origin, *u_axis, *v_axis),
        Surface::PlaneThrough { origin, u_point, v_point } => {
            plane(w, *origin, sub(*u_point, *origin), sub(*v_point, *origin))
        }
        other => {
            Err(bad(format!("face {fi}: a {} surface without its NURBS form is not written yet", surface_name(other))))
        }
    }
}

/// The (u, v) points of a pcurve's control polygon.
fn pcurve_points(c: &Curve2) -> Vec<[f64; 2]> {
    match c {
        Curve2::Spline { points, .. } => points.clone(),
        Curve2::RationalSpline { homogeneous, .. } => {
            homogeneous.iter().filter(|h| h[2] > 0.0).map(|h| [h[0] / h[2], h[1] / h[2]]).collect()
        }
    }
}

/// Twice the signed area of a loop in (u, v), from its pcurves' control polygons: positive for a
/// counter-clockwise (outer) loop.
fn loop_area(uses: &[OrientedEdge]) -> f64 {
    let mut pts: Vec<[f64; 2]> = Vec::new();
    for u in uses {
        let mut p = pcurve_points(&u.pcurve);
        if !u.forward {
            p.reverse();
        }
        pts.extend(p);
    }
    let n = pts.len();
    (0..n)
        .filter_map(|i| {
            let (a, b) = (pts.get(i)?, pts.get((i + 1) % n)?);
            Some(a[0] * b[1] - b[0] * a[1])
        })
        .sum()
}

/// The order loops are written in and whether the first is the outer one: a lone edge loop is
/// outer; among several, the one loop that runs counter-clockwise in (u, v).
fn loop_order(face: &Face) -> (Vec<usize>, bool) {
    let n = face.loops.len();
    if n == 1 {
        return (vec![0], matches!(face.loops.first(), Some(Loop::Edges { .. })));
    }
    let ccw: Vec<usize> = face
        .loops
        .iter()
        .enumerate()
        .filter_map(|(i, l)| match l {
            Loop::Edges { edges } if loop_area(edges) > 0.0 => Some(i),
            _ => None,
        })
        .collect();
    match ccw.as_slice() {
        [outer] => (std::iter::once(*outer).chain((0..n).filter(|i| i != outer)).collect(), true),
        _ => ((0..n).collect(), false),
    }
}

/// One loop entry: `(is_vertex, list index, along)`.
type Use = (bool, usize, bool);

/// The parameters of a shell (514): its faces, with their vertex list, edge list and loops
/// written first.
fn shell(w: &mut W, b: &Brep, si: usize) -> Result<Vec<String>, IoError> {
    let Some(Shell::Faces { faces, .. }) = b.shells.get(si) else {
        return Err(bad(format!("shell {si} is not a face shell")));
    };
    if faces.is_empty() {
        return Err(bad(format!("shell {si} has no faces")));
    }
    // Vertices and edges of the shell, in first-use order.
    let (mut vidx, mut verts): (HashMap<u32, usize>, Vec<V3>) = (HashMap::new(), Vec::new());
    let (mut eidx, mut edges): (HashMap<u32, usize>, Vec<u32>) = (HashMap::new(), Vec::new());
    let vertex = |v: u32, vidx: &mut HashMap<u32, usize>, verts: &mut Vec<V3>| -> Result<usize, IoError> {
        if let Some(k) = vidx.get(&v) {
            return Ok(*k);
        }
        let p = b.vertices.get(v as usize).ok_or_else(|| bad(format!("vertex {v} missing")))?.point;
        vidx.insert(v, verts.len());
        verts.push(p);
        Ok(verts.len() - 1)
    };
    let mut face_data = Vec::with_capacity(faces.len());
    for of in faces {
        let face = b.faces.get(of.face as usize).ok_or_else(|| bad(format!("face {} missing", of.face)))?;
        if face.loops.is_empty() {
            return Err(bad(format!("face {} has no loops", of.face)));
        }
        let (order, has_outer) = loop_order(face);
        let mut loops: Vec<Vec<Use>> = Vec::with_capacity(order.len());
        for li in order {
            match face.loops.get(li) {
                Some(Loop::Edges { edges: uses }) => {
                    let mut l = Vec::with_capacity(uses.len());
                    for u in uses {
                        let k = match eidx.get(&u.edge) {
                            Some(k) => *k,
                            None => {
                                let e = b
                                    .edges
                                    .get(u.edge as usize)
                                    .ok_or_else(|| bad(format!("edge {} missing", u.edge)))?;
                                for v in e.vertices {
                                    vertex(v, &mut vidx, &mut verts)?;
                                }
                                eidx.insert(u.edge, edges.len());
                                edges.push(u.edge);
                                edges.len() - 1
                            }
                        };
                        l.push((false, k, u.forward));
                    }
                    if l.is_empty() {
                        return Err(bad(format!("face {}: empty loop", of.face)));
                    }
                    loops.push(l);
                }
                Some(Loop::Vertex { vertex: v }) => loops.push(vec![(true, vertex(*v, &mut vidx, &mut verts)?, true)]),
                None => return Err(bad(format!("face {}: loop {li} missing", of.face))),
            }
        }
        face_data.push((of.face, face, of.forward, has_outer, loops));
    }
    let mut vp = vec![verts.len().to_string()];
    for v in &verts {
        vp.extend(w.coords(*v)?);
    }
    let vlist = w.add(502, 1, vp);
    // Edges run along their curves, from the record edge's start (interval min) to its end.
    let mut ep = vec![edges.len().to_string()];
    for &e in &edges {
        let edge = b.edges.get(e as usize).ok_or_else(|| bad(format!("edge {e} missing")))?;
        let lookup = |v: u32| vidx.get(&v).copied().ok_or_else(|| bad(format!("edge {e}: vertex {v} not listed")));
        let (sv, tv) = (lookup(edge.vertices[0])?, lookup(edge.vertices[1])?);
        let point = |k: usize| verts.get(k).copied().ok_or_else(|| bad(format!("edge {e}: vertex missing")));
        let (pa, pb) = (point(sv)?, point(tv)?);
        let cd = curve(w, e, edge, pa, pb, sv == tv)?;
        ep.extend([cd.to_string(), vlist.to_string(), (sv + 1).to_string(), vlist.to_string(), (tv + 1).to_string()]);
    }
    let elist = w.add(504, 1, ep);
    let mut sp = vec![face_data.len().to_string()];
    for (fi, face, forward, has_outer, loops) in face_data {
        let sd = surface(w, fi, face)?;
        let mut lids = Vec::with_capacity(loops.len());
        for l in loops {
            let mut lp = vec![l.len().to_string()];
            for (is_vertex, k, along) in l {
                let (kind, list) = if is_vertex { ("1", vlist) } else { ("0", elist) };
                lp.extend([
                    kind.into(),
                    list.to_string(),
                    (k + 1).to_string(),
                    if along { "1" } else { "0" }.into(),
                    "0".into(),
                ]);
            }
            lids.push(w.add(508, 1, lp).to_string());
        }
        let mut fp = vec![sd.to_string(), lids.len().to_string(), if has_outer { "1" } else { "0" }.into()];
        fp.extend(lids);
        let fd = w.add(510, 1, fp);
        // The record's `forward`: the face's normal in the shell is its surface's natural normal.
        sp.extend([fd.to_string(), if forward { "1" } else { "0" }.into()]);
    }
    Ok(sp)
}

/// An independent, named entity: its name as a name property (406 form 15).
fn top_level(w: &mut W, typ: i64, form: i64, mut params: Vec<String>, name: &str) -> i64 {
    let np = w.add(406, 15, vec!["1".into(), holl(name)]);
    params.extend(["0".into(), "1".into(), np.to_string()]);
    let d = w.add(typ, form, params);
    if let Some(ent) = w.get_mut(d) {
        ent.dependent = false;
        ent.label = name.chars().filter(char::is_ascii_alphanumeric).take(8).collect();
    }
    d
}

/// Solids (outer shell, void shells) and sheets (open face shells) of a record.
struct Layout {
    solids: Vec<(usize, Vec<usize>)>,
    sheets: Vec<usize>,
}

fn layout_of(bi: usize, b: &Brep) -> Result<Layout, IoError> {
    let to = |s: u32| usize::try_from(s).map_err(|_| bad("shell index"));
    let mut used = vec![false; b.shells.len()];
    let mark = |s: usize, used: &mut Vec<bool>| -> Result<(), IoError> {
        *used.get_mut(s).ok_or_else(|| bad(format!("body {bi}: shell {s} missing")))? = true;
        Ok(())
    };
    // Shells that bound a bounded region (its outer shell).
    let mut bounding = vec![false; b.shells.len()];
    for reg in b.regions.iter().skip(1) {
        if let Some(&s) = reg.shells.first()
            && let Some(x) = bounding.get_mut(to(s)?)
        {
            *x = true;
        }
    }
    let mut out = Layout { solids: Vec::new(), sheets: Vec::new() };
    for (ri, reg) in b.regions.iter().enumerate().skip(1) {
        if reg.void {
            continue;
        }
        let Some(&outer) = reg.shells.first() else { continue };
        let outer = to(outer)?;
        if !matches!(b.shells.get(outer), Some(Shell::Faces { .. })) {
            return Err(bad(format!("body {bi}: region {ri}'s outer shell is not a face shell")));
        }
        mark(outer, &mut used)?;
        let mut voids = Vec::new();
        for &s in reg.shells.iter().skip(1) {
            let s = to(s)?;
            if bounding.get(s).copied().unwrap_or(false) {
                mark(s, &mut used)?;
                voids.push(s);
            }
        }
        out.solids.push((outer, voids));
    }
    for (si, sh) in b.shells.iter().enumerate() {
        if used.get(si).copied().unwrap_or(true) {
            continue;
        }
        match sh {
            Shell::Faces { .. } => out.sheets.push(si),
            Shell::Wire { .. } => return Err(bad(format!("body {bi}: wire shell {si} is not written"))),
            Shell::Vertex { .. } => return Err(bad(format!("body {bi}: vertex shell {si} is not written"))),
        }
    }
    Ok(out)
}

/// IGES 5.3 text (millimetres) for record bodies: bounded material regions as manifold solid
/// B-rep objects (186), other face shells as open shells (514 form 2), each named `name`.
pub fn write_iges(name: &str, bodies: &[&Brep]) -> Result<String, IoError> {
    if bodies.is_empty() {
        return Err(bad("nothing to export"));
    }
    let mut w = W { ents: Vec::new(), max_coord: 1.0 };
    for (bi, b) in bodies.iter().enumerate() {
        b.validate().map_err(|e| bad(format!("body {bi}: {e}")))?;
        let items = layout_of(bi, b)?;
        // A closed shell is form 1, an open one form 2.
        for (outer, voids) in &items.solids {
            let params = shell(&mut w, b, *outer)?;
            let o = w.add(514, 1, params);
            let mut p = vec![o.to_string(), "1".into(), voids.len().to_string()];
            for &v in voids {
                let params = shell(&mut w, b, v)?;
                p.extend([w.add(514, 1, params).to_string(), "1".into()]);
            }
            top_level(&mut w, 186, 0, p, name);
        }
        for &si in &items.sheets {
            let params = shell(&mut w, b, si)?;
            top_level(&mut w, 514, 2, params, name);
        }
    }
    if w.ents.is_empty() {
        return Err(bad("nothing to export: no faces"));
    }
    layout(&w, name)
}

/// The file: start, global, directory and parameter sections and terminator, 80 columns.
fn layout(w: &W, name: &str) -> Result<String, IoError> {
    if 2 * w.ents.len() > MAX_SEQUENCE {
        return Err(bad("too many entities for the directory section"));
    }
    let mut out = String::new();
    let line = |out: &mut String, data: &str, sect: char, seq: usize| {
        let _ = writeln!(out, "{data:<72}{sect}{seq:>7}");
    };
    line(&mut out, "SurfaceCraft IGES export", 'S', 1);
    let stamp = holl("20260101.000000");
    let g = [
        "1H,".to_string(),
        "1H;".to_string(),
        holl(name),
        holl(&format!("{name}.igs")),
        holl("SurfaceCraft"),
        holl("SurfaceCraft IGES 5.3"),
        "32".into(),
        "38".into(),
        "6".into(),
        "308".into(),
        "15".into(),
        holl(name),
        real(1.0),
        "2".into(),
        holl("MM"),
        "1".into(),
        real(1.0),
        stamp.clone(),
        real(1e-6),
        real(w.max_coord),
        holl(""),
        holl(""),
        "11".into(),
        "0".into(),
        stamp,
    ];
    let mut gseq = 0;
    for chunk in wrap(&g.iter().map(String::as_str).collect::<Vec<_>>(), ',', ';', 72) {
        gseq += 1;
        line(&mut out, &chunk, 'G', gseq);
    }
    // Parameter lines per entity.
    let mut pdata: Vec<Vec<String>> = Vec::with_capacity(w.ents.len());
    for e in &w.ents {
        let mut fields = vec![e.typ.to_string()];
        fields.extend(e.params.iter().cloned());
        pdata.push(wrap(&fields.iter().map(String::as_str).collect::<Vec<_>>(), ',', ';', 64));
    }
    let total: usize = pdata.iter().map(Vec::len).sum();
    if total > MAX_SEQUENCE {
        return Err(bad("too many parameter lines"));
    }
    let f8 = |x: i64| format!("{x:>8}");
    let mut next = 1usize;
    for (k, (e, p)) in w.ents.iter().zip(&pdata).enumerate() {
        let status = if e.dependent { "00010000" } else { "00000000" };
        let a = format!(
            "{}{}{}{}{}{}{}{}{status}",
            f8(e.typ),
            f8(next as i64),
            f8(0),
            f8(0),
            f8(0),
            f8(0),
            f8(e.xform),
            f8(0)
        );
        let b = format!(
            "{}{}{}{}{}{}{}{:>8}{}",
            f8(e.typ),
            f8(0),
            f8(0),
            f8(p.len() as i64),
            f8(e.form),
            f8(0),
            f8(0),
            e.label,
            f8(0)
        );
        line(&mut out, &a, 'D', 2 * k + 1);
        line(&mut out, &b, 'D', 2 * k + 2);
        next += p.len();
    }
    let mut pseq = 0;
    for (k, p) in pdata.iter().enumerate() {
        for l in p {
            pseq += 1;
            line(&mut out, &format!("{l:<64}{:>8}", 2 * k + 1), 'P', pseq);
        }
    }
    line(&mut out, &format!("S{:>7}G{:>7}D{:>7}P{:>7}", 1, gseq, 2 * w.ents.len(), pseq), 'T', 1);
    Ok(out)
}

/// Fields joined by `delim`, ended by `end`, broken into lines of at most `width` columns
/// between fields (a longer field is split, as strings may be).
fn wrap(fields: &[&str], delim: char, end: char, width: usize) -> Vec<String> {
    let mut lines = Vec::new();
    let mut cur = String::new();
    for (i, f) in fields.iter().enumerate() {
        let sep = if i + 1 == fields.len() { end } else { delim };
        let item = format!("{f}{sep}");
        if cur.len() + item.len() > width && !cur.is_empty() {
            lines.push(std::mem::take(&mut cur));
        }
        if item.len() > width {
            let chars: Vec<char> = item.chars().collect();
            for c in chars.chunks(width) {
                let piece: String = c.iter().collect();
                if piece.len() == width {
                    lines.push(piece);
                } else {
                    cur = piece;
                }
            }
            continue;
        }
        cur.push_str(&item);
    }
    if !cur.is_empty() {
        lines.push(cur);
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_kernel::nurbs::{fit_grid, sheet_from_spline};

    fn sheet() -> Brep {
        let grid: Vec<Vec<[f64; 3]>> = (0..24)
            .map(|i| {
                (0..20)
                    .map(|j| {
                        let (x, y) = (60.0 * f64::from(i) / 23.0, 40.0 * f64::from(j) / 19.0);
                        [x, y, surfacecraft_geom::synth::wave_height(3.0, x, y)]
                    })
                    .collect()
            })
            .collect();
        sheet_from_spline(&fit_grid(&grid, 10, 9, 3).unwrap()).unwrap()
    }

    /// A directory entry: (type, parameter pointer, parameter line count, form).
    struct De {
        typ: i64,
        pstart: usize,
        pcount: usize,
        form: i64,
    }

    /// Checks the fixed layout and returns the directory entries and the parameter lines
    /// (data columns 1–64, DE back pointer).
    fn parse(text: &str) -> (Vec<De>, Vec<(String, usize)>) {
        let lines: Vec<&str> = text.lines().collect();
        let mut counts: HashMap<char, usize> = HashMap::new();
        let mut last = ' ';
        for l in &lines {
            assert_eq!(l.len(), 80, "line is not 80 columns: {l:?}");
            let sect = l.as_bytes()[72] as char;
            assert!("SGDPT".contains(sect), "{l:?}");
            assert!("SGDPT".find(sect) >= "SGDPT".find(last) || last == ' ', "sections out of order");
            last = sect;
            let seq: usize = l[73..].trim().parse().unwrap();
            let c = counts.entry(sect).or_default();
            *c += 1;
            assert_eq!(seq, *c, "sequence numbers of {sect}");
        }
        let t = lines.last().unwrap();
        assert_eq!(t.as_bytes()[72], b'T');
        assert_eq!(counts[&'T'], 1);
        let field = |i: usize| -> usize { t[i * 8 + 1..i * 8 + 8].trim().parse().unwrap() };
        assert_eq!(field(0), counts[&'S']);
        assert_eq!(field(1), counts[&'G']);
        assert_eq!(field(2), counts[&'D']);
        assert_eq!(field(3), counts[&'P']);
        let d: Vec<&&str> = lines.iter().filter(|l| l.as_bytes()[72] == b'D').collect();
        assert_eq!(d.len() % 2, 0);
        let f = |l: &str, i: usize| -> i64 { l[i * 8..i * 8 + 8].trim().parse().unwrap() };
        let des: Vec<De> = d
            .chunks(2)
            .map(|p| {
                assert_eq!(f(p[0], 0), f(p[1], 0), "both directory lines name one type");
                De { typ: f(p[0], 0), pstart: f(p[0], 1) as usize, pcount: f(p[1], 3) as usize, form: f(p[1], 4) }
            })
            .collect();
        let p: Vec<(String, usize)> = lines
            .iter()
            .filter(|l| l.as_bytes()[72] == b'P')
            .map(|l| (l[..64].to_string(), l[64..72].trim().parse().unwrap()))
            .collect();
        for (k, de) in des.iter().enumerate() {
            let first = &p[de.pstart - 1];
            assert_eq!(first.1, 2 * k + 1, "parameter line points back at its entry");
            let typ: i64 = first.0.split([',', ';']).next().unwrap().trim().parse().unwrap();
            assert_eq!(typ, de.typ, "entry {k}'s parameters start with its type");
            for i in 0..de.pcount {
                assert_eq!(p[de.pstart - 1 + i].1, 2 * k + 1);
            }
        }
        (des, p)
    }

    /// The numeric parameters of the entity at directory index `k`.
    fn params(des: &[De], p: &[(String, usize)], k: usize) -> Vec<f64> {
        let de = &des[k];
        let text: String = p[de.pstart - 1..de.pstart - 1 + de.pcount].iter().map(|l| l.0.as_str()).collect();
        let body = text.split(';').next().unwrap();
        body.split(',').skip(1).map(|s| s.trim().parse::<f64>().unwrap()).collect()
    }

    #[test]
    fn filleted_box_is_one_manifold_solid() {
        let b = surfacecraft_kernel::solid::make_box([0.0; 3], [20.0, 10.0, 5.0]).unwrap();
        let f = surfacecraft_kernel::solid::fillet(&b, &[0], 2.0).unwrap();
        assert_eq!(f.faces.len(), 7);
        let text = write_iges("filleted box", &[&f]).unwrap();
        let (des, p) = parse(&text);
        assert_eq!(des.iter().filter(|d| d.typ == 186).count(), 1);
        assert_eq!(des.iter().filter(|d| d.typ == 510).count(), 7);
        let shells: Vec<usize> = (0..des.len()).filter(|&k| des[k].typ == 514).collect();
        assert_eq!(shells.len(), 1);
        assert_eq!(des[shells[0]].form, 1);
        assert_eq!(params(&des, &p, shells[0])[0], 7.0, "the shell lists 7 faces");
        assert!(text.contains("2HMM"));
        // every face has its outer loop flagged
        for k in (0..des.len()).filter(|&k| des[k].typ == 510) {
            assert_eq!(params(&des, &p, k)[2], 1.0, "face {k} has an outer loop");
        }
        let kinds: std::collections::BTreeSet<i64> = des.iter().map(|d| d.typ).collect();
        eprintln!("filleted box IGES entity types: {kinds:?}");
        // every entity a 510 face names is a surface
        for k in (0..des.len()).filter(|&k| des[k].typ == 510) {
            let sd = params(&des, &p, k)[0] as usize;
            assert_eq!(des[(sd - 1) / 2].typ, 128);
        }
    }

    #[test]
    fn sheet_is_an_open_shell_with_exact_geometry() {
        let b = sheet();
        let text = write_iges("wave", &[&b]).unwrap();
        let (des, p) = parse(&text);
        let shells: Vec<usize> = (0..des.len()).filter(|&k| des[k].typ == 514).collect();
        assert_eq!(shells.len(), 1);
        assert_eq!(des[shells[0]].form, 2);
        assert_eq!(des.iter().filter(|d| d.typ == 186).count(), 0);
        assert_eq!(des.iter().filter(|d| d.typ == 126).count(), 4);
        // geometry round trip of the 128 surface
        let k = (0..des.len()).find(|&k| des[k].typ == 128).unwrap();
        let v = params(&des, &p, k);
        let Surface::Spline { u_knots, v_knots, points } = &b.faces[0].surface else { panic!("spline sheet") };
        let (k1, k2, m1, m2) = (v[0] as usize, v[1] as usize, v[2] as usize, v[3] as usize);
        assert_eq!((k1 + 1, k2 + 1), (points.len(), points[0].len()));
        assert_eq!(v[6], 1.0, "polynomial");
        let (nu, nv) = (k1 + 1, k2 + 1);
        let mut at = 9;
        let ku = &v[at..at + nu + m1 + 1];
        at += nu + m1 + 1;
        let kv = &v[at..at + nv + m2 + 1];
        at += nv + m2 + 1;
        assert!(ku.iter().zip(u_knots).all(|(a, b)| (a - b).abs() < 1e-12));
        assert!(kv.iter().zip(v_knots).all(|(a, b)| (a - b).abs() < 1e-12));
        assert!(v[at..at + nu * nv].iter().all(|w| *w == 1.0));
        at += nu * nv;
        let mut max: f64 = 0.0;
        for j in 0..nv {
            for (i, row) in points.iter().enumerate() {
                let q = &v[at + 3 * (j * nu + i)..at + 3 * (j * nu + i) + 3];
                for c in 0..3 {
                    max = max.max((q[c] - row[j][c]).abs());
                }
            }
        }
        assert!(max <= 1e-9, "control points differ by {max}");
        assert_eq!(v.len(), at + 3 * nu * nv + 4);
    }

    #[test]
    fn hostile_input_is_refused() {
        assert!(write_iges("x", &[]).is_err());
        // a face on a cylinder without its NURBS form
        let mut b = sheet();
        b.faces[0].surface = Surface::Cylinder {
            frame: surfacecraft_kernel::record::brep::Frame {
                origin: [0.0; 3],
                x_axis: [1.0, 0.0, 0.0],
                y_axis: [0.0, 1.0, 0.0],
            },
            radius: 1.0,
        };
        match write_iges("x", &[&b]) {
            Err(IoError::Export(m)) => assert!(m.contains("without its NURBS form"), "{m}"),
            other => panic!("{other:?}"),
        }
        // an edge on a procedural intersection curve has no exact IGES form
        let mut b = sheet();
        let plane = |z: f64| {
            Box::new(Surface::Plane { origin: [0.0, 0.0, z], u_axis: [1.0, 0.0, 0.0], v_axis: [0.0, 1.0, 0.0] })
        };
        let leader = Box::new(b.edges[0].curve.clone());
        b.edges[0].curve = Curve::SurfaceIntersection { surfaces: [plane(0.0), plane(1.0)], leader, tolerance: 1e-6 };
        match write_iges("x", &[&b]) {
            Err(IoError::Export(m)) => assert!(m.contains("surface intersection"), "{m}"),
            other => panic!("{other:?}"),
        }
        // dangling references and non-finite numbers
        let mut b = sheet();
        if let Loop::Edges { edges } = &mut b.faces[0].loops[0] {
            edges[0].edge = 99;
        }
        assert!(write_iges("x", &[&b]).is_err());
        let mut b = sheet();
        b.vertices[0].point[0] = f64::NAN;
        assert!(write_iges("x", &[&b]).is_err());
        let mut b = sheet();
        b.shells.clear();
        assert!(write_iges("x", &[&b]).is_err());
        // names are sanitised to printable ASCII
        let text = write_iges("ż\u{1}", &[&sheet()]).unwrap();
        assert!(text.contains("2H__"));
    }

    #[test]
    fn helpers() {
        assert_eq!(real(1.0), "1.0");
        assert_eq!(real(1e-20), "1.E-20");
        assert_eq!(holl("MM"), "2HMM");
        let w = wrap(&["a"; 40], ',', ';', 10);
        assert!(w.iter().all(|l| l.len() <= 10));
        assert_eq!(w.concat(), format!("{};", ["a"; 40].join(",")));
    }
}

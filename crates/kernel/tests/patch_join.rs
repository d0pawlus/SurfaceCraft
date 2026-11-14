//! Measurement (not a gate): can a trimmed `fit_patch` cap be sewn to the faces that end on the same
//! side curves into a closed shell that behaves as a solid?
//!
//! The scenario is a 40 x 30 x 20 block whose top face is replaced by a `fit_patch` cap whose four
//! sides are the block's four top edges, in three variants (flat, domed, curved top edges). For each
//! the test prints the patch report (gap of every side, control net), then for stitch tolerances of
//! 1e-6 to 1e-2 mm whether the pieces sew, whether the shell is closed, its volume against the exact
//! one, a STEP round trip and a boolean cut by a 10 x 10 x 30 box. Nothing is asserted: every kernel
//! error is printed as `FAILED: <error>` and the run goes on. Run it with
//! `cargo test -p surfacecraft-kernel --test patch_join -- --ignored --nocapture`.
//!
//! The kernel has no function that merges several sheet records into one record before `stitch`
//! (`extract_face` takes one face out of a body, `sheet_from_patches` and `sheet_from_bezier_network`
//! build only from patch grids and Bezier nets, `stitch` takes one record). Route A therefore
//! assembles the record by hand (this file), then calls `nurbs::stitch`. Route A1 uses the cap as
//! `fit_patch` returns it (one edge per outline corner, hundreds of them); route A2 first merges the
//! cap's edges into one edge per side. Route B cuts a taller block by the untrimmed patch surface
//! (`hybrid::cut_by_spline`), route C replaces the top face of the box (`hybrid::replace_face`); both
//! let the kernel make the common edges, so no stitching is involved.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::fmt::Display;
use surfacecraft_kernel::feature::{BoolKind, boolean};
use surfacecraft_kernel::hybrid::{cut_by_spline, replace_face};
use surfacecraft_kernel::nurbs::{SurfaceProbe, domain, is_closed, stitch};
use surfacecraft_kernel::patch_fit::{Conflicts, Continuity, Request, Side, fit_patch};
use surfacecraft_kernel::record::brep::{
    Brep, Curve, Curve2, Edge, EdgeUseRef, Face, Loop, OrientedEdge, OrientedFace, Region, Shell, Surface, Vertex,
};
use surfacecraft_kernel::{solid, step};

const SIZE: [f64; 3] = [40.0, 30.0, 20.0];
const TOLERANCES: [f64; 5] = [1e-6, 1e-5, 1e-4, 1e-3, 1e-2];
/// A loop vertex this close to a corner of the block starts a side (mm).
const CORNER_RADIUS: f64 = 0.05;

type Height = fn(f64, f64) -> f64;
type Top = fn(f64) -> f64;

struct Variant {
    name: &'static str,
    title: &'static str,
    /// Height of the cap over (x, y): the exact surface the data lies on.
    height: Height,
    /// Height of the top edges over x (the sides of the loop).
    top: Top,
    data: bool,
}

fn level(_x: f64) -> f64 {
    SIZE[2]
}

fn flat(_x: f64, _y: f64) -> f64 {
    SIZE[2]
}

fn dome(x: f64, y: f64) -> f64 {
    SIZE[2] + 2.0 * (1.0 - (x / 20.0 - 1.0).powi(2)) * (1.0 - (y / 15.0 - 1.0).powi(2))
}

/// Top edges of variant c: a parabola of rise 1.5 mm over the 40 mm, zero rise at both ends. (A sine
/// is not a bicubic; a parabola is, so the data and the cutter can be exact.)
fn arch_z(x: f64) -> f64 {
    let t = x / SIZE[0];
    SIZE[2] + 6.0 * t * (1.0 - t)
}

fn arch(x: f64, _y: f64) -> f64 {
    arch_z(x)
}

fn variants() -> Vec<Variant> {
    vec![
        Variant { name: "a", title: "flat cap, z = 20", height: flat, top: level, data: false },
        Variant { name: "b", title: "domed cap, rise 2 mm in the middle", height: dome, top: level, data: true },
        Variant {
            name: "c",
            title: "cap on curved sides, parabolic top edges, rise 1.5 mm",
            height: arch,
            top: arch_z,
            data: true,
        },
    ]
}

// ---------------------------------------------------------------------------------------------
// small helpers

fn msg<E: Display>(e: E) -> String {
    e.to_string()
}

/// Runs a kernel call, turning a panic into an error text so that the measurement goes on.
fn guard<T>(f: impl FnOnce() -> Result<T, String>) -> Result<T, String> {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(r) => r,
        Err(_) => Err("panic inside the kernel call".to_string()),
    }
}

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn seg_dist(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    let t = if len2 > 0.0 { ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2).clamp(0.0, 1.0) } else { 0.0 };
    dist3(p, [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]])
}

/// Composite Simpson integral of `h` over the rectangle (exact for a polynomial up to degree 3).
fn simpson(h: Height, x: [f64; 2], y: [f64; 2]) -> f64 {
    let n = 120usize;
    let w = |i: usize| {
        if i == 0 || i == n {
            1.0
        } else if i % 2 == 1 {
            4.0
        } else {
            2.0
        }
    };
    let (dx, dy) = ((x[1] - x[0]) / n as f64, (y[1] - y[0]) / n as f64);
    let mut sum = 0.0;
    for i in 0..=n {
        for j in 0..=n {
            sum += w(i) * w(j) * h(x[0] + i as f64 * dx, y[0] + j as f64 * dy);
        }
    }
    sum * dx * dy / 9.0
}

// ---------------------------------------------------------------------------------------------
// the fit request

fn loop_sides(top: Top) -> Vec<Side> {
    let (lx, ly) = (SIZE[0], SIZE[1]);
    let corners = [[0.0, 0.0], [lx, 0.0], [lx, ly], [0.0, ly]];
    let counts = [200usize, 150, 200, 150];
    (0..4)
        .map(|k| {
            let (a, b) = (corners[k], corners[(k + 1) % 4]);
            let points = (0..=counts[k])
                .map(|i| {
                    let t = i as f64 / counts[k] as f64;
                    let x = a[0] + t * (b[0] - a[0]);
                    [x, a[1] + t * (b[1] - a[1]), top(x)]
                })
                .collect();
            Side { points, continuity: Continuity::G0, reference: None, uv: None, soft: false }
        })
        .collect()
}

fn corners3d(top: Top) -> [[f64; 3]; 4] {
    let (lx, ly) = (SIZE[0], SIZE[1]);
    [[0.0, 0.0, top(0.0)], [lx, 0.0, top(lx)], [lx, ly, top(lx)], [0.0, ly, top(0.0)]]
}

fn request(v: &Variant) -> Request {
    let mut data = Vec::new();
    if v.data {
        for i in 1..40 {
            for j in 1..30 {
                let (x, y) = (f64::from(i), f64::from(j));
                data.push([x, y, (v.height)(x, y)]);
            }
        }
    }
    Request {
        sides: loop_sides(v.top),
        data,
        data_uv: None,
        hard_points: Vec::new(),
        soft_points: Vec::new(),
        curves: Vec::new(),
        tolerance: 0.01,
        smoothing: None,
        control: None,
        max_control: 16,
        conflicts: Conflicts::Relax,
        side_ids: Vec::new(),
    }
}

// ---------------------------------------------------------------------------------------------
// record assembly: the one thing the kernel API lacks

struct Part<'a> {
    body: &'a Brep,
    /// A face of the body left out.
    skip: Option<usize>,
    /// The face's use in the new shell (default: as in the body's shell).
    forward: Option<bool>,
}

/// The record's canonical order (what `Brep::validate` demands): edges by first appearance walking
/// the faces (backwards in a face used reversed), vertices by first appearance in the edges, every
/// edge's uses rebuilt in walk order. One open shell of all the faces, in the given order.
fn canonical(
    mut faces: Vec<Face>,
    dirs: Vec<bool>,
    edges: Vec<Edge>,
    vertices: Vec<Vertex>,
    tolerance: f64,
    version: u32,
) -> Result<Brep, String> {
    let mut new_of_edge = vec![u32::MAX; edges.len()];
    let mut order: Vec<usize> = Vec::new();
    let mut uses: Vec<Vec<EdgeUseRef>> = vec![Vec::new(); edges.len()];
    for (fi, f) in faces.iter().enumerate() {
        let forward = dirs.get(fi).copied().unwrap_or(true);
        for (li, l) in f.loops.iter().enumerate() {
            let Loop::Edges { edges: le } = l else { return Err("a vertex loop cannot be assembled".into()) };
            let n = le.len();
            for k in 0..n {
                let p = if forward { k } else { n - 1 - k };
                let u = le.get(p).ok_or("loop position")?;
                let e = u.edge as usize;
                let slot = new_of_edge.get_mut(e).ok_or("edge index out of range")?;
                if *slot == u32::MAX {
                    *slot = order.len() as u32;
                    order.push(e);
                }
                uses.get_mut(e).ok_or("edge index out of range")?.push(EdgeUseRef {
                    face: fi as u32,
                    r#loop: li as u32,
                    position: p as u32,
                    upper: !u.forward,
                });
            }
        }
    }
    for f in &mut faces {
        for l in &mut f.loops {
            if let Loop::Edges { edges: le } = l {
                for u in le {
                    u.edge = new_of_edge.get(u.edge as usize).copied().ok_or("edge index out of range")?;
                }
            }
        }
    }
    let mut new_edges: Vec<Edge> = Vec::with_capacity(order.len());
    for &e in &order {
        let mut ed = edges.get(e).ok_or("edge index out of range")?.clone();
        ed.uses = std::mem::take(uses.get_mut(e).ok_or("edge index out of range")?);
        new_edges.push(ed);
    }
    let mut new_of_vertex = vec![u32::MAX; vertices.len()];
    let mut vorder: Vec<usize> = Vec::new();
    for e in &new_edges {
        for &v in &e.vertices {
            let slot = new_of_vertex.get_mut(v as usize).ok_or("vertex index out of range")?;
            if *slot == u32::MAX {
                *slot = vorder.len() as u32;
                vorder.push(v as usize);
            }
        }
    }
    for e in &mut new_edges {
        for v in &mut e.vertices {
            *v = new_of_vertex.get(*v as usize).copied().ok_or("vertex index out of range")?;
        }
    }
    let mut new_vertices: Vec<Vertex> = Vec::with_capacity(vorder.len());
    for &v in &vorder {
        new_vertices.push(vertices.get(v).ok_or("vertex index out of range")?.clone());
    }
    let oriented: Vec<OrientedFace> =
        dirs.iter().enumerate().map(|(i, &forward)| OrientedFace { face: i as u32, forward }).collect();
    let brep = Brep {
        version,
        tolerance,
        regions: vec![Region { shells: vec![0], void: false }],
        shells: vec![Shell::Faces { faces: oriented, wires: vec![] }],
        faces,
        edges: new_edges,
        vertices: new_vertices,
    };
    brep.validate().map_err(|e| format!("the assembled record is invalid: {e}"))?;
    Ok(brep)
}

/// One open shell of the faces of all `parts` (minus the skipped ones), with their edges kept apart:
/// nothing is sewn here.
fn assemble(parts: &[Part]) -> Result<Brep, String> {
    let (mut faces, mut dirs): (Vec<Face>, Vec<bool>) = (Vec::new(), Vec::new());
    let (mut edges, mut vertices): (Vec<Edge>, Vec<Vertex>) = (Vec::new(), Vec::new());
    let (mut tolerance, mut version) = (0.0f64, 0u32);
    for part in parts {
        let b = part.body;
        let (e0, v0) = (edges.len() as u32, vertices.len() as u32);
        tolerance = tolerance.max(b.tolerance);
        version = b.version;
        for e in &b.edges {
            let mut e = e.clone();
            e.uses.clear();
            e.vertices = [e.vertices[0] + v0, e.vertices[1] + v0];
            edges.push(e);
        }
        vertices.extend(b.vertices.iter().cloned());
        for shell in &b.shells {
            let Shell::Faces { faces: oriented, .. } = shell else { continue };
            for of in oriented {
                if part.skip == Some(of.face as usize) {
                    continue;
                }
                let mut f = b.faces.get(of.face as usize).ok_or("face index out of range")?.clone();
                for l in &mut f.loops {
                    match l {
                        Loop::Edges { edges: le } => {
                            for oe in le {
                                oe.edge += e0;
                            }
                        }
                        Loop::Vertex { vertex } => *vertex += v0,
                    }
                }
                faces.push(f);
                dirs.push(part.forward.unwrap_or(of.forward));
            }
        }
    }
    canonical(faces, dirs, edges, vertices, tolerance, version)
}

fn face_points(b: &Brep, fi: usize) -> Vec<[f64; 3]> {
    let mut out = Vec::new();
    if let Some(f) = b.faces.get(fi) {
        for l in &f.loops {
            if let Loop::Edges { edges } = l {
                for oe in edges {
                    if let Some(e) = b.edges.get(oe.edge as usize) {
                        out.extend(e.vertices.iter().filter_map(|&v| b.vertices.get(v as usize)).map(|p| p.point));
                    }
                }
            }
        }
    }
    out
}

/// The face of the box whose vertices all lie at the top height.
fn find_top(b: &Brep) -> Result<usize, String> {
    (0..b.faces.len())
        .find(|&fi| {
            let pts = face_points(b, fi);
            !pts.is_empty() && pts.iter().all(|p| (p[2] - SIZE[2]).abs() < 1e-6)
        })
        .ok_or_else(|| "no face of the box lies at the top height".to_string())
}

/// The cap's edges merged into one edge per side: consecutive edges between two corners of the
/// block become one degree-1 spline through all their points (and one pcurve through all theirs).
fn merge_sides(cap: &Brep, corners: &[[f64; 3]; 4]) -> Result<Brep, String> {
    let face = cap.faces.first().ok_or("the cap has no face")?;
    let [Loop::Edges { edges: le }] = face.loops.as_slice() else { return Err("expected one edge loop".into()) };
    let start_of = |oe: &OrientedEdge| -> Result<u32, String> {
        let e = cap.edges.get(oe.edge as usize).ok_or("edge index out of range")?;
        if oe.forward { Ok(e.vertices[0]) } else { Err("a reversed edge use".to_string()) }
    };
    let near_corner = |v: u32| {
        cap.vertices.get(v as usize).is_some_and(|p| corners.iter().any(|c| dist3(p.point, *c) <= CORNER_RADIUS))
    };
    let n = le.len();
    let mut first = None;
    for (i, oe) in le.iter().enumerate() {
        if near_corner(start_of(oe)?) {
            first = Some(i);
            break;
        }
    }
    let i0 = first.ok_or("no loop vertex lies near a corner")?;
    let mut groups: Vec<Vec<&OrientedEdge>> = Vec::new();
    for k in 0..n {
        let oe = le.get((i0 + k) % n).ok_or("loop position")?;
        if near_corner(start_of(oe)?) {
            groups.push(Vec::new());
        }
        groups.last_mut().ok_or("no group")?.push(oe);
    }
    if groups.len() != corners.len() {
        return Err(format!("found {} corner groups, expected {}", groups.len(), corners.len()));
    }
    let (mut vertices, mut edges, mut uses) = (Vec::new(), Vec::new(), Vec::new());
    for (gi, g) in groups.iter().enumerate() {
        let (mut pts, mut uv): (Vec<[f64; 3]>, Vec<[f64; 2]>) = (Vec::new(), Vec::new());
        let mut tol = 0.0f64;
        for oe in g {
            let e = cap.edges.get(oe.edge as usize).ok_or("edge index out of range")?;
            let Curve::Spline { points, .. } = &e.curve else { return Err("an edge is not a spline".into()) };
            let Curve2::Spline { points: pc, .. } = &oe.pcurve else { return Err("a pcurve is not a spline".into()) };
            if points.len() != pc.len() {
                return Err("an edge and its pcurve differ in size".into());
            }
            let skip = usize::from(!pts.is_empty());
            pts.extend(points.iter().skip(skip).copied());
            uv.extend(pc.iter().skip(skip).copied());
            tol = tol.max(e.tolerance);
        }
        let m = pts.len().checked_sub(1).filter(|m| *m >= 1).ok_or("a side has no segment")?;
        let mut knots = vec![0.0];
        knots.extend((0..=m).map(|i| i as f64));
        knots.push(m as f64);
        let head = g.first().ok_or("empty group")?;
        let v0 = start_of(head)?;
        vertices.push(cap.vertices.get(v0 as usize).ok_or("vertex index out of range")?.clone());
        edges.push(Edge {
            curve: Curve::Spline { knots: knots.clone(), points: pts },
            nurbs: None,
            interval: [0.0, m as f64],
            vertices: [gi as u32, ((gi + 1) % groups.len()) as u32],
            tolerance: tol,
            uses: Vec::new(),
        });
        uses.push(OrientedEdge { edge: gi as u32, forward: true, pcurve: Curve2::Spline { knots, points: uv } });
    }
    let merged = Face {
        surface: face.surface.clone(),
        nurbs: face.nurbs.clone(),
        domain: face.domain,
        loops: vec![Loop::Edges { edges: uses }],
    };
    canonical(vec![merged], vec![true], edges, vertices, cap.tolerance, cap.version)
}

/// Does the natural normal of the cap point up (+z) at the middle of its domain?
fn faces_up(surface: &Surface) -> Result<bool, String> {
    let d = domain(surface).map_err(msg)?;
    let probe = SurfaceProbe::new(surface).map_err(msg)?;
    let (u, v) = (0.5 * (d[0][0] + d[0][1]), 0.5 * (d[1][0] + d[1][1]));
    let (du, dv) = (1e-3 * (d[0][1] - d[0][0]), 1e-3 * (d[1][1] - d[1][0]));
    let (a, b) = (probe.eval(u + du, v).map_err(msg)?, probe.eval(u - du, v).map_err(msg)?);
    let (c, e) = (probe.eval(u, v + dv).map_err(msg)?, probe.eval(u, v - dv).map_err(msg)?);
    let tu = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let tv = [c[0] - e[0], c[1] - e[1], c[2] - e[2]];
    Ok(tu[0] * tv[1] - tu[1] * tv[0] > 0.0)
}

// ---------------------------------------------------------------------------------------------
// the open shells

/// The block's cutter of variant c: the arch extruded along y, as an exact bicubic.
fn arch_cutter() -> Surface {
    let (x0, x1, y0, y1) = (-5.0, 45.0, -5.0, 35.0);
    let (z0, zm, z1) = (arch_z(x0), arch_z(0.5 * (x0 + x1)), arch_z(x1));
    let zc = 2.0 * zm - 0.5 * (z0 + z1);
    let zs = [z0, (z0 + 2.0 * zc) / 3.0, (2.0 * zc + z1) / 3.0, z1];
    let points: Vec<Vec<[f64; 3]>> = (0..4)
        .map(|i| (0..4).map(|j| [x0 + (x1 - x0) * i as f64 / 3.0, y0 + (y1 - y0) * j as f64 / 3.0, zs[i]]).collect())
        .collect();
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    Surface::Spline { u_knots: knots.clone(), v_knots: knots, points }
}

/// The five other faces of the block (variants a and b), or of the block with the arched top (c).
fn open_shell(v: &Variant) -> Result<Brep, String> {
    if v.name != "c" {
        let b = guard(|| solid::make_box([0.0; 3], SIZE).map_err(msg))?;
        let top = find_top(&b)?;
        return assemble(&[Part { body: &b, skip: Some(top), forward: None }]);
    }
    let tall = guard(|| solid::make_box([0.0; 3], [SIZE[0], SIZE[1], 25.0]).map_err(msg))?;
    let cutter = arch_cutter();
    let cut = guard(|| cut_by_spline(&tall, &cutter, [0.0, 0.0, 1.0]).map_err(msg))
        .map_err(|e| format!("hybrid::cut_by_spline of a taller block by the arch surface failed: {e}"))?;
    let free: Vec<usize> = cut
        .faces
        .iter()
        .enumerate()
        .filter(|(_, f)| !matches!(f.surface, Surface::Plane { .. } | Surface::PlaneThrough { .. }))
        .map(|(i, _)| i)
        .collect();
    match free.as_slice() {
        [i] => assemble(&[Part { body: &cut, skip: Some(*i), forward: None }]),
        other => Err(format!("expected one freeform face after the cut, found {}", other.len())),
    }
}

// ---------------------------------------------------------------------------------------------
// what is measured on a joined body

fn step_round_trip(body: &Brep, faces: usize, volume: Option<f64>) -> String {
    let r = guard(|| {
        let text = step::export_solids(&[("patch_join", body)]).map_err(|e| format!("export: {e}"))?;
        let back = step::import(text.as_bytes()).map_err(|e| format!("import: {e}"))?;
        let counts: Vec<usize> = back.iter().map(|b| b.faces.len()).collect();
        let vols: Vec<Result<f64, String>> = back.iter().map(|b| solid::volume(b).map_err(msg)).collect();
        let same = back.len() == 1
            && counts.first() == Some(&faces)
            && matches!((volume, vols.first()), (Some(a), Some(Ok(b))) if (a - b).abs() <= 1e-6 * a.abs().max(1.0));
        let shown: Vec<String> = vols
            .iter()
            .map(|v| match v {
                Ok(v) => format!("{v:.4}"),
                Err(e) => format!("FAILED: {e}"),
            })
            .collect();
        Ok(format!(
            "{} bodies, faces {counts:?} (was {faces}), volumes {shown:?}: {}",
            back.len(),
            if same { "SAME" } else { "DIFFERENT" }
        ))
    });
    r.unwrap_or_else(|e| format!("FAILED: {e}"))
}

fn cut_check(body: &Brep, expected: f64) -> String {
    let r = guard(|| {
        let cutter = solid::make_box([15.0, 10.0, -5.0], [10.0, 10.0, 30.0]).map_err(msg)?;
        let out = boolean(body, &cutter, BoolKind::Subtract).map_err(msg)?;
        let v = solid::volume(&out).map_err(msg)?;
        Ok((out.faces.len(), is_closed(&out), v))
    });
    match r {
        Ok((f, closed, v)) => format!(
            "ok, faces {f}, closed {closed}, volume {v:.4} (exp {expected:.4}, rel {:+.1e})",
            (v - expected) / expected
        ),
        Err(e) => format!("FAILED: {e}"),
    }
}

/// One line: topology, volume, STEP round trip and boolean of a body.
fn inspect(body: &Brep, exp: f64, exp_cut: f64) -> String {
    let c = solid::counts(body);
    let vol = guard(|| solid::volume(body).map_err(msg));
    let vol_text = match &vol {
        Ok(v) => format!("{v:.4} (exp {exp:.4}, rel {:+.1e})", (v - exp) / exp),
        Err(e) => format!("FAILED: {e}"),
    };
    let step = step_round_trip(body, c.faces, vol.as_ref().ok().copied());
    let cut = cut_check(body, exp_cut);
    format!(
        "closed {} faces {} edges {} | volume {vol_text}\n        step: {step}\n        cut: {cut}",
        is_closed(body),
        c.faces,
        c.edges
    )
}

fn join_rows(label: &str, open: &Brep, cap: &Brep, up: bool, exp: f64, exp_cut: f64) {
    let parts = [Part { body: open, skip: None, forward: None }, Part { body: cap, skip: None, forward: Some(up) }];
    let joined = match assemble(&parts) {
        Ok(j) => j,
        Err(e) => {
            eprintln!("  {label}: FAILED: {e}");
            return;
        }
    };
    eprintln!(
        "  {label}: assembled {} faces, {} edges, closed {} (nothing sewn yet)",
        joined.faces.len(),
        joined.edges.len(),
        is_closed(&joined)
    );
    for tol in TOLERANCES {
        match guard(|| stitch(&joined, tol).map_err(msg)) {
            Ok((b, all)) => eprintln!("    {label} stitch {tol:e}: all sewn {all}, {}", inspect(&b, exp, exp_cut)),
            Err(e) => eprintln!("    {label} stitch {tol:e}: FAILED: {e}"),
        }
    }
}

fn run_variant(v: &Variant) {
    eprintln!("\n=== variant {}: {} ===", v.name, v.title);
    let open = match open_shell(v) {
        Ok(o) => o,
        Err(e) if v.name == "c" => {
            eprintln!("variant c: SKIPPED ({e})");
            return;
        }
        Err(e) => {
            eprintln!("  open shell FAILED: {e}");
            return;
        }
    };
    eprintln!("  open shell: {} faces, {} edges, closed {}", open.faces.len(), open.edges.len(), is_closed(&open));
    let req = request(v);
    let patch = match guard(|| fit_patch(&req).map_err(msg)) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("  fit_patch FAILED: {e}");
            return;
        }
    };
    let r = &patch.report;
    eprintln!(
        "  patch: control net {:?}, rounds {}, data max {:.2e} over {} points, within_targets {}, warnings {:?}",
        r.control, r.rounds, r.max, r.points, r.within_targets, r.warnings
    );
    for (k, sd) in r.sides.iter().enumerate() {
        eprintln!("    side {k}: gap_max {:.3e} mm ({} samples)", sd.gap_max, sd.samples);
    }
    // how far the cap's outline edges are from the block's top edges (the gap a stitch must close)
    let sides = &req.sides;
    let mut worst = 0.0f64;
    for e in &patch.sheet.edges {
        if let Curve::Spline { points, .. } = &e.curve {
            for p in points {
                let pt = *p;
                let d = sides
                    .iter()
                    .flat_map(|s| s.points.windows(2).map(move |w| seg_dist(pt, w[0], w[1])))
                    .fold(f64::INFINITY, f64::min);
                worst = worst.max(d);
            }
        }
    }
    let exp = simpson(v.height, [0.0, SIZE[0]], [0.0, SIZE[1]]);
    let exp_cut = exp - simpson(v.height, [15.0, 25.0], [10.0, 20.0]);
    eprintln!(
        "  cap outline: {} edges, farthest point {worst:.3e} mm from the top edges; expected volume {exp:.4}, after the cut {exp_cut:.4}",
        patch.sheet.edges.len()
    );
    let up = match faces_up(&patch.surface) {
        Ok(u) => u,
        Err(e) => {
            eprintln!("  normal of the patch FAILED: {e}; taken as up");
            true
        }
    };
    eprintln!("  cap normal points up: {up}");
    eprintln!("  route A1 (hand-assembled record, cap as fit_patch returns it, then nurbs::stitch):");
    join_rows("A1", &open, &patch.sheet, up, exp, exp_cut);
    eprintln!("  route A2 (hand-assembled record, cap edges merged to one per side, then nurbs::stitch):");
    match merge_sides(&patch.sheet, &corners3d(v.top)) {
        Ok(merged) => {
            eprintln!("  A2 cap: {} edges", merged.edges.len());
            join_rows("A2", &open, &merged, up, exp, exp_cut);
        }
        Err(e) => eprintln!("  A2: FAILED merging the cap's edges: {e}"),
    }
    let b = guard(|| {
        let tall = solid::make_box([0.0; 3], [SIZE[0], SIZE[1], 25.0]).map_err(msg)?;
        cut_by_spline(&tall, &patch.surface, [0.0, 0.0, 1.0]).map_err(msg)
    });
    match b {
        Ok(b) => eprintln!(
            "  route B (hybrid::cut_by_spline of a taller block by the patch surface): {}",
            inspect(&b, exp, exp_cut)
        ),
        Err(e) => eprintln!("  route B (hybrid::cut_by_spline of a taller block by the patch surface): FAILED: {e}"),
    }
    if v.name == "c" {
        eprintln!("  route C (hybrid::replace_face): not applicable, the top face is not planar");
        return;
    }
    let c = guard(|| {
        let b = solid::make_box([0.0; 3], SIZE).map_err(msg)?;
        replace_face(&b, find_top(&b)?, &patch.surface).map_err(msg)
    });
    match c {
        Ok(b) => eprintln!(
            "  route C (hybrid::replace_face of the box top by the patch surface): {}",
            inspect(&b, exp, exp_cut)
        ),
        Err(e) => eprintln!("  route C (hybrid::replace_face of the box top by the patch surface): FAILED: {e}"),
    }
}

#[test]
#[ignore = "measurement: prints how fit_patch caps join other faces"]
fn measure_patch_join() {
    eprintln!("patch_join: block {SIZE:?} mm, stitch tolerances {TOLERANCES:?} mm");
    eprintln!(
        "cannot combine sheets: no kernel function merges several sheet records into one record (extract_face takes one face out of a body, sheet_from_patches and sheet_from_bezier_network build from patch grids and Bezier nets only, stitch takes one record); route A builds the record by hand in this file"
    );
    for v in variants() {
        run_variant(&v);
    }
}

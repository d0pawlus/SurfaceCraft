//! Tangent-continuous patch networks for `surface.wrap` (plan `2026-11-11-mcp-agent-modelling`,
//! organic parts; milestone P9-O2).
//!
//! Every coarse triangle becomes three bicubic patches (as in [`crate::patch::wrap_patches`]), but
//! the patches are made *tangent* across every edge that is not a crease, and the creases stay sharp.
//!
//! Method. The scan decides what is a crease and what is smooth ([`crate::wrap_normals`]), and gives
//! every layout vertex one normal per smooth sector around it.
//! * The boundary curves have their end tangents in the tangent planes of their ends (a crease curve
//!   along the intersection of the planes of its two sides), and handle lengths fitted to the scan.
//! * Along a smooth edge both neighbours are asked for the same thing: the cross-boundary
//!   derivative must be perpendicular to one normal field `n(t)`, the end normals blended and made
//!   perpendicular to the edge. Both patches then have the tangent plane `n(t)^⊥` along the edge, so
//!   they are tangent to each other there. The condition is linear in the control points, which
//!   decouples the patches: each solves its four inner control points by weighted least squares
//!   from the scan samples under it and the conditions on its smooth sides.
//! * The conditions are met exactly at the vertices (the tangents lie in the planes) and in the
//!   least-squares sense along the edges, also at vertices of any valence: no vertex-enclosure
//!   compatibility is needed because the neighbours do not have to agree on a *specific* cross
//!   derivative, only on its plane. What is left is measured ([`jump_stats`]), not claimed zero.
//!
//! The patches are the same bicubic nets as before; the surface evaluation they use is
//! [`crate::patch::eval`] (the listed APP-GEOMETRY file).
use crate::mls::ScanSurface;
use crate::patch::{
    PatchSet, WRAP_FACING_COS, bern, bezier_tangent, cross_weights, eval, normal, tensor_weights, weighted_sum,
    wrap_curve, wrap_net,
};
pub use crate::patch_jumps::{JumpStats, jump_stats};
use crate::wrap_normals::{EdgeKey, Layout, LayoutNormals, edge_faces, edge_slot, key, layout_normals};
use nalgebra::{SMatrix, SVector};
use std::collections::{BTreeMap, BTreeSet};
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3, normalize};

/// Parameters of the wrap.
#[derive(Clone, Copy, Debug)]
pub struct G1Options {
    /// Make the patches tangent across the smooth edges (false: the patches meet G0 only).
    pub g1: bool,
    /// Edges whose two sides differ by more than this angle (degrees) are creases.
    pub crease_deg: f64,
}

/// The patches and what the scan said about their layout.
#[derive(Clone, Debug, Default)]
pub struct G1Wrap {
    /// The patches.
    pub set: PatchSet,
    /// Layout edges (the two halves of every crease edge of the coarse mesh) that are creases.
    pub creases: BTreeSet<(u32, u32)>,
    /// Coarse edges that are creases.
    pub crease_edges: usize,
    /// Coarse edges between two faces that are not creases.
    pub smooth_edges: usize,
    /// Patches the tangent construction could not fit (a fold, an inverted corner): built unconstrained.
    pub fallbacks: usize,
}

/// Cubic control points.
type Curve = [Vec3; 4];
/// The 4 × 4 control points of a patch.
type Grid = [[Vec3; 4]; 4];

/// Control rows of the sides of a patch (bottom, right, top, left): the boundary and the row next to it.
type Cells = [(usize, usize); 4];
const SIDES: [(Cells, Cells); 4] = [
    ([(0, 0), (1, 0), (2, 0), (3, 0)], [(0, 1), (1, 1), (2, 1), (3, 1)]),
    ([(3, 0), (3, 1), (3, 2), (3, 3)], [(2, 0), (2, 1), (2, 2), (2, 3)]),
    ([(0, 3), (1, 3), (2, 3), (3, 3)], [(0, 2), (1, 2), (2, 2), (3, 2)]),
    ([(0, 0), (0, 1), (0, 2), (0, 3)], [(1, 0), (1, 1), (1, 2), (1, 3)]),
];
/// Scan samples a patch is fitted to, in the parameter of each direction.
const FIT_SAMPLES: [f64; 4] = [0.125, 0.375, 0.625, 0.875];
/// Samples along a side at which the tangent-plane condition is imposed.
const G1_SAMPLES: [f64; 5] = [1.0 / 6.0, 1.0 / 3.0, 0.5, 2.0 / 3.0, 5.0 / 6.0];
/// Noise of a scan sample (mm) and tolerated tangent-plane error (rad): the weights of the two kinds of rows.
const FIT_SIGMA: f64 = 0.03;
const G1_SIGMAS: [f64; 2] = [0.004, 0.04];
/// Parameters at which a patch is checked against the coarse face, and the least cosine with its normal.
const CHECK_GRID: [f64; 5] = [0.05, 0.25, 0.5, 0.75, 0.95];
const MIN_FACING: f64 = 0.3;
const EDGE_CHECK: [f64; 7] = [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875];
/// A tangent patch is kept when its rms distance from the scan is at most this multiple of that of the
/// unconstrained patch plus this floor (mm): the tangent condition may not cost accuracy.
const ACCURACY_SLACK: f64 = 1.25;
const ACCURACY_FLOOR: f64 = 0.02;
/// Pull of the inner control points to their first guess, against the weight of one fit row.
const RIDGE: f64 = 1e-3;
/// Rounds of fitting the inner control points (each re-projects the samples onto the scan).
const FIT_PASSES: usize = 3;
/// The cross product of two unit side normals shorter than this (about a hundred rounding errors of the
/// normals) is zero: the two tangent planes are the same plane.
const PARALLEL_SIN: f64 = 1e-9;
/// Handle lengths of an edge curve, as shares of its chord.
const HANDLE_RANGE: (f64, f64) = (0.1, 0.7);
/// Sideways lean of a handle (share of the chord) at most, and the pull of the lean toward none.
const MAX_LEAN: f64 = 0.4;
const LEAN_RIDGE: f64 = 0.02;

fn a3(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn tangent_toward(n: &Vec3, d: &Vec3) -> Vec3 {
    normalize(d - n * d.dot(n)).unwrap_or_else(|| normalize(*d).unwrap_or_else(Vec3::x))
}

/// An end of an edge curve: its point, the unit direction the curve leaves (or arrives) along, and the
/// unit direction in the tangent plane perpendicular to it that the handle may also lean toward (none
/// where the direction is fixed, as along a crease).
#[derive(Clone, Copy)]
struct End {
    p: Vec3,
    t: Vec3,
    side: Option<Vec3>,
}

impl End {
    fn new(p: Vec3, t: Vec3, normal: Option<&Vec3>) -> Self {
        Self { p, t, side: normal.and_then(|n| normalize(n.cross(&t))) }
    }
}

/// The cubic from `a` to `b`: its handles lie in the tangent planes of its ends (along the end
/// direction, leaning toward the side direction where that is free), and have the lengths and lean
/// that bring the curve closest to the scan points `project` finds under its quarter points.
fn tangent_curve(a: End, b: End, project: &dyn Fn(Vec3) -> Vec3) -> Curve {
    let len = (b.p - a.p).norm();
    let mut n = (SMatrix::<f64, 4, 4>::zeros(), SVector::<f64, 4>::zeros());
    for t in [0.25f64, 0.5, 0.75] {
        let [b0, b1, b2, b3] = bern(t);
        let r = project(a.p * (1.0 - t) + b.p * t) - a.p * (b0 + b1) - b.p * (b2 + b3);
        let cols =
            [a.t * b1, a.side.map_or(Vec3::zeros(), |s| s * b1), -b.t * b2, b.side.map_or(Vec3::zeros(), |s| -s * b2)];
        for i in 0..4 {
            n.1[i] += cols[i].dot(&r);
            for j in 0..4 {
                n.0[(i, j)] += cols[i].dot(&cols[j]);
            }
        }
    }
    for i in [1, 3] {
        n.0[(i, i)] += LEAN_RIDGE + 1e-9;
    }
    n.0[(0, 0)] += 1e-9;
    n.0[(2, 2)] += 1e-9;
    let (lo, hi) = (HANDLE_RANGE.0 * len, HANDLE_RANGE.1 * len);
    let x = n.0.cholesky().map(|c| c.solve(&n.1)).filter(|x| x.iter().all(|v| v.is_finite()));
    let x = x.map_or([len / 3.0, 0.0, len / 3.0, 0.0], |x| {
        let lean = MAX_LEAN * len;
        [x[0].clamp(lo, hi), x[1].clamp(-lean, lean), x[2].clamp(lo, hi), x[3].clamp(-lean, lean)]
    });
    let ha = a.t * x[0] + a.side.map_or(Vec3::zeros(), |s| s * x[1]);
    let hb = b.t * x[2] + b.side.map_or(Vec3::zeros(), |s| s * x[3]);
    [a.p, a.p + ha, b.p - hb, b.p]
}

/// The tangent plane normal along a side: the end normals blended and made perpendicular to the curve.
fn field_normal(c: &Curve, n0: &Vec3, n1: &Vec3, t: f64) -> Vec3 {
    let n = n0 * (1.0 - t) + n1 * t;
    let n = match normalize(bezier_tangent(c, t)) {
        Some(th) => n - th * n.dot(&th),
        None => n,
    };
    normalize(n).unwrap_or_else(Vec3::z)
}

/// A side of a patch that is to be tangent to its neighbour: the normals at its two ends.
#[derive(Clone, Copy)]
struct SideField {
    n0: Vec3,
    n1: Vec3,
}

/// The slot of the inner control point `(i, j)` among the four unknowns.
fn inner_slot(i: usize, j: usize) -> Option<usize> {
    ((1..3).contains(&i) && (1..3).contains(&j)).then(|| (i - 1) * 2 + (j - 1))
}

type Normal12 = (SMatrix<f64, 12, 12>, SVector<f64, 12>);

fn add_row(n: &mut Normal12, coef: &SVector<f64, 12>, rhs: f64, weight: f64) {
    n.0 += coef * coef.transpose() * weight;
    n.1 += coef * (rhs * weight);
}

/// Sum of the boundary control points of `p` weighted by their Bernstein products at `(u, v)`.
fn ring_value(p: &Grid, u: f64, v: f64) -> Vec3 {
    weighted_sum(p, u, v, &|i, j| inner_slot(i, j).is_some())
}

/// Rows asking the inner points to pass through the scan points `targets` at their parameters.
fn fit_rows(n: &mut Normal12, p: &Grid, targets: &[((f64, f64), Vec3)]) {
    let weight = 1.0 / (FIT_SIGMA * FIT_SIGMA);
    for &((u, v), target) in targets {
        let w = tensor_weights(u, v);
        let rest = target - ring_value(p, u, v);
        for c in 0..3 {
            let mut coef = SVector::<f64, 12>::zeros();
            for (i, j) in [(1, 1), (1, 2), (2, 1), (2, 2)] {
                if let Some(s) = inner_slot(i, j) {
                    coef[s * 3 + c] = w[i][j];
                }
            }
            add_row(n, &coef, rest[c], weight);
        }
    }
}

/// Rows asking the cross-boundary derivative of side `s` to be perpendicular to the normal field.
fn tangent_rows(n: &mut Normal12, p: &Grid, s: usize, field: &SideField, sigma: f64) {
    let (base, next) = SIDES[s];
    let curve: Curve =
        [p[base[0].0][base[0].1], p[base[1].0][base[1].1], p[base[2].0][base[2].1], p[base[3].0][base[3].1]];
    let len = (curve[3] - curve[0]).norm().max(1e-9);
    let weight = 1.0 / (sigma * sigma);
    for t in G1_SAMPLES {
        let nt = field_normal(&curve, &field.n0, &field.n1, t);
        let b = cross_weights(t);
        let mut coef = SVector::<f64, 12>::zeros();
        let mut constant = 0.0;
        for k in 0..4 {
            constant -= b[k] * nt.dot(&p[base[k].0][base[k].1]);
            match inner_slot(next[k].0, next[k].1) {
                Some(slot) => (0..3).for_each(|c| coef[slot * 3 + c] += b[k] * nt[c] / len),
                None => constant += b[k] * nt.dot(&p[next[k].0][next[k].1]),
            }
        }
        add_row(n, &coef, -constant / len, weight);
    }
}

fn inner_vector(p: &Grid) -> SVector<f64, 12> {
    let mut x = SVector::<f64, 12>::zeros();
    for (i, j) in [(1, 1), (1, 2), (2, 1), (2, 2)] {
        if let Some(s) = inner_slot(i, j) {
            (0..3).for_each(|c| x[s * 3 + c] = p[i][j][c]);
        }
    }
    x
}

fn with_inner(p: &Grid, x: &SVector<f64, 12>) -> Grid {
    let mut out = *p;
    for (i, j) in [(1, 1), (1, 2), (2, 1), (2, 2)] {
        if let Some(s) = inner_slot(i, j) {
            out[i][j] = Vec3::new(x[s * 3], x[s * 3 + 1], x[s * 3 + 2]);
        }
    }
    out
}

fn to_net(p: &Grid) -> crate::patch::Net {
    let mut net = [[[0.0; 3]; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            net[i][j] = a3(p[i][j]);
        }
    }
    net
}

/// The inner control points that fit `targets` best among those tangent to `sides`.
fn solve_inner(
    p: &Grid,
    x0: &SVector<f64, 12>,
    targets: &[((f64, f64), Vec3)],
    sides: &[Option<SideField>; 4],
    sigma: f64,
) -> Option<SVector<f64, 12>> {
    let mut n: Normal12 = (SMatrix::zeros(), SVector::zeros());
    fit_rows(&mut n, p, targets);
    for (s, field) in sides.iter().enumerate() {
        if let Some(f) = field {
            tangent_rows(&mut n, p, s, f, sigma);
        }
    }
    let ridge = RIDGE / (FIT_SIGMA * FIT_SIGMA);
    for k in 0..12 {
        n.0[(k, k)] += ridge;
        n.1[k] += ridge * x0[k];
    }
    let x = n.0.cholesky()?.solve(&n.1);
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// The patch with boundary curves `bot` (c0 → c1), `right` (c1 → c2), `top` (c3 → c2), `left` (c0 → c3)
/// whose inner points follow the scan (`project`) and are tangent to the neighbours across `sides`.
fn tangent_net(
    bot: &Curve,
    right: &Curve,
    top: &Curve,
    left: &Curve,
    sides: &[Option<SideField>; 4],
    sigma: f64,
    project: &dyn Fn(Vec3) -> Vec3,
) -> crate::patch::Net {
    let mut p: Grid = [[Vec3::zeros(); 4]; 4];
    for k in 0..4 {
        p[k][0] = bot[k];
        p[3][k] = right[k];
        p[k][3] = top[k];
        p[0][k] = left[k];
    }
    p[1][1] = p[1][0] + p[0][1] - p[0][0];
    p[2][1] = p[2][0] + p[3][1] - p[3][0];
    p[1][2] = p[1][3] + p[0][2] - p[0][3];
    p[2][2] = p[2][3] + p[3][2] - p[3][3];
    let x0 = inner_vector(&p);
    for _ in 0..FIT_PASSES {
        let net = to_net(&p);
        let mut targets = Vec::with_capacity(FIT_SAMPLES.len() * FIT_SAMPLES.len());
        for &u in &FIT_SAMPLES {
            for &v in &FIT_SAMPLES {
                targets.push(((u, v), project(eval(&net, u, v).0)));
            }
        }
        match solve_inner(&p, &x0, &targets, sides, sigma) {
            Some(x) => p = with_inner(&p, &x),
            None => break,
        }
    }
    to_net(&p)
}

/// The coarse mesh, the scan it was reduced from and the normals the scan gives it.
struct Layer<'a> {
    coarse: &'a Mesh,
    scan: &'a Bvh,
    reach: f64,
    face_normal: Vec<Vec3>,
    normals: LayoutNormals,
    /// Coarse edges with fewer or more than two faces: no neighbour to be tangent to.
    free: BTreeSet<EdgeKey>,
    g1: bool,
}

impl Layer<'_> {
    /// The scan point nearest to `at` on a side that faces like one of `facing`.
    fn project(&self, at: Vec3, facing: &[Vec3]) -> Vec3 {
        let hit = self.scan.closest_facing(&Point3::from(at), self.reach, facing, WRAP_FACING_COS);
        hit.map_or(at, |h| h.point.coords)
    }

    fn corner_normal(&self, f: usize, v: u32) -> Vec3 {
        let slot = self.coarse.triangles.get(f).and_then(|t| t.iter().position(|&u| u == v));
        slot.and_then(|k| self.normals.corner.get(f)?.get(k).copied()).unwrap_or_else(Vec3::z)
    }

    fn mid_normal(&self, f: usize, k: usize) -> Vec3 {
        self.normals.mid.get(f).and_then(|m| m.get(k)).copied().unwrap_or_else(Vec3::z)
    }
}

/// How a crease edge continues: a unit direction in both side planes (their intersection line), toward
/// `toward`. Where the planes are numerically the same plane the edge is smooth for this purpose and the
/// direction is taken in the plane the two normals share, which is in both.
fn crease_direction(n1: &Vec3, n2: &Vec3, toward: &Vec3) -> Vec3 {
    let c = n1.cross(n2);
    if c.norm() > PARALLEL_SIN * n1.norm() * n2.norm() {
        let d = c.normalize();
        return if d.dot(toward) < 0.0 { -d } else { d };
    }
    tangent_toward(&normalize(n1 + n2).unwrap_or(*n1), toward)
}

/// Directions the curves of the coarse edge `a`-`b` leave `a` and `b` along, and the direction `a → b`
/// at the midpoint.
struct EdgeTangents {
    a: Vec3,
    b: Vec3,
    m: Vec3,
    /// Normals at the ends and the middle where the handles may lean within the tangent plane (not along a crease).
    normals: Option<[Vec3; 3]>,
}

fn edge_tangents(
    l: &Layer,
    vertices: &[Vec3],
    a: u32,
    b: u32,
    pm: Vec3,
    faces: &[usize],
    crease: bool,
) -> Option<EdgeTangents> {
    let (pa, pb) = (*vertices.get(a as usize)?, *vertices.get(b as usize)?);
    let slot = |f: usize| l.coarse.triangles.get(f).and_then(|t| edge_slot(t, a, b));
    let f1 = *faces.first()?;
    let (na, nb, nm) = (l.corner_normal(f1, a), l.corner_normal(f1, b), l.mid_normal(f1, slot(f1)?));
    let smooth = EdgeTangents {
        a: tangent_toward(&na, &(pm - pa)),
        b: tangent_toward(&nb, &(pm - pb)),
        m: tangent_toward(&nm, &(pb - pa)),
        normals: Some([na, nb, nm]),
    };
    let (Some(&f2), true) = (faces.get(1), crease) else { return Some(smooth) };
    let (ma, mb, mm) = (l.corner_normal(f2, a), l.corner_normal(f2, b), l.mid_normal(f2, slot(f2)?));
    Some(EdgeTangents {
        a: crease_direction(&na, &ma, &(pm - pa)),
        b: crease_direction(&nb, &mb, &(pm - pb)),
        m: crease_direction(&nm, &mm, &(pb - pa)),
        normals: None,
    })
}

type Curves = BTreeMap<EdgeKey, Curve>;

/// One midpoint per coarse edge and the two curves from its ends to it.
fn edge_curves(l: &Layer, edges: &BTreeMap<EdgeKey, Vec<usize>>, build: &mut Build) -> BTreeMap<EdgeKey, u32> {
    let mut mid = BTreeMap::new();
    for (&(a, b), faces) in edges {
        let (Some(&pa), Some(&pb)) = (build.vertices.get(a as usize), build.vertices.get(b as usize)) else { continue };
        let facing: Vec<Vec3> = faces.iter().filter_map(|&f| l.face_normal.get(f).copied()).collect();
        let on_edge = |at: Vec3| l.project(at, &facing);
        let m = build.vertices.len() as u32;
        let pm = on_edge((pa + pb) * 0.5);
        build.vertices.push(pm);
        mid.insert((a, b), m);
        let crease = l.normals.creases.contains(&(a, b));
        let tangents = if l.g1 && !l.free.contains(&(a, b)) {
            edge_tangents(l, &build.vertices, a, b, pm, faces, crease)
        } else {
            None
        };
        let Some(t) = tangents else {
            build.curves.insert((a, m), wrap_curve(pa, pm, &on_edge));
            build.curves.insert((b, m), wrap_curve(pb, pm, &on_edge));
            continue;
        };
        let [na, nb, nm] = t.normals.map_or([None; 3], |n| n.map(Some));
        let (ea, eb) = (End::new(pa, t.a, na.as_ref()), End::new(pb, t.b, nb.as_ref()));
        let from_a = tangent_curve(ea, End::new(pm, t.m, nm.as_ref()), &on_edge);
        let from_b = tangent_curve(eb, End::new(pm, -t.m, nm.as_ref()), &on_edge);
        build.curves.insert((a, m), from_a);
        build.curves.insert((b, m), from_b);
    }
    mid
}

/// The curve from `a` to `b` as stored (smaller vertex id first), read from `a`.
fn side(curves: &Curves, a: u32, b: u32) -> Option<Curve> {
    let c = curves.get(&key(a, b))?;
    Some(if a < b { *c } else { [c[3], c[2], c[1], c[0]] })
}

/// The layout vertices, boundary curves and patches built so far.
struct Build {
    vertices: Vec<Vec3>,
    curves: Curves,
    set: PatchSet,
    /// Patches that fell back to the unconstrained construction.
    fallbacks: usize,
}

/// One coarse triangle: its corners and the midpoints of its edges (edge `k` runs from corner `k`).
struct Triangle {
    f: usize,
    corners: [u32; 3],
    mids: [u32; 3],
}

/// The centre of the triangle and the three curves from the edge midpoints to it; returns the centre's id.
fn add_spokes(l: &Layer, tri: &Triangle, nf: Vec3, build: &mut Build) -> u32 {
    let on_face = |at: Vec3| l.project(at, &[nf]);
    let p = |v: u32| build.vertices.get(v as usize).copied().unwrap_or_else(Vec3::zeros);
    let pg = on_face((p(tri.corners[0]) + p(tri.corners[1]) + p(tri.corners[2])) / 3.0);
    let g = build.vertices.len() as u32;
    let ng = l.normals.centre.get(tri.f).copied().unwrap_or(nf);
    let mut spokes = Vec::with_capacity(3);
    for (k, &m) in tri.mids.iter().enumerate() {
        let pm = p(m);
        spokes.push(if l.g1 {
            let nm = l.mid_normal(tri.f, k);
            let (tm, tg) = (tangent_toward(&nm, &(pg - pm)), tangent_toward(&ng, &(pg - pm)));
            tangent_curve(End::new(pm, tm, Some(&nm)), End::new(pg, tg, Some(&ng)), &on_face)
        } else {
            wrap_curve(pm, pg, &on_face)
        });
    }
    for (&m, c) in tri.mids.iter().zip(spokes) {
        build.curves.insert((m, g), c);
    }
    build.vertices.push(pg);
    g
}

/// Does the patch face the way the coarse face does, inside and along its boundary (no fold, no inverted
/// corner)? The boundary is checked at the points the jumps across it are measured at. An undefined
/// normal (a collapsed or singular patch there) is a failure.
fn follows(net: &crate::patch::Net, nf: &Vec3) -> bool {
    let faces = |u: f64, v: f64| normal(net, u, v).is_some_and(|n| n.dot(nf) > MIN_FACING);
    let inside = CHECK_GRID.iter().all(|&u| CHECK_GRID.iter().all(|&v| faces(u, v)));
    let along = EDGE_CHECK.iter().all(|&t| faces(t, 0.0) && faces(t, 1.0) && faces(0.0, t) && faces(1.0, t));
    inside && along
}

/// Root mean square distance of the patch from the scan over a grid of points on it.
fn scan_rms(net: &crate::patch::Net, project: &dyn Fn(Vec3) -> Vec3) -> f64 {
    let mut sum = 0.0;
    for &u in &CHECK_GRID {
        for &v in &CHECK_GRID {
            let p = eval(net, u, v).0;
            sum += (project(p) - p).norm_squared();
        }
    }
    (sum / (CHECK_GRID.len() * CHECK_GRID.len()) as f64).sqrt()
}

/// Patch `k` of the triangle: tangent to its neighbours where the scan allows (the patch neither
/// folds nor follows the scan clearly worse than the unconstrained one), else unconstrained.
fn patch_net(l: &Layer, tri: &Triangle, k: usize, curves: [Curve; 4], ng: Vec3, nf: Vec3) -> (crate::patch::Net, bool) {
    let [bot, right, top, left] = curves;
    let on_face = |at: Vec3| l.project(at, &[nf]);
    let free = wrap_net(&bot, &right, &top, &left, &on_face);
    if !l.g1 {
        return (free, true);
    }
    let (nc, mb, ml) =
        (l.corner_normal(tri.f, tri.corners[k]), l.mid_normal(tri.f, k), l.mid_normal(tri.f, (k + 2) % 3));
    let edge = |e: usize| l.coarse.triangles.get(tri.f).map(|t| key(t[e], t[(e + 1) % 3]));
    let open = |e: usize, f: SideField| edge(e).is_some_and(|k| !l.free.contains(&k)).then_some(f);
    let sides = [
        open(k, SideField { n0: nc, n1: mb }),
        Some(SideField { n0: mb, n1: ng }),
        Some(SideField { n0: ml, n1: ng }),
        open((k + 2) % 3, SideField { n0: nc, n1: ml }),
    ];
    let limit = ACCURACY_SLACK * scan_rms(&free, &on_face) + ACCURACY_FLOOR;
    for sigma in G1_SIGMAS {
        let net = tangent_net(&bot, &right, &top, &left, &sides, sigma, &on_face);
        if follows(&net, &nf) && scan_rms(&net, &on_face) <= limit {
            return (net, true);
        }
    }
    (free, false)
}

fn add_triangle(l: &Layer, tri: &Triangle, build: &mut Build) {
    let nf = l.face_normal.get(tri.f).copied().unwrap_or_else(Vec3::z);
    let ng = l.normals.centre.get(tri.f).copied().unwrap_or(nf);
    let g = add_spokes(l, tri, nf, build);
    for k in 0..3 {
        let (c0, c1, c3) = (tri.corners[k], tri.mids[k], tri.mids[(k + 2) % 3]);
        let c = &build.curves;
        let (Some(bot), Some(right), Some(top), Some(left)) =
            (side(c, c0, c1), side(c, c1, g), side(c, c3, g), side(c, c0, c3))
        else {
            continue;
        };
        let (net, tangent) = patch_net(l, tri, k, [bot, right, top, left], ng, nf);
        build.fallbacks += usize::from(!tangent);
        build.set.nets.push(net);
        build.set.corners.push([c0, c1, g, c3]);
    }
}

/// Wraps the coarse triangle mesh `coarse` (a reduction of the scan: manifold, close to it) in
/// bicubic patches fitted to the scan (`scan`, `surface`), tangent across the edges that are not
/// creases when `options.g1` is set. The patches and their layout vertices are numbered exactly as
/// [`crate::patch::wrap_patches`] numbers them.
#[must_use]
pub fn wrap_patches_g1(coarse: &Mesh, scan: &Bvh, surface: &ScanSurface, reach: f64, options: &G1Options) -> G1Wrap {
    let face_normal: Vec<Vec3> =
        (0..coarse.triangle_count()).map(|f| coarse.face_normal(f).unwrap_or_else(Vec3::z)).collect();
    let vertices: Vec<Vec3> =
        coarse.positions.iter().map(|p| scan.closest(p, reach).map_or(p.coords, |h| h.point.coords)).collect();
    let edges = edge_faces(coarse);
    let normals = {
        let layout = Layout { coarse, vertices: &vertices, face_normal: &face_normal, surface, edges: &edges };
        layout_normals(&layout, options.crease_deg)
    };
    let free = edges.iter().filter(|(_, f)| f.len() != 2).map(|(k, _)| *k).collect();
    let layer = Layer { coarse, scan, reach, face_normal, normals, free, g1: options.g1 };
    let mut build = Build { vertices, curves: Curves::new(), set: PatchSet::default(), fallbacks: 0 };
    let mid = edge_curves(&layer, &edges, &mut build);
    for (f, t) in coarse.triangles.iter().enumerate() {
        let m = |k: usize| mid.get(&key(t[k], t[(k + 1) % 3])).copied();
        let (Some(m0), Some(m1), Some(m2)) = (m(0), m(1), m(2)) else { continue };
        add_triangle(&layer, &Triangle { f, corners: *t, mids: [m0, m1, m2] }, &mut build);
    }
    let Build { vertices, mut set, fallbacks, .. } = build;
    set.vertices = vertices.into_iter().map(a3).collect();
    let creases = layer
        .normals
        .creases
        .iter()
        .flat_map(|&(a, b)| mid.get(&(a, b)).into_iter().flat_map(move |&m| [key(a, m), key(b, m)]))
        .collect();
    G1Wrap {
        set,
        creases,
        crease_edges: layer.normals.creases.len(),
        smooth_edges: layer.normals.smooth_edges,
        fallbacks,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Rng, synth};

    fn wrap(coarse: &Mesh, scan: &Mesh, g1: bool) -> G1Wrap {
        let surface = ScanSurface::oriented(&scan.positions, &scan.vertex_normals());
        let bvh = Bvh::build(scan);
        wrap_patches_g1(coarse, &bvh, &surface, 4.0, &G1Options { g1, crease_deg: 35.0 })
    }

    fn sphere_pair() -> (Mesh, Mesh) {
        let mut scan = synth::sphere(Point3::origin(), 20.0, 120, 60);
        synth::add_noise(&mut scan, 0.03, &mut Rng::new(5));
        (synth::sphere(Point3::origin(), 20.0, 16, 8), scan)
    }

    #[test]
    fn tangent_patches_on_a_sphere_meet_within_a_degree_and_g0_ones_do_not() {
        // Arrange
        let (coarse, scan) = sphere_pair();

        // Act
        let g1 = wrap(&coarse, &scan, true);
        let g0 = wrap(&coarse, &scan, false);
        let (tangent, _) = jump_stats(&g1.set, &g1.creases);
        let (free, _) = jump_stats(&g0.set, &g0.creases);

        // Assert
        assert_eq!(g1.crease_edges, 0, "a sphere has no creases");
        assert_eq!(g1.set.nets.len(), g0.set.nets.len());
        assert!(tangent.count > 0 && tangent.mean_deg < 1.0, "{tangent:?}");
        assert!(free.mean_deg > 2.0 * tangent.mean_deg, "g0 {free:?} g1 {tangent:?}");
    }

    #[test]
    fn a_box_keeps_its_edges_sharp_and_its_faces_tangent() {
        // Arrange: a coarse box (two triangles per face) over a dense noisy one
        let mut scan = synth::cuboid(Point3::origin(), Vec3::new(20.0, 20.0, 20.0), 20);
        synth::add_noise(&mut scan, 0.02, &mut Rng::new(3));
        let coarse = synth::cuboid(Point3::origin(), Vec3::new(20.0, 20.0, 20.0), 2);

        // Act
        let w = wrap(&coarse, &scan, true);
        let (smooth, sharp) = jump_stats(&w.set, &w.creases);

        // Assert
        assert!(w.crease_edges >= 12, "the box edges are creases: {}", w.crease_edges);
        assert!(sharp.count > 0 && (sharp.mean_deg - 90.0).abs() < 3.0, "{sharp:?}");
        assert!(smooth.mean_deg < 1.0, "{smooth:?}");
    }

    fn rotated(deg: f64) -> Vec3 {
        let a = deg.to_radians();
        Vec3::new(a.sin(), 0.0, a.cos())
    }

    #[test]
    fn a_shallow_crease_direction_lies_in_both_side_planes() {
        // Arrange: sides 5 degrees apart (far above rounding, below the old 11.5 degree cut-off)
        let (n1, n2) = (rotated(-2.5), rotated(2.5));

        // Act
        let d = crease_direction(&n1, &n2, &Vec3::y());

        // Assert
        assert!(d.dot(&n1).abs() < 1e-12 && d.dot(&n2).abs() < 1e-12, "{d:?}");
        assert!(d.dot(&Vec3::y()) > 0.99, "{d:?}");
    }

    #[test]
    fn a_crease_direction_lies_in_both_planes_down_to_parallel_normals() {
        for deg in [180.0, 90.0, 1.0, 1e-3, 1e-8, 1e-12, 0.0] {
            let (n1, n2) = (rotated(0.0), rotated(deg));
            let d = crease_direction(&n1, &n2, &Vec3::y());
            assert!(d.dot(&n1).abs() < 1e-9 && d.dot(&n2).abs() < 1e-9, "{deg}: {d:?}");
            assert!((d.norm() - 1.0).abs() < 1e-12, "{deg}: {d:?}");
        }
    }

    #[test]
    fn a_collapsed_patch_corner_does_not_follow_the_face() {
        // Arrange: the whole side u = 0 collapsed to one point, so the normal is undefined along it
        let mut net = [[[0.0; 3]; 4]; 4];
        for (i, row) in net.iter_mut().enumerate() {
            for (j, p) in row.iter_mut().enumerate() {
                *p = [i as f64, j as f64, 0.0];
            }
        }
        for p in net[0].iter_mut() {
            *p = [0.0, 0.0, 0.0];
        }

        // Act / Assert
        assert!(normal(&net, 0.0, 0.5).is_none());
        assert!(!follows(&net, &Vec3::z()), "an undefined normal must not be accepted");
    }

    /// A height field with a curved crease: `z = SLOPE |x - 1.5 sin(0.4 y)|`, whose two sides meet at
    /// 5 degrees. `grid(n, step)` has its lines along the crease curve.
    fn curved_crease_surface(nx: usize, ny: usize, step: f64) -> Mesh {
        const SLOPE: f64 = 0.043_66;
        let mut m = Mesh::default();
        for j in 0..=ny {
            for i in 0..=nx {
                let (u, y) = ((i as f64 - nx as f64 / 2.0) * step, j as f64 * step);
                let x = u + 1.5 * (0.4 * y).sin();
                m.positions.push(Point3::new(x, y, SLOPE * u.abs()));
            }
        }
        for j in 0..ny {
            for i in 0..nx {
                let a = (j * (nx + 1) + i) as u32;
                let w = (nx + 1) as u32;
                m.triangles.push([a, a + 1, a + w + 1]);
                m.triangles.push([a, a + w + 1, a + w]);
            }
        }
        m
    }

    #[test]
    fn a_curved_shallow_crease_leaves_the_smooth_edges_beside_it_tangent() {
        // Arrange: crease_deg 1 makes the 5 degree crease a crease
        let scan = curved_crease_surface(40, 40, 0.5);
        let coarse = curved_crease_surface(4, 4, 5.0);
        let surface = ScanSurface::oriented(&scan.positions, &scan.vertex_normals());
        let bvh = Bvh::build(&scan);

        // Act
        let w = wrap_patches_g1(&coarse, &bvh, &surface, 4.0, &G1Options { g1: true, crease_deg: 1.0 });
        let (smooth, _) = jump_stats(&w.set, &w.creases);

        // Assert
        assert!(w.crease_edges >= 2, "the 5 degree crease is found: {}", w.crease_edges);
        assert_eq!((smooth.invalid_samples, smooth.invalid_edges), (0, 0), "{smooth:?}");
        assert!(
            smooth.count > 0 && smooth.mean_deg < 1.0 && smooth.p99_deg < 5.0,
            "smooth edges beside the crease: {smooth:?}"
        );
    }

    #[test]
    fn the_wrap_is_deterministic() {
        let (coarse, scan) = sphere_pair();
        let (a, b) = (wrap(&coarse, &scan, true), wrap(&coarse, &scan, true));
        assert_eq!(a.set.nets, b.set.nets);
        assert_eq!(a.creases, b.creases);
    }

    #[test]
    fn hostile_coarse_meshes_do_not_panic() {
        let (_, scan) = sphere_pair();
        let nan = Point3::new(f64::NAN, 0.0, 0.0);
        let meshes = [
            Mesh::default(),
            Mesh { positions: vec![Point3::origin(); 3], triangles: vec![[0, 1, 2]] },
            Mesh { positions: vec![nan, Point3::origin(), Point3::new(1.0, 0.0, 0.0)], triangles: vec![[0, 1, 2]] },
            Mesh { positions: vec![Point3::origin()], triangles: vec![[0, 5, 9]] },
        ];
        for m in &meshes {
            let w = wrap(m, &scan, true);
            assert!(w.set.nets.len() <= 3 * m.triangles.len());
        }
    }
}

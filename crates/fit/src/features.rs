//! Second freeform strategy (design doc §6.3, milestone P4): a patch network bounded by the
//! feature curves of the scan, for housings with clear edges.
//!
//! APP-GEOMETRY: stands in for a solidcore constrained Bézier surface fit (fixed boundary curves,
//! least-squares interior) and Bézier evaluation; the B-rep of the result is built by `kernel::nurbs`.
//!
//! Sharp edges (dihedral angle above a threshold) cut the mesh into smooth regions. Vertices where
//! three or more regions (or two regions and the mesh boundary) meet are *corners*; the mesh
//! edges between two corners are the feature curves, each fitted once as a Bézier curve of the
//! chosen degree (least squares, corners interpolated) so the regions on both sides share the
//! same curve. A region with exactly four corners becomes one Bézier patch of that degree: its
//! four boundary curves are the feature curves, its interior control points are a regularised
//! least-squares fit of the region's vertices (parameters from the Coons patch of the
//! boundary, refined by closest-point iterations). Regions whose boundary is not one outline, or
//! that have no three distinct corners, are listed as skipped. Sharp edges stay sharp: the patches
//! meet there without any tangent continuity.
use nalgebra::DMatrix;
use surfacecraft_geom::{Mesh, Point3, Vec3};

pub use crate::features_network::feature_network_with;
pub use crate::features_ngon::{NgonMode, NgonRegion, NgonSide};

/// Control net `net[i][j]` of a Bézier patch of any degree (`i` along u, `j` along v).
pub type NetN = Vec<Vec<[f64; 3]>>;

/// Parameters of [`feature_network`].
#[derive(Clone, Debug)]
pub struct FeatureOptions {
    /// Edges whose dihedral angle exceeds this are sharp (degrees).
    pub angle_deg: f64,
    /// Regions with fewer triangles are ignored.
    pub min_triangles: usize,
    /// Degree of the patches and feature curves (2 to 8).
    pub degree: usize,
}

impl Default for FeatureOptions {
    fn default() -> Self {
        Self { angle_deg: 35.0, min_triangles: 30, degree: 5 }
    }
}

/// The patch network of the feature curves.
#[derive(Clone, Debug, Default)]
pub struct FeatureNetwork {
    /// Corner points.
    pub corner_points: Vec<[f64; 3]>,
    /// One control net per patch.
    pub nets: Vec<NetN>,
    /// Corner ids of each patch (counter-clockwise seen from outside, matching the net's corners).
    pub corners: Vec<[u32; 4]>,
    /// Triangles (ids into the input mesh) of the region of each patch.
    pub region_triangles: Vec<Vec<u32>>,
    /// Regions that did not become patches, with the reason.
    pub skipped: Vec<String>,
    /// Distinct feature curves fitted.
    pub curves: usize,
    /// Total length of the sharp edges (mm).
    pub feature_length: f64,
    /// Smooth regions found (large enough).
    pub regions: usize,
    /// With [`NgonMode::Patch`]: the regions that are not four-sided (and have one outline), for
    /// the caller to cover with a patch each. Empty otherwise.
    pub ngons: Vec<NgonRegion>,
}

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

pub(crate) fn a3(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

/// Bernstein basis of degree `n` at `t`.
fn bernstein(n: usize, t: f64) -> Vec<f64> {
    let mut b = vec![0.0; n + 1];
    b[0] = 1.0;
    let s = 1.0 - t;
    for j in 1..=n {
        let mut saved = 0.0;
        for k in 0..j {
            let tmp = b[k];
            b[k] = saved + s * tmp;
            saved = t * tmp;
        }
        b[j] = saved;
    }
    b
}

/// Point of a Bézier patch.
#[must_use]
pub fn eval_net(net: &NetN, u: f64, v: f64) -> Vec3 {
    let n = net.len().saturating_sub(1);
    let (bu, bv) = (bernstein(n, u), bernstein(n, v));
    let mut p = Vec3::zeros();
    for (i, row) in net.iter().enumerate() {
        for (j, c) in row.iter().enumerate() {
            p += v3(*c) * (bu[i] * bv.get(j).copied().unwrap_or(0.0));
        }
    }
    p
}

/// Tessellation of Bézier patches (`n × n` quads each).
#[must_use]
pub fn tessellate_nets(nets: &[NetN], n: usize) -> Mesh {
    let n = n.max(1);
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for net in nets {
        let base = positions.len() as u32;
        for i in 0..=n {
            for j in 0..=n {
                positions.push(Point3::from(eval_net(net, i as f64 / n as f64, j as f64 / n as f64)));
            }
        }
        let w = (n + 1) as u32;
        for i in 0..n as u32 {
            for j in 0..n as u32 {
                let a = base + i * w + j;
                triangles.push([a, a + w, a + w + 1]);
                triangles.push([a, a + w + 1, a + 1]);
            }
        }
    }
    Mesh { positions, triangles }
}

pub(crate) fn bezier_point(c: &[Vec3], t: f64) -> Vec3 {
    let b = bernstein(c.len() - 1, t);
    c.iter().zip(&b).map(|(p, w)| p * *w).sum()
}

/// Degree-`n` Bézier curve through the first and last point of `pts`, least squares over the
/// rest (chord-length parameters).
pub(crate) fn fit_curve(pts: &[Vec3], n: usize) -> Vec<Vec3> {
    let (a, b) = (pts[0], pts[pts.len() - 1]);
    let m = pts.len();
    let mut ts = vec![0.0; m];
    for k in 1..m {
        ts[k] = ts[k - 1] + (pts[k] - pts[k - 1]).norm();
    }
    let total = ts[m - 1].max(1e-12);
    for t in &mut ts {
        *t /= total;
    }
    // straight chord control polygon as the fallback
    let line: Vec<Vec3> = (0..=n).map(|i| a + (b - a) * (i as f64 / n as f64)).collect();
    if m <= 2 || n < 2 {
        return line;
    }
    let free = n - 1;
    let mut ata = DMatrix::<f64>::zeros(free, free);
    let mut rhs = DMatrix::<f64>::zeros(free, 3);
    for (k, &t) in ts.iter().enumerate() {
        let bs = bernstein(n, t);
        let r = pts[k] - a * bs[0] - b * bs[n];
        for i in 0..free {
            for j in 0..free {
                ata[(i, j)] += bs[i + 1] * bs[j + 1];
            }
            for c in 0..3 {
                rhs[(i, c)] += bs[i + 1] * r[c];
            }
        }
    }
    let ridge = 1e-6 * (1.0 + ata.trace() / free as f64);
    for i in 0..free {
        ata[(i, i)] += ridge;
    }
    let Some(chol) = ata.cholesky() else { return line };
    let sol = chol.solve(&rhs);
    let mut out = vec![a];
    for i in 0..free {
        out.push(Vec3::new(sol[(i, 0)], sol[(i, 1)], sol[(i, 2)]));
    }
    out.push(b);
    if out.iter().any(|p| !p.iter().all(|x| x.is_finite())) { line } else { out }
}

/// De Casteljau split of the Bézier curve `c` at `t`: the control points of both parts.
pub(crate) fn split_bezier(c: &[Vec3], t: f64) -> (Vec<Vec3>, Vec<Vec3>) {
    let n = c.len();
    let (mut left, mut right) = (Vec::with_capacity(n), vec![Vec3::zeros(); n]);
    let mut row = c.to_vec();
    for k in 0..n {
        left.push(row[0]);
        right[n - 1 - k] = row[n - 1 - k];
        row = row.windows(2).map(|w| w[0] * (1.0 - t) + w[1] * t).collect();
    }
    (left, right)
}

struct Coons<'a> {
    bottom: &'a [Vec3],
    top: &'a [Vec3],
    left: &'a [Vec3],
    right: &'a [Vec3],
}

impl Coons<'_> {
    fn eval(&self, u: f64, v: f64) -> Vec3 {
        let (c0, c1) = (bezier_point(self.bottom, u), bezier_point(self.top, u));
        let (d0, d1) = (bezier_point(self.left, v), bezier_point(self.right, v));
        let (p00, p10) = (self.bottom[0], self.bottom[self.bottom.len() - 1]);
        let (p01, p11) = (self.top[0], self.top[self.top.len() - 1]);
        c0 * (1.0 - v) + c1 * v + d0 * (1.0 - u) + d1 * u
            - (p00 * ((1.0 - u) * (1.0 - v)) + p10 * (u * (1.0 - v)) + p01 * ((1.0 - u) * v) + p11 * (u * v))
    }
}

/// Parameters of `p` on a surface `f`: best grid node, then Gauss–Newton.
fn locate(f: &dyn Fn(f64, f64) -> Vec3, p: &Vec3, start: Option<(f64, f64)>) -> (f64, f64) {
    let (mut u, mut v) = start.unwrap_or_else(|| {
        let mut best = (f64::INFINITY, 0.5, 0.5);
        for i in 0..=16 {
            for j in 0..=16 {
                let (uu, vv) = (f64::from(i) / 16.0, f64::from(j) / 16.0);
                let d = (f(uu, vv) - p).norm_squared();
                if d < best.0 {
                    best = (d, uu, vv);
                }
            }
        }
        (best.1, best.2)
    });
    let h = 1e-4;
    for _ in 0..6 {
        let c = f(u, v);
        let fu =
            (f((u + h).min(1.0), v) - f((u - h).max(0.0), v)) / (((u + h).min(1.0)) - ((u - h).max(0.0))).max(1e-12);
        let fv =
            (f(u, (v + h).min(1.0)) - f(u, (v - h).max(0.0))) / (((v + h).min(1.0)) - ((v - h).max(0.0))).max(1e-12);
        let r = p - c;
        let (a, b, d) = (fu.dot(&fu), fu.dot(&fv), fv.dot(&fv));
        let det = a * d - b * b;
        if det.abs() < 1e-14 {
            break;
        }
        let (ru, rv) = (fu.dot(&r), fv.dot(&r));
        let du = (ru * d - rv * b) / det;
        let dv = (a * rv - b * ru) / det;
        let (nu, nv) = ((u + du).clamp(0.0, 1.0), (v + dv).clamp(0.0, 1.0));
        let moved = (nu - u).abs() + (nv - v).abs();
        u = nu;
        v = nv;
        if moved < 1e-9 {
            break;
        }
    }
    (u, v)
}

/// Fewest points for the least-squares fit of a patch interior.
const MIN_FIT_POINTS: usize = 8;

/// The four boundary curves of a patch net as control polygons: bottom, top, left, right.
fn net_sides(net: &NetN, n: usize) -> [Vec<Vec3>; 4] {
    let side = |f: &dyn Fn(usize) -> [f64; 3]| -> Vec<Vec3> { (0..=n).map(|k| v3(f(k))).collect() };
    [side(&|k| net[k][0]), side(&|k| net[k][n]), side(&|k| net[0][k]), side(&|k| net[n][k])]
}

/// Starts the interior of `net` from the Coons net of its boundary (exact for the blended boundary).
fn seed_coons_interior(net: &mut NetN, coons: &Coons<'_>, n: usize) {
    let (bottom, top, left, right) = (coons.bottom, coons.top, coons.left, coons.right);
    let corner = |i: usize, j: usize| -> Vec3 {
        let (s, t) = (i as f64 / n as f64, j as f64 / n as f64);
        bottom[0] * ((1.0 - s) * (1.0 - t)) + bottom[n] * (s * (1.0 - t)) + top[0] * ((1.0 - s) * t) + top[n] * (s * t)
    };
    for i in 1..n {
        for j in 1..n {
            let (s, t) = (i as f64 / n as f64, j as f64 / n as f64);
            let p = bottom[i] * (1.0 - t) + top[i] * t + left[j] * (1.0 - s) + right[j] * s - corner(i, j);
            net[i][j] = a3(p);
        }
    }
}

/// Surface parameters of every point (on the Coons patch in pass 0, on `net` after); `params` keeps
/// them as the starting values of the next pass.
fn locate_points(
    net: &NetN,
    coons: &Coons<'_>,
    points: &[Vec3],
    params: &mut [Option<(f64, f64)>],
    pass: usize,
) -> Vec<(f64, f64)> {
    let snapshot = net.clone();
    let surf = |u: f64, v: f64| eval_net(&snapshot, u, v);
    let cn = |u: f64, v: f64| coons.eval(u, v);
    let uv: Vec<(f64, f64)> = points
        .iter()
        .enumerate()
        .map(|(k, p)| if pass == 0 { locate(&cn, p, params[k]) } else { locate(&surf, p, params[k]) })
        .collect();
    for (k, q) in uv.iter().enumerate() {
        params[k] = Some(*q);
    }
    uv
}

/// `p` minus the contribution of the boundary control points of `net` at the Bernstein weights `bu`, `bv`.
fn boundary_residual(net: &NetN, p: &Vec3, bu: &[f64], bv: &[f64], n: usize) -> Vec3 {
    let mut r = *p;
    for i in 0..=n {
        for j in 0..=n {
            if i == 0 || j == 0 || i == n || j == n {
                r -= v3(net[i][j]) * (bu[i] * bv[j]);
            }
        }
    }
    r
}

/// Adds the terms of one point (weights `bu`, `bv`, residual `r`) to the normal equations.
fn add_point_terms(ata: &mut DMatrix<f64>, rhs: &mut DMatrix<f64>, n: usize, bu: &[f64], bv: &[f64], r: &Vec3) {
    for i in 1..n {
        for j in 1..n {
            let a = (i - 1) * (n - 1) + (j - 1);
            let wa = bu[i] * bv[j];
            for i2 in 1..n {
                for j2 in 1..n {
                    ata[(a, (i2 - 1) * (n - 1) + (j2 - 1))] += wa * bu[i2] * bv[j2];
                }
            }
            for c in 0..3 {
                rhs[(a, c)] += wa * r[c];
            }
        }
    }
}

/// Normal equations of the interior control points of `net` for the points at parameters `uv`.
fn interior_normal_equations(net: &NetN, points: &[Vec3], uv: &[(f64, f64)], n: usize) -> (DMatrix<f64>, DMatrix<f64>) {
    let free = (n - 1) * (n - 1);
    let mut ata = DMatrix::<f64>::zeros(free, free);
    let mut rhs = DMatrix::<f64>::zeros(free, 3);
    for (k, p) in points.iter().enumerate() {
        let (u, v) = uv[k];
        let (bu, bv) = (bernstein(n, u), bernstein(n, v));
        // boundary contribution
        let r = boundary_residual(net, p, &bu, &bv, n);
        add_point_terms(&mut ata, &mut rhs, n, &bu, &bv, &r);
    }
    (ata, rhs)
}

/// Regularises the normal equations toward the current interior of `net` (Coons on the first pass).
fn regularise(ata: &mut DMatrix<f64>, rhs: &mut DMatrix<f64>, net: &NetN, n: usize) {
    let free = (n - 1) * (n - 1);
    let lambda = 1e-4 * (1.0 + ata.trace() / free as f64);
    for i in 1..n {
        for j in 1..n {
            let a = (i - 1) * (n - 1) + (j - 1);
            ata[(a, a)] += lambda;
            for c in 0..3 {
                rhs[(a, c)] += lambda * net[i][j][c];
            }
        }
    }
}

/// Solves the normal equations and stores the finite solutions as the interior of `net`; false when
/// the matrix cannot be factorised (the net is then left as it was).
fn solve_interior(net: &mut NetN, ata: DMatrix<f64>, rhs: &DMatrix<f64>, n: usize) -> bool {
    let Some(chol) = ata.cholesky() else { return false };
    let sol = chol.solve(rhs);
    for i in 1..n {
        for j in 1..n {
            let a = (i - 1) * (n - 1) + (j - 1);
            let q = [sol[(a, 0)], sol[(a, 1)], sol[(a, 2)]];
            if q.iter().all(|x| x.is_finite()) {
                net[i][j] = q;
            }
        }
    }
    true
}

/// The interior of one patch: boundary control points fixed, interior least squares to `points`.
pub(crate) fn fit_interior(net: &mut NetN, points: &[Vec3]) {
    let n = net.len() - 1;
    if n < 2 {
        return;
    }
    let [bottom, top, left, right] = net_sides(net, n);
    let coons = Coons { bottom: &bottom, top: &top, left: &left, right: &right };
    seed_coons_interior(net, &coons, n);
    if points.len() < MIN_FIT_POINTS {
        return; // too few points to solve: the Coons interior stays
    }
    let mut params: Vec<Option<(f64, f64)>> = vec![None; points.len()];
    for pass in 0..3 {
        let uv = locate_points(net, &coons, points, &mut params, pass);
        let (mut ata, mut rhs) = interior_normal_equations(net, points, &uv, n);
        regularise(&mut ata, &mut rhs, net, n);
        if !solve_interior(net, ata, &rhs, n) {
            return;
        }
    }
}

/// Builds the patch network of the feature curves of the triangles `tris` of `mesh`; a region that
/// is not four-sided splits the whole network ([`NgonMode::Split`]).
///
/// # Errors
/// An error message for an invalid mesh, degree or empty region.
pub fn feature_network(mesh: &Mesh, tris: &[u32], opts: &FeatureOptions) -> Result<FeatureNetwork, String> {
    feature_network_with(mesh, tris, opts, NgonMode::Split)
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::synth;

    #[test]
    fn cube_gives_six_patches_and_twelve_curves() {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(10.0, 12.0, 8.0), 6);
        m.weld(1e-9);
        let all: Vec<u32> = (0..m.triangle_count() as u32).collect();
        let net =
            feature_network(&m, &all, &FeatureOptions { min_triangles: 4, degree: 3, ..FeatureOptions::default() })
                .unwrap();
        assert_eq!(net.nets.len(), 6, "{:?}", net.skipped);
        assert_eq!(net.curves, 12);
        assert_eq!(net.corner_points.len(), 8);
        assert!(net.skipped.is_empty());
        // every patch is a planar face of the cube: its centre lies on one of the six planes
        for n in &net.nets {
            let c = eval_net(n, 0.5, 0.5);
            let on = [
                c.x.abs() < 1e-3,
                (c.x - 10.0).abs() < 1e-3,
                c.y.abs() < 1e-3,
                (c.y - 12.0).abs() < 1e-3,
                c.z.abs() < 1e-3,
                (c.z - 8.0).abs() < 1e-3,
            ];
            assert!(on.iter().any(|&b| b), "centre {c:?}");
        }
        // bad parameters are errors
        assert!(feature_network(&m, &all, &FeatureOptions { degree: 1, ..FeatureOptions::default() }).is_err());
        assert!(feature_network(&m, &all, &FeatureOptions { angle_deg: 0.0, ..FeatureOptions::default() }).is_err());
    }

    #[test]
    fn split_bezier_halves_reproduce_the_curve() {
        let c: Vec<Vec3> = (0..6).map(|k| Vec3::new(k as f64, ((k * k) % 5) as f64, (k % 2) as f64)).collect();
        let (lo, hi) = split_bezier(&c, 0.5);
        assert_eq!(lo.last(), hi.first());
        for t in [0.0, 0.3, 0.77, 1.0] {
            assert!((bezier_point(&lo, t) - bezier_point(&c, 0.5 * t)).norm() < 1e-12);
            assert!((bezier_point(&hi, t) - bezier_point(&c, 0.5 + 0.5 * t)).norm() < 1e-12);
        }
    }
}

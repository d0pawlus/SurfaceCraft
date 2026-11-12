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
use crate::features_ngon::{RegionRec, ngon_patches, quad_patches};
use nalgebra::DMatrix;
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use surfacecraft_geom::{KdTree, Mesh, Point3, Vec3};

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
}

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

fn a3(v: Vec3) -> [f64; 3] {
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

fn bezier_point(c: &[Vec3], t: f64) -> Vec3 {
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

/// The interior of one patch: boundary control points fixed, interior least squares to `points`.
pub(crate) fn fit_interior(net: &mut NetN, points: &[Vec3]) {
    let n = net.len() - 1;
    if n < 2 || points.len() < 8 {
        return;
    }
    let side = |f: &dyn Fn(usize) -> [f64; 3]| -> Vec<Vec3> { (0..=n).map(|k| v3(f(k))).collect() };
    let bottom = side(&|k| net[k][0]);
    let top = side(&|k| net[k][n]);
    let left = side(&|k| net[0][k]);
    let right = side(&|k| net[n][k]);
    let coons = Coons { bottom: &bottom, top: &top, left: &left, right: &right };
    // the Coons net as the starting interior (exact for the blended boundary)
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
    let free = (n - 1) * (n - 1);
    let mut params: Vec<Option<(f64, f64)>> = vec![None; points.len()];
    for pass in 0..3 {
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
        let mut ata = DMatrix::<f64>::zeros(free, free);
        let mut rhs = DMatrix::<f64>::zeros(free, 3);
        for (k, p) in points.iter().enumerate() {
            let (u, v) = uv[k];
            let (bu, bv) = (bernstein(n, u), bernstein(n, v));
            // boundary contribution
            let mut r = *p;
            for i in 0..=n {
                for j in 0..=n {
                    if i == 0 || j == 0 || i == n || j == n {
                        r -= v3(net[i][j]) * (bu[i] * bv[j]);
                    }
                }
            }
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
        // regularise toward the current interior (Coons on the first pass)
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
        let Some(chol) = ata.cholesky() else { return };
        let sol = chol.solve(&rhs);
        for i in 1..n {
            for j in 1..n {
                let a = (i - 1) * (n - 1) + (j - 1);
                let q = [sol[(a, 0)], sol[(a, 1)], sol[(a, 2)]];
                if q.iter().all(|x| x.is_finite()) {
                    net[i][j] = q;
                }
            }
        }
    }
}

/// Builds the patch network of the feature curves of the triangles `tris` of `mesh`.
///
/// # Errors
/// An error message for an invalid mesh, degree or empty region.
pub fn feature_network(mesh: &Mesh, tris: &[u32], opts: &FeatureOptions) -> Result<FeatureNetwork, String> {
    if !(2..=8).contains(&opts.degree) {
        return Err("degree must be 2..=8".into());
    }
    if !(opts.angle_deg.is_finite() && opts.angle_deg > 1.0 && opts.angle_deg < 175.0) {
        return Err("angle must be between 1 and 175 degrees".into());
    }
    let sub = mesh.submesh(tris);
    if sub.triangle_count() < 4 {
        return Err("too few triangles".into());
    }
    let nt = sub.triangle_count();
    let nb = sub.triangle_neighbours();
    let fnorm: Vec<Vec3> = (0..nt).map(|i| sub.face_normal(i).unwrap_or_else(Vec3::z)).collect();
    let cos = opts.angle_deg.to_radians().cos();
    let sharp = |t: usize, k: usize| -> bool {
        let n = nb[t][k];
        n != u32::MAX && fnorm[t].dot(&fnorm[n as usize]) < cos
    };
    // regions: flood fill across smooth edges
    let mut region = vec![u32::MAX; nt];
    let mut sizes: Vec<usize> = Vec::new();
    for s in 0..nt {
        if region[s] != u32::MAX {
            continue;
        }
        let id = sizes.len() as u32;
        let mut count = 0;
        let mut queue = VecDeque::from([s]);
        region[s] = id;
        while let Some(t) = queue.pop_front() {
            count += 1;
            for k in 0..3 {
                let n = nb[t][k];
                if n != u32::MAX && region[n as usize] == u32::MAX && !sharp(t, k) {
                    region[n as usize] = id;
                    queue.push_back(n as usize);
                }
            }
        }
        sizes.push(count);
    }
    // tiny regions (spikes, noise) join the large neighbour they share most edges with
    for _ in 0..4 {
        let mut size = vec![0usize; sizes.len()];
        for &r in &region {
            size[r as usize] += 1;
        }
        let mut votes: BTreeMap<(u32, u32), usize> = BTreeMap::new();
        for t in 0..nt {
            for k in 0..3 {
                let n = nb[t][k];
                if n == u32::MAX {
                    continue;
                }
                let (a, b) = (region[t], region[n as usize]);
                if a != b && size[a as usize] < opts.min_triangles && size[b as usize] >= opts.min_triangles {
                    *votes.entry((a, b)).or_insert(0) += 1;
                }
            }
        }
        let mut best: BTreeMap<u32, (usize, u32)> = BTreeMap::new();
        for (&(a, b), &v) in &votes {
            let e = best.entry(a).or_insert((0, b));
            if v > e.0 {
                *e = (v, b);
            }
        }
        if best.is_empty() {
            break;
        }
        for r in &mut region {
            if let Some(&(_, b)) = best.get(r) {
                *r = b;
            }
        }
    }
    let mut sizes = vec![0usize; sizes.len()];
    for &r in &region {
        sizes[r as usize] += 1;
    }
    let big: Vec<bool> = sizes.iter().map(|&c| c >= opts.min_triangles).collect();
    let live = |t: usize| big.get(region[t] as usize).copied().unwrap_or(false);
    // vertices → regions touching them
    let nv = sub.positions.len();
    let mut vreg: Vec<BTreeSet<u32>> = vec![BTreeSet::new(); nv];
    let mut edge_len_sum = 0.0;
    let mut edge_n = 0usize;
    let mut feature_length = 0.0;
    for t in 0..nt {
        if !live(t) {
            continue;
        }
        let tri = sub.triangles[t];
        for k in 0..3 {
            let (a, b) = (tri[k] as usize, tri[(k + 1) % 3] as usize);
            vreg[a].insert(region[t]);
            let l = (sub.positions[b] - sub.positions[a]).norm();
            edge_len_sum += l;
            edge_n += 1;
            if nb[t][k] == u32::MAX || sharp(t, k) {
                if nb[t][k] == u32::MAX || (t as u32) < nb[t][k] {
                    feature_length += l;
                }
                // the mesh boundary counts as one more region
                if nb[t][k] == u32::MAX {
                    vreg[a].insert(u32::MAX);
                    vreg[b].insert(u32::MAX);
                }
            }
        }
    }
    let mean_edge = edge_len_sum / edge_n.max(1) as f64;
    let mut out =
        FeatureNetwork { feature_length, regions: big.iter().filter(|&&b| b).count(), ..FeatureNetwork::default() };
    // corners: junction vertices clustered by distance
    let junction: Vec<u32> = (0..nv as u32).filter(|&v| vreg[v as usize].len() >= 3).collect();
    let jpts: Vec<Point3> = junction.iter().map(|&v| sub.positions[v as usize]).collect();
    let tree = KdTree::build(&jpts);
    let mut parent: Vec<usize> = (0..junction.len()).collect();
    fn find(p: &mut [usize], mut x: usize) -> usize {
        while p[x] != x {
            p[x] = p[p[x]];
            x = p[x];
        }
        x
    }
    for (i, p) in jpts.iter().enumerate() {
        for j in tree.within(p, 2.5 * mean_edge) {
            let (a, b) = (find(&mut parent, i), find(&mut parent, j));
            if a != b {
                parent[a.max(b)] = a.min(b);
            }
        }
    }
    let mut cluster_of_vertex: BTreeMap<u32, u32> = BTreeMap::new();
    let mut cluster_ids: BTreeMap<usize, u32> = BTreeMap::new();
    let mut sums: Vec<(Vec3, usize)> = Vec::new();
    for (i, &v) in junction.iter().enumerate() {
        let r = find(&mut parent, i);
        let id = *cluster_ids.entry(r).or_insert_with(|| {
            sums.push((Vec3::zeros(), 0));
            (sums.len() - 1) as u32
        });
        cluster_of_vertex.insert(v, id);
        sums[id as usize].0 += sub.positions[v as usize].coords;
        sums[id as usize].1 += 1;
    }
    let cpoints: Vec<Vec3> = sums.iter().map(|(s, n)| s / *n as f64).collect();
    out.corner_points = cpoints.iter().map(|p| a3(*p)).collect();
    // boundary half-edges (triangle, edge) of each live region, the region on the left of the winding
    let is_boundary = |t: usize, k: usize| -> bool {
        let n = nb[t][k];
        n == u32::MAX || region[n as usize] != region[t] || !live(n as usize)
    };
    let mut boundary: BTreeMap<u32, BTreeSet<(usize, usize)>> = BTreeMap::new();
    for t in 0..nt {
        if !live(t) {
            continue;
        }
        for k in 0..3 {
            if is_boundary(t, k) {
                boundary.entry(region[t]).or_default().insert((t, k));
            }
        }
    }
    // the boundary half-edge after (t, k): rotate about its end vertex through the region
    // (so a vertex the region touches twice is walked as two visits, not as a dead end)
    let next_half_edge = |t: usize, k: usize| -> Option<(usize, usize)> {
        let (mut ct, mut ck) = (t, (k + 1) % 3);
        for _ in 0..4096 {
            if is_boundary(ct, ck) {
                return Some((ct, ck));
            }
            let n = nb[ct][ck] as usize;
            let (va, vb) = (sub.triangles[ct][ck], sub.triangles[ct][(ck + 1) % 3]);
            let nk = (0..3).find(|&j| sub.triangles[n][j] == vb && sub.triangles[n][(j + 1) % 3] == va)?;
            ct = n;
            ck = (nk + 1) % 3;
        }
        None
    };
    let nrm = sub.vertex_normals();
    let degree = opts.degree;
    let mut recs: Vec<RegionRec> = Vec::new();
    let mut side_ids: BTreeMap<(u32, u32, u32, usize), usize> = BTreeMap::new();
    let mut side_curves: Vec<Vec<Vec3>> = Vec::new();
    for (&rid, set) in &boundary {
        let tri_ids: Vec<u32> =
            (0..nt).filter(|&t| region[t] == rid).map(|t| tris.get(t).copied().unwrap_or(0)).collect();
        let mut visited: BTreeSet<(usize, usize)> = BTreeSet::new();
        let mut loops: Vec<Vec<u32>> = Vec::new();
        let mut broken = false;
        for &h in set {
            if visited.contains(&h) {
                continue;
            }
            let mut lp = Vec::new();
            let mut cur = h;
            loop {
                visited.insert(cur);
                lp.push(sub.triangles[cur.0][cur.1]);
                let Some(n) = next_half_edge(cur.0, cur.1) else {
                    broken = true;
                    break;
                };
                if n == h {
                    break;
                }
                if visited.contains(&n) {
                    broken = true;
                    break;
                }
                cur = n;
            }
            loops.push(lp);
        }
        // the longest loop is the outline; small ones are holes of a few triangles, which the
        // patch simply covers
        loops.sort_by_key(|l| std::cmp::Reverse(l.len()));
        if broken || loops.is_empty() || loops.get(1).is_some_and(|l| l.len() > 12) {
            out.skipped.push(format!(
                "region of {} triangles: {} boundary loops of lengths {:?} (a patch needs one outline)",
                tri_ids.len(),
                loops.len(),
                loops.iter().map(Vec::len).collect::<Vec<_>>()
            ));
            continue;
        }
        let lp = loops.swap_remove(0);
        // corners along the loop: runs of vertices of one cluster
        let tag: Vec<Option<u32>> = lp.iter().map(|v| cluster_of_vertex.get(v).copied()).collect();
        let m = lp.len();
        // start the walk just after a non-corner vertex so that no run wraps around
        let Some(origin) = (0..m).find(|&i| tag[i].is_none()) else {
            out.skipped.push(format!("region of {} triangles: the boundary is all corners", tri_ids.len()));
            continue;
        };
        let mut corners: Vec<(u32, usize, usize)> = Vec::new(); // (cluster, first index, last index) in walk order
        for step in 0..m {
            let i = (origin + step) % m;
            if let Some(c) = tag[i] {
                match corners.last_mut() {
                    Some(l) if l.0 == c && l.2 + 1 == step => l.2 = step,
                    _ => corners.push((c, step, step)),
                }
            }
        }
        // the same cluster must not appear twice
        let distinct: BTreeSet<u32> = corners.iter().map(|c| c.0).collect();
        if corners.len() < 3 || distinct.len() != corners.len() {
            out.skipped.push(format!(
                "region of {} triangles has {} corners (a patch needs at least 3, all distinct)",
                tri_ids.len(),
                corners.len()
            ));
            continue;
        }
        // loop vertex at walk position
        let at = |step: usize| lp[(origin + step) % m];
        let count = corners.len();
        let mut rec = RegionRec {
            corners: corners.iter().map(|c| c.0).collect(),
            sides: Vec::with_capacity(count),
            forward: Vec::with_capacity(count),
            points: Vec::new(),
            mean_normal: lp.iter().map(|&v| nrm[v as usize]).sum(),
            triangles: tri_ids,
        };
        // register the sides (canonical direction: smaller corner id first)
        for c in 0..count {
            let (ca, _, last_a) = corners[c];
            let (cb, first_b, _) = corners[(c + 1) % count];
            let mut pts = vec![cpoints[ca as usize]];
            let end = if (c + 1) % count == 0 { m + first_b } else { first_b };
            for step in (last_a + 1)..end {
                pts.push(sub.positions[at(step % m) as usize].coords);
            }
            pts.push(cpoints[cb as usize]);
            let forward = ca < cb;
            if !forward {
                pts.reverse();
            }
            let (lo, hi) = (ca.min(cb), ca.max(cb));
            // first interior vertex id keeps two curves between the same corners apart
            let tagv = if pts.len() > 2 {
                let q = pts[1];
                ((q.x * 1e3).round() as i64 as u32) ^ ((q.y * 1e3).round() as i64 as u32).rotate_left(11)
            } else {
                u32::MAX
            };
            let id = *side_ids.entry((lo, hi, tagv, pts.len())).or_insert_with(|| {
                side_curves.push(fit_curve(&pts, degree));
                side_curves.len() - 1
            });
            rec.sides.push(id);
            rec.forward.push(forward);
        }
        let mut seen: BTreeSet<u32> = BTreeSet::new();
        for t in 0..nt {
            if region[t] == rid {
                seen.extend(sub.triangles[t]);
            }
        }
        rec.points = seen.into_iter().map(|v| sub.positions[v as usize].coords).collect();
        recs.push(rec);
    }
    out.curves = side_curves.len();
    let patches = if recs.iter().all(|r| r.corners.len() == 4) {
        quad_patches(&recs, &side_curves)
    } else {
        let first_new = out.corner_points.len() as u32;
        ngon_patches(&recs, &side_curves, first_new, degree)
    };
    out.corner_points.extend(patches.extra_points.iter().copied());
    out.nets = patches.nets;
    out.corners = patches.corners;
    out.region_triangles = patches.region_triangles;
    Ok(out)
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

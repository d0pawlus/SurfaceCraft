//! One bicubic Bézier patch per quad of an unstructured layout, with shared boundaries and
//! shared tangent planes (design doc §6.3, milestone P4).
//!
//! APP-GEOMETRY: stands in for a solidcore Bézier patch network fit with G1 boundary constraints (and
//! Bézier patch evaluation); the B-rep of the result is built by `kernel::nurbs`.
//!
//! Neighbouring patches meet along a common cubic Bézier edge (so the network is watertight by
//! construction) and have the same tangent plane at every layout vertex (the plane of the
//! vertex normal, which every edge leaving the vertex lies in). Along an edge the cross-boundary
//! control rows are interpolated from the two end tangents and flattened into one plane per
//! edge that both neighbours compute identically; the two predictions of an interior control
//! point (one from each adjacent side) are averaged. The tangent planes therefore agree exactly
//! at the vertices and approximately along the edges: the jump is *measured*
//! ([`edge_normal_jumps`]), not claimed zero. Exact G1 along edges at vertices of any valence
//! needs the vertex-enclosure construction (a dedicated kernel primitive, see docs/STATUS.md).
use crate::mls::ScanSurface;
use crate::retopo::QuadMesh;
use std::collections::BTreeMap;
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3, normalize};

/// Control net of a bicubic patch, `net[i][j]` with `i` along u and `j` along v.
pub type Net = [[[f64; 3]; 4]; 4];

/// The patches of a quad layout.
#[derive(Clone, Debug, Default)]
pub struct PatchSet {
    /// One control net per quad, corners `[0][0], [3][0], [3][3], [0][3]` = the quad's vertices.
    pub nets: Vec<Net>,
    /// Layout vertex ids of each patch's corners.
    pub corners: Vec<[u32; 4]>,
    /// Layout vertices.
    pub vertices: Vec<[f64; 3]>,
}

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

fn a3(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn tangent_toward(n: &Vec3, d: &Vec3) -> Vec3 {
    normalize(d - n * d.dot(n)).unwrap_or(*d)
}

/// The cubic edge `a → b`: end tangents in the vertex tangent planes, handle lengths fitted to
/// the scan through three projected chord points.
fn edge_curve(pa: &Point3, na: &Vec3, pb: &Point3, nb: &Vec3, scan: &ScanSurface, radius: f64) -> [Vec3; 4] {
    let chord = pb - pa;
    let len = chord.norm();
    let ta = tangent_toward(na, &chord);
    let tb = tangent_toward(nb, &chord);
    let (mut alpha, mut beta) = (len / 3.0, len / 3.0);
    let mut sa = [[0.0f64; 2]; 2];
    let mut rhs = [0.0f64; 2];
    let mut used = 0;
    for &t in &[0.25, 0.5, 0.75] {
        let q = Point3::from(pa.coords * (1.0 - t) + pb.coords * t);
        let hint = normalize(na * (1.0 - t) + nb * t).unwrap_or(*na);
        let Some((s, _)) = scan.project(&q, radius, &hint) else { continue };
        let b0 = (1.0 - t).powi(3);
        let b1 = 3.0 * t * (1.0 - t).powi(2);
        let b2 = 3.0 * t * t * (1.0 - t);
        let b3 = t.powi(3);
        let r = s.coords - pa.coords * (b0 + b1) - pb.coords * (b2 + b3);
        let (u, w) = (ta * b1, -tb * b2);
        sa[0][0] += u.dot(&u);
        sa[0][1] += u.dot(&w);
        sa[1][1] += w.dot(&w);
        rhs[0] += u.dot(&r);
        rhs[1] += w.dot(&r);
        used += 1;
    }
    if used >= 2 {
        let det = sa[0][0] * sa[1][1] - sa[0][1] * sa[0][1];
        if det.abs() > 1e-12 * (sa[0][0] * sa[1][1]).abs().max(1e-30) {
            let x = (rhs[0] * sa[1][1] - rhs[1] * sa[0][1]) / det;
            let y = (sa[0][0] * rhs[1] - sa[0][1] * rhs[0]) / det;
            if x.is_finite() && y.is_finite() {
                alpha = x.clamp(0.15 * len, 0.6 * len);
                beta = y.clamp(0.15 * len, 0.6 * len);
            }
        }
    }
    [pa.coords, pa.coords + ta * alpha, pb.coords - tb * beta, pb.coords]
}

/// The plane normal the cross-boundary row of edge `b` (control points, end normals) is
/// flattened into at parameter `t`: the interpolated vertex normal made perpendicular to the edge.
fn edge_plane_normal(b: &[Vec3; 4], na: &Vec3, nb: &Vec3, t: f64) -> Vec3 {
    let n = na * (1.0 - t) + nb * t;
    let d =
        ((b[1] - b[0]) * (1.0 - t) * (1.0 - t) + (b[2] - b[1]) * (2.0 * t * (1.0 - t)) + (b[3] - b[2]) * (t * t)) * 3.0;
    let n = match normalize(d) {
        Some(th) => n - th * n.dot(&th),
        None => n,
    };
    normalize(n).unwrap_or_else(Vec3::z)
}

/// Predicted second control row of a side: `P_k + d_k / 3`, `k = 1, 2`, with `d` interpolating
/// the corner cross-derivatives `d0`, `d3` and flattened into the edge's plane.
fn second_row(b: &[Vec3; 4], d0: &Vec3, d3: &Vec3, na: &Vec3, nb: &Vec3) -> [Vec3; 2] {
    let mut out = [Vec3::zeros(); 2];
    for k in 1..3usize {
        let t = k as f64 / 3.0;
        let d = if k == 1 { (d0 * 2.0 + d3) / 3.0 } else { (d0 + d3 * 2.0) / 3.0 };
        let n = edge_plane_normal(b, na, nb, t);
        out[k - 1] = b[k] + (d - n * d.dot(&n)) / 3.0;
    }
    out
}

/// Builds the patches of `q`; `scan` shapes the edges and `radius` is its projection support.
#[must_use]
pub fn build_patches(q: &QuadMesh, scan: &ScanSurface, radius: f64) -> PatchSet {
    let mut curves: BTreeMap<(u32, u32), [Vec3; 4]> = BTreeMap::new();
    for f in &q.faces {
        for k in 0..4 {
            let (a, b) = (f[k], f[(k + 1) % 4]);
            let key = (a.min(b), a.max(b));
            if curves.contains_key(&key) {
                continue;
            }
            let (Some(pa), Some(pb), Some(na), Some(nb)) = (
                q.vertices.get(key.0 as usize),
                q.vertices.get(key.1 as usize),
                q.normals.get(key.0 as usize),
                q.normals.get(key.1 as usize),
            ) else {
                continue;
            };
            curves.insert(key, edge_curve(pa, na, pb, nb, scan, radius));
        }
    }
    let side = |a: u32, b: u32| -> Option<[Vec3; 4]> {
        let c = curves.get(&(a.min(b), a.max(b)))?;
        Some(if a < b { *c } else { [c[3], c[2], c[1], c[0]] })
    };
    let mut set = PatchSet {
        nets: Vec::with_capacity(q.faces.len()),
        corners: Vec::with_capacity(q.faces.len()),
        vertices: q.vertices.iter().map(|p| [p.x, p.y, p.z]).collect(),
    };
    for f in &q.faces {
        let (Some(bot), Some(right), Some(top), Some(left)) =
            (side(f[0], f[1]), side(f[1], f[2]), side(f[3], f[2]), side(f[0], f[3]))
        else {
            continue;
        };
        let n = |i: u32| q.normals.get(i as usize).copied().unwrap_or_else(Vec3::z);
        let mut p = [[Vec3::zeros(); 4]; 4];
        for k in 0..4 {
            p[k][0] = bot[k];
            p[3][k] = right[k];
            p[k][3] = top[k];
            p[0][k] = left[k];
        }
        let mut acc = [[Vec3::zeros(); 4]; 4];
        let mut cnt = [[0u8; 4]; 4];
        let mut add = |i: usize, j: usize, v: Vec3| {
            acc[i][j] += v;
            cnt[i][j] += 1;
        };
        // bottom (v = 0): rows P[i][1]
        let sr = second_row(&bot, &((p[0][1] - p[0][0]) * 3.0), &((p[3][1] - p[3][0]) * 3.0), &n(f[0]), &n(f[1]));
        add(1, 1, sr[0]);
        add(2, 1, sr[1]);
        // top (v = 1): rows P[i][2]
        let sr = second_row(&top, &((p[0][2] - p[0][3]) * 3.0), &((p[3][2] - p[3][3]) * 3.0), &n(f[3]), &n(f[2]));
        add(1, 2, sr[0]);
        add(2, 2, sr[1]);
        // left (u = 0): columns P[1][j]
        let sr = second_row(&left, &((p[1][0] - p[0][0]) * 3.0), &((p[1][3] - p[0][3]) * 3.0), &n(f[0]), &n(f[3]));
        add(1, 1, sr[0]);
        add(1, 2, sr[1]);
        // right (u = 1): columns P[2][j]
        let sr = second_row(&right, &((p[2][0] - p[3][0]) * 3.0), &((p[2][3] - p[3][3]) * 3.0), &n(f[1]), &n(f[2]));
        add(2, 1, sr[0]);
        add(2, 2, sr[1]);
        for i in 1..3 {
            for j in 1..3 {
                p[i][j] = acc[i][j] / f64::from(cnt[i][j].max(1));
            }
        }
        let mut net = [[[0.0; 3]; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                net[i][j] = a3(p[i][j]);
            }
        }
        set.nets.push(net);
        set.corners.push(*f);
    }
    set
}

fn bern(t: f64) -> [f64; 4] {
    let s = 1.0 - t;
    [s * s * s, 3.0 * t * s * s, 3.0 * t * t * s, t * t * t]
}

fn dbern(t: f64) -> [f64; 4] {
    let s = 1.0 - t;
    [-3.0 * s * s, 3.0 * s * s - 6.0 * t * s, 6.0 * t * s - 3.0 * t * t, 3.0 * t * t]
}

/// The point and the two partial derivatives of `net` at `(u, v)`.
#[must_use]
pub fn eval(net: &Net, u: f64, v: f64) -> (Vec3, Vec3, Vec3) {
    let (bu, bv, du, dv) = (bern(u), bern(v), dbern(u), dbern(v));
    let (mut p, mut pu, mut pv) = (Vec3::zeros(), Vec3::zeros(), Vec3::zeros());
    for i in 0..4 {
        for j in 0..4 {
            let c = v3(net[i][j]);
            p += c * (bu[i] * bv[j]);
            pu += c * (du[i] * bv[j]);
            pv += c * (bu[i] * dv[j]);
        }
    }
    (p, pu, pv)
}

/// Unit normal of `net` at `(u, v)` (None where the partials are parallel).
#[must_use]
pub fn normal(net: &Net, u: f64, v: f64) -> Option<Vec3> {
    let (_, pu, pv) = eval(net, u, v);
    normalize(pu.cross(&pv))
}

/// Parameters `(u, v)` at fraction `t` of the loop side `s` (0 bottom, 1 right, 2 top, 3 left),
/// walking the loop `c0 → c1 → c2 → c3`.
fn side_uv(s: usize, t: f64) -> (f64, f64) {
    match s {
        0 => (t, 0.0),
        1 => (1.0, t),
        2 => (1.0 - t, 1.0),
        _ => (0.0, 1.0 - t),
    }
}

/// Angles (rad) between the normals of the two patches on both sides of every interior edge,
/// sampled at 7 interior points of the edge.
#[must_use]
pub fn edge_normal_jumps(set: &PatchSet) -> Vec<f64> {
    let mut by_edge: BTreeMap<(u32, u32), Vec<(usize, usize, bool)>> = BTreeMap::new();
    for (fi, c) in set.corners.iter().enumerate() {
        for s in 0..4 {
            let (a, b) = (c[s], c[(s + 1) % 4]);
            by_edge.entry((a.min(b), a.max(b))).or_default().push((fi, s, a < b));
        }
    }
    let mut out = Vec::new();
    for uses in by_edge.values() {
        let [(f0, s0, fw0), (f1, s1, fw1)] = uses.as_slice() else { continue };
        let (Some(n0), Some(n1)) = (set.nets.get(*f0), set.nets.get(*f1)) else { continue };
        let mut worst: f64 = 0.0;
        for k in 1..8 {
            let t = f64::from(k) / 8.0;
            // t runs a → b with a < b: patch uses running the other way read 1 − t
            let (u0, v0) = side_uv(*s0, if *fw0 { t } else { 1.0 - t });
            let (u1, v1) = side_uv(*s1, if *fw1 { t } else { 1.0 - t });
            if let (Some(a), Some(b)) = (normal(n0, u0, v0), normal(n1, u1, v1)) {
                worst = worst.max(a.dot(&b).clamp(-1.0, 1.0).acos());
            }
        }
        out.push(worst);
    }
    out
}

/// Tessellation of the patches (`n × n` quads each, two triangles per quad).
#[must_use]
pub fn tessellate(set: &PatchSet, n: usize) -> Mesh {
    let n = n.max(1);
    let mut positions = Vec::with_capacity(set.nets.len() * (n + 1) * (n + 1));
    let mut triangles = Vec::with_capacity(set.nets.len() * n * n * 2);
    for net in &set.nets {
        let base = positions.len() as u32;
        for i in 0..=n {
            for j in 0..=n {
                let (p, _, _) = eval(net, i as f64 / n as f64, j as f64 / n as f64);
                positions.push(Point3::from(p));
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

/// Deviation of scan points from the patches.
#[derive(Clone, Copy, Debug, Default)]
pub struct Deviation {
    /// Largest distance of a covered point (mm).
    pub max: f64,
    /// RMS distance of the covered points (mm).
    pub rms: f64,
    /// Points counted.
    pub measured: usize,
    /// Share of points whose nearest patch point is on the free boundary (beyond the layout).
    pub uncovered: f64,
}

/// Distances of `points` from the patches (tessellated `n × n` per patch). Points whose nearest
/// patch point is on a free boundary edge of the network are not counted (the layout does not
/// reach them) and reported as `uncovered`.
#[must_use]
pub fn deviation(set: &PatchSet, points: &[Point3], n: usize) -> Deviation {
    let tess = tessellate(set, n);
    // edges of the tessellation used by one triangle only
    let mut count: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for t in &tess.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            *count.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    // interior seams between patches also appear as single-use edges of the per-patch grids
    // when patches do not share tessellation vertices; those are matched by position
    let mut seam: BTreeMap<[i64; 6], u32> = BTreeMap::new();
    let key = |a: &Point3, b: &Point3| -> [i64; 6] {
        let q = |x: f64| (x * 1e6).round() as i64;
        let (ka, kb) = ([q(a.x), q(a.y), q(a.z)], [q(b.x), q(b.y), q(b.z)]);
        if ka <= kb { [ka[0], ka[1], ka[2], kb[0], kb[1], kb[2]] } else { [kb[0], kb[1], kb[2], ka[0], ka[1], ka[2]] }
    };
    for (&(a, b), &c) in &count {
        if c == 1
            && let (Some(pa), Some(pb)) = (tess.positions.get(a as usize), tess.positions.get(b as usize))
        {
            *seam.entry(key(pa, pb)).or_insert(0) += 1;
        }
    }
    let bvh = Bvh::build(&tess);
    let (mut max, mut sq, mut measured, mut uncovered) = (0.0f64, 0.0, 0usize, 0usize);
    for p in points {
        let Some(h) = bvh.closest(p, f64::INFINITY) else { continue };
        let on_free_edge = tess.triangles.get(h.triangle).is_some_and(|t| {
            (0..3).any(|k| {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let (Some(pa), Some(pb)) = (tess.positions.get(a as usize), tess.positions.get(b as usize)) else {
                    return false;
                };
                if seam.get(&key(pa, pb)).copied().unwrap_or(0) != 1 {
                    return false;
                }
                // is the hit point on this edge?
                let e = pb - pa;
                let l2 = e.norm_squared().max(1e-30);
                let s = ((h.point - pa).dot(&e) / l2).clamp(0.0, 1.0);
                (pa + e * s - h.point).norm() < 1e-7
            })
        });
        if on_free_edge && h.distance > 1e-6 {
            uncovered += 1;
            continue;
        }
        max = max.max(h.distance);
        sq += h.distance * h.distance;
        measured += 1;
    }
    Deviation {
        max,
        rms: (sq / measured.max(1) as f64).sqrt(),
        measured,
        uncovered: uncovered as f64 / points.len().max(1) as f64,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::retopo::{PolyMesh, RetopoOptions, retopo};
    use surfacecraft_geom::{Rng, synth};

    #[test]
    fn sphere_patches_are_accurate_and_nearly_g1() {
        let mut scan = synth::sphere(Point3::origin(), 20.0, 120, 60);
        synth::add_noise(&mut scan, 0.03, &mut Rng::new(5));
        let (poly, rep) = retopo(&scan, &RetopoOptions { target_faces: 100, ..RetopoOptions::default() }).unwrap();
        eprintln!("layout {rep:?}");
        let ss = ScanSurface::new(&scan.positions);
        let radius = 4.0;
        let project = |p: &Point3, n: &Vec3| ss.project(p, radius, n);
        let poly = PolyMesh {
            vertices: poly.vertices.iter().zip(&poly.normals).map(|(p, n)| project(p, n).map_or(*p, |x| x.0)).collect(),
            normals: poly.vertices.iter().zip(&poly.normals).map(|(p, n)| project(p, n).map_or(*n, |x| x.1)).collect(),
            faces: poly.faces,
        };
        let quads = poly.split_to_quads(&project);
        let set = build_patches(&quads, &ss, radius);
        assert_eq!(set.nets.len(), quads.faces.len());
        let dev = deviation(&set, &scan.positions, 12);
        let jumps = edge_normal_jumps(&set);
        let worst = jumps.iter().copied().fold(0.0, f64::max);
        let mean = jumps.iter().sum::<f64>() / jumps.len() as f64;
        eprintln!(
            "patches {}: max dev {:.4} rms {:.4}; normal jump max {:.3} deg mean {:.3} deg over {} edges",
            set.nets.len(),
            dev.max,
            dev.rms,
            worst.to_degrees(),
            mean.to_degrees(),
            jumps.len()
        );
        assert!(dev.max < 0.2, "{dev:?}");
        assert!(worst.to_degrees() < 5.0);
    }
}

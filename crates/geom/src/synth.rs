//! Synthetic scans: meshes sampled from known shapes, with optional Gaussian noise along the
//! normal and outliers. The ground truth is exact, so the thresholds of design doc §11.1 can be
//! tested in CI without any third-party scan (clean-room corpus level 1).
use crate::{Mesh, Point3, Rng, Vec3};
use std::f64::consts::{PI, TAU};

fn grid(nu: usize, nv: usize, wrap_u: bool, f: impl Fn(f64, f64) -> Point3) -> Mesh {
    let nu = nu.max(1);
    let nv = nv.max(1);
    let cols = if wrap_u { nu } else { nu + 1 };
    let mut positions = Vec::with_capacity(cols * (nv + 1));
    for j in 0..=nv {
        for i in 0..cols {
            positions.push(f(i as f64 / nu as f64, j as f64 / nv as f64));
        }
    }
    let mut triangles = Vec::with_capacity(nu * nv * 2);
    for j in 0..nv {
        for i in 0..nu {
            let i1 = if wrap_u { (i + 1) % nu } else { i + 1 };
            let a = (j * cols + i) as u32;
            let b = (j * cols + i1) as u32;
            let c = ((j + 1) * cols + i1) as u32;
            let d = ((j + 1) * cols + i) as u32;
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    Mesh { positions, triangles }
}

/// A `w × h` rectangle in the plane through `origin` spanned by unit `u`, `v` (normal `u × v`).
#[must_use]
pub fn plane(origin: Point3, u: Vec3, v: Vec3, w: f64, h: f64, nu: usize, nv: usize) -> Mesh {
    grid(nu, nv, false, |s, t| origin + u * (s * w) + v * (t * h))
}

/// A UV sphere (poles welded).
#[must_use]
pub fn sphere(center: Point3, r: f64, nu: usize, nv: usize) -> Mesh {
    let mut m = grid(nu, nv, true, |s, t| {
        let th = TAU * s;
        let ph = PI * t;
        center + Vec3::new(ph.sin() * th.cos(), ph.sin() * th.sin(), -ph.cos()) * r
    });
    m.weld(r * 1e-9);
    m
}

/// An open cylinder of radius `r` and height `h` along `axis` (unit) from `base`.
#[must_use]
pub fn cylinder(base: Point3, axis: Vec3, r: f64, h: f64, nu: usize, nv: usize) -> Mesh {
    let a = crate::normalize(axis).unwrap_or_else(Vec3::z);
    let x = crate::any_perpendicular(&a);
    let y = a.cross(&x);
    grid(nu, nv, true, |s, t| {
        let th = TAU * s;
        base + (x * th.cos() + y * th.sin()) * r + a * (t * h)
    })
}

/// An open cone frustum from radius `r0` at `base` to `r1` at height `h` along `axis`.
#[must_use]
pub fn cone(base: Point3, axis: Vec3, r0: f64, r1: f64, h: f64, nu: usize, nv: usize) -> Mesh {
    let a = crate::normalize(axis).unwrap_or_else(Vec3::z);
    let x = crate::any_perpendicular(&a);
    let y = a.cross(&x);
    grid(nu, nv, true, |s, t| {
        let th = TAU * s;
        let r = r0 + (r1 - r0) * t;
        base + (x * th.cos() + y * th.sin()) * r + a * (t * h)
    })
}

/// A torus with major radius `big_r` and minor radius `r` around `axis` through `center`.
#[must_use]
pub fn torus(center: Point3, axis: Vec3, big_r: f64, r: f64, nu: usize, nv: usize) -> Mesh {
    let a = crate::normalize(axis).unwrap_or_else(Vec3::z);
    let x = crate::any_perpendicular(&a);
    let y = a.cross(&x);
    let mut m = grid(nu, nv, true, |s, t| {
        let th = TAU * s;
        let ph = TAU * t;
        let radial = x * th.cos() + y * th.sin();
        center + radial * (big_r + r * ph.cos()) + a * (r * ph.sin())
    });
    m.weld(r * 1e-9);
    m
}

/// A closed axis-aligned box `[min, min + size]`, each face split `n × n`.
#[must_use]
pub fn cuboid(min: Point3, size: Vec3, n: usize) -> Mesh {
    let mut m = Mesh::default();
    let (sx, sy, sz) = (size.x, size.y, size.z);
    let faces: [(Point3, Vec3, Vec3, f64, f64); 6] = [
        (min, Vec3::y(), Vec3::x(), sy, sx),                           // z = min (normal −z)
        (min + Vec3::new(0.0, 0.0, sz), Vec3::x(), Vec3::y(), sx, sy), // z = max
        (min, Vec3::x(), Vec3::z(), sx, sz),                           // y = min (normal −y)
        (min + Vec3::new(0.0, sy, 0.0), Vec3::z(), Vec3::x(), sz, sx), // y = max
        (min, Vec3::z(), Vec3::y(), sz, sy),                           // x = min (normal −x)
        (min + Vec3::new(sx, 0.0, 0.0), Vec3::y(), Vec3::z(), sy, sz), // x = max
    ];
    for (o, u, v, w, h) in faces {
        m.append(&plane(o, u, v, w, h, n, n));
    }
    m.weld(1e-9);
    m
}

/// Displaces every vertex along its normal by `N(0, sigma²)`.
pub fn add_noise(mesh: &mut Mesh, sigma: f64, rng: &mut Rng) {
    let normals = mesh.vertex_normals();
    for (p, n) in mesh.positions.iter_mut().zip(normals) {
        *p += n * (sigma * rng.gauss());
    }
}

/// Moves a fraction of the vertices by up to `magnitude` in a random direction.
pub fn add_outliers(mesh: &mut Mesh, fraction: f64, magnitude: f64, rng: &mut Rng) {
    let n = mesh.positions.len();
    let count = ((n as f64) * fraction.clamp(0.0, 1.0)) as usize;
    for _ in 0..count {
        let i = rng.index(n);
        let d = rng.unit_vector() * rng.range(0.5, 1.0) * magnitude;
        if let Some(p) = mesh.positions.get_mut(i) {
            *p += d;
        }
    }
}

/// The freeform test surface `z = a·sin(x/12)·cos(y/9)` over `[0, w] × [0, h]`.
#[must_use]
pub fn wave_height(a: f64, x: f64, y: f64) -> f64 {
    a * (x / 12.0).sin() * (y / 9.0).cos()
}

/// A freeform patch (height field [`wave_height`]) of `w × h`, amplitude `a`, offset to `origin`.
#[must_use]
pub fn wave(origin: Point3, w: f64, h: f64, a: f64, nu: usize, nv: usize) -> Mesh {
    grid(nu, nv, false, |s, t| {
        let (x, y) = (s * w, t * h);
        origin + Vec3::new(x, y, wave_height(a, x, y))
    })
}

/// A test part for alignment: a 60 × 40 × 20 block with a cylindrical boss on top, so no
/// symmetry makes the registration ambiguous.
#[must_use]
pub fn bracket() -> Mesh {
    let mut m = cuboid(Point3::new(0.0, 0.0, 0.0), Vec3::new(60.0, 40.0, 20.0), 12);
    m.append(&cylinder(Point3::new(45.0, 12.0, 20.0), Vec3::z(), 6.0, 10.0, 48, 6));
    m.append(&sphere(Point3::new(15.0, 28.0, 20.0), 5.0, 32, 16));
    m
}

/// Scan defects applied by [`scanify`] (P1 corpus level 1: CAD meshes made to look scanned).
#[derive(Clone, Debug, PartialEq)]
pub struct ScanDefects {
    /// Target edge length at the low-x end of the part (mm).
    pub spacing: f64,
    /// Edge length grows linearly to `spacing × (1 + density_ramp)` at the high-x end
    /// (non-uniform density, as from a scan taken from one side).
    pub density_ramp: f64,
    /// Gaussian noise σ along the normal (mm).
    pub noise: f64,
    /// Number of holes (missing data) punched as balls of `hole_radius`.
    pub holes: usize,
    /// Radius of the holes (mm).
    pub hole_radius: f64,
    /// Fraction of vertices turned into spikes along the normal.
    pub spikes: f64,
    /// Number of small floating debris patches.
    pub debris: usize,
}

impl Default for ScanDefects {
    fn default() -> Self {
        Self { spacing: 0.8, density_ramp: 1.0, noise: 0.03, holes: 3, hole_radius: 1.5, spikes: 0.0005, debris: 4 }
    }
}

/// Splits edges longer than `h(midpoint)` until none is (conforming: shared edges split once).
#[must_use]
pub fn refine(mesh: &Mesh, h: impl Fn(&Point3) -> f64) -> Mesh {
    let mut m = mesh.clone();
    if m.validate().is_err() {
        return m;
    }
    for _pass in 0..16 {
        let mut mid: std::collections::HashMap<(u32, u32), u32> = std::collections::HashMap::new();
        let mut any = false;
        let tris = std::mem::take(&mut m.triangles);
        let mut out = Vec::with_capacity(tris.len() * 2);
        for t in tris {
            let mut e = [u32::MAX; 3];
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let (pa, pb) = (m.positions[a as usize], m.positions[b as usize]);
                let c = Point3::from((pa.coords + pb.coords) / 2.0);
                let lim = h(&c);
                if lim > 0.0 && (pb - pa).norm() > lim {
                    let key = (a.min(b), a.max(b));
                    let id = *mid.entry(key).or_insert_with(|| {
                        m.positions.push(c);
                        (m.positions.len() - 1) as u32
                    });
                    e[k] = id;
                    any = true;
                }
            }
            let [a, b, c] = t;
            let [ab, bc, ca] = e;
            let s = |x: u32| x != u32::MAX;
            match (s(ab), s(bc), s(ca)) {
                (false, false, false) => out.push(t),
                (true, false, false) => out.extend([[a, ab, c], [ab, b, c]]),
                (false, true, false) => out.extend([[a, b, bc], [a, bc, c]]),
                (false, false, true) => out.extend([[a, b, ca], [ca, b, c]]),
                (true, true, false) => out.extend([[a, ab, c], [ab, bc, c], [ab, b, bc]]),
                (false, true, true) => out.extend([[a, b, ca], [ca, b, bc], [ca, bc, c]]),
                (true, false, true) => out.extend([[a, ab, ca], [ab, b, c], [ab, c, ca]]),
                (true, true, true) => out.extend([[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]]),
            }
        }
        m.triangles = out;
        if !any {
            break;
        }
        if m.triangles.len() > 20_000_000 {
            break;
        }
    }
    m
}

/// Makes a clean (e.g. tessellated CAD) mesh look like a raw scan: dense, non-uniform sampling,
/// noise, holes, spikes and debris. The input must be welded.
#[must_use]
pub fn scanify(mesh: &Mesh, d: &ScanDefects, rng: &mut Rng) -> Mesh {
    let b = mesh.bbox();
    if b.is_empty() || !(d.spacing > 0.0) {
        return mesh.clone();
    }
    let (x0, dx) = (b.min.x, (b.max.x - b.min.x).max(1e-9));
    let ramp = d.density_ramp.max(0.0);
    let mut m = refine(mesh, |p| d.spacing * (1.0 + ramp * ((p.x - x0) / dx).clamp(0.0, 1.0)));
    // holes: drop triangles whose centroid falls in a ball around a random surface point
    let centres: Vec<Point3> = m.sample_points(d.holes, rng).into_iter().map(|(p, _)| p).collect();
    if !centres.is_empty() {
        let r2 = d.hole_radius * d.hole_radius;
        let keep: Vec<u32> = (0..m.triangle_count())
            .filter(|&t| m.face_centroid(t).is_some_and(|c| centres.iter().all(|h| (c - h).norm_squared() > r2)))
            .map(|t| t as u32)
            .collect();
        m = m.submesh(&keep);
    }
    if d.noise > 0.0 {
        add_noise(&mut m, d.noise, rng);
    }
    // spikes along the normal
    let normals = m.vertex_normals();
    let n = m.positions.len();
    let count = (n as f64 * d.spikes.clamp(0.0, 1.0)) as usize;
    for _ in 0..count {
        let i = rng.index(n);
        let mag = d.spacing * rng.range(3.0, 8.0) * if rng.uniform() < 0.5 { -1.0 } else { 1.0 };
        if let (Some(p), Some(nn)) = (m.positions.get_mut(i), normals.get(i)) {
            *p += nn * mag;
        }
    }
    // floating debris: tiny patches a little off the surface
    for _ in 0..d.debris {
        let Some((p, nn)) = m.sample_points(1, rng).into_iter().next() else { break };
        let u = crate::any_perpendicular(&nn);
        let v = nn.cross(&u);
        let o = p + nn * (3.0 * d.spacing) - u * d.spacing - v * d.spacing;
        let mut patch = plane(o, u, v, 2.0 * d.spacing, 2.0 * d.spacing, 2, 2);
        patch.weld(1e-9);
        m.append(&patch);
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn refine_is_conforming_and_scanify_adds_defects() {
        let mut b = cuboid(Point3::origin(), Vec3::new(20.0, 10.0, 5.0), 1);
        b.weld(1e-9);
        let r = refine(&b, |_| 1.0);
        // every edge short, still closed (no boundary edges: each directed edge has its reverse)
        let mut dir = std::collections::HashSet::new();
        for t in &r.triangles {
            for k in 0..3 {
                let (a, c) = (t[k], t[(k + 1) % 3]);
                assert!((r.positions[a as usize] - r.positions[c as usize]).norm() <= 1.0 + 1e-12);
                dir.insert((a, c));
            }
        }
        assert!(dir.iter().all(|(a, c)| dir.contains(&(*c, *a))));
        assert!((r.area() - b.area()).abs() < 1e-9);
        let d = ScanDefects { holes: 2, debris: 3, ..ScanDefects::default() };
        let s = scanify(&b, &d, &mut Rng::new(7));
        assert!(s.triangle_count() > r.triangle_count() / 2);
        assert!(s.area() < b.area() + 3.0 * 4.0 * d.spacing * d.spacing);
    }

    #[test]
    fn shapes_are_on_their_surfaces() {
        let s = sphere(Point3::new(1.0, 1.0, 1.0), 3.0, 16, 8);
        assert!(s.positions.iter().all(|p| ((p - Point3::new(1.0, 1.0, 1.0)).norm() - 3.0).abs() < 1e-12));
        assert!(s.validate().is_ok());
        let c = cylinder(Point3::origin(), Vec3::z(), 2.0, 5.0, 16, 4);
        assert!(c.positions.iter().all(|p| (p.x.hypot(p.y) - 2.0).abs() < 1e-12));
        let b = cuboid(Point3::origin(), Vec3::new(1.0, 2.0, 3.0), 2);
        assert!((b.area() - 22.0).abs() < 1e-9);
        // closed and outward: every edge has a neighbour, normals point away from the centre
        assert!(b.triangle_neighbours().iter().flatten().all(|&n| n != u32::MAX));
        let c0 = b.centroid();
        for i in 0..b.triangle_count() {
            let f = b.face_centroid(i).unwrap();
            assert!(b.face_normal(i).unwrap().dot(&(f - c0)) > 0.0);
        }
        let t = torus(Point3::origin(), Vec3::z(), 5.0, 1.0, 24, 12);
        assert!(t.positions.iter().all(|p| ((p.x.hypot(p.y) - 5.0).hypot(p.z) - 1.0).abs() < 1e-12));
    }
}

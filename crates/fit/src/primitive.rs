//! Analytic primitives fitted to mesh points: plane, sphere, cylinder, cone, torus.
//!
//! Each fit starts from a closed-form estimate (PCA, algebraic sphere/circle, normal-based axis,
//! a scan over the torus tube radius) and is refined by Levenberg–Marquardt on the orthogonal
//! (geometric) distance. The results are the exact analytic surfaces the kernel record holds
//! (`Plane`, `Sphere`, `Cylinder`, `Cone`, `Torus`), so a fitted primitive can later become a B-rep
//! face without approximation.
use crate::lm;
use nalgebra::{DVector, Matrix3, Matrix4, Vector3, Vector4};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};
use surfacecraft_geom::{Mat3, Mesh, Point3, Vec3, any_perpendicular, normalize};

/// The kinds of primitive, in order of complexity (number of parameters).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum PrimitiveKind {
    /// 3 parameters.
    Plane,
    /// 4 parameters.
    Sphere,
    /// 5 parameters.
    Cylinder,
    /// 6 parameters.
    Cone,
    /// 7 parameters.
    Torus,
}

impl PrimitiveKind {
    /// Every kind, simplest first.
    pub const ALL: [PrimitiveKind; 5] = [
        PrimitiveKind::Plane,
        PrimitiveKind::Sphere,
        PrimitiveKind::Cylinder,
        PrimitiveKind::Cone,
        PrimitiveKind::Torus,
    ];
}

/// A primitive surface.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Primitive {
    /// Plane through `point` with unit `normal`.
    Plane {
        /// A point on the plane.
        point: [f64; 3],
        /// Unit normal.
        normal: [f64; 3],
    },
    /// Sphere.
    Sphere {
        /// Centre.
        center: [f64; 3],
        /// Radius.
        radius: f64,
    },
    /// Infinite cylinder: axis through `point` along unit `axis`.
    Cylinder {
        /// A point on the axis.
        point: [f64; 3],
        /// Unit axis direction.
        axis: [f64; 3],
        /// Radius.
        radius: f64,
    },
    /// Cone with apex `apex`, opening along unit `axis` with `half_angle` (radians).
    Cone {
        /// Apex.
        apex: [f64; 3],
        /// Unit axis, pointing into the opening.
        axis: [f64; 3],
        /// Half angle, radians, in (0, π/2).
        half_angle: f64,
    },
    /// Torus around unit `axis` through `center`.
    Torus {
        /// Centre.
        center: [f64; 3],
        /// Unit axis.
        axis: [f64; 3],
        /// Distance from the axis to the tube centre.
        major_radius: f64,
        /// Tube radius.
        minor_radius: f64,
    },
}

fn v(a: &[f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

impl Primitive {
    /// Its kind.
    #[must_use]
    pub fn kind(&self) -> PrimitiveKind {
        match self {
            Primitive::Plane { .. } => PrimitiveKind::Plane,
            Primitive::Sphere { .. } => PrimitiveKind::Sphere,
            Primitive::Cylinder { .. } => PrimitiveKind::Cylinder,
            Primitive::Cone { .. } => PrimitiveKind::Cone,
            Primitive::Torus { .. } => PrimitiveKind::Torus,
        }
    }

    /// Signed orthogonal distance (positive outside / along the normal).
    #[must_use]
    pub fn distance(&self, p: &Point3) -> f64 {
        let p = p.coords;
        match self {
            Primitive::Plane { point, normal } => (p - v(point)).dot(&v(normal)),
            Primitive::Sphere { center, radius } => (p - v(center)).norm() - radius,
            Primitive::Cylinder { point, axis, radius } => {
                let w = p - v(point);
                let d = v(axis);
                (w - d * w.dot(&d)).norm() - radius
            }
            Primitive::Cone { apex, axis, half_angle } => {
                let w = p - v(apex);
                let d = v(axis);
                let h = w.dot(&d);
                let rho = (w - d * h).norm();
                rho * half_angle.cos() - h * half_angle.sin()
            }
            Primitive::Torus { center, axis, major_radius, minor_radius } => {
                let w = p - v(center);
                let d = v(axis);
                let h = w.dot(&d);
                let rho = (w - d * h).norm();
                (rho - major_radius).hypot(h) - minor_radius
            }
        }
    }

    /// Unit outward normal at (or near) `p`.
    #[must_use]
    pub fn normal(&self, p: &Point3) -> Vec3 {
        let q = p.coords;
        let fallback = Vec3::z();
        match self {
            Primitive::Plane { normal, .. } => v(normal),
            Primitive::Sphere { center, .. } => normalize(q - v(center)).unwrap_or(fallback),
            Primitive::Cylinder { point, axis, .. } => {
                let w = q - v(point);
                let d = v(axis);
                normalize(w - d * w.dot(&d)).unwrap_or_else(|| any_perpendicular(&d))
            }
            Primitive::Cone { apex, axis, half_angle } => {
                let w = q - v(apex);
                let d = v(axis);
                let radial = normalize(w - d * w.dot(&d)).unwrap_or_else(|| any_perpendicular(&d));
                radial * half_angle.cos() - d * half_angle.sin()
            }
            Primitive::Torus { center, axis, major_radius, .. } => {
                let w = q - v(center);
                let d = v(axis);
                let radial = normalize(w - d * w.dot(&d)).unwrap_or_else(|| any_perpendicular(&d));
                let core = radial * *major_radius;
                normalize(w - core).unwrap_or(radial)
            }
        }
    }

    /// The closest point on the surface (one Newton projection; exact for every kind but the
    /// cone near its apex).
    #[must_use]
    pub fn project(&self, p: &Point3) -> Point3 {
        p - self.normal(p) * self.distance(p)
    }

    /// Are all numbers finite and sizes positive?
    #[must_use]
    pub fn is_valid(&self) -> bool {
        let f3 = |a: &[f64; 3]| a.iter().all(|x| x.is_finite());
        match self {
            Primitive::Plane { point, normal } => f3(point) && f3(normal),
            Primitive::Sphere { center, radius } => f3(center) && radius.is_finite() && *radius > 0.0,
            Primitive::Cylinder { point, axis, radius } => f3(point) && f3(axis) && radius.is_finite() && *radius > 0.0,
            Primitive::Cone { apex, axis, half_angle } => {
                f3(apex) && f3(axis) && half_angle.is_finite() && *half_angle > 0.0 && *half_angle < PI / 2.0
            }
            Primitive::Torus { center, axis, major_radius, minor_radius } => {
                f3(center) && f3(axis) && major_radius.is_finite() && minor_radius.is_finite() && *minor_radius > 0.0
            }
        }
    }

    /// A frame (origin, x, y, z) of the primitive; z is the normal/axis.
    #[must_use]
    pub fn frame(&self) -> (Vec3, Vec3, Vec3, Vec3) {
        let (o, z) = match self {
            Primitive::Plane { point, normal } => (v(point), v(normal)),
            Primitive::Sphere { center, .. } => (v(center), Vec3::z()),
            Primitive::Cylinder { point, axis, .. } => (v(point), v(axis)),
            Primitive::Cone { apex, axis, .. } => (v(apex), v(axis)),
            Primitive::Torus { center, axis, .. } => (v(center), v(axis)),
        };
        let z = normalize(z).unwrap_or_else(Vec3::z);
        let x = any_perpendicular(&z);
        (o, x, z.cross(&x), z)
    }

    /// Natural parameters `(u, v)` of the projection of `p` (plane: in-plane coordinates;
    /// sphere: longitude, latitude; cylinder/cone: angle, height; torus: angle, tube angle).
    #[must_use]
    pub fn param(&self, p: &Point3) -> (f64, f64) {
        let (o, x, y, z) = self.frame();
        let w = p.coords - o;
        let ang = |a: f64, b: f64| b.atan2(a);
        match self {
            Primitive::Plane { .. } => (w.dot(&x), w.dot(&y)),
            Primitive::Sphere { .. } => {
                let r = w.norm().max(1e-300);
                (ang(w.dot(&x), w.dot(&y)), (w.dot(&z) / r).clamp(-1.0, 1.0).asin())
            }
            Primitive::Cylinder { .. } | Primitive::Cone { .. } => (ang(w.dot(&x), w.dot(&y)), w.dot(&z)),
            Primitive::Torus { major_radius, .. } => {
                let u = ang(w.dot(&x), w.dot(&y));
                let rho = (w - z * w.dot(&z)).norm();
                (u, ang(rho - major_radius, w.dot(&z)))
            }
        }
    }

    /// The point at natural parameters `(u, v)`.
    #[must_use]
    pub fn eval(&self, u: f64, vv: f64) -> Point3 {
        let (o, x, y, z) = self.frame();
        let radial = x * u.cos() + y * u.sin();
        Point3::from(match self {
            Primitive::Plane { .. } => o + x * u + y * vv,
            Primitive::Sphere { radius, .. } => o + (radial * vv.cos() + z * vv.sin()) * *radius,
            Primitive::Cylinder { radius, .. } => o + radial * *radius + z * vv,
            Primitive::Cone { half_angle, .. } => o + radial * (vv * half_angle.tan()) + z * vv,
            Primitive::Torus { major_radius, minor_radius, .. } => {
                o + radial * (major_radius + minor_radius * vv.cos()) + z * (minor_radius * vv.sin())
            }
        })
    }

    /// A display mesh of the part of the surface covered by `points` (parameter bounding box,
    /// full turns for closed directions when the points wrap around).
    #[must_use]
    pub fn patch_mesh(&self, points: &[Point3], n: usize) -> Mesh {
        let n = n.clamp(2, 256);
        if points.is_empty() {
            return Mesh::default();
        }
        let params: Vec<(f64, f64)> = points.iter().map(|p| self.param(p)).collect();
        let periodic_u = !matches!(self, Primitive::Plane { .. });
        let periodic_v = matches!(self, Primitive::Torus { .. });
        let (u0, u1) = if periodic_u {
            angular_range(params.iter().map(|p| p.0))
        } else {
            linear_range(params.iter().map(|p| p.0))
        };
        let (v0, v1) = if periodic_v {
            angular_range(params.iter().map(|p| p.1))
        } else {
            linear_range(params.iter().map(|p| p.1))
        };
        let mut positions = Vec::with_capacity((n + 1) * (n + 1));
        for j in 0..=n {
            for i in 0..=n {
                let u = u0 + (u1 - u0) * i as f64 / n as f64;
                let vv = v0 + (v1 - v0) * j as f64 / n as f64;
                positions.push(self.eval(u, vv));
            }
        }
        let mut triangles = Vec::with_capacity(n * n * 2);
        let w = (n + 1) as u32;
        for j in 0..n as u32 {
            for i in 0..n as u32 {
                let a = j * w + i;
                triangles.push([a, a + 1, a + w + 1]);
                triangles.push([a, a + w + 1, a + w]);
            }
        }
        Mesh { positions, triangles }
    }
}

fn linear_range(it: impl Iterator<Item = f64>) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for x in it.filter(|x| x.is_finite()) {
        lo = lo.min(x);
        hi = hi.max(x);
    }
    if lo > hi { (0.0, 1.0) } else { (lo, hi) }
}

/// Smallest arc covering the angles (full turn when they wrap around).
fn angular_range(it: impl Iterator<Item = f64>) -> (f64, f64) {
    let mut a: Vec<f64> = it.filter(|x| x.is_finite()).map(|x| x.rem_euclid(TAU)).collect();
    if a.len() < 2 {
        return (0.0, TAU);
    }
    a.sort_by(f64::total_cmp);
    let mut gap = 0.0;
    let mut start = 0.0;
    for w in a.windows(2) {
        if let [x, y] = w
            && y - x > gap
        {
            gap = y - x;
            start = *y;
        }
    }
    let (first, last) = (a.first().copied().unwrap_or(0.0), a.last().copied().unwrap_or(TAU));
    let wrap = first + TAU - last;
    if wrap >= gap {
        return if wrap > 0.3 { (first, last) } else { (0.0, TAU) };
    }
    if gap < 0.3 {
        return (0.0, TAU);
    }
    (start, start + TAU - gap)
}

/// A primitive with its fit statistics.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct PrimitiveFit {
    /// The surface.
    pub primitive: Primitive,
    /// RMS orthogonal distance (mm).
    pub rms: f64,
    /// Largest |distance| (mm).
    pub max: f64,
    /// Points used.
    pub count: usize,
}

fn stats(prim: Primitive, pts: &[Point3]) -> Option<PrimitiveFit> {
    if !prim.is_valid() || pts.is_empty() {
        return None;
    }
    let mut sq = 0.0;
    let mut max: f64 = 0.0;
    for p in pts {
        let d = prim.distance(p);
        sq += d * d;
        max = max.max(d.abs());
    }
    let rms = (sq / pts.len() as f64).sqrt();
    rms.is_finite().then_some(PrimitiveFit { primitive: prim, rms, max, count: pts.len() })
}

/// Basis perpendicular to `d`.
fn basis(d: &Vec3) -> (Vec3, Vec3) {
    let u = any_perpendicular(d);
    (u, d.cross(&u))
}

fn axis_from(d0: &Vec3, u: &Vec3, w: &Vec3, a: f64, b: f64) -> Vec3 {
    normalize(d0 + u * a + w * b).unwrap_or(*d0)
}

/// Eigenvector of the smallest eigenvalue of a symmetric 3×3 matrix.
fn min_eigvec(m: &Mat3) -> Vec3 {
    let e = m.symmetric_eigen();
    let mut k = 0;
    for i in 1..3 {
        if e.eigenvalues[i] < e.eigenvalues[k] {
            k = i;
        }
    }
    normalize(e.eigenvectors.column(k).into_owned()).unwrap_or_else(Vec3::z)
}

fn centroid(pts: &[Point3]) -> Vec3 {
    pts.iter().map(|p| p.coords).sum::<Vec3>() / pts.len().max(1) as f64
}

/// Algebraic circle in 2-D (Kåsa): centre and radius.
fn circle_2d(xy: &[(f64, f64)]) -> Option<(f64, f64, f64)> {
    let mut a = Matrix3::zeros();
    let mut b = Vector3::zeros();
    for &(x, y) in xy {
        let row = Vector3::new(x, y, 1.0);
        a += row * row.transpose();
        b += row * -(x * x + y * y);
    }
    let s = a.lu().solve(&b)?;
    let (cx, cy) = (-s[0] / 2.0, -s[1] / 2.0);
    let r2 = cx * cx + cy * cy - s[2];
    (r2 > 0.0 && r2.is_finite()).then(|| (cx, cy, r2.sqrt()))
}

const MAX_LM_POINTS: usize = 4000;

/// An evenly strided subset for the iterative refinement (statistics use all points).
fn subset(pts: &[Point3]) -> Vec<Point3> {
    let stride = pts.len().div_ceil(MAX_LM_POINTS).max(1);
    pts.iter().step_by(stride).copied().collect()
}

/// Fits a plane.
#[must_use]
pub fn fit_plane(pts: &[Point3]) -> Option<PrimitiveFit> {
    let f = crate::plane::fit_plane(pts, None)?;
    stats(Primitive::Plane { point: f.point.into(), normal: f.normal.into() }, pts)
}

/// Fits a sphere.
#[must_use]
pub fn fit_sphere(pts: &[Point3]) -> Option<PrimitiveFit> {
    if pts.len() < 4 {
        return None;
    }
    let c0 = centroid(pts);
    let mut a = Matrix4::zeros();
    let mut b = Vector4::zeros();
    for p in pts {
        let q = p.coords - c0;
        let row = Vector4::new(q.x, q.y, q.z, 1.0);
        a += row * row.transpose();
        b += row * -q.norm_squared();
    }
    let s = a.lu().solve(&b)?;
    let c = Vec3::new(-s[0] / 2.0, -s[1] / 2.0, -s[2] / 2.0);
    let r2 = c.norm_squared() - s[3];
    if !(r2 > 0.0) {
        return None;
    }
    let sub = subset(pts);
    let p0 = DVector::from_vec(vec![c.x, c.y, c.z, r2.sqrt()]);
    let res = lm::minimize(p0, sub.len(), 100, |p, out| {
        let c = Vec3::new(p[0], p[1], p[2]);
        for (o, q) in out.iter_mut().zip(&sub) {
            *o = (q.coords - c0 - c).norm() - p[3];
        }
    })?;
    let p = res.params;
    stats(Primitive::Sphere { center: (c0 + Vec3::new(p[0], p[1], p[2])).into(), radius: p[3].abs() }, pts)
}

/// Cylinder from a given axis estimate: circle in the perpendicular plane, then LM.
fn cylinder_from_axis(pts: &[Point3], d0: Vec3) -> Option<PrimitiveFit> {
    let c0 = centroid(pts);
    let (u, w) = basis(&d0);
    let xy: Vec<(f64, f64)> = pts.iter().map(|p| ((p.coords - c0).dot(&u), (p.coords - c0).dot(&w))).collect();
    let (cx, cy, r) = circle_2d(&xy)?;
    let base = c0 + u * cx + w * cy;
    let sub = subset(pts);
    let res = lm::minimize(DVector::from_vec(vec![0.0, 0.0, 0.0, 0.0, r]), sub.len(), 100, |p, out| {
        let d = axis_from(&d0, &u, &w, p[0], p[1]);
        let c = base + u * p[2] + w * p[3];
        for (o, q) in out.iter_mut().zip(&sub) {
            let x = q.coords - c;
            *o = (x - d * x.dot(&d)).norm() - p[4];
        }
    })?;
    let p = res.params;
    let d = axis_from(&d0, &u, &w, p[0], p[1]);
    let c = base + u * p[2] + w * p[3];
    // put the axis point at the foot of the centroid
    let c = c + d * (c0 - c).dot(&d);
    stats(Primitive::Cylinder { point: c.into(), axis: d.into(), radius: p[4].abs() }, pts)
}

/// Axis candidates from normals: perpendicular to all normals (cylinder) or at a constant
/// angle to them (cone, centred covariance).
fn normal_axes(normals: &[Vec3]) -> Vec<Vec3> {
    let ns: Vec<&Vec3> = normals.iter().filter(|n| surfacecraft_geom::finite_vec(n) && n.norm() > 0.5).collect();
    if ns.len() < 3 {
        return Vec::new();
    }
    let mut m = Mat3::zeros();
    let mean: Vec3 = ns.iter().copied().sum::<Vec3>() / ns.len() as f64;
    let mut mc = Mat3::zeros();
    for n in &ns {
        m += *n * n.transpose();
        let d = *n - mean;
        mc += d * d.transpose();
    }
    vec![min_eigvec(&m), min_eigvec(&mc)]
}

/// Fits a cylinder (normals give the axis estimate; PCA directions are tried as well).
#[must_use]
pub fn fit_cylinder(pts: &[Point3], normals: &[Vec3]) -> Option<PrimitiveFit> {
    if pts.len() < 6 {
        return None;
    }
    let mut cands = normal_axes(normals);
    cands.extend(pca_axes(pts));
    cands.into_iter().filter_map(|d| cylinder_from_axis(pts, d)).min_by(|a, b| a.rms.total_cmp(&b.rms))
}

fn pca_axes(pts: &[Point3]) -> Vec<Vec3> {
    let c = centroid(pts);
    let mut m = Mat3::zeros();
    for p in pts {
        let d = p.coords - c;
        m += d * d.transpose();
    }
    let e = m.symmetric_eigen();
    (0..3).filter_map(|i| normalize(e.eigenvectors.column(i).into_owned())).collect()
}

fn cone_from_axis(pts: &[Point3], d0: Vec3) -> Option<PrimitiveFit> {
    // radius as a linear function of height along the axis: r(h) = r0 + h·tan(α)
    let c0 = centroid(pts);
    let (u, w) = basis(&d0);
    let xy: Vec<(f64, f64)> = pts.iter().map(|p| ((p.coords - c0).dot(&u), (p.coords - c0).dot(&w))).collect();
    let (cx, cy, _) = circle_2d(&xy)?;
    let axis_pt = c0 + u * cx + w * cy;
    let (mut s_h, mut s_r, mut s_hh, mut s_hr) = (0.0, 0.0, 0.0, 0.0);
    for p in pts {
        let x = p.coords - axis_pt;
        let h = x.dot(&d0);
        let r = (x - d0 * h).norm();
        s_h += h;
        s_r += r;
        s_hh += h * h;
        s_hr += h * r;
    }
    let n = pts.len() as f64;
    let den = n * s_hh - s_h * s_h;
    if den.abs() < 1e-300 {
        return None;
    }
    let slope = (n * s_hr - s_h * s_r) / den;
    let r0 = (s_r - slope * s_h) / n;
    if slope.abs() < 1e-9 {
        return None;
    }
    let (d, slope) = if slope > 0.0 { (d0, slope) } else { (-d0, -slope) };
    let apex0 = axis_pt - d * (r0 / slope);
    let alpha0 = slope.atan();
    let (u, w) = basis(&d);
    let sub = subset(pts);
    let res = lm::minimize(
        DVector::from_vec(vec![0.0, 0.0, apex0.x, apex0.y, apex0.z, alpha0]),
        sub.len(),
        150,
        |p, out| {
            let dd = axis_from(&d, &u, &w, p[0], p[1]);
            let apex = Vec3::new(p[2], p[3], p[4]);
            let (sa, ca) = p[5].sin_cos();
            for (o, q) in out.iter_mut().zip(&sub) {
                let x = q.coords - apex;
                let h = x.dot(&dd);
                *o = (x - dd * h).norm() * ca - h * sa;
            }
        },
    )?;
    let p = res.params;
    let mut dd = axis_from(&d, &u, &w, p[0], p[1]);
    let mut alpha = p[5];
    if alpha < 0.0 {
        alpha = -alpha;
        dd = -dd;
    }
    stats(Primitive::Cone { apex: [p[2], p[3], p[4]], axis: dd.into(), half_angle: alpha }, pts)
}

/// Fits a cone.
#[must_use]
pub fn fit_cone(pts: &[Point3], normals: &[Vec3]) -> Option<PrimitiveFit> {
    if pts.len() < 8 {
        return None;
    }
    let mut cands = normal_axes(normals);
    cands.extend(pca_axes(pts));
    cands.into_iter().filter_map(|d| cone_from_axis(pts, d)).min_by(|a, b| a.rms.total_cmp(&b.rms))
}

/// Fits a torus (needs outward normals: tube centres are `p − r·n`).
#[must_use]
pub fn fit_torus(pts: &[Point3], normals: &[Vec3]) -> Option<PrimitiveFit> {
    if pts.len() < 10 || normals.len() != pts.len() {
        return None;
    }
    let sub = subset(pts);
    let nsub: Vec<Vec3> = {
        let stride = pts.len().div_ceil(MAX_LM_POINTS).max(1);
        normals.iter().step_by(stride).copied().collect()
    };
    let ext = surfacecraft_geom::Aabb::from_points(&sub).diagonal().max(1e-9);
    // tube centres for a trial tube radius s lie on a planar circle: score s by that
    // returns (cost, circle centre, axis, major radius, unit direction from the centre to the
    // middle of the arc the tube centres cover)
    let score = |s: f64| -> Option<(f64, Vec3, Vec3, f64, Vec3)> {
        let cs: Vec<Point3> = sub.iter().zip(&nsub).map(|(p, n)| p - n * s).collect();
        let pl = crate::plane::fit_plane(&cs, None)?;
        let (u, w) = basis(&pl.normal);
        let o = pl.point.coords;
        let xy: Vec<(f64, f64)> = cs.iter().map(|c| ((c.coords - o).dot(&u), (c.coords - o).dot(&w))).collect();
        let (cx, cy, r) = circle_2d(&xy)?;
        let mut sq = 0.0;
        for &(x, y) in &xy {
            sq += ((x - cx).hypot(y - cy) - r).powi(2);
        }
        let cost = pl.rms * pl.rms + sq / xy.len() as f64;
        let n = xy.len() as f64;
        let (mx, my) = (xy.iter().map(|q| q.0).sum::<f64>() / n - cx, xy.iter().map(|q| q.1).sum::<f64>() / n - cy);
        let dir = (u * mx + w * my).try_normalize(1e-12).unwrap_or_else(Vec3::zeros);
        Some((cost, o + u * cx + w * cy, pl.normal, r, dir))
    };
    // On a partial torus patch the tube centres cover a short arc, whose circle is ill-determined:
    // (tube radius, major radius) lie along a nearly flat valley of the planar-circle cost, and a
    // start from the best circle can end in a poor local minimum of the final fit (a spindle-like
    // torus) while the true torus is several times larger. So each trial tube radius is also tried
    // with larger circles through the same arc, every trial is scored by its true torus residual,
    // and the few best distinct trials start the fit.
    let steps = 80;
    // (true torus residual, tube radius, circle centre, axis, major radius)
    let mut best_per_factor: Vec<Option<(f64, f64, Vec3, Vec3, f64)>> = vec![None; TORUS_FACTORS.len()];
    for k in 1..=steps {
        // geometric from 1 % to 400 % of the extent: the tube of a partial torus patch can be
        // larger than the patch, so a linear scan up to half the extent never reaches its radius
        let s = ext * 0.01 * 400f64.powf(k as f64 / steps as f64);
        let Some((_, c0, a, big0, dir)) = score(s) else { continue };
        let arc_mid = c0 + dir * big0;
        for (fi, f) in TORUS_FACTORS.iter().enumerate() {
            let big = big0 * f;
            let c = arc_mid - dir * big;
            let geo = sub
                .iter()
                .map(|q| {
                    let x = q.coords - c;
                    let h = x.dot(&a);
                    let rho = (x - a * h).norm();
                    ((rho - big).hypot(h) - s).powi(2)
                })
                .sum::<f64>()
                / sub.len() as f64;
            if let Some(slot) = best_per_factor.get_mut(fi)
                && geo.is_finite()
                && slot.as_ref().is_none_or(|b| geo < b.0)
            {
                *slot = Some((geo, s, c, a, big));
            }
        }
    }
    let starts: Vec<(f64, f64, Vec3, Vec3, f64)> = best_per_factor.into_iter().flatten().collect();
    let mut best_fit: Option<(f64, Primitive)> = None;
    for &(_, s, c, a, big) in &starts {
        let (u, w) = basis(&a);
        // A patch of a torus with a large tube is nearly as well described by a spindle torus (tube
        // reaching past the axis, minor radius above the major one) or by a negative major radius,
        // and the fit drifts along that flat valley to a surface the auto fit discards. A penalty
        // residual keeps the tube inside the ring: minor <= 0.95 major.
        let n_pen = (sub.len() as f64).sqrt() * 10.0;
        let Some(res) = lm::minimize(
            DVector::from_vec(vec![0.0, 0.0, c.x, c.y, c.z, big.max(s / 0.9), s]),
            sub.len() + 1,
            150,
            |p, out| {
                let d = axis_from(&a, &u, &w, p[0], p[1]);
                let cc = Vec3::new(p[2], p[3], p[4]);
                for (o, q) in out.iter_mut().zip(&sub) {
                    let x = q.coords - cc;
                    let h = x.dot(&d);
                    let rho = (x - d * h).norm();
                    *o = (rho - p[5]).hypot(h) - p[6];
                }
                if let Some(o) = out.get_mut(sub.len()) {
                    *o = n_pen * (p[6].abs() - 0.95 * p[5]).max(0.0);
                }
            },
        ) else {
            continue;
        };
        let p = res.params;
        let d = axis_from(&a, &u, &w, p[0], p[1]);
        let prim = Primitive::Torus {
            center: [p[2], p[3], p[4]],
            axis: d.into(),
            major_radius: p[5],
            minor_radius: p[6].abs(),
        };
        if prim.is_valid() && res.cost.is_finite() && best_fit.as_ref().is_none_or(|b| res.cost < b.0) {
            best_fit = Some((res.cost, prim));
        }
        // the first start usually converges to the right torus: the others only run when the fit
        // is poor relative to the size of the patch
        let good = (TORUS_GOOD_RMS * ext).powi(2) * sub.len() as f64;
        if best_fit.as_ref().is_some_and(|b| b.0 <= good) {
            break;
        }
    }
    stats(best_fit?.1, pts)
}

/// RMS (relative to the patch extent) below which [`fit_torus`] stops trying further starts.
const TORUS_GOOD_RMS: f64 = 0.003;

/// Factors by which [`fit_torus`] enlarges the circle of tube centres of each trial tube radius;
/// the best trial of each factor starts one fit.
const TORUS_FACTORS: [f64; 4] = [1.0, 2.5, 6.0, 15.0];

/// Fits a primitive of the given kind.
#[must_use]
pub fn fit(kind: PrimitiveKind, pts: &[Point3], normals: &[Vec3]) -> Option<PrimitiveFit> {
    let pts_ok: Vec<Point3> = pts.iter().copied().filter(surfacecraft_geom::finite).collect();
    if pts_ok.len() != pts.len() {
        return None;
    }
    match kind {
        PrimitiveKind::Plane => fit_plane(pts),
        PrimitiveKind::Sphere => fit_sphere(pts),
        PrimitiveKind::Cylinder => fit_cylinder(pts, normals),
        PrimitiveKind::Cone => fit_cone(pts, normals),
        PrimitiveKind::Torus => fit_torus(pts, normals),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Rng, synth};

    pub(crate) fn sample(m: &Mesh) -> (Vec<Point3>, Vec<Vec3>) {
        (m.positions.clone(), m.vertex_normals())
    }

    fn noisy(mut m: Mesh, sigma: f64) -> Mesh {
        synth::add_noise(&mut m, sigma, &mut Rng::new(11));
        m
    }

    #[test]
    fn exact_fits_recover_parameters() {
        let s = synth::sphere(Point3::new(1.0, 2.0, 3.0), 7.0, 32, 16);
        let (p, n) = sample(&s);
        let f = fit(PrimitiveKind::Sphere, &p, &n).unwrap();
        assert!(f.rms < 1e-9, "{f:?}");

        let axis = Vec3::new(1.0, 1.0, 2.0).normalize();
        let c = synth::cylinder(Point3::new(5.0, 0.0, -3.0), axis, 4.0, 20.0, 48, 10);
        let (p, n) = sample(&c);
        let f = fit(PrimitiveKind::Cylinder, &p, &n).unwrap();
        assert!(f.rms < 1e-8, "{f:?}");
        let Primitive::Cylinder { axis: a, radius, .. } = f.primitive else { panic!() };
        assert!((radius - 4.0).abs() < 1e-8 && v(&a).cross(&axis).norm() < 1e-8);

        let k = synth::cone(Point3::new(0.0, 0.0, 0.0), axis, 10.0, 4.0, 15.0, 48, 10);
        let (p, n) = sample(&k);
        let f = fit(PrimitiveKind::Cone, &p, &n).unwrap();
        assert!(f.rms < 1e-7, "{f:?}");
        let Primitive::Cone { half_angle, .. } = f.primitive else { panic!() };
        assert!((half_angle - (6.0f64 / 15.0).atan()).abs() < 1e-7);

        let t = synth::torus(Point3::new(1.0, -1.0, 2.0), axis, 12.0, 3.0, 48, 24);
        let (p, n) = sample(&t);
        let f = fit(PrimitiveKind::Torus, &p, &n).unwrap();
        assert!(f.rms < 1e-7, "{f:?}");
        let Primitive::Torus { major_radius, minor_radius, .. } = f.primitive else { panic!() };
        assert!((major_radius - 12.0).abs() < 1e-6 && (minor_radius - 3.0).abs() < 1e-6);
    }

    /// Small patches of a torus with a large tube (a yoke-like body): the tube radius exceeds half
    /// the patch extent and the circle of tube centres is only a short arc, so the fit must not
    /// drift to a spindle torus or a negative major radius.
    #[test]
    fn torus_patches_with_large_tubes_are_fitted() {
        let axis = Vec3::new(0.3, -0.2, 1.0).normalize();
        let center = Point3::new(1.0, -2.0, 3.0);
        let x = surfacecraft_geom::any_perpendicular(&axis);
        let y = axis.cross(&x);
        // narrow in the major angle (±8°), wide in the tube angle (±60°) around the outer equator
        for (k, (big_r, r)) in [(34.0, 20.0), (40.0, 21.0), (46.0, 25.0)].into_iter().enumerate() {
            let full = synth::torus(center, axis, big_r, r, 180, 120);
            let normals = full.vertex_normals();
            let mut rng = Rng::new(3 + k as u64);
            let (mut p, mut n) = (Vec::new(), Vec::new());
            for (q, nq) in full.positions.iter().zip(&normals) {
                let d = q - center;
                let (h, th) = (d.dot(&axis), d.dot(&y).atan2(d.dot(&x)));
                let ph = h.atan2(d.dot(&x).hypot(d.dot(&y)) - big_r);
                if th.to_degrees().abs() <= 8.0 && ph.to_degrees().abs() <= 60.0 {
                    // noise of the scan (σ = 0.05 mm) and of its vertex normals
                    p.push(q + Vec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * 0.05);
                    n.push((nq + Vec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * 0.05).normalize());
                }
            }
            let f = fit(PrimitiveKind::Torus, &p, &n).expect("a torus fit");
            let Primitive::Torus { major_radius, minor_radius, .. } = f.primitive else { panic!() };
            assert!(f.rms <= 0.1, "torus {k}: rms {} major {major_radius} minor {minor_radius}", f.rms);
            assert!(minor_radius < major_radius, "torus {k}: spindle torus {major_radius} {minor_radius}");
        }
    }

    /// Gate M2: RMS ≤ 0.1 mm on noisy synthetic data (σ = 0.05 mm) for every kind.
    #[test]
    fn gate_m2_noisy_fits_within_rms() {
        let axis = Vec3::new(0.3, -0.2, 1.0).normalize();
        let cases: Vec<(PrimitiveKind, Mesh)> = vec![
            (PrimitiveKind::Plane, synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 40.0, 30.0, 40, 30)),
            (PrimitiveKind::Sphere, synth::sphere(Point3::new(3.0, 4.0, 5.0), 15.0, 64, 32)),
            (PrimitiveKind::Cylinder, synth::cylinder(Point3::origin(), axis, 8.0, 30.0, 64, 20)),
            (PrimitiveKind::Cone, synth::cone(Point3::origin(), axis, 12.0, 5.0, 20.0, 64, 20)),
            (PrimitiveKind::Torus, synth::torus(Point3::origin(), axis, 20.0, 5.0, 96, 32)),
        ];
        for (kind, mesh) in cases {
            let m = noisy(mesh, 0.05);
            let (p, n) = sample(&m);
            let f = fit(kind, &p, &n).unwrap_or_else(|| panic!("{kind:?} failed"));
            eprintln!("gate_m2 {kind:?}: rms {:.4} mm max {:.4} mm", f.rms, f.max);
            assert!(f.rms <= 0.1, "{kind:?} rms {}", f.rms);
        }
    }

    #[test]
    fn patch_mesh_covers_points() {
        let c = synth::cylinder(Point3::origin(), Vec3::z(), 5.0, 10.0, 32, 4);
        let (p, n) = sample(&c);
        let f = fit(PrimitiveKind::Cylinder, &p, &n).unwrap();
        let m = f.primitive.patch_mesh(&p, 16);
        let b = m.bbox();
        assert!((b.size().z - 10.0).abs() < 1e-6 && (b.size().x - 10.0).abs() < 0.1);
        assert!(angular_range([0.1, 0.2, 6.2].into_iter()).1 - angular_range([0.1, 0.2, 6.2].into_iter()).0 < 1.0);
    }

    #[test]
    fn bad_input_gives_none() {
        assert!(fit(PrimitiveKind::Sphere, &[], &[]).is_none());
        let line: Vec<Point3> = (0..20).map(|i| Point3::new(f64::from(i), 0.0, 0.0)).collect();
        assert!(fit(PrimitiveKind::Plane, &line, &[]).is_none());
        assert!(fit(PrimitiveKind::Sphere, &[Point3::new(f64::NAN, 0.0, 0.0); 10], &[]).is_none());
    }
}

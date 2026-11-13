//! Quadric fits: a plane, a sphere, a cylinder and a cone from points and their normals.

use super::estimate::*;
use super::model::*;
use super::*;

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

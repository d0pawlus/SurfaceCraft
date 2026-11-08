//! Triangle primitives: normal, area, closest point, ray hit.
use crate::{Point3, Vec3};

/// Unnormalised normal `(b − a) × (c − a)` (length = 2·area).
#[must_use]
pub fn cross(a: &Point3, b: &Point3, c: &Point3) -> Vec3 {
    (b - a).cross(&(c - a))
}

/// Area of the triangle.
#[must_use]
pub fn area(a: &Point3, b: &Point3, c: &Point3) -> f64 {
    0.5 * cross(a, b, c).norm()
}

/// Unit normal or `None` for a degenerate triangle.
#[must_use]
pub fn normal(a: &Point3, b: &Point3, c: &Point3) -> Option<Vec3> {
    crate::normalize(cross(a, b, c))
}

/// Closest point on triangle `abc` to `p` (Ericson, Real-Time Collision Detection §5.1.5).
#[must_use]
pub fn closest_point(p: &Point3, a: &Point3, b: &Point3, c: &Point3) -> Point3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return *a;
    }
    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return *b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let den = d1 - d3;
        let v = if den.abs() > 0.0 { d1 / den } else { 0.0 };
        return a + ab * v;
    }
    let cp = p - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return *c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let den = d2 - d6;
        let w = if den.abs() > 0.0 { d2 / den } else { 0.0 };
        return a + ac * w;
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let den = (d4 - d3) + (d5 - d6);
        let w = if den.abs() > 0.0 { (d4 - d3) / den } else { 0.0 };
        return b + (c - b) * w;
    }
    let sum = va + vb + vc;
    if sum.abs() <= 0.0 {
        return *a;
    }
    let denom = 1.0 / sum;
    let v = vb * denom;
    let w = vc * denom;
    a + ab * v + ac * w
}

/// Ray/triangle intersection (Möller–Trumbore): the ray parameter `t ≥ 0` of the hit.
#[must_use]
pub fn ray_hit(origin: &Point3, dir: &Vec3, a: &Point3, b: &Point3, c: &Point3) -> Option<f64> {
    let e1 = b - a;
    let e2 = c - a;
    let pv = dir.cross(&e2);
    let det = e1.dot(&pv);
    if det.abs() < 1e-14 {
        return None;
    }
    let inv = 1.0 / det;
    let tv = origin - a;
    let u = tv.dot(&pv) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let qv = tv.cross(&e1);
    let v = dir.dot(&qv) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = e2.dot(&qv) * inv;
    (t >= 0.0 && t.is_finite()).then_some(t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closest_point_regions() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(1.0, 0.0, 0.0);
        let c = Point3::new(0.0, 1.0, 0.0);
        assert!((closest_point(&Point3::new(0.2, 0.2, 5.0), &a, &b, &c) - Point3::new(0.2, 0.2, 0.0)).norm() < 1e-15);
        assert_eq!(closest_point(&Point3::new(-1.0, -1.0, 0.0), &a, &b, &c), a);
        assert_eq!(closest_point(&Point3::new(2.0, -1.0, 0.0), &a, &b, &c), b);
        let q = closest_point(&Point3::new(1.0, 1.0, 0.0), &a, &b, &c);
        assert!((q - Point3::new(0.5, 0.5, 0.0)).norm() < 1e-12);
        assert!((area(&a, &b, &c) - 0.5).abs() < 1e-15);
        let t = ray_hit(&Point3::new(0.2, 0.2, 3.0), &Vec3::new(0.0, 0.0, -1.0), &a, &b, &c).unwrap();
        assert!((t - 3.0).abs() < 1e-12);
        assert!(ray_hit(&Point3::new(2.0, 2.0, 3.0), &Vec3::new(0.0, 0.0, -1.0), &a, &b, &c).is_none());
    }
}

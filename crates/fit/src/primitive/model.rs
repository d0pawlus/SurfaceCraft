//! The primitive kinds and the analytic surfaces: their definition, evaluation and distances.

use super::estimate::*;
use super::*;

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

pub(super) fn v(a: &[f64; 3]) -> Vec3 {
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

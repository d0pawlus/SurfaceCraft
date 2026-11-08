//! Rigid transforms (rotation + translation), the currency of alignment.
use crate::{Mat3, Mat4, Point3, Vec3};
use nalgebra::{Rotation3, UnitQuaternion};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// A rigid motion `p ↦ R·p + t` with `R` a proper rotation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    /// Rotation part.
    pub rotation: Rotation3<f64>,
    /// Translation part.
    pub translation: Vec3,
}

impl Default for Transform {
    fn default() -> Self {
        Self::identity()
    }
}

impl Transform {
    /// The identity.
    #[must_use]
    pub fn identity() -> Self {
        Self { rotation: Rotation3::identity(), translation: Vec3::zeros() }
    }

    /// A pure translation.
    #[must_use]
    pub fn translation(t: Vec3) -> Self {
        Self { rotation: Rotation3::identity(), translation: t }
    }

    /// A rotation of `angle` radians about the unit `axis` through the origin.
    #[must_use]
    pub fn rotation_axis_angle(axis: &Vec3, angle: f64) -> Self {
        let r = crate::normalize(*axis)
            .map(|a| Rotation3::from_axis_angle(&nalgebra::Unit::new_unchecked(a), angle))
            .unwrap_or_else(Rotation3::identity);
        Self { rotation: r, translation: Vec3::zeros() }
    }

    /// Builds from a rotation matrix (re-orthonormalised) and translation.
    #[must_use]
    pub fn from_parts(r: &Mat3, t: Vec3) -> Self {
        let rot = Rotation3::from_matrix(r);
        Self { rotation: rot, translation: t }
    }

    /// Applies to a point.
    #[must_use]
    pub fn apply(&self, p: &Point3) -> Point3 {
        self.rotation * p + self.translation
    }

    /// Applies the rotation only (for normals and directions).
    #[must_use]
    pub fn apply_vec(&self, v: &Vec3) -> Vec3 {
        self.rotation * v
    }

    /// `self ∘ other`: first `other`, then `self`.
    #[must_use]
    pub fn then_after(&self, other: &Transform) -> Transform {
        Transform {
            rotation: self.rotation * other.rotation,
            translation: self.rotation * other.translation + self.translation,
        }
    }

    /// The inverse motion.
    #[must_use]
    pub fn inverse(&self) -> Transform {
        let ri = self.rotation.inverse();
        Transform { rotation: ri, translation: -(ri * self.translation) }
    }

    /// The homogeneous 4×4 matrix.
    #[must_use]
    pub fn to_matrix(&self) -> Mat4 {
        let mut m = Mat4::identity();
        m.fixed_view_mut::<3, 3>(0, 0).copy_from(self.rotation.matrix());
        m.fixed_view_mut::<3, 1>(0, 3).copy_from(&self.translation);
        m
    }

    /// Row-major 4×4 array (the JSON and kernel form).
    #[must_use]
    pub fn to_rows(&self) -> [[f64; 4]; 4] {
        let m = self.to_matrix();
        let mut out = [[0.0; 4]; 4];
        for (i, row) in out.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v = m[(i, j)];
            }
        }
        out
    }

    /// From a row-major 4×4 array; the 3×3 block is projected onto the nearest rotation.
    /// `None` when not finite or the block is not close to a rotation (|det − 1| > 1e-3).
    #[must_use]
    pub fn from_rows(rows: &[[f64; 4]; 4]) -> Option<Transform> {
        if rows.iter().flatten().any(|v| !v.is_finite()) {
            return None;
        }
        let r = Mat3::from_fn(|i, j| rows.get(i).and_then(|row| row.get(j)).copied().unwrap_or(0.0));
        if (r.determinant() - 1.0).abs() > 1e-3 {
            return None;
        }
        let t = Vec3::new(rows[0][3], rows[1][3], rows[2][3]);
        Some(Transform::from_parts(&r, t))
    }

    /// Rotation angle in radians (0..π).
    #[must_use]
    pub fn angle(&self) -> f64 {
        self.rotation.angle()
    }

    /// Unit quaternion of the rotation.
    #[must_use]
    pub fn quaternion(&self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_rotation_matrix(&self.rotation)
    }

    /// Largest displacement of the given points between `self` and `other` (mm).
    #[must_use]
    pub fn max_displacement<'a>(&self, other: &Transform, pts: impl IntoIterator<Item = &'a Point3>) -> f64 {
        pts.into_iter().map(|p| (self.apply(p) - other.apply(p)).norm()).fold(0.0, f64::max)
    }
}

impl Serialize for Transform {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        self.to_rows().serialize(s)
    }
}

impl<'de> Deserialize<'de> for Transform {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let rows = <[[f64; 4]; 4]>::deserialize(d)?;
        Transform::from_rows(&rows).ok_or_else(|| serde::de::Error::custom("not a rigid transform"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compose_inverse_roundtrip() {
        let a = Transform::rotation_axis_angle(&Vec3::new(1.0, 2.0, 3.0), 0.7)
            .then_after(&Transform::translation(Vec3::new(1.0, -2.0, 5.0)));
        let p = Point3::new(3.0, 4.0, 5.0);
        let q = a.inverse().apply(&a.apply(&p));
        assert!((p - q).norm() < 1e-12);
        let rows = a.to_rows();
        let b = Transform::from_rows(&rows).unwrap();
        assert!((b.apply(&p) - a.apply(&p)).norm() < 1e-12);
        let json = serde_json::to_string(&a).unwrap();
        let c: Transform = serde_json::from_str(&json).unwrap();
        assert!((c.apply(&p) - a.apply(&p)).norm() < 1e-12);
        assert!(Transform::from_rows(&[[2.0, 0.0, 0.0, 0.0], [0.0; 4], [0.0; 4], [0.0; 4]]).is_none());
    }
}

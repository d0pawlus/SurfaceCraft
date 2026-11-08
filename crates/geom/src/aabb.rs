//! Axis-aligned bounding boxes.
use crate::{Point3, Vec3};
use serde::{Deserialize, Serialize};

/// An axis-aligned box; `min > max` on any axis means empty.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Aabb {
    /// Lower corner.
    pub min: Point3,
    /// Upper corner.
    pub max: Point3,
}

impl Default for Aabb {
    fn default() -> Self {
        Self::empty()
    }
}

impl Aabb {
    /// The empty box.
    #[must_use]
    pub fn empty() -> Self {
        Self {
            min: Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            max: Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }

    /// The box around `points` (empty for none; non-finite points are skipped).
    pub fn from_points<'a>(points: impl IntoIterator<Item = &'a Point3>) -> Self {
        let mut b = Self::empty();
        for p in points {
            b.grow(p);
        }
        b
    }

    /// Extends the box to contain `p` (non-finite points are ignored).
    pub fn grow(&mut self, p: &Point3) {
        if !crate::finite(p) {
            return;
        }
        self.min = self.min.inf(p);
        self.max = self.max.sup(p);
    }

    /// Is the box empty?
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    /// Size along each axis (zero when empty).
    #[must_use]
    pub fn size(&self) -> Vec3 {
        if self.is_empty() { Vec3::zeros() } else { self.max - self.min }
    }

    /// Centre (origin when empty).
    #[must_use]
    pub fn center(&self) -> Point3 {
        if self.is_empty() { Point3::origin() } else { nalgebra::center(&self.min, &self.max) }
    }

    /// Length of the diagonal (zero when empty).
    #[must_use]
    pub fn diagonal(&self) -> f64 {
        self.size().norm()
    }

    /// Squared distance from `p` to the box (zero inside).
    #[must_use]
    pub fn dist2(&self, p: &Point3) -> f64 {
        let mut d = 0.0;
        for i in 0..3 {
            let v = p[i];
            if v < self.min[i] {
                d += (self.min[i] - v).powi(2);
            } else if v > self.max[i] {
                d += (v - self.max[i]).powi(2);
            }
        }
        d
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grows_and_measures() {
        let b = Aabb::from_points(&[Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 4.0, 6.0)]);
        assert_eq!(b.size(), Vec3::new(2.0, 4.0, 6.0));
        assert_eq!(b.center(), Point3::new(1.0, 2.0, 3.0));
        assert!(Aabb::empty().is_empty());
        assert_eq!(b.dist2(&Point3::new(1.0, 1.0, 1.0)), 0.0);
        assert_eq!(b.dist2(&Point3::new(3.0, 0.0, 0.0)), 1.0);
    }
}

//! Point clouds: positions with optional per-point normals (what a scanner exports before any
//! surface exists). Reconstruction to a triangle mesh lives in [`crate::recon`].
use crate::{Aabb, Point3, Vec3};
use serde::{Deserialize, Serialize};

/// Why a point cloud is not valid.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum CloudError {
    /// A coordinate is NaN or infinite.
    #[error("point {0} has a non-finite coordinate")]
    NonFinite(usize),
    /// A normal is NaN or infinite.
    #[error("normal {0} is not finite")]
    NonFiniteNormal(usize),
    /// There is not exactly one normal per point.
    #[error("{normals} normals for {points} points")]
    NormalCount {
        /// Points.
        points: usize,
        /// Normals.
        normals: usize,
    },
    /// More points than a `u32` index can address.
    #[error("too many points ({0})")]
    TooManyPoints(usize),
}

/// An unordered set of points in millimetres, optionally with one normal per point.
///
/// Normals, when present, are not required to be unit length or consistently oriented; a zero
/// normal means "unknown" for that point.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PointCloud {
    /// Point positions.
    pub positions: Vec<Point3>,
    /// One normal per point, when the source had them.
    pub normals: Option<Vec<Vec3>>,
}

impl PointCloud {
    /// A cloud without normals.
    #[must_use]
    pub fn from_points(positions: Vec<Point3>) -> Self {
        Self { positions, normals: None }
    }

    /// A validated cloud.
    pub fn new(positions: Vec<Point3>, normals: Option<Vec<Vec3>>) -> Result<Self, CloudError> {
        let c = Self { positions, normals };
        c.validate()?;
        Ok(c)
    }

    /// Checks counts and finiteness.
    pub fn validate(&self) -> Result<(), CloudError> {
        if self.positions.len() > u32::MAX as usize {
            return Err(CloudError::TooManyPoints(self.positions.len()));
        }
        if let Some(i) = self.positions.iter().position(|p| !crate::finite(p)) {
            return Err(CloudError::NonFinite(i));
        }
        if let Some(n) = &self.normals {
            if n.len() != self.positions.len() {
                return Err(CloudError::NormalCount { points: self.positions.len(), normals: n.len() });
            }
            if let Some(i) = n.iter().position(|v| !crate::finite_vec(v)) {
                return Err(CloudError::NonFiniteNormal(i));
            }
        }
        Ok(())
    }

    /// Number of points.
    #[must_use]
    pub fn len(&self) -> usize {
        self.positions.len()
    }

    /// Is the cloud empty?
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.positions.is_empty()
    }

    /// Bounding box of the points.
    #[must_use]
    pub fn bbox(&self) -> Aabb {
        Aabb::from_points(&self.positions)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validation_catches_bad_clouds() {
        let p = vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)];
        assert!(PointCloud::new(p.clone(), None).is_ok());
        assert!(PointCloud::new(p.clone(), Some(vec![Vec3::z()])).is_err());
        assert!(PointCloud::new(p.clone(), Some(vec![Vec3::z(), Vec3::new(f64::NAN, 0.0, 0.0)])).is_err());
        assert!(PointCloud::new(vec![Point3::new(f64::INFINITY, 0.0, 0.0)], None).is_err());
        assert_eq!(PointCloud::from_points(p).len(), 2);
    }
}

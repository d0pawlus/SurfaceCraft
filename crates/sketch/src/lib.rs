//! SurfaceCraft sketches (layer L1), design doc §6.4.
//!
//! A [`Sketch`] lies on a plane: points (the unknowns, two per point, plus one radius per
//! circle), entities (lines, arcs, circles) between them, and constraints. [`solve`] runs a
//! damped Gauss–Newton (Levenberg–Marquardt) on the constraint residuals and analyses the
//! Jacobian at the result: the rank gives the remaining degrees of freedom, and constraints
//! that add no rank are redundant — conflicting when their residual cannot vanish
//! ("sketch is overdefined").
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

pub mod chains;
pub mod constraint;
pub mod fit2d;
pub mod moving;
pub mod profile;
pub mod solve;

pub use constraint::Constraint;
pub use moving::move_point;
pub use profile::{Segment3, open_paths, path_length, path_samples, profiles};
pub use solve::{SATISFIED_TOL, SolveReport, SolveStatus, polish, solve};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Point3, Vec3};

/// The plane a sketch lies on: `origin + x·u + y·v`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct SketchPlane {
    /// Origin.
    pub origin: [f64; 3],
    /// Unit x direction.
    pub x: [f64; 3],
    /// Unit y direction (perpendicular to x).
    pub y: [f64; 3],
}

impl Default for SketchPlane {
    fn default() -> Self {
        Self { origin: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0] }
    }
}

impl SketchPlane {
    /// The plane through `origin` with normal `normal` (x chosen perpendicular).
    #[must_use]
    pub fn from_normal(origin: [f64; 3], normal: [f64; 3]) -> Option<Self> {
        let n = surfacecraft_geom::normalize(Vec3::from(normal))?;
        let x = surfacecraft_geom::any_perpendicular(&n);
        let y = n.cross(&x);
        Some(Self { origin, x: x.into(), y: y.into() })
    }

    /// Unit normal `x × y`.
    #[must_use]
    pub fn normal(&self) -> Vec3 {
        Vec3::from(self.x).cross(&Vec3::from(self.y))
    }

    /// 3-D point of sketch coordinates.
    #[must_use]
    pub fn to_3d(&self, p: [f64; 2]) -> Point3 {
        Point3::from(self.origin) + Vec3::from(self.x) * p[0] + Vec3::from(self.y) * p[1]
    }

    /// Sketch coordinates of a 3-D point (projected).
    #[must_use]
    pub fn to_2d(&self, p: &Point3) -> [f64; 2] {
        let w = p - Point3::from(self.origin);
        [w.dot(&Vec3::from(self.x)), w.dot(&Vec3::from(self.y))]
    }

    /// Is it a proper orthonormal frame?
    #[must_use]
    pub fn is_valid(&self) -> bool {
        let (x, y) = (Vec3::from(self.x), Vec3::from(self.y));
        self.origin.iter().all(|v| v.is_finite())
            && (x.norm() - 1.0).abs() < 1e-6
            && (y.norm() - 1.0).abs() < 1e-6
            && x.dot(&y).abs() < 1e-6
    }
}

/// A sketch entity; points are indices into [`Sketch::points`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Entity {
    /// Segment from `a` to `b`.
    Line {
        /// Start point.
        a: usize,
        /// End point.
        b: usize,
    },
    /// Arc around `center`, counter-clockwise from `a` to `b` (|a−c| = |b−c| is enforced).
    Arc {
        /// Centre point.
        center: usize,
        /// Start point.
        a: usize,
        /// End point.
        b: usize,
    },
    /// Full circle around `center` with a radius unknown.
    Circle {
        /// Centre point.
        center: usize,
        /// Radius (an unknown of the solve).
        radius: f64,
    },
}

/// A 2-D sketch.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Sketch {
    /// Its plane.
    pub plane: SketchPlane,
    /// Points (sketch coordinates).
    pub points: Vec<[f64; 2]>,
    /// Entities.
    pub entities: Vec<Entity>,
    /// Constraints.
    pub constraints: Vec<Constraint>,
}

/// Why a sketch is malformed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum SketchError {
    /// An index refers to nothing.
    #[error("{0}")]
    Index(String),
    /// A value is not finite or out of range.
    #[error("{0}")]
    Value(String),
}

/// Largest sketch accepted (points + entities + constraints).
pub const MAX_ITEMS: usize = 20_000;

impl Sketch {
    /// Adds a point, returning its index.
    pub fn add_point(&mut self, p: [f64; 2]) -> usize {
        self.points.push(p);
        self.points.len() - 1
    }

    /// Adds a line between new points, returning the entity index.
    pub fn add_line(&mut self, a: [f64; 2], b: [f64; 2]) -> usize {
        let (ia, ib) = (self.add_point(a), self.add_point(b));
        self.entities.push(Entity::Line { a: ia, b: ib });
        self.entities.len() - 1
    }

    /// Checks indices and values (hostile input).
    pub fn validate(&self) -> Result<(), SketchError> {
        if self.points.len() + self.entities.len() + self.constraints.len() > MAX_ITEMS {
            return Err(SketchError::Value(format!("sketch over {MAX_ITEMS} items")));
        }
        if !self.plane.is_valid() {
            return Err(SketchError::Value("the sketch plane is not an orthonormal frame".into()));
        }
        if self.points.iter().flatten().any(|v| !v.is_finite()) {
            return Err(SketchError::Value("a point is not finite".into()));
        }
        let np = self.points.len();
        let pt = |i: usize, what: &str| -> Result<(), SketchError> {
            if i < np {
                Ok(())
            } else {
                Err(SketchError::Index(format!("{what}: point {i} does not exist ({np} points)")))
            }
        };
        for (k, e) in self.entities.iter().enumerate() {
            match e {
                Entity::Line { a, b } => {
                    pt(*a, "line")?;
                    pt(*b, "line")?;
                    if a == b {
                        return Err(SketchError::Index(format!("entity {k}: a line needs two points")));
                    }
                }
                Entity::Arc { center, a, b } => {
                    pt(*center, "arc")?;
                    pt(*a, "arc")?;
                    pt(*b, "arc")?;
                }
                Entity::Circle { center, radius } => {
                    pt(*center, "circle")?;
                    if !(radius.is_finite() && *radius > 0.0) {
                        return Err(SketchError::Value(format!("entity {k}: circle radius must be positive")));
                    }
                }
            }
        }
        for (k, c) in self.constraints.iter().enumerate() {
            c.validate(self).map_err(|m| SketchError::Index(format!("constraint {k}: {m}")))?;
        }
        Ok(())
    }

    /// Endpoints of an entity (`None` for a circle).
    #[must_use]
    pub fn endpoints(&self, e: usize) -> Option<(usize, usize)> {
        match self.entities.get(e)? {
            Entity::Line { a, b } | Entity::Arc { a, b, .. } => Some((*a, *b)),
            Entity::Circle { .. } => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plane_roundtrip_and_validation() {
        let pl = SketchPlane::from_normal([1.0, 2.0, 3.0], [0.0, 0.0, 2.0]).unwrap();
        let p = pl.to_3d([4.0, 5.0]);
        let q = pl.to_2d(&p);
        assert!((q[0] - 4.0).abs() < 1e-12 && (q[1] - 5.0).abs() < 1e-12);
        let mut s = Sketch::default();
        s.add_line([0.0, 0.0], [1.0, 0.0]);
        assert!(s.validate().is_ok());
        s.entities.push(Entity::Line { a: 0, b: 9 });
        assert!(s.validate().is_err());
    }
}

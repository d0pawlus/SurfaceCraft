//! The relation kinds, the relations found and the mirror planes, with the failures of imposing them.

use super::*;

/// Kinds of relation [`apply`] can impose.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RelKind {
    /// A plane touches a sphere, a cylinder or a torus tube; two parallel cylinders touch;
    /// a cylinder touches a coaxial torus.
    Tangent,
    /// `a` is the mirror image of `b` about `mirror`.
    Symmetric,
    /// `b` lies halfway between `a` and `c` (planes along their normal; parallel axes).
    EqualDistance,
    /// Parallel axes on one line (carried along so that it keeps holding).
    Coaxial,
    /// Equal radii (carried along so that it keeps holding).
    EqualRadius,
}

/// A relation between primitives, by index into the slice given to the functions here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rel {
    /// Kind.
    pub kind: RelKind,
    /// First primitive.
    pub a: usize,
    /// Second primitive (the middle one of an equal distance).
    pub b: usize,
    /// Third primitive (equal distance only).
    pub c: Option<usize>,
    /// Index of the mirror plane (symmetric only).
    pub mirror: Option<usize>,
}

/// A mirror plane: through `point`, with unit `normal`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mirror {
    /// A point on the plane.
    pub point: Vec3,
    /// Unit normal.
    pub normal: Vec3,
}

impl Mirror {
    /// A mirror through `point` with `normal` (normalised); `None` when the normal is zero.
    #[must_use]
    pub fn new(point: Vec3, normal: Vec3) -> Option<Mirror> {
        normalize(normal).map(|n| Mirror { point, normal: n })
    }

    /// The mirror image of a direction.
    pub(super) fn dir(&self, d: &Vec3) -> Vec3 {
        d - self.normal * (2.0 * d.dot(&self.normal))
    }

    /// The mirror image of a point (as a vector).
    pub(super) fn pt(&self, x: &Vec3) -> Vec3 {
        x - self.normal * (2.0 * (x - self.point).dot(&self.normal))
    }
}

/// Why [`apply`] gave up.
#[derive(Clone, Debug, PartialEq)]
pub struct Failure {
    /// What went wrong, for the user.
    pub message: String,
    /// The relation (index into the list given to [`apply`]) that conflicts most, if known.
    pub worst: Option<usize>,
}

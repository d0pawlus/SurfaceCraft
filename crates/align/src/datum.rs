//! 3-2-1 alignment: a primary, secondary and tertiary datum (planes, axes, points) define a
//! coordinate system; the result moves the model so that frame becomes the world frame.
//!
//! - Primary fixes **+Z** (plane normal or axis direction).
//! - Secondary fixes **+X** (its direction made perpendicular to Z, Gram-Schmidt).
//! - The origin satisfies, in order, the position constraints of the datums: a plane gives one
//!   (`n·x = n·p`), an axis two (the point lies on it), a point three; constraints that add
//!   nothing new are skipped until three independent ones are found.
use crate::AlignError;
use nalgebra::{Matrix3, Vector3};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Mat3, Point3, Transform, Vec3};

/// A datum entity.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Datum {
    /// A plane through `point` with normal `normal`.
    Plane {
        /// A point on the plane.
        point: [f64; 3],
        /// Its normal (need not be unit).
        normal: [f64; 3],
    },
    /// A line through `point` along `direction`.
    Axis {
        /// A point on the axis.
        point: [f64; 3],
        /// Its direction.
        direction: [f64; 3],
    },
    /// A point.
    Point {
        /// The point.
        point: [f64; 3],
    },
}

impl Datum {
    fn direction(&self) -> Option<Vec3> {
        match self {
            Datum::Plane { normal, .. } => surfacecraft_geom::normalize(Vec3::from(*normal)),
            Datum::Axis { direction, .. } => surfacecraft_geom::normalize(Vec3::from(*direction)),
            Datum::Point { .. } => None,
        }
    }

    fn point(&self) -> Point3 {
        match self {
            Datum::Plane { point, .. } | Datum::Axis { point, .. } | Datum::Point { point } => Point3::from(*point),
        }
    }

    /// Position constraints `a·x = b`.
    fn constraints(&self) -> Vec<(Vec3, f64)> {
        let p = self.point();
        match self {
            Datum::Plane { .. } => self.direction().map(|n| vec![(n, n.dot(&p.coords))]).unwrap_or_default(),
            Datum::Axis { .. } => self
                .direction()
                .map(|d| {
                    let u = surfacecraft_geom::any_perpendicular(&d);
                    let v = d.cross(&u);
                    vec![(u, u.dot(&p.coords)), (v, v.dot(&p.coords))]
                })
                .unwrap_or_default(),
            Datum::Point { .. } => {
                vec![(Vec3::x(), p.x), (Vec3::y(), p.y), (Vec3::z(), p.z)]
            }
        }
    }

    fn finite(&self) -> bool {
        let (a, b) = match self {
            Datum::Plane { point, normal } => (*point, *normal),
            Datum::Axis { point, direction } => (*point, *direction),
            Datum::Point { point } => (*point, [0.0; 3]),
        };
        a.iter().chain(b.iter()).all(|v| v.is_finite())
    }
}

/// The motion taking the datum frame to the world frame.
pub fn datum_321(primary: &Datum, secondary: &Datum, tertiary: &Datum) -> Result<Transform, AlignError> {
    if ![primary, secondary, tertiary].iter().all(|d| d.finite()) {
        return Err(AlignError::Degenerate("datum values must be finite".into()));
    }
    let z = primary
        .direction()
        .ok_or_else(|| AlignError::Degenerate("the primary datum must be a plane or an axis".into()))?;
    let s = secondary
        .direction()
        .ok_or_else(|| AlignError::Degenerate("the secondary datum must be a plane or an axis".into()))?;
    let x = surfacecraft_geom::normalize(s - z * s.dot(&z))
        .ok_or_else(|| AlignError::Degenerate("the secondary direction is parallel to the primary".into()))?;
    if (s - z * s.dot(&z)).norm() < 1e-6 {
        return Err(AlignError::Degenerate("the secondary direction is parallel to the primary".into()));
    }
    let y = z.cross(&x);
    // origin from independent constraints, in datum order
    let mut rows: Vec<(Vec3, f64)> = Vec::new();
    for d in [primary, secondary, tertiary] {
        for (a, b) in d.constraints() {
            if rows.len() == 3 {
                break;
            }
            let mut m = Mat3::zeros();
            for (i, (r, _)) in rows.iter().chain(std::iter::once(&(a, b))).enumerate() {
                m.set_row(i, &r.transpose());
            }
            let rank = m.svd(false, false).singular_values.iter().filter(|s| **s > 1e-9).count();
            if rank == rows.len() + 1 {
                rows.push((a, b));
            }
        }
    }
    if rows.len() < 3 {
        return Err(AlignError::Degenerate(
            "the datums do not fix an origin (need three independent constraints)".into(),
        ));
    }
    let a = Matrix3::from_rows(&[rows[0].0.transpose(), rows[1].0.transpose(), rows[2].0.transpose()]);
    let b = Vector3::new(rows[0].1, rows[1].1, rows[2].1);
    let origin = a.lu().solve(&b).ok_or_else(|| AlignError::Degenerate("origin constraints are singular".into()))?;
    let r = Mat3::from_rows(&[x.transpose(), y.transpose(), z.transpose()]);
    Ok(Transform::from_parts(&r, -(r * origin)))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plane(p: Point3, n: Vec3) -> Datum {
        Datum::Plane { point: p.into(), normal: n.into() }
    }

    /// Gate M1: the faces of a displaced block, used as 3-2-1 datums, recover the displacement
    /// to within 1e-6 mm (noise-free).
    #[test]
    fn gate_m1_datum_321_recovers_known_motion() {
        let truth = Transform::rotation_axis_angle(&Vec3::new(1.0, 2.0, -0.5), 0.9)
            .then_after(&Transform::translation(Vec3::new(30.0, -12.0, 4.0)));
        let moved = truth.inverse();
        // block faces z=0 (normal +z), x=0 (normal +x), y=0 (normal +y), seen after displacement
        let pz = plane(moved.apply(&Point3::new(5.0, 5.0, 0.0)), moved.apply_vec(&Vec3::z()));
        let px = plane(moved.apply(&Point3::new(0.0, 3.0, 2.0)), moved.apply_vec(&Vec3::x()));
        let py = plane(moved.apply(&Point3::new(4.0, 0.0, 1.0)), moved.apply_vec(&Vec3::y()));
        let t = datum_321(&pz, &px, &py).unwrap();
        let probe: Vec<Point3> = (0..8)
            .map(|i| Point3::new(f64::from(i & 1) * 60.0, f64::from((i >> 1) & 1) * 40.0, f64::from(i >> 2) * 20.0))
            .collect();
        let err = t.then_after(&moved).max_displacement(&Transform::identity(), &probe);
        assert!(err <= 1e-6, "{err}");
    }

    #[test]
    fn axis_primary_and_point_tertiary() {
        // a cylinder axis along world Y through (1,0,2) as primary, plane x=1 as secondary, point
        let t = datum_321(
            &Datum::Axis { point: [1.0, 0.0, 2.0], direction: [0.0, 1.0, 0.0] },
            &plane(Point3::new(1.0, 0.0, 0.0), Vec3::x()),
            &Datum::Point { point: [1.0, 5.0, 2.0] },
        )
        .unwrap();
        let o = t.apply(&Point3::new(1.0, 5.0, 2.0));
        assert!(o.coords.norm() < 1e-12, "{o:?}");
        assert!((t.apply_vec(&Vec3::y()) - Vec3::z()).norm() < 1e-12);
    }

    #[test]
    fn degenerate_datums_are_errors() {
        let p = plane(Point3::origin(), Vec3::z());
        assert!(datum_321(&p, &p, &p).is_err());
        assert!(datum_321(&Datum::Point { point: [0.0; 3] }, &p, &p).is_err());
        let q = plane(Point3::origin(), Vec3::x());
        assert!(datum_321(&p, &q, &q).is_err()); // origin along y not fixed
        assert!(datum_321(&plane(Point3::new(f64::NAN, 0.0, 0.0), Vec3::z()), &q, &q).is_err());
    }
}

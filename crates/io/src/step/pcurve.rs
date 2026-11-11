//! PCURVE entities: an edge in the parameters of a face's surface (ISO 10303-42), as a
//! DEFINITIONAL_REPRESENTATION of a 2-D B-spline.

use super::p21::Param;
use super::topology::ShellWriter;
use super::{r, s};
use crate::IoError;
use surfacecraft_kernel::record::brep::Curve2;

/// What a face's edges need from it: its surface entity, which side its written normal takes,
/// and a pcurve for each edge use of each loop (`pcurves[loop][position]`).
#[derive(Clone, Debug)]
pub(super) struct FaceGeometry {
    pub(super) surface: u64,
    /// The written surface's natural normal opposes the record's: the ADVANCED_FACE's sense flips.
    pub(super) flip: bool,
    /// The surface is closed in a direction (a cylinder, cone, sphere or torus), so an edge a face
    /// uses twice may be a seam; on any other surface it is a slit.
    pub(super) periodic: bool,
    pub(super) pcurves: Vec<Vec<Curve2>>,
    /// The surface in STEP's parameters, for the tests' gap measurements.
    #[cfg(test)]
    pub(super) written: surfacecraft_kernel::record::brep::Surface,
}

impl ShellWriter<'_> {
    /// The 2-D parametric context a DEFINITIONAL_REPRESENTATION lives in.
    fn context_2d(&mut self) -> u64 {
        self.w.add_complex(&[
            ("GEOMETRIC_REPRESENTATION_CONTEXT", vec![Param::Int(2)]),
            ("PARAMETRIC_REPRESENTATION_CONTEXT", vec![]),
            ("REPRESENTATION_CONTEXT", vec![Param::Str("2D SPACE".into()), s()]),
        ])
    }

    /// The 2-D B-spline of a pcurve.
    fn curve_2d(&mut self, c: &Curve2) -> Result<u64, IoError> {
        match c {
            Curve2::Spline { knots, points } => {
                let ids = points.iter().map(|p| self.point2(*p).map(r)).collect::<Result<Vec<_>, _>>()?;
                self.bspline_curve(knots, ids, None)
            }
            Curve2::RationalSpline { knots, homogeneous } => {
                let mut ids = Vec::with_capacity(homogeneous.len());
                let mut ws = Vec::with_capacity(homogeneous.len());
                for h in homogeneous {
                    if !(h[2] > 0.0) || !h[2].is_finite() {
                        return Err(super::bad("rational weight must be positive"));
                    }
                    ids.push(r(self.point2([h[0] / h[2], h[1] / h[2]])?));
                    ws.push(h[2]);
                }
                self.bspline_curve(knots, ids, Some(&ws))
            }
        }
    }

    /// `PCURVE('', surface, DEFINITIONAL_REPRESENTATION('', (curve), context))`.
    pub(super) fn pcurve(&mut self, surface: u64, c: &Curve2) -> Result<u64, IoError> {
        let curve = self.curve_2d(c)?;
        let ctx = self.context_2d();
        let rep = self.w.add_distinct("DEFINITIONAL_REPRESENTATION", &[s(), Param::List(vec![r(curve)]), r(ctx)]);
        Ok(self.w.add_distinct("PCURVE", &[s(), r(surface), r(rep)]))
    }
}

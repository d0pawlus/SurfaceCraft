//! Reference faces: the unit normal of a body face and its normal curvature in a tangent
//! direction, through solidcore's evaluators (`SurfaceEval`, `normal_section`). Nothing here
//! evaluates a surface by hand.
use super::geom::{COORD_LIMIT, V3, dot, finite3, scale, sub, unit};
use crate::record::brep::{Face, Surface};
use crate::{KernelError, Result, op_err};
use solidcore::eval::normal_section::normal_section;
use solidcore::eval::spline::Partials;
use solidcore::eval::{Starts, SurfaceEval};
use solidcore::math::Vec3;

/// A point of a reference face within this share of its coordinate size (at least 1) counts as on it.
const ACCEPT_REL: f64 = 1e-2;

fn arg(msg: impl Into<String>) -> KernelError {
    KernelError::Argument(msg.into())
}

fn v3(a: V3) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

/// Everything the driver needs of a reference face at one point: its unit normal and the partial
/// derivatives the curvature of a normal section is computed from.
#[derive(Clone)]
pub(super) struct RefSample {
    /// Unit normal `S_u × S_v` of the face's own parameters.
    pub normal: V3,
    partials: Partials,
}

impl RefSample {
    /// Normal curvature (1/mm, with respect to `normal`) along the tangent direction `t`: `t` is
    /// projected into the tangent plane first. `None` where the section is not defined.
    pub fn curvature(&self, t: V3) -> Option<f64> {
        let w = unit(sub(t, scale(self.normal, dot(t, self.normal))))?;
        normal_section(&self.partials, v3(self.normal), v3(w)).map(|j| j.curvature)
    }
}

/// A reference face prepared for repeated queries.
pub(super) struct RefProbe {
    eval: SurfaceEval,
    starts: Starts,
}

impl RefProbe {
    /// The evaluator of the face's surface; a ball patch or a placed revolution (which solidcore
    /// does not evaluate to second order by its label) is evaluated on its NURBS form.
    pub fn new(face: &Face) -> Result<Self> {
        let own: &Surface = match (&face.surface, &face.nurbs) {
            (Surface::BallPatch { .. } | Surface::PlacedRevolution { .. }, Some(n)) => n,
            (s, _) => s,
        };
        let k = solidcore_convert::surface(own).map_err(|e| op_err("convert surface", e))?;
        let eval = SurfaceEval::new(&k).map_err(|e| op_err("surface evaluator", e))?;
        Ok(Self { eval, starts: Starts::default() })
    }

    /// The point of the face's surface closest to `p`; `None` when `p` is farther than `accept` from it.
    pub fn closest(&self, p: V3, accept: f64) -> Result<Option<V3>> {
        if !finite3(&p) {
            return Err(arg("the point must be finite"));
        }
        let Some(uv) = self.eval.project_with(&self.starts, v3(p), None, accept) else { return Ok(None) };
        let q = self.eval.point(uv).map(|q| [q.x, q.y, q.z]).filter(finite3);
        Ok(q)
    }

    /// The face's normal and derivatives at the point of its surface closest to `p`.
    pub fn locate(&self, p: V3) -> Result<RefSample> {
        if !finite3(&p) {
            return Err(arg("the point must be finite"));
        }
        let size = p.iter().fold(1.0_f64, |m, v| m.max(v.abs()));
        let uv =
            self.eval.project_with(&self.starts, v3(p), None, ACCEPT_REL * size.min(COORD_LIMIT)).ok_or_else(|| {
                arg("a point of the side does not lie on its reference face (is the face adjacent to the side?)")
            })?;
        let d = self
            .eval
            .partials(uv[0], uv[1])
            .ok_or_else(|| op_err("surface partials", "the reference face has no derivatives there"))?;
        let n = d[1][0].cross(d[0][1]);
        let normal = unit([n.x, n.y, n.z])
            .ok_or_else(|| arg("the reference face has no normal at a point of the side (a pole?)"))?;
        Ok(RefSample { normal, partials: d })
    }
}

/// Unit normal of `face` at the point of its surface closest to `p`, and its normal curvature in
/// the tangent direction `t` there (through solidcore evaluation; `Err` for a face it cannot
/// evaluate). The curvature is `n · r''` along `t` with the returned normal `n`: negative where the
/// surface bends away from `n`; `t` is projected into the tangent plane and need not be unit.
pub fn face_normal_curvature(face: &Face, p: [f64; 3], t: [f64; 3]) -> Result<([f64; 3], f64)> {
    if !finite3(&p) || !finite3(&t) {
        return Err(arg("the point and the direction must be finite"));
    }
    let sample = RefProbe::new(face)?.locate(p)?;
    let kappa = sample.curvature(t).ok_or_else(|| arg("the direction has no normal curvature there"))?;
    Ok((sample.normal, kappa))
}

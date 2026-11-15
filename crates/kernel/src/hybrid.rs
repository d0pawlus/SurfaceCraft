//! The hybrid model (design doc §6.3, plan P3): one closed solid from scanned primitives and
//! freeform patches, built by trimming a stock.
//!
//! Every recognised surface is a *cut* of the stock. A plane keeps the material side of a
//! kernel split (`ops::split::split_body`, the plane as the tool); a convex cylinder bridging two
//! planes is a fillet of their edge (`ops::blend`); a full-revolution cylinder, cone, sphere or
//! torus is a solid of that shape intersected with (convex) or subtracted from (concave) the
//! stock; a freeform patch is a spline tool that splits the stock and keeps its material side.
//! The faces of the result are therefore exactly the primitives trimmed to the contours where
//! they meet their neighbours, which is the "primitive → trimmed B-rep face" step of the plan.
//! All B-rep construction is the kernel's; this file sequences the calls and chooses sides.
use crate::nurbs::{SurfaceProbe, canonicalize};
use crate::record::brep::{Brep, Frame, Loop, Region, Shell, Surface};
use crate::solid::{decode, encode};
use crate::{KernelError, Result, finite_all, op_err, solid};
use solidcore_ops as ops;

/// An analytic surface of a recognised region.
#[derive(Clone, Debug, PartialEq)]
pub enum Shape {
    /// Plane through `point` with unit `normal` pointing out of the material.
    Plane {
        /// A point of the plane.
        point: [f64; 3],
        /// Unit normal.
        normal: [f64; 3],
    },
    /// Cylinder about the axis through `point` along unit `axis`.
    Cylinder {
        /// A point of the axis.
        point: [f64; 3],
        /// Unit axis.
        axis: [f64; 3],
        /// Radius.
        radius: f64,
    },
    /// Cone with `apex`, unit `axis` pointing into its opening, `half_angle` in (0, π/2).
    Cone {
        /// Apex.
        apex: [f64; 3],
        /// Unit axis into the opening.
        axis: [f64; 3],
        /// Half angle, radians.
        half_angle: f64,
    },
    /// Sphere.
    Sphere {
        /// Centre.
        center: [f64; 3],
        /// Radius.
        radius: f64,
    },
    /// Ring torus about unit `axis`.
    Torus {
        /// Centre.
        center: [f64; 3],
        /// Unit axis.
        axis: [f64; 3],
        /// Major radius.
        major_radius: f64,
        /// Minor radius.
        minor_radius: f64,
    },
}

/// A primitive cut of the stock.
#[derive(Clone, Debug, PartialEq)]
pub struct Cut {
    /// The surface.
    pub shape: Shape,
    /// The material lies inside the shape (a boss, an outer wall, a fillet), not outside it
    /// (a hole, a countersink). Ignored for planes: their material is behind the normal.
    pub convex: bool,
    /// The region wraps all the way round the axis (cylinder, cone). A partial convex cylinder
    /// is a fillet and needs the two planes it is tangent to.
    pub full: bool,
}

/// A freeform cut: a spline tool over its parameter box.
#[derive(Clone, Debug, PartialEq)]
pub struct FreeCut {
    /// A record spline surface (see [`crate::nurbs::fit_grid`]).
    pub surface: Surface,
    /// Direction the region's outward normals point on average (picks the material side).
    pub outward: [f64; 3],
}

/// What [`solid_from_cuts`] did.
#[derive(Clone, Debug)]
pub struct Built {
    /// The closed solid.
    pub brep: Brep,
    /// Per primitive cut: how it was applied, or why it was left out.
    pub primitives: Vec<CutStatus>,
    /// Per freeform cut: applied?
    pub freeform: Vec<CutStatus>,
    /// Per local tool (blind hole, pocket, slot): applied?
    pub tools: Vec<CutStatus>,
    /// Per local addition (boss, dome): applied?
    pub adds: Vec<CutStatus>,
}

/// The outcome of one cut.
#[derive(Clone, Debug, PartialEq)]
pub enum CutStatus {
    /// Applied; the text says how (`halfspace`, `fillet`, `intersect`, `subtract`, `split`).
    Applied(&'static str),
    /// Left out, with the reason.
    Skipped(String),
}

mod build;
mod cutters;
mod faces;
mod patch;
#[cfg(test)]
mod tests;
mod vector;

pub use build::{solid_from_cuts, solid_from_cuts_and_tools, solid_from_plan};
pub(crate) use cutters::keep_front;
pub use faces::{analytic_faces, extract_face, faces_on_shape, faces_on_spline, planar_face_outward};
pub(crate) use faces::{axis_distance, face_vertices, frame_axis};
pub use patch::{cut_by_spline, patch_rise, replace_face, replace_face_up_and_down};
pub(crate) use vector::{V, add, corners, cross, dot, frame_matrix, norm, perpendicular, scale, sub, unit};

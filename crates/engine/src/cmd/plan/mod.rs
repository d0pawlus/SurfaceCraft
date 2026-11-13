//! The planner of the hybrid model (plan P3b, Z1 and Z2; P8-B2): which fitted regions are *global*
//! cuts of the stock (the outer envelope of the part) and which are *local* (a pocket floor and
//! walls, a blind hole's wall and bottom, a slot's walls, a step's riser, a rib's sides), and the
//! local tools the latter form (see `pocket`, `slot` and `loops`).
//!
//! The test for a plane is the one a person would make: is there scan in front of it? A global
//! plane has none (the part lies wholly behind it); a pocket floor has the rim of the pocket in
//! front of it. The same logic drives `body.from_regions` (which builds the solid from the plan)
//! and `body.plan_regions` (which only reports it), so the diagnostic never disagrees with the
//! build. Everything here is mesh-side geometry on the points of the regions; the solids of the
//! tools are the kernel's (`surfacecraft_kernel::local`).
mod extent;
mod joint;
mod loops;
mod pocket;
mod profile;
mod scan;
mod slot;
mod void;
use pocket::{Env, plan_pockets};
pub use scan::Scan;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use slot::plan_slots;
use surfacecraft_doc::ObjectId;
use surfacecraft_geom::{KdTree, Point3, Vec3, normalize};
use surfacecraft_kernel::hybrid::{Cut, Shape};
use surfacecraft_kernel::local::{HoleBottom, LocalAdd, LocalTool};

/// Fraction of the scan allowed in front of a plane (or outside a cylinder) that is still the
/// part's envelope: outliers, not geometry.
const ENVELOPE_FRACTION: f64 = 0.005;
/// Largest gap between two regions that are neighbours (mm): the width of an unmodelled fillet.
const NEIGHBOUR_GAP: f64 = 3.0;

/// A fitted region the planner looks at.
pub struct Item {
    /// The primitive object.
    pub id: ObjectId,
    /// Its kind of surface, for reports.
    pub kind: &'static str,
    /// Cut (shape, material side, full revolution) from the scan normals.
    pub cut: Cut,
    /// The points of its region.
    pub pts: Vec<Point3>,
}

/// The role of a region in the solid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RoleKind {
    /// A cut of the stock's envelope (half space, full revolution).
    Global,
    /// A fillet of two global planes.
    Blend,
    /// Wall, floor or bottom of a tool that is subtracted (hole, pocket, slot).
    LocalSubtract,
    /// A boss or dome that adds material.
    LocalAdd,
    /// Not used, with the reason.
    Skipped,
}

impl RoleKind {
    /// Name in reports.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            RoleKind::Global => "global",
            RoleKind::Blend => "blend",
            RoleKind::LocalSubtract => "local-subtract",
            RoleKind::LocalAdd => "local-add",
            RoleKind::Skipped => "skipped",
        }
    }
}

/// What the planner decided for one item.
#[derive(Clone, Debug)]
pub struct Role {
    /// The role.
    pub kind: RoleKind,
    /// Why.
    pub how: String,
    /// Tools this region belongs to (indices into [`Plan::tools`]).
    pub tools: Vec<usize>,
    /// Local additions this region belongs to (indices into [`Plan::adds`]).
    pub adds: Vec<usize>,
    /// The surface as planned (snapped axes, refitted cone).
    pub shape: Shape,
}

/// A contact of two neighbouring regions.
#[derive(Clone, Debug)]
pub struct Contact {
    /// First item.
    pub a: usize,
    /// Second item.
    pub b: usize,
    /// `convex`, `concave` or `tangent`.
    pub kind: &'static str,
    /// Angle between the surface normals at the contact (degrees; 0 is tangent).
    pub angle_deg: f64,
    /// Smallest distance between the regions' points (mm).
    pub distance: f64,
}

/// A local tool and the regions that form it.
#[derive(Clone, Debug)]
pub struct PlannedTool {
    /// The tool for the kernel.
    pub tool: LocalTool,
    /// `blind_hole`, `pocket` (walls round a floor, with the islands it leaves), `step` (walls with a
    /// floor that run out of the part), `rib` (a face cut down round raised islands) or `through_slot`.
    pub kind: &'static str,
    /// Items that form it.
    pub members: Vec<usize>,
    /// Numbers for the report.
    pub detail: String,
}

/// A local addition (boss, dome) and the regions that form it.
#[derive(Clone, Debug)]
pub struct PlannedAdd {
    /// The solid for the kernel.
    pub add: LocalAdd,
    /// `boss` or `dome`.
    pub kind: &'static str,
    /// Items that form it.
    pub members: Vec<usize>,
    /// Numbers for the report.
    pub detail: String,
}

/// A global plane the planner tried to square to a model axis.
#[derive(Clone, Debug)]
pub struct SquaredPlane {
    /// The item (base of its node).
    pub item: usize,
    /// Angle between the fitted and the squared normal, degrees.
    pub angle_deg: f64,
    /// Largest distance the squaring moves the plane over its region, mm.
    pub max_shift: f64,
    /// The largest shift allowed, mm.
    pub limit: f64,
    /// Squared (false: left as fitted because the shift exceeds the limit).
    pub applied: bool,
}

/// The plan.
#[derive(Clone, Debug)]
pub struct Plan {
    /// Planes the planner tried to square.
    pub squared: Vec<SquaredPlane>,
    /// Cuts for the kernel's global steps.
    pub cuts: Vec<Cut>,
    /// Items per cut (merged coplanar planes share one).
    pub cut_members: Vec<Vec<usize>>,
    /// Local tools.
    pub tools: Vec<PlannedTool>,
    /// Local additions (bosses, domes).
    pub adds: Vec<PlannedAdd>,
    /// Role per item.
    pub roles: Vec<Role>,
    /// Neighbour graph with the kind of each contact.
    pub contacts: Vec<Contact>,
}

mod boss;
mod nodes;
mod planner;
mod report;
#[cfg(test)]
mod tests;

use boss::*;
use nodes::*;
use planner::*;

pub use planner::plan;
pub use report::{ContactOut, PlanOut, PlannedRegion, PlannedToolOut, SquaredOut, report};

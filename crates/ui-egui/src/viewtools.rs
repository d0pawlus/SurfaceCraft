//! The viewport's interaction tools, wired to the application: triangle selection (brush, lasso,
//! smart select), picking B-rep faces and edges, the move/rotate gizmo, standard views with the
//! view cube, and the section plane.
//!
//! The maths lives in [`crate::select`], [`crate::brep`], [`crate::gizmo`], [`crate::viewcube`]
//! and [`crate::sectionview`] (testable without a GPU); this module holds the state, turns pointer
//! events into those calls and feeds the results into commands (`region.create`,
//! `mesh.transform`, the `edges` / `faces` / `triangles` parameters of the palette).
use crate::brep::{BrepHit, BrepSelection};
use crate::camera::ViewAnim;
use crate::gizmo::{Axis, Gizmo};
use crate::sectionview::SectionPlane;
use crate::select::{SelectOp, TriSelection};
use section::CapPaint;
use std::collections::HashMap;
use std::sync::Arc;
use surfacecraft_doc::ObjectId;
use surfacecraft_geom::Point3;

mod input;
mod paint;
mod picking;
mod section;
mod selection;
mod transform;
mod views;

/// How far from an edge a click still picks it (px).
pub const EDGE_PICK_PX: f64 = 7.0;
/// The tool the primary mouse button drives in the viewport.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Tool {
    /// Orbit; a click picks a point.
    #[default]
    Navigate,
    /// Paint triangles (Shift erases).
    Brush,
    /// Select the triangles inside a drawn outline.
    Lasso,
    /// Select the smooth patch around a clicked triangle.
    Smart,
    /// Pick faces of bodies.
    Faces,
    /// Pick edges of bodies.
    Edges,
    /// Move the selected mesh along an axis.
    Move,
    /// Rotate the selected mesh about an axis.
    Rotate,
}

impl Tool {
    /// Every tool, in toolbar order.
    pub const ALL: [Tool; 8] =
        [Tool::Navigate, Tool::Brush, Tool::Lasso, Tool::Smart, Tool::Faces, Tool::Edges, Tool::Move, Tool::Rotate];

    /// The catalogue key of the tool's name.
    #[must_use]
    pub fn key(self) -> &'static str {
        match self {
            Tool::Navigate => "tool.navigate",
            Tool::Brush => "tool.brush",
            Tool::Lasso => "tool.lasso",
            Tool::Smart => "tool.smart",
            Tool::Faces => "tool.faces",
            Tool::Edges => "tool.edges",
            Tool::Move => "tool.move",
            Tool::Rotate => "tool.rotate",
        }
    }

    /// The catalogue key of the sentence that says what the tool does.
    #[must_use]
    pub fn hint_key(self) -> &'static str {
        match self {
            Tool::Navigate => "tool.navigate_hint",
            Tool::Brush => "tool.brush_hint",
            Tool::Lasso => "tool.lasso_hint",
            Tool::Smart => "tool.smart_hint",
            Tool::Faces | Tool::Edges => "tool.brep_hint",
            Tool::Move | Tool::Rotate => "tool.gizmo_hint",
        }
    }

    /// The icon of the tool in the navigation bar.
    #[must_use]
    pub fn icon(self) -> &'static str {
        match self {
            Tool::Navigate => "tool-navigate",
            Tool::Brush => "tool-brush",
            Tool::Lasso => "tool-lasso",
            Tool::Smart => "tool-smart",
            Tool::Faces => "tool-faces",
            Tool::Edges => "tool-edges",
            Tool::Move => "tool-move",
            Tool::Rotate => "tool-rotate",
        }
    }
}
/// The state of the viewport tools.
#[derive(Clone, Debug)]
pub struct ViewTools {
    /// The active tool.
    pub tool: Tool,
    /// Brush radius (px).
    pub brush_radius: f64,
    /// Largest dihedral angle smart select crosses (degrees).
    pub smart_angle: f64,
    /// Lasso and brush also take hidden and back-facing triangles.
    pub through: bool,
    /// The triangle selection (ids of the full-resolution mesh).
    pub selection: TriSelection,
    /// The selected faces and edges of a body.
    pub brep: BrepSelection,
    /// The face or edge under the pointer.
    pub hover: Option<(ObjectId, BrepHit)>,
    /// The manipulator.
    pub gizmo: Gizmo,
    /// The section plane.
    pub section: SectionPlane,
    /// A view change in progress.
    pub anim: Option<ViewAnim>,
    /// The outline being drawn by the lasso (px, viewport-local).
    pub lasso: Vec<[f64; 2]>,
    last_brush: Option<[f64; 2]>,
    gizmo_target: Option<ObjectId>,
    gizmo_hover: Option<Axis>,
    gizmo_grabbed: bool,
    pointer: Option<[f64; 2]>,
    /// Bumped on every change of what is highlighted.
    version: u64,
    /// Highlight in drawn triangles per object: `(draw key, version, triangles)`.
    drawn: HashMap<ObjectId, (usize, u64, Arc<Vec<u32>>)>,
    /// Section cut cache (outline and cap per mesh): `(item keys, plane, cuts)`.
    cut: Option<(Vec<usize>, SectionPlane, Arc<Vec<surfacecraft_geom::cap::Cut>>)>,
    /// The section cap as last painted (see [`CapPaint`]).
    cap_paint: Option<CapPaint>,
    /// Pivot cache of the gizmo: `(mesh key, centre)`.
    pivot: Option<(usize, Point3)>,
}

impl Default for ViewTools {
    fn default() -> Self {
        Self {
            tool: Tool::Navigate,
            brush_radius: 24.0,
            smart_angle: 20.0,
            through: false,
            selection: TriSelection::default(),
            brep: BrepSelection::default(),
            hover: None,
            gizmo: Gizmo::default(),
            section: SectionPlane::default(),
            anim: None,
            lasso: Vec::new(),
            last_brush: None,
            gizmo_target: None,
            gizmo_hover: None,
            gizmo_grabbed: false,
            pointer: None,
            version: 0,
            drawn: HashMap::new(),
            cut: None,
            cap_paint: None,
            pivot: None,
        }
    }
}

impl ViewTools {
    /// Notes that what is highlighted changed.
    pub fn touch(&mut self) {
        self.version += 1;
    }

    /// The object the gizmo is moving right now.
    #[must_use]
    pub fn gizmo_target(&self) -> Option<ObjectId> {
        self.gizmo_target
    }
}
fn modifiers_op(shift: bool, ctrl: bool) -> SelectOp {
    if ctrl {
        SelectOp::Subtract
    } else if shift {
        SelectOp::Add
    } else {
        SelectOp::Replace
    }
}

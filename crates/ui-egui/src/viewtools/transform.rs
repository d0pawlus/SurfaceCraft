//! The move and rotate gizmo: picking its handles, the drag and its single `mesh.transform` command.
use super::*;
use crate::app::SurfaceCraftApp;
use crate::gizmo::{Gizmo, GizmoMode};
use crate::select::View;
use serde_json::json;
use surfacecraft_doc::ObjectKind;
use surfacecraft_engine::DisplayItem;
use surfacecraft_geom::Point3;

impl SurfaceCraftApp {
    // ------------------------------------------------------------------- gizmo

    /// The mesh the gizmo would move: the selected object when it is a scan.
    pub(crate) fn gizmo_mesh(&mut self) -> Option<DisplayItem> {
        let id = self.selected_object?;
        self.session.display_items().into_iter().find(|d| d.id == id && d.kind == ObjectKind::Mesh)
    }

    pub(super) fn pivot_of(&mut self, d: &DisplayItem) -> Point3 {
        if let Some((k, p)) = self.tools.pivot
            && k == d.pick_key
        {
            return p;
        }
        let c = d.pick_mesh.bbox().center();
        self.tools.pivot = Some((d.pick_key, c));
        c
    }

    /// Starts a gizmo drag when the press at `(x, y)` is on an axis handle. False otherwise.
    pub fn gizmo_begin(&mut self, x: f64, y: f64, w: f64, h: f64) -> bool {
        let mode = match self.tools.tool {
            Tool::Move => GizmoMode::Translate,
            Tool::Rotate => GizmoMode::Rotate,
            _ => return false,
        };
        let Some(d) = self.gizmo_mesh() else { return false };
        let cam = self.camera.clone();
        let pivot = self.pivot_of(&d);
        let mut g = Gizmo::new(mode, pivot);
        let view = View { camera: &cam, w, h };
        let Some(axis) = g.handle_at(&view, [x, y]) else { return false };
        if !g.begin(&view, axis, [x, y]) {
            return false;
        }
        self.tools.gizmo = g;
        self.tools.gizmo_target = Some(d.id);
        true
    }

    /// Moves the drag to pixel `(x, y)` (a preview: no command runs yet).
    pub fn gizmo_drag(&mut self, x: f64, y: f64, w: f64, h: f64) {
        let cam = self.camera.clone();
        self.tools.gizmo.update(&View { camera: &cam, w, h }, [x, y]);
    }

    /// Ends the drag: one `mesh.transform` (one undo step) when the object was moved. Returns
    /// whether a command ran successfully.
    pub fn gizmo_end(&mut self) -> bool {
        let target = self.tools.gizmo_target.take();
        let Some(m) = self.tools.gizmo.finish() else { return false };
        let Some(id) = target else { return false };
        self.run_command("mesh.transform", json!({"object": id.0, "matrix": m})).is_ok()
    }

    /// Esc during a drag: nothing is applied.
    pub fn gizmo_cancel(&mut self) {
        self.tools.gizmo.cancel();
        self.tools.gizmo_target = None;
    }
}

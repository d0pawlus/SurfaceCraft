//! Standard views and their animation, and what the frame draws over the scene (clip plane, gizmo
//! preview, highlighted triangles).
use crate::app::SurfaceCraftApp;
use crate::brep::BrepHit;
use crate::camera::{StandardView, ViewAnim};
use crate::viewport::Overlay;
use std::sync::Arc;
use surfacecraft_engine::DisplayItem;

impl SurfaceCraftApp {
    // -------------------------------------------------------------------- views

    /// Turns the view to a standard view with a short animation.
    pub fn set_standard_view(&mut self, view: StandardView) {
        self.tools.anim = Some(ViewAnim::to_view(&self.camera, view));
    }

    /// Advances a running view animation by `dt` seconds; true while it is still running.
    pub fn tick_view_animation(&mut self, dt: f64) -> bool {
        let Some(mut a) = self.tools.anim.take() else { return false };
        let running = a.step(&mut self.camera, dt);
        if running {
            self.tools.anim = Some(a);
        }
        running
    }
    // ----------------------------------------------------- what the frame draws

    /// Clip plane and the gizmo's preview motion for the frame, given the drawn items.
    pub(crate) fn frame_overlay(&self, items: &[DisplayItem]) -> Overlay {
        let mut o = Overlay { clip: self.tools.section.world_plane(), models: Vec::new() };
        if self.tools.gizmo.dragging()
            && let Some(id) = self.tools.gizmo_target
        {
            for d in items.iter().filter(|d| d.id == id) {
                o.models.push((d.key, self.tools.gizmo.matrix()));
            }
        }
        o
    }

    /// The triangles of `d` to draw highlighted: picks and the triangle selection (mapped onto the
    /// level of detail), the selected faces of a body and the face under the pointer.
    pub(crate) fn highlight_for(&mut self, d: &DisplayItem) -> Option<Arc<Vec<u32>>> {
        let version = self.tools.version;
        let base = match self.tools.drawn.get(&d.id) {
            Some((k, v, h)) if *k == d.key && *v == version => h.clone(),
            _ => {
                let mut orig: Vec<u32> = self.highlights.get(&d.id).map(|h| h.as_ref().clone()).unwrap_or_default();
                if self.tools.selection.object == Some(d.id) {
                    orig.extend(self.tools.selection.tris.iter().copied());
                }
                let mut drawn = d.drawn_triangles(&orig);
                if let Some(topo) = &d.topo
                    && self.tools.brep.body == Some(d.id)
                {
                    drawn.extend(topo.triangles_of_faces(&self.tools.brep.face_list()));
                }
                let h = Arc::new(drawn);
                self.tools.drawn.insert(d.id, (d.key, version, h.clone()));
                h
            }
        };
        let hover_face = match (&d.topo, self.tools.hover) {
            (Some(topo), Some((id, BrepHit::Face(f)))) if id == d.id => Some(topo.triangles_of_faces(&[f])),
            _ => None,
        };
        match hover_face {
            Some(extra) => {
                let mut v = base.as_ref().clone();
                v.extend(extra);
                Some(Arc::new(v))
            }
            None if base.is_empty() => None,
            None => Some(base),
        }
    }
}

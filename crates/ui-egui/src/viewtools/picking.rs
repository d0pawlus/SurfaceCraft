//! Picking B-rep faces and edges of bodies under the pointer.
use super::*;
use crate::app::SurfaceCraftApp;
use crate::brep::{self, BrepHit, BrepSelection, BrepTarget};
use crate::i18n::tf;
use crate::select::View;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_engine::DisplayItem;

impl SurfaceCraftApp {
    // ------------------------------------------------------------ faces and edges

    pub(super) fn brep_hit_at(&mut self, x: f64, y: f64, w: f64, h: f64) -> Option<(ObjectId, BrepHit)> {
        let target = if self.tools.tool == Tool::Edges { BrepTarget::Edges } else { BrepTarget::Faces };
        self.brep_hit_at_target(x, y, w, h, target)
    }

    /// The face or edge (as `target` says) of the nearest body under pixel `(x, y)`.
    pub(crate) fn brep_hit_at_target(
        &mut self,
        x: f64,
        y: f64,
        w: f64,
        h: f64,
        target: BrepTarget,
    ) -> Option<(ObjectId, BrepHit)> {
        let cam = self.camera.clone();
        let bodies: Vec<DisplayItem> = self
            .session
            .display_items()
            .into_iter()
            .filter(|d| d.kind == ObjectKind::Body && d.topo.is_some())
            .collect();
        let mut best: Option<(f64, ObjectId, BrepHit)> = None;
        for d in &bodies {
            let Some(topo) = d.topo.clone() else { continue };
            let bvh = self.bvh_of(d);
            let hit = brep::pick(&topo, bvh, &View { camera: &cam, w, h }, [x, y], target, EDGE_PICK_PX);
            if let Some((hit, score)) = hit
                && best.is_none_or(|(s, _, _)| score < s)
            {
                best = Some((score, d.id, hit));
            }
        }
        best.map(|(_, id, hit)| (id, hit))
    }

    /// A click on the viewport with a face or edge tool: picks (or, with `add`, toggles) the face
    /// or edge under pixel `(x, y)`; a click on nothing clears. Returns whether something was hit.
    pub fn brep_click(&mut self, x: f64, y: f64, w: f64, h: f64, add: bool) -> bool {
        let hit = self.brep_hit_at(x, y, w, h);
        match hit {
            Some((id, hit)) => {
                self.tools.brep.click(id, Some(hit), add);
                self.selected_object = Some(id);
            }
            None => {
                if !add {
                    self.tools.brep = BrepSelection::default();
                }
            }
        }
        self.tools.touch();
        self.refresh_highlights();
        if let (Some(b), false) = (self.tools.brep.body, self.tools.brep.is_empty()) {
            self.status = tf(
                "status.brep_selection",
                &[
                    ("body", &b.to_string()),
                    ("faces", &self.tools.brep.faces.len().to_string()),
                    ("edges", &self.tools.brep.edges.len().to_string()),
                ],
            );
        }
        hit.is_some()
    }
}

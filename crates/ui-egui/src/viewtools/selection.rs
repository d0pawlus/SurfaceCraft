//! Triangle selection: the scan the tools work on, brush, lasso and smart select, and the region from the selection.
use crate::app::SurfaceCraftApp;
use crate::brep::BrepSelection;
use crate::i18n::tf;
use crate::select::{self, Scan, SelectOp, TriSelection, View};
use serde_json::json;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_engine::DisplayItem;
use surfacecraft_geom::Bvh;

impl SurfaceCraftApp {
    // ---------------------------------------------------------------- selection

    /// The scan the selection tools work on: the selected object if it is a mesh, else the mesh
    /// already being selected, else the first visible mesh.
    fn mesh_item(&mut self) -> Option<DisplayItem> {
        let meshes: Vec<DisplayItem> =
            self.session.display_items().into_iter().filter(|d| d.kind == ObjectKind::Mesh).collect();
        let has = |id: ObjectId| meshes.iter().any(|d| d.id == id);
        let want = self
            .selected_object
            .filter(|id| has(*id))
            .or(self.tools.selection.object.filter(|id| has(*id)))
            .or_else(|| meshes.first().map(|d| d.id))?;
        meshes.into_iter().find(|d| d.id == want)
    }

    pub(super) fn bvh_of(&mut self, d: &DisplayItem) -> &Bvh {
        self.bvh_cache.entry(d.pick_key).or_insert_with(|| Bvh::build(&d.pick_mesh))
    }

    fn apply_selection(&mut self, d: &DisplayItem, op: SelectOp, tris: &[u32]) {
        if self.tools.selection.object != Some(d.id) {
            self.tools.selection = TriSelection { object: Some(d.id), tris: Default::default() };
        }
        self.tools.selection.apply(op, tris);
        self.selected_object = Some(d.id);
        self.tools.touch();
        self.refresh_highlights();
        self.status = tf("status.selection", &[("n", &self.tools.selection.tris.len().to_string())]);
    }

    /// Paints (or, with `erase`, unpaints) a brush stroke from `from` to `to` (viewport pixels of a
    /// `w × h` viewport). `from` is `None` for the first dab of a stroke.
    pub fn select_brush(&mut self, from: Option<[f64; 2]>, to: [f64; 2], w: f64, h: f64, erase: bool) {
        let Some(d) = self.mesh_item() else { return };
        let (cam, radius, through) = (self.camera.clone(), self.tools.brush_radius, self.tools.through);
        let centres = select::stroke_points(from.unwrap_or(to), to, radius);
        let scan = Scan { mesh: &d.pick_mesh, bvh: self.bvh_of(&d) };
        let tris = select::brush_select(scan, &View { camera: &cam, w, h }, &centres, radius, through);
        let op = if erase { SelectOp::Subtract } else { SelectOp::Add };
        self.apply_selection(&d, op, &tris);
    }

    /// Lasso: selects the triangles inside the screen polygon `poly`.
    pub fn select_lasso(&mut self, poly: &[[f64; 2]], w: f64, h: f64, op: SelectOp) {
        let Some(d) = self.mesh_item() else { return };
        let (cam, through) = (self.camera.clone(), self.tools.through);
        let scan = Scan { mesh: &d.pick_mesh, bvh: self.bvh_of(&d) };
        let tris = select::lasso_select(scan, &View { camera: &cam, w, h }, poly, through);
        self.apply_selection(&d, op, &tris);
    }

    /// Smart select: the smooth patch around the triangle under pixel `(x, y)`. False when the
    /// click hit no scan.
    pub fn select_smart(&mut self, x: f64, y: f64, w: f64, h: f64, op: SelectOp) -> bool {
        let Some(d) = self.mesh_item() else { return false };
        let (origin, dir) = self.camera.ray(x, y, w, h);
        let hit = self.bvh_of(&d).ray_cast(&origin, &dir);
        let Some((tri, _)) = hit else { return false };
        let tris = select::smart_select(&d.pick_mesh, tri as u32, self.tools.smart_angle);
        self.apply_selection(&d, op, &tris);
        true
    }

    /// Clears the triangle and the face/edge selections.
    pub fn clear_selection(&mut self) {
        self.tools.selection = TriSelection::default();
        self.tools.brep = BrepSelection::default();
        self.tools.touch();
        self.refresh_highlights();
    }

    /// `region.create` from the triangle selection (original triangle ids).
    pub fn create_region_from_selection(&mut self) -> bool {
        let (Some(mesh), false) = (self.tools.selection.object, self.tools.selection.is_empty()) else {
            return false;
        };
        let tris = self.tools.selection.to_vec();
        self.run_command("region.create", json!({"mesh": mesh.0, "triangles": tris})).is_ok()
    }
}

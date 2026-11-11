//! The viewport's interaction tools, wired to the application: triangle selection (brush, lasso,
//! smart select), picking B-rep faces and edges, the move/rotate gizmo, standard views with the
//! view cube, and the section plane.
//!
//! The maths lives in [`crate::select`], [`crate::brep`], [`crate::gizmo`], [`crate::viewcube`]
//! and [`crate::sectionview`] (testable without a GPU); this module holds the state, turns pointer
//! events into those calls and feeds the results into commands (`region.create`,
//! `mesh.transform`, the `edges` / `faces` / `triangles` parameters of the palette).
use crate::app::SurfaceCraftApp;
use crate::brep::{self, BrepHit, BrepSelection, BrepTarget};
use crate::camera::{StandardView, ViewAnim};
use crate::gizmo::{Axis, Gizmo, GizmoMode};
use crate::i18n::{t, tf};
use crate::sectionview::{SectionAxis, SectionPlane};
use crate::select::{self, Scan, SelectOp, TriSelection, View};
use crate::theme::Token;
use crate::viewcube;
use crate::viewport::Overlay;
use serde_json::json;
use std::collections::HashMap;
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_engine::DisplayItem;
use surfacecraft_geom::{Bvh, Point3};

/// How far from an edge a click still picks it (px).
pub const EDGE_PICK_PX: f64 = 7.0;
/// Half-size of the view cube (px).
const CUBE_HALF: f64 = 44.0;
const CUBE_MARGIN: f64 = 14.0;
/// Opacity of the section cap (the cut face) against the accent colour.
const CAP_FILL: f32 = 0.22;
/// Inset (px) and text size of the note about the section cap.
const CAP_NOTE_INSET: f32 = 8.0;
const CAP_NOTE_PX: f32 = 12.0;
/// The near plane of the cap, as a fraction of the camera distance: the cap is cut there, so a
/// triangle reaching behind the eye keeps the part in front (the projection stops much nearer).
const CAP_NEAR_FRACTION: f64 = 1e-3;

/// The cap of `cuts` as one flat fill on the screen, for the viewport `rect`. A triangle with a
/// corner behind the eye is cut at the near plane, not dropped, so the cap stays when the camera is
/// close to it.
fn build_cap_mesh(
    cuts: &[surfacecraft_geom::cap::Cut],
    cam: &crate::camera::Camera,
    rect: egui::Rect,
    fill: egui::Color32,
) -> egui::Mesh {
    let (w, h) = (f64::from(rect.width()), f64::from(rect.height()));
    let near = cam.distance * CAP_NEAR_FRACTION;
    let depth = |p: &surfacecraft_geom::Point3| cam.depth(p);
    let mut mesh = egui::Mesh::default();
    for cut in cuts {
        for tri in &cut.cap.triangles {
            let corners = crate::sectionview::clip_front(tri, &depth, near);
            let screen: Vec<egui::Pos2> = corners
                .iter()
                .filter_map(|p| cam.project(p, w, h))
                .map(|s| egui::pos2(rect.min.x + s[0] as f32, rect.min.y + s[1] as f32))
                .collect();
            let Some((first, rest)) = screen.split_first() else { continue };
            // a fan from the first corner of the clipped polygon
            for pair in rest.windows(2) {
                let base = mesh.vertices.len() as u32;
                for c in [*first, pair[0], pair[1]] {
                    mesh.colored_vertex(c, fill);
                }
                mesh.add_triangle(base, base + 1, base + 2);
            }
        }
    }
    mesh
}

/// The cap as it was last painted. It is reused while the cut, the camera, the viewport and the
/// colour stay the same.
#[derive(Clone, Debug)]
struct CapPaint {
    cuts: Arc<Vec<surfacecraft_geom::cap::Cut>>,
    camera: crate::camera::Camera,
    rect: egui::Rect,
    fill: egui::Color32,
    mesh: Arc<egui::Mesh>,
}

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

    fn bvh_of(&mut self, d: &DisplayItem) -> &Bvh {
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

    // ------------------------------------------------------------ faces and edges

    fn brep_hit_at(&mut self, x: f64, y: f64, w: f64, h: f64) -> Option<(ObjectId, BrepHit)> {
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

    // ------------------------------------------------------------------- gizmo

    /// The mesh the gizmo would move: the selected object when it is a scan.
    pub(crate) fn gizmo_mesh(&mut self) -> Option<DisplayItem> {
        let id = self.selected_object?;
        self.session.display_items().into_iter().find(|d| d.id == id && d.kind == ObjectKind::Mesh)
    }

    fn pivot_of(&mut self, d: &DisplayItem) -> Point3 {
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

    // ------------------------------------------------------------------ section

    /// Enables the section plane at the middle of the scene along `axis`.
    pub fn enable_section(&mut self, axis: SectionAxis) {
        let b = self.session.doc.bbox();
        let mid = if b.is_empty() { 0.0 } else { axis.coord(&b.center()) };
        self.tools.section = SectionPlane { enabled: true, axis, offset: mid, flip: self.tools.section.flip };
    }

    /// The section cut of the visible geometry: outline and cap per mesh (cached until the plane or
    /// a mesh changes).
    pub fn section_cut(&mut self) -> Arc<Vec<surfacecraft_geom::cap::Cut>> {
        let plane = self.tools.section;
        if !plane.enabled {
            return Arc::new(Vec::new());
        }
        let items = self.session.display_items();
        let keys: Vec<usize> = items.iter().map(|d| d.key).collect();
        if let Some((k, p, cuts)) = &self.tools.cut
            && *k == keys
            && *p == plane
        {
            return cuts.clone();
        }
        let meshes: Vec<&surfacecraft_geom::Mesh> = items.iter().map(|d| d.mesh.as_ref()).collect();
        let cuts = Arc::new(plane.cut(&meshes));
        self.tools.cut = Some((keys, plane, cuts.clone()));
        cuts
    }

    /// The cap of `cuts` on the screen, rebuilt only when the cut, the camera, the viewport or the
    /// colour changed since it was last built.
    fn cap_mesh(
        &mut self,
        cuts: &Arc<Vec<surfacecraft_geom::cap::Cut>>,
        cam: &crate::camera::Camera,
        rect: egui::Rect,
        fill: egui::Color32,
    ) -> Arc<egui::Mesh> {
        if let Some(c) = &self.tools.cap_paint
            && Arc::ptr_eq(&c.cuts, cuts)
            && c.camera == *cam
            && c.rect == rect
            && c.fill == fill
        {
            return c.mesh.clone();
        }
        let mesh = Arc::new(build_cap_mesh(cuts, cam, rect, fill));
        self.tools.cap_paint =
            Some(CapPaint { cuts: cuts.clone(), camera: cam.clone(), rect, fill, mesh: mesh.clone() });
        mesh
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

    // ------------------------------------------------------------ pointer input

    fn cube_centre(rect: egui::Rect) -> [f64; 2] {
        let half = CUBE_HALF + CUBE_MARGIN;
        [f64::from(rect.max.x) - half - 4.0, f64::from(rect.min.y) + half + 4.0]
    }

    fn cube_rect(rect: egui::Rect) -> egui::Rect {
        let c = Self::cube_centre(rect);
        let half = (CUBE_HALF + CUBE_MARGIN) as f32;
        egui::Rect::from_center_size(egui::pos2(c[0] as f32, c[1] as f32), egui::Vec2::splat(2.0 * half))
    }

    /// All pointer interaction of the viewport: navigation, the view cube and the active tool.
    pub(crate) fn view_input(&mut self, ui: &mut egui::Ui, resp: &egui::Response, rect: egui::Rect) {
        use egui::PointerButton::{Middle, Primary, Secondary};
        let (w, h) = (f64::from(rect.width()), f64::from(rect.height()));
        let local = |p: egui::Pos2| [f64::from(p.x - rect.min.x), f64::from(p.y - rect.min.y)];
        let mods = ui.input(|i| i.modifiers);
        if self.tools.anim.is_some() {
            let dt = f64::from(ui.input(|i| i.stable_dt));
            if self.tick_view_animation(dt) {
                ui.ctx().request_repaint();
            }
        }
        if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.cancel_interaction();
        }
        // a form field is being picked: clicks fill it, navigation stays
        let picking = self.field_pick.is_some();
        if picking && resp.hovered() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
        }
        self.tools.pointer = resp.hover_pos().map(local);
        let press = ui.input(|i| i.pointer.press_origin());
        let cube = Self::cube_rect(rect);
        let press_in_cube = press.is_some_and(|p| cube.contains(p));

        // pan and zoom: always
        if resp.dragged_by(Secondary) || resp.dragged_by(Middle) {
            let d = resp.drag_delta();
            self.camera.pan(f64::from(d.x), f64::from(d.y), h);
        }
        if resp.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.camera.zoom(f64::from(scroll));
            }
        }
        // the view cube
        if resp.clicked_by(Primary)
            && let Some(p) = resp.interact_pointer_pos()
            && cube.contains(p)
        {
            let faces = viewcube::visible_faces(&self.camera, Self::cube_centre(rect), CUBE_HALF);
            if let Some(v) = viewcube::face_at(&faces, [f64::from(p.x), f64::from(p.y)]) {
                self.set_standard_view(v);
            }
            return;
        }
        if press_in_cube {
            return;
        }

        let alt = mods.alt;
        let primary_drag = resp.dragged_by(Primary);
        let pos = resp.interact_pointer_pos().map(local);
        let tool = if alt || picking { Tool::Navigate } else { self.tools.tool };

        // hover feedback
        if picking {
            if resp.clicked_by(Secondary) {
                self.stop_field_pick();
            } else if !primary_drag {
                match self.tools.pointer {
                    Some(p) => self.field_pick_hover(p, w, h),
                    None => self.field_pick_unhover(),
                }
            }
        } else if !primary_drag {
            self.tools.hover = None;
            self.tools.gizmo_hover = None;
            if let Some(p) = self.tools.pointer {
                match tool {
                    Tool::Faces | Tool::Edges => self.tools.hover = self.brep_hit_at(p[0], p[1], w, h),
                    Tool::Move | Tool::Rotate => self.hover_gizmo(p, w, h),
                    _ => {}
                }
            }
        }

        match tool {
            Tool::Navigate => {
                if primary_drag {
                    let d = resp.drag_delta();
                    self.camera.orbit(f64::from(d.x), f64::from(d.y));
                }
                if picking {
                    // the second click of a double click is not a second pick
                    if resp.clicked()
                        && !resp.double_clicked()
                        && let Some(p) = pos
                    {
                        self.field_pick_click(p[0], p[1], w, h);
                    }
                } else if resp.double_clicked() {
                    self.fit_pending = true;
                } else if resp.clicked()
                    && let Some(p) = pos
                {
                    self.pick_at(p[0], p[1], w, h, mods.shift);
                }
            }
            Tool::Brush => {
                if primary_drag && let Some(p) = pos {
                    let from = self.tools.last_brush;
                    self.select_brush(from, p, w, h, mods.shift);
                    self.tools.last_brush = Some(p);
                } else if resp.clicked()
                    && let Some(p) = pos
                {
                    self.select_brush(None, p, w, h, mods.shift);
                }
                if resp.drag_stopped() {
                    self.tools.last_brush = None;
                }
            }
            Tool::Lasso => {
                if primary_drag
                    && let Some(p) = pos
                    && self.tools.lasso.last().is_none_or(|l| (l[0] - p[0]).hypot(l[1] - p[1]) >= 2.0)
                {
                    self.tools.lasso.push(p);
                }
                if resp.drag_stopped() {
                    let poly = std::mem::take(&mut self.tools.lasso);
                    self.select_lasso(&poly, w, h, modifiers_op(mods.shift, mods.command));
                }
            }
            Tool::Smart => {
                if resp.clicked()
                    && let Some(p) = pos
                {
                    self.select_smart(p[0], p[1], w, h, modifiers_op(mods.shift, mods.command));
                }
            }
            Tool::Faces | Tool::Edges => {
                if resp.clicked()
                    && let Some(p) = pos
                {
                    self.brep_click(p[0], p[1], w, h, mods.shift);
                }
            }
            Tool::Move | Tool::Rotate => {
                if resp.drag_started_by(Primary) {
                    let start = press.map(local).or(pos);
                    self.tools.gizmo_grabbed = start.is_some_and(|s| self.gizmo_begin(s[0], s[1], w, h));
                }
                if primary_drag {
                    if self.tools.gizmo_grabbed {
                        if let Some(p) = pos {
                            self.gizmo_drag(p[0], p[1], w, h);
                        }
                    } else {
                        let d = resp.drag_delta();
                        self.camera.orbit(f64::from(d.x), f64::from(d.y));
                    }
                }
                if resp.drag_stopped() {
                    if self.tools.gizmo_grabbed {
                        self.gizmo_end();
                    }
                    self.tools.gizmo_grabbed = false;
                }
            }
        }
    }

    fn hover_gizmo(&mut self, p: [f64; 2], w: f64, h: f64) {
        let Some(d) = self.gizmo_mesh() else { return };
        let mode = if self.tools.tool == Tool::Rotate { GizmoMode::Rotate } else { GizmoMode::Translate };
        let pivot = self.pivot_of(&d);
        let g = Gizmo::new(mode, pivot);
        let cam = self.camera.clone();
        self.tools.gizmo_hover = g.handle_at(&View { camera: &cam, w, h }, p);
    }

    /// Esc: abandons a gizmo drag or a lasso in progress.
    pub fn cancel_interaction(&mut self) {
        if self.tools.gizmo.dragging() {
            self.gizmo_cancel();
            self.tools.gizmo_grabbed = false;
        }
        self.tools.lasso.clear();
        self.tools.last_brush = None;
        self.stop_field_pick();
    }

    // ----------------------------------------------------------------- painting

    pub(crate) fn token_color(&self, token: Token) -> egui::Color32 {
        self.tokens.color(token)
    }

    /// Paints the tool overlays and the toolbar on top of the viewport.
    pub(crate) fn paint_view_overlays(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        let painter = ui.painter_at(rect);
        let (w, h) = (f64::from(rect.width()), f64::from(rect.height()));
        let cam = self.camera.clone();
        let to_screen = |p: [f64; 3]| egui::pos2(rect.min.x + p[0] as f32, rect.min.y + p[1] as f32);
        let line = |painter: &egui::Painter, pts: &[Point3], stroke: egui::Stroke| {
            let mut run: Vec<egui::Pos2> = Vec::new();
            for p in pts {
                match cam.project(p, w, h) {
                    Some(s) => run.push(to_screen(s)),
                    None => {
                        if run.len() > 1 {
                            painter.add(egui::Shape::line(std::mem::take(&mut run), stroke));
                        }
                        run.clear();
                    }
                }
            }
            if run.len() > 1 {
                painter.add(egui::Shape::line(run, stroke));
            }
        };

        // section: the cap (where the plane is seen from the removed side), the outline, and the
        // open outlines that have no cap
        if self.tools.section.enabled {
            let accent = self.token_color(Token::AccentPrimary);
            let stroke = egui::Stroke::new(2.0, accent);
            let cuts = self.section_cut();
            if self.tools.section.cap_seen_from(&cam.eye()) {
                let mesh = self.cap_mesh(&cuts, &cam, rect, accent.gamma_multiply(CAP_FILL));
                if !mesh.indices.is_empty() {
                    painter.add(egui::Shape::Mesh(mesh));
                }
            }
            for pl in cuts.iter().flat_map(|c| c.lines.iter()) {
                let mut pts = pl.points.clone();
                if pl.closed
                    && let Some(first) = pts.first().copied()
                {
                    pts.push(first);
                }
                line(&painter, &pts, stroke);
            }
            let open: usize = cuts.iter().map(|c| c.cap.open_loops).sum();
            let failed: usize = cuts.iter().map(|c| c.cap.failed_loops).sum();
            let too_large = cuts.iter().any(|c| c.cap.over_limit);
            if too_large || open + failed > 0 {
                let text = if too_large {
                    t("section.cap_too_large").to_string()
                } else {
                    tf("section.cap_note", &[("open", &open.to_string()), ("failed", &failed.to_string())])
                };
                let at = rect.min + egui::vec2(CAP_NOTE_INSET, CAP_NOTE_INSET);
                painter.text(at, egui::Align2::LEFT_TOP, text, egui::FontId::proportional(CAP_NOTE_PX), accent);
            }
        }

        // selected and hovered edges of bodies
        let bodies: Vec<DisplayItem> = self
            .session
            .display_items()
            .into_iter()
            .filter(|d| d.kind == ObjectKind::Body && d.topo.is_some())
            .collect();
        let (sel_body, sel_edges) = self.highlighted_edges();
        for d in &bodies {
            let Some(topo) = &d.topo else { continue };
            if sel_body == Some(d.id) {
                let stroke = egui::Stroke::new(3.0, self.token_color(Token::Highlight));
                for &e in &sel_edges {
                    if let Some(pl) = topo.edges.get(e) {
                        line(&painter, pl, stroke);
                    }
                }
            }
            if let Some((id, BrepHit::Edge(e))) = self.tools.hover
                && id == d.id
                && let Some(pl) = topo.edges.get(e)
            {
                line(&painter, pl, egui::Stroke::new(3.0, self.token_color(Token::FocusRing)));
            }
        }

        self.paint_field_marks(&painter, rect);

        // brush cursor and lasso outline
        if self.tools.tool == Tool::Brush
            && let Some(p) = self.tools.pointer
        {
            painter.circle_stroke(
                egui::pos2(rect.min.x + p[0] as f32, rect.min.y + p[1] as f32),
                self.tools.brush_radius as f32,
                egui::Stroke::new(1.5, self.token_color(Token::Highlight)),
            );
        }
        if self.tools.lasso.len() > 1 {
            let mut pts: Vec<egui::Pos2> = self
                .tools
                .lasso
                .iter()
                .map(|p| egui::pos2(rect.min.x + p[0] as f32, rect.min.y + p[1] as f32))
                .collect();
            if let Some(f) = pts.first().copied() {
                pts.push(f);
            }
            painter.add(egui::Shape::line(pts, egui::Stroke::new(1.5, self.token_color(Token::Highlight))));
        }

        self.paint_gizmo(&painter, rect);
        self.paint_view_cube(ui, &painter, rect);
        self.tool_strip(ui.ctx(), rect);
    }

    /// The points clicked for a form field (numbered) and the one under the pointer.
    fn paint_field_marks(&self, painter: &egui::Painter, rect: egui::Rect) {
        let (marks, hover) = self.field_pick_marks();
        let (w, h) = (f64::from(rect.width()), f64::from(rect.height()));
        let at = |p: [f64; 3]| {
            self.camera
                .project(&Point3::new(p[0], p[1], p[2]), w, h)
                .map(|s| egui::pos2(rect.min.x + s[0] as f32, rect.min.y + s[1] as f32))
        };
        let accent = self.token_color(Token::Highlight);
        for (i, p) in marks.iter().enumerate() {
            if let Some(c) = at(*p) {
                painter.circle_filled(c, 5.0, accent);
                painter.circle_stroke(c, 5.0, egui::Stroke::new(1.5, self.token_color(Token::TextPrimary)));
                painter.text(
                    c + egui::vec2(8.0, -8.0),
                    egui::Align2::LEFT_BOTTOM,
                    (i + 1).to_string(),
                    egui::FontId::proportional(13.0),
                    self.token_color(Token::TextPrimary),
                );
            }
        }
        if let Some(c) = hover.and_then(at) {
            painter.circle_stroke(c, 7.0, egui::Stroke::new(1.5, self.token_color(Token::FocusRing)));
        }
    }

    fn paint_gizmo(&mut self, painter: &egui::Painter, rect: egui::Rect) {
        if !matches!(self.tools.tool, Tool::Move | Tool::Rotate) {
            return;
        }
        let Some(d) = self.gizmo_mesh() else { return };
        let (w, h) = (f64::from(rect.width()), f64::from(rect.height()));
        let mode = if self.tools.tool == Tool::Rotate { GizmoMode::Rotate } else { GizmoMode::Translate };
        let dragging = self.tools.gizmo.dragging();
        let g = if dragging { self.tools.gizmo.clone() } else { Gizmo::new(mode, self.pivot_of(&d)) };
        let cam = self.camera.clone();
        let view = View { camera: &cam, w, h };
        let centre = g.transform().apply(&g.center);
        let len = g.handle_length(&view);
        let to_screen = |p: [f64; 3]| egui::pos2(rect.min.x + p[0] as f32, rect.min.y + p[1] as f32);
        for axis in Axis::ALL {
            let token = match axis {
                Axis::X => Token::AxisX,
                Axis::Y => Token::AxisY,
                Axis::Z => Token::AxisZ,
            };
            let active = g.active_axis() == Some(axis) || (!dragging && self.tools.gizmo_hover == Some(axis));
            let color = self.token_color(if active { Token::Highlight } else { token });
            let stroke = egui::Stroke::new(if active { 4.0 } else { 3.0 }, color);
            match g.mode {
                GizmoMode::Translate => {
                    let tip = centre + axis.dir() * len;
                    if let (Some(a), Some(b)) = (cam.project(&centre, w, h), cam.project(&tip, w, h)) {
                        let (a, b) = (to_screen(a), to_screen(b));
                        painter.line_segment([a, b], stroke);
                        let dir = (b - a).normalized();
                        let side = egui::vec2(-dir.y, dir.x) * 5.0;
                        let back = b - dir * 12.0;
                        painter.add(egui::Shape::convex_polygon(vec![b, back + side, back - side], color, stroke));
                    }
                }
                GizmoMode::Rotate => {
                    let mut moved = g.clone();
                    moved.center = centre;
                    let ring = moved.ring(axis, len * 0.8);
                    let pts: Vec<egui::Pos2> =
                        ring.iter().filter_map(|p| cam.project(p, w, h)).map(to_screen).collect();
                    if pts.len() > 1 {
                        painter.add(egui::Shape::line(pts, stroke));
                    }
                }
            }
        }
    }

    fn paint_view_cube(&mut self, ui: &egui::Ui, painter: &egui::Painter, rect: egui::Rect) {
        let centre = Self::cube_centre(rect);
        let faces = viewcube::visible_faces(&self.camera, centre, CUBE_HALF);
        let hover = ui
            .input(|i| i.pointer.hover_pos())
            .filter(|p| rect.contains(*p))
            .and_then(|p| viewcube::face_at(&faces, [f64::from(p.x), f64::from(p.y)]));
        let font = egui::FontId::proportional(11.0);
        for f in &faces {
            let pts: Vec<egui::Pos2> = f.quad.iter().map(|q| egui::pos2(q[0] as f32, q[1] as f32)).collect();
            let fill =
                self.token_color(if hover == Some(f.view) { Token::SurfaceActive } else { Token::SurfaceRaised });
            painter.add(egui::Shape::convex_polygon(
                pts,
                fill,
                egui::Stroke::new(1.0, self.token_color(Token::BorderStrong)),
            ));
            let mid =
                egui::pos2(((f.quad[0][0] + f.quad[2][0]) / 2.0) as f32, ((f.quad[0][1] + f.quad[2][1]) / 2.0) as f32);
            painter.text(
                mid,
                egui::Align2::CENTER_CENTER,
                t(f.view.key()),
                font.clone(),
                self.token_color(Token::TextPrimary),
            );
        }
    }
}

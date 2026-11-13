//! The section plane: its cut of the visible meshes, and the cap painted on the cut.
use crate::app::SurfaceCraftApp;
use crate::sectionview::{SectionAxis, SectionPlane};
use std::sync::Arc;

/// Opacity of the section cap (the cut face) against the accent colour.
pub(super) const CAP_FILL: f32 = 0.22;
/// Inset (px) and text size of the note about the section cap.
pub(super) const CAP_NOTE_INSET: f32 = 8.0;
pub(super) const CAP_NOTE_PX: f32 = 12.0;
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
pub(super) struct CapPaint {
    cuts: Arc<Vec<surfacecraft_geom::cap::Cut>>,
    camera: crate::camera::Camera,
    rect: egui::Rect,
    fill: egui::Color32,
    mesh: Arc<egui::Mesh>,
}
impl SurfaceCraftApp {
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
    pub(super) fn cap_mesh(
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
}

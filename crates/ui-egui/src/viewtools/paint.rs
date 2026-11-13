//! Painting of the viewport overlays: the section cap and outline, highlighted edges, the field
//! marks, the brush and lasso, the gizmo and the view cube.
use super::input::*;
use super::section::*;
use super::*;
use crate::app::SurfaceCraftApp;
use crate::brep::BrepHit;
use crate::gizmo::{Axis, Gizmo, GizmoMode};
use crate::i18n::{t, tf};
use crate::select::View;
use crate::theme::Token;
use crate::viewcube;
use surfacecraft_doc::ObjectKind;
use surfacecraft_engine::DisplayItem;
use surfacecraft_geom::Point3;

impl SurfaceCraftApp {
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
        crate::deviation_panel::readout::paint(self, ui, rect);

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

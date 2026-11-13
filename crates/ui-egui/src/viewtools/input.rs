//! The pointer input of the viewport: navigation, the view cube and the active tool's press, drag and release.
use super::*;
use crate::app::SurfaceCraftApp;
use crate::gizmo::{Gizmo, GizmoMode};
use crate::select::View;
use crate::viewcube;

/// Half-size of the view cube (px).
pub(super) const CUBE_HALF: f64 = 44.0;
const CUBE_MARGIN: f64 = 14.0;
impl SurfaceCraftApp {
    // ------------------------------------------------------------ pointer input

    pub(super) fn cube_centre(rect: egui::Rect) -> [f64; 2] {
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
                    crate::deviation_panel::readout::pin_click(self, p[0], p[1], w, h, mods.alt);
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
}

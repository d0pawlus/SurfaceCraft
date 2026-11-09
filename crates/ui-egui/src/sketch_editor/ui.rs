//! The egui side of the sketch editor: pointer and keyboard in, the 2-D overlay and the editor's
//! controls out. All geometry decisions are in the state machine of the parent module; this file
//! only maps pixels to sketch coordinates ([`PlaneView`]) and runs the commands the machine
//! returns. Colours are theme tokens and texts catalogue keys.
use super::snap::{self, Hit, SnapKind, Want, snap_point};
use super::view::{PlaneView, align_camera, sketch_bounds};
use super::{
    Cmd, SketchEditor, Tool, constraint_anchors, constraint_entities, constraint_mark, fmt_value, parse_value,
    status_view,
};
use crate::app::SurfaceCraftApp;
use crate::i18n::{self, t, tf};
use crate::theme::Token;
use egui::{FontId, Pos2, Rect, Stroke, Vec2};
use serde_json::{Value, json};
use surfacecraft_doc::ObjectId;
use surfacecraft_sketch::{Constraint, Sketch, SolveReport};

/// Pick and snap radius in pixels.
const PICK_PX: f64 = 10.0;
/// Closest grid nodes in pixels.
const GRID_PX: f64 = 14.0;

fn rect_of(r: [f32; 4]) -> Rect {
    Rect::from_min_max(Pos2::new(r[0], r[1]), Pos2::new(r[2], r[3]))
}

impl SurfaceCraftApp {
    /// Is a sketch being edited (and does it still exist)?
    #[must_use]
    pub fn sketch_editing(&self) -> bool {
        self.sketch_editor.target.is_some_and(|id| self.session.doc.sketch(id).is_some())
    }

    fn current_sketch(&self) -> Option<(ObjectId, String, Sketch, Option<SolveReport>)> {
        let id = self.sketch_editor.target?;
        let o = self.session.doc.sketch(id)?;
        Some((id, o.name.clone(), o.sketch.clone(), o.report.clone()))
    }

    /// Enters the editing mode on sketch `id`: the camera looks straight onto its plane.
    pub fn begin_sketch_edit(&mut self, id: ObjectId) {
        let Some(o) = self.session.doc.sketch(id) else {
            self.status = t("sketch.notice.no_sketch").to_string();
            return;
        };
        let (centre, extent) = sketch_bounds(&o.sketch);
        let plane = o.sketch.plane.clone();
        let name = o.name.clone();
        align_camera(&mut self.camera, &plane, centre, extent.max(40.0));
        self.sketch_editor.begin(id);
        self.selected_object = Some(id);
        self.status = tf("sketch.status.editing", &[("name", &name)]);
    }

    /// Leaves the editing mode.
    pub fn end_sketch_edit(&mut self) {
        self.sketch_editor.end();
        self.status = t("status.ready").to_string();
    }

    /// Creates a sketch on the plane with `normal` through the origin and starts editing it.
    pub fn new_sketch_on(&mut self, normal: [f64; 3]) {
        let params = json!({"plane": {"origin": [0.0, 0.0, 0.0], "normal": normal}}); // not a colour
        if let Ok(v) = self.run_command("sketch.create", params)
            && let Some(id) = v.get("id").and_then(Value::as_u64)
        {
            self.begin_sketch_edit(ObjectId(id));
        }
    }

    /// The entries of the Sketch menu that belong to the editor.
    pub(crate) fn sketch_menu_items(&mut self, ui: &mut egui::Ui) {
        for (key, normal) in [
            ("sketch.menu.new_xy", [0.0, 0.0, 1.0]), // not a colour
            ("sketch.menu.new_xz", [0.0, -1.0, 0.0]),
            ("sketch.menu.new_yz", [1.0, 0.0, 0.0]), // not a colour
        ] {
            if ui.button(t(key)).clicked() {
                self.new_sketch_on(normal);
                ui.close();
            }
        }
        let selected = self.selected_object.filter(|id| self.session.doc.sketch(*id).is_some());
        if ui.add_enabled(selected.is_some(), egui::Button::new(t("sketch.menu.edit"))).clicked()
            && let Some(id) = selected
        {
            self.begin_sketch_edit(id);
            ui.close();
        }
        if ui.add_enabled(self.sketch_editing(), egui::Button::new(t("sketch.finish"))).clicked() {
            self.end_sketch_edit();
            ui.close();
        }
        ui.separator();
    }

    fn run_sketch_cmds(&mut self, cmds: Vec<Cmd>) {
        for c in cmds {
            let _ = self.run_command(c.id, c.params);
        }
    }

    /// Pointer and keyboard of the editing mode, called by the viewport panel in place of the
    /// camera-orbit and picking gestures (panning with the right/middle button and zooming
    /// with the wheel stay the viewport's).
    pub(crate) fn sketch_input(&mut self, ui: &mut egui::Ui, resp: &egui::Response, rect: Rect) {
        let Some((_, _, sk, _)) = self.current_sketch() else {
            self.sketch_editor.end();
            return;
        };
        let view = PlaneView::new(&self.camera, &sk.plane, rect.width(), rect.height());
        let upp = view.units_per_px();
        let tol = PICK_PX * upp;
        let grid = snap::grid_step(upp, GRID_PX);
        let keyboard_free = !ui.ctx().egui_wants_keyboard_input();
        let ed = &mut self.sketch_editor;
        ed.sanitize(&sk);
        let over_chrome = |p: Pos2, ed: &SketchEditor| ed.chrome.iter().any(|r| rect_of(*r).contains(p));
        let to_plane = |p: Pos2| view.to_plane(p.x - rect.min.x, p.y - rect.min.y);
        ed.hover = resp
            .hover_pos()
            .filter(|p| !over_chrome(*p, ed))
            .and_then(to_plane)
            .map(|q| snap_point(&sk, q, ed.anchor(), &ed.gesture, tol, grid, &ed.snap));

        let mut cmds: Vec<Cmd> = Vec::new();
        let (esc, enter, events) =
            ui.input(|i| (i.key_pressed(egui::Key::Escape), i.key_pressed(egui::Key::Enter), i.events.clone()));
        if esc {
            ed.escape();
        }
        if keyboard_free && ed.pending.is_none() && ed.anchor().is_some() {
            // dynamic input: digits typed while drawing set the next length (or radius)
            for e in &events {
                match e {
                    egui::Event::Text(s) => {
                        ed.typed.extend(s.chars().filter(|c| c.is_ascii_digit() || *c == '.' || *c == ','))
                    }
                    egui::Event::Key { key: egui::Key::Backspace, pressed: true, .. } => {
                        ed.typed.pop();
                    }
                    _ => {}
                }
            }
        }
        if enter && keyboard_free && ed.pending.is_none() {
            if let (false, Some(v)) = (ed.typed.is_empty(), parse_value(&ed.typed.clone())) {
                let cursor = ed.hover.map_or([1.0, 0.0], |h| h.pos);
                cmds.extend(ed.type_value(&sk, v, cursor));
            } else if ed.typed.is_empty() {
                cmds.extend(ed.finish());
            } else {
                ed.typed.clear();
                ed.notice = Some("sketch.notice.value");
            }
        }
        if resp.clicked_by(egui::PointerButton::Primary)
            && let Some(pos) = resp.interact_pointer_pos()
            && !over_chrome(pos, ed)
        {
            if let Some(i) = ed.dimension_at(pos.x - rect.min.x, pos.y - rect.min.y) {
                if resp.double_clicked() {
                    ed.begin_edit(&sk, i);
                }
            } else if let Some(q) = to_plane(pos) {
                let at = snap_point(&sk, q, ed.anchor(), &ed.gesture, tol, grid, &ed.snap);
                cmds.extend(ed.click(&sk, at, tol));
            }
        }
        if resp.clicked_by(egui::PointerButton::Secondary) {
            if ed.tool == Tool::Polyline && ed.gesture.len() >= 2 {
                cmds.extend(ed.finish());
            } else {
                ed.escape();
            }
        }
        self.run_sketch_cmds(cmds);
    }

    /// The 2-D overlay of the editing mode over the viewport: grid, reference sketches, the
    /// sketch with its constraints and dimensions, the rubber band, the snap marker and the
    /// editor's controls.
    pub(crate) fn sketch_overlay(&mut self, ui: &mut egui::Ui, rect: Rect) {
        let Some((_, name, sk, report)) = self.current_sketch() else { return };
        let view = PlaneView::new(&self.camera, &sk.plane, rect.width(), rect.height());
        let tk = self.tokens.clone();
        let painter = ui.painter().with_clip_rect(rect);
        let at = |q: [f32; 2]| Pos2::new(rect.min.x + q[0], rect.min.y + q[1]);
        let sp = |p: [f64; 2]| view.to_screen(p).map(at);
        let line = |painter: &egui::Painter, pts: &[Pos2], width: f32, token: Token| {
            if pts.len() >= 2 {
                painter.add(egui::Shape::line(pts.to_vec(), Stroke::new(width, tk.color(token))));
            }
        };
        let upp = view.units_per_px();
        let ed_snap = self.sketch_editor.snap;

        // grid nodes
        if ed_snap.grid {
            let step = snap::grid_step(upp, GRID_PX);
            let corners: Vec<[f64; 2]> =
                [(0.0, 0.0), (rect.width(), 0.0), (rect.width(), rect.height()), (0.0, rect.height())]
                    .iter()
                    .filter_map(|(x, y)| view.to_plane(*x, *y))
                    .collect();
            if corners.len() == 4 {
                let lo = [0, 1].map(|k| corners.iter().map(|c| c[k]).fold(f64::INFINITY, f64::min));
                let hi = [0, 1].map(|k| corners.iter().map(|c| c[k]).fold(f64::NEG_INFINITY, f64::max));
                let (nx, ny) = (((hi[0] - lo[0]) / step).ceil(), ((hi[1] - lo[1]) / step).ceil());
                if nx * ny <= 9000.0 {
                    let (x0, y0) = ((lo[0] / step).floor(), (lo[1] / step).floor());
                    for i in 0..=(nx as i64 + 1) {
                        for j in 0..=(ny as i64 + 1) {
                            let p = [(x0 + i as f64) * step, (y0 + j as f64) * step];
                            if let Some(s) = sp(p) {
                                painter.circle_filled(s, 0.9, tk.color(Token::BorderSubtle));
                            }
                        }
                    }
                }
            }
        }

        // reference geometry: the other sketches (sections of the scan, say), through the camera
        for o in &self.session.doc.sketches {
            if Some(o.id) == self.sketch_editor.target || !o.visible {
                continue;
            }
            for e in 0..o.sketch.entities.len() {
                let pts: Vec<Pos2> = snap::entity_polyline(&o.sketch, e, 48)
                    .iter()
                    .filter_map(|p| view.project(&o.sketch.plane.to_3d(*p)).map(at))
                    .collect();
                line(&painter, &pts, 1.0, Token::TextMuted);
            }
        }

        let ed = &mut self.sketch_editor;
        let status = status_view(report.as_ref());
        let bad: Vec<usize> = status
            .conflicting
            .iter()
            .flat_map(|i| sk.constraints.get(*i).map(constraint_entities).unwrap_or_default())
            .collect();
        let hover_hit = ed.hover.filter(|_| !ed.tool.draws()).and_then(|h| {
            snap::pick(
                &sk,
                h.pos,
                PICK_PX * upp,
                if ed.tool == Tool::Select { Want::PointOrEntity } else { ed.tool.want() },
            )
        });

        // the sketch
        for e in 0..sk.entities.len() {
            let pts: Vec<Pos2> = snap::entity_polyline(&sk, e, 64).iter().filter_map(|p| sp(*p)).collect();
            let token = if bad.contains(&e) {
                Token::StatusError
            } else if ed.picks.contains(&Hit::Entity(e)) || hover_hit == Some(Hit::Entity(e)) {
                Token::Highlight
            } else {
                Token::ObjectSketch
            };
            line(&painter, &pts, 2.0, token);
        }
        for (i, p) in sk.points.iter().enumerate() {
            if let Some(s) = sp(*p) {
                let lit = ed.picks.contains(&Hit::Point(i)) || hover_hit == Some(Hit::Point(i));
                painter.circle_filled(
                    s,
                    if lit { 5.0 } else { 3.0 },
                    tk.color(if lit { Token::Highlight } else { Token::ObjectSketch }),
                );
            }
        }

        // constraints and dimensions
        ed.labels.clear();
        let font = FontId::proportional(12.0);
        for (i, c) in sk.constraints.iter().enumerate() {
            let token = if status.conflicting.contains(&i) {
                Token::StatusError
            } else if status.redundant.contains(&i) {
                Token::StatusWarning
            } else if super::DimKind::of(c).is_some() {
                Token::TextPrimary
            } else {
                Token::TextMuted
            };
            let dim = super::DimKind::of(c).is_some();
            for a in constraint_anchors(&sk, c) {
                let Some(s) = sp(a) else { continue };
                let galley = painter.layout_no_wrap(constraint_mark(c), font.clone(), tk.color(token));
                let pos = s + Vec2::new(8.0, -8.0 - galley.size().y);
                let r = Rect::from_min_size(pos, galley.size()).expand(2.0);
                if dim {
                    painter.rect_filled(r, 3.0, tk.color(Token::SurfaceRaised));
                    painter.rect_stroke(r, 3.0, Stroke::new(1.0, tk.color(token)), egui::StrokeKind::Inside);
                    ed.labels.push((
                        i,
                        [r.min.x - rect.min.x, r.min.y - rect.min.y, r.max.x - rect.min.x, r.max.y - rect.min.y],
                    ));
                }
                painter.galley(pos, galley, tk.color(token));
            }
        }

        // rubber band, snap marker and live read-out
        if let Some(h) = ed.hover {
            let accent = tk.color(Token::AccentPrimary);
            for pts in ed.preview(h.pos) {
                let pts: Vec<Pos2> = pts.iter().filter_map(|p| sp(*p)).collect();
                line(&painter, &pts, 1.5, Token::AccentPrimary);
            }
            for g in &ed.gesture {
                if let Some(s) = sp(*g) {
                    painter.circle_filled(s, 3.5, accent);
                }
            }
            if let Some(s) = sp(h.pos) {
                if matches!(h.kind, SnapKind::Horizontal | SnapKind::Vertical)
                    && let Some(a) = ed.anchor().and_then(sp)
                {
                    line(&painter, &[a, s], 1.0, Token::AccentPrimary);
                }
                match h.kind {
                    SnapKind::Midpoint => {
                        painter.rect_stroke(
                            Rect::from_center_size(s, Vec2::splat(9.0)),
                            0.0,
                            Stroke::new(1.5, accent),
                            egui::StrokeKind::Middle,
                        );
                    }
                    SnapKind::Free => {}
                    SnapKind::Grid => {
                        painter.circle_filled(s, 2.0, accent);
                    }
                    _ => {
                        painter.circle_stroke(s, 5.5, Stroke::new(1.5, accent));
                    }
                }
                let mut text = String::new();
                if h.kind != SnapKind::Free && h.kind != SnapKind::Grid {
                    text.push_str(t(h.kind.label_key()));
                }
                if let Some(r) = ed.readout(h.pos) {
                    let key = if r.radius { "sketch.readout.radius" } else { "sketch.readout.length" };
                    let v = if ed.typed.is_empty() { fmt_value(r.length) } else { format!("{}|", ed.typed) };
                    if !text.is_empty() {
                        text.push_str("  ");
                    }
                    text.push_str(&tf(key, &[("value", &v), ("angle", &fmt_value(r.angle))]));
                }
                if !text.is_empty() {
                    let galley = painter.layout_no_wrap(text, font.clone(), tk.color(Token::TextPrimary));
                    let pos = s + Vec2::new(12.0, 12.0);
                    painter.rect_filled(
                        Rect::from_min_size(pos, galley.size()).expand(3.0),
                        3.0,
                        tk.color(Token::SurfaceRaised),
                    );
                    painter.galley(pos, galley, tk.color(Token::TextPrimary));
                }
            }
        }

        // controls
        let mut cmds: Vec<Cmd> = Vec::new();
        let mut finish = false;
        let mut refit = false;
        let chrome_rect = Rect::from_min_size(
            rect.min + Vec2::splat(8.0),
            Vec2::new((rect.width() - 16.0).clamp(120.0, 560.0), (rect.height() - 16.0).max(60.0)),
        );
        let mut child = ui.new_child(egui::UiBuilder::new().max_rect(chrome_rect));
        let shown = egui::Frame::NONE
            .fill(tk.color(Token::SurfaceRaised))
            .stroke(Stroke::new(1.0, tk.color(Token::BorderSubtle)))
            .corner_radius(6.0)
            .inner_margin(egui::Margin::same(8))
            .show(&mut child, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.strong(tf("sketch.title", &[("name", &name)]));
                    if ui.button(t("sketch.fit")).clicked() {
                        refit = true;
                    }
                    if ui.button(t("sketch.finish")).clicked() {
                        finish = true;
                    }
                });
                ui.horizontal_wrapped(|ui| {
                    for tool in std::iter::once(Tool::Select).chain(Tool::DRAW) {
                        if ui.selectable_label(ed.tool == tool, t(tool.label_key())).clicked() {
                            ed.set_tool(tool);
                        }
                    }
                });
                ui.horizontal_wrapped(|ui| {
                    for tool in Tool::CONSTRAIN {
                        if ui.selectable_label(ed.tool == tool, t(tool.label_key())).clicked() {
                            ed.set_tool(tool);
                        }
                    }
                });
                ui.horizontal_wrapped(|ui| {
                    ui.checkbox(&mut ed.snap.points, t("sketch.snap.points"));
                    ui.checkbox(&mut ed.snap.midpoints, t("sketch.snap.midpoints"));
                    ui.checkbox(&mut ed.snap.inference, t("sketch.snap.inference"));
                    ui.checkbox(&mut ed.snap.grid, t("sketch.snap.grid_on"));
                });
                ui.weak(t(ed.tool.hint_key()));
                let col = tk.color(status.token);
                ui.label(
                    egui::RichText::new(tf(
                        status.key,
                        &[("dof", &status.dof.to_string()), ("n", &status.conflicting.len().to_string())],
                    ))
                    .color(col)
                    .strong(),
                );
                if !status.conflicting.is_empty() {
                    let list: Vec<String> = status.conflicting.iter().map(|i| format!("#{i}")).collect();
                    ui.label(
                        egui::RichText::new(tf("sketch.status.conflicting", &[("list", &list.join(", "))]))
                            .color(tk.color(Token::StatusError)),
                    );
                }
                if let Some(msg) = report
                    .as_ref()
                    .filter(|r| r.status == surfacecraft_sketch::SolveStatus::Failed)
                    .map(|r| r.message.clone())
                {
                    ui.weak(msg);
                }
                if let Some(n) = ed.notice {
                    ui.label(egui::RichText::new(t(n)).color(tk.color(Token::StatusError)));
                }
                if let Some(p) = ed.pending.as_mut() {
                    ui.separator();
                    let mut commit = false;
                    let mut delete = None;
                    ui.horizontal(|ui| {
                        let l = ui.label(t(p.kind.label_key()));
                        let r = ui.add(egui::TextEdit::singleline(&mut p.text).desired_width(90.0)).labelled_by(l.id);
                        if r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                            commit = true;
                        } else if !r.has_focus() {
                            r.request_focus();
                        }
                        if ui.button(t("sketch.dim.apply")).clicked() {
                            commit = true;
                        }
                        if let Some(i) = p.index
                            && ui.button(t("sketch.dim.delete")).clicked()
                        {
                            delete = Some(i);
                        }
                    });
                    if commit {
                        cmds.extend(ed.commit_pending());
                    }
                    if let Some(i) = delete {
                        ed.pending = None;
                        cmds.extend(ed.delete_constraint(i));
                    }
                }
                egui::CollapsingHeader::new(tf("sketch.constraints", &[("n", &sk.constraints.len().to_string())]))
                    .id_salt("sketch-constraints")
                    .show(ui, |ui| {
                        egui::ScrollArea::vertical().id_salt("sketch-constraint-list").max_height(140.0).show(
                            ui,
                            |ui| {
                                for (i, c) in sk.constraints.iter().enumerate() {
                                    let token = if status.conflicting.contains(&i) {
                                        Token::StatusError
                                    } else if status.redundant.contains(&i) {
                                        Token::StatusWarning
                                    } else {
                                        Token::TextPrimary
                                    };
                                    ui.horizontal(|ui| {
                                        let kind = constraint_kind_name(c);
                                        let text = format!("#{i} {} {}", kind, constraint_mark(c));
                                        let r = ui.add(
                                            egui::Label::new(egui::RichText::new(text).color(tk.color(token)))
                                                .sense(egui::Sense::click()),
                                        );
                                        if r.clicked() {
                                            ed.begin_edit(&sk, i);
                                        }
                                        if ui.small_button("x").on_hover_text(t("sketch.dim.delete")).clicked() {
                                            cmds.extend(ed.delete_constraint(i));
                                        }
                                    });
                                }
                            },
                        );
                    });
            });
        ed.chrome = vec![[
            shown.response.rect.min.x,
            shown.response.rect.min.y,
            shown.response.rect.max.x,
            shown.response.rect.max.y,
        ]];
        if refit {
            let (centre, extent) = sketch_bounds(&sk);
            align_camera(&mut self.camera, &sk.plane, centre, extent.max(40.0));
        }
        if finish {
            self.end_sketch_edit();
        }
        self.run_sketch_cmds(cmds);
    }
}

/// The translated name of a constraint's kind (the serde tag as the key's tail).
fn constraint_kind_name(c: &Constraint) -> String {
    let tag = serde_json::to_value(c)
        .ok()
        .and_then(|v| v.get("type").and_then(Value::as_str).map(str::to_string))
        .unwrap_or_default();
    i18n::find(&format!("sketch.c.{tag}")).map_or(tag.clone(), str::to_string)
}

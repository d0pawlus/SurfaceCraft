//! The navigation bar of the viewport: a card at the bottom centre with the pointer tools, the
//! section plane and the views as icon buttons, and in a second card above it the settings of the
//! active tool and what is selected. The tools themselves are in [`crate::viewtools`].
use crate::app::SurfaceCraftApp;
use crate::camera::StandardView;
use crate::i18n::{t, tf};
use crate::sectionview::SectionAxis;
use crate::theme::Token;
use crate::viewtools::Tool;

/// Gap between the navigation bar and the viewport's edge (px).
const NAV_MARGIN: f32 = 12.0;
/// Gap between two buttons of the bar (px).
const NAV_SPACING: f32 = 2.0;
/// Buttons of the bar that are not pointer tools: the section plane and the two views.
const VIEW_BUTTONS: usize = 3;
/// Separators of the one-row bar, and the width one takes with its spacing (px).
const SEPARATORS: f32 = 2.0;
const SEPARATOR_WIDTH: f32 = 10.0;
/// Gap between the navigation bar and the card of the active tool's settings above it (px).
const OPTIONS_GAP: f32 = 6.0;

impl SurfaceCraftApp {
    /// The navigation bar at the bottom centre of the viewport: the pointer tools, the section
    /// plane and the views, as icons. What the active tool can be set to (brush size, section
    /// offset, the count of what is selected) is a card of its own above the bar, so the buttons
    /// stay where they are whichever tool is active.
    pub(crate) fn tool_strip(&mut self, ctx: &egui::Context, rect: egui::Rect) {
        let frame = crate::theme::card_frame(&self.tokens);
        let max_width = (rect.width() - 2.0 * NAV_MARGIN - 16.0).max(160.0);
        let card = |id: &'static str, bottom: f32| {
            egui::Area::new(egui::Id::unique(id))
                .pivot(egui::Align2::CENTER_BOTTOM)
                .fixed_pos(egui::pos2(rect.center().x, bottom))
                .constrain_to(rect)
                .order(egui::Order::Middle)
        };
        let bar = card("viewport-tools", rect.bottom() - NAV_MARGIN)
            .show(ctx, |ui| {
                frame.show(ui, |ui| {
                    ui.set_max_width(max_width);
                    if max_width >= self.nav_bar_width() {
                        ui.horizontal(|ui| {
                            self.tool_buttons(ui);
                            ui.separator();
                            self.view_buttons(ui);
                        });
                    } else {
                        // a viewport narrower than the bar: the pointer tools on one row, the
                        // section plane and the views together under them
                        ui.horizontal_wrapped(|ui| self.tool_buttons(ui));
                        ui.horizontal_wrapped(|ui| self.view_buttons(ui));
                    }
                });
            })
            .response
            .rect;
        if !self.has_tool_options() {
            return;
        }
        card("viewport-tool-options", bar.top() - OPTIONS_GAP).show(ctx, |ui| {
            frame.show(ui, |ui| {
                ui.set_max_width(max_width);
                self.tool_options(ui);
            });
        });
    }

    /// One icon button of the navigation bar; returns whether it was clicked.
    fn nav_button(&self, ui: &mut egui::Ui, icon: &str, label: &str, hint: &str, selected: bool) -> bool {
        let m = self.settings.theme.density.metrics();
        let style = crate::theme::icon_button_style(&self.tokens);
        let inset = ((m.nav_button - m.icon) / 2.0).max(0.0);
        craft_ui::buttons::IconButton::new(label, egui::Vec2::splat(m.nav_button), &style)
            .selected(selected)
            .show(ui, |ui, rect, tint| crate::icons::paint(ui, rect.shrink(inset), icon, tint))
            .on_hover_ui(|ui| {
                ui.set_max_width(260.0);
                ui.strong(label);
                if !hint.is_empty() {
                    ui.label(hint);
                }
            })
            .clicked()
    }

    /// The width of the bar with every button on one row (px).
    fn nav_bar_width(&self) -> f32 {
        let button = self.settings.theme.density.metrics().nav_button;
        let buttons = (Tool::ALL.len() + VIEW_BUTTONS) as f32;
        buttons * (button + NAV_SPACING) + SEPARATORS * SEPARATOR_WIDTH
    }

    /// The pointer tools.
    fn tool_buttons(&mut self, ui: &mut egui::Ui) {
        ui.spacing_mut().item_spacing.x = NAV_SPACING;
        for tool in Tool::ALL {
            if self.nav_button(ui, tool.icon(), t(tool.key()), t(tool.hint_key()), self.tools.tool == tool) {
                self.cancel_interaction();
                self.tools.tool = tool;
            }
        }
    }

    /// The section plane and the two views.
    fn view_buttons(&mut self, ui: &mut egui::Ui) {
        ui.spacing_mut().item_spacing.x = NAV_SPACING;
        let on = self.tools.section.enabled;
        if self.nav_button(ui, "section-plane", t("section.title"), t("section.hint"), on) {
            if on {
                self.tools.section.enabled = false;
            } else {
                let axis = self.tools.section.axis;
                self.enable_section(axis);
            }
        }
        ui.separator();
        if self.nav_button(ui, "view-home", t("view.iso"), t("view.iso_hint"), false) {
            self.set_standard_view(StandardView::Iso);
        }
        if self.nav_button(ui, "fit", t("view.fit"), t("view.fit_hint"), false) {
            self.fit_pending = true;
        }
    }

    /// Whether [`Self::tool_options`] has anything to show.
    fn has_tool_options(&mut self) -> bool {
        let tool_has_settings = match self.tools.tool {
            Tool::Brush | Tool::Lasso | Tool::Smart => true,
            Tool::Move | Tool::Rotate => self.gizmo_mesh().is_none(),
            Tool::Faces | Tool::Edges | Tool::Navigate => false,
        };
        let brep = &self.tools.brep;
        tool_has_settings
            || self.field_pick_hint().is_some()
            || !self.tools.selection.tris.is_empty()
            || !brep.faces.is_empty()
            || !brep.edges.is_empty()
            || self.tools.section.enabled
    }

    /// The settings of the active tool and what is selected (see [`Self::has_tool_options`]).
    fn tool_options(&mut self, ui: &mut egui::Ui) {
        match self.tools.tool {
            Tool::Brush => {
                ui.horizontal(|ui| {
                    let l = ui.label(t("tool.brush_radius"));
                    ui.add(egui::Slider::new(&mut self.tools.brush_radius, 4.0..=120.0).suffix(" px"))
                        .labelled_by(l.id);
                    ui.checkbox(&mut self.tools.through, t("tool.through"));
                });
            }
            Tool::Lasso => {
                ui.checkbox(&mut self.tools.through, t("tool.through"));
            }
            Tool::Smart => {
                ui.horizontal(|ui| {
                    let l = ui.label(t("tool.smart_angle"));
                    ui.add(egui::Slider::new(&mut self.tools.smart_angle, 1.0..=90.0).suffix("°")).labelled_by(l.id);
                });
            }
            Tool::Move | Tool::Rotate => {
                if self.gizmo_mesh().is_none() {
                    ui.colored_label(self.token_color(Token::StatusWarning), t("tool.gizmo_no_mesh"));
                }
            }
            Tool::Faces | Tool::Edges | Tool::Navigate => {}
        }
        self.field_pick_strip(ui);
        self.selection_chip(ui);
        self.section_controls(ui);
    }

    /// While a form field is being picked: what to click, and a button to stop.
    fn field_pick_strip(&mut self, ui: &mut egui::Ui) {
        let Some((_, hint)) = self.field_pick_hint() else { return };
        ui.horizontal_wrapped(|ui| {
            ui.colored_label(self.token_color(Token::AccentPrimary), hint);
            if ui.small_button(t("fieldpick.stop")).clicked() {
                self.stop_field_pick();
            }
        });
    }

    fn selection_chip(&mut self, ui: &mut egui::Ui) {
        let n = self.tools.selection.tris.len();
        let (nf, ne) = (self.tools.brep.faces.len(), self.tools.brep.edges.len());
        if n == 0 && nf + ne == 0 {
            return;
        }
        ui.horizontal(|ui| {
            if n > 0 {
                ui.label(tf("select.count", &[("n", &n.to_string())]));
                if ui.button(t("select.create_region")).clicked() {
                    self.create_region_from_selection();
                }
            }
            if nf + ne > 0 {
                ui.label(tf("select.brep_count", &[("faces", &nf.to_string()), ("edges", &ne.to_string())]));
            }
            if ui.button(t("select.clear")).clicked() {
                self.clear_selection();
            }
        });
    }

    fn section_controls(&mut self, ui: &mut egui::Ui) {
        if !self.tools.section.enabled {
            return;
        }
        ui.horizontal(|ui| {
            ui.label(t("section.title"));
            for axis in SectionAxis::ALL {
                if ui.selectable_label(self.tools.section.axis == axis, t(axis.key())).clicked()
                    && self.tools.section.axis != axis
                {
                    self.enable_section(axis);
                    self.tools.section.axis = axis;
                }
            }
            ui.checkbox(&mut self.tools.section.flip, t("section.flip"));
        });
        let b = self.session.doc.bbox();
        let axis = self.tools.section.axis;
        let (lo, hi) = if b.is_empty() { (-100.0, 100.0) } else { (axis.coord(&b.min), axis.coord(&b.max)) };
        let (lo, hi) = if hi > lo { (lo, hi) } else { (lo - 1.0, lo + 1.0) };
        ui.horizontal(|ui| {
            let l = ui.label(t("section.offset"));
            ui.add(egui::Slider::new(&mut self.tools.section.offset, lo..=hi)).labelled_by(l.id);
        });
    }
}

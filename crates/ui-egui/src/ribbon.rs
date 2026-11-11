//! The ribbon: one tab per stage of the work ([`crate::workflow`]) and, under the tabs, the
//! stage's tools as buttons with an icon over a name. The search field at the end of the tab row
//! finds any tool by name; while it holds text, the tool row shows what it found.
//!
//! A tool button opens the command in the tool panel (it never runs anything by itself), so a
//! click is always safe to try.
use crate::app::SurfaceCraftApp;
use crate::i18n::{t, tf};
use crate::shell::AppAction;
use crate::theme::{self, Metrics, Token, Tokens};
use crate::workflow::{self, Entry, Stage};
use egui::text::LayoutJob;
use surfacecraft_engine::cmd::CommandSpec;

/// Text size of a tool button's name.
const TOOL_FONT: f32 = 11.5;
/// Text size of a stage tab.
const TAB_FONT: f32 = 13.0;
/// Width of the search field.
const SEARCH_WIDTH: f32 = 190.0;
/// Widest tooltip of a tool.
const TIP_WIDTH: f32 = 280.0;
/// The most search results the tool row shows.
const SEARCH_LIMIT: usize = 12;

/// The world planes a new sketch starts on: catalogue key and plane normal.
const SKETCH_PLANES: [(&str, [f64; 3]); 3] = [
    ("sketch.menu.new_xy", [0.0, 0.0, 1.0]), // not a colour
    ("sketch.menu.new_xz", [0.0, -1.0, 0.0]),
    ("sketch.menu.new_yz", [1.0, 0.0, 0.0]), // not a colour
];

/// What a click in the ribbon asks for (applied after the row is drawn).
enum Clicked {
    Command(&'static str),
    Action(AppAction),
    NewSketch([f64; 3]),
    EditSketch(surfacecraft_doc::ObjectId),
    FinishSketch,
}

/// A ribbon tool button: `icon` over `label`, as wide as the label needs. `menu` adds the mark of
/// a button that opens a list.
pub(crate) fn tool_button(
    ui: &mut egui::Ui,
    tokens: &Tokens,
    m: &Metrics,
    icon: &str,
    label: &str,
    selected: bool,
    menu: bool,
) -> egui::Response {
    let enabled = ui.is_enabled();
    let ink = tokens.color(if selected { Token::TextPrimary } else { Token::TextMuted });
    let ink = if enabled && !selected { tokens.color(Token::TextPrimary) } else { ink };
    let mut job = LayoutJob::simple(
        label.to_owned(),
        egui::FontId::proportional(TOOL_FONT),
        ink,
        (m.tool_button.x * 1.4).round(),
    );
    job.halign = egui::Align::Center;
    job.wrap.max_rows = 2;
    let galley = ui.painter().layout_job(job);
    let width = (galley.size().x + 14.0).max(m.tool_button.x);
    let (rect, resp) = ui.allocate_exact_size(egui::vec2(width, m.tool_button.y), egui::Sense::click());
    resp.widget_info(|| egui::WidgetInfo::selected(egui::Role::Button, enabled, selected, label));
    if !ui.is_rect_visible(rect) {
        return resp;
    }
    let painter = ui.painter();
    let radius = egui::CornerRadius::same(theme::RADIUS);
    if selected {
        let stroke = egui::Stroke::new(1.0, tokens.color(Token::AccentPrimary));
        painter.rect(rect, radius, tokens.color(Token::AccentSoft), stroke, egui::StrokeKind::Inside);
    } else if resp.is_pointer_button_down_on() {
        painter.rect_filled(rect, radius, tokens.color(Token::SurfaceActive));
    } else if resp.hovered() {
        painter.rect_filled(rect, radius, tokens.color(Token::SurfaceHover));
    }
    if resp.has_focus() {
        let ring = egui::Stroke::new(1.5, tokens.color(Token::FocusRing));
        painter.rect_stroke(rect, radius, ring, egui::StrokeKind::Inside);
    }
    let icon_centre = egui::pos2(rect.center().x, rect.top() + 5.0 + m.tool_icon / 2.0);
    let icon_rect = egui::Rect::from_center_size(icon_centre, egui::Vec2::splat(m.tool_icon));
    let tint = tokens.color(if selected { Token::AccentHover } else { Token::TextPrimary });
    crate::icons::paint(ui, icon_rect, icon, tint);
    if menu {
        let mark = egui::Rect::from_center_size(
            egui::pos2(icon_rect.right() + 7.0, icon_rect.center().y),
            egui::Vec2::splat(10.0),
        );
        crate::icons::paint(ui, mark, "chevron-down", tokens.color(Token::TextMuted));
    }
    ui.painter().galley(egui::pos2(rect.center().x, icon_rect.bottom() + 2.0), galley, ink);
    resp
}

/// A stage tab: the stage's name, underlined in the accent when it is the open one.
fn stage_tab(ui: &mut egui::Ui, tokens: &Tokens, m: &Metrics, label: &str, selected: bool) -> egui::Response {
    let ink = tokens.color(if selected { Token::TextPrimary } else { Token::TextMuted });
    let galley = ui.painter().layout_no_wrap(label.to_owned(), egui::FontId::proportional(TAB_FONT), ink);
    let size = egui::vec2(galley.size().x + 22.0, m.ribbon_tab_height);
    let (rect, resp) = ui.allocate_exact_size(size, egui::Sense::click());
    resp.widget_info(|| egui::WidgetInfo::selected(egui::Role::Button, ui.is_enabled(), selected, label));
    if !ui.is_rect_visible(rect) {
        return resp;
    }
    let painter = ui.painter();
    let radius = egui::CornerRadius::same(theme::RADIUS);
    if resp.hovered() && !selected {
        painter.rect_filled(rect.shrink2(egui::vec2(2.0, 3.0)), radius, tokens.color(Token::SurfaceHover));
    }
    if resp.has_focus() {
        let ring = egui::Stroke::new(1.5, tokens.color(Token::FocusRing));
        painter.rect_stroke(rect.shrink(1.0), radius, ring, egui::StrokeKind::Inside);
    }
    painter.galley(rect.center() - galley.size() / 2.0, galley, ink);
    if selected {
        let bar = egui::Rect::from_min_max(
            egui::pos2(rect.left() + 8.0, rect.bottom() - 2.0),
            egui::pos2(rect.right() - 8.0, rect.bottom()),
        );
        painter.rect_filled(bar, egui::CornerRadius::same(1), tokens.color(Token::AccentPrimary));
    }
    resp
}

/// The tooltip of a tool: its name, what it does in a sentence, and (muted) the command's id for
/// scripts.
fn tool_tip(ui: &mut egui::Ui, name: &str, hint: &str, id: Option<&str>) {
    ui.set_max_width(TIP_WIDTH);
    ui.strong(name);
    if !hint.is_empty() {
        ui.label(hint);
    }
    if let Some(id) = id {
        ui.weak(id);
    }
}

/// A vertical rule between two groups of tools.
fn group_gap(ui: &mut egui::Ui, tokens: &Tokens, m: &Metrics) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(9.0, m.tool_button.y), egui::Sense::hover());
    let x = rect.center().x;
    let stroke = egui::Stroke::new(1.0, tokens.color(Token::BorderSubtle));
    ui.painter().line_segment([egui::pos2(x, rect.top() + 8.0), egui::pos2(x, rect.bottom() - 8.0)], stroke);
}

impl SurfaceCraftApp {
    /// Draws the ribbon: the stage tabs with the search field, then the tool row.
    pub(crate) fn ribbon(&mut self, ui: &mut egui::Ui) {
        let m = self.settings.theme.density.metrics();
        let tokens = self.tokens.clone();
        ui.spacing_mut().item_spacing.x = 2.0;
        ui.horizontal(|ui| {
            for stage in Stage::ALL {
                let open = self.stage == stage && self.tool_search.trim().is_empty();
                let tab = stage_tab(ui, &tokens, &m, t(stage.title_key()), open).on_hover_text(t(stage.hint_key()));
                if tab.clicked() {
                    self.stage = stage;
                    self.tool_search.clear();
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| self.search_field(ui, &tokens));
        });
        let mut clicked: Option<Clicked> = None;
        egui::ScrollArea::horizontal().id_salt("ribbon-tools").auto_shrink([false, true]).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.set_min_height(m.tool_button.y);
                if self.tool_search.trim().is_empty() {
                    clicked = self.stage_tools(ui, &tokens, &m);
                } else {
                    clicked = self.search_results(ui, &tokens, &m);
                }
            });
        });
        if let Some(c) = clicked {
            self.ribbon_clicked(c);
        }
    }

    /// The search field at the end of the tab row (laid out right to left).
    fn search_field(&mut self, ui: &mut egui::Ui, tokens: &Tokens) {
        ui.add_space(6.0);
        if !self.tool_search.is_empty() {
            let size = egui::Vec2::splat(18.0);
            let style = theme::icon_button_style(tokens);
            let clear = craft_ui::buttons::IconButton::new(t("ribbon.search_clear"), size, &style)
                .show(ui, |ui, rect, tint| crate::icons::paint(ui, rect.shrink(3.0), "close", tint))
                .on_hover_text(t("ribbon.search_clear"));
            if clear.clicked() {
                self.tool_search.clear();
            }
        }
        let field = ui.add(
            egui::TextEdit::singleline(&mut self.tool_search)
                .hint_text(t("ribbon.search_hint"))
                .desired_width(SEARCH_WIDTH),
        );
        // the magnifier names the field for screen readers and tests
        let (rect, mark) = ui.allocate_exact_size(egui::Vec2::splat(16.0), egui::Sense::hover());
        mark.widget_info(|| egui::WidgetInfo::labeled(egui::Role::Label, true, t("ribbon.search")));
        crate::icons::paint(ui, rect, "search", tokens.color(Token::TextMuted));
        let field = field.labelled_by(mark.id);
        if field.has_focus() && ui.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.tool_search.clear();
        }
        if self.focus_search {
            field.request_focus();
            self.focus_search = false;
        }
    }

    /// The tool row of the open stage: its promoted entries, then the list of the rest.
    fn stage_tools(&mut self, ui: &mut egui::Ui, tokens: &Tokens, m: &Metrics) -> Option<Clicked> {
        let mut clicked = None;
        for entry in self.stage.entries() {
            match *entry {
                Entry::Gap => group_gap(ui, tokens, m),
                Entry::Cmd(id) => {
                    if let Some(spec) = surfacecraft_engine::cmd::find(id)
                        && self.command_button(ui, tokens, m, spec).clicked()
                    {
                        clicked = Some(Clicked::Command(spec.id));
                    }
                }
                Entry::Act(action) => {
                    if self.action_button(ui, tokens, m, action).clicked() {
                        clicked = Some(Clicked::Action(action));
                    }
                }
                Entry::NewSketch => {
                    let label = t("ribbon.new_sketch");
                    let button = tool_button(ui, tokens, m, "sketch-new", label, false, true)
                        .on_hover_ui(|ui| tool_tip(ui, label, t("ribbon.new_sketch.hint"), None));
                    egui::Popup::menu(&button).show(|ui| {
                        for (key, normal) in SKETCH_PLANES {
                            if ui.button(t(key)).clicked() {
                                clicked = Some(Clicked::NewSketch(normal));
                                ui.close();
                            }
                        }
                    });
                }
                Entry::EditSketch => clicked = self.edit_sketch_button(ui, tokens, m).or(clicked),
            }
        }
        let more = self.stage.more(self.session.commands());
        if !more.is_empty() {
            group_gap(ui, tokens, m);
            let label = t("ribbon.more");
            let button = tool_button(ui, tokens, m, "more", label, false, true)
                .on_hover_ui(|ui| tool_tip(ui, label, t("ribbon.more.hint"), None));
            egui::Popup::menu(&button).show(|ui| {
                for spec in more {
                    let name = workflow::label_of(spec);
                    if ui
                        .button(name.as_str())
                        .on_hover_ui(|ui| tool_tip(ui, &name, &workflow::hint_of(spec), Some(spec.id)))
                        .clicked()
                    {
                        clicked = Some(Clicked::Command(spec.id));
                        ui.close();
                    }
                }
            });
        }
        clicked
    }

    /// The button of a command: selected while the command is the open tool.
    fn command_button(&self, ui: &mut egui::Ui, tokens: &Tokens, m: &Metrics, spec: &CommandSpec) -> egui::Response {
        let name = workflow::label_of(spec);
        let open = self.palette_command == Some(spec.id);
        tool_button(ui, tokens, m, workflow::icon_of(spec.id), &name, open, false)
            .on_hover_ui(|ui| tool_tip(ui, &name, &workflow::hint_of(spec), Some(spec.id)))
    }

    /// The button of a shell action (a file dialog); disabled with the reason while it cannot run.
    fn action_button(&self, ui: &mut egui::Ui, tokens: &Tokens, m: &Metrics, action: AppAction) -> egui::Response {
        let label = t(action.short_key());
        let hint = t(action.hint_key());
        ui.add_enabled_ui(self.enabled(action), |ui| {
            tool_button(ui, tokens, m, workflow::action_icon(action), label, false, false)
        })
        .inner
        .on_hover_ui(|ui| tool_tip(ui, label, hint, None))
        .on_disabled_hover_ui(|ui| tool_tip(ui, label, hint, None))
    }

    /// "Edit sketch" for the selected sketch, or "Finish sketch" while one is being edited.
    fn edit_sketch_button(&self, ui: &mut egui::Ui, tokens: &Tokens, m: &Metrics) -> Option<Clicked> {
        if self.sketch_editing() {
            let label = t("sketch.finish");
            let button = tool_button(ui, tokens, m, "check", label, true, false)
                .on_hover_ui(|ui| tool_tip(ui, label, t("ribbon.finish_sketch.hint"), None));
            return button.clicked().then_some(Clicked::FinishSketch);
        }
        let selected = self.selected_object.filter(|id| self.session.doc.sketch(*id).is_some());
        let label = t("ribbon.edit_sketch");
        let hint = t("ribbon.edit_sketch.hint");
        let button = ui
            .add_enabled_ui(selected.is_some(), |ui| tool_button(ui, tokens, m, "sketch-edit", label, false, false))
            .inner
            .on_hover_ui(|ui| tool_tip(ui, label, hint, None))
            .on_disabled_hover_ui(|ui| tool_tip(ui, label, hint, None));
        selected.filter(|_| button.clicked()).map(Clicked::EditSketch)
    }

    /// The tool row while the search field holds text: the tools it found.
    fn search_results(&mut self, ui: &mut egui::Ui, tokens: &Tokens, m: &Metrics) -> Option<Clicked> {
        let found = workflow::search(self.session.commands(), &self.tool_search);
        if found.is_empty() {
            ui.add_space(8.0);
            ui.weak(tf("ribbon.search_none", &[("query", self.tool_search.trim())]));
            return None;
        }
        let mut clicked = None;
        for spec in found.iter().take(SEARCH_LIMIT) {
            if self.command_button(ui, tokens, m, spec).clicked() {
                clicked = Some(Clicked::Command(spec.id));
            }
        }
        if found.len() > SEARCH_LIMIT {
            ui.add_space(8.0);
            ui.weak(tf("ribbon.search_more", &[("n", &(found.len() - SEARCH_LIMIT).to_string())]));
        }
        clicked
    }

    fn ribbon_clicked(&mut self, clicked: Clicked) {
        match clicked {
            Clicked::Command(id) => {
                self.tool_search.clear();
                self.choose_command(id);
            }
            Clicked::Action(action) => self.perform(action),
            Clicked::NewSketch(normal) => self.new_sketch_on(normal),
            Clicked::EditSketch(id) => self.begin_sketch_edit(id),
            Clicked::FinishSketch => self.end_sketch_edit(),
        }
    }
}

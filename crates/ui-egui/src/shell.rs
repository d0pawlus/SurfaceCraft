//! The window shell: menu bar, toolbar, status bar, the dock area and the settings the shell
//! saves. Controls ask for meaning (theme tokens, catalogue keys, density metrics); the shell
//! turns a click into an [`AppAction`] and performs it as engine commands.
use crate::app::SurfaceCraftApp;
use crate::dock::{self, Panel};
use crate::i18n::{self, Lang, t};
use crate::theme::{self, Density, Preset};
use craft_ui::docking::DockArea;
use craft_ui::layout::fit_toolbar;
use serde_json::Value;
#[cfg(not(target_arch = "wasm32"))]
use serde_json::json;
use surfacecraft_doc::ObjectKind;

/// What a menu item or toolbar button does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AppAction {
    /// New document.
    New,
    /// Open a project file.
    Open,
    /// Save the project.
    Save,
    /// Import a mesh.
    ImportMesh,
    /// Import a STEP file.
    ImportStep,
    /// Export the selected body or surface as STEP.
    ExportStep,
    /// Export the selected mesh.
    ExportMesh,
    /// Undo.
    Undo,
    /// Redo.
    Redo,
    /// Frame everything in the viewport.
    FitAll,
    /// Restore the default dock layout.
    ResetLayout,
}

impl AppAction {
    /// The catalogue key of the action's label.
    #[must_use]
    pub fn label_key(self) -> &'static str {
        match self {
            AppAction::New => "action.new",
            AppAction::Open => "action.open",
            AppAction::Save => "action.save",
            AppAction::ImportMesh => "action.import_mesh",
            AppAction::ImportStep => "action.import_step",
            AppAction::ExportStep => "action.export_step",
            AppAction::ExportMesh => "action.export_mesh",
            AppAction::Undo => "action.undo",
            AppAction::Redo => "action.redo",
            AppAction::FitAll => "action.fit_all",
            AppAction::ResetLayout => "action.reset_layout",
        }
    }

    /// The icon of the action (`None`: menu only).
    #[must_use]
    pub fn icon(self) -> Option<&'static str> {
        match self {
            AppAction::New => Some("new"),
            AppAction::Open => Some("open"),
            AppAction::Save => Some("save"),
            AppAction::ImportMesh | AppAction::ImportStep => Some("import"),
            AppAction::ExportStep | AppAction::ExportMesh => Some("export"),
            AppAction::Undo => Some("undo"),
            AppAction::Redo => Some("redo"),
            AppAction::FitAll => Some("fit"),
            AppAction::ResetLayout => None,
        }
    }
}

/// What a file dialog's answer is for (the continuation `craft_dialogs` hands back).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DialogThen {
    /// Open a project.
    OpenProject,
    /// Save the project.
    SaveProject,
    /// Import a mesh.
    ImportMesh,
    /// Import a STEP file.
    ImportStep,
    /// Export this object as STEP.
    ExportStep(surfacecraft_doc::ObjectId),
    /// Export this object as a mesh.
    ExportMesh(surfacecraft_doc::ObjectId),
}

/// The toolbar's groups, in order; a group is never split by the overflow menu.
const TOOLBAR: [&[AppAction]; 4] = [
    &[AppAction::New, AppAction::Open, AppAction::Save],
    &[AppAction::ImportMesh, AppAction::ExportMesh],
    &[AppAction::Undo, AppAction::Redo],
    &[AppAction::FitAll],
];

impl SurfaceCraftApp {
    /// Installs fonts-independent context state once and re-applies the theme when it changed.
    pub fn prepare(&mut self, ctx: &egui::Context) {
        i18n::set_lang(self.settings.lang);
        if self.ctx.is_none() {
            self.ctx = Some(ctx.clone());
        }
        if !self.prepared {
            crate::icons::install(ctx);
            self.prepared = true;
        }
        let key = (self.settings.theme, self.settings.lang);
        if self.applied != Some(key) {
            theme::apply(ctx, &self.settings.theme);
            self.tokens = theme::Tokens::resolve(self.settings.theme.preset);
            self.applied = Some(key);
        }
    }

    /// Draws the whole UI (the body of `eframe::App::ui`).
    pub fn draw(&mut self, ui: &mut egui::Ui) {
        self.prepare(ui.ctx());
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.poll_control();
            self.poll_dialogs();
        }
        #[cfg(target_arch = "wasm32")]
        self.poll_inbox();
        self.shortcuts(ui);
        egui::Panel::top("menu").show(ui, |ui| self.menu_bar(ui));
        egui::Panel::top("toolbar").show(ui, |ui| self.toolbar(ui));
        egui::Panel::bottom("status").show(ui, |ui| self.status_bar(ui));
        egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| self.dock_area(ui));
    }

    /// The docks and the viewport: draws the layout and applies the actions the user's gestures
    /// emitted (the area itself never changes the layout).
    fn dock_area(&mut self, ui: &mut egui::Ui) {
        let metrics = self.settings.theme.density.metrics();
        let style = theme::dock_style(ui, &self.tokens, &metrics);
        let layout = self.workspace.layout.clone();
        let output = DockArea::new(egui::Id::unique("surfacecraft-docking"))
            .locked(self.workspace.locked)
            .show_content(ui, &layout, &style, &mut dock::Panels { app: self });
        if let Some(error) = output.error {
            self.status = crate::i18n::tf("status.dock_rejected", &[("error", &error.to_string())]);
        }
        if !output.actions.is_empty()
            && let Err(error) = dock::apply_actions(&mut self.workspace, output.actions)
        {
            self.status = crate::i18n::tf("status.dock_rejected", &[("error", &error)]);
        }
    }

    /// Whether `action` can run now (a disabled control stays visible).
    #[must_use]
    pub fn enabled(&self, action: AppAction) -> bool {
        match action {
            AppAction::Undo => !self.session.history.undo_labels().is_empty(),
            AppAction::Redo => self.session.history.redo_len() > 0,
            AppAction::ExportStep => {
                self.selected_kind().is_some_and(|k| matches!(k, ObjectKind::Body | ObjectKind::Surface))
            }
            AppAction::ExportMesh => self.selected_kind() == Some(ObjectKind::Mesh),
            #[cfg(target_arch = "wasm32")]
            AppAction::Open | AppAction::Save => false,
            _ => true,
        }
    }

    fn selected_kind(&self) -> Option<ObjectKind> {
        self.selected_object.and_then(|id| self.session.doc.kind_of(id))
    }

    /// Performs `action`: engine commands, or a queued file dialog on the desktop.
    pub fn perform(&mut self, action: AppAction) {
        match action {
            AppAction::New => {
                let _ = self.run_command("doc.new", Value::Null);
            }
            AppAction::Undo => {
                let _ = self.run_command("edit.undo", Value::Null);
            }
            AppAction::Redo => {
                let _ = self.run_command("edit.redo", Value::Null);
            }
            AppAction::FitAll => self.fit_pending = true,
            AppAction::ResetLayout => {
                self.workspace = dock::default_workspace(self.settings.theme.density.metrics().side_dock_width);
            }
            other => self.perform_file_action(other),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn perform_file_action(&mut self, action: AppAction) {
        use craft_dialogs::FileDialog;
        let project = |d: FileDialog| d.filter(t("filter.project"), &["scproj"]);
        let mesh = |d: FileDialog| d.filter(t("filter.mesh"), &["stl", "obj", "ply", "xyz", "asc"]);
        let step = |d: FileDialog| d.filter(t("filter.step"), &["step", "stp"]);
        let title = t(action.label_key()).to_string();
        let selected = self.selected_object;
        match action {
            AppAction::Open => self.dialogs.ask(project(FileDialog::open().title(title)), DialogThen::OpenProject),
            AppAction::Save => self.dialogs.ask(project(FileDialog::save().title(title)), DialogThen::SaveProject),
            AppAction::ImportMesh => self.dialogs.ask(mesh(FileDialog::open().title(title)), DialogThen::ImportMesh),
            AppAction::ImportStep => self.dialogs.ask(step(FileDialog::open().title(title)), DialogThen::ImportStep),
            AppAction::ExportStep => {
                if let Some(id) = selected {
                    self.dialogs.ask(step(FileDialog::save().title(title)), DialogThen::ExportStep(id));
                }
            }
            AppAction::ExportMesh => {
                if let Some(id) = selected {
                    let d = FileDialog::save().title(title).filter(t("filter.mesh_export"), &["stl", "obj", "ply"]);
                    self.dialogs.ask(d, DialogThen::ExportMesh(id));
                }
            }
            _ => {}
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn perform_file_action(&mut self, action: AppAction) {
        let Some(ctx) = self.ctx.clone() else { return };
        match action {
            AppAction::ImportMesh => crate::web::pick_file(&self.inbox, &["stl", "obj", "ply"], ctx),
            AppAction::ImportStep => crate::web::pick_file(&self.inbox, &["step", "stp"], ctx),
            AppAction::ExportStep | AppAction::ExportMesh => self.download_selected(),
            _ => self.status = t("status.not_on_web").to_string(),
        }
    }

    /// Runs the continuations of the file dialogs that answered since the last frame.
    #[cfg(not(target_arch = "wasm32"))]
    fn poll_dialogs(&mut self) {
        use craft_dialogs::Answer;
        for (then, answer) in self.dialogs.take_answers() {
            let path = match answer {
                Answer::Paths(p) => p.into_iter().next(),
                Answer::Unavailable(why) => {
                    self.status = crate::i18n::tf("status.dialog_unavailable", &[("why", &why)]);
                    continue;
                }
                _ => continue,
            };
            let Some(p) = path else { continue };
            let p = p.to_string_lossy().to_string();
            match then {
                DialogThen::OpenProject => {
                    let _ = self.run_command("doc.open", json!({"path": p}));
                    self.fit_pending = true;
                }
                DialogThen::SaveProject => {
                    let _ = self.run_command("doc.save", json!({"path": p}));
                }
                DialogThen::ImportMesh => {
                    let _ = self.run_command("mesh.import", json!({"path": p}));
                    self.fit_pending = true;
                }
                DialogThen::ImportStep => {
                    let _ = self.run_command("import.step", json!({"path": p}));
                    self.fit_pending = true;
                }
                DialogThen::ExportStep(id) => {
                    let _ = self.run_command("export.step", json!({"objects": [id.0], "path": p}));
                }
                DialogThen::ExportMesh(id) => {
                    let _ = self.run_command("mesh.export", json!({"object": id.0, "path": p}));
                }
            }
        }
    }

    /// A menu item for `action` (disabled while it cannot run).
    fn action_item(&mut self, ui: &mut egui::Ui, action: AppAction) {
        let enabled = self.enabled(action);
        if ui.add_enabled(enabled, egui::Button::new(t(action.label_key()))).clicked() {
            self.perform(action);
            ui.close();
        }
    }

    fn menu_bar(&mut self, ui: &mut egui::Ui) {
        egui::MenuBar::new().ui(ui, |ui| {
            ui.menu_button(t("menu.file"), |ui| {
                self.action_item(ui, AppAction::New);
                #[cfg(not(target_arch = "wasm32"))]
                {
                    self.action_item(ui, AppAction::Open);
                    self.action_item(ui, AppAction::Save);
                }
                ui.separator();
                self.action_item(ui, AppAction::ImportMesh);
                self.action_item(ui, AppAction::ImportStep);
                self.action_item(ui, AppAction::ExportStep);
                self.action_item(ui, AppAction::ExportMesh);
            });
            ui.menu_button(t("menu.edit"), |ui| {
                self.action_item(ui, AppAction::Undo);
                self.action_item(ui, AppAction::Redo);
            });
            ui.menu_button(t("menu.view"), |ui| self.view_menu(ui));
            // one menu per command group: every command is reachable from the menus too
            let mut groups: Vec<&'static str> = self.session.commands().iter().map(|c| c.group).collect();
            groups.sort_unstable();
            groups.dedup();
            for g in groups.into_iter().filter(|g| !matches!(*g, "File" | "Edit" | "View" | "App")) {
                let label = i18n::find(&format!("group.{g}")).unwrap_or(g).to_string();
                ui.menu_button(label, |ui| {
                    for c in self.session.commands().iter().filter(|c| c.group == g) {
                        if ui.button(c.label).on_hover_text(c.summary).clicked() {
                            self.choose_command(c.id);
                            ui.close();
                        }
                    }
                });
            }
        });
    }

    fn view_menu(&mut self, ui: &mut egui::Ui) {
        self.action_item(ui, AppAction::FitAll);
        ui.separator();
        ui.menu_button(t("menu.panels"), |ui| {
            for panel in Panel::ALL.into_iter().filter(|p| p.closable()) {
                let mut shown = self.workspace.layout.contains(&panel);
                if ui.checkbox(&mut shown, t(panel.title_key())).changed()
                    && let Err(e) = dock::set_visible(&mut self.workspace, panel, shown)
                {
                    self.status = crate::i18n::tf("status.dock_rejected", &[("error", &e)]);
                }
            }
        });
        ui.menu_button(t("menu.theme"), |ui| {
            for preset in Preset::ALL {
                ui.radio_value(&mut self.settings.theme.preset, preset, t(preset.name_key()));
            }
        });
        ui.menu_button(t("menu.density"), |ui| {
            for density in Density::ALL {
                ui.radio_value(&mut self.settings.theme.density, density, t(density.name_key()));
            }
        });
        ui.menu_button(t("menu.language"), |ui| {
            for lang in Lang::ALL {
                ui.radio_value(&mut self.settings.lang, lang, t(lang.name_key()));
            }
        });
        ui.separator();
        self.action_item(ui, AppAction::ResetLayout);
    }

    /// The toolbar: icon buttons from the catalogue, grouped, with an overflow menu for the groups
    /// that do not fit.
    fn toolbar(&mut self, ui: &mut egui::Ui) {
        let m = self.settings.theme.density.metrics();
        let spacing = m.item_spacing.x;
        let sep = spacing * 3.0;
        let natural: Vec<f32> =
            TOOLBAR.iter().map(|g| g.len() as f32 * m.icon_button + (g.len() as f32 - 1.0) * spacing + sep).collect();
        let overflow_width = 70.0;
        let fit = fit_toolbar(&natural, ui.available_width(), spacing, overflow_width);
        let shown = fit.as_ref().map_or(TOOLBAR.len(), |f| f.shown.len());
        let style = theme::icon_button_style(&self.tokens);
        let inset = ((m.icon_button - m.icon) / 2.0).max(0.0);
        let mut clicked: Option<AppAction> = None;
        ui.horizontal(|ui| {
            for group in TOOLBAR.iter().take(shown) {
                for &action in *group {
                    let Some(icon) = action.icon() else { continue };
                    ui.add_enabled_ui(self.enabled(action), |ui| {
                        let label = t(action.label_key());
                        let r = craft_ui::buttons::IconButton::new(label, egui::Vec2::splat(m.icon_button), &style)
                            .show(ui, |ui, rect, tint| crate::icons::paint(ui, rect.shrink(inset), icon, tint))
                            .on_hover_text(label);
                        if r.clicked() {
                            clicked = Some(action);
                        }
                    });
                }
                ui.add_space(sep - spacing);
                ui.separator();
            }
            if shown < TOOLBAR.len() {
                ui.menu_button(t("toolbar.more"), |ui| {
                    for group in TOOLBAR.iter().skip(shown) {
                        for &action in *group {
                            if ui.add_enabled(self.enabled(action), egui::Button::new(t(action.label_key()))).clicked()
                            {
                                clicked = Some(action);
                                ui.close();
                            }
                        }
                    }
                });
            }
        });
        if let Some(action) = clicked {
            self.perform(action);
        }
    }

    fn status_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(&self.status);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let n = self.session.doc.summaries().len();
                ui.weak(crate::i18n::tf("status.objects", &[("n", &n.to_string())]));
            });
        });
    }
}

impl eframe::App for SurfaceCraftApp {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        self.draw(ui);
        #[cfg(not(target_arch = "wasm32"))]
        self.dialogs.show_pending(Some(frame));
        #[cfg(target_arch = "wasm32")]
        let _ = frame;
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        self.settings.workspace = Some(self.workspace.clone());
        storage.set_string(crate::settings::STORAGE_KEY, self.settings.to_json());
    }

    #[cfg(target_arch = "wasm32")]
    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(&mut *self)
    }
}

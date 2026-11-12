//! The window shell: the menu bar with its quick-access buttons, the ribbon ([`crate::ribbon`]),
//! the status bar, the dock area and the settings the shell saves. Controls ask for meaning (theme tokens, catalogue keys, density metrics); the shell
//! turns a click into an [`AppAction`] and performs it as engine commands.
use crate::app::SurfaceCraftApp;
use crate::dock::{self, Panel};
use crate::i18n::{self, Lang, t};
use crate::theme::{self, Density, Preset};
use craft_ui::docking::DockArea;
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
    /// Load a small sample part to try the tools on.
    SampleScan,
    /// Put the keyboard in the ribbon's search field.
    SearchTools,
    /// Close the open tool and show the guide.
    ShowGuide,
    /// Show or hide the command log.
    ToggleLog,
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
            AppAction::SampleScan => "action.sample_scan",
            AppAction::SearchTools => "action.search_tools",
            AppAction::ShowGuide => "action.show_guide",
            AppAction::ToggleLog => "action.toggle_log",
        }
    }

    /// Every action (catalogue checks).
    pub const ALL: [AppAction; 15] = [
        AppAction::New,
        AppAction::Open,
        AppAction::Save,
        AppAction::ImportMesh,
        AppAction::ImportStep,
        AppAction::ExportStep,
        AppAction::ExportMesh,
        AppAction::Undo,
        AppAction::Redo,
        AppAction::FitAll,
        AppAction::ResetLayout,
        AppAction::SampleScan,
        AppAction::SearchTools,
        AppAction::ShowGuide,
        AppAction::ToggleLog,
    ];

    /// The catalogue key of the action's short name (a ribbon button, a welcome button).
    #[must_use]
    pub fn short_key(self) -> &'static str {
        match self {
            AppAction::ImportMesh => "action.import_mesh.short",
            AppAction::ImportStep => "action.import_step.short",
            AppAction::ExportStep => "action.export_step.short",
            AppAction::ExportMesh => "action.export_mesh.short",
            AppAction::Open => "action.open.short",
            AppAction::Save => "action.save.short",
            other => other.label_key(),
        }
    }

    /// The catalogue key of the sentence that says what the action does (its tooltip).
    #[must_use]
    pub fn hint_key(self) -> &'static str {
        match self {
            AppAction::New => "action.new.hint",
            AppAction::Open => "action.open.hint",
            AppAction::Save => "action.save.hint",
            AppAction::ImportMesh => "action.import_mesh.hint",
            AppAction::ImportStep => "action.import_step.hint",
            AppAction::ExportStep => "action.export_step.hint",
            AppAction::ExportMesh => "action.export_mesh.hint",
            AppAction::Undo => "action.undo.hint",
            AppAction::Redo => "action.redo.hint",
            AppAction::FitAll => "action.fit_all.hint",
            AppAction::ResetLayout => "action.reset_layout.hint",
            AppAction::SampleScan => "action.sample_scan.hint",
            AppAction::SearchTools => "action.search_tools.hint",
            AppAction::ShowGuide => "action.show_guide.hint",
            AppAction::ToggleLog => "action.toggle_log.hint",
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
            AppAction::SampleScan => Some("sample"),
            AppAction::SearchTools => Some("search"),
            AppAction::ShowGuide => Some("guide"),
            AppAction::ToggleLog => Some("log"),
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

/// The quick-access buttons beside the menus: what is used all the time, whatever the stage.
const QUICK_ACCESS: [AppAction; 3] = [AppAction::Save, AppAction::Undo, AppAction::Redo];

/// The sample part of the welcome card: the synthetic bracket (a block with a boss and a dome).
const SAMPLE_SHAPE: &str = "bracket";

impl SurfaceCraftApp {
    /// Installs fonts-independent context state once and re-applies the theme when it changed.
    pub fn prepare(&mut self, ctx: &egui::Context) {
        i18n::set_lang(self.settings.lang);
        // the browser says which keyboard it runs on; the desktop build knows from its target
        i18n::set_mac_keys(matches!(ctx.os(), egui::os::OperatingSystem::Mac | egui::os::OperatingSystem::IOS));
        if self.ctx.is_none() {
            self.ctx = Some(ctx.clone());
        }
        if !self.prepared {
            crate::icons::install(ctx);
            self.prepared = true;
        }
        let key = (self.settings.theme, self.settings.lang);
        if self.applied != Some(key) {
            // The status line follows the language: the idle text is translated (the app starts
            // before the language is known), and a message written in the language just left
            // gives way to the idle text instead of staying on screen in the wrong language.
            let was = self.applied.map(|(_, lang)| lang);
            let idle = Lang::ALL.iter().any(|l| self.status == i18n::lookup(*l, "status.ready"));
            if idle || was.is_some_and(|lang| lang != self.settings.lang) {
                self.status = t("status.ready").to_string();
            }
            theme::apply(ctx, &self.settings.theme);
            self.tokens = theme::Tokens::resolve(self.settings.theme.preset);
            self.applied = Some(key);
        }
    }

    /// Draws the whole UI (the body of `eframe::App::ui`).
    pub fn draw(&mut self, ui: &mut egui::Ui) {
        self.prepare(ui.ctx());
        // a finished background command is taken over before anything reads the session
        let now = ui.input(|i| i.time);
        self.advance_job(now);
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.poll_control();
            self.poll_dialogs();
        }
        #[cfg(target_arch = "wasm32")]
        self.poll_inbox();
        self.shortcuts(ui);
        // decided before anything is drawn, so a key that ends a pick or leaves a text field
        // this frame does not also confirm or close the tool
        self.keys_free = !(self.is_busy()
            || ui.memory(|m| m.focused().is_some())
            || egui::Popup::is_any_open(ui.ctx())
            || ui.input(|i| i.pointer.any_down())
            || self.field_pick.is_some()
            || self.sketch_editing()
            || !self.tools.lasso.is_empty());
        let tokens = self.tokens.clone();
        let chrome = |x: i8, y: i8| theme::chrome_frame(&tokens, egui::Margin::symmetric(x, y));
        egui::Panel::top("menu").frame(chrome(6, 2)).show_separator_line(false).show(ui, |ui| self.menu_bar(ui));
        egui::Panel::top("ribbon").frame(chrome(6, 3)).show(ui, |ui| self.ribbon(ui));
        egui::Panel::bottom("status").frame(chrome(8, 3)).show(ui, |ui| self.status_bar(ui));
        egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| self.dock_area(ui));
        self.busy_overlay(ui.ctx());
    }

    /// The docks and the viewport: draws the layout and applies the actions the user's gestures
    /// emitted (the area itself never changes the layout).
    fn dock_area(&mut self, ui: &mut egui::Ui) {
        // A window that is not sized yet (the web canvas on its first frame) has nothing to lay
        // out; the dock area would call that an invalid geometry. The next frame (a resize wakes
        // the runner) draws it once the size is known.
        let rect = ui.available_rect_before_wrap();
        let sized = |v: f32| v.is_finite() && v > 0.0 && v <= 1_000_000.0;
        if !(rect.is_finite() && sized(rect.width()) && sized(rect.height())) {
            return;
        }
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
            AppAction::SampleScan => {
                let _ = self.run_command("mesh.synth", serde_json::json!({"shape": SAMPLE_SHAPE, "name": "sample"}));
                self.fit_pending = true;
            }
            AppAction::SearchTools => self.focus_search = true,
            AppAction::ShowGuide => self.close_tool(),
            AppAction::ToggleLog => {
                let shown = self.workspace.layout.contains(&Panel::Log);
                if let Err(e) = dock::set_visible(&mut self.workspace, Panel::Log, !shown) {
                    self.status = crate::i18n::tf("status.dock_rejected", &[("error", &e)]);
                }
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
        // an answer that arrives while a command runs waits for it (it is not dropped)
        if self.is_busy() {
            return;
        }
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
            use crate::jobs::After;
            match then {
                DialogThen::OpenProject => {
                    let _ = self.run_command("doc.open", json!({"path": p}));
                    self.fit_pending = true;
                }
                DialogThen::SaveProject => {
                    let _ = self.run_command("doc.save", json!({"path": p}));
                }
                DialogThen::ImportMesh => {
                    let units = self.settings.import_units;
                    self.launch("mesh.import", json!({"path": p, "units": units}), After::Fit);
                }
                DialogThen::ImportStep => {
                    self.launch("import.step", json!({"path": p}), After::Fit);
                }
                DialogThen::ExportStep(id) => {
                    self.launch("export.step", json!({"objects": [id.0], "path": p}), After::Nothing);
                }
                DialogThen::ExportMesh(id) => {
                    self.launch("mesh.export", json!({"object": id.0, "path": p}), After::Nothing);
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

    /// The menu bar: File, Edit, View, Tools (every tool, by stage) and Help, then the
    /// quick-access buttons.
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
                self.action_item(ui, AppAction::SampleScan);
                ui.separator();
                self.action_item(ui, AppAction::ExportStep);
                self.action_item(ui, AppAction::ExportMesh);
            });
            ui.menu_button(t("menu.edit"), |ui| {
                self.action_item(ui, AppAction::Undo);
                self.action_item(ui, AppAction::Redo);
            });
            ui.menu_button(t("menu.view"), |ui| self.view_menu(ui));
            ui.menu_button(t("menu.tools"), |ui| self.tools_menu(ui));
            ui.menu_button(t("menu.help"), |ui| {
                self.action_item(ui, AppAction::ShowGuide);
                self.action_item(ui, AppAction::SearchTools);
                ui.separator();
                ui.label(t("help.mouse"));
                ui.weak(t("help.mouse.orbit"));
                ui.weak(t("help.mouse.pan"));
                ui.weak(t("help.mouse.zoom"));
                ui.weak(t("help.mouse.pick"));
            });
            ui.add_space(8.0);
            ui.separator();
            self.quick_access(ui);
        });
    }

    /// The Tools menu: every tool of the engine, one submenu per stage in the order of the work
    /// (what the ribbon shows, in list form).
    fn tools_menu(&mut self, ui: &mut egui::Ui) {
        self.action_item(ui, AppAction::SearchTools);
        ui.separator();
        for stage in crate::workflow::Stage::ALL {
            ui.menu_button(t(stage.title_key()), |ui| {
                if stage == crate::workflow::Stage::Sketch {
                    self.sketch_menu_items(ui);
                }
                for c in stage.all(self.session.commands()) {
                    let name = crate::workflow::label_of(c);
                    if ui.button(name).on_hover_text(crate::workflow::hint_of(c)).clicked() {
                        self.choose_command(c.id);
                        ui.close();
                    }
                }
            });
        }
    }

    /// The quick-access buttons beside the menus.
    fn quick_access(&mut self, ui: &mut egui::Ui) {
        let m = self.settings.theme.density.metrics();
        let style = theme::icon_button_style(&self.tokens);
        let side = m.icon_button - 4.0;
        let inset = ((side - m.icon) / 2.0).max(0.0);
        let mut clicked: Option<AppAction> = None;
        for action in QUICK_ACCESS {
            let Some(icon) = action.icon() else { continue };
            ui.add_enabled_ui(self.enabled(action), |ui| {
                let label = t(action.label_key());
                let r = craft_ui::buttons::IconButton::new(label, egui::Vec2::splat(side), &style)
                    .show(ui, |ui, rect, tint| crate::icons::paint(ui, rect.shrink(inset), icon, tint))
                    .on_hover_text(t(action.hint_key()));
                if r.clicked() {
                    clicked = Some(action);
                }
            });
        }
        if let Some(action) = clicked {
            self.perform(action);
        }
    }

    fn view_menu(&mut self, ui: &mut egui::Ui) {
        self.action_item(ui, AppAction::FitAll);
        self.action_item(ui, AppAction::ToggleLog);
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

    /// The status bar: the last message, what the pointer does in the viewport now, and the
    /// number of objects.
    fn status_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(&self.status);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let n = self.session.doc.summaries().len();
                ui.weak(crate::i18n::tf("status.objects", &[("n", &n.to_string())]));
                ui.separator();
                ui.add(egui::Label::new(egui::RichText::new(t(self.pointer_hint_key())).weak()).truncate());
            });
        });
    }

    /// The catalogue key of the sentence that says what the mouse does in the viewport now.
    #[must_use]
    pub fn pointer_hint_key(&self) -> &'static str {
        use crate::viewtools::Tool;
        if self.sketch_editing() {
            return "hint.sketch";
        }
        if self.field_pick.is_some() {
            return "hint.field_pick";
        }
        match self.tools.tool {
            Tool::Navigate => "hint.navigate",
            Tool::Brush => "hint.brush",
            Tool::Lasso => "hint.lasso",
            Tool::Smart => "hint.smart",
            Tool::Faces | Tool::Edges => "hint.brep",
            Tool::Move | Tool::Rotate => "hint.gizmo",
        }
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

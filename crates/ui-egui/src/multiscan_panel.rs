//! The "Many scans" wizard in the tool panel: the glue between the state machine of
//! [`crate::multiscan`] and the app. The wizard's commands run the way every heavy command does
//! (in the background, with the busy dialog and its Cancel, [`crate::jobs`]) and their outcomes
//! come back through [`SurfaceCraftApp::wizard_result`]; what the wizard shows of a proposal is
//! drawn the way an auto-align proposal is: the scans in the proposed pose, the document untouched.
//! The step bodies are in [`steps`].
mod steps;

use crate::app::SurfaceCraftApp;
use crate::camera::Camera;
use crate::i18n::{engine_message, t};
use crate::jobs::After;
use crate::multiscan::report::Matrix;
use crate::multiscan::{Action, Effects, Problem, Purpose, Step, Wizard};
use crate::theme::Token;
use serde_json::Value;
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::EngineError;
use surfacecraft_geom::{Aabb, Point3, Transform};

/// With this many meshes or fewer in the document (and at least two), all of them are chosen
/// when the wizard opens, in the order of the object list.
const PRESELECT_MAX: usize = 8;

/// The wizard while it is open.
pub(crate) struct ActiveWizard {
    wizard: Wizard,
    /// The view from before the camera first followed a proposal: where it returns on cancel.
    camera_before: Option<Camera>,
}

impl SurfaceCraftApp {
    /// The wizard, while it is open.
    #[must_use]
    pub fn wizard(&self) -> Option<&Wizard> {
        self.multiscan.as_ref().map(|a| &a.wizard)
    }

    /// Opens the "Many scans" wizard in the tool panel (a tool that was open closes; a wizard
    /// that was open is shown again).
    pub fn open_multiscan(&mut self) {
        self.close_tool();
        let _ = crate::dock::set_visible(&mut self.workspace, crate::dock::Panel::Properties, true);
        self.stage = crate::workflow::Stage::Align;
        if self.multiscan.is_some() {
            return;
        }
        let meshes: Vec<ObjectId> = self.session.doc.meshes.iter().map(|m| m.id).collect();
        let chosen = if (2..=PRESELECT_MAX).contains(&meshes.len()) { meshes } else { Vec::new() };
        let wizard = Wizard::new(&self.undo_labels(), chosen);
        self.multiscan = Some(ActiveWizard { wizard, camera_before: None });
    }

    fn undo_labels(&self) -> Vec<String> {
        self.session.history.undo_labels().into_iter().map(str::to_string).collect()
    }

    /// Does what the user did in the wizard.
    pub fn wizard_act(&mut self, action: Action) {
        let history = self.undo_labels();
        let Some(active) = self.multiscan.as_mut() else { return };
        let effects = active.wizard.act(action, &history);
        self.wizard_carry_out(effects);
    }

    /// Takes the outcome of a command the wizard started (see `jobs::After::Wizard`).
    pub(crate) fn wizard_result(&mut self, purpose: Purpose, result: Result<Value, EngineError>) {
        let history = self.undo_labels();
        let Some(active) = self.multiscan.as_mut() else { return };
        let effects = active.wizard.on_result(purpose, result.map_err(|e| e.to_string()), &history);
        self.wizard_carry_out(effects);
    }

    fn wizard_carry_out(&mut self, effects: Effects) {
        for _ in 0..effects.undo {
            if let Err(e) = self.run_command("edit.undo", Value::Null) {
                if let Some(active) = self.multiscan.as_mut() {
                    active.wizard.fail(e.to_string());
                }
                break;
            }
        }
        if effects.close {
            self.close_wizard(effects.select);
            return;
        }
        if let Some(request) = effects.run {
            self.launch(request.command, request.params, After::Wizard(request.purpose));
        }
        self.frame_wizard_preview();
    }

    fn close_wizard(&mut self, select: Option<u64>) {
        let Some(active) = self.multiscan.take() else { return };
        match select {
            Some(id) => {
                self.selected_object = Some(ObjectId(id));
                self.fit_pending = true;
            }
            None => {
                if let Some(camera) = active.camera_before {
                    self.camera = camera;
                }
            }
        }
    }

    /// The scans the wizard shows in a proposed pose, with their motions.
    #[must_use]
    pub(crate) fn wizard_preview(&self) -> Vec<(ObjectId, Matrix)> {
        if self.palette_command.is_some() {
            return Vec::new();
        }
        self.wizard().map(Wizard::preview).unwrap_or_default()
    }

    /// Looks at where the proposal puts the scans (a scan from another view is often far away).
    fn frame_wizard_preview(&mut self) {
        let Some(shown) = self.wizard_framed_box() else { return };
        if let Some(active) = self.multiscan.as_mut() {
            let camera = &self.camera;
            active.camera_before.get_or_insert_with(|| camera.clone());
        }
        self.camera.fit(&shown);
    }

    fn wizard_framed_box(&self) -> Option<Aabb> {
        let corners: Vec<Point3> = self
            .wizard_preview()
            .iter()
            .filter_map(|(id, matrix)| Some((self.session.doc.mesh(*id)?, Transform::from_rows(matrix)?)))
            .filter(|(m, _)| !m.mesh.bbox().is_empty())
            .flat_map(|(m, motion)| {
                let moved = crate::align_proposal::moved_box(&m.mesh.bbox(), &motion);
                [moved.min, moved.max]
            })
            .collect();
        (!corners.is_empty()).then(|| Aabb::from_points(&corners))
    }

    /// Drops the wizard when a scan it works on is gone (a new or opened document, a deletion):
    /// nothing is undone, because the history it recorded is not there any more.
    pub(crate) fn wizard_housekeeping(&mut self) {
        let gone = self.wizard().is_some_and(|w| !w.refers_only_to(|id| self.session.doc.mesh(id).is_some()));
        if gone && !self.is_busy() {
            self.multiscan = None;
            self.status = t("wizard.gone").to_string();
        }
    }

    /// The tool panel's body while the wizard is open.
    pub(crate) fn multiscan_panel(&mut self, ui: &mut egui::Ui) {
        let Some(wizard) = self.wizard().cloned() else { return };
        let scrolled = egui::ScrollArea::vertical().id_salt("dock-wizard").auto_shrink([false, false]).show(ui, |ui| {
            ui.strong(t("wizard.title"));
            ui.weak(t("wizard.intro"));
            ui.add_space(6.0);
            self.wizard_steps_bar(ui, wizard.step());
            ui.add_space(4.0);
            ui.weak(t(wizard.step().hint_key()));
            ui.add_space(4.0);
            let chosen = self.wizard_step_body(ui, &wizard);
            ui.add_space(8.0);
            self.wizard_problem(ui, &wizard);
            ui.separator();
            self.wizard_footer(ui, &wizard).or(chosen)
        });
        if let Some(action) = scrolled.inner {
            self.wizard_act(action);
        }
    }

    fn wizard_steps_bar(&self, ui: &mut egui::Ui, current: Step) {
        let at = Step::ALL.iter().position(|s| *s == current).unwrap_or(0);
        ui.horizontal(|ui| {
            for (i, step) in Step::ALL.into_iter().enumerate() {
                self.step_badge(ui, i + 1, i < at, i == at);
                let title = egui::RichText::new(t(step.title_key()));
                ui.label(if i == at { title.strong() } else { title.color(self.tokens.color(Token::TextMuted)) });
            }
        });
    }

    fn wizard_problem(&self, ui: &mut egui::Ui, wizard: &Wizard) {
        let Some(problem) = wizard.problem() else { return };
        let text = match problem {
            Problem::Engine(message) => engine_message(message),
            Problem::Stale => t("wizard.problem.stale").to_string(),
            Problem::BadTolerance => t("wizard.problem.bad_tolerance").to_string(),
            Problem::NothingMoved => t("wizard.problem.nothing_moved").to_string(),
            Problem::Unreadable => t("wizard.problem.unreadable").to_string(),
        };
        ui.colored_label(self.tokens.color(Token::StatusError), text);
    }

    /// Back, Next (or Finish) and Cancel.
    fn wizard_footer(&self, ui: &mut egui::Ui, wizard: &Wizard) -> Option<Action> {
        use crate::multiscan::MergeState;
        let mut action = None;
        let idle = wizard.running().is_none();
        ui.horizontal(|ui| {
            let back = ui.add_enabled(idle && wizard.step() != Step::Pick, egui::Button::new(t("wizard.back")));
            if back.on_hover_text(t("wizard.back.hint")).clicked() {
                action = Some(Action::Back);
            }
            match (wizard.step(), wizard.merge()) {
                (Step::Merge, MergeState::Done(_)) => {
                    if ui.add(crate::theme::primary_button(&self.tokens, t("wizard.finish"))).clicked() {
                        action = Some(Action::Finish);
                    }
                }
                (Step::Merge, _) => {}
                _ => {
                    let next =
                        ui.add_enabled(wizard.can_next(), crate::theme::primary_button(&self.tokens, t("wizard.next")));
                    if next.clicked() {
                        action = Some(Action::Next);
                    }
                }
            }
            if ui
                .add_enabled(idle, egui::Button::new(t("form.cancel")))
                .on_hover_text(t("wizard.cancel.hint"))
                .clicked()
            {
                action = Some(Action::Cancel);
            }
        });
        if wizard.problem() == Some(&Problem::Stale) && ui.button(t("wizard.leave")).clicked() {
            action = Some(Action::Leave);
        }
        action
    }
}

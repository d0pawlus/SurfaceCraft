//! The assistant: a language model that runs commands towards a goal (see `surfacecraft_ai`).
use crate::SurfaceCraftApp;
use crate::i18n::{t, tf};

impl SurfaceCraftApp {
    /// Uses `p` as the assistant's language model.
    pub fn set_provider(&mut self, p: Box<dyn surfacecraft_ai::Provider>) {
        self.provider = Some(p);
    }

    /// Runs the assistant on the current goal with a picture of the viewport. Refused while a
    /// command runs in the background.
    pub fn ask_assistant(&mut self) {
        let Some(mut p) = self.provider.take() else {
            self.status = t("assistant.no_provider_status").to_string();
            return;
        };
        let mode = if self.assistant_align { surfacecraft_ai::Mode::Align } else { surfacecraft_ai::Mode::Workflow };
        let cam = self.camera.clone();
        let goal = self.assistant_goal.clone();
        let outcome = self.idle_session().map(|session| {
            let png = crate::shot::screenshot_png(session, Some(&cam), 512, 384);
            surfacecraft_ai::run(session, p.as_mut(), mode, &goal, png.as_deref(), 20)
        });
        self.provider = Some(p);
        // a refusal has already put "… is still running" on the status line
        let Ok(outcome) = outcome else { return };
        self.status = match outcome {
            Ok(o) => tf("assistant.done", &[("summary", &o.summary), ("n", &o.steps.len().to_string())]),
            Err(e) => tf("assistant.failed", &[("error", &crate::i18n::engine_message(&e.to_string()))]),
        };
        self.fit_pending = true;
    }

    pub(crate) fn assistant_panel(&mut self, ui: &mut egui::Ui) {
        ui.strong(t("assistant.title"));
        ui.horizontal(|ui| {
            ui.radio_value(&mut self.assistant_align, true, t("assistant.align"));
            ui.radio_value(&mut self.assistant_align, false, t("assistant.workflow"));
        });
        let l = ui.label(t("assistant.goal"));
        ui.add(egui::TextEdit::singleline(&mut self.assistant_goal).hint_text(t("assistant.goal_hint")))
            .labelled_by(l.id);
        if ui.button(t("assistant.ask")).clicked() {
            self.ask_assistant();
        }
    }
}

/// The language model named by the environment, if any (none in the browser).
pub(crate) fn default_provider() -> Option<Box<dyn surfacecraft_ai::Provider>> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        surfacecraft_ai::CommandProvider::from_env().map(|p| Box::new(p) as Box<dyn surfacecraft_ai::Provider>)
    }
    #[cfg(target_arch = "wasm32")]
    {
        None
    }
}

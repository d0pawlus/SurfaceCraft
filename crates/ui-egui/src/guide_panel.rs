//! What a newcomer sees first: the guide in the tool panel (the steps from a scan to an exported
//! model, the next one explained) and the welcome card over an empty viewport. The logic of the
//! steps is in [`crate::guide`]; this module draws it.
use crate::app::SurfaceCraftApp;
use crate::i18n::{t, tf};
use crate::theme::Token;

/// Width of the welcome card's content (points).
const WELCOME_WIDTH: f32 = 400.0;
/// The smallest viewport the welcome card is shown in.
const WELCOME_MIN: egui::Vec2 = egui::vec2(460.0, 320.0);

impl SurfaceCraftApp {
    /// The guide: the steps from a scan to an exported model, the next one explained with a
    /// button that opens its stage.
    pub(crate) fn guide_panel(&mut self, ui: &mut egui::Ui) {
        use crate::guide::Step;
        let progress = self.progress();
        let next = progress.next();
        ui.strong(t("guide.title"));
        ui.weak(t("guide.intro"));
        ui.add_space(6.0);
        let mut go: Option<Step> = None;
        for (i, step) in Step::ALL.into_iter().enumerate() {
            let done = progress.done(step);
            let current = next == Some(step);
            ui.horizontal(|ui| {
                self.step_badge(ui, i + 1, done, current);
                let title = t(step.title_key());
                let text = if current { egui::RichText::new(title).strong() } else { egui::RichText::new(title) };
                let text = if done { text.color(self.tokens.color(Token::TextMuted)) } else { text };
                if ui.add(egui::Label::new(text).sense(egui::Sense::click())).clicked() {
                    go = Some(step);
                }
                if step.optional() && !done {
                    ui.weak(t("guide.optional"));
                }
            });
            if current {
                ui.indent(("guide-step", i), |ui| {
                    ui.label(t(step.hint_key()));
                    ui.horizontal(|ui| {
                        if step == Step::Import {
                            let import = crate::shell::AppAction::ImportMesh;
                            if ui.add(crate::theme::primary_button(&self.tokens, t(import.short_key()))).clicked() {
                                self.perform(import);
                            }
                            let sample = crate::shell::AppAction::SampleScan;
                            if ui.button(t(sample.label_key())).on_hover_text(t(sample.hint_key())).clicked() {
                                self.perform(sample);
                            }
                        } else {
                            let stage = t(step.stage().title_key());
                            let label = tf("guide.go", &[("stage", stage)]);
                            if ui.add(crate::theme::primary_button(&self.tokens, &label)).clicked() {
                                go = Some(step);
                            }
                        }
                    });
                });
                ui.add_space(4.0);
            }
        }
        if next.is_none() {
            ui.add_space(4.0);
            ui.colored_label(self.tokens.color(Token::StatusOk), t("guide.finished"));
        }
        if let Some(step) = go {
            self.stage = step.stage();
            self.tool_search.clear();
        }
    }

    /// The round badge of a guide step: a check when done, its number otherwise (in the accent
    /// for the step to do now).
    fn step_badge(&self, ui: &mut egui::Ui, number: usize, done: bool, current: bool) {
        let (rect, _) = ui.allocate_exact_size(egui::Vec2::splat(20.0), egui::Sense::hover());
        let c = |token| self.tokens.color(token);
        let painter = ui.painter();
        if done {
            painter.circle_filled(rect.center(), 9.0, c(Token::StatusOk));
            crate::icons::paint(ui, rect.shrink(4.0), "check", c(Token::SurfaceBase));
            return;
        }
        let ring = if current { Token::AccentPrimary } else { Token::BorderStrong };
        if current {
            painter.circle_filled(rect.center(), 9.0, c(Token::AccentSoft));
        }
        painter.circle_stroke(rect.center(), 9.0, egui::Stroke::new(1.5, c(ring)));
        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            number.to_string(),
            egui::FontId::proportional(11.0),
            c(if current { Token::TextPrimary } else { Token::TextMuted }),
        );
    }

    /// The card in the middle of an empty viewport: what the application is for and the three
    /// ways to start (a scan of your own, the sample part, a saved project).
    pub(crate) fn welcome_card(&mut self, ctx: &egui::Context, rect: egui::Rect) {
        use crate::shell::AppAction;
        if !self.session.doc.summaries().is_empty() || rect.width() < WELCOME_MIN.x || rect.height() < WELCOME_MIN.y {
            return;
        }
        let frame = crate::theme::card_frame(&self.tokens).inner_margin(egui::Margin::same(20));
        let mut clicked: Option<AppAction> = None;
        egui::Area::new(egui::Id::unique("welcome"))
            .pivot(egui::Align2::CENTER_CENTER)
            .fixed_pos(rect.center() - egui::vec2(0.0, 24.0))
            .order(egui::Order::Middle)
            .show(ctx, |ui| {
                frame.show(ui, |ui| {
                    ui.set_max_width(WELCOME_WIDTH);
                    ui.vertical_centered(|ui| {
                        ui.heading(t("welcome.title"));
                        ui.label(t("welcome.intro"));
                        ui.add_space(10.0);
                        let import = AppAction::ImportMesh;
                        let button = crate::theme::primary_button(&self.tokens, t("welcome.import"))
                            .min_size(egui::vec2(WELCOME_WIDTH * 0.6, 32.0));
                        if ui.add(button).on_hover_text(t(import.hint_key())).clicked() {
                            clicked = Some(import);
                        }
                        ui.add_space(2.0);
                        ui.horizontal(|ui| {
                            // the two secondary ways to start share a row, centred under the first
                            let pad = (ui.available_width() - WELCOME_WIDTH * 0.6).max(0.0) / 2.0;
                            ui.add_space(pad);
                            for action in [AppAction::SampleScan, AppAction::Open] {
                                if !self.enabled(action) {
                                    continue;
                                }
                                if ui.button(t(action.short_key())).on_hover_text(t(action.hint_key())).clicked() {
                                    clicked = Some(action);
                                }
                            }
                        });
                        ui.add_space(10.0);
                        ui.weak(t("welcome.steps"));
                        ui.weak(t("welcome.mouse"));
                    });
                });
            });
        if let Some(action) = clicked {
            self.perform(action);
        }
    }
}

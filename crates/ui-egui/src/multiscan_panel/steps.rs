//! The bodies of the wizard's four steps. Each is drawn from a copy of the wizard's state and
//! answers with the [`Action`] the user chose, if one; nothing here runs a command.
use crate::app::SurfaceCraftApp;
use crate::i18n::{engine_message, t, tf};
use crate::multiscan::report::{GlobalReport, MergeReport};
use crate::multiscan::{Action, GlobalState, MergeState, Pair, PairState, StartChoice, Step, Wizard};
use crate::theme::Token;
use surfacecraft_doc::ObjectId;

/// Decimals of a distance in millimetres.
const MM_DECIMALS: usize = 3;

fn percent(share: f64) -> String {
    format!("{:.0}", 100.0 * share)
}

fn millimetres(value: f64) -> String {
    format!("{value:.MM_DECIMALS$}")
}

impl SurfaceCraftApp {
    /// The name of a scan as the object list shows it.
    fn scan_name(&self, id: ObjectId) -> String {
        self.session.doc.mesh(id).map_or_else(|| format!("#{}", id.0), |m| m.name.clone())
    }

    /// The name of the scan at position `index` of the wizard's list.
    fn scan_at(&self, wizard: &Wizard, index: usize) -> String {
        wizard.scans().get(index).map_or_else(|| format!("#{index}"), |id| self.scan_name(*id))
    }

    pub(super) fn wizard_step_body(&self, ui: &mut egui::Ui, wizard: &Wizard) -> Option<Action> {
        match wizard.step() {
            Step::Pick => self.pick_body(ui, wizard),
            Step::Pairs => self.pairs_body(ui, wizard),
            Step::Global => self.global_body(ui, wizard),
            Step::Merge => self.merge_body(ui, wizard),
        }
    }

    fn running_line(&self, ui: &mut egui::Ui, key: &str) {
        ui.horizontal(|ui| {
            ui.spinner();
            ui.label(t(key));
        });
    }

    // ------------------------------------------------------------------- pick

    fn pick_body(&self, ui: &mut egui::Ui, wizard: &Wizard) -> Option<Action> {
        let mut action = None;
        if self.session.doc.meshes.is_empty() {
            ui.weak(t("wizard.pick.none"));
            return None;
        }
        for mesh in &self.session.doc.meshes {
            let place = wizard.scans().iter().position(|s| *s == mesh.id);
            let label = match place {
                Some(n) => format!("{}. {}", n + 1, mesh.name),
                None => mesh.name.clone(),
            };
            let mut chosen = place.is_some();
            if ui.checkbox(&mut chosen, label).changed() {
                action = Some(Action::ToggleScan(mesh.id));
            }
        }
        ui.add_space(4.0);
        let n = wizard.scans().len();
        if n < 2 {
            ui.colored_label(self.tokens.color(Token::StatusWarning), t("wizard.pick.need_two"));
        } else {
            ui.weak(tf("wizard.pick.count", &[("n", &n.to_string())]));
        }
        action
    }

    // ------------------------------------------------------------------ pairs

    fn pairs_body(&self, ui: &mut egui::Ui, wizard: &Wizard) -> Option<Action> {
        let current = wizard.current_pair().map(|(i, _)| i);
        let mut action = None;
        for (i, pair) in wizard.pairs().iter().enumerate() {
            let head = tf(
                "wizard.pair.title",
                &[("object", &self.scan_name(pair.object)), ("target", &self.scan_name(pair.target))],
            );
            if current == Some(i) {
                ui.label(egui::RichText::new(head).strong());
                action = self.current_pair_body(ui, wizard, pair);
            } else {
                ui.horizontal(|ui| {
                    ui.label(head);
                    ui.weak(self.pair_state_text(&pair.state));
                });
            }
        }
        if wizard.pairs_resolved() {
            ui.add_space(4.0);
            ui.weak(t("wizard.pairs.done"));
        }
        action
    }

    fn pair_state_text(&self, state: &PairState) -> String {
        match state {
            PairState::Pending | PairState::Proposed(_) => t("wizard.pair.waiting").to_string(),
            PairState::Accepted(_) => t("wizard.pair.accepted").to_string(),
            PairState::Rejected => t("wizard.pair.rejected").to_string(),
            PairState::Failed(e) => tf("wizard.pair.failed", &[("error", &engine_message(e))]),
        }
    }

    fn current_pair_body(&self, ui: &mut egui::Ui, wizard: &Wizard, pair: &Pair) -> Option<Action> {
        let idle = wizard.running().is_none();
        if !idle {
            self.running_line(ui, "wizard.pair.running");
            return None;
        }
        let mut action = None;
        match &pair.state {
            PairState::Proposed(p) => {
                let (verdict_key, token) = match p.verdict.as_str() {
                    "unique" => ("wizard.pair.unique", Token::StatusOk),
                    "no_match" => ("wizard.pair.no_match", Token::StatusError),
                    _ => ("wizard.pair.ambiguous", Token::StatusWarning),
                };
                ui.colored_label(self.tokens.color(token), t(verdict_key));
                if let Some(pose) = p.chosen_pose() {
                    let overlap = [("moving", percent(pose.overlap_moving)), ("fixed", percent(pose.overlap_fixed))];
                    ui.weak(tf("wizard.pair.overlap", &[(overlap[0].0, &overlap[0].1), (overlap[1].0, &overlap[1].1)]));
                }
                ui.weak(t("wizard.pair.shown"));
                if p.poses.len() > 1 {
                    for k in 0..p.poses.len() {
                        let label = tf("wizard.pair.pose", &[("n", &(k + 1).to_string())]);
                        if ui.radio(p.chosen == k, label).clicked() {
                            action = Some(Action::ChoosePose(k));
                        }
                    }
                }
                ui.horizontal(|ui| {
                    let accept = egui::Button::new(t("wizard.accept"));
                    let accept =
                        if p.unique { crate::theme::primary_button(&self.tokens, t("wizard.accept")) } else { accept };
                    if ui.add_enabled(p.acceptable(), accept).on_hover_text(t("wizard.accept.hint")).clicked() {
                        action = Some(Action::AcceptPair);
                    }
                    if ui.button(t("wizard.reject")).on_hover_text(t("wizard.reject.hint")).clicked() {
                        action = Some(Action::RejectPair);
                    }
                });
            }
            PairState::Failed(e) => {
                let text = if e.is_empty() { t("wizard.problem.unreadable").to_string() } else { engine_message(e) };
                ui.colored_label(self.tokens.color(Token::StatusError), text);
                ui.horizontal(|ui| {
                    if ui.button(t("wizard.retry")).clicked() {
                        action = Some(Action::RetryPair);
                    }
                    if ui.button(t("wizard.reject")).on_hover_text(t("wizard.reject.hint")).clicked() {
                        action = Some(Action::RejectPair);
                    }
                });
            }
            _ => {}
        }
        action
    }

    // ----------------------------------------------------------------- global

    fn global_body(&self, ui: &mut egui::Ui, wizard: &Wizard) -> Option<Action> {
        let mut action = None;
        let applied = matches!(wizard.global(), GlobalState::Applied(_));
        ui.add_enabled_ui(wizard.running().is_none() && !applied, |ui| {
            ui.horizontal(|ui| {
                ui.label(t("wizard.global.start"));
                for (choice, key) in
                    [(StartChoice::Current, "wizard.global.current"), (StartChoice::Search, "wizard.global.search")]
                {
                    let hint = format!("{key}.hint");
                    if ui.radio(wizard.start() == choice, t(key)).on_hover_text(t(&hint)).clicked() {
                        action = Some(Action::SetStart(choice));
                    }
                }
            });
        });
        if wizard.running().is_some() {
            self.running_line(ui, "wizard.global.running");
            return None;
        }
        match wizard.global() {
            GlobalState::Idle => {}
            GlobalState::Proposed(r) => action = self.global_report_body(ui, wizard, r, false).or(action),
            GlobalState::Applied(r) => action = self.global_report_body(ui, wizard, r, true).or(action),
        }
        action
    }

    fn global_report_body(
        &self,
        ui: &mut egui::Ui,
        wizard: &Wizard,
        r: &GlobalReport,
        applied: bool,
    ) -> Option<Action> {
        let mut action = None;
        let (verdict_key, token) = match r.verdict.as_str() {
            "aligned" => ("wizard.global.aligned", Token::StatusOk),
            "failed" => ("wizard.global.failed", Token::StatusError),
            _ => ("wizard.global.partial", Token::StatusWarning),
        };
        ui.colored_label(self.tokens.color(token), t(verdict_key));
        for issue in &r.issues {
            ui.weak(t(&format!("wizard.global.issue.{issue}")));
        }
        self.loop_line(ui, r);
        ui.weak(tf(
            "wizard.global.edges",
            &[("used", &r.used_edges.to_string()), ("rejected", &r.rejected_edges.to_string())],
        ));
        ui.add_space(2.0);
        for scan in &r.scans {
            let name = self.scan_at(wizard, scan.index);
            let state_key = match (scan.fixed, scan.placed, scan.held_by_loop) {
                (true, ..) => "wizard.global.scan.fixed",
                (_, false, _) => "wizard.global.scan.unplaced",
                (_, _, true) => "wizard.global.scan.held",
                _ => "wizard.global.scan.weak",
            };
            ui.horizontal(|ui| {
                ui.label(name);
                ui.weak(t(state_key));
            });
            if scan.weakly_held() {
                ui.indent(("weak-scan", scan.index), |ui| {
                    if let Some(edge) = r.edges.iter().find(|e| e.moving == scan.index || e.fixed == scan.index) {
                        let numbers = [("rms", millimetres(edge.rms)), ("overlap", percent(edge.overlap_moving))];
                        ui.weak(tf(
                            "wizard.global.weak.evidence",
                            &[(numbers[0].0, &numbers[0].1), (numbers[1].0, &numbers[1].1)],
                        ));
                    }
                    let mut accepted = wizard.accepted_weak().contains(&scan.index);
                    if ui.checkbox(&mut accepted, t("wizard.global.weak.accept")).changed() {
                        action = Some(Action::ToggleWeak(scan.index));
                    }
                });
            }
        }
        ui.add_space(4.0);
        if applied {
            ui.colored_label(
                self.tokens.color(Token::StatusOk),
                tf("wizard.global.applied", &[("n", &r.moved.len().to_string())]),
            );
        } else {
            ui.weak(t("wizard.global.preview"));
            let apply = crate::theme::primary_button(&self.tokens, t("wizard.apply"));
            if ui.add_enabled(!r.failed(), apply).on_hover_text(t("wizard.apply.hint")).clicked() {
                action = Some(Action::ApplyGlobal);
            }
            ui.weak(t("wizard.global.skip"));
        }
        action
    }

    fn loop_line(&self, ui: &mut egui::Ui, r: &GlobalReport) {
        if r.loops.cycles == 0 {
            ui.weak(t("wizard.global.no_loop"));
            return;
        }
        let numbers = [
            ("before", millimetres(r.loops.worst_before_mm)),
            ("after", millimetres(r.loops.worst_after_mm)),
            ("cycles", r.loops.cycles.to_string()),
        ];
        let args: Vec<(&str, &str)> = numbers.iter().map(|(k, v)| (*k, v.as_str())).collect();
        ui.label(tf("wizard.global.loop", &args));
    }

    // ------------------------------------------------------------------ merge

    fn merge_body(&self, ui: &mut egui::Ui, wizard: &Wizard) -> Option<Action> {
        if wizard.running().is_some() {
            self.running_line(ui, "wizard.merge.running");
            return None;
        }
        let mut action = None;
        let done = matches!(wizard.merge(), MergeState::Done(_));
        ui.add_enabled_ui(!done, |ui| {
            let options = wizard.options();
            let mut tolerance = options.tolerance.clone();
            ui.horizontal(|ui| {
                ui.label(t("wizard.merge.tolerance"));
                if ui.add(egui::TextEdit::singleline(&mut tolerance).desired_width(80.0)).changed() {
                    action = Some(Action::SetTolerance(tolerance.clone()));
                }
            });
            let mut name = options.name.clone();
            ui.horizontal(|ui| {
                ui.label(t("wizard.merge.name"));
                if ui
                    .add(
                        egui::TextEdit::singleline(&mut name)
                            .hint_text(t("wizard.merge.name_hint"))
                            .desired_width(120.0),
                    )
                    .changed()
                {
                    action = Some(Action::SetName(name.clone()));
                }
            });
            let mut warn = options.warn;
            if ui.checkbox(&mut warn, t("wizard.merge.warn")).on_hover_text(t("wizard.merge.warn.hint")).changed() {
                action = Some(Action::SetWarn(warn));
            }
        });
        ui.add_space(4.0);
        match wizard.merge() {
            MergeState::Idle => {
                let merge = crate::theme::primary_button(&self.tokens, t("wizard.merge.run"));
                if ui.add(merge).on_hover_text(t("wizard.merge.run.hint")).clicked() {
                    action = Some(Action::Merge);
                }
            }
            MergeState::Refused(r) => {
                action = self.refusal_body(ui, wizard, r).or(action);
            }
            MergeState::Done(r) => {
                action = self.merged_body(ui, wizard, r).or(action);
            }
        }
        action
    }

    /// One finding of the merge: what it is about in a few words, then the engine's message.
    fn issue_lines(
        &self,
        ui: &mut egui::Ui,
        wizard: &Wizard,
        issue: &crate::multiscan::report::MergeIssue,
        token: Token,
    ) {
        let key = format!("wizard.merge.code.{}", issue.code);
        let headline = crate::i18n::find(&key).unwrap_or(issue.code.as_str());
        let scans: Vec<String> = issue.scans.iter().map(|i| self.scan_at(wizard, *i)).collect();
        ui.colored_label(self.tokens.color(token), format!("{headline} ({})", scans.join(", ")));
        ui.weak(engine_message(&issue.message));
    }

    fn refusal_body(&self, ui: &mut egui::Ui, wizard: &Wizard, r: &MergeReport) -> Option<Action> {
        let mut action = None;
        ui.colored_label(self.tokens.color(Token::StatusError), t("wizard.merge.refused"));
        for issue in r.refusals() {
            self.issue_lines(ui, wizard, issue, Token::StatusError);
        }
        ui.add_space(2.0);
        if r.warn_would_help() {
            ui.label(t("wizard.merge.help.misaligned"));
        } else {
            ui.label(t("wizard.merge.help.fatal"));
        }
        if ui.button(t("wizard.merge.run")).clicked() {
            action = Some(Action::Merge);
        }
        action
    }

    fn merged_body(&self, ui: &mut egui::Ui, wizard: &Wizard, r: &MergeReport) -> Option<Action> {
        let mut action = None;
        let numbers = [
            ("triangles", r.triangles.to_string()),
            ("pieces", r.pieces.to_string()),
            ("tolerance", millimetres(r.tolerance)),
        ];
        let args: Vec<(&str, &str)> = numbers.iter().map(|(k, v)| (*k, v.as_str())).collect();
        ui.colored_label(self.tokens.color(Token::StatusOk), tf("wizard.merge.done", &args));
        if r.open_edges > 0 {
            let n = r.open_edges.to_string();
            ui.colored_label(self.tokens.color(Token::StatusWarning), tf("wizard.merge.open_edges", &[("n", &n)]));
        }
        for issue in r.issues.iter().filter(|i| i.severity == "warning") {
            self.issue_lines(ui, wizard, issue, Token::StatusWarning);
        }
        ui.add_space(2.0);
        if ui.button(t("wizard.merge.retry")).on_hover_text(t("wizard.merge.retry.hint")).clicked() {
            action = Some(Action::RetryMerge);
        }
        action
    }
}

//! Running commands from the UI: at once for the quick ones, in the background for the heavy
//! ones (`surfacecraft_engine::cmd::is_heavy`), so the window keeps painting while one runs.
//!
//! A heavy command runs on a fork of the session on another thread (`surfacecraft_engine::Job`).
//! The app keeps its own session meanwhile and draws it: the document from before the command.
//! When the job is done its session replaces the app's, with the log entry, undo step and status
//! line a run in place gives. While a job runs nothing else may start: a dialog over the window
//! says what is running and for how long, and offers Cancel, which abandons the job (the app's
//! session never changed) and asks the command to stop.
//!
//! Where there is no other thread (a browser without threads, or a page that is not cross-origin
//! isolated) a heavy command runs in place after the dialog has been painted with a warning, so
//! the user knows why the page stops answering.
use crate::SurfaceCraftApp;
use crate::i18n::{t, tf};
use crate::theme::Token;
use serde_json::Value;
use surfacecraft_engine::{EngineError, Job, JobPoll, Session, cmd};

/// How often the UI looks whether the job is done, in seconds (it never waits for it).
const POLL_SECONDS: f32 = 0.05;
/// How long the warning of a run in place stays up before the run starts and the UI stops, in
/// seconds: long enough for the dialog to have faded in and been read as a dialog.
const WARNING_SECONDS: f64 = 0.6;
/// Width of the dialog shown while a command runs.
const BUSY_WIDTH: f32 = 320.0;

/// What a control wants done once the command it launched has finished.
pub(crate) enum After {
    /// Nothing besides the status line.
    Nothing,
    /// Close the open tool when the command changed the document (the tool panel's OK).
    CloseTool,
    /// Frame the document in the viewport (an import).
    Fit,
    /// Offer the bytes the command returned as a download under this file name.
    #[cfg(target_arch = "wasm32")]
    Download(&'static str),
    /// Answer a request of the control channel.
    #[cfg(not(target_arch = "wasm32"))]
    Reply(std::sync::mpsc::Sender<Value>),
}

/// Where the running command runs.
enum How {
    /// On another thread, on a fork of the session.
    Background(Job),
    /// In place, once the warning has been up for [`WARNING_SECONDS`].
    InPlace,
}

/// The command the app is busy with.
pub(crate) struct Busy {
    id: String,
    /// The tool's name, as the ribbon shows it.
    name: String,
    params: Value,
    /// Objects in the document when the command started.
    had: usize,
    /// When the UI first saw the command running (egui time, seconds).
    started: Option<f64>,
    after: After,
    how: How,
}

impl SurfaceCraftApp {
    /// Executes a command in place, the way every quick UI control does, updating the status
    /// line. Refused while a background command runs.
    pub fn run_command(&mut self, id: &str, params: Value) -> Result<Value, EngineError> {
        if let Some(busy) = &self.busy {
            self.status = tf("status.busy", &[("name", &busy.name)]);
            return Err(EngineError::Unavailable(format!("{} is running", busy.id)));
        }
        let had = self.session.doc.summaries().len();
        let r = self.session.execute(id, params);
        self.settle(id, had, &r);
        r
    }

    /// What follows a command whichever way it ran: the status line, and what the UI shows next.
    fn settle(&mut self, id: &str, had: usize, r: &Result<Value, EngineError>) {
        let name = tool_name(id);
        match r {
            Ok(_) => {
                self.status = tf("status.done", &[("name", &name)]);
                if had == 0 && !self.session.doc.summaries().is_empty() {
                    self.fit_pending = true;
                }
                if id == "deviation.compute" {
                    // a new map is what the Deviation tab shows
                    crate::dock::show_tab(&mut self.workspace, crate::dock::Panel::Deviation);
                }
            }
            Err(e) => {
                let error = crate::i18n::engine_message(&e.to_string());
                self.status = tf("status.failed", &[("name", &name), ("error", &error)]);
            }
        }
    }

    /// Runs the open tool with its edited parameters. A command that changed the document closes
    /// its tool (the guide shows what comes next); a failed one, or one that only reports
    /// (statistics, information), stays open with its message or result. Returns whether it ran
    /// without an error, or was started in the background.
    pub fn run_palette(&mut self) -> bool {
        self.field_pick = None;
        let Some(id) = self.palette_command else {
            self.status = t("status.choose_command").to_string();
            return false;
        };
        match serde_json::from_str::<Value>(&self.palette_params) {
            Ok(v) => self.launch(id, v, After::CloseTool),
            Err(e) => {
                self.status = tf("status.params_not_json", &[("error", &e.to_string())]);
                false
            }
        }
    }

    /// Is a command running (in the background, or about to run in place)?
    #[must_use]
    pub fn is_busy(&self) -> bool {
        self.busy.is_some()
    }

    /// Runs a command for a control: a heavy one in the background when
    /// [`SurfaceCraftApp::background_jobs`] is on, anything else in place. `after` happens once
    /// it has finished. Returns whether it ran without an error or was started; false also when
    /// another command is running.
    pub(crate) fn launch(&mut self, id: &str, params: Value, after: After) -> bool {
        if let Some(busy) = &self.busy {
            self.status = tf("status.busy", &[("name", &busy.name)]);
            self.finish(after, id, Err(EngineError::Unavailable(format!("{} is running", busy.id))));
            return false;
        }
        if !(self.background_jobs && cmd::is_heavy(id)) {
            let r = self.run_command(id, params);
            let ok = r.is_ok();
            self.finish(after, id, r);
            return ok;
        }
        let how = match Job::start(&self.session, id, params.clone()) {
            Some(job) => How::Background(job),
            None => How::InPlace,
        };
        let name = tool_name(id);
        self.status = tf("status.running", &[("name", &name)]);
        self.field_pick = None;
        self.cancel_interaction();
        let had = self.session.doc.summaries().len();
        self.busy = Some(Busy { id: id.to_string(), name, params, had, started: None, after, how });
        true
    }

    /// Moves the running command on: takes the result of a finished job, or starts a run in
    /// place whose warning has been shown. `now` is the UI clock in seconds. Returns whether a
    /// command is still running (the caller then asks for another frame soon). Never blocks,
    /// except for the run in place itself.
    pub fn advance_job(&mut self, now: f64) -> bool {
        let Some(mut busy) = self.busy.take() else { return false };
        let waited = now - *busy.started.get_or_insert(now);
        let result = match &mut busy.how {
            How::Background(job) => match job.poll() {
                JobPoll::Running => None,
                JobPoll::Finished(done) => {
                    self.adopt(done.session);
                    Some(done.result)
                }
                JobPoll::Lost => Some(Err(EngineError::Internal(busy.id.clone()))),
            },
            How::InPlace if waited < WARNING_SECONDS => None,
            How::InPlace => {
                let had = self.session.doc.summaries().len();
                busy.had = had;
                Some(self.session.execute(&busy.id, busy.params.clone()))
            }
        };
        let Some(result) = result else {
            self.busy = Some(busy);
            return true;
        };
        self.settle(&busy.id, busy.had, &result);
        self.finish(busy.after, &busy.id, result);
        false
    }

    /// The session a finished job hands back becomes the app's.
    fn adopt(&mut self, session: Session) {
        self.session = session;
        self.refresh_highlights();
    }

    /// Abandons the running command: the app's session is the state from before it, the log
    /// says it was cancelled, and the command is asked to stop (see `Job::cancel` for which do).
    pub fn cancel_job(&mut self) {
        let Some(busy) = self.busy.take() else { return };
        if let How::Background(job) = &busy.how {
            job.cancel();
        }
        self.session.note_cancelled(&busy.id, busy.params);
        self.status = tf("status.cancelled", &[("name", &busy.name)]);
        self.finish(busy.after, &busy.id, Err(EngineError::Failed(cmd::CANCELLED.into())));
    }

    /// What the control that launched the command asked for.
    fn finish(&mut self, after: After, id: &str, result: Result<Value, EngineError>) {
        match after {
            After::Nothing => {}
            After::CloseTool => {
                if result.is_ok() && cmd::find(id).is_some_and(|c| c.undoable) {
                    self.close_tool();
                }
            }
            After::Fit => self.fit_pending |= result.is_ok(),
            #[cfg(target_arch = "wasm32")]
            After::Download(file) => {
                if let Ok(v) = result
                    && let Some(Ok(bytes)) = v.get("data").and_then(Value::as_str).map(surfacecraft_doc::base64::decode)
                {
                    crate::web::download(file, &bytes);
                }
            }
            #[cfg(not(target_arch = "wasm32"))]
            After::Reply(reply) => {
                let _ = reply.send(match result {
                    Ok(v) => serde_json::json!({"ok": v}),
                    Err(e) => serde_json::json!({"err": e}),
                });
            }
        }
    }

    /// The dialog over the window while a command runs: what runs, for how long, and Cancel.
    /// It takes the pointer and the keyboard, so nothing else can be started meanwhile.
    pub(crate) fn busy_overlay(&mut self, ctx: &egui::Context) {
        let Some(busy) = &self.busy else { return };
        ctx.request_repaint_after(std::time::Duration::from_secs_f32(POLL_SECONDS));
        let now = ctx.input(|i| i.time);
        let seconds = (now - busy.started.unwrap_or(now)).max(0.0) as u64;
        let in_place = matches!(busy.how, How::InPlace);
        let warning = self.tokens.color(Token::StatusError);
        let mut cancel = false;
        egui::Modal::new(egui::Id::unique("surfacecraft-busy")).frame(crate::theme::card_frame(&self.tokens)).show(
            ctx,
            |ui| {
                ui.set_width(BUSY_WIDTH);
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.strong(tf("busy.running", &[("name", &busy.name)]));
                });
                if in_place {
                    ui.colored_label(warning, t("busy.in_place"));
                    return;
                }
                ui.weak(tf("busy.elapsed", &[("seconds", &seconds.to_string())]));
                ui.weak(t("busy.hint"));
                ui.add_space(4.0);
                cancel = ui.button(t("busy.cancel")).on_hover_text(t("busy.cancel_hint")).clicked();
            },
        );
        if cancel {
            self.cancel_job();
        }
    }
}

/// The name of command `id` as the ribbon shows it (the id itself for an unknown command).
fn tool_name(id: &str) -> String {
    cmd::find(id).map_or_else(|| id.to_string(), crate::workflow::label_of)
}

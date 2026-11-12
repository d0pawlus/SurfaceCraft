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
//!
//! Whatever changes the session goes through [`SurfaceCraftApp::idle_session`], which refuses
//! while a command runs; what feeds commands in from outside the window (the control channel,
//! files picked in the browser) waits for the running command instead of being dropped.
use crate::SurfaceCraftApp;
use crate::i18n::{t, tf};
use crate::theme::Token;
use serde_json::{Value, json};
use std::sync::Arc;
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
    /// Shared with the job: the parameters may hold a whole file and are not copied here.
    params: Arc<Value>,
    /// Objects in the document when the command started.
    had: usize,
    /// When the UI first saw the command running (egui time, seconds).
    started: Option<f64>,
    after: After,
    how: How,
}

impl SurfaceCraftApp {
    /// The session, for whatever changes it: refused (with the reason on the status line) while
    /// a command runs, because the session the app holds then is only what it draws, and the
    /// running command's session will replace it. Every `&mut` use of the session outside
    /// drawing goes through here.
    pub(crate) fn idle_session(&mut self) -> Result<&mut Session, EngineError> {
        if let Some(busy) = &self.busy {
            self.status = tf("status.busy", &[("name", &busy.name)]);
            return Err(EngineError::Unavailable(format!("{} is running", busy.id)));
        }
        Ok(&mut self.session)
    }

    /// Executes a command in place, the way every quick UI control does, updating the status
    /// line. Refused while a background command runs.
    pub fn run_command(&mut self, id: &str, params: Value) -> Result<Value, EngineError> {
        let had = self.session.doc.summaries().len();
        let r = self.idle_session()?.execute(id, params);
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
            Ok(v) => {
                if let Some(units) = crate::import_units::chosen(id, &v) {
                    self.settings.import_units = units;
                }
                self.launch(id, v, After::CloseTool)
            }
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
        if self.still_finishing() {
            self.status = t("status.finishing").to_string();
            let refused = EngineError::Unavailable("the cancelled command is still finishing".into());
            self.finish(after, id, Err(refused));
            return false;
        }
        let params = Arc::new(params);
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

    /// Is the thread of a cancelled command still at work? A command that does not look at the
    /// cancellation flag runs to its end (its result is dropped); until then no other heavy
    /// command starts, so abandoned work cannot pile up or, in the browser, hold the pool's
    /// workers while a new command waits behind it.
    fn still_finishing(&mut self) -> bool {
        if self.finishing.as_ref().is_some_and(Job::is_running) {
            return true;
        }
        self.finishing = None;
        false
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
                busy.had = self.session.doc.summaries().len();
                // `busy` was taken above, so the session is idle; nobody else holds the parameters
                let params = Arc::unwrap_or_clone(std::mem::take(&mut busy.params));
                Some(self.idle_session().and_then(|session| session.execute(&busy.id, params)))
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
        if let How::Background(job) = busy.how {
            job.cancel();
            self.finishing = Some(job);
        }
        self.session.note_cancelled(&busy.id, Arc::unwrap_or_clone(busy.params));
        self.status = tf("status.cancelled", &[("name", &busy.name)]);
        self.finish(busy.after, &busy.id, Err(EngineError::Failed(cmd::CANCELLED.into())));
    }

    /// What the control that launched the command asked for.
    fn finish(&mut self, after: After, id: &str, result: Result<Value, EngineError>) {
        if let Ok(v) = &result
            && matches!(id, "mesh.import" | "mesh.set_units")
            && let Some(text) = crate::import_units::warning_text(v)
        {
            self.status = text;
        }
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
                    Ok(v) => json!({"ok": v}),
                    Err(e) => json!({"err": e}),
                });
            }
        }
    }

    /// Feeds this app from a control channel (the desktop app opens one; tests bring their own).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn attach_control(&mut self, requests: std::sync::mpsc::Receiver<crate::control::Request>) {
        self.control = Some(requests);
    }

    /// Answers control-channel requests (bridged MCP) between frames.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn poll_control(&mut self) {
        // one request at a time: a heavy command runs in the background and the requests behind
        // it wait in the channel until it has answered
        while !self.is_busy() {
            let Some(r) = self.control.as_ref().and_then(|rx| rx.try_recv().ok()) else { return };
            let reply = match r.method.as_str() {
                "engine.execute" => {
                    let cmd = r.params.get("command").and_then(Value::as_str).unwrap_or("").to_string();
                    let params = r.params.get("params").cloned().unwrap_or(Value::Null);
                    self.launch(&cmd, params, After::Reply(r.reply));
                    continue;
                }
                "ui.screenshot" => {
                    let dim = |k: &str, d: u64| r.params.get(k).and_then(Value::as_u64).unwrap_or(d) as u32;
                    let cam = self.camera.clone();
                    let (w, h) = (dim("width", 800), dim("height", 600));
                    match self.idle_session().map(|session| crate::shot::screenshot_png(session, Some(&cam), w, h)) {
                        Ok(Some(png)) => json!({"png": surfacecraft_doc::base64::encode(&png)}),
                        Ok(None) => json!({"err": "no renderer"}),
                        Err(e) => json!({"err": e}),
                    }
                }
                other => json!({"err": format!("unknown method {other}")}),
            };
            let _ = r.reply.send(reply);
        }
    }

    /// Imports files the browser picked (web build).
    #[cfg(target_arch = "wasm32")]
    pub(crate) fn poll_inbox(&mut self) {
        // one file at a time: an import may run in the background, and the next waits for it
        while !self.is_busy() {
            let mut inbox = self.inbox.borrow_mut();
            if inbox.is_empty() {
                return;
            }
            let (name, bytes) = inbox.remove(0);
            drop(inbox);
            let lower = name.to_lowercase();
            let ext = lower.rsplit('.').next().unwrap_or("").to_string();
            let stem = name.rsplit_once('.').map_or(name.as_str(), |(s, _)| s).to_string();
            let data = surfacecraft_doc::base64::encode(&bytes);
            let units = self.settings.import_units;
            let (id, params) = if ext == "step" || ext == "stp" {
                ("import.step", json!({"data": data, "name": stem}))
            } else {
                ("mesh.import", json!({"data": data, "format": ext, "name": stem, "units": units}))
            };
            self.launch(id, params, After::Fit);
        }
    }

    /// Downloads the selected object: a mesh as STL, a body or surface as STEP (web build).
    #[cfg(target_arch = "wasm32")]
    pub(crate) fn download_selected(&mut self) {
        let Some(id) = self.selected_object else {
            self.status = t("status.select_first").to_string();
            return;
        };
        let (cmd, params, file) = match self.session.doc.kind_of(id) {
            Some(surfacecraft_doc::ObjectKind::Mesh) => {
                ("mesh.export", json!({"object": id.0, "format": "stl"}), "mesh.stl")
            }
            Some(surfacecraft_doc::ObjectKind::Body | surfacecraft_doc::ObjectKind::Surface) => {
                ("export.step", json!({"objects": [id.0]}), "model.step")
            }
            _ => {
                self.status = t("status.download_kinds").to_string();
                return;
            }
        };
        self.launch(cmd, params, After::Download(file));
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

//! Reducing a large scan at import. A file picked for import whose header declares more than
//! [`OFFER_ABOVE`] triangles is not imported at once: a dialog says how large it is and offers to
//! reduce it while it is read (`mesh.import` with `reduce_to`), to import it in full where this
//! platform can hold it, or to leave it. In the browser, whose import takes far fewer triangles
//! than the desktop's, this is the only way such a scan comes in.
//!
//! Only imports the user starts from the interface ask; a command (the form, a script, MCP) never
//! opens the dialog and states `reduce_to` itself.
//!
//! Contents picked without a path (the browser) reach `mesh.import` as a picked file of the
//! session (`surfacecraft_engine::picked`), never as base64 data.
use crate::SurfaceCraftApp;
use crate::i18n::{t, tf};
use serde_json::{Value, json};
use surfacecraft_engine::cmd::mesh::{MIN_REDUCE_TO, declared_triangles, import_triangle_limit};

/// A file that declares more triangles than this is offered a reduction.
pub const OFFER_ABOVE: u64 = 5_000_000;
/// The reduction the dialog starts with, triangles.
pub const DEFAULT_REDUCE_TO: usize = 2_000_000;
/// Bytes of a file read to find what its header declares.
#[cfg(not(target_arch = "wasm32"))]
const HEAD_BYTES: u64 = 1 << 20;
/// Width of the dialog.
const OFFER_WIDTH: f32 = 360.0;
/// Triangles one step of the dialog's number field changes.
const TARGET_STEP: f64 = 10_000.0;

/// What the user answers the offer with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImportChoice {
    /// Import reduced to this many triangles.
    Reduce(usize),
    /// Import every triangle (only where the platform can hold the file).
    Full,
    /// Do not import.
    Cancel,
}

/// A picked file waiting for the user's answer.
pub(crate) struct PendingImport {
    /// The import's parameters without the file's contents.
    params: Value,
    /// The file's name and contents, when it was not picked by path (the browser).
    bytes: Option<(String, Vec<u8>)>,
    /// Triangles the header declares.
    triangles: u64,
    /// The reduction the dialog shows.
    reduce_to: usize,
}

/// What the offer says, for the tests and the dialog.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Offer {
    /// Triangles the file declares.
    pub triangles: u64,
    /// The reduction proposed, triangles.
    pub reduce_to: usize,
    /// Can the file be imported in full here?
    pub full_allowed: bool,
}

/// A number with its thousands apart, as the dialog and the status line write triangle counts.
#[must_use]
pub fn grouped(n: u64) -> String {
    let digits = n.to_string();
    let mut out = String::with_capacity(digits.len() + digits.len() / 3);
    for (i, c) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i).is_multiple_of(3) {
            out.push('\u{a0}');
        }
        out.push(c);
    }
    out
}

/// What a finished import says about its reduction, when it reduced the file.
#[must_use]
pub fn result_note(result: &Value) -> Option<String> {
    let reduced = result.get("reduced")?;
    let from = reduced.get("source_triangles")?.as_u64()?;
    let to = result.get("triangles")?.as_u64()?;
    let error = reduced.get("max_error")?.as_f64()?;
    Some(tf("reduce.done", &[("from", &grouped(from)), ("to", &grouped(to)), ("error", &format!("{error:.3}"))]))
}

/// The leading bytes of the file at `path` (empty when it cannot be read: the import says why).
#[cfg(not(target_arch = "wasm32"))]
fn head_of(path: &str) -> Vec<u8> {
    use std::io::Read;
    let mut head = Vec::new();
    if let Ok(file) = std::fs::File::open(path) {
        // a failed read leaves what was read: the header is then simply not recognised
        let _ = file.take(HEAD_BYTES).read_to_end(&mut head);
    }
    head
}

fn extension_of(name: &str) -> String {
    name.rsplit_once('.').map_or("", |(_, e)| e).to_ascii_lowercase()
}

impl SurfaceCraftApp {
    /// Imports the mesh file the user picked at `path`, or asks first when it is large.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn import_picked_path(&mut self, path: &str) {
        let declared = declared_triangles(&head_of(path), &extension_of(path));
        self.import_or_offer(json!({"path": path}), None, declared);
    }

    /// Imports the mesh file `name` the user picked, given by its contents (the browser's file
    /// picker), or asks first when it is large.
    pub fn import_picked_bytes(&mut self, name: &str, bytes: Vec<u8>) {
        let ext = extension_of(name);
        let stem = name.rsplit_once('.').map_or(name, |(s, _)| s);
        let declared = declared_triangles(&bytes, &ext);
        self.import_or_offer(json!({"format": ext, "name": stem}), Some((name.to_string(), bytes)), declared);
    }

    fn import_or_offer(&mut self, params: Value, bytes: Option<(String, Vec<u8>)>, declared: Option<u64>) {
        // the file already waiting for its answer is never replaced: this one is refused, in words
        if self.pending_import.is_some() {
            self.status = tf("status.busy", &[("name", t("reduce.title"))]);
            return;
        }
        let pending = PendingImport {
            params,
            bytes,
            triangles: declared.unwrap_or(0),
            reduce_to: DEFAULT_REDUCE_TO.min(import_triangle_limit()),
        };
        if pending.triangles > OFFER_ABOVE {
            self.pending_import = Some(pending);
        } else {
            self.start_import(pending, None);
        }
    }

    /// Runs the import. Picked contents go to the session as a picked file (a `picked:` path),
    /// not into the parameters as base64: the file is then held once, and the log keeps its path.
    fn start_import(&mut self, pending: PendingImport, reduce_to: Option<usize>) {
        let mut params = pending.params;
        let picked = pending.bytes.map(|(name, bytes)| self.session.picked.put(&name, bytes));
        if let Some(path) = &picked {
            params["path"] = json!(path);
        }
        if let Some(target) = reduce_to {
            params["reduce_to"] = json!(target);
        }
        let started = self.import_mesh_file(params);
        // a refused import (another command runs) never reads its file: it is not kept
        if let Some(path) = picked.filter(|_| !started) {
            self.session.picked.discard(&path);
        }
    }

    /// The offer waiting for an answer, when a large file was picked.
    #[must_use]
    pub fn import_offer(&self) -> Option<Offer> {
        self.pending_import.as_ref().map(|p| Offer {
            triangles: p.triangles,
            reduce_to: p.reduce_to,
            full_allowed: p.triangles <= import_triangle_limit() as u64,
        })
    }

    /// Answers the offer: imports the waiting file reduced or in full, or drops it.
    pub fn answer_import_offer(&mut self, choice: ImportChoice) {
        let Some(pending) = self.pending_import.take() else { return };
        match choice {
            ImportChoice::Reduce(target) => {
                let target = target.clamp(MIN_REDUCE_TO, import_triangle_limit());
                self.start_import(pending, Some(target));
            }
            ImportChoice::Full => self.start_import(pending, None),
            ImportChoice::Cancel => {}
        }
    }

    /// The dialog of the offer, over the window.
    pub(crate) fn import_offer_dialog(&mut self, ctx: &egui::Context) {
        let Some(offer) = self.import_offer() else { return };
        let limit = import_triangle_limit();
        let mut reduce_to = offer.reduce_to;
        let mut choice = None;
        let tokens = self.tokens.clone();
        egui::Modal::new(egui::Id::unique("surfacecraft-import-reduce")).frame(crate::theme::card_frame(&tokens)).show(
            ctx,
            |ui| {
                ui.set_width(OFFER_WIDTH);
                ui.strong(t("reduce.title"));
                ui.label(tf("reduce.text", &[("triangles", &grouped(offer.triangles))]));
                let why = if offer.full_allowed {
                    t("reduce.optional").to_string()
                } else {
                    tf("reduce.over_limit", &[("limit", &grouped(limit as u64))])
                };
                ui.label(why);
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    let label = ui.label(t("reduce.target"));
                    ui.add(egui::DragValue::new(&mut reduce_to).range(MIN_REDUCE_TO..=limit).speed(TARGET_STEP))
                        .labelled_by(label.id);
                });
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    if ui.add(crate::theme::primary_button(&tokens, t("reduce.confirm"))).clicked() {
                        choice = Some(ImportChoice::Reduce(reduce_to));
                    }
                    if offer.full_allowed && ui.button(t("reduce.full")).clicked() {
                        choice = Some(ImportChoice::Full);
                    }
                    if ui.button(t("reduce.cancel")).clicked() {
                        choice = Some(ImportChoice::Cancel);
                    }
                });
            },
        );
        if let Some(p) = self.pending_import.as_mut() {
            p.reduce_to = reduce_to;
        }
        if let Some(choice) = choice {
            self.answer_import_offer(choice);
        }
    }
}

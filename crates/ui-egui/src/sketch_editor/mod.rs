//! The sketch editing mode (G5): draw on the sketch plane with the mouse, constrain by clicking,
//! dimension by typing a value, and see the solver's status live.
//!
//! The editor is a plain state machine over sketch coordinates ([`SketchEditor`]): a click, a
//! typed value or a key goes in, a list of [`Cmd`]s comes out. Every [`Cmd`] is one `sketch.*`
//! engine command (`sketch.add`, `sketch.constrain`, `sketch.edit_constraint`), run by the app
//! like any other action, so one gesture is one command is one undo step; `Esc` cancels the
//! gesture in progress without a command. The state machine knows nothing about egui or the GPU
//! and is tested without them; [`ui`] draws the overlay and feeds the pointer in.
mod constrain;
mod dimension;
pub mod drag;
mod gesture;
mod marks;
mod preview;
pub mod snap;
mod status;
mod tool;
mod ui;
pub mod view;

pub use dimension::{DimKind, Pending, Readout, fmt_value, parse_value};
pub use marks::{constraint_anchors, constraint_entities, constraint_mark};
pub use snap::{Hit, Snap, SnapConfig, SnapKind, Want};
pub use status::{StatusView, status_view};
pub use tool::Tool;

use serde_json::Value;
use surfacecraft_doc::ObjectId;

/// One engine command the editor wants run.
#[derive(Clone, Debug, PartialEq)]
pub struct Cmd {
    /// The command id (`sketch.add`, `sketch.constrain`, `sketch.edit_constraint`).
    pub id: &'static str,
    /// Its parameters.
    pub params: Value,
}
/// The state of the sketch editing mode.
#[derive(Clone, Debug)]
pub struct SketchEditor {
    /// The sketch being edited (`None`: not editing).
    pub target: Option<ObjectId>,
    /// The active tool.
    pub tool: Tool,
    /// Which snaps are on.
    pub snap: SnapConfig,
    /// Points placed by the gesture in progress (sketch coordinates).
    pub gesture: Vec<[f64; 2]>,
    /// Entities and points picked by the constraint tool in progress.
    pub picks: Vec<Hit>,
    /// A dimension waiting for its value.
    pub pending: Option<Pending>,
    /// A number typed while drawing (dynamic input).
    pub typed: String,
    /// The snapped cursor position.
    pub hover: Option<Snap>,
    /// The catalogue key of the last problem (nothing drawn: collinear arc, bad value).
    pub notice: Option<&'static str>,
    /// Where the dimension labels were drawn this frame (screen points), by constraint index.
    pub labels: Vec<(usize, [f32; 4])>,
    /// Where the editor's own controls were drawn this frame (screen points, `[x0, y0, x1, y1]`):
    /// clicks there are not clicks on the sketch.
    pub chrome: Vec<[f32; 4]>,
    /// The strip's details (snap options, hints, the constraint list) are shown.
    pub details_open: bool,
    /// A point being dragged with the mouse (see [`drag`]).
    pub drag: Option<drag::Drag>,
}

impl Default for SketchEditor {
    fn default() -> Self {
        Self {
            target: None,
            tool: Tool::Select,
            snap: SnapConfig::default(),
            gesture: Vec::new(),
            picks: Vec::new(),
            pending: None,
            typed: String::new(),
            hover: None,
            notice: None,
            labels: Vec::new(),
            chrome: Vec::new(),
            details_open: false,
            drag: None,
        }
    }
}

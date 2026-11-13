//! The sketch editor (G5) without a GPU: the tool state machines, the snapping maths, constraint
//! picking, dimension editing and the solver status, then the same through the real UI with
//! simulated pointer events. Every action must reach the engine as one `sketch.*` command.
#[path = "sketch_editor/common.rs"]
mod common;
#[path = "sketch_editor/constraints.rs"]
mod constraints;
#[path = "sketch_editor/dimensions.rs"]
mod dimensions;
#[path = "sketch_editor/drag.rs"]
mod drag;
#[path = "sketch_editor/drag_ui.rs"]
mod drag_ui;
#[path = "sketch_editor/drawing.rs"]
mod drawing;
#[path = "sketch_editor/mouse_ui.rs"]
mod mouse_ui;
#[path = "sketch_editor/plane_status.rs"]
mod plane_status;
#[path = "sketch_editor/snap_pick.rs"]
mod snap_pick;

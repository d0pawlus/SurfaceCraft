//! SurfaceCraft egui front end (layer L5), shared by the desktop and web apps.
//!
//! Every action in the UI (menus, ribbon, tool panel) is a command executed through
//! [`surfacecraft_engine::Session::execute`]; the log panel shows each call as the JSON a
//! `surfacecraft-cli run` script or an MCP client would send.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

pub mod app;
pub mod assistant;
pub mod brep;
pub mod camera;
pub mod command_panel;
#[cfg(not(target_arch = "wasm32"))]
pub mod control;
pub mod deviation_panel;
pub mod dock;
pub mod fieldpick;
pub mod forms;
pub mod gizmo;
pub mod guide;
pub mod guide_panel;
pub mod history_panel;
pub mod i18n;
pub mod icons;
pub mod import_units;
pub mod jobs;
pub mod navbar;
pub mod ribbon;
pub mod sectionview;
pub mod select;
pub mod settings;
pub mod shell;
pub mod shot;
pub mod sketch_cache;
pub mod sketch_editor;
pub mod theme;
pub mod viewcube;
pub mod viewport;
pub mod viewtools;
#[cfg(target_arch = "wasm32")]
pub mod web;
pub mod workflow;

pub use app::SurfaceCraftApp;

//! SurfaceCraft egui front end (layer L5), shared by the desktop and web apps.
//!
//! Every action in the UI (menus, toolbar, palette) is a command executed through
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
pub mod camera;
pub mod viewport;

pub use app::SurfaceCraftApp;

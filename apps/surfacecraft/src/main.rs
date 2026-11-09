//! SurfaceCraft desktop application.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]
#![cfg_attr(all(not(debug_assertions), target_os = "windows"), windows_subsystem = "windows")]

/// The bundle id, also the application id the window manager groups the windows under.
const APP_ID: &str = "dev.pawlus.surfacecraft";

fn main() -> eframe::Result {
    let viewport = egui::ViewportBuilder::default()
        .with_title("SurfaceCraft")
        .with_app_id(APP_ID)
        .with_inner_size([1400.0, 900.0])
        .with_min_inner_size([800.0, 520.0]);
    // Who draws the window frame is decided by craft-window like in every Craft app
    // (`SURFACECRAFT_TITLEBAR=app|system` overrides). SurfaceCraft has no header bar of its own
    // yet, so the desktop's frame is kept when the decision would be the app's.
    let choice = craft_window::Choice::probe(craft_window::Prefix::new("SURFACECRAFT"));
    let viewport =
        if choice.frame().decorations() { craft_window::decorate(viewport, choice.frame()) } else { viewport };
    let options = eframe::NativeOptions {
        viewport,
        renderer: eframe::Renderer::Wgpu,
        // the window's size and place are remembered between runs
        persist_window: true,
        ..Default::default()
    };
    eframe::run_native(
        "SurfaceCraft",
        options,
        Box::new(|cc| Ok(Box::new(surfacecraft_ui_egui::SurfaceCraftApp::new(cc)))),
    )
}

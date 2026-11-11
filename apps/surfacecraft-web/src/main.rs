//! SurfaceCraft web entry point (eframe web runner). Built with `cargo xtask web` for
//! `wasm32-unknown-unknown`; on native targets it only explains how to build it.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

#[cfg(target_arch = "wasm32")]
mod threads;

#[cfg(not(target_arch = "wasm32"))]
fn main() {
    println!("surfacecraft-web is the browser build: cargo xtask web");
}

#[cfg(target_arch = "wasm32")]
fn main() {
    use wasm_bindgen::JsCast;
    // The threaded bundle runs this start function in every worker of the pool too: only the
    // page (the thread with a window) starts the app.
    let Some(doc) = web_sys::window().and_then(|w| w.document()) else { return };
    wasm_bindgen_futures::spawn_local(async move {
        let Some(canvas) =
            doc.get_element_by_id("surfacecraft").and_then(|e| e.dyn_into::<web_sys::HtmlCanvasElement>().ok())
        else {
            return;
        };
        let _ = eframe::WebRunner::new()
            .start(
                canvas,
                eframe::WebOptions::default(),
                Box::new(|cc| Ok(Box::new(surfacecraft_ui_egui::SurfaceCraftApp::new(cc)))),
            )
            .await;
    });
    // the app does not wait for the pool: until it runs, heavy commands run in place
    wasm_bindgen_futures::spawn_local(threads::start());
}

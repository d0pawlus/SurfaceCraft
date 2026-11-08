//! Browser file exchange for the web build (design doc §8 M6): picking a file gives its bytes
//! to the same commands with inline `data`; exporting downloads the bytes a command returned.
use std::cell::RefCell;
use std::rc::Rc;
use wasm_bindgen::JsCast;

/// Files picked by the user, waiting for the next frame: `(name, bytes)`.
pub type Inbox = Rc<RefCell<Vec<(String, Vec<u8>)>>>;

/// Opens the browser's file picker; the chosen file lands in `inbox`.
pub fn pick_file(inbox: &Inbox, extensions: &'static [&'static str], ctx: egui::Context) {
    let inbox = inbox.clone();
    wasm_bindgen_futures::spawn_local(async move {
        if let Some(f) = rfd::AsyncFileDialog::new().add_filter("files", extensions).pick_file().await {
            let bytes = f.read().await;
            inbox.borrow_mut().push((f.file_name(), bytes));
            ctx.request_repaint();
        }
    });
}

/// Offers `bytes` to the user as a download named `name`.
pub fn download(name: &str, bytes: &[u8]) -> bool {
    let Some(doc) = web_sys::window().and_then(|w| w.document()) else { return false };
    let arr = js_sys::Uint8Array::from(bytes);
    let parts = js_sys::Array::new();
    parts.push(&arr.buffer());
    let Ok(blob) = web_sys::Blob::new_with_buffer_source_sequence(&parts) else { return false };
    let Ok(url) = web_sys::Url::create_object_url_with_blob(&blob) else { return false };
    let Some(a) = doc.create_element("a").ok().and_then(|e| e.dyn_into::<web_sys::HtmlAnchorElement>().ok()) else {
        return false;
    };
    a.set_href(&url);
    a.set_download(name);
    a.click();
    let _ = web_sys::Url::revoke_object_url(&url);
    true
}

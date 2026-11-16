//! Files the user picked in the interface, held in memory until the command they were picked for
//! reads them. A front end without a file system (the browser) hands the bytes of a picked file
//! to the session and gives the command a `picked:` path instead of the bytes as base64 data:
//! the file then exists once in memory, not as the bytes, their base64 text, its copies in the
//! command's parameters and the copy the log keeps (measured: a 5 million triangle STL took
//! 1.97 GB at its peak as data and 0.97 GB as a file, and the log kept 333 MB of base64).
//!
//! A picked file is read once: [`crate::cmd::read_file`] takes it out of the store, so its memory
//! goes with the command that read it. The store is shared by a session and its forks, so a
//! command running in the background reads the file its front end picked.
use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

/// The start of every path of a picked file.
pub const PICKED_PREFIX: &str = "picked:";

/// The files picked and not yet read.
#[derive(Clone, Debug, Default)]
pub struct PickedFiles(Arc<Mutex<Store>>);

#[derive(Debug, Default)]
struct Store {
    /// Number of the next file, so two files of one name get two paths.
    next: u64,
    files: HashMap<String, Vec<u8>>,
}

impl PickedFiles {
    /// Is `path` the path of a picked file (whether or not it is still held)?
    #[must_use]
    pub fn is_picked(path: &str) -> bool {
        path.starts_with(PICKED_PREFIX)
    }

    fn store(&self) -> MutexGuard<'_, Store> {
        // the store holds plain data and every change is one map operation: a panic elsewhere
        // while it was locked leaves it consistent
        self.0.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Holds `bytes`, the contents of the file `name`, and returns the path a command reads them
    /// by. The path ends with `name` (separators replaced), so its stem and extension are the
    /// file's.
    pub fn put(&self, name: &str, bytes: Vec<u8>) -> String {
        let name: String = name.chars().map(|c| if matches!(c, '/' | '\\') { '_' } else { c }).collect();
        let mut store = self.store();
        let path = format!("{PICKED_PREFIX}{}/{name}", store.next);
        store.next = store.next.wrapping_add(1);
        store.files.insert(path.clone(), bytes);
        path
    }

    /// Takes the file at `path` out of the store; `None` when it is not held (never picked, or
    /// already read).
    pub fn take(&self, path: &str) -> Option<Vec<u8>> {
        self.store().files.remove(path)
    }

    /// Drops the file at `path` when it is still held (its command did not run).
    pub fn discard(&self, path: &str) {
        self.store().files.remove(path);
    }

    /// Drops the picked file the command parameters `params` name in `path`, when it is still
    /// held: a command that failed before reading it, or was cancelled before it ran, leaves
    /// nothing behind.
    pub fn release_named_in(&self, params: &serde_json::Value) {
        if let Some(path) = params.get("path").and_then(serde_json::Value::as_str)
            && Self::is_picked(path)
        {
            self.discard(path);
        }
    }

    /// Bytes of every file held.
    #[must_use]
    pub fn held_bytes(&self) -> usize {
        self.store().files.values().map(Vec::len).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_picked_file_is_read_once() {
        let picked = PickedFiles::default();
        let path = picked.put("scan.stl", vec![1, 2, 3]);
        assert!(PickedFiles::is_picked(&path));
        assert!(path.ends_with("/scan.stl"));
        assert_eq!(picked.held_bytes(), 3);
        assert_eq!(picked.take(&path), Some(vec![1, 2, 3]));
        assert_eq!(picked.take(&path), None);
        assert_eq!(picked.held_bytes(), 0);
    }

    #[test]
    fn two_files_of_one_name_get_two_paths_and_clones_share_the_store() {
        let picked = PickedFiles::default();
        let shared = picked.clone();
        let a = picked.put("scan.stl", vec![1]);
        let b = picked.put("scan.stl", vec![2]);
        assert_ne!(a, b);
        assert_eq!(shared.take(&b), Some(vec![2]));
        shared.discard(&a);
        assert_eq!(picked.held_bytes(), 0);
    }

    #[test]
    fn the_file_named_in_parameters_is_released_and_nothing_else() {
        let picked = PickedFiles::default();
        let named = picked.put("a.stl", vec![1]);
        let other = picked.put("b.stl", vec![2]);
        picked.release_named_in(&serde_json::json!({"path": "/tmp/a.stl"}));
        picked.release_named_in(&serde_json::json!({"path": 7}));
        assert_eq!(picked.held_bytes(), 2);
        picked.release_named_in(&serde_json::json!({"path": named}));
        assert_eq!(picked.take(&other), Some(vec![2]));
        assert_eq!(picked.held_bytes(), 0);
    }

    #[test]
    fn a_name_cannot_add_directories_to_the_path() {
        let picked = PickedFiles::default();
        let path = picked.put("../etc/passwd", Vec::new());
        assert_eq!(path.matches('/').count(), 1, "{path}");
        assert!(path.ends_with("/.._etc_passwd"));
    }
}

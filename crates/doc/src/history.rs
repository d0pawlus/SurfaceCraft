//! Undo/redo by document snapshots. Heavy data is shared through `Arc`, so a snapshot costs
//! the object lists, not the geometry; the snapshots themselves are shared too, so a copy of the
//! history (a session fork) costs one pointer per step.
use crate::Document;
use std::sync::Arc;

/// Default maximum number of undo steps kept.
pub const DEFAULT_DEPTH: usize = 100;

/// Undo and redo stacks.
#[derive(Clone, Debug)]
pub struct History {
    undo: Vec<(String, Arc<Document>)>,
    redo: Vec<(String, Arc<Document>)>,
    depth: usize,
}

impl Default for History {
    fn default() -> Self {
        Self { undo: Vec::new(), redo: Vec::new(), depth: DEFAULT_DEPTH }
    }
}

impl History {
    /// Records the state before a change labelled `label`; clears redo.
    pub fn push(&mut self, label: &str, before: Document) {
        self.undo.push((label.to_string(), Arc::new(before)));
        if self.undo.len() > self.depth {
            self.undo.remove(0);
        }
        self.redo.clear();
    }

    /// Steps back: returns the state to restore, given the current one.
    pub fn undo(&mut self, current: Document) -> Option<(String, Document)> {
        let (label, prev) = self.undo.pop()?;
        self.redo.push((label.clone(), Arc::new(current)));
        Some((label, Arc::unwrap_or_clone(prev)))
    }

    /// Steps forward.
    pub fn redo(&mut self, current: Document) -> Option<(String, Document)> {
        let (label, next) = self.redo.pop()?;
        self.undo.push((label.clone(), Arc::new(current)));
        Some((label, Arc::unwrap_or_clone(next)))
    }

    /// Labels that can be undone, most recent last.
    #[must_use]
    pub fn undo_labels(&self) -> Vec<&str> {
        self.undo.iter().map(|(l, _)| l.as_str()).collect()
    }

    /// Number of redo steps.
    #[must_use]
    pub fn redo_len(&self) -> usize {
        self.redo.len()
    }

    /// Forgets everything (new or opened document).
    pub fn clear(&mut self) {
        self.undo.clear();
        self.redo.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::Mesh;

    #[test]
    fn undo_redo_cycle() {
        let mut h = History::default();
        let mut d = Document::new();
        h.push("add", d.clone());
        d.add_mesh("a", Mesh::default());
        let (_, prev) = h.undo(d.clone()).unwrap();
        assert!(prev.meshes.is_empty());
        let (_, next) = h.redo(prev).unwrap();
        assert_eq!(next.meshes.len(), 1);
        assert!(h.redo(next).is_none());
    }
}

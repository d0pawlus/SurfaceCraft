//! The drawing of sketches in the viewport, tessellated once (P8-H7). The polylines of a sketch's
//! entities depend only on the sketch, not on the camera, so they are kept per sketch and
//! recomputed only when that sketch's content changes (points, entities, plane or constraints
//! are compared with the copy the cache holds). The camera projection still runs every frame.
use std::collections::HashMap;

use surfacecraft_doc::ObjectId;
use surfacecraft_sketch::Sketch;

use crate::sketch_editor::snap::entity_polyline;

/// A tessellated sketch: the sketch it was made from and one polyline per entity.
struct Entry {
    sketch: Sketch,
    lines: Vec<Vec<[f64; 2]>>,
}

/// The tessellations of the sketches on screen, by sketch and by segments per curve.
#[derive(Default)]
pub struct SketchDrawCache {
    entries: HashMap<(ObjectId, usize), Entry>,
    /// How many sketches were tessellated so far (a counter for the tests and for measurements).
    pub tessellations: usize,
}

impl SketchDrawCache {
    /// The polylines of the entities of `sk` (sketch coordinates), with `segments` per curve.
    /// Tessellates only if the cached copy of sketch `id` differs from `sk`.
    pub fn lines(&mut self, id: ObjectId, sk: &Sketch, segments: usize) -> &[Vec<[f64; 2]>] {
        let key = (id, segments);
        let is_fresh = self.entries.get(&key).is_some_and(|e| e.sketch == *sk);
        if !is_fresh {
            self.tessellations += 1;
            let lines = (0..sk.entities.len()).map(|e| entity_polyline(sk, e, segments)).collect();
            self.entries.insert(key, Entry { sketch: sk.clone(), lines });
        }
        self.entries.get(&key).map_or(&[], |e| e.lines.as_slice())
    }

    /// Forgets the sketches that are gone from the document.
    pub fn retain(&mut self, exists: impl Fn(ObjectId) -> bool) {
        self.entries.retain(|(id, _), _| exists(*id));
    }
}

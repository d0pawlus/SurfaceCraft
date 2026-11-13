//! Names of objects: unique within a document, and renaming.
use crate::object::{Document, Object, ObjectId, stores};

impl Document {
    /// A name not used by any object, built from `base`.
    #[must_use]
    pub fn unique_name(&self, base: &str) -> String {
        let base = if base.trim().is_empty() { "object" } else { base.trim() };
        if !self.name_taken(base) {
            return base.to_string();
        }
        (2..).map(|i| format!("{base} {i}")).find(|n| !self.name_taken(n)).unwrap_or_else(|| base.to_string())
    }

    fn name_taken(&self, n: &str) -> bool {
        let mut taken = false;
        stores!(self, s => taken |= s.iter().any(|o| o.name() == n));
        // parked features and bodies are inactive but keep their names
        taken || self.rolled_back.iter().any(|f| f.name == n) || self.dormant_bodies.iter().any(|b| b.name == n)
    }

    /// Renames an object; false when it does not exist. Its own name is not a clash: renaming to it
    /// changes nothing; another object's name gets the next free suffix.
    pub fn rename(&mut self, id: ObjectId, name: &str) -> bool {
        let (mut exists, mut own) = (false, false);
        stores!(self, s => if let Some(o) = s.iter().find(|o| o.id() == id) { exists = true; own = o.name() == name; });
        if !exists || own {
            return exists;
        }
        let name = self.unique_name(name);
        stores!(mut self, s => if let Some(o) = s.iter_mut().find(|o| o.id() == id) { *o.name_mut() = name.clone(); });
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn renaming_to_its_own_name_changes_nothing_and_to_a_taken_name_takes_the_next_suffix() {
        let mut d = Document::default();
        let a = d.add_mesh("solid", surfacecraft_geom::Mesh::default());
        let b = d.add_mesh("other", surfacecraft_geom::Mesh::default());
        assert!(d.rename(a, "solid"));
        assert_eq!(d.mesh(a).unwrap().name, "solid");
        assert!(d.rename(b, "solid"));
        assert_eq!(d.mesh(b).unwrap().name, "solid 2");
        assert!(d.rename(b, "solid 2"), "its own name again");
        assert_eq!(d.mesh(b).unwrap().name, "solid 2");
        assert!(!d.rename(ObjectId(999), "x"));
    }
}

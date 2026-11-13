//! The IGES writer state: the entity records and the directory and parameter sections being written.

use super::format::*;
use super::*;

pub(super) struct Ent {
    pub(super) typ: i64,
    pub(super) form: i64,
    pub(super) params: Vec<String>,
    pub(super) xform: i64,
    pub(super) label: String,
    pub(super) dependent: bool,
}

pub(super) struct W {
    pub(super) ents: Vec<Ent>,
    /// The largest coordinate written (the global section's maximum coordinate).
    pub(super) max_coord: f64,
}

impl W {
    /// Adds an entity; its directory-entry pointer.
    pub(super) fn add(&mut self, typ: i64, form: i64, params: Vec<String>) -> i64 {
        self.ents.push(Ent { typ, form, params, xform: 0, label: String::new(), dependent: true });
        2 * self.ents.len() as i64 - 1
    }
    pub(super) fn get_mut(&mut self, de: i64) -> Option<&mut Ent> {
        usize::try_from((de - 1) / 2).ok().and_then(|i| self.ents.get_mut(i))
    }
    /// Coordinates of a model point, noted for the maximum coordinate.
    pub(super) fn coords(&mut self, p: V3) -> Result<[String; 3], IoError> {
        if !p.iter().all(|x| x.is_finite()) {
            return Err(bad("non-finite coordinate"));
        }
        self.max_coord = p.iter().fold(self.max_coord, |m, x| m.max(x.abs()));
        Ok(p.map(real))
    }
    /// Transformation (124) taking a frame's local coordinates to the model.
    pub(super) fn frame(&mut self, o: V3, x: V3, y: V3) -> Result<i64, IoError> {
        let z = cross(x, y);
        let mut rows = Vec::with_capacity(12);
        for i in 0..3 {
            for v in [x[i], y[i], z[i], o[i]] {
                rows.push(num(v)?);
            }
        }
        Ok(self.add(124, 0, rows))
    }
}

/// An independent, named entity: its name as a name property (406 form 15).
pub(super) fn top_level(w: &mut W, typ: i64, form: i64, mut params: Vec<String>, name: &str) -> i64 {
    let np = w.add(406, 15, vec!["1".into(), holl(name)]);
    params.extend(["0".into(), "1".into(), np.to_string()]);
    let d = w.add(typ, form, params);
    if let Some(ent) = w.get_mut(d) {
        ent.dependent = false;
        ent.label = name.chars().filter(char::is_ascii_alphanumeric).take(8).collect();
    }
    d
}

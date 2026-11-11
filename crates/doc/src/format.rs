//! The project file: `{"format": "surfacecraft-project", "version": N, "document": {…}}` as
//! JSON (decision recorded in docs/STATUS.md: JSON keeps projects diff-able and readable by
//! agents; meshes are embedded as base64 so the file stays a fraction of a JSON number list).
use crate::Document;
use serde::{Deserialize, Serialize};

/// Format tag.
pub const FORMAT: &str = "surfacecraft-project";
/// Current version.
pub const VERSION: u32 = 1;

#[derive(Serialize, Deserialize)]
struct Envelope {
    format: String,
    version: u32,
    document: Document,
}

/// Serialises a document to project JSON.
pub fn to_json(doc: &Document) -> Result<String, String> {
    let env = Envelope { format: FORMAT.into(), version: VERSION, document: doc.clone() };
    serde_json::to_string(&env).map_err(|e| e.to_string())
}

/// Parses project JSON (any version up to [`VERSION`]).
pub fn from_json(text: &str) -> Result<Document, String> {
    let env: Envelope = serde_json::from_str(text).map_err(|e| format!("not a SurfaceCraft project: {e}"))?;
    if env.format != FORMAT {
        return Err(format!("unknown format {:?}", env.format));
    }
    if env.version > VERSION {
        return Err(format!("project version {} is newer than this build ({VERSION})", env.version));
    }
    for m in &env.document.meshes {
        m.mesh.validate().map_err(|e| format!("mesh {}: {e}", m.name))?;
        if let Some(w) = &m.working {
            w.mesh.validate().map_err(|e| format!("working copy of {}: {e}", m.name))?;
        }
    }
    let mut doc = env.document;
    let max_id = doc.meshes.iter().map(|m| m.id.0).chain(doc.bodies.iter().map(|b| b.id.0)).max().unwrap_or(0);
    doc.next_id = doc.next_id.max(max_id);
    Ok(doc)
}

/// Serde adapter: a mesh as `{"vertices": n, "triangles": m, "positions": b64(f64 LE), "indices": b64(u32 LE)}`.
pub mod mesh_b64 {
    use crate::base64;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use std::sync::Arc;
    use surfacecraft_geom::{Mesh, Point3};

    #[derive(Serialize, Deserialize)]
    struct Packed {
        vertices: usize,
        triangles: usize,
        positions: String,
        indices: String,
    }

    /// Serialises.
    pub fn serialize<S: Serializer>(m: &Arc<Mesh>, s: S) -> Result<S::Ok, S::Error> {
        let mut pos = Vec::with_capacity(m.positions.len() * 24);
        for p in &m.positions {
            for c in [p.x, p.y, p.z] {
                pos.extend_from_slice(&c.to_le_bytes());
            }
        }
        let mut idx = Vec::with_capacity(m.triangles.len() * 12);
        for t in &m.triangles {
            for i in t {
                idx.extend_from_slice(&i.to_le_bytes());
            }
        }
        Packed {
            vertices: m.positions.len(),
            triangles: m.triangles.len(),
            positions: base64::encode(&pos),
            indices: base64::encode(&idx),
        }
        .serialize(s)
    }

    /// Deserialises (lengths are checked against the declared counts).
    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Arc<Mesh>, D::Error> {
        use serde::de::Error;
        let p = Packed::deserialize(d)?;
        let pos = base64::decode(&p.positions).map_err(D::Error::custom)?;
        let idx = base64::decode(&p.indices).map_err(D::Error::custom)?;
        if Some(pos.len()) != p.vertices.checked_mul(24) || Some(idx.len()) != p.triangles.checked_mul(12) {
            return Err(D::Error::custom("mesh payload length does not match its counts"));
        }
        let (coords, _) = pos.as_chunks::<8>();
        let coords: Vec<f64> = coords.iter().map(|c| f64::from_le_bytes(*c)).collect();
        let (xyz, _) = coords.as_chunks::<3>();
        let positions: Vec<Point3> = xyz.iter().map(|c| Point3::new(c[0], c[1], c[2])).collect();
        let (ints, _) = idx.as_chunks::<4>();
        let ints: Vec<u32> = ints.iter().map(|c| u32::from_le_bytes(*c)).collect();
        let (tris, _) = ints.as_chunks::<3>();
        let triangles: Vec<[u32; 3]> = tris.to_vec();
        let mesh = Mesh::new(positions, triangles).map_err(D::Error::custom)?;
        Ok(Arc::new(mesh))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::synth;

    #[test]
    fn project_roundtrip() {
        let mut d = Document::new();
        let m = synth::sphere(surfacecraft_geom::Point3::new(1.0, 2.0, 3.0), 4.0, 12, 6);
        let id = d.add_mesh("ball", m.clone());
        let b = surfacecraft_kernel::solid::make_box([0.0; 3], [1.0, 2.0, 3.0]).unwrap();
        d.add_body("box", b);
        let text = to_json(&d).unwrap();
        let back = from_json(&text).unwrap();
        assert_eq!(*back.mesh(id).unwrap().mesh, m);
        assert_eq!(back.bodies.len(), 1);
        assert_eq!(back.next_id, d.next_id);
        assert!(from_json("{}").is_err());
        assert!(from_json("not json").is_err());
        let tampered = text.replace("\"vertices\":", "\"vertices\":1");
        assert!(from_json(&tampered).is_err());
    }

    #[test]
    fn working_copy_roundtrip() {
        use crate::WorkingCopy;
        use crate::object::Object;
        use std::sync::Arc;
        use surfacecraft_geom::{Point3, Vec3};
        let mut d = Document::new();
        let original = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 2.0, 2.0, 2, 2);
        let id = d.add_mesh("sheet", original.clone());
        let plain = to_json(&d).unwrap();
        assert!(!plain.contains("\"working\""), "no copy, no key");
        let copy = synth::plane(Point3::new(0.0, 0.0, 1.0), Vec3::x(), Vec3::y(), 2.0, 2.0, 3, 3);
        d.mesh_mut(id).unwrap().working = Some(WorkingCopy { mesh: Arc::new(copy.clone()) });
        let back = from_json(&to_json(&d).unwrap()).unwrap();
        let obj = back.mesh(id).unwrap();
        assert_eq!(*obj.mesh, original);
        assert_eq!(obj.working.as_ref().map(|w| (*w.mesh).clone()), Some(copy));
        assert!(obj.detail().contains("working copy"), "{}", obj.detail());
    }
}

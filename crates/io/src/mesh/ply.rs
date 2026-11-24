//! Stanford PLY: the header, ASCII and binary bodies, and the writer.
use super::obj::*;
use super::*;
use crate::ply::{Cursor, Element, Encoding, PlyHeader, Prop, Scalar, parse_header};
use crate::{IoError, bad};
use surfacecraft_geom::{Mesh, Point3, Vec3};

// ---------------------------------------------------------------- PLY

pub(super) fn read_ply(bytes: &[u8], limits: &Limits) -> Result<Mesh, IoError> {
    let (positions, _, triangles) = parse_ply(bytes, limits)?;
    if triangles.is_empty() {
        return Err(bad("PLY", "no faces"));
    }
    Ok(Mesh { positions, triangles })
}

type PlyData = (Vec<Point3>, Option<Vec<Vec3>>, Vec<[u32; 3]>);

pub(super) fn parse_ply(bytes: &[u8], limits: &Limits) -> Result<PlyData, IoError> {
    let PlyHeader { enc, elements, body_start } = parse_header(bytes)?;
    let body = bytes.get(body_start..).unwrap_or(&[]);
    check_element_sizes(enc, &elements, body.len())?;
    check_vertex_limit(&elements, limits)?;
    let mut out = PlyOut::new(&elements);
    let mut reader = PlyReader::new(body, enc)?;
    for e in &elements {
        read_element(e, &mut reader, limits, &mut out)?;
    }
    Ok((out.positions, out.has_normals.then_some(out.normals), out.triangles))
}

/// Refuses an element that declares more items than the body can hold.
fn check_element_sizes(enc: Encoding, elements: &[Element], body_len: usize) -> Result<(), IoError> {
    for e in elements {
        let min_item = min_item_size(enc, e);
        if e.count.checked_mul(min_item).is_none_or(|n| n > body_len) {
            return Err(bad("PLY", format!("element {} declares {} items, more than the file holds", e.name, e.count)));
        }
    }
    Ok(())
}

/// The fewest body units one item of `e` takes: every item needs at least one byte (binary) or
/// one token (ascii).
fn min_item_size(enc: Encoding, e: &Element) -> usize {
    match enc {
        Encoding::Ascii => 2,
        _ => e
            .props
            .iter()
            .map(|p| match p {
                Prop::Scalar(_, s) => s.size(),
                Prop::List(_, c, _) => c.size(),
            })
            .sum::<usize>()
            .max(1),
    }
}

/// Refuses a vertex element larger than the vertex limit.
fn check_vertex_limit(elements: &[Element], limits: &Limits) -> Result<(), IoError> {
    if let Some(v) = elements.iter().find(|e| e.name == "vertex")
        && v.count > limits.max_vertices
    {
        return Err(IoError::Limit(format!("{} vertices exceed the limit of {}", v.count, limits.max_vertices)));
    }
    Ok(())
}

/// What the PLY body has produced so far.
struct PlyOut {
    positions: Vec<Point3>,
    normals: Vec<Vec3>,
    triangles: Vec<[u32; 3]>,
    has_normals: bool,
}

impl PlyOut {
    /// An empty result; vertex normals are kept when the vertex element declares `nx`, `ny`, `nz`.
    fn new(elements: &[Element]) -> Self {
        let has_normals = elements.iter().find(|e| e.name == "vertex").is_some_and(|e| {
            ["nx", "ny", "nz"].iter().all(|n| e.props.iter().any(|p| matches!(p, Prop::Scalar(name, _) if name == n)))
        });
        Self { positions: Vec::new(), normals: Vec::new(), triangles: Vec::new(), has_normals }
    }

    /// Adds the vertex `item` read as `rec`, refusing one without finite coordinates.
    fn push_vertex(&mut self, item: usize, rec: &PlyItem) -> Result<(), IoError> {
        let p = Point3::new(rec.xyz[0], rec.xyz[1], rec.xyz[2]);
        if !surfacecraft_geom::finite(&p) {
            return Err(bad("PLY", format!("vertex {item} has no finite x, y, z")));
        }
        self.positions.push(p);
        if self.has_normals {
            let n = Vec3::new(rec.nxyz[0], rec.nxyz[1], rec.nxyz[2]);
            // unusable normals are "unknown", not an error
            self.normals.push(if surfacecraft_geom::finite_vec(&n) { n } else { Vec3::zeros() });
        }
        Ok(())
    }

    /// Fan-triangulates a face, refusing a result over the triangle limit.
    fn push_face(&mut self, poly: &[u32], limits: &Limits) -> Result<(), IoError> {
        fan(poly, &mut self.triangles);
        if self.triangles.len() > limits.max_triangles {
            return Err(IoError::Limit(format!("more than {} triangles", limits.max_triangles)));
        }
        Ok(())
    }
}

/// The scalars of the PLY body, from the ASCII tokens or the binary bytes.
struct PlyReader<'a> {
    tokens: Option<std::str::SplitWhitespace<'a>>,
    cur: Cursor<'a>,
}

impl<'a> PlyReader<'a> {
    /// A reader of `body`; an ASCII body must be text.
    fn new(body: &'a [u8], enc: Encoding) -> Result<Self, IoError> {
        let tokens = if enc == Encoding::Ascii {
            Some(std::str::from_utf8(body).map_err(|_| bad("PLY", "ASCII body is not text"))?.split_whitespace())
        } else {
            None
        };
        Ok(Self { tokens, cur: Cursor { b: body, at: 0, enc } })
    }

    /// The next scalar of type `s`; `None` when the body is truncated (or a token is no number).
    fn read_scalar(&mut self, s: Scalar) -> Option<f64> {
        match self.tokens.as_mut() {
            Some(t) => t.next()?.parse::<f64>().ok(),
            None => self.cur.read(s),
        }
    }
}

/// The element being read: which item, what it describes and how many vertices exist so far.
#[derive(Clone, Copy)]
struct ItemCtx<'a> {
    e: &'a Element,
    item: usize,
    vertex: bool,
    face: bool,
    nverts: usize,
}

/// The values of one item: vertex coordinates and normal, face corner indices.
struct PlyItem {
    xyz: [f64; 3],
    nxyz: [f64; 3],
    poly: Vec<u32>,
}

impl PlyItem {
    /// Stores a vertex property by name (other names are ignored).
    fn set_coord(&mut self, name: &str, v: f64) {
        match name {
            "x" => self.xyz[0] = v,
            "y" => self.xyz[1] = v,
            "z" => self.xyz[2] = v,
            "nx" => self.nxyz[0] = v,
            "ny" => self.nxyz[1] = v,
            "nz" => self.nxyz[2] = v,
            _ => {}
        }
    }
}

/// Reads all items of one element into `out`.
fn read_element(e: &Element, rd: &mut PlyReader<'_>, limits: &Limits, out: &mut PlyOut) -> Result<(), IoError> {
    let (vertex, face) = (e.name == "vertex", e.name == "face");
    if vertex {
        out.positions.reserve(e.count);
    }
    for item in 0..e.count {
        let rec = read_item(ItemCtx { e, item, vertex, face, nverts: out.positions.len() }, rd, limits)?;
        if vertex {
            out.push_vertex(item, &rec)?;
        }
        if face {
            out.push_face(&rec.poly, limits)?;
        }
    }
    Ok(())
}

/// Reads the properties of one item.
fn read_item(ctx: ItemCtx<'_>, rd: &mut PlyReader<'_>, limits: &Limits) -> Result<PlyItem, IoError> {
    let ItemCtx { e, item, vertex, .. } = ctx;
    let mut rec = PlyItem { xyz: [f64::NAN; 3], nxyz: [0.0f64; 3], poly: Vec::new() };
    for p in &e.props {
        match p {
            Prop::Scalar(name, s) => {
                let v = rd.read_scalar(*s).ok_or_else(|| bad("PLY", format!("{} {item}: truncated", e.name)))?;
                if vertex {
                    rec.set_coord(name, v);
                }
            }
            Prop::List(name, c, i) => read_list(ctx, name, (*c, *i), rd, limits, &mut rec.poly)?,
        }
    }
    Ok(rec)
}

/// Reads one list property (count type, index type); face corner lists go to `poly`.
fn read_list(
    ctx: ItemCtx<'_>,
    name: &str,
    (c, i): (Scalar, Scalar),
    rd: &mut PlyReader<'_>,
    limits: &Limits,
    poly: &mut Vec<u32>,
) -> Result<(), IoError> {
    let ItemCtx { e, item, face, nverts, .. } = ctx;
    let n = rd.read_scalar(c).ok_or_else(|| bad("PLY", format!("{} {item}: truncated list", e.name)))?;
    if !(0.0..=limits.max_polygon as f64).contains(&n) {
        return Err(IoError::Limit(format!("{} {item}: list of {n} items", e.name)));
    }
    for _ in 0..n as usize {
        let v = rd.read_scalar(i).ok_or_else(|| bad("PLY", format!("{} {item}: truncated list", e.name)))?;
        if face && (name == "vertex_indices" || name == "vertex_index") {
            if !(v >= 0.0 && v < nverts as f64 && v.fract() == 0.0) {
                return Err(bad("PLY", format!("face {item}: index {v} out of range")));
            }
            poly.push(v as u32);
        }
    }
    Ok(())
}

pub(super) fn write_ply(mesh: &Mesh) -> Vec<u8> {
    let header = format!(
        "ply\nformat binary_little_endian 1.0\ncomment SurfaceCraft\nelement vertex {}\nproperty double x\nproperty double y\nproperty double z\nelement face {}\nproperty list uchar uint vertex_indices\nend_header\n",
        mesh.vertex_count(),
        mesh.triangle_count()
    );
    let mut out = header.into_bytes();
    for p in &mesh.positions {
        for c in [p.x, p.y, p.z] {
            out.extend_from_slice(&c.to_le_bytes());
        }
    }
    for t in &mesh.triangles {
        out.push(3);
        for i in t {
            out.extend_from_slice(&i.to_le_bytes());
        }
    }
    out
}

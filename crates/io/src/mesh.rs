//! Mesh files: STL (binary and ASCII), OBJ, PLY (ASCII, binary little/big endian).
use crate::{IoError, bad};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Mesh, Point3, tri};

/// Mesh file formats.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum MeshFormat {
    /// Stereolithography (binary on write).
    Stl,
    /// Wavefront OBJ.
    Obj,
    /// Stanford PLY (binary little endian on write).
    Ply,
}

impl MeshFormat {
    /// From a file extension (case-insensitive).
    #[must_use]
    pub fn from_extension(ext: &str) -> Option<Self> {
        match ext.to_ascii_lowercase().as_str() {
            "stl" => Some(Self::Stl),
            "obj" => Some(Self::Obj),
            "ply" => Some(Self::Ply),
            _ => None,
        }
    }

    /// From the leading bytes of a file.
    #[must_use]
    pub fn sniff(bytes: &[u8]) -> Option<Self> {
        if bytes.starts_with(b"ply") {
            return Some(Self::Ply);
        }
        let head = &bytes[..bytes.len().min(512)];
        let text = String::from_utf8_lossy(head);
        if text.lines().any(|l| l.trim_start().starts_with("v ")) && !text.trim_start().starts_with("solid") {
            return Some(Self::Obj);
        }
        if bytes.len() >= 84 || text.trim_start().starts_with("solid") {
            return Some(Self::Stl);
        }
        None
    }
}

/// Hard limits applied while reading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limits {
    /// Maximum triangles (design doc §12: MVP targets ~10 M; the default leaves headroom).
    pub max_triangles: usize,
    /// Maximum vertices.
    pub max_vertices: usize,
    /// Maximum corners of one polygon (OBJ/PLY faces are fan-triangulated).
    pub max_polygon: usize,
}

impl Default for Limits {
    fn default() -> Self {
        Self { max_triangles: 30_000_000, max_vertices: 30_000_000, max_polygon: 256 }
    }
}

/// Reads a mesh. STL soups are welded (exact duplicates), degenerate triangles removed.
pub fn read_mesh(bytes: &[u8], format: MeshFormat, limits: &Limits) -> Result<Mesh, IoError> {
    let mut mesh = match format {
        MeshFormat::Stl => read_stl(bytes, limits)?,
        MeshFormat::Obj => read_obj(bytes, limits)?,
        MeshFormat::Ply => read_ply(bytes, limits)?,
    };
    mesh.validate().map_err(|e| bad("mesh", e.to_string()))?;
    mesh.remove_degenerate(0.0);
    Ok(mesh)
}

/// Writes a mesh.
#[must_use]
pub fn write_mesh(mesh: &Mesh, format: MeshFormat) -> Vec<u8> {
    match format {
        MeshFormat::Stl => write_stl(mesh),
        MeshFormat::Obj => write_obj(mesh),
        MeshFormat::Ply => write_ply(mesh),
    }
}

// ---------------------------------------------------------------- STL

fn le_u32(b: &[u8], at: usize) -> Option<u32> {
    let s = b.get(at..at.checked_add(4)?)?;
    Some(u32::from_le_bytes([*s.first()?, *s.get(1)?, *s.get(2)?, *s.get(3)?]))
}

fn le_f32(b: &[u8], at: usize) -> Option<f32> {
    le_u32(b, at).map(f32::from_bits)
}

fn read_stl(bytes: &[u8], limits: &Limits) -> Result<Mesh, IoError> {
    let declared = le_u32(bytes, 80).map(|n| n as usize);
    let binary_fits = declared.and_then(|n| n.checked_mul(50)).and_then(|s| s.checked_add(84)) == Some(bytes.len());
    let looks_ascii = bytes.get(..bytes.len().min(256)).is_some_and(|h| {
        let t = String::from_utf8_lossy(h);
        t.trim_start().starts_with("solid") && t.contains("facet")
    });
    if binary_fits || !looks_ascii { read_stl_binary(bytes, limits) } else { read_stl_ascii(bytes, limits) }
}

fn read_stl_binary(bytes: &[u8], limits: &Limits) -> Result<Mesh, IoError> {
    let n = le_u32(bytes, 80).ok_or_else(|| bad("STL", "file shorter than the 84-byte header"))? as usize;
    let available = bytes.len().saturating_sub(84) / 50;
    if n > available {
        return Err(bad("STL", format!("header declares {n} triangles but the file holds {available}")));
    }
    if n > limits.max_triangles {
        return Err(IoError::Limit(format!("{n} triangles exceed the limit of {}", limits.max_triangles)));
    }
    let mut positions = Vec::with_capacity(n * 3);
    let mut triangles = Vec::with_capacity(n);
    for t in 0..n {
        let base = 84 + t * 50 + 12;
        let mut corners = [Point3::origin(); 3];
        for (k, c) in corners.iter_mut().enumerate() {
            let at = base + k * 12;
            let (Some(x), Some(y), Some(z)) = (le_f32(bytes, at), le_f32(bytes, at + 4), le_f32(bytes, at + 8)) else {
                return Err(bad("STL", "truncated triangle"));
            };
            *c = Point3::new(f64::from(x), f64::from(y), f64::from(z));
        }
        if corners.iter().any(|p| !surfacecraft_geom::finite(p)) {
            return Err(bad("STL", format!("triangle {t} has a non-finite coordinate")));
        }
        let i = positions.len() as u32;
        positions.extend_from_slice(&corners);
        triangles.push([i, i + 1, i + 2]);
    }
    let mut m = Mesh { positions, triangles };
    m.weld(0.0);
    Ok(m)
}

fn read_stl_ascii(bytes: &[u8], limits: &Limits) -> Result<Mesh, IoError> {
    let text = std::str::from_utf8(bytes).map_err(|_| bad("STL", "ASCII STL is not valid UTF-8"))?;
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    let mut corner = 0usize;
    for (ln, line) in text.lines().enumerate() {
        let mut it = line.split_whitespace();
        match it.next() {
            Some("vertex") => {
                let p = parse3(&mut it).ok_or_else(|| bad("STL", format!("line {}: bad vertex", ln + 1)))?;
                positions.push(p);
                corner += 1;
                if corner == 3 {
                    let i = positions.len() as u32 - 3;
                    triangles.push([i, i + 1, i + 2]);
                    corner = 0;
                    if triangles.len() > limits.max_triangles {
                        return Err(IoError::Limit(format!("more than {} triangles", limits.max_triangles)));
                    }
                }
            }
            Some("endloop") if corner != 0 => {
                return Err(bad("STL", format!("line {}: facet without 3 vertices", ln + 1)));
            }
            _ => {}
        }
    }
    if triangles.is_empty() {
        return Err(bad("STL", "no facets"));
    }
    let mut m = Mesh { positions, triangles };
    m.weld(0.0);
    Ok(m)
}

fn parse3<'a>(it: &mut impl Iterator<Item = &'a str>) -> Option<Point3> {
    let x: f64 = it.next()?.parse().ok()?;
    let y: f64 = it.next()?.parse().ok()?;
    let z: f64 = it.next()?.parse().ok()?;
    let p = Point3::new(x, y, z);
    surfacecraft_geom::finite(&p).then_some(p)
}

fn write_stl(mesh: &Mesh) -> Vec<u8> {
    let mut out = Vec::with_capacity(84 + mesh.triangle_count() * 50);
    let mut header = [0u8; 80];
    let tag = b"SurfaceCraft binary STL";
    header[..tag.len()].copy_from_slice(tag);
    out.extend_from_slice(&header);
    out.extend_from_slice(&(mesh.triangle_count() as u32).to_le_bytes());
    for i in 0..mesh.triangle_count() {
        let Some([a, b, c]) = mesh.triangle(i) else { continue };
        let n = tri::normal(&a, &b, &c).unwrap_or_else(surfacecraft_geom::Vec3::zeros);
        for v in [n.x, n.y, n.z, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z] {
            out.extend_from_slice(&(v as f32).to_le_bytes());
        }
        out.extend_from_slice(&[0, 0]);
    }
    out
}

// ---------------------------------------------------------------- OBJ

fn obj_index(tok: &str, count: usize) -> Option<u32> {
    let first = tok.split('/').next()?;
    let i: i64 = first.parse().ok()?;
    let idx = if i > 0 {
        i - 1
    } else if i < 0 {
        count as i64 + i
    } else {
        return None;
    };
    (idx >= 0 && (idx as usize) < count).then_some(idx as u32)
}

fn read_obj(bytes: &[u8], limits: &Limits) -> Result<Mesh, IoError> {
    let text = String::from_utf8_lossy(bytes);
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for (ln, line) in text.lines().enumerate() {
        let mut it = line.split_whitespace();
        match it.next() {
            Some("v") => {
                let p = parse3(&mut it).ok_or_else(|| bad("OBJ", format!("line {}: bad vertex", ln + 1)))?;
                if positions.len() >= limits.max_vertices {
                    return Err(IoError::Limit(format!("more than {} vertices", limits.max_vertices)));
                }
                positions.push(p);
            }
            Some("f") => {
                let mut poly = Vec::new();
                for tok in it {
                    if poly.len() >= limits.max_polygon {
                        return Err(IoError::Limit(format!(
                            "line {}: polygon over {} corners",
                            ln + 1,
                            limits.max_polygon
                        )));
                    }
                    let i = obj_index(tok, positions.len())
                        .ok_or_else(|| bad("OBJ", format!("line {}: face index {tok:?} out of range", ln + 1)))?;
                    poly.push(i);
                }
                fan(&poly, &mut triangles);
                if triangles.len() > limits.max_triangles {
                    return Err(IoError::Limit(format!("more than {} triangles", limits.max_triangles)));
                }
            }
            _ => {}
        }
    }
    if triangles.is_empty() {
        return Err(bad("OBJ", "no faces"));
    }
    Ok(Mesh { positions, triangles })
}

fn fan(poly: &[u32], out: &mut Vec<[u32; 3]>) {
    if let Some((&a, rest)) = poly.split_first() {
        for w in rest.windows(2) {
            if let [b, c] = w {
                out.push([a, *b, *c]);
            }
        }
    }
}

fn write_obj(mesh: &Mesh) -> Vec<u8> {
    use std::fmt::Write;
    let mut s = String::with_capacity(mesh.vertex_count() * 40 + mesh.triangle_count() * 24);
    s.push_str("# SurfaceCraft OBJ\n");
    for p in &mesh.positions {
        let _ = writeln!(s, "v {} {} {}", p.x, p.y, p.z);
    }
    for t in &mesh.triangles {
        let _ = writeln!(s, "f {} {} {}", t[0] + 1, t[1] + 1, t[2] + 1);
    }
    s.into_bytes()
}

// ---------------------------------------------------------------- PLY

#[derive(Clone, Copy, Debug, PartialEq)]
enum Scalar {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl Scalar {
    fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "char" | "int8" => Self::I8,
            "uchar" | "uint8" => Self::U8,
            "short" | "int16" => Self::I16,
            "ushort" | "uint16" => Self::U16,
            "int" | "int32" => Self::I32,
            "uint" | "uint32" => Self::U32,
            "float" | "float32" => Self::F32,
            "double" | "float64" => Self::F64,
            _ => return None,
        })
    }
    fn size(self) -> usize {
        match self {
            Self::I8 | Self::U8 => 1,
            Self::I16 | Self::U16 => 2,
            Self::I32 | Self::U32 | Self::F32 => 4,
            Self::F64 => 8,
        }
    }
}

#[derive(Clone, Debug)]
enum Prop {
    Scalar(String, Scalar),
    List(String, Scalar, Scalar),
}

#[derive(Clone, Debug)]
struct Element {
    name: String,
    count: usize,
    props: Vec<Prop>,
}

#[derive(Clone, Copy, PartialEq)]
enum Encoding {
    Ascii,
    Le,
    Be,
}

struct Cursor<'a> {
    b: &'a [u8],
    at: usize,
    enc: Encoding,
}

impl Cursor<'_> {
    fn read(&mut self, s: Scalar) -> Option<f64> {
        let n = s.size();
        let raw = self.b.get(self.at..self.at.checked_add(n)?)?;
        self.at += n;
        let mut buf = [0u8; 8];
        buf.get_mut(..n)?.copy_from_slice(raw);
        if self.enc == Encoding::Be {
            buf.get_mut(..n)?.reverse();
        }
        Some(match s {
            Scalar::I8 => f64::from(buf[0] as i8),
            Scalar::U8 => f64::from(buf[0]),
            Scalar::I16 => f64::from(i16::from_le_bytes([buf[0], buf[1]])),
            Scalar::U16 => f64::from(u16::from_le_bytes([buf[0], buf[1]])),
            Scalar::I32 => f64::from(i32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]])),
            Scalar::U32 => f64::from(u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]])),
            Scalar::F32 => f64::from(f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]])),
            Scalar::F64 => f64::from_le_bytes(buf),
        })
    }
}

fn read_ply(bytes: &[u8], limits: &Limits) -> Result<Mesh, IoError> {
    // header
    let end = find(bytes, b"end_header").ok_or_else(|| bad("PLY", "no end_header"))?;
    let header = std::str::from_utf8(bytes.get(..end).unwrap_or(&[])).map_err(|_| bad("PLY", "header is not text"))?;
    let mut body_start = end + b"end_header".len();
    // skip the line terminator
    if bytes.get(body_start) == Some(&b'\r') {
        body_start += 1;
    }
    if bytes.get(body_start) == Some(&b'\n') {
        body_start += 1;
    }
    let mut lines = header.lines();
    if lines.next().map(str::trim) != Some("ply") {
        return Err(bad("PLY", "missing magic"));
    }
    let mut enc = None;
    let mut elements: Vec<Element> = Vec::new();
    for line in lines {
        let toks: Vec<&str> = line.split_whitespace().collect();
        match toks.as_slice() {
            ["format", f, _] => {
                enc = Some(match *f {
                    "ascii" => Encoding::Ascii,
                    "binary_little_endian" => Encoding::Le,
                    "binary_big_endian" => Encoding::Be,
                    other => return Err(bad("PLY", format!("unknown format {other}"))),
                });
            }
            ["element", name, count] => {
                let count: usize = count.parse().map_err(|_| bad("PLY", format!("bad element count {count}")))?;
                elements.push(Element { name: (*name).to_string(), count, props: Vec::new() });
            }
            ["property", "list", ct, it, name] => {
                let (Some(c), Some(i)) = (Scalar::parse(ct), Scalar::parse(it)) else {
                    return Err(bad("PLY", format!("unknown list types {ct} {it}")));
                };
                elements.last_mut().ok_or_else(|| bad("PLY", "property before element"))?.props.push(Prop::List(
                    (*name).to_string(),
                    c,
                    i,
                ));
            }
            ["property", ty, name] => {
                let t = Scalar::parse(ty).ok_or_else(|| bad("PLY", format!("unknown type {ty}")))?;
                elements
                    .last_mut()
                    .ok_or_else(|| bad("PLY", "property before element"))?
                    .props
                    .push(Prop::Scalar((*name).to_string(), t));
            }
            _ => {}
        }
    }
    let enc = enc.ok_or_else(|| bad("PLY", "missing format line"))?;
    let body = bytes.get(body_start..).unwrap_or(&[]);
    for e in &elements {
        // every element needs at least one byte (binary) or one token (ascii) per item
        let min_item = match enc {
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
        };
        if e.count.checked_mul(min_item).is_none_or(|n| n > body.len()) {
            return Err(bad("PLY", format!("element {} declares {} items, more than the file holds", e.name, e.count)));
        }
    }
    if let Some(v) = elements.iter().find(|e| e.name == "vertex")
        && v.count > limits.max_vertices
    {
        return Err(IoError::Limit(format!("{} vertices exceed the limit of {}", v.count, limits.max_vertices)));
    }
    let mut positions: Vec<Point3> = Vec::new();
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    let mut tokens = if enc == Encoding::Ascii {
        Some(std::str::from_utf8(body).map_err(|_| bad("PLY", "ASCII body is not text"))?.split_whitespace())
    } else {
        None
    };
    let mut cur = Cursor { b: body, at: 0, enc };
    let mut next = |s: Scalar| -> Option<f64> {
        match tokens.as_mut() {
            Some(t) => t.next()?.parse::<f64>().ok(),
            None => cur.read(s),
        }
    };
    for e in &elements {
        let is_vertex = e.name == "vertex";
        let is_face = e.name == "face";
        if is_vertex {
            positions.reserve(e.count);
        }
        for item in 0..e.count {
            let mut xyz = [f64::NAN; 3];
            let mut poly: Vec<u32> = Vec::new();
            for p in &e.props {
                match p {
                    Prop::Scalar(name, s) => {
                        let v = next(*s).ok_or_else(|| bad("PLY", format!("{} {item}: truncated", e.name)))?;
                        if is_vertex {
                            match name.as_str() {
                                "x" => xyz[0] = v,
                                "y" => xyz[1] = v,
                                "z" => xyz[2] = v,
                                _ => {}
                            }
                        }
                    }
                    Prop::List(name, c, i) => {
                        let n = next(*c).ok_or_else(|| bad("PLY", format!("{} {item}: truncated list", e.name)))?;
                        if !(0.0..=limits.max_polygon as f64).contains(&n) {
                            return Err(IoError::Limit(format!("{} {item}: list of {n} items", e.name)));
                        }
                        for _ in 0..n as usize {
                            let v = next(*i).ok_or_else(|| bad("PLY", format!("{} {item}: truncated list", e.name)))?;
                            if is_face && (name == "vertex_indices" || name == "vertex_index") {
                                if !(v >= 0.0 && v < positions.len() as f64 && v.fract() == 0.0) {
                                    return Err(bad("PLY", format!("face {item}: index {v} out of range")));
                                }
                                poly.push(v as u32);
                            }
                        }
                    }
                }
            }
            if is_vertex {
                let p = Point3::new(xyz[0], xyz[1], xyz[2]);
                if !surfacecraft_geom::finite(&p) {
                    return Err(bad("PLY", format!("vertex {item} has no finite x, y, z")));
                }
                positions.push(p);
            }
            if is_face {
                fan(&poly, &mut triangles);
                if triangles.len() > limits.max_triangles {
                    return Err(IoError::Limit(format!("more than {} triangles", limits.max_triangles)));
                }
            }
        }
    }
    if triangles.is_empty() {
        return Err(bad("PLY", "no faces"));
    }
    Ok(Mesh { positions, triangles })
}

fn find(hay: &[u8], needle: &[u8]) -> Option<usize> {
    // the header is short; bound the search to keep hostile inputs cheap
    let lim = hay.len().min(1 << 20);
    hay.get(..lim)?.windows(needle.len()).position(|w| w == needle)
}

fn write_ply(mesh: &Mesh) -> Vec<u8> {
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

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Rng, synth};

    fn sample() -> Mesh {
        synth::cuboid(Point3::new(1.0, 2.0, 3.0), surfacecraft_geom::Vec3::new(4.0, 5.0, 6.0), 2)
    }

    #[test]
    fn roundtrip_every_format() {
        let m = sample();
        for f in [MeshFormat::Stl, MeshFormat::Obj, MeshFormat::Ply] {
            let bytes = write_mesh(&m, f);
            assert_eq!(MeshFormat::sniff(&bytes), Some(f), "{f:?}");
            let back = read_mesh(&bytes, f, &Limits::default()).unwrap();
            assert_eq!(back.triangle_count(), m.triangle_count(), "{f:?}");
            assert_eq!(back.vertex_count(), m.vertex_count(), "{f:?}");
            assert!((back.area() - m.area()).abs() < 1e-4, "{f:?}");
        }
    }

    #[test]
    fn ascii_stl_and_ply_variants() {
        let stl = "solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid t\n";
        let m = read_mesh(stl.as_bytes(), MeshFormat::Stl, &Limits::default()).unwrap();
        assert_eq!(m.triangle_count(), 1);
        let ply = "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0 1\n1 0 0 1\n1 1 0 1\n0 1 0 1\n4 0 1 2 3\n";
        let m = read_mesh(ply.as_bytes(), MeshFormat::Ply, &Limits::default()).unwrap();
        assert_eq!(m.triangle_count(), 2);
        // big endian float
        let mut be = b"ply\nformat binary_big_endian 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n".to_vec();
        for v in [0f32, 0., 0., 1., 0., 0., 0., 1., 0.] {
            be.extend_from_slice(&v.to_be_bytes());
        }
        be.push(3);
        for i in [0i32, 1, 2] {
            be.extend_from_slice(&i.to_be_bytes());
        }
        let m = read_mesh(&be, MeshFormat::Ply, &Limits::default()).unwrap();
        assert!((m.area() - 0.5).abs() < 1e-12);
        let obj = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf -4/1/1 -3/2/2 -2 -1\n";
        let m = read_mesh(obj.as_bytes(), MeshFormat::Obj, &Limits::default()).unwrap();
        assert_eq!(m.triangle_count(), 2);
    }

    #[test]
    fn hostile_headers_are_refused() {
        // STL declaring a billion triangles in a tiny file
        let mut stl = vec![0u8; 84];
        stl[80..84].copy_from_slice(&1_000_000_000u32.to_le_bytes());
        assert!(read_mesh(&stl, MeshFormat::Stl, &Limits::default()).is_err());
        // PLY declaring huge element counts
        let ply = b"ply\nformat binary_little_endian 1.0\nelement vertex 4000000000\nproperty float x\nend_header\n";
        assert!(read_mesh(ply, MeshFormat::Ply, &Limits::default()).is_err());
        let ply = b"ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n3 0 0 7\n";
        assert!(read_mesh(ply, MeshFormat::Ply, &Limits::default()).is_err());
        let obj = b"v 0 0 0\nf 1 2 3\n";
        assert!(read_mesh(obj, MeshFormat::Obj, &Limits::default()).is_err());
        let obj = b"v nan 0 0\n";
        assert!(read_mesh(obj, MeshFormat::Obj, &Limits::default()).is_err());
        let limits = Limits { max_triangles: 1, ..Limits::default() };
        assert!(matches!(
            read_mesh(&write_mesh(&sample(), MeshFormat::Stl), MeshFormat::Stl, &limits),
            Err(IoError::Limit(_))
        ));
    }

    /// Random bytes and random corruptions of valid files never panic (spike §3.2 item 7).
    #[test]
    fn hostile_bytes_never_panic() {
        let mut rng = Rng::new(42);
        let valid: Vec<(MeshFormat, Vec<u8>)> = [MeshFormat::Stl, MeshFormat::Obj, MeshFormat::Ply]
            .iter()
            .map(|&f| (f, write_mesh(&sample(), f)))
            .collect();
        for round in 0..3000 {
            let (f, base) = &valid[round % valid.len()];
            let mut bytes = if round % 4 == 0 {
                (0..rng.index(400)).map(|_| rng.next_u64() as u8).collect::<Vec<u8>>()
            } else {
                base.clone()
            };
            let flips = 1 + rng.index(8);
            for _ in 0..flips {
                if bytes.is_empty() {
                    break;
                }
                let i = rng.index(bytes.len());
                bytes[i] = rng.next_u64() as u8;
            }
            if round % 3 == 0 {
                let cut = rng.index(bytes.len() + 1);
                bytes.truncate(cut);
            }
            for fmt in [*f, MeshFormat::Stl, MeshFormat::Obj, MeshFormat::Ply] {
                if let Ok(m) = read_mesh(&bytes, fmt, &Limits::default()) {
                    assert!(m.validate().is_ok());
                }
            }
            let _ = MeshFormat::sniff(&bytes);
        }
    }
}

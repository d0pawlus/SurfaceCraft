//! Streaming binary PLY. Faces are read in order from one handle; the vertices they refer to are
//! fetched from the vertex element of the file through a direct-mapped page cache on a second
//! handle, so no vertex array is held (memory: [`PAGE_SLOTS`] pages of [`PAGE_VERTICES`] vertices).
//! Meshes whose faces refer to nearby vertices (any exporter that writes a surface patch by patch)
//! hit the cache; a shuffled vertex order still works, only slower.
//!
//! Limits of this reader: binary encodings only; the vertex element must come before the face
//! element, and every element before the face element must have fixed-size records (no lists).
use super::{Batcher, Open, io_err};
use crate::mesh::Limits;
use crate::ply::{Cursor, Element, Encoding, Prop, Scalar, parse_header};
use crate::{IoError, bad};
use std::io::{BufReader, Read, Seek, SeekFrom};
use surfacecraft_geom::Point3;

/// Largest header read (the in-memory reader searches the same prefix).
const HEADER_LIMIT: u64 = 1 << 20;
/// Vertices per cache page.
pub(crate) const PAGE_VERTICES: usize = 1024;
/// Pages in the cache.
pub(crate) const PAGE_SLOTS: usize = 1024;
const READ_BUFFER: usize = 1 << 20;

fn read_scalar<R: Read>(r: &mut R, s: Scalar, enc: Encoding) -> Result<f64, IoError> {
    let mut buf = [0u8; 8];
    let raw = buf.get_mut(..s.size()).ok_or_else(|| bad("PLY", "scalar wider than 8 bytes"))?;
    r.read_exact(raw).map_err(|_| bad("PLY", "truncated"))?;
    Cursor { b: raw, at: 0, enc }.read(s).ok_or_else(|| bad("PLY", "truncated"))
}

/// Bytes of one record of `e` if all its properties are scalars.
fn record_size(e: &Element) -> Option<usize> {
    e.props.iter().try_fold(0usize, |n, p| match p {
        Prop::Scalar(_, s) => n.checked_add(s.size()),
        Prop::List(..) => None,
    })
}

/// Where the vertex and face elements lie in the file.
struct Layout {
    vertex: Element,
    vertex_base: u64,
    face: Element,
    face_start: u64,
}

fn layout(elements: &[Element], body_start: usize, len: u64, limits: &Limits) -> Result<Layout, IoError> {
    let mut offset = body_start as u64;
    let mut vertex: Option<(Element, u64)> = None;
    for e in elements {
        if e.name == "face" {
            let (v, base) = vertex.ok_or_else(|| IoError::Unsupported("PLY faces before vertices".into()))?;
            let min_item: usize = e
                .props
                .iter()
                .map(|p| match p {
                    Prop::Scalar(_, s) => s.size(),
                    Prop::List(_, c, _) => c.size(),
                })
                .sum::<usize>()
                .max(1);
            if (e.count as u64).checked_mul(min_item as u64).is_none_or(|n| n > len.saturating_sub(offset)) {
                return Err(bad("PLY", format!("{} faces declared, more than the file holds", e.count)));
            }
            return Ok(Layout { vertex: v, vertex_base: base, face: e.clone(), face_start: offset });
        }
        let rec = record_size(e).ok_or_else(|| {
            IoError::Unsupported(format!("PLY element {} has variable-size records and precedes the faces", e.name))
        })?;
        let bytes = (e.count as u64).checked_mul(rec as u64).and_then(|b| b.checked_add(offset));
        let end = bytes
            .filter(|&b| b <= len)
            .ok_or_else(|| bad("PLY", format!("element {} is larger than the file", e.name)))?;
        if e.name == "vertex" {
            if e.count > limits.max_vertices {
                return Err(IoError::Limit(format!(
                    "{} vertices exceed the limit of {}",
                    e.count, limits.max_vertices
                )));
            }
            vertex = Some((e.clone(), offset));
        }
        offset = end;
    }
    Err(bad("PLY", "no faces"))
}

struct Page {
    number: usize,
    points: Vec<Point3>,
}

/// The vertices of the file, fetched on demand.
struct VertexTable<R> {
    rd: R,
    base: u64,
    record: usize,
    count: usize,
    enc: Encoding,
    /// Per property of a record: its type and the axis (0, 1, 2) it holds, if any.
    props: Vec<(Scalar, Option<usize>)>,
    slots: Vec<Option<Page>>,
    scratch: Vec<u8>,
}

impl<R: Read + Seek> VertexTable<R> {
    fn new(rd: R, v: &Element, base: u64, enc: Encoding) -> Result<Self, IoError> {
        let axis = |n: &str| match n {
            "x" => Some(0),
            "y" => Some(1),
            "z" => Some(2),
            _ => None,
        };
        let props: Vec<(Scalar, Option<usize>)> = v
            .props
            .iter()
            .filter_map(|p| match p {
                Prop::Scalar(n, s) => Some((*s, axis(n))),
                Prop::List(..) => None,
            })
            .collect();
        if (0..3).any(|a| !props.iter().any(|(_, x)| *x == Some(a))) {
            return Err(bad("PLY", "the vertex element has no x, y, z properties"));
        }
        let record = record_size(v).ok_or_else(|| IoError::Unsupported("PLY vertex records with lists".into()))?;
        Ok(Self {
            rd,
            base,
            record,
            count: v.count,
            enc,
            props,
            slots: (0..PAGE_SLOTS).map(|_| None).collect(),
            scratch: Vec::new(),
        })
    }

    fn load(&mut self, number: usize) -> Result<Page, IoError> {
        let first = number * PAGE_VERTICES;
        let n = PAGE_VERTICES.min(self.count.saturating_sub(first));
        let at = self.base + (first * self.record) as u64;
        self.rd.seek(SeekFrom::Start(at)).map_err(|e| io_err(&e))?;
        self.scratch.resize(n * self.record, 0);
        self.rd.read_exact(&mut self.scratch).map_err(|_| bad("PLY", "truncated vertex data"))?;
        let mut points = Vec::with_capacity(n);
        for rec in self.scratch.chunks_exact(self.record.max(1)) {
            let mut cur = Cursor { b: rec, at: 0, enc: self.enc };
            let mut xyz = [f64::NAN; 3];
            for (s, axis) in &self.props {
                let v = cur.read(*s).ok_or_else(|| bad("PLY", "truncated vertex data"))?;
                if let Some(slot) = axis.and_then(|a| xyz.get_mut(a)) {
                    *slot = v;
                }
            }
            points.push(Point3::new(xyz[0], xyz[1], xyz[2]));
        }
        Ok(Page { number, points })
    }

    fn get(&mut self, i: usize) -> Result<Point3, IoError> {
        if i >= self.count {
            return Err(bad("PLY", format!("face index {i} out of range")));
        }
        let (number, local) = (i / PAGE_VERTICES, i % PAGE_VERTICES);
        let slot = number % PAGE_SLOTS;
        if self.slots.get(slot).and_then(Option::as_ref).is_none_or(|p| p.number != number) {
            let page = self.load(number)?;
            if let Some(s) = self.slots.get_mut(slot) {
                *s = Some(page);
            }
        }
        let p = self.slots.get(slot).and_then(Option::as_ref).and_then(|p| p.points.get(local)).copied();
        match p {
            Some(p) if surfacecraft_geom::finite(&p) => Ok(p),
            Some(_) => Err(bad("PLY", format!("vertex {i} has no finite x, y, z"))),
            None => Err(bad("PLY", format!("vertex {i} is missing"))),
        }
    }
}

/// Reads the faces of a binary PLY, fan-triangulating polygons.
pub(crate) fn read_binary<R: Read + Seek>(
    open: Open<'_, R>,
    limits: &Limits,
    out: &mut Batcher<'_>,
) -> Result<(), IoError> {
    let mut first = open().map_err(|e| io_err(&e))?;
    let len = first.seek(SeekFrom::End(0)).map_err(|e| io_err(&e))?;
    first.seek(SeekFrom::Start(0)).map_err(|e| io_err(&e))?;
    let mut head = Vec::new();
    first.by_ref().take(HEADER_LIMIT).read_to_end(&mut head).map_err(|e| io_err(&e))?;
    let header = parse_header(&head)?;
    if header.enc == Encoding::Ascii {
        return Err(IoError::Unsupported("streaming import reads binary PLY; use mesh.import for ASCII PLY".into()));
    }
    let lay = layout(&header.elements, header.body_start, len, limits)?;
    let mut vertices = VertexTable::new(open().map_err(|e| io_err(&e))?, &lay.vertex, lay.vertex_base, header.enc)?;
    first.seek(SeekFrom::Start(lay.face_start)).map_err(|e| io_err(&e))?;
    let mut faces = BufReader::with_capacity(READ_BUFFER, first);
    let mut poly: Vec<usize> = Vec::new();
    for item in 0..lay.face.count {
        poly.clear();
        for p in &lay.face.props {
            match p {
                Prop::Scalar(_, s) => {
                    read_scalar(&mut faces, *s, header.enc)?;
                }
                Prop::List(name, c, i) => {
                    let n = read_scalar(&mut faces, *c, header.enc)?;
                    if !(0.0..=limits.max_polygon as f64).contains(&n) {
                        return Err(IoError::Limit(format!("face {item}: list of {n} items")));
                    }
                    let wanted = name == "vertex_indices" || name == "vertex_index";
                    for _ in 0..n as usize {
                        let v = read_scalar(&mut faces, *i, header.enc)?;
                        if wanted {
                            if !(v >= 0.0 && v < lay.vertex.count as f64 && v.fract() == 0.0) {
                                return Err(bad("PLY", format!("face {item}: index {v} out of range")));
                            }
                            poly.push(v as usize);
                        }
                    }
                }
            }
        }
        emit_fan(&poly, &mut vertices, out)?;
    }
    Ok(())
}

fn emit_fan<R: Read + Seek>(
    poly: &[usize],
    vertices: &mut VertexTable<R>,
    out: &mut Batcher<'_>,
) -> Result<(), IoError> {
    let Some((&a, rest)) = poly.split_first() else { return Ok(()) };
    for w in rest.windows(2) {
        if let [b, c] = w {
            out.push([vertices.get(a)?, vertices.get(*b)?, vertices.get(*c)?])?;
        }
    }
    Ok(())
}

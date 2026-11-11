//! The PLY header and scalar decoding shared by the in-memory reader (`mesh.rs`) and the streaming
//! reader (`stream/ply.rs`).
use crate::{IoError, bad};

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Scalar {
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
    pub(crate) fn parse(s: &str) -> Option<Self> {
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
    pub(crate) fn size(self) -> usize {
        match self {
            Self::I8 | Self::U8 => 1,
            Self::I16 | Self::U16 => 2,
            Self::I32 | Self::U32 | Self::F32 => 4,
            Self::F64 => 8,
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) enum Prop {
    Scalar(String, Scalar),
    List(String, Scalar, Scalar),
}

#[derive(Clone, Debug)]
pub(crate) struct Element {
    pub(crate) name: String,
    pub(crate) count: usize,
    pub(crate) props: Vec<Prop>,
}

#[derive(Clone, Copy, PartialEq)]
pub(crate) enum Encoding {
    Ascii,
    Le,
    Be,
}

pub(crate) struct Cursor<'a> {
    pub(crate) b: &'a [u8],
    pub(crate) at: usize,
    pub(crate) enc: Encoding,
}

impl Cursor<'_> {
    pub(crate) fn read(&mut self, s: Scalar) -> Option<f64> {
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

/// A parsed PLY header: encoding, elements and where the body starts.
pub(crate) struct PlyHeader {
    pub(crate) enc: Encoding,
    pub(crate) elements: Vec<Element>,
    pub(crate) body_start: usize,
}

/// Parses the header found in the leading bytes of a PLY file (`end_header` must lie within the
/// first MiB).
pub(crate) fn parse_header(bytes: &[u8]) -> Result<PlyHeader, IoError> {
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
    Ok(PlyHeader { enc, elements, body_start })
}

fn find(hay: &[u8], needle: &[u8]) -> Option<usize> {
    // the header is short; bound the search to keep hostile inputs cheap
    let lim = hay.len().min(1 << 20);
    hay.get(..lim)?.windows(needle.len()).position(|w| w == needle)
}

//! Binary and ASCII STL: the binary reader (corners joined as read), the welded ASCII reader and
//! the writer.
use super::stl_corners::CornerFault;
use super::*;
use crate::{IoError, bad};
use surfacecraft_geom::{Mesh, tri};

// ---------------------------------------------------------------- STL

fn le_u32(b: &[u8], at: usize) -> Option<u32> {
    let s = b.get(at..at.checked_add(4)?)?;
    Some(u32::from_le_bytes([*s.first()?, *s.get(1)?, *s.get(2)?, *s.get(3)?]))
}

pub(super) fn read_stl(bytes: &[u8], limits: &Limits) -> Result<Mesh, IoError> {
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
    if n.checked_mul(3).is_none_or(|c| c > u32::MAX as usize) {
        return Err(IoError::Limit(format!("{n} triangles need more vertices than an index can address")));
    }
    // bit-equal corners are joined as they are read; welding joins what is left below the
    // tolerance, so the mesh is the one of welding every corner
    let mut m = super::stl_corners::indexed(bytes, n, limits.max_vertices).map_err(|f| match f {
        CornerFault::Truncated => bad("STL", "truncated triangle"),
        CornerFault::NonFinite(t) => bad("STL", format!("triangle {t} has a non-finite coordinate")),
        CornerFault::TooManyVertices => IoError::Limit(format!(
            "more than {} distinct corners (the triangles share too few of them)",
            limits.max_vertices
        )),
        CornerFault::Colliding => bad("STL", "corner coordinates made to collide in the reader's table"),
    })?;
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
pub(super) fn write_stl(mesh: &Mesh) -> Vec<u8> {
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

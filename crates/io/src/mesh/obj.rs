//! Wavefront OBJ: vertices and faces read as fan-triangulated polygons, and the writer.
use super::*;
use crate::{IoError, bad};
use surfacecraft_geom::{Mesh, Point3};

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

pub(super) fn read_obj(bytes: &[u8], limits: &Limits) -> Result<Mesh, IoError> {
    let (positions, triangles) = parse_obj(bytes, limits)?;
    if triangles.is_empty() {
        return Err(bad("OBJ", "no faces"));
    }
    Ok(Mesh { positions, triangles })
}

pub(super) fn parse_obj(bytes: &[u8], limits: &Limits) -> Result<(Vec<Point3>, Vec<[u32; 3]>), IoError> {
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
    Ok((positions, triangles))
}

pub(super) fn fan(poly: &[u32], out: &mut Vec<[u32; 3]>) {
    if let Some((&a, rest)) = poly.split_first() {
        for w in rest.windows(2) {
            if let [b, c] = w {
                out.push([a, *b, *c]);
            }
        }
    }
}

pub(super) fn write_obj(mesh: &Mesh) -> Vec<u8> {
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

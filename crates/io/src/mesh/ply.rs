//! Stanford PLY: the header, ASCII and binary bodies, and the writer.
use super::obj::*;
use super::*;
use crate::ply::{Cursor, Encoding, PlyHeader, Prop, Scalar, parse_header};
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
    let mut normals: Vec<Vec3> = Vec::new();
    let has_normals = elements.iter().find(|e| e.name == "vertex").is_some_and(|e| {
        ["nx", "ny", "nz"].iter().all(|n| e.props.iter().any(|p| matches!(p, Prop::Scalar(name, _) if name == n)))
    });
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
            let mut nxyz = [0.0f64; 3];
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
                                "nx" => nxyz[0] = v,
                                "ny" => nxyz[1] = v,
                                "nz" => nxyz[2] = v,
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
                if has_normals {
                    let n = Vec3::new(nxyz[0], nxyz[1], nxyz[2]);
                    // unusable normals are "unknown", not an error
                    normals.push(if surfacecraft_geom::finite_vec(&n) { n } else { Vec3::zeros() });
                }
            }
            if is_face {
                fan(&poly, &mut triangles);
                if triangles.len() > limits.max_triangles {
                    return Err(IoError::Limit(format!("more than {} triangles", limits.max_triangles)));
                }
            }
        }
    }
    Ok((positions, has_normals.then_some(normals), triangles))
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

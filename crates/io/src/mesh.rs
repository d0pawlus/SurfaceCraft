//! Mesh files: STL (binary and ASCII), OBJ, PLY (ASCII, binary little/big endian).
use crate::{IoError, bad};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Mesh, Point3, PointCloud};

mod obj;
mod ply;
mod stl;
#[cfg(test)]
mod tests;
mod xyz;

use obj::*;
use ply::*;
use stl::*;
use xyz::*;

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
    /// Plain text points, one `x y z [nx ny nz]` per line (point clouds; `.xyz`, `.asc`, `.txt`, `.pts`).
    Xyz,
}

impl MeshFormat {
    /// From a file extension (case-insensitive).
    #[must_use]
    pub fn from_extension(ext: &str) -> Option<Self> {
        match ext.to_ascii_lowercase().as_str() {
            "stl" => Some(Self::Stl),
            "obj" => Some(Self::Obj),
            "ply" => Some(Self::Ply),
            "xyz" | "asc" | "txt" | "pts" => Some(Self::Xyz),
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
        if looks_like_xyz(&text) {
            return Some(Self::Xyz);
        }
        if bytes.len() >= 84 || text.trim_start().starts_with("solid") {
            return Some(Self::Stl);
        }
        None
    }
}
/// What a file holds: a mesh, or only points.
#[derive(Clone, Debug, PartialEq)]
pub enum Scan {
    /// Triangles (STL, or OBJ/PLY with faces).
    Mesh(Mesh),
    /// Points without faces (XYZ, or OBJ/PLY without faces).
    Points(PointCloud),
}

/// Reads a mesh or, when the file has no faces, a point cloud. Meshes are cleaned as in
/// [`read_mesh`]; the limits of [`Limits`] apply to both.
pub fn read_scan(bytes: &[u8], format: MeshFormat, limits: &Limits) -> Result<Scan, IoError> {
    match format {
        MeshFormat::Stl => read_mesh(bytes, format, limits).map(Scan::Mesh),
        MeshFormat::Xyz => read_xyz(bytes, limits).map(Scan::Points),
        MeshFormat::Obj | MeshFormat::Ply => {
            let (positions, normals, triangles) = if format == MeshFormat::Obj {
                let (p, t) = parse_obj(bytes, limits)?;
                (p, None, t)
            } else {
                parse_ply(bytes, limits)?
            };
            let name = if format == MeshFormat::Obj { "OBJ" } else { "PLY" };
            if triangles.is_empty() {
                if positions.is_empty() {
                    return Err(bad(name, "no faces and no vertices"));
                }
                let cloud = PointCloud::new(positions, normals).map_err(|e| bad(name, e.to_string()))?;
                return Ok(Scan::Points(cloud));
            }
            let mut mesh = Mesh { positions, triangles };
            mesh.validate().map_err(|e| bad("mesh", e.to_string()))?;
            mesh.remove_degenerate(0.0);
            Ok(Scan::Mesh(mesh))
        }
    }
}

/// Reads only the points of any file: a point cloud as it is, a mesh by its vertices.
pub fn read_points(bytes: &[u8], format: MeshFormat, limits: &Limits) -> Result<PointCloud, IoError> {
    match format {
        MeshFormat::Xyz => read_xyz(bytes, limits),
        MeshFormat::Ply => {
            let (positions, normals, _) = parse_ply(bytes, limits)?;
            PointCloud::new(positions, normals).map_err(|e| bad("PLY", e.to_string()))
        }
        MeshFormat::Obj => {
            let (positions, _) = parse_obj(bytes, limits)?;
            PointCloud::new(positions, None).map_err(|e| bad("OBJ", e.to_string()))
        }
        MeshFormat::Stl => {
            let m = read_mesh(bytes, format, limits)?;
            Ok(PointCloud::from_points(m.positions))
        }
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
        MeshFormat::Xyz => return Err(bad("XYZ", "a plain point list has no faces; read it as a point cloud")),
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
        MeshFormat::Xyz => write_xyz(mesh),
    }
}
fn parse3<'a>(it: &mut impl Iterator<Item = &'a str>) -> Option<Point3> {
    let x: f64 = it.next()?.parse().ok()?;
    let y: f64 = it.next()?.parse().ok()?;
    let z: f64 = it.next()?.parse().ok()?;
    let p = Point3::new(x, y, z);
    surfacecraft_geom::finite(&p).then_some(p)
}

//! Plain text point lists (`x y z [nx ny nz]` per line): the sniffing rule, the reader and the writer.
use super::*;
use crate::{IoError, bad};
use surfacecraft_geom::{Mesh, Point3, PointCloud, Vec3};

/// Do the first data lines of `text` read as `x y z ...` rows? (The last line of a 512-byte sniff
/// window may be cut, so it is ignored.)
pub(super) fn looks_like_xyz(text: &str) -> bool {
    let mut rows = 0;
    let lines: Vec<&str> = text.lines().collect();
    for line in lines.iter().take(lines.len().saturating_sub(1)) {
        let t = line.trim();
        if t.is_empty() || t.starts_with('#') || t.starts_with("//") {
            continue;
        }
        if xyz_row(t).is_none() {
            // a first line may be a header such as a point count
            if rows == 0 && t.split_whitespace().count() == 1 {
                continue;
            }
            return false;
        }
        rows += 1;
    }
    rows > 0
}
// ---------------------------------------------------------------- XYZ

/// Splits a row on whitespace, commas and semicolons and parses `x y z ...`; `None` unless the
/// first three columns are finite numbers. Returns the leading columns that parse as numbers.
fn xyz_row(line: &str) -> Option<Vec<f64>> {
    let mut v = Vec::with_capacity(6);
    for tok in line.split(|c: char| c.is_whitespace() || c == ',' || c == ';').filter(|t| !t.is_empty()) {
        match tok.parse::<f64>() {
            Ok(x) if x.is_finite() => v.push(x),
            _ => {
                if v.len() < 3 {
                    return None;
                }
                break; // trailing text columns (labels) are ignored
            }
        }
        if v.len() >= 9 {
            break;
        }
    }
    (v.len() >= 3).then_some(v)
}

pub(super) fn read_xyz(bytes: &[u8], limits: &Limits) -> Result<PointCloud, IoError> {
    let text = String::from_utf8_lossy(bytes);
    let mut positions: Vec<Point3> = Vec::new();
    let mut normals: Vec<Vec3> = Vec::new();
    let mut columns = 0usize;
    let mut header_skipped = false;
    for (ln, line) in text.lines().enumerate() {
        let t = line.trim();
        if t.is_empty() || t.starts_with('#') || t.starts_with("//") {
            continue;
        }
        let Some(row) = xyz_row(t) else {
            // one header line (a count or column names) before the first point is tolerated
            if positions.is_empty() && !header_skipped {
                header_skipped = true;
                continue;
            }
            return Err(bad("XYZ", format!("line {}: expected `x y z`", ln + 1)));
        };
        if columns == 0 {
            // exactly six columns are `x y z nx ny nz`; anything else carries no normals
            columns = if row.len() == 6 { 6 } else { 3 };
        }
        if positions.len() >= limits.max_vertices {
            return Err(IoError::Limit(format!("more than {} points", limits.max_vertices)));
        }
        let get = |k: usize| row.get(k).copied().unwrap_or(0.0);
        positions.push(Point3::new(get(0), get(1), get(2)));
        if columns == 6 {
            normals.push(Vec3::new(get(3), get(4), get(5)));
        }
    }
    if positions.is_empty() {
        return Err(bad("XYZ", "no points"));
    }
    let normals = (columns == 6 && normals.len() == positions.len()).then_some(normals);
    PointCloud::new(positions, normals).map_err(|e| bad("XYZ", e.to_string()))
}

pub(super) fn write_xyz(mesh: &Mesh) -> Vec<u8> {
    use std::fmt::Write;
    let normals = mesh.vertex_normals();
    let mut s = String::with_capacity(mesh.vertex_count() * 60);
    for (p, n) in mesh.positions.iter().zip(&normals) {
        let _ = writeln!(s, "{} {} {} {} {} {}", p.x, p.y, p.z, n.x, n.y, n.z);
    }
    s.into_bytes()
}

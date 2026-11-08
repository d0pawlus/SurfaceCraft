//! Indexed triangle meshes.
use crate::{Aabb, Point3, Rng, Transform, Vec3, tri};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Why a mesh is not valid.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum MeshError {
    /// A triangle refers to a vertex that does not exist.
    #[error("triangle {tri} refers to vertex {index}, but there are only {count} vertices")]
    IndexOutOfRange {
        /// Triangle index.
        tri: usize,
        /// Offending vertex index.
        index: u32,
        /// Vertex count.
        count: usize,
    },
    /// A vertex coordinate is NaN or infinite.
    #[error("vertex {0} has a non-finite coordinate")]
    NonFinite(usize),
    /// More vertices than a `u32` index can address.
    #[error("too many vertices ({0})")]
    TooManyVertices(usize),
}

/// Read access to a set of points, so algorithms do not assume a whole in-RAM mesh
/// (design doc §9: the path to out-of-core stays open).
pub trait PointSource {
    /// Number of points.
    fn point_count(&self) -> usize;
    /// Point `i`, or `None` past the end.
    fn point_at(&self, i: usize) -> Option<Point3>;
}

impl PointSource for [Point3] {
    fn point_count(&self) -> usize {
        self.len()
    }
    fn point_at(&self, i: usize) -> Option<Point3> {
        self.get(i).copied()
    }
}

impl PointSource for Vec<Point3> {
    fn point_count(&self) -> usize {
        self.len()
    }
    fn point_at(&self, i: usize) -> Option<Point3> {
        self.get(i).copied()
    }
}

/// An indexed triangle mesh in millimetres. Triangles are counter-clockwise seen from outside.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Mesh {
    /// Vertex positions.
    pub positions: Vec<Point3>,
    /// Triangles as vertex index triples.
    pub triangles: Vec<[u32; 3]>,
}

impl PointSource for Mesh {
    fn point_count(&self) -> usize {
        self.positions.len()
    }
    fn point_at(&self, i: usize) -> Option<Point3> {
        self.positions.get(i).copied()
    }
}

impl Mesh {
    /// A validated mesh: every index in range, every coordinate finite.
    pub fn new(positions: Vec<Point3>, triangles: Vec<[u32; 3]>) -> Result<Self, MeshError> {
        let m = Mesh { positions, triangles };
        m.validate()?;
        Ok(m)
    }

    /// Checks indices and coordinates.
    pub fn validate(&self) -> Result<(), MeshError> {
        let count = self.positions.len();
        if u32::try_from(count).is_err() {
            return Err(MeshError::TooManyVertices(count));
        }
        if let Some(i) = self.positions.iter().position(|p| !crate::finite(p)) {
            return Err(MeshError::NonFinite(i));
        }
        for (t, tri) in self.triangles.iter().enumerate() {
            for &index in tri {
                if index as usize >= count {
                    return Err(MeshError::IndexOutOfRange { tri: t, index, count });
                }
            }
        }
        Ok(())
    }

    /// Number of triangles.
    #[must_use]
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Number of vertices.
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.positions.len()
    }

    /// Corners of triangle `i`.
    #[must_use]
    pub fn triangle(&self, i: usize) -> Option<[Point3; 3]> {
        let t = self.triangles.get(i)?;
        Some([
            *self.positions.get(t[0] as usize)?,
            *self.positions.get(t[1] as usize)?,
            *self.positions.get(t[2] as usize)?,
        ])
    }

    /// Unit normal of triangle `i` (`None` when degenerate or missing).
    #[must_use]
    pub fn face_normal(&self, i: usize) -> Option<Vec3> {
        let [a, b, c] = self.triangle(i)?;
        tri::normal(&a, &b, &c)
    }

    /// Centroid of triangle `i`.
    #[must_use]
    pub fn face_centroid(&self, i: usize) -> Option<Point3> {
        let [a, b, c] = self.triangle(i)?;
        Some(Point3::from((a.coords + b.coords + c.coords) / 3.0))
    }

    /// Area of triangle `i` (0 when missing).
    #[must_use]
    pub fn face_area(&self, i: usize) -> f64 {
        self.triangle(i).map_or(0.0, |[a, b, c]| tri::area(&a, &b, &c))
    }

    /// Area-weighted vertex normals (zero vector for isolated vertices).
    #[must_use]
    pub fn vertex_normals(&self) -> Vec<Vec3> {
        let mut n = vec![Vec3::zeros(); self.positions.len()];
        for t in &self.triangles {
            let (Some(a), Some(b), Some(c)) = (
                self.positions.get(t[0] as usize),
                self.positions.get(t[1] as usize),
                self.positions.get(t[2] as usize),
            ) else {
                continue;
            };
            let w = tri::cross(a, b, c);
            for &i in t {
                if let Some(v) = n.get_mut(i as usize) {
                    *v += w;
                }
            }
        }
        for v in &mut n {
            *v = crate::normalize(*v).unwrap_or_else(Vec3::zeros);
        }
        n
    }

    /// Total surface area.
    #[must_use]
    pub fn area(&self) -> f64 {
        (0..self.triangles.len()).map(|i| self.face_area(i)).sum()
    }

    /// Bounding box of the vertices.
    #[must_use]
    pub fn bbox(&self) -> Aabb {
        Aabb::from_points(&self.positions)
    }

    /// Area-weighted centroid of the surface (vertex mean when the area is zero).
    #[must_use]
    pub fn centroid(&self) -> Point3 {
        let mut acc = Vec3::zeros();
        let mut w = 0.0;
        for i in 0..self.triangles.len() {
            if let Some(c) = self.face_centroid(i) {
                let a = self.face_area(i);
                acc += c.coords * a;
                w += a;
            }
        }
        if w > 0.0 {
            return Point3::from(acc / w);
        }
        if self.positions.is_empty() {
            return Point3::origin();
        }
        let s: Vec3 = self.positions.iter().map(|p| p.coords).sum();
        Point3::from(s / self.positions.len() as f64)
    }

    /// A transformed copy.
    #[must_use]
    pub fn transformed(&self, t: &Transform) -> Mesh {
        Mesh { positions: self.positions.iter().map(|p| t.apply(p)).collect(), triangles: self.triangles.clone() }
    }

    /// Scales every coordinate by `s` about the origin (unit conversion on import).
    pub fn scale(&mut self, s: f64) {
        for p in &mut self.positions {
            *p = Point3::from(p.coords * s);
        }
    }

    /// Merges vertices closer than `tol` (grid hashing) and drops triangles that collapse.
    /// Returns the number of removed vertices.
    pub fn weld(&mut self, tol: f64) -> usize {
        let tol = if tol > 0.0 && tol.is_finite() { tol } else { 1e-9 };
        let inv = 1.0 / tol;
        let mut map: HashMap<[i64; 3], u32> = HashMap::with_capacity(self.positions.len());
        let mut remap = Vec::with_capacity(self.positions.len());
        let mut out: Vec<Point3> = Vec::with_capacity(self.positions.len());
        for p in &self.positions {
            let key = [(p.x * inv).round() as i64, (p.y * inv).round() as i64, (p.z * inv).round() as i64];
            let id = *map.entry(key).or_insert_with(|| {
                out.push(*p);
                (out.len() - 1) as u32
            });
            remap.push(id);
        }
        let removed = self.positions.len() - out.len();
        self.positions = out;
        let tris = std::mem::take(&mut self.triangles);
        self.triangles = tris
            .into_iter()
            .filter_map(|t| {
                let m = [*remap.get(t[0] as usize)?, *remap.get(t[1] as usize)?, *remap.get(t[2] as usize)?];
                (m[0] != m[1] && m[1] != m[2] && m[0] != m[2]).then_some(m)
            })
            .collect();
        removed
    }

    /// Drops triangles with area below `min_area`; returns how many were removed.
    pub fn remove_degenerate(&mut self, min_area: f64) -> usize {
        let before = self.triangles.len();
        let pos = &self.positions;
        self.triangles.retain(|t| match (pos.get(t[0] as usize), pos.get(t[1] as usize), pos.get(t[2] as usize)) {
            (Some(a), Some(b), Some(c)) => tri::area(a, b, c) > min_area,
            _ => false,
        });
        before - self.triangles.len()
    }

    /// A new mesh made of the listed triangles (vertices compacted). Unknown ids are skipped.
    #[must_use]
    pub fn submesh(&self, tri_ids: &[u32]) -> Mesh {
        let mut remap: HashMap<u32, u32> = HashMap::new();
        let mut positions = Vec::new();
        let mut triangles = Vec::with_capacity(tri_ids.len());
        for &ti in tri_ids {
            let Some(t) = self.triangles.get(ti as usize) else { continue };
            let mut nt = [0u32; 3];
            let mut ok = true;
            for (k, &vi) in t.iter().enumerate() {
                let Some(p) = self.positions.get(vi as usize) else {
                    ok = false;
                    break;
                };
                let id = *remap.entry(vi).or_insert_with(|| {
                    positions.push(*p);
                    (positions.len() - 1) as u32
                });
                if let Some(slot) = nt.get_mut(k) {
                    *slot = id;
                }
            }
            if ok {
                triangles.push(nt);
            }
        }
        Mesh { positions, triangles }
    }

    /// Per triangle, the neighbouring triangle across each edge (`u32::MAX` for a boundary or
    /// a non-manifold edge). Edge `k` runs from corner `k` to corner `k + 1`.
    #[must_use]
    pub fn triangle_neighbours(&self) -> Vec<[u32; 3]> {
        // sort half-edges by their undirected key (linear memory, no hash map: 10 M triangles)
        let mut half: Vec<(u64, u32)> = Vec::with_capacity(self.triangles.len() * 3);
        for (ti, t) in self.triangles.iter().enumerate() {
            for k in 0..3u32 {
                let a = t[k as usize];
                let b = t[((k + 1) % 3) as usize];
                let key = (u64::from(a.min(b)) << 32) | u64::from(a.max(b));
                half.push((key, ti as u32 * 3 + k));
            }
        }
        crate::par::sort_unstable(&mut half);
        let mut n = vec![[u32::MAX; 3]; self.triangles.len()];
        let mut i = 0;
        while i < half.len() {
            let mut j = i + 1;
            while j < half.len() && half[j].0 == half[i].0 {
                j += 1;
            }
            // manifold edges only: exactly two half-edges
            if j - i == 2 {
                let (h0, h1) = (half[i].1, half[i + 1].1);
                if let Some(s) = n.get_mut((h0 / 3) as usize).and_then(|x| x.get_mut((h0 % 3) as usize)) {
                    *s = h1 / 3;
                }
                if let Some(s) = n.get_mut((h1 / 3) as usize).and_then(|x| x.get_mut((h1 % 3) as usize)) {
                    *s = h0 / 3;
                }
            }
            i = j;
        }
        n
    }

    /// `n` points sampled uniformly by area, with the normal of the triangle they lie on.
    #[must_use]
    pub fn sample_points(&self, n: usize, rng: &mut Rng) -> Vec<(Point3, Vec3)> {
        let mut cdf = Vec::with_capacity(self.triangles.len());
        let mut acc = 0.0;
        for i in 0..self.triangles.len() {
            acc += self.face_area(i);
            cdf.push(acc);
        }
        let mut out = Vec::with_capacity(n);
        if acc <= 0.0 {
            return out;
        }
        for _ in 0..n {
            let r = rng.uniform() * acc;
            let ti = cdf.partition_point(|&c| c < r).min(self.triangles.len().saturating_sub(1));
            let (Some([a, b, c]), Some(nrm)) = (self.triangle(ti), self.face_normal(ti)) else { continue };
            let (mut u, mut v) = (rng.uniform(), rng.uniform());
            if u + v > 1.0 {
                u = 1.0 - u;
                v = 1.0 - v;
            }
            out.push((a + (b - a) * u + (c - a) * v, nrm));
        }
        out
    }

    /// Appends `other` (vertex indices shifted).
    pub fn append(&mut self, other: &Mesh) {
        let base = self.positions.len() as u32;
        self.positions.extend_from_slice(&other.positions);
        self.triangles.extend(other.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quad() -> Mesh {
        Mesh::new(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )
        .unwrap()
    }

    #[test]
    fn basics() {
        let m = quad();
        assert!((m.area() - 1.0).abs() < 1e-15);
        assert_eq!(m.face_normal(0).unwrap(), Vec3::z());
        assert_eq!(m.vertex_normals()[0], Vec3::z());
        let n = m.triangle_neighbours();
        assert_eq!(n[0][2], 1);
        assert_eq!(n[1][0], 0);
        let s = m.submesh(&[1, 99]);
        assert_eq!(s.triangle_count(), 1);
        assert_eq!(s.vertex_count(), 3);
        assert!((m.centroid() - Point3::new(0.5, 0.5, 0.0)).norm() < 1e-12);
        let pts = m.sample_points(100, &mut Rng::new(1));
        assert_eq!(pts.len(), 100);
        assert!(pts.iter().all(|(p, _)| p.z == 0.0 && (0.0..=1.0).contains(&p.x)));
    }

    #[test]
    fn rejects_bad_input() {
        assert!(matches!(Mesh::new(vec![Point3::origin()], vec![[0, 0, 5]]), Err(MeshError::IndexOutOfRange { .. })));
        assert!(matches!(Mesh::new(vec![Point3::new(f64::NAN, 0.0, 0.0)], vec![]), Err(MeshError::NonFinite(0))));
    }

    #[test]
    fn weld_merges_duplicates() {
        // STL-style soup: each triangle with its own vertices
        let mut m = Mesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            triangles: vec![[0, 1, 2], [3, 4, 5]],
        };
        assert_eq!(m.weld(1e-6), 2);
        assert_eq!(m.vertex_count(), 4);
        assert_eq!(m.triangle_neighbours()[0][2], 1);
    }
}

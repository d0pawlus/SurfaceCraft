//! The scanned cloud the planner measures against, with the area each vertex stands for (so that
//! what is measured does not depend on how densely the scan was taken) and the lengths the profile
//! code scales with the scan's spacing.
use std::sync::OnceLock;
use surfacecraft_geom::{KdTree, Mesh, Point3, Vec3};

/// Triangles whose edges are measured for the spacing (an even sample).
const SPACING_SAMPLE: usize = 20_000;
/// Spacing assumed when the scan has no measurable edge (mm).
const DEFAULT_SPACING: f64 = 1.0;
/// Vertex area over spacing squared for a mesh of equilateral triangles.
const VERTEX_AREA_PER_SPACING_SQ: f64 = 0.866;
/// The spacing (mm) at which the lengths of the planner were set; [`Scale::mm`] scales them.
const REFERENCE_SPACING: f64 = 0.9;

/// A scan: the vertices, the area of surface each stands for and the typical edge length.
pub struct Scan {
    /// The vertices.
    pub points: Vec<Point3>,
    /// A third of the area of the triangles round each vertex (mm²).
    pub areas: Vec<f64>,
    /// Median edge length (mm).
    pub spacing: f64,
    bounds: Option<(Vec3, Vec3)>,
    tree: OnceLock<KdTree>,
}

fn bounds_of(points: &[Point3]) -> Option<(Vec3, Vec3)> {
    let (lo, hi) = points
        .iter()
        .filter(|p| p.coords.iter().all(|c| c.is_finite()))
        .fold((Vec3::repeat(f64::INFINITY), Vec3::repeat(f64::NEG_INFINITY)), |(lo, hi), p| {
            (lo.inf(&p.coords), hi.sup(&p.coords))
        });
    lo.iter().chain(hi.iter()).all(|c| c.is_finite()).then_some((lo, hi))
}

/// Median edge length of an even sample of the triangles.
fn median_edge(mesh: &Mesh) -> f64 {
    let step = (mesh.triangles.len() / SPACING_SAMPLE).max(1);
    let mut lengths: Vec<f64> = Vec::new();
    for t in mesh.triangles.iter().step_by(step) {
        for k in 0..3 {
            let (a, b) = (t.get(k).copied(), t.get((k + 1) % 3).copied());
            if let (Some(p), Some(q)) =
                (a.and_then(|i| mesh.positions.get(i as usize)), b.and_then(|i| mesh.positions.get(i as usize)))
            {
                let l = (p - q).norm();
                if l.is_finite() && l > 0.0 {
                    lengths.push(l);
                }
            }
        }
    }
    lengths.sort_by(f64::total_cmp);
    lengths.get(lengths.len() / 2).copied().unwrap_or(DEFAULT_SPACING)
}

impl Scan {
    /// The vertices of `mesh` with their areas.
    #[must_use]
    pub fn from_mesh(mesh: &Mesh) -> Scan {
        let mut areas = vec![0.0; mesh.positions.len()];
        for (t, tri) in mesh.triangles.iter().enumerate() {
            let a = mesh.face_area(t) / 3.0;
            if a.is_finite() {
                for &v in tri {
                    if let Some(x) = areas.get_mut(v as usize) {
                        *x += a;
                    }
                }
            }
        }
        Scan {
            points: mesh.positions.clone(),
            areas,
            spacing: median_edge(mesh),
            bounds: bounds_of(&mesh.positions),
            tree: OnceLock::new(),
        }
    }

    /// Points of a scan of the given spacing, each standing for the area of a regular mesh.
    #[must_use]
    pub fn from_points(points: Vec<Point3>, spacing: f64) -> Scan {
        let spacing = if spacing.is_finite() && spacing > 0.0 { spacing } else { DEFAULT_SPACING };
        let areas = vec![VERTEX_AREA_PER_SPACING_SQ * spacing * spacing; points.len()];
        Scan { bounds: bounds_of(&points), points, areas, spacing, tree: OnceLock::new() }
    }

    /// Points with the area of each (for scans that are not meshes).
    #[must_use]
    pub fn from_points_with_areas(points: Vec<Point3>, areas: Vec<f64>, spacing: f64) -> Scan {
        let spacing = if spacing.is_finite() && spacing > 0.0 { spacing } else { DEFAULT_SPACING };
        Scan { bounds: bounds_of(&points), points, areas, spacing, tree: OnceLock::new() }
    }

    /// The corners of the box that holds the scan.
    #[must_use]
    pub fn bounds(&self) -> Option<(Vec3, Vec3)> {
        self.bounds
    }

    /// The vertices within `radius` of `c` (indices into `points`).
    #[must_use]
    pub fn within(&self, c: &Point3, radius: f64) -> Vec<usize> {
        self.tree.get_or_init(|| KdTree::build(&self.points)).within(c, radius)
    }

    /// The lengths of the planner at this scan's spacing.
    #[must_use]
    pub(super) fn scale(&self) -> Scale {
        Scale { spacing: self.spacing }
    }
}

/// Lengths of the planner that depend on the size of the scan's features: set in millimetres at
/// [`REFERENCE_SPACING`] and scaled with the scan's spacing.
#[derive(Clone, Copy, Debug)]
pub(super) struct Scale {
    pub(super) spacing: f64,
}

impl Scale {
    /// `mm` (a length at the reference spacing) at this scan's spacing.
    pub(super) fn mm(&self, mm: f64) -> f64 {
        mm * self.spacing / REFERENCE_SPACING
    }
    /// An area in mm² at the reference spacing, scaled likewise.
    pub(super) fn mm2(&self, mm2: f64) -> f64 {
        let k = self.spacing / REFERENCE_SPACING;
        mm2 * k * k
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spacing_and_vertex_areas_follow_the_mesh() {
        let (m, n) = (20, 20);
        let h = 0.5;
        let mut positions = Vec::new();
        for j in 0..=n {
            for i in 0..=m {
                positions.push(Point3::new(i as f64 * h, j as f64 * h, 0.0));
            }
        }
        let mut triangles = Vec::new();
        for j in 0..n {
            for i in 0..m {
                let a = (j * (m + 1) + i) as u32;
                let (b, c, d) = (a + 1, a + (m + 1) as u32, a + (m + 1) as u32 + 1);
                triangles.extend([[a, b, c], [b, d, c]]);
            }
        }
        let scan = Scan::from_mesh(&Mesh::new(positions, triangles).unwrap());
        let total: f64 = scan.areas.iter().sum();
        assert!((total - 100.0).abs() < 1e-9, "the vertex areas add up to the surface: {total}");
        assert!((scan.spacing - h).abs() < 1e-9 || (scan.spacing - h * 2f64.sqrt()).abs() < 1e-9, "{}", scan.spacing);
        assert!(Scan::from_points(vec![], f64::NAN).spacing > 0.0);
    }
}

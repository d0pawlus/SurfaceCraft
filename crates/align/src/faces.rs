//! Face statistics for orientation voting (`auto`): per-triangle normal, centroid and area, a
//! coarse histogram of normal directions for seeds, and a robust planar-cluster extraction.
//!
//! A scan's per-triangle normals scatter by several degrees at 0.5 mm spacing (more where the
//! noise is large against the spacing), so each face takes the blend of its corners' area-weighted
//! vertex normals, which averages over about two rings of triangles. The histogram only finds
//! *seeds*; every seed is then refined by mean shift on the normals, a peak search on
//! the plane offsets (so that two parallel faces at different heights stay apart) and a trimmed
//! least-squares plane fit of the triangle centroids. Spikes and debris fall outside the slab.
use nalgebra::SymmetricEigen;
use surfacecraft_geom::{Mat3, Mesh, Point3, Vec3, normalize, par};

/// Faces closer than 30 degrees to a seed normal take part in its cluster.
pub(crate) const GATE_COS: f64 = 0.866_025_403_784_438_6;
/// Cells per side of each cube-map face of the direction histogram.
const GRID: usize = 16;
/// Two seeds are at least 25 degrees apart.
const SEED_SEPARATION_COS: f64 = 0.906_307_787_036_65;
/// A seed needs this share of the total area in its cell.
const SEED_MIN_SHARE: f64 = 0.01;
/// Refinement rounds of a planar cluster (select, fit, shrink the slab).
const PLANE_ROUNDS: usize = 4;
/// A seed's candidate faces lie within 50 degrees of it (the mean shift moves less than 20).
const CONE_COS: f64 = 0.642_787_609_686_539_4;
/// Mean-shift rounds of a seed direction.
const SHIFT_ROUNDS: usize = 4;
/// Slab search bin as a share of the bounding-box diagonal.
const SLAB_BIN: f64 = 0.01;
/// Floor of the slab half width as a share of the diagonal (noise-free meshes).
const MIN_HALF_WIDTH: f64 = 2e-4;
/// Slab half width in robust standard deviations of the residuals.
const SLAB_SIGMAS: f64 = 3.0;
/// Scale of the median absolute deviation to a standard deviation.
const MAD_TO_SIGMA: f64 = 1.482_6;
/// A cluster is flat when its slab half width is at most this share of the square root of its
/// area (a cap of a sphere or a cylinder's tangent strip is as thick as it is wide).
const FLATNESS: f64 = 0.02;

/// One triangle.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Face {
    /// Index of the triangle in the mesh.
    pub tri: u32,
    pub n: Vec3,
    pub c: Point3,
    pub w: f64,
}

/// The usable triangles of a mesh with their totals.
pub(crate) struct FaceSet<'m> {
    pub mesh: &'m Mesh,
    pub faces: Vec<Face>,
    pub total: f64,
    pub diag: f64,
    /// Area-weighted centroid of the surface.
    pub centroid: Point3,
}

/// A flat part of the surface: a planar cluster or an open rim standing for a missing face.
#[derive(Clone, Debug)]
pub(crate) struct PlaneFace {
    /// Unit normal (its sign follows the triangles' winding; nothing relies on it).
    pub normal: Vec3,
    /// Area-weighted centroid of the face.
    pub point: Point3,
    pub area: f64,
    /// Half width of the slab the cluster was cut with.
    pub half_width: f64,
    pub rim: bool,
}

/// The triangle `i` with the blend of its corners' vertex normals (its own normal where that
/// blend is missing or points the other way).
fn face_of(mesh: &Mesh, vertex_normals: &[Vec3], i: usize) -> Option<Face> {
    let [a, b, c] = mesh.triangle(i)?;
    let corner = mesh.triangles.get(i)?;
    let cross = (b - a).cross(&(c - a));
    let len = cross.norm();
    let centre = Point3::from((a.coords + b.coords + c.coords) / 3.0);
    if !(len > 1e-18) || !len.is_finite() || !surfacecraft_geom::finite(&centre) {
        return None;
    }
    let own = cross / len;
    let blend: Vec3 = corner.iter().filter_map(|&v| vertex_normals.get(v as usize)).sum();
    let n = normalize(blend).filter(|b| b.dot(&own) > 0.0).unwrap_or(own);
    Some(Face { tri: i as u32, n, c: centre, w: 0.5 * len })
}

impl<'m> FaceSet<'m> {
    /// The triangles of `mesh` with an area and finite corners; `None` when there are none.
    pub(crate) fn new(mesh: &'m Mesh) -> Option<Self> {
        let vertex_normals = mesh.vertex_normals();
        let faces: Vec<Face> =
            par::map(mesh.triangle_count(), |i| face_of(mesh, &vertex_normals, i)).into_iter().flatten().collect();
        let total: f64 = faces.iter().map(|f| f.w).sum();
        if faces.is_empty() || !(total > 0.0) || !total.is_finite() {
            return None;
        }
        let sum: Vec3 = faces.iter().map(|f| f.c.coords * f.w).sum();
        let diag = mesh.bbox().diagonal();
        if !(diag > 0.0) || !diag.is_finite() {
            return None;
        }
        Some(Self { mesh, faces, total, diag, centroid: Point3::from(sum / total) })
    }
}

/// The cell of the cube-map histogram that holds direction `n`.
fn cell(n: &Vec3) -> usize {
    let (ax, ay, az) = (n.x.abs(), n.y.abs(), n.z.abs());
    let (face, a, b, m) = if ax >= ay && ax >= az {
        (usize::from(n.x < 0.0), n.y, n.z, ax)
    } else if ay >= az {
        (2 + usize::from(n.y < 0.0), n.x, n.z, ay)
    } else {
        (4 + usize::from(n.z < 0.0), n.x, n.y, az)
    };
    let bin = |t: f64| (((t / m + 1.0) * 0.5 * GRID as f64) as usize).min(GRID - 1);
    face * GRID * GRID + bin(b) * GRID + bin(a)
}

/// Up to `count` well-separated dominant normal directions, heaviest first (coarse: refine them
/// with [`plane_face`]).
pub(crate) fn dominant_directions(set: &FaceSet, count: usize) -> Vec<Vec3> {
    let mut weight = vec![0.0_f64; 6 * GRID * GRID];
    let mut sum = vec![Vec3::zeros(); 6 * GRID * GRID];
    for f in &set.faces {
        let i = cell(&f.n);
        if let (Some(w), Some(s)) = (weight.get_mut(i), sum.get_mut(i)) {
            *w += f.w;
            *s += f.n * f.w;
        }
    }
    let mut dirs: Vec<(f64, Vec3)> =
        weight.iter().zip(&sum).filter_map(|(w, s)| normalize(*s).map(|d| (*w, d))).collect();
    dirs.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut seeds: Vec<Vec3> = Vec::new();
    for (w, d) in dirs {
        if seeds.len() >= count || w < SEED_MIN_SHARE * set.total {
            break;
        }
        if seeds.iter().all(|s| s.dot(&d) < SEED_SEPARATION_COS) {
            seeds.push(d);
        }
    }
    seeds
}

fn mean_shift(cone: &[&Face], seed: Vec3) -> Option<Vec3> {
    let mut n = seed;
    for _ in 0..SHIFT_ROUNDS {
        let s = cone.iter().filter(|f| f.n.dot(&n) > GATE_COS).fold(Vec3::zeros(), |a, f| a + f.n * f.w);
        n = normalize(s)?;
    }
    Some(n)
}

/// The offset (along `n`) where the faces within the cone pile up: the heaviest two adjacent bins.
fn peak_offset(cone: &[&Face], n: &Vec3, bin: f64) -> Option<f64> {
    let gated = || cone.iter().filter(|f| f.n.dot(n) > GATE_COS);
    let (lo, hi) = gated().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), f| {
        let d = f.c.coords.dot(n);
        (lo.min(d), hi.max(d))
    });
    if !(hi >= lo) {
        return None;
    }
    let bins = (((hi - lo) / bin).min(65_536.0) as usize) + 2;
    let mut acc = vec![(0.0_f64, 0.0_f64); bins];
    for f in gated() {
        let d = f.c.coords.dot(n);
        let i = (((d - lo) / bin) as usize).min(bins - 1);
        if let Some(a) = acc.get_mut(i) {
            a.0 += f.w;
            a.1 += f.w * d;
        }
    }
    let pair = |i: usize| {
        let (a, b) = (acc.get(i).copied().unwrap_or_default(), acc.get(i + 1).copied().unwrap_or_default());
        (a.0 + b.0, a.1 + b.1)
    };
    let (w, wd) = (0..bins).map(pair).max_by(|a, b| a.0.total_cmp(&b.0))?;
    (w > 0.0).then(|| wd / w)
}

/// Weighted centroid and the least-squares plane normal (oriented along `orient`) of faces.
fn weighted_plane(items: &[&Face], orient: &Vec3) -> Option<(Point3, Vec3)> {
    let w: f64 = items.iter().map(|f| f.w).sum();
    if items.len() < 4 || !(w > 0.0) {
        return None;
    }
    let mean: Vec3 = items.iter().map(|f| f.c.coords * f.w).sum::<Vec3>() / w;
    let mut cov = Mat3::zeros();
    for f in items {
        let d = f.c.coords - mean;
        cov += (d * d.transpose()) * f.w;
    }
    let eig = SymmetricEigen::new(cov);
    let mut order = [0usize, 1, 2];
    order.sort_by(|&i, &j| eig.eigenvalues[i].total_cmp(&eig.eigenvalues[j]));
    // a line of centroids is no plane
    if eig.eigenvalues[order[1]] <= 1e-12 * eig.eigenvalues[order[2]].max(1e-300) {
        return None;
    }
    let mut normal = normalize(eig.eigenvectors.column(order[0]).into_owned())?;
    if normal.dot(orient) < 0.0 {
        normal = -normal;
    }
    Some((Point3::from(mean), normal))
}

fn slab<'a>(cone: &[&'a Face], n: &Vec3, d0: f64, half: f64) -> Vec<&'a Face> {
    cone.iter().copied().filter(|f| f.n.dot(n) > GATE_COS && (f.c.coords.dot(n) - d0).abs() <= half).collect()
}

/// Half width of the slab: three robust standard deviations of the residuals, kept between the
/// noise-free floor and the search bin.
fn robust_half_width(items: &[&Face], centre: &Point3, n: &Vec3, diag: f64, bin: f64) -> f64 {
    let mut res: Vec<f64> = items.iter().map(|f| ((f.c - centre).dot(n)).abs()).collect();
    let mid = res.len() / 2;
    if res.is_empty() {
        return MIN_HALF_WIDTH * diag;
    }
    let median = *res.select_nth_unstable_by(mid, f64::total_cmp).1;
    (SLAB_SIGMAS * MAD_TO_SIGMA * median).clamp(MIN_HALF_WIDTH * diag, 1.5 * bin)
}

/// The largest planar cluster of triangles whose normals lie within 30 degrees of `seed`: the
/// face a part could lie on, with its exact plane. `None` when nothing planar is there.
pub(crate) fn plane_face(set: &FaceSet, seed: &Vec3) -> Option<PlaneFace> {
    let seed = normalize(*seed)?;
    let cone: Vec<&Face> = set.faces.iter().filter(|f| f.n.dot(&seed) > CONE_COS).collect();
    let mut n = mean_shift(&cone, seed)?;
    let bin = (SLAB_BIN * set.diag).max(1e-9);
    let mut d0 = peak_offset(&cone, &n, bin)?;
    let mut half = 1.5 * bin;
    let mut fit: Option<(Point3, Vec3)> = None;
    for _ in 0..PLANE_ROUNDS {
        let items = slab(&cone, &n, d0, half);
        let (c, m) = weighted_plane(&items, &n)?;
        n = m;
        d0 = c.coords.dot(&n);
        half = robust_half_width(&items, &c, &n, set.diag, bin);
        fit = Some((c, n));
    }
    let (_, normal) = fit?;
    let items = slab(&cone, &normal, d0, half);
    let area: f64 = items.iter().map(|f| f.w).sum();
    if !(area > 0.0) || half > FLATNESS * area.sqrt() {
        return None;
    }
    let point = Point3::from(items.iter().map(|f| f.c.coords * f.w).sum::<Vec3>() / area);
    Some(PlaneFace { normal, point, area, half_width: half, rim: false })
}

/// The share of a triangle's area where its linear height is above `t`, given the heights of
/// its corners (exact: the triangle is clipped by the level line).
pub(crate) fn fraction_above(heights: [f64; 3], t: f64) -> f64 {
    let mut h = heights;
    h.sort_by(f64::total_cmp);
    let [h0, h1, h2] = h;
    if t >= h2 {
        0.0
    } else if t < h0 {
        1.0
    } else if t >= h1 {
        (h2 - t) * (h2 - t) / ((h2 - h0) * (h2 - h1))
    } else {
        1.0 - (t - h0) * (t - h0) / ((h2 - h0) * (h1 - h0))
    }
}

/// How much of the surface lies on the minority side of the plane of `face` (beyond twice the
/// slab half width), and the direction pointing into the majority side (where the material is).
/// A face a part can stand on has nothing on its minority side. Triangles that cross the
/// tolerance planes are clipped, so the area on each side does not depend on how the surface is
/// divided into triangles.
pub(crate) fn material_side(set: &FaceSet, face: &PlaneFace) -> (f64, Vec3) {
    let tol = 2.0 * face.half_width;
    let (mut pos, mut neg) = (0.0_f64, 0.0_f64);
    for f in &set.faces {
        let Some(corners) = set.mesh.triangle(f.tri as usize) else { continue };
        let d = corners.map(|p| (p - face.point).dot(&face.normal));
        pos += f.w * fraction_above(d, tol);
        neg += f.w * (1.0 - fraction_above(d, -tol));
    }
    let up = if pos >= neg { face.normal } else { -face.normal };
    (pos.min(neg) / set.total, up)
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Rng, synth};

    fn noisy_box() -> Mesh {
        let mut m = synth::cuboid(Point3::new(1.0, 2.0, 3.0), Vec3::new(60.0, 40.0, 20.0), 30);
        synth::add_noise(&mut m, 0.05, &mut Rng::new(3));
        m
    }

    #[test]
    fn dominant_directions_find_the_six_faces() {
        let mesh = noisy_box();
        let set = FaceSet::new(&mesh).unwrap();
        let dirs = dominant_directions(&set, 10);
        assert_eq!(dirs.len(), 6, "{dirs:?}");
        for axis in [Vec3::x(), Vec3::y(), Vec3::z()] {
            assert!(dirs.iter().any(|d| d.dot(&axis) > 0.99));
            assert!(dirs.iter().any(|d| d.dot(&-axis) > 0.99));
        }
    }

    #[test]
    fn plane_face_is_exact_and_keeps_opposite_faces_apart() {
        let mesh = noisy_box();
        let set = FaceSet::new(&mesh).unwrap();
        let top = plane_face(&set, &Vec3::new(0.05, 0.0, 1.0)).unwrap();
        assert!(top.normal.dot(&Vec3::z()) > 0.999_999, "{:?}", top.normal);
        assert!((top.point.z - 23.0).abs() < 0.01, "{}", top.point.z);
        assert!((top.area - 2400.0).abs() < 30.0, "{}", top.area);
        let bottom = plane_face(&set, &-Vec3::z()).unwrap();
        assert!((bottom.point.z - 3.0).abs() < 0.01);
        let (minority, up) = material_side(&set, &bottom);
        assert!(minority < 1e-3 && up.dot(&Vec3::z()) > 0.99, "{minority} {up:?}");
    }

    #[test]
    fn a_crossing_triangle_counts_the_area_on_each_side() {
        // heights [-1, 1, 1]: a quarter of the area lies below 0
        assert!((1.0 - fraction_above([-1.0, 1.0, 1.0], 0.0) - 0.25).abs() < 1e-15);
        assert_eq!(fraction_above([1.0, 2.0, 3.0], 0.5), 1.0);
        assert_eq!(fraction_above([1.0, 2.0, 3.0], 3.0), 0.0);
        assert!((fraction_above([0.0, 0.0, 2.0], 1.0) - 0.25).abs() < 1e-15);
        assert_eq!(fraction_above([0.0, 0.0, 0.0], 0.0), 0.0);
    }

    /// A sheet crossing the base plane diagonally: the overhang is the same for a 2-triangle and
    /// a 40 x 40 triangulation of it, and equals the analytic area.
    #[test]
    fn the_support_test_does_not_depend_on_the_triangulation() {
        let tilt = |n: usize| {
            let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 20.0, 10.0, n, n);
            for p in &mut m.positions {
                p.z = 0.5 * (p.x - 12.0); // crosses z = 0 at x = 12, 40 % along the sheet
            }
            m
        };
        let base = PlaneFace { normal: Vec3::z(), point: Point3::origin(), area: 1.0, half_width: 0.25, rim: false };
        let results: Vec<f64> = [1usize, 2, 7, 40]
            .iter()
            .map(|&n| {
                let m = tilt(n);
                let set = FaceSet::new(&m).unwrap();
                material_side(&set, &base).0 * set.total
            })
            .collect();
        let want = 70.0 * 1.25_f64.sqrt(); // the sheet is tilted: its area is 1.118 times its footprint
        // below the tolerance plane z = -0.5: x < 11, area 11 x 10; above z = 0.5: x > 13: 7 x 10
        for r in &results {
            assert!((r - want).abs() < 1e-9 * want, "{results:?}");
        }
    }

    #[test]
    fn a_plane_with_a_protrusion_is_not_supported() {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), 20);
        m.append(&synth::cylinder(Point3::new(30.0, 20.0, 20.0), Vec3::z(), 8.0, 15.0, 48, 8));
        let set = FaceSet::new(&m).unwrap();
        let top = plane_face(&set, &Vec3::z()).unwrap();
        let bottom = plane_face(&set, &-Vec3::z()).unwrap();
        assert!(material_side(&set, &top).0 > 0.05);
        assert!(material_side(&set, &bottom).0 < 1e-3);
    }
}

//! Hotspots: groups of scan triangles that deviate on one side, and the volume error of the scan.

use super::sample::*;
use super::*;

/// The side of a group: `positive` is excess material (the scan lies outside the reference),
/// `negative` is missing material.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Sign {
    /// Scan outside the reference: excess material.
    Positive,
    /// Scan inside the reference: missing material.
    Negative,
}

/// A connected group of scan triangles whose |deviation| is above the tolerance.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Hotspot {
    /// Triangles in the group.
    pub triangles: usize,
    /// Footprint area on the reference (mm²): each triangle's area projected along the reference
    /// normal. Sloped borders of a step add little here; a flat patch has its plain area.
    pub area: f64,
    /// Surface area of the scan triangles (mm²), sloped borders included.
    pub surface_area: f64,
    /// Signed deviation of the triangle with the largest |deviation| (mm).
    pub max: f64,
    /// Area-weighted mean signed deviation (mm).
    pub mean: f64,
    /// Area-weighted centroid (mm).
    pub centroid: [f64; 3],
    /// Side of the group.
    pub sign: Sign,
}

/// Volume error of the scan against the reference: the sum over triangles of footprint area × mean
/// signed deviation (the material between the two surfaces, measured along the reference normal),
/// split by sign.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Volume {
    /// Excess material, mm³ (≥ 0): positive triangles.
    pub excess_mm3: f64,
    /// Missing material, mm³ (≤ 0): negative triangles.
    pub missing_mm3: f64,
    /// Excess plus missing, mm³.
    pub net_mm3: f64,
    /// Footprint area of the measured triangles on the reference (mm²), see [`Hotspot::area`].
    pub area_mm2: f64,
    /// Measured triangles: all three corners counted and in scope.
    pub triangles: usize,
}

/// Hotspot groups (sorted by area, largest first) and the volume error.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct HotspotReport {
    /// Groups with area at least `min_area`, largest first.
    pub groups: Vec<Hotspot>,
    /// Volume error over every measured triangle.
    pub volume: Volume,
}

/// A measured scan triangle: mean signed deviation of its corners, footprint and surface areas, and
/// centroid.
#[derive(Clone, Copy)]
struct TriValue {
    d: f64,
    area: f64,
    surface: f64,
    centre: Point3,
}

/// The value of triangle `t` when it is in scope and all three corners are counted. Its footprint
/// is the triangle's area projected along the mean reference normal of its corners; without
/// reference normals (a freeform surface) the footprint is the surface area.
fn triangle_value(mesh: &Mesh, samples: &[Sample], t: usize) -> Option<TriValue> {
    let corners = mesh.triangles.get(t)?;
    let mut sum = 0.0;
    let mut refs = Vec3::zeros();
    for &vi in corners {
        let s = samples.get(vi as usize)?;
        if !s.counted() {
            return None;
        }
        sum += s.d;
        refs += Vec3::from(s.reference);
    }
    let [a, b, c] = mesh.triangle(t)?;
    let surface = mesh.face_area(t);
    let area = match surfacecraft_geom::normalize(refs) {
        Some(n) => 0.5 * (b - a).cross(&(c - a)).dot(&n).abs(),
        None => surface,
    };
    Some(TriValue { d: sum / 3.0, area, surface, centre: mesh.face_centroid(t)? })
}

/// Connected groups of scan triangles with |deviation| above `tolerance`, and the volume error.
///
/// Triangles are measured when they are in `in_scope` and all three corners are counted (see
/// [`Sample`]); their deviation is the mean of the corners. Areas are footprints on the reference:
/// a triangle's area projected along the reference normal. Sloped borders of a step therefore add
/// little area, and a wall (normal perpendicular to the reference) adds none. Groups connect through edges shared by
/// exactly two triangles, and only triangles of the same sign join a group. Groups smaller than
/// `min_area` (mm²) are dropped; the rest are sorted by area, largest first.
#[must_use]
pub fn hotspots(mesh: &Mesh, samples: &[Sample], in_scope: &[bool], tolerance: f64, min_area: f64) -> HotspotReport {
    let nt = mesh.triangle_count();
    let tri: Vec<Option<TriValue>> = (0..nt)
        .map(|t| if in_scope.get(t).copied().unwrap_or(false) { triangle_value(mesh, samples, t) } else { None })
        .collect();
    let mut volume = Volume::default();
    for v in tri.iter().flatten() {
        volume.area_mm2 += v.area;
        volume.triangles += 1;
        if v.d > 0.0 {
            volume.excess_mm3 += v.area * v.d;
        } else if v.d < 0.0 {
            volume.missing_mm3 += v.area * v.d;
        }
    }
    volume.net_mm3 = volume.excess_mm3 + volume.missing_mm3;
    let neighbours = mesh.triangle_neighbours();
    let mut seen = vec![false; nt];
    let mut groups = Vec::new();
    for start in 0..nt {
        let already = seen.get(start).copied().unwrap_or(true);
        let Some(first) = tri.get(start).copied().flatten() else { continue };
        if already || first.d.abs() <= tolerance {
            continue;
        }
        let positive = first.d > 0.0;
        if let Some(s) = seen.get_mut(start) {
            *s = true;
        }
        let mut stack = vec![start];
        let mut members: Vec<TriValue> = Vec::new();
        while let Some(t) = stack.pop() {
            let Some(v) = tri.get(t).copied().flatten() else { continue };
            members.push(v);
            for &nb in neighbours.get(t).into_iter().flatten() {
                let nb = nb as usize;
                // boundary edges hold u32::MAX, which is out of range
                let Some(mark) = seen.get_mut(nb) else { continue };
                if *mark {
                    continue;
                }
                let Some(w) = tri.get(nb).copied().flatten() else { continue };
                if w.d.abs() <= tolerance || (w.d > 0.0) != positive {
                    continue;
                }
                *mark = true;
                stack.push(nb);
            }
        }
        let n = members.len();
        let (mut area, mut surface, mut area_d, mut sum_d, mut area_c, mut sum_c) =
            (0.0, 0.0, 0.0, 0.0, Vec3::zeros(), Vec3::zeros());
        let mut max = first.d;
        for v in &members {
            area += v.area;
            surface += v.surface;
            area_d += v.area * v.d;
            sum_d += v.d;
            area_c += v.centre.coords * v.area;
            sum_c += v.centre.coords;
            if v.d.abs() > max.abs() {
                max = v.d;
            }
        }
        let (mean, centre) =
            if area > 0.0 { (area_d / area, area_c / area) } else { (sum_d / n as f64, sum_c / n as f64) };
        if area >= min_area {
            groups.push(Hotspot {
                triangles: n,
                area,
                surface_area: surface,
                max,
                mean,
                centroid: [centre.x, centre.y, centre.z],
                sign: if positive { Sign::Positive } else { Sign::Negative },
            });
        }
    }
    groups.sort_by(|a, b| b.area.total_cmp(&a.area));
    HotspotReport { groups, volume }
}

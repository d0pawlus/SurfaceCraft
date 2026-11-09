//! SurfaceCraft deviation analysis (layer L2): how far scan points are from a reference, as
//! statistics and a colour map (design doc §7: "deviation as constant feedback").
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_fit::Primitive;
use surfacecraft_geom::{Bvh, KdTree, Mesh, Point3, Vec3};

/// Colours of the deviation map, linear RGBA in 0..1. [`colorize`] paints the viewport from these
/// constants, and the UI legend resolves the same values through its theme tokens, so the legend
/// and the map cannot drift apart.
pub mod palette {
    /// Within ±tolerance (green).
    pub const IN_TOLERANCE: [f32; 4] = [0.2, 0.8, 0.3, 1.0];
    /// Just above +tolerance: the start of the above-ramp (yellow-orange).
    pub const ABOVE_NEAR: [f32; 4] = [1.0, 0.85, 0.1, 1.0];
    /// At +3·tolerance and beyond: the end of the above-ramp (red).
    pub const ABOVE_FAR: [f32; 4] = [1.0, 0.0, 0.1, 1.0];
    /// Just below −tolerance: the start of the below-ramp (cyan).
    pub const BELOW_NEAR: [f32; 4] = [0.1, 0.75, 1.0, 1.0];
    /// At −3·tolerance and beyond: the end of the below-ramp (blue).
    pub const BELOW_FAR: [f32; 4] = [0.1, 0.0, 1.0, 1.0];
    /// Points that were not measured (NaN, farther than the reference's reach).
    pub const UNMEASURED: [f32; 4] = [0.45, 0.45, 0.45, 1.0];
}

/// Linear interpolation of two RGBA colours (`k` in 0..=1).
fn lerp(a: [f32; 4], b: [f32; 4], k: f32) -> [f32; 4] {
    [a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k, a[2] + (b[2] - a[2]) * k, 1.0]
}

/// Deviation statistics.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct DeviationStats {
    /// Points measured (facing the reference, see [`Sample`]).
    pub count: usize,
    /// Points left out because their normal points away from the reference normal.
    #[serde(default)]
    pub not_facing: usize,
    /// Smallest signed deviation (mm; 0 without points).
    pub min: f64,
    /// Largest signed deviation (mm; 0 without points).
    pub max: f64,
    /// Largest |deviation| (mm).
    pub max_abs: f64,
    /// Mean signed deviation (mm).
    pub mean: f64,
    /// RMS deviation (mm).
    pub rms: f64,
    /// Tolerance used for the colour map and the in-tolerance share (mm).
    pub tolerance: f64,
    /// Share of points within ±tolerance (0..1).
    pub within: f64,
    /// Ten histogram bins of signed deviation over [−3·tol, 3·tol] (outside counts in the end bins).
    pub histogram: [usize; 10],
}

/// Statistics of signed deviations.
#[must_use]
pub fn stats(values: &[f64], tolerance: f64) -> DeviationStats {
    let tol = if tolerance.is_finite() && tolerance > 0.0 { tolerance } else { 0.1 };
    let vals: Vec<f64> = values.iter().copied().filter(|v| v.is_finite()).collect();
    let n = vals.len();
    let mut h = [0usize; 10];
    let (mut sum, mut sq, mut max_abs, mut within) = (0.0, 0.0, 0.0f64, 0usize);
    let (mut min, mut max) = (f64::INFINITY, f64::NEG_INFINITY);
    for &d in &vals {
        sum += d;
        sq += d * d;
        min = min.min(d);
        max = max.max(d);
        max_abs = max_abs.max(d.abs());
        if d.abs() <= tol {
            within += 1;
        }
        let bin = (((d / (3.0 * tol)) + 1.0) * 5.0).floor().clamp(0.0, 9.0) as usize;
        if let Some(b) = h.get_mut(bin) {
            *b += 1;
        }
    }
    let nf = n.max(1) as f64;
    let (min, max) = if n == 0 { (0.0, 0.0) } else { (min, max) };
    DeviationStats {
        count: n,
        not_facing: 0,
        min,
        max,
        max_abs,
        mean: sum / nf,
        rms: (sq / nf).sqrt(),
        tolerance: tol,
        within: within as f64 / nf,
        histogram: h,
    }
}

/// Signed deviation of each point from a primitive.
#[must_use]
pub fn against_primitive(points: &[Point3], prim: &Primitive) -> Vec<f64> {
    points.iter().map(|p| prim.distance(p)).collect()
}

/// Signed deviation of each point from a mesh surface (sign from the triangle normal; NaN when
/// farther than `max_distance`).
#[must_use]
pub fn against_mesh(points: &[Point3], target: &Bvh, max_distance: f64) -> Vec<f64> {
    surfacecraft_geom::par::map_slice(points, |p| match target.closest(p, max_distance) {
        Some(h) => {
            let sign = target.triangle_normal(h.triangle).map_or(1.0, |n| (p - h.point).dot(&n).signum());
            h.distance * if sign == 0.0 { 1.0 } else { sign }
        }
        None => f64::NAN,
    })
}

/// One scan point measured against a reference: its signed deviation and whether it faces it.
///
/// Sign convention: the deviation is positive when the scan lies outside the reference material,
/// judged by the reference surface normal (outward for bodies, spheres, cylinders, cones and tori;
/// for a plane, see [`oriented_plane`]). A point whose own normal points away from the reference
/// normal (dot < 0) is not facing: its nearest reference surface is then another wall of the part
/// (the far side of a thin plate), so it is left out of the statistics.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// Signed deviation (mm); NaN when the point is not measured.
    pub d: f64,
    /// False when the scan normal points away from the reference normal.
    pub facing: bool,
    /// Unit reference normal at the nearest reference point (zero when unknown, as for freeform
    /// surfaces). Areas are projected along it, see [`hotspots`].
    pub reference: [f64; 3],
}

impl Sample {
    /// A point that was not measured (outside the region, or farther than the reach).
    pub const UNMEASURED: Sample = Sample { d: f64::NAN, facing: false, reference: [0.0; 3] };

    /// Measured and facing: counted in the statistics.
    #[must_use]
    pub fn counted(&self) -> bool {
        self.d.is_finite() && self.facing
    }

    /// Measured, but not facing the reference: left out of the statistics.
    #[must_use]
    pub fn not_facing(&self) -> bool {
        self.d.is_finite() && !self.facing
    }
}

/// Whether a scan normal faces the reference normal. A missing or non-finite comparison counts as
/// facing: nothing in the data contradicts the measurement.
fn faces(scan: Option<&Vec3>, reference: &Vec3) -> bool {
    scan.is_none_or(|n| !(n.dot(reference) < 0.0))
}

/// Samples of points against a primitive. `normals[i]` is the scan normal of `points[i]` (missing
/// normals count as facing).
#[must_use]
pub fn samples_primitive(points: &[Point3], normals: &[Vec3], prim: &Primitive) -> Vec<Sample> {
    points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let n = prim.normal(p);
            Sample { d: prim.distance(p), facing: faces(normals.get(i), &n), reference: [n.x, n.y, n.z] }
        })
        .collect()
}

/// Samples of points against a mesh surface: the sign comes from the normal of the nearest
/// triangle, and points farther than `max_distance` are not measured. `normals[i]` is the scan
/// normal of `points[i]`.
#[must_use]
pub fn samples_mesh(points: &[Point3], normals: &[Vec3], target: &Bvh, max_distance: f64) -> Vec<Sample> {
    surfacecraft_geom::par::map(points.len(), |i| {
        let Some(p) = points.get(i) else { return Sample::UNMEASURED };
        let Some(h) = target.closest(p, max_distance) else { return Sample::UNMEASURED };
        let Some(n) = target.triangle_normal(h.triangle) else {
            // a degenerate reference triangle has no side: its distance is taken as outside
            return Sample { d: h.distance, facing: true, reference: [0.0; 3] };
        };
        let outside = (p - h.point).dot(&n) >= 0.0;
        Sample {
            d: if outside { h.distance } else { -h.distance },
            facing: faces(normals.get(i), &n),
            reference: [n.x, n.y, n.z],
        }
    })
}

/// The deviation values of the counted points, NaN for the others (for the colour map).
#[must_use]
pub fn counted_values(samples: &[Sample]) -> Vec<f64> {
    samples.iter().map(|s| if s.counted() { s.d } else { f64::NAN }).collect()
}

/// [`stats`] of the samples, with the points that do not face the reference counted apart.
#[must_use]
pub fn stats_of(samples: &[Sample], tolerance: f64) -> DeviationStats {
    let mut st = stats(&counted_values(samples), tolerance);
    st.not_facing = samples.iter().filter(|s| s.not_facing()).count();
    st
}

/// The primitive with a plane's normal turned to agree with `normals` (the scan normals of the
/// points it describes). A plane has no inherent side, so its material side comes from the scan:
/// the normal is flipped when the scan normals point against it on balance. Other primitives are
/// returned unchanged, since their normals point outward by construction.
#[must_use]
pub fn oriented_plane(prim: &Primitive, normals: &[Vec3]) -> Primitive {
    if let Primitive::Plane { point, normal } = prim {
        let n = Vec3::new(normal[0], normal[1], normal[2]);
        let vote: f64 =
            normals.iter().filter(|v| surfacecraft_geom::finite_vec(v) && v.norm() > 0.5).map(|v| v.dot(&n)).sum();
        if vote < 0.0 {
            return Primitive::Plane { point: *point, normal: [-normal[0], -normal[1], -normal[2]] };
        }
    }
    prim.clone()
}

/// Largest number of histogram bins [`report`] accepts.
pub const MAX_BINS: usize = 1000;
/// Largest number of tolerance bands [`report`] accepts.
pub const MAX_BANDS: usize = 64;

/// Percentiles of the signed deviation (mm), linearly interpolated between sorted values.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Percentiles {
    /// 1st percentile.
    pub p1: f64,
    /// 5th percentile.
    pub p5: f64,
    /// Median.
    pub p50: f64,
    /// 95th percentile.
    pub p95: f64,
    /// 99th percentile.
    pub p99: f64,
}

/// Histogram of the signed deviation: equal bins over `lo..hi`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Histogram {
    /// Lower edge of the first bin (mm).
    pub lo: f64,
    /// Upper edge of the last bin (mm).
    pub hi: f64,
    /// Counted points per bin. Values outside `lo..hi` fall in the end bins, so the counts sum to
    /// the number of counted points.
    pub counts: Vec<usize>,
}

/// The share of counted points whose |deviation| is within one tolerance.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Band {
    /// Tolerance (mm).
    pub tolerance: f64,
    /// Counted points with |deviation| ≤ tolerance.
    pub count: usize,
    /// Their share of the counted points (0..1).
    pub share: f64,
}

/// Statistics of the signed deviation of a set of samples.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Report {
    /// Counted points (measured and facing the reference).
    pub count: usize,
    /// Points left out because their normal points away from the reference normal.
    pub not_facing: usize,
    /// Points not measured: outside the region, or farther than the reach of the reference.
    pub unmeasured: usize,
    /// Smallest signed deviation (mm; 0 without counted points).
    pub min: f64,
    /// Largest signed deviation (mm; 0 without counted points).
    pub max: f64,
    /// Mean signed deviation (mm).
    pub mean: f64,
    /// RMS of the signed deviation (mm).
    pub rms: f64,
    /// Percentiles of the signed deviation.
    pub percentiles: Percentiles,
    /// Histogram of the signed deviation.
    pub histogram: Histogram,
    /// Share within each requested tolerance of |deviation|.
    pub bands: Vec<Band>,
}

/// Options of [`report`].
#[derive(Clone, Debug, PartialEq)]
pub struct ReportOptions {
    /// Histogram bins, 1 to [`MAX_BINS`].
    pub bins: usize,
    /// Histogram range (mm, finite, lower < upper). Default: ±p99 of |deviation|, or ±1 mm when
    /// every deviation is zero.
    pub range: Option<[f64; 2]>,
    /// Tolerances of the bands (mm, positive, at most [`MAX_BANDS`]).
    pub bands: Vec<f64>,
}

/// Linear interpolation of the `p`-th percentile (0..=100) of sorted values; 0 when empty.
fn percentile(sorted: &[f64], p: f64) -> f64 {
    let Some(last) = sorted.len().checked_sub(1) else { return 0.0 };
    let rank = (p / 100.0).clamp(0.0, 1.0) * last as f64;
    let lo = rank.floor();
    let hi = rank.ceil();
    let a = sorted.get(lo as usize).copied().unwrap_or(0.0);
    let b = sorted.get(hi as usize).copied().unwrap_or(0.0);
    a + (b - a) * (rank - lo)
}

/// Statistics of the signed deviation of the counted samples: extremes, mean, RMS, percentiles, a
/// histogram and the share within each tolerance band.
///
/// # Errors
/// Returns a message when the options are out of range (bins, range, bands).
pub fn report(samples: &[Sample], opts: &ReportOptions) -> Result<Report, String> {
    if opts.bins == 0 || opts.bins > MAX_BINS {
        return Err(format!("bins must be between 1 and {MAX_BINS}"));
    }
    if opts.bands.len() > MAX_BANDS {
        return Err(format!("at most {MAX_BANDS} bands"));
    }
    if let Some(t) = opts.bands.iter().find(|t| !(t.is_finite() && **t > 0.0)) {
        return Err(format!("band tolerance {t} must be positive"));
    }
    if let Some([lo, hi]) = opts.range
        && !(lo.is_finite() && hi.is_finite() && lo < hi)
    {
        return Err(format!("range {lo} to {hi} must be finite with lower < upper"));
    }
    let mut vals: Vec<f64> = samples.iter().filter(|s| s.counted()).map(|s| s.d).collect();
    vals.sort_by(f64::total_cmp);
    let count = vals.len();
    let mut abs: Vec<f64> = vals.iter().map(|d| d.abs()).collect();
    abs.sort_by(f64::total_cmp);
    let (min, max, mean, rms) = if count == 0 {
        (0.0, 0.0, 0.0, 0.0)
    } else {
        let n = count as f64;
        let sum: f64 = vals.iter().sum();
        let sq: f64 = vals.iter().map(|d| d * d).sum();
        let min = vals.first().copied().unwrap_or(0.0);
        let max = vals.last().copied().unwrap_or(0.0);
        (min, max, sum / n, (sq / n).sqrt())
    };
    let (lo, hi) = match opts.range {
        Some([lo, hi]) => (lo, hi),
        None => {
            let p99 = percentile(&abs, 99.0);
            let half = if p99 > 0.0 { p99 } else { abs.last().copied().unwrap_or(0.0) };
            let half = if half > 0.0 { half } else { 1.0 };
            (-half, half)
        }
    };
    let width = (hi - lo) / opts.bins as f64;
    let last = (opts.bins - 1) as f64;
    let mut counts = vec![0usize; opts.bins];
    for d in &vals {
        let k = ((d - lo) / width).floor();
        let idx = if k.is_nan() { 0 } else { k.clamp(0.0, last) as usize };
        if let Some(c) = counts.get_mut(idx) {
            *c += 1;
        }
    }
    let bands = opts
        .bands
        .iter()
        .map(|&tolerance| {
            let n = abs.partition_point(|&a| a <= tolerance);
            Band { tolerance, count: n, share: if count > 0 { n as f64 / count as f64 } else { 0.0 } }
        })
        .collect();
    Ok(Report {
        count,
        not_facing: samples.iter().filter(|s| s.not_facing()).count(),
        unmeasured: samples.iter().filter(|s| !s.d.is_finite()).count(),
        min,
        max,
        mean,
        rms,
        percentiles: Percentiles {
            p1: percentile(&vals, 1.0),
            p5: percentile(&vals, 5.0),
            p50: percentile(&vals, 50.0),
            p95: percentile(&vals, 95.0),
            p99: percentile(&vals, 99.0),
        },
        histogram: Histogram { lo, hi, counts },
        bands,
    })
}

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

/// Colour per value: green within ±tol, through yellow/orange to red above, through cyan to
/// blue below; grey for NaN (not measured).
#[must_use]
pub fn colorize(values: &[f64], tolerance: f64) -> Vec<[f32; 4]> {
    let tol = if tolerance.is_finite() && tolerance > 0.0 { tolerance } else { 0.1 };
    values
        .iter()
        .map(|&d| {
            if !d.is_finite() {
                return palette::UNMEASURED;
            }
            let t = (d / tol) as f32;
            if t.abs() <= 1.0 {
                palette::IN_TOLERANCE
            } else if t > 0.0 {
                let k = ((t - 1.0) / 2.0).clamp(0.0, 1.0);
                lerp(palette::ABOVE_NEAR, palette::ABOVE_FAR, k)
            } else {
                let k = ((-t - 1.0) / 2.0).clamp(0.0, 1.0);
                lerp(palette::BELOW_NEAR, palette::BELOW_FAR, k)
            }
        })
        .collect()
}

/// Noise estimate of a scan (σ, mm): median distance of vertices to a local quadric through
/// their neighbours, scaled to a standard deviation (design doc §10: warn when a tolerance is
/// below the scanner noise). Samples whose neighbourhood spans a sharp edge (neighbour normals
/// more than 30° off the local plane) are left out when enough smooth samples remain, so edges
/// and corners do not inflate the estimate.
#[must_use]
pub fn estimate_noise(mesh: &Mesh) -> f64 {
    let n = mesh.vertex_count();
    if n < 16 {
        return 0.0;
    }
    let tree = KdTree::build(&mesh.positions);
    let stride = n.div_ceil(4000).max(1);
    let normals = mesh.vertex_normals();
    let cos_sharp = 30f64.to_radians().cos();
    let mut d: Vec<f64> = Vec::new();
    let mut d_smooth: Vec<f64> = Vec::new();
    for p in mesh.positions.iter().step_by(stride) {
        let near = tree.knn(p, 17);
        // neighbours without the point itself (leave-one-out), local quadric height field
        let nb: Vec<Point3> =
            near.iter().filter(|(_, d2)| *d2 > 0.0).filter_map(|(i, _)| mesh.positions.get(*i).copied()).collect();
        let Some(f) = surfacecraft_fit::plane::fit_plane(&nb, None) else { continue };
        let z = f.normal;
        let sharp =
            near.iter().filter_map(|(i, _)| normals.get(*i)).any(|nn| nn.norm() > 0.5 && nn.dot(&z).abs() < cos_sharp);
        let x = surfacecraft_geom::any_perpendicular(&z);
        let y = z.cross(&x);
        let o = f.point.coords;
        let mut ata = nalgebra::Matrix6::<f64>::zeros();
        let mut atb = nalgebra::Vector6::<f64>::zeros();
        for q in &nb {
            let w = q.coords - o;
            let (u, v, h) = (w.dot(&x), w.dot(&y), w.dot(&z));
            let row = nalgebra::Vector6::new(u * u, u * v, v * v, u, v, 1.0);
            ata += row * row.transpose();
            atb += row * h;
        }
        let Some(c) = ata.lu().solve(&atb) else { continue };
        let w = p.coords - o;
        let (u, v, h) = (w.dot(&x), w.dot(&y), w.dot(&z));
        let pred = c[0] * u * u + c[1] * u * v + c[2] * v * v + c[3] * u + c[4] * v + c[5];
        d.push((h - pred).abs());
        if !sharp {
            d_smooth.push((h - pred).abs());
        }
    }
    if d_smooth.len() >= 50 && d_smooth.len() * 5 >= d.len() {
        d = d_smooth;
    }
    if d.is_empty() {
        return 0.0;
    }
    d.sort_by(f64::total_cmp);
    // median |x| of a normal variable is 0.6745 σ; the prediction's own error adds ≈ 10 %
    d.get(d.len() / 2).copied().unwrap_or(0.0) / 0.6745 / 1.1
}

/// Unused helper kept for symmetry with primitives: the outward normal of a primitive at `p`.
#[must_use]
pub fn primitive_normal(prim: &Primitive, p: &Point3) -> Vec3 {
    prim.normal(p)
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Rng, synth};

    #[test]
    fn stats_colors_and_noise() {
        let s = stats(&[0.0, 0.05, -0.05, 0.3, f64::NAN], 0.1);
        assert_eq!(s.count, 4);
        assert!((s.within - 0.75).abs() < 1e-12);
        assert!((s.max_abs - 0.3).abs() < 1e-12);
        assert!((s.min + 0.05).abs() < 1e-12 && (s.max - 0.3).abs() < 1e-12);
        assert_eq!(s.histogram.iter().sum::<usize>(), s.count);
        let c = colorize(&[0.0, 0.3, -0.3, f64::NAN], 0.1);
        assert_eq!(c[0], [0.2, 0.8, 0.3, 1.0]);
        assert!(c[1][0] > 0.9 && c[2][2] > 0.9);
        let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 50.0, 50.0, 60, 60);
        synth::add_noise(&mut m, 0.05, &mut Rng::new(1));
        let sigma = estimate_noise(&m);
        assert!((0.03..0.08).contains(&sigma), "noise estimate {sigma}");
        let flat = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 50.0, 50.0, 60, 60);
        let dev = against_mesh(&m.positions, &Bvh::build(&flat), 1.0);
        assert!((stats(&dev, 0.1).rms - 0.05).abs() < 0.01);
    }

    /// P1: edges and corners do not inflate the estimate (was 0.078 for σ = 0.05 on the bracket),
    /// nor does the density ramp of a scan.
    #[test]
    fn noise_estimate_is_robust_to_edges_and_density() {
        let mut m = synth::bracket();
        synth::add_noise(&mut m, 0.05, &mut Rng::new(4));
        let sigma = estimate_noise(&m);
        eprintln!("bracket σ=0.05 → {sigma:.4}");
        assert!((0.04..0.062).contains(&sigma), "bracket σ estimate {sigma}");
        let mut b = synth::cuboid(Point3::origin(), Vec3::new(30.0, 20.0, 10.0), 1);
        b.weld(1e-9);
        let d = synth::ScanDefects { noise: 0.03, holes: 0, spikes: 0.0, debris: 0, ..synth::ScanDefects::default() };
        let s = synth::scanify(&b, &d, &mut Rng::new(2));
        let sigma = estimate_noise(&s);
        eprintln!("scanned box σ=0.03 → {sigma:.4}");
        assert!((0.022..0.04).contains(&sigma), "scanned box σ estimate {sigma}");
    }
}

//! Freeform consolidation: the fragments an organic surface falls into become a few large
//! regions.
//!
//! Fit-and-grow finds a primitive wherever a small patch happens to be flat enough, so a scan of
//! a car panel or a casting ends in thousands of slivers, each "a plane" or "a cylinder" that
//! holds a few hundred triangles of millions, with the rest unassigned. Such a fragment is weak
//! evidence of a primitive: the surface is not one primitive for any length. A region is *weak*
//! when it holds fewer triangles than [`Settings::strong_tris`] (a share of the scan, never less
//! than a few seed patches) or when it is a curved primitive spanning less than a radius (a
//! gentle patch of a surface that a cone or torus fits by chance); a larger, better supported
//! primitive is kept whatever lies around it.
//!
//! The weak regions and the unassigned triangles form the *freeform mask*. Its connected pieces,
//! cut at sharp creases (the face normals of two neighbouring triangles further apart than
//! the crease angle), that hold at least [`Settings::min_cluster`] weak regions are organic
//! areas: each becomes one region flagged freeform, unless one primitive fits the whole area
//! within the tolerance (then the surface was a primitive that a defect fragmented, and the area
//! becomes that primitive's region). A piece with fewer weak regions (one small
//! boss, a fillet between two faces) is no cluster of fragments; its regions stay as found, and
//! so do unassigned triangles that touch no cluster. A strong primitive whose boundary runs
//! mostly into a freeform area without a crease is an island of it (a patch a cone happens to
//! fit) and joins the area; one that meets it at creases (a wall, a fillet) stays.
//!
//! Everything is a pure function of the regions, in triangle-index order, so the result does not
//! depend on the number of threads.
use super::Segment;
use super::fits::{fit_kind_tris, fit_tris};
use crate::primitive::{Primitive, PrimitiveKind};
use std::collections::{BTreeMap, VecDeque};
use surfacecraft_geom::{Mesh, Point3, Vec3};

/// Triangles looked at to measure a region's extent.
const SAMPLE_TRIANGLES: usize = 1500;
/// A curved primitive needs its region to span this many radii (or it is a patch of something
/// else).
const MIN_SUPPORT: f64 = 1.0;
/// A strong region is an island of an organic area when at least this share of its boundary
/// edges runs into it without a crease, and at most [`ISLAND_CREASE`] of them are creases (a
/// wall or a fillet next to a freeform top has the crease along the top and none into it).
const ISLAND_ORGANIC: f64 = 0.3;
const ISLAND_CREASE: f64 = 0.3;
/// Rounds of absorbing islands (an absorbed island can make the next one an island).
const ISLAND_PASSES: usize = 3;
/// Owner of a triangle in no region, and component of one outside the mask.
const NONE: u32 = u32::MAX;

/// What makes a region weak and a cluster of fragments an organic area.
#[derive(Clone, Copy, Debug)]
pub(super) struct Settings {
    /// Regions with fewer triangles are weak evidence of a primitive.
    pub strong_tris: usize,
    /// Fewest weak regions in one piece of the mask for it to be a freeform area.
    pub min_cluster: usize,
    /// Fewest triangles of a freeform area.
    pub min_triangles: usize,
    /// Distance tolerance of a primitive (mm): an area one primitive fits within it is no
    /// freeform area.
    pub tolerance: f64,
    /// Cosine of the crease angle: a mask edge turning further is not crossed.
    pub cos_crease: f64,
}

/// Corners of at most [`SAMPLE_TRIANGLES`] triangles of `tris`, evenly strided.
fn sample_corners(mesh: &Mesh, tris: &[u32]) -> Vec<Point3> {
    let stride = tris.len().div_ceil(SAMPLE_TRIANGLES).max(1);
    tris.iter().step_by(stride).filter_map(|&t| mesh.triangle(t as usize)).flatten().collect()
}

/// Diagonal of the box around `pts` after projecting them onto the plane normal to `axis` (the
/// chord of a region of a cylinder or cone), or the plain box diagonal without an axis.
fn extent(pts: &[Point3], axis: Option<Vec3>) -> f64 {
    let Some(axis) = axis.and_then(surfacecraft_geom::normalize) else {
        return surfacecraft_geom::Aabb::from_points(pts).diagonal();
    };
    let u = surfacecraft_geom::any_perpendicular(&axis);
    let v = axis.cross(&u);
    let flat: Vec<Point3> = pts.iter().map(|p| Point3::new(p.coords.dot(&u), p.coords.dot(&v), 0.0)).collect();
    surfacecraft_geom::Aabb::from_points(&flat).diagonal()
}

/// How far a curved region reaches around its primitive: the extent over which the surface
/// curves, in radii (`INFINITY` for a plane, which has no curvature to support). A region that
/// spans a fraction of a radius is a gentle patch of some surface that a curved primitive fits
/// by chance.
fn curvature_support(mesh: &Mesh, seg: &Segment) -> f64 {
    let pts = sample_corners(mesh, &seg.triangles);
    let v = |x: &[f64; 3]| Vec3::new(x[0], x[1], x[2]);
    let (chord, radius) = match &seg.primitive {
        Primitive::Plane { .. } => return f64::INFINITY,
        Primitive::Sphere { radius, .. } => (extent(&pts, None), radius.abs()),
        Primitive::Torus { minor_radius, .. } => (extent(&pts, None), minor_radius.abs()),
        Primitive::Cylinder { axis, radius, .. } => (extent(&pts, Some(v(axis))), radius.abs()),
        Primitive::Cone { apex, axis, .. } => {
            // the local radius: the mean distance of the region's points from the axis
            let (a, x) = (v(axis).normalize(), v(apex));
            let r: f64 = pts.iter().map(|p| (p.coords - x - a * (p.coords - x).dot(&a)).norm()).sum::<f64>();
            (extent(&pts, Some(a)), r / pts.len().max(1) as f64)
        }
    };
    if radius > 0.0 { chord / radius } else { 0.0 }
}

/// Is `seg` weak evidence of a primitive: few triangles, or a curved region that spans less
/// than [`MIN_SUPPORT`] radii.
fn is_weak(mesh: &Mesh, seg: &Segment, set: &Settings) -> bool {
    seg.triangles.len() < set.strong_tris || curvature_support(mesh, seg) < MIN_SUPPORT
}

/// Region index of every triangle ([`NONE`] when in none).
fn owners(n: usize, segs: &[Segment]) -> Vec<u32> {
    let mut owner = vec![NONE; n];
    for (i, s) in segs.iter().enumerate() {
        for &t in &s.triangles {
            if let Some(o) = owner.get_mut(t as usize) {
                *o = i as u32;
            }
        }
    }
    owner
}

/// Do the face normals of neighbouring triangles `a` and `b` stay within the crease angle?
fn is_smooth_edge(fnorm: &[Option<Vec3>], a: usize, b: usize, cos_crease: f64) -> bool {
    match (fnorm.get(a).copied().flatten(), fnorm.get(b).copied().flatten()) {
        (Some(x), Some(y)) => x.dot(&y) >= cos_crease,
        _ => true,
    }
}

/// Connected pieces of the triangles in `mask` across smooth edges: a label per triangle
/// ([`NONE`] outside the mask) and the number of pieces; labels follow the lowest triangle index.
fn components(nb: &[[u32; 3]], fnorm: &[Option<Vec3>], mask: &[bool], cos_crease: f64) -> (Vec<u32>, usize) {
    let mut label = vec![NONE; mask.len()];
    let mut count = 0u32;
    let mut queue: VecDeque<u32> = VecDeque::new();
    for start in 0..mask.len() {
        if !mask[start] || label[start] != NONE {
            continue;
        }
        label[start] = count;
        queue.push_back(start as u32);
        while let Some(t) = queue.pop_front() {
            for &o in nb.get(t as usize).into_iter().flatten() {
                let o = o as usize;
                if o < mask.len() && mask[o] && label[o] == NONE && is_smooth_edge(fnorm, t as usize, o, cos_crease) {
                    label[o] = count;
                    queue.push_back(o as u32);
                }
            }
        }
        count += 1;
    }
    (label, count as usize)
}

/// The piece of the mask each weak region belongs to: the one of its first triangle ([`NONE`]
/// for a strong region).
fn region_pieces(segs: &[Segment], weak: &[bool], label: &[u32]) -> Vec<u32> {
    segs.iter()
        .zip(weak)
        .map(|(s, &w)| {
            let first = s.triangles.first().and_then(|&t| label.get(t as usize)).copied();
            if w { first.unwrap_or(NONE) } else { NONE }
        })
        .collect()
}

/// Which pieces are freeform areas: enough weak regions and enough triangles.
fn qualifying(region_piece: &[u32], label: &[u32], pieces: usize, set: &Settings) -> Vec<bool> {
    let mut fragments = vec![0usize; pieces];
    for &p in region_piece {
        if let Some(c) = fragments.get_mut(p as usize) {
            *c += 1;
        }
    }
    let mut size = vec![0usize; pieces];
    for &l in label.iter().filter(|&&l| l != NONE) {
        if let Some(c) = size.get_mut(l as usize) {
            *c += 1;
        }
    }
    fragments.iter().zip(&size).map(|(&f, &n)| f >= set.min_cluster && n >= set.min_triangles).collect()
}

/// The stand-in primitive a freeform region carries: its least-squares plane (meaningless as a
/// model of the region, which is why the region is flagged).
fn stand_in(mesh: &Mesh, normals: &[Vec3], tris: &[u32]) -> (Primitive, f64) {
    match fit_kind_tris(mesh, normals, tris, PrimitiveKind::Plane) {
        Some(f) => (f.primitive, f.rms),
        None => (Primitive::Plane { point: [0.0; 3], normal: [0.0, 0.0, 1.0] }, 0.0),
    }
}

/// Where every triangle stands: its region, the piece of the mask it lies in, and which pieces
/// are organic.
struct Layout {
    /// Region of every triangle ([`NONE`] for none).
    owner: Vec<u32>,
    /// Piece of the mask of every free triangle.
    label: Vec<u32>,
    /// Piece a region belongs to: a weak region the one of its first triangle, a strong one
    /// [`NONE`] until it is absorbed.
    region_piece: Vec<u32>,
    organic: Vec<bool>,
}

impl Layout {
    /// The piece triangle `t` belongs to: that of its region, or its own when it is free.
    fn piece_of(&self, t: usize) -> u32 {
        match self.owner.get(t).copied() {
            Some(o) if o != NONE => self.region_piece.get(o as usize).copied().unwrap_or(NONE),
            _ => self.label.get(t).copied().unwrap_or(NONE),
        }
    }

    fn is_organic(&self, piece: u32) -> bool {
        self.organic.get(piece as usize).copied().unwrap_or(false)
    }
}

/// Strong regions whose boundary runs mostly into organic areas without a crease: an island of
/// the freeform area (a patch of it a cone or a plane happens to fit), with the piece that
/// borders it most.
fn islands(nb: &[[u32; 3]], face: &[Option<Vec3>], lay: &Layout, cos_crease: f64) -> Vec<(usize, u32)> {
    let regions = lay.region_piece.len();
    let mut edges = vec![0usize; regions];
    let mut creases = vec![0usize; regions];
    let mut votes: Vec<BTreeMap<u32, usize>> = vec![BTreeMap::new(); regions];
    for (t, &o) in lay.owner.iter().enumerate().filter(|(_, o)| **o != NONE) {
        let region = o as usize;
        if lay.region_piece.get(region) != Some(&NONE) {
            continue;
        }
        for &n in nb.get(t).into_iter().flatten() {
            if n != NONE && lay.owner.get(n as usize) == Some(&o) {
                continue;
            }
            edges[region] += 1;
            let piece = if n == NONE { NONE } else { lay.piece_of(n as usize) };
            if n == NONE || !is_smooth_edge(face, t, n as usize, cos_crease) {
                creases[region] += 1;
            } else if lay.is_organic(piece) {
                *votes[region].entry(piece).or_default() += 1;
            }
        }
    }
    (0..regions)
        .filter_map(|r| {
            let total: usize = votes[r].values().sum();
            let best = votes[r].iter().max_by_key(|(piece, n)| (**n, std::cmp::Reverse(**piece)))?;
            let (edge_count, crease_count) = (edges[r] as f64, creases[r] as f64);
            (total as f64 >= ISLAND_ORGANIC * edge_count && crease_count <= ISLAND_CREASE * edge_count)
                .then_some((r, *best.0))
        })
        .collect()
}

/// The region of one organic area: a primitive when a single one fits the whole area within the
/// tolerance (the surface was one primitive that noise or a defect fragmented), else freeform.
fn area_segment(mesh: &Mesh, normals: &[Vec3], tris: Vec<u32>, set: &Settings) -> Segment {
    match fit_tris(mesh, normals, &tris) {
        Some(f) if f.rms <= set.tolerance => {
            Segment { triangles: tris, primitive: f.primitive, rms: f.rms, freeform: false }
        }
        _ => {
            let (primitive, rms) = stand_in(mesh, normals, &tris);
            Segment { triangles: tris, primitive, rms, freeform: true }
        }
    }
}

/// Replaces the weak regions and unassigned triangles of every organic area by one freeform
/// region; everything else is returned as it was (retained regions first, in order, then the
/// freeform ones by their lowest triangle).
pub(super) fn consolidate(
    mesh: &Mesh,
    normals: &[Vec3],
    nb: &[[u32; 3]],
    segs: Vec<Segment>,
    set: &Settings,
) -> Vec<Segment> {
    let weak: Vec<bool> = segs.iter().map(|s| is_weak(mesh, s, set)).collect();
    if !weak.iter().any(|&w| w) {
        return segs;
    }
    let owner = owners(mesh.triangle_count(), &segs);
    let mask: Vec<bool> = owner.iter().map(|&o| o == NONE || weak.get(o as usize).copied().unwrap_or(false)).collect();
    // raw face normals: the smoothed ones blur a sharp edge into steps too small to cut at
    let face = surfacecraft_geom::par::map(mesh.triangle_count(), |t| mesh.face_normal(t));
    let (label, pieces) = components(nb, &face, &mask, set.cos_crease);
    let region_piece = region_pieces(&segs, &weak, &label);
    let organic = qualifying(&region_piece, &label, pieces, set);
    if !organic.iter().any(|&o| o) {
        return segs;
    }
    let mut lay = Layout { owner, label, region_piece, organic };
    for _ in 0..ISLAND_PASSES {
        let found = islands(nb, &face, &lay, set.cos_crease);
        if found.is_empty() {
            break;
        }
        for (region, piece) in found {
            lay.region_piece[region] = piece;
        }
    }
    let mut areas: Vec<Vec<u32>> = vec![Vec::new(); pieces];
    for t in 0..lay.owner.len() {
        let piece = lay.piece_of(t);
        if lay.is_organic(piece) {
            areas[piece as usize].push(t as u32);
        }
    }
    let mut out: Vec<Segment> =
        segs.into_iter().zip(&lay.region_piece).filter(|(_, piece)| !lay.is_organic(**piece)).map(|(s, _)| s).collect();
    out.extend(areas.into_iter().filter(|a| !a.is_empty()).map(|tris| area_segment(mesh, normals, tris, set)));
    out
}

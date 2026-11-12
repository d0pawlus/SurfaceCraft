//! Freeform consolidation: the fragments an organic surface falls into become a few large
//! regions.
//!
//! Fit-and-grow finds a primitive wherever a small patch happens to be flat enough, so a scan of
//! a car panel or a casting ends in thousands of slivers, each "a plane" or "a cylinder" that
//! holds a few hundred triangles of millions, with the rest unassigned. Such a fragment is weak
//! evidence of a primitive: the surface is not one primitive for any length.
//!
//! Every region has a *standing*. Being small (fewer triangles than [`Settings::strong_tris`], a
//! share of the scan never less than a few seed patches) or spanning less than a radius of a
//! curved primitive only triggers a closer look at its fit; the fit decides:
//! - the noise level of the scan is the lower quartile of the RMS of its regions (at most a
//!   quarter of the tolerance); a fit is *sound* when its RMS is within [`SOUND_FACTOR`] of it;
//! - a large region with a sound fit is [`Standing::Protected`]: it stays a primitive whatever
//!   lies around it (a mounting face in a sculpted surface), if it is a plane or spans a radius
//!   of its curved primitive, or if it is a major part of the scan (a shallow cylindrical panel).
//!   A curved primitive that spans less than a radius and is no major part of the scan is a
//!   [`Standing::Fragment`] however well it fits (a gentle patch of a freeform surface fits a
//!   torus within the noise);
//! - a large region whose fit is above the noise is [`Standing::Plain`]: a primitive of its own
//!   unless it is an island of a freeform area (below), and a [`Standing::Fragment`] when it also
//!   spans less than a radius of its curved primitive;
//! - a small region with a sound fit that is a plane or spans a radius of its curved primitive is
//!   [`Standing::Neutral`]: it may join a freeform area, but it is no evidence of one (five small
//!   genuine features joined by tangent fillets all fit at the noise level and span their radii,
//!   so they stay primitives);
//! - any other small region (a fit above the noise, or a patch of less than a radius) is a
//!   [`Standing::Fragment`].
//!
//! The fragments, the neutral regions and the unassigned triangles form the *freeform mask*. Its
//! connected pieces, cut at sharp creases (the face normals of two neighbouring triangles further
//! apart than the crease angle), that hold at least [`Settings::min_cluster`] fragments are
//! organic areas: each becomes one region flagged freeform, unless one primitive fits the whole
//! area within the tolerance (then a defect fragmented a primitive, and the area becomes that
//! primitive's region). Every triangle goes by its own piece, so a region that spans a crease is
//! split there. A piece with fewer fragments is no cluster; its regions stay as found, and so do
//! the unassigned triangles that touch no cluster. A plain primitive whose boundary runs into a
//! freeform area without a crease, and seldom along one, is an island of it (a patch a cone
//! happens to fit): it joins the area, which can join more pieces; one that meets the area at
//! creases (a wall, a fillet) stays.
//!
//! Everything is a pure function of the regions, in triangle-index order, so the result does not
//! depend on the number of threads.
use super::Segment;
use super::fits::{fit_kind_tris, fit_tris};
use crate::primitive::{Primitive, PrimitiveKind};
use std::collections::VecDeque;
use surfacecraft_geom::{Mesh, Point3, Vec3};

/// Triangles looked at to measure a region's extent.
const SAMPLE_TRIANGLES: usize = 1500;
/// A curved primitive needs its region to span this many radii (or it is a patch of something
/// else).
const MIN_SUPPORT: f64 = 1.0;
/// A small region's fit is sound when its RMS is within this factor of the noise level.
const SOUND_FACTOR: f64 = 1.2;
/// A large region's fit is sound when the robust spread of its residuals is within this factor of
/// the noise level, and its RMS within [`LARGE_RMS_FACTOR`].
const LARGE_SOUND_FACTOR: f64 = 1.05;
const LARGE_RMS_FACTOR: f64 = 2.0;
/// Scales a median absolute deviation to a standard deviation.
const MAD_TO_SIGMA: f64 = 1.4826;
/// A plain region is an island of an organic area when at least this share of its boundary
/// edges runs into it without a crease, and at most [`ISLAND_CREASE`] of them are creases (a
/// wall or a fillet next to a freeform top has the crease along the top and none into it).
const ISLAND_ORGANIC: f64 = 0.3;
const ISLAND_CREASE: f64 = 0.5;
/// Rounds of absorbing islands (an absorbed island can make the next one an island).
const ISLAND_PASSES: usize = 4;
/// Owner of a triangle in no region, and component of one outside the mask.
const NONE: u32 = u32::MAX;

/// What makes a region weak and a cluster of fragments an organic area.
#[derive(Clone, Copy, Debug)]
pub(super) struct Settings {
    /// Regions with fewer triangles are small.
    pub strong_tris: usize,
    /// Regions with at least this many triangles are a major part of the scan: a curved primitive
    /// that large is believed even when it spans less than a radius (a shallow panel).
    pub dominant_tris: usize,
    /// Fewest fragments in one piece of the mask for it to be a freeform area.
    pub min_cluster: usize,
    /// Fewest triangles of a freeform area.
    pub min_triangles: usize,
    /// Scanner noise sigma (mm): given or measured when `noise_known`, else the fallback.
    /// Never derived from the fits it judges.
    pub noise: f64,
    /// Is `noise` known? When not, no large region is demoted.
    pub noise_known: bool,
    /// Distance tolerance of a primitive (mm): an area one primitive fits within it is no
    /// freeform area.
    pub tolerance: f64,
    /// Cosine of the crease angle: a mask edge turning further is not crossed.
    pub cos_crease: f64,
}

/// How far a region is trusted to be a primitive (see the module documentation).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Standing {
    /// Large, sound fit: stays whatever its surroundings.
    Protected,
    /// Large, fit above the noise: a primitive unless an island of a freeform area.
    Plain,
    /// Small, sound fit: in the mask, but no evidence of an organic area.
    Neutral,
    /// Evidence of an organic area (and in the mask).
    Fragment,
}

impl Standing {
    fn in_mask(self) -> bool {
        matches!(self, Standing::Neutral | Standing::Fragment)
    }
}

/// The robust spread of the residuals of `seg` to its primitive (1.4826 x the median absolute
/// distance of a sample of its corners): a shoulder where a region grew into a tangent
/// neighbour (its tail) does not inflate it as it does the RMS.
fn robust_sigma(mesh: &Mesh, seg: &Segment) -> f64 {
    let mut d: Vec<f64> =
        sample_corners(mesh, &seg.triangles).iter().map(|p| seg.primitive.distance(p).abs()).collect();
    d.sort_by(f64::total_cmp);
    MAD_TO_SIGMA * d.get(d.len() / 2).copied().unwrap_or(f64::INFINITY)
}

/// The standing of `seg` (see the module documentation). Without a known noise level
/// ([`Settings::noise_known`]) no fit is judged by it: a large region is never demoted, a small
/// one is judged against the fallback.
fn standing_of(mesh: &Mesh, seg: &Segment, set: &Settings) -> Standing {
    let large = seg.triangles.len() >= set.strong_tris;
    if large && !set.noise_known {
        return Standing::Protected;
    }
    let sound = if large {
        // a large region is judged by the spread of most of its residuals (a shoulder where it
        // grew into a tangent neighbour is not misfit), and by an RMS cap
        robust_sigma(mesh, seg) <= LARGE_SOUND_FACTOR * set.noise && seg.rms <= LARGE_RMS_FACTOR * set.noise
    } else {
        seg.rms <= SOUND_FACTOR * set.noise
    };
    let supported = curvature_support(mesh, seg) >= MIN_SUPPORT;
    match (large, sound, supported) {
        (true, true, true) => Standing::Protected,
        (true, true, false) if seg.triangles.len() >= set.dominant_tris => Standing::Protected,
        (true, false, true) => Standing::Plain,
        (true, _, _) => Standing::Fragment,
        (false, true, true) => Standing::Neutral,
        (false, _, _) => Standing::Fragment,
    }
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
    /// Piece of the mask of every triangle in it ([`NONE`] for the others).
    label: Vec<u32>,
    organic: Vec<bool>,
}

impl Layout {
    fn is_organic(&self, piece: u32) -> bool {
        self.organic.get(piece as usize).copied().unwrap_or(false)
    }
}

/// Which pieces are freeform areas: enough fragments and enough triangles. A fragment counts
/// once in every piece it occupies (one that crosses a crease is a fragment on both sides).
fn qualifying(segs: &[Segment], standing: &[Standing], label: &[u32], pieces: usize, set: &Settings) -> Vec<bool> {
    let mut fragments = vec![0usize; pieces];
    for (s, _) in segs.iter().zip(standing).filter(|(_, st)| **st == Standing::Fragment) {
        let mut occupied: Vec<u32> =
            s.triangles.iter().filter_map(|&t| label.get(t as usize).copied()).filter(|&l| l != NONE).collect();
        occupied.sort_unstable();
        occupied.dedup();
        for piece in occupied {
            if let Some(c) = fragments.get_mut(piece as usize) {
                *c += 1;
            }
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

/// The layout of the mask for regions of the given `standing`.
fn layout(
    mesh: &Mesh,
    nb: &[[u32; 3]],
    face: &[Option<Vec3>],
    segs: &[Segment],
    standing: &[Standing],
    set: &Settings,
) -> Layout {
    let owner = owners(mesh.triangle_count(), segs);
    let mask: Vec<bool> =
        owner.iter().map(|&o| o == NONE || standing.get(o as usize).is_some_and(|st| st.in_mask())).collect();
    let (label, pieces) = components(nb, face, &mask, set.cos_crease);
    let organic = qualifying(segs, standing, &label, pieces, set);
    Layout { owner, label, organic }
}

/// Plain regions whose boundary runs mostly into organic areas without a crease: islands of a
/// freeform area (a patch of it a cone or a plane happens to fit).
fn islands(nb: &[[u32; 3]], face: &[Option<Vec3>], lay: &Layout, standing: &[Standing], cos_crease: f64) -> Vec<usize> {
    let regions = standing.len();
    let mut edges = vec![0usize; regions];
    let mut creases = vec![0usize; regions];
    let mut organic_edges = vec![0usize; regions];
    for (t, &o) in lay.owner.iter().enumerate().filter(|(_, o)| **o != NONE) {
        let region = o as usize;
        if standing.get(region) != Some(&Standing::Plain) {
            continue;
        }
        for &n in nb.get(t).into_iter().flatten() {
            if n != NONE && lay.owner.get(n as usize) == Some(&o) {
                continue;
            }
            edges[region] += 1;
            if n == NONE || !is_smooth_edge(face, t, n as usize, cos_crease) {
                creases[region] += 1;
            } else if lay.label.get(n as usize).is_some_and(|&l| lay.is_organic(l)) {
                organic_edges[region] += 1;
            }
        }
    }
    (0..regions)
        .filter(|&r| {
            let total = edges[r] as f64;
            organic_edges[r] as f64 >= ISLAND_ORGANIC * total
                && creases[r] as f64 <= ISLAND_CREASE * total
                && total > 0.0
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

/// Replaces the fragments of every organic area, the regions among them and the unassigned
/// triangles between them by one region per area; everything else is returned as it was
/// (retained regions first, in order, then the new ones by their lowest triangle).
pub(super) fn consolidate(
    mesh: &Mesh,
    normals: &[Vec3],
    nb: &[[u32; 3]],
    segs: Vec<Segment>,
    set: &Settings,
) -> Vec<Segment> {
    let mut standing: Vec<Standing> = segs.iter().map(|s| standing_of(mesh, s, set)).collect();
    if !standing.contains(&Standing::Fragment) {
        return segs;
    }
    // raw face normals: the smoothed ones blur a sharp edge into steps too small to cut at
    let face = surfacecraft_geom::par::map(mesh.triangle_count(), |t| mesh.face_normal(t));
    let mut lay = layout(mesh, nb, &face, &segs, &standing, set);
    if !lay.organic.iter().any(|&o| o) {
        return segs;
    }
    // an absorbed island is a fragment: it can join pieces it separated (one that was alone on
    // its side of it) and make the next island
    for _ in 0..ISLAND_PASSES {
        let found = islands(nb, &face, &lay, &standing, set.cos_crease);
        if found.is_empty() {
            break;
        }
        for region in found {
            standing[region] = Standing::Fragment;
        }
        lay = layout(mesh, nb, &face, &segs, &standing, set);
    }
    assemble(mesh, normals, nb, segs, &standing, &lay, set)
}

/// Connected pieces of `tris` (sorted) across shared edges, ordered by their lowest triangle.
fn connected_pieces(nb: &[[u32; 3]], tris: &[u32]) -> Vec<Vec<u32>> {
    let mut seen = vec![false; tris.len()];
    let mut pieces = Vec::new();
    for start in 0..tris.len() {
        if seen[start] {
            continue;
        }
        seen[start] = true;
        let mut piece = Vec::new();
        let mut stack = vec![start];
        while let Some(i) = stack.pop() {
            piece.push(tris[i]);
            for &o in nb.get(tris[i] as usize).into_iter().flatten() {
                if let Ok(j) = tris.binary_search(&o)
                    && !seen[j]
                {
                    seen[j] = true;
                    stack.push(j);
                }
            }
        }
        piece.sort_unstable();
        pieces.push(piece);
    }
    pieces
}

/// A piece cut from a region: its own fit, or `None` (released) when it is under `min_triangles`
/// or no primitive fits it within the tolerance any more.
fn refit_piece(mesh: &Mesh, normals: &[Vec3], tris: &[u32], set: &Settings) -> Option<Segment> {
    if tris.len() < set.min_triangles {
        return None;
    }
    let f = fit_tris(mesh, normals, tris).filter(|f| f.rms <= set.tolerance)?;
    Some(Segment { triangles: tris.to_vec(), primitive: f.primitive, rms: f.rms, freeform: false })
}

/// The regions that remain and the region of every organic area, each triangle going by its own
/// piece. A region that lost triangles to an area is cut into its connected pieces (by lowest
/// triangle); each is fitted again, and a piece under `min_triangles`, or that no primitive fits
/// within the tolerance, is released (left unassigned). Order: untouched regions, the pieces of
/// the cut ones, then the areas.
fn assemble(
    mesh: &Mesh,
    normals: &[Vec3],
    nb: &[[u32; 3]],
    segs: Vec<Segment>,
    standing: &[Standing],
    lay: &Layout,
    set: &Settings,
) -> Vec<Segment> {
    let mut areas: Vec<Vec<u32>> = vec![Vec::new(); lay.organic.len()];
    for (t, &piece) in lay.label.iter().enumerate().filter(|(_, p)| **p != NONE) {
        if lay.is_organic(piece) {
            areas[piece as usize].push(t as u32);
        }
    }
    let mut out: Vec<Segment> = Vec::new();
    let mut cut: Vec<Vec<u32>> = Vec::new();
    for (s, st) in segs.into_iter().zip(standing) {
        let kept: Vec<u32> = if st.in_mask() {
            s.triangles
                .iter()
                .copied()
                .filter(|&t| !lay.label.get(t as usize).is_some_and(|&l| lay.is_organic(l)))
                .collect()
        } else {
            out.push(s);
            continue;
        };
        if kept.len() == s.triangles.len() {
            out.push(s);
        } else {
            cut.extend(connected_pieces(nb, &kept));
        }
    }
    let refitted = surfacecraft_geom::par::map_slice(&cut, |tris| refit_piece(mesh, normals, tris, set));
    out.extend(refitted.into_iter().flatten());
    out.extend(areas.into_iter().filter(|a| !a.is_empty()).map(|tris| area_segment(mesh, normals, tris, set)));
    out
}

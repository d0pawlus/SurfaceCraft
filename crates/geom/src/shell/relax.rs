//! Where the offset would fold, the offset distance is reduced locally.
//!
//! Offsetting a surface by `d` along its normals folds it wherever the radius of curvature on the
//! side of the offset is smaller than `d` (the offset surface of a sphere of radius `r` has
//! radius `r − d` and turns inside out for `d > r`; Farouki, "The approximation of non-degenerate
//! offset surfaces", CAGD 3(1), 1986). A fold shows in the triangles first: an offset triangle
//! shrinks to a fraction of the area of the original or turns its normal away from it. Every vertex
//! of such a triangle has its offset multiplied by [`SHRINK`], and the triangles around it are
//! tested again, until none is left. The reduction is tapered into the neighbourhood (the offset
//! distance changes by at most [`MAX_OFFSET_SLOPE`] times the distance between vertices), so that
//! the relaxed vertices do not fold their unrelaxed neighbours in turn. Only triangles around the vertices changed are re-tested, so the
//! cost stays near one pass over the mesh.
//!
//! Before the first round the offset of every vertex is bounded by the local feature size of the sheet
//! ([`feature`]: the distance along the offset direction to another part of the sheet), so the
//! offset sheet starts on its own side of the scanned one. After the rounds the exact crossing test
//! of [`crossing`] runs on the shell mesh as it is returned (both layers and every row of the strip
//! along the rim), around the reduced vertices and along the whole strip.
use super::crossing::{self, Found, Surface};
use super::feature;
use super::strip::{self, Rim};
use super::topology::Csr;
use crate::{Bvh, Mesh, Point3, Vec3, normalize, par};

/// A triangle whose offset copy keeps less than this share of its area is folded.
pub(super) const MIN_AREA_RATIO: f64 = 0.15;
/// A triangle whose offset copy has a normal with a smaller cosine to the original's is folded.
pub(super) const MIN_NORMAL_COS: f64 = 0.3;
/// Triangles thinner than this (twice the area over the longest edge squared; 0.87 for an
/// equilateral one) are left out of the test: their shape changes with any offset.
const MIN_QUALITY: f64 = 0.05;
/// Factor a flagged vertex's offset is multiplied by in every round.
const SHRINK: f64 = 0.8;
/// Largest change of the offset distance per distance between neighbouring vertices.
const MAX_OFFSET_SLOPE: f64 = 0.5;
/// The offset of a vertex is never reduced below this share of the thickness: a fold that cannot be
/// resolved above it is refused (below it the two sheets would touch).
pub(super) const MIN_SCALE: f64 = 0.1;
/// Rounds of the exact crossing test around the reduced vertices at most.
const MAX_CROSSING_ROUNDS: usize = 40;
/// Rounds at most; `SHRINK^ROUNDS` is the smallest share of the offset a vertex can keep.
const MAX_ROUNDS: usize = 60;

/// What the test reads.
#[derive(Clone, Copy)]
pub(super) struct Field<'a> {
    pub positions: &'a [Point3],
    pub normals: &'a [Vec3],
    pub triangles: &'a [[u32; 3]],
    /// Offsets (mm along the normals) of the layers that move; `[]` entries of zero are left out.
    pub offsets: &'a [f64],
    /// Offsets of both layers `[a, b]` (one of them may be 0).
    pub layers: [f64; 2],
    /// Triangles around every vertex.
    pub around: &'a Csr,
    /// Neighbours of every vertex.
    pub adjacent: &'a Csr,
    /// The thickness (mm): the offsets are `scale` times it.
    pub thickness: f64,
    /// Median edge length of the mesh (mm).
    pub edge: f64,
    /// Diagonal of the bounding box of the mesh (mm).
    pub size: f64,
    /// Boundary edges (the strip along them joins the two layers).
    pub boundary: &'a [(u32, u32)],
    /// Ids of the vertices of the strip along the boundary.
    pub rim: &'a Rim,
}

impl Field<'_> {
    fn corner(&self, v: u32, scale: &[f64], offset: f64) -> Option<Point3> {
        let p = self.positions.get(v as usize)?;
        let n = self.normals.get(v as usize)?;
        Some(p + n * (offset * scale.get(v as usize)?))
    }

    /// Does the offset copy of triangle `ti` fold?
    fn folded(&self, ti: u32, scale: &[f64]) -> bool {
        let Some(t) = self.triangles.get(ti as usize) else { return false };
        let (Some(a), Some(b), Some(c)) =
            (self.positions.get(t[0] as usize), self.positions.get(t[1] as usize), self.positions.get(t[2] as usize))
        else {
            return false;
        };
        let reference = (b - a).cross(&(c - a));
        let area = reference.norm();
        let longest2 =
            [(b - a).norm_squared(), (c - b).norm_squared(), (a - c).norm_squared()].into_iter().fold(0.0, f64::max);
        if !(area > MIN_QUALITY * longest2) {
            return false;
        }
        self.offsets.iter().any(|&o| {
            let moved = (self.corner(t[0], scale, o), self.corner(t[1], scale, o), self.corner(t[2], scale, o));
            let (Some(p), Some(q), Some(r)) = moved else { return true };
            let now = (q - p).cross(&(r - p));
            let size = now.norm();
            !(size.is_finite() && size >= MIN_AREA_RATIO * area && now.dot(&reference) >= MIN_NORMAL_COS * size * area)
        })
    }
}

/// The vertices of the triangles around `dirty` that fold, ascending.
fn flagged(field: &Field, scale: &[f64], dirty: &[u32]) -> Vec<u32> {
    let mut tris: Vec<u32> = dirty.iter().flat_map(|&v| field.around.row(v as usize).iter().copied()).collect();
    tris.sort_unstable();
    tris.dedup();
    let bad: Vec<bool> = par::map(tris.len(), |i| tris.get(i).is_some_and(|&ti| field.folded(ti, scale)));
    let mut out: Vec<u32> = Vec::new();
    for (&ti, _) in tris.iter().zip(&bad).filter(|&(_, &b)| b) {
        if let Some(t) = field.triangles.get(ti as usize) {
            out.extend_from_slice(t);
        }
    }
    out.sort_unstable();
    out.dedup();
    out
}

/// Lowers the scale of the neighbours of `changed` so that it differs by at most the slope bound
/// per edge; returns every vertex whose scale changed (`changed` included), ascending.
fn taper(field: &Field, scale: &mut [f64], changed: &[u32]) -> Vec<u32> {
    let per_mm = MAX_OFFSET_SLOPE / field.thickness;
    let mut queue: std::collections::VecDeque<u32> = changed.iter().copied().collect();
    let mut all: Vec<u32> = changed.to_vec();
    while let Some(i) = queue.pop_front() {
        let Some(&si) = scale.get(i as usize) else { continue };
        let Some(pi) = field.positions.get(i as usize) else { continue };
        for &j in field.adjacent.row(i as usize) {
            let Some(pj) = field.positions.get(j as usize) else { continue };
            let bound = si + per_mm * (pj - pi).norm();
            if let Some(sj) = scale.get_mut(j as usize)
                && bound < *sj - 1e-12
            {
                *sj = bound;
                queue.push_back(j);
                all.push(j);
            }
        }
    }
    all.sort_unstable();
    all.dedup();
    all
}

/// The outcome of the relaxation.
pub(super) struct Relaxed {
    /// The share of the offset every vertex keeps (1 where nothing folds and nothing was tapered).
    pub scale: Vec<f64>,
    /// Vertices of triangles that folded at the full offset or in a round of the per-triangle test, and
    /// that were reduced (the others were only tapered). Crossings are counted apart (`unresolved`).
    pub folded: usize,
    /// Vertices of the scan that, after all rounds, lie on a triangle of the shell mesh that crosses
    /// another one (the exact test of [`crossing`] on the final state: both layers, every row of the strip;
    /// both triangles of a pair count) or on a folded triangle at the floor [`MIN_SCALE`]. Sharp
    /// creases on the side of the offset: no wall that thick fits there. Counted once.
    pub unresolved: usize,
    /// The normals the offset follows: those given, smoothed around vertices that stayed unresolved.
    pub normals: Vec<Vec3>,
    /// Vertices whose offset the local feature size of the scan bounded before any round.
    pub limited: usize,
    /// The most triangles an exhaustive pass found crossing a part of the scan far from them along the surface.
    pub far: usize,
}

/// What the rounds change.
struct State {
    scale: Vec<f64>,
    ever: Vec<bool>,
    frozen: Vec<bool>,
    uncertain: Vec<bool>,
    /// The most far crossings (triangles) any pass found.
    far: usize,
}

impl State {
    /// Splits `bad` into the vertices that can still be reduced (returned) and those already at the
    /// floor (frozen), and records that they all folded.
    fn partition(&mut self, bad: Vec<u32>, fold: bool) -> Vec<u32> {
        let mut movable = Vec::new();
        for v in bad {
            let at_floor = self.scale.get(v as usize).is_some_and(|&s| s <= MIN_SCALE * 1.000_001);
            if let (Some(f), Some(e)) = (self.frozen.get_mut(v as usize), self.ever.get_mut(v as usize)) {
                *e |= fold;
                if at_floor {
                    *f = true;
                } else {
                    movable.push(v);
                }
            }
        }
        movable
    }

    /// Vertices on a triangle that overlapped too many others to be tested: they are stuck (shrinking
    /// them cannot tell) and count as unresolved.
    fn mark_uncertain(&mut self, vertices: &[u32]) {
        for &v in vertices {
            if let Some(u) = self.uncertain.get_mut(v as usize) {
                *u = true;
            }
        }
        self.freeze(vertices);
    }

    /// Marks `vertices` as stuck: they cannot be relaxed further.
    fn freeze(&mut self, vertices: &[u32]) {
        for &v in vertices {
            if let Some(f) = self.frozen.get_mut(v as usize) {
                *f = true;
            }
        }
    }

    /// Reduces `movable` and tapers the reduction; returns the vertices changed.
    fn shrink(&mut self, field: &Field, movable: &[u32]) -> Vec<u32> {
        for &v in movable {
            if let Some(s) = self.scale.get_mut(v as usize) {
                *s = (*s * SHRINK).max(MIN_SCALE);
            }
        }
        taper(field, &mut self.scale, movable)
    }
}

/// Rounds of the per-triangle test from the vertices `dirty`; the number of vertices that still fold
/// when [`MAX_ROUNDS`] could not resolve them is the error.
fn fold_rounds(field: &Field, st: &mut State, mut dirty: Vec<u32>) -> Result<(), usize> {
    for _ in 0..MAX_ROUNDS {
        let movable = {
            let bad = flagged(field, &st.scale, &dirty);
            st.partition(bad, true)
        };
        if movable.is_empty() {
            return Ok(());
        }
        dirty = st.shrink(field, &movable);
    }
    let left = flagged(field, &st.scale, &dirty)
        .into_iter()
        .filter(|&v| !st.frozen.get(v as usize).copied().unwrap_or(true))
        .count();
    if left == 0 { Ok(()) } else { Err(left) }
}

/// The shell mesh of the current offsets, and the offsets at the last test.
struct ShellTest {
    mesh: Mesh,
    /// The scales and normals when the mesh was last tested (`None` before the first test).
    tested: Option<(Vec<f64>, Vec<Vec3>)>,
}

impl ShellTest {
    fn new(field: &Field) -> Self {
        let triangles = strip::shell_triangles(field.triangles, field.rim, field.boundary);
        ShellTest { mesh: Mesh { positions: Vec::new(), triangles }, tested: None }
    }

    /// The triangles of the shell mesh to test: all of them the first time; afterwards those that touch
    /// a vertex whose offset (scale or normal) changed since, or one of `also` (both layers; the rows of
    /// the strip along a rim edge with such an end). A pair of triangles that did not move and does not
    /// touch `also` has been tested already: if it crossed, its vertices were reduced (and so changed)
    /// or are among `also`.
    fn ids(&self, field: &Field, scale: &[f64], also: &[u32]) -> Vec<u32> {
        let total = self.mesh.triangles.len();
        let Some((scales, normals)) = &self.tested else { return (0..total as u32).collect() };
        let mut marked = vec![false; field.positions.len()];
        for (v, slot) in marked.iter_mut().enumerate() {
            *slot = scale.get(v) != scales.get(v) || field.normals.get(v) != normals.get(v);
        }
        for &v in also {
            if let Some(slot) = marked.get_mut(v as usize) {
                *slot = true;
            }
        }
        let is_marked = |v: u32| marked.get(v as usize).copied().unwrap_or(false);
        let mut tris: Vec<u32> = (0..field.positions.len() as u32)
            .filter(|&v| is_marked(v))
            .flat_map(|v| field.around.row(v as usize).iter().copied())
            .collect();
        tris.sort_unstable();
        tris.dedup();
        let layer = field.triangles.len() as u32;
        let mut ids: Vec<u32> = tris.iter().flat_map(|&i| [i, i + layer]).collect();
        let per_edge = 2 * field.rim.rows;
        for (e, &(u, v)) in field.boundary.iter().enumerate() {
            if is_marked(u) || is_marked(v) {
                let first = 2 * field.triangles.len() + e * per_edge;
                ids.extend((first..(first + per_edge).min(total)).map(|i| i as u32));
            }
        }
        ids
    }

    /// The vertices of the scan on a local crossing of the shell mesh among the triangles [`Self::ids`]
    /// picks (with `also` always included).
    fn crossing_vertices(&mut self, field: &Field, scale: &[f64], surface: &Surface, also: &[u32]) -> Found {
        let (a, b) = (field.layers[0], field.layers[1]);
        let outer = strip::layer(field.positions, field.normals, scale, a);
        let inner = strip::layer(field.positions, field.normals, scale, b);
        self.mesh.positions = strip::shell_positions(&outer, &inner, field.rim);
        let ids = self.ids(field, scale, also);
        self.tested = Some((scale.to_vec(), field.normals.to_vec()));
        if ids.is_empty() {
            return Found::default();
        }
        let bvh = Bvh::build(&self.mesh);
        crossing::local_vertices(&self.mesh, &bvh, surface, &ids)
    }
}

/// Bounds the offset of every vertex by the local feature size of the scan and tapers the bound into
/// the neighbours; returns the vertices the bound itself reduced.
fn limit_by_feature_size(field: &Field, surface: &Surface, st: &mut State) -> usize {
    let sheet = Bvh::build(&Mesh { positions: field.positions.to_vec(), triangles: field.triangles.to_vec() });
    let caps = feature::caps(field, &sheet, surface, MIN_SCALE);
    let limited: Vec<u32> =
        (0..caps.len() as u32).filter(|&v| caps.get(v as usize).is_some_and(|&c| c < 1.0)).collect();
    st.scale = caps;
    taper(field, &mut st.scale, &limited);
    limited.len()
}

/// Share of the vertices on triangles that overlap too many others to be tested, above which the
/// relaxation stops and reports them (the call is refused above 0.1 % anyway).
const UNCERTAIN_SHARE: f64 = 0.001;
/// Rounds of normal smoothing around the vertices that stay on a crossing at the floor, at most.
const MAX_SMOOTHING_PASSES: usize = 8;

#[cfg(test)]
thread_local! {
    /// Lowers the number of crossing rounds in a test (the break after the last round is then reached).
    pub(super) static ROUND_LIMIT: std::cell::Cell<Option<usize>> = const { std::cell::Cell::new(None) };
}

/// Crossing rounds at most.
fn round_limit() -> usize {
    #[cfg(test)]
    if let Some(n) = ROUND_LIMIT.with(std::cell::Cell::get) {
        return n;
    }
    MAX_CROSSING_ROUNDS
}

/// Crossing rounds: the vertices of local crossings above the floor are reduced (and the folds this
/// causes relaxed) until none is left or [`MAX_CROSSING_ROUNDS`] rounds are done.
fn crossing_rounds(field: &Field, st: &mut State, surface: &Surface, shell: &mut ShellTest) -> Result<(), usize> {
    let limit = round_limit();
    for round in 0..=limit {
        let found = shell.crossing_vertices(field, &st.scale, surface, &[]);
        st.mark_uncertain(&found.uncertain);
        st.far = st.far.max(found.far);
        let movable = st.partition(found.crossing, false);
        if movable.is_empty() {
            break;
        }
        if round == limit {
            // out of rounds: these vertices still cross, and the final test must see them again
            st.freeze(&movable);
            break;
        }
        let changed = st.shrink(field, &movable);
        fold_rounds(field, st, changed)?;
    }
    Ok(())
}

/// Replaces the normals of `region` by the mean of their own and their neighbours' normals: where the
/// offset of neighbouring vertices diverges at the floor (a cluster of tiny triangles whose normals
/// disagree) the offset layer then moves as a whole, and its triangles stop cutting the sheet.
fn smoothed_normals(field: &Field, normals: &[Vec3], region: &[u32]) -> Vec<Vec3> {
    let mut out = normals.to_vec();
    for &v in region {
        let sum: Vec3 = std::iter::once(v)
            .chain(field.adjacent.row(v as usize).iter().copied().take(MAX_SMOOTHING_ROW))
            .filter_map(|w| normals.get(w as usize))
            .sum();
        if let (Some(slot), Some(n)) = (out.get_mut(v as usize), normalize(sum)) {
            *slot = n;
        }
    }
    out
}

/// A row of the adjacency is read up to this many entries when normals are smoothed.
const MAX_SMOOTHING_ROW: usize = 64;

/// Relaxes the offset, or returns the number of vertices that still fold when [`MAX_ROUNDS`] rounds
/// could not resolve the others. The offset is first bounded by the local feature size
/// ([`limit_by_feature_size`]). After the per-triangle rounds, the shell mesh is tested exactly for
/// triangles that cross; their vertices are reduced, for at most [`MAX_CROSSING_ROUNDS`] rounds. The
/// vertices that cannot be reduced further (at the floor) and still cross get their normals smoothed
/// ([`smoothed_normals`]) for at most [`MAX_SMOOTHING_PASSES`] passes; what still crosses then is
/// `unresolved`.
pub(super) fn relax(field: &Field) -> Result<Relaxed, usize> {
    let n = field.positions.len();
    let mut st =
        State { scale: vec![1.0; n], ever: vec![false; n], frozen: vec![false; n], uncertain: vec![false; n], far: 0 };
    let surface = Surface::new(field.positions, field.adjacent, field.rim, field.thickness, field.edge, field.size);
    // what would fold at the full offset is counted as folded, whatever the bound does to it
    for v in flagged(field, &st.scale, &(0..n as u32).collect::<Vec<_>>()) {
        if let Some(e) = st.ever.get_mut(v as usize) {
            *e = true;
        }
    }
    let limited = limit_by_feature_size(field, &surface, &mut st);
    fold_rounds(field, &mut st, (0..n as u32).collect())?;
    let mut shell = ShellTest::new(field);
    let mut normals = field.normals.to_vec();
    crossing_rounds(field, &mut st, &surface, &mut shell)?;
    // a pile of triangles that cannot be tested (coincident sheets): nothing below can change that
    let uncertain = st.uncertain.iter().filter(|&&u| u).count();
    let passes = if uncertain as f64 > UNCERTAIN_SHARE * n as f64 { 0 } else { MAX_SMOOTHING_PASSES };
    for _ in 0..passes {
        let now = Field { normals: &normals, ..*field };
        let stuck = stuck_vertices(&now, &st, &surface, &mut shell);
        if stuck.is_empty() {
            break;
        }
        let mut region = stuck.clone();
        region.extend(stuck.iter().flat_map(|&v| now.adjacent.row(v as usize).iter().copied().take(MAX_SMOOTHING_ROW)));
        region.sort_unstable();
        region.dedup();
        let next = smoothed_normals(&now, &normals, &region);
        normals = next;
        let now = Field { normals: &normals, ..*field };
        for &v in &region {
            if let Some(f) = st.frozen.get_mut(v as usize) {
                *f = false;
            }
        }
        fold_rounds(&now, &mut st, region)?;
        crossing_rounds(&now, &mut st, &surface, &mut shell)?;
    }
    let now = Field { normals: &normals, ..*field };
    let unresolved = if passes == 0 { uncertain } else { stuck_vertices(&now, &st, &surface, &mut shell).len() };
    Ok(Relaxed {
        folded: st.ever.iter().zip(&st.scale).filter(|&(&e, &s)| e && s < 1.0).count(),
        unresolved,
        limited,
        far: st.far,
        scale: st.scale,
        normals,
    })
}

/// The vertices that are on a fold at the floor of the offset or on a crossing of the shell mesh in
/// its current state, ascending, each once. Exact: what changed since the last test is tested again,
/// and so is everything around the vertices that were stuck at the floor before (their crossing may
/// have been resolved by the movement of the other triangle).
fn stuck_vertices(field: &Field, st: &State, surface: &Surface, shell: &mut ShellTest) -> Vec<u32> {
    let frozen: Vec<u32> = (0..st.frozen.len() as u32).filter(|&v| st.frozen.get(v as usize) == Some(&true)).collect();
    let found = shell.crossing_vertices(field, &st.scale, surface, &frozen);
    let mut stuck = found.crossing;
    stuck.extend(found.uncertain);
    stuck.extend((0..st.uncertain.len() as u32).filter(|&v| st.uncertain.get(v as usize) == Some(&true)));
    let all: Vec<u32> = (0..field.positions.len() as u32).collect();
    let at_floor = |v: &u32| st.scale.get(*v as usize).is_some_and(|&s| s <= MIN_SCALE * 1.000_001);
    stuck.extend(flagged(field, &st.scale, &all).into_iter().filter(at_floor));
    stuck.sort_unstable();
    stuck.dedup();
    stuck
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fold_that_survives_the_floor_and_the_smoothing_is_counted_as_unresolved() {
        // Arrange: a triangle whose normals point up, up and down, without neighbours to smooth them with:
        // with an offset this large it stands on its edge at every scale down to the floor
        let positions = [Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];
        let normals = [Vec3::z(), Vec3::z(), -Vec3::z()];
        let triangles = [[0u32, 1, 2]];
        let around = Csr::from_pairs(3, &[(0, 0), (1, 0), (2, 0)]);
        let adjacent = Csr::from_pairs(3, &[]);
        let offsets = [-1000.0];
        let rim = Rim::new(3, 1, &[]);
        let field = Field {
            positions: &positions,
            normals: &normals,
            triangles: &triangles,
            offsets: &offsets,
            layers: [offsets[0], 0.0],
            around: &around,
            adjacent: &adjacent,
            thickness: 1000.0,
            edge: 1.0,
            boundary: &[],
            rim: &rim,
            size: 1000.0,
        };

        // Act
        let result = relax(&field);

        // Assert: they stay at the floor and are counted
        let relaxed = result.expect("relaxed");
        assert_eq!(relaxed.unresolved, 3);
        assert!(relaxed.scale.iter().all(|&s| (s - MIN_SCALE).abs() < 1e-9));
    }

    #[test]
    fn a_fold_at_the_floor_is_resolved_by_smoothing_the_normals_with_those_of_the_neighbours() {
        // Arrange: the same triangle, now with its neighbours: the mean of the three normals is up
        let positions = [Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];
        let normals = [Vec3::z(), Vec3::z(), -Vec3::z()];
        let triangles = [[0u32, 1, 2]];
        let around = Csr::from_pairs(3, &[(0, 0), (1, 0), (2, 0)]);
        let adjacent = Csr::from_pairs(3, &[(0, 1), (0, 2), (1, 0), (1, 2), (2, 0), (2, 1)]);
        let offsets = [-1000.0];
        let rim = Rim::new(3, 1, &[]);
        let field = Field {
            positions: &positions,
            normals: &normals,
            triangles: &triangles,
            offsets: &offsets,
            layers: [offsets[0], 0.0],
            around: &around,
            adjacent: &adjacent,
            thickness: 1000.0,
            edge: 1.0,
            boundary: &[],
            rim: &rim,
            size: 1000.0,
        };

        // Act
        let relaxed = relax(&field).expect("relaxed");

        // Assert
        assert_eq!(relaxed.unresolved, 0);
        assert!(relaxed.normals.iter().all(|n| (n - Vec3::z()).norm() < 1e-9), "{:?}", relaxed.normals);
    }

    #[test]
    fn crossings_left_when_the_rounds_run_out_are_counted_as_unresolved() {
        // Arrange: a sharp ridge that crosses itself at the full offset, and no round to relax it in
        use crate::{Mesh, synth};
        let mut panel = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 14.0, 8.0, 93, 53);
        for p in &mut panel.positions {
            p.z = -5.0 * (p.x - 7.0).abs();
        }
        let params = super::super::ThickenParams { thickness: 1.0, symmetric: false };
        let _: &Mesh = &panel;
        ROUND_LIMIT.with(|c| c.set(Some(0)));

        // Act
        let result = super::super::thicken(&panel, &params);
        ROUND_LIMIT.with(|c| c.set(None));

        // Assert: exactly what the freeze gives: crossings counted as unresolved, here more than 0.1 %
        // of the vertices, hence the refusal (without the freeze the shell came back "clean")
        assert!(
            matches!(result, Err(super::super::ThickenError::UnresolvedCrossings { .. })),
            "{:?}",
            result.map(|t| t.report)
        );
    }
}

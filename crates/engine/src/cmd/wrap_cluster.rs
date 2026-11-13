//! Grid clustering of a coarse mesh: the last resort that brings a mesh under the patch cap when
//! edge collapse stalls on its topology (thousands of small handles, bridges and boundary loops).
//!
//! Merging the vertices of a grid cell has no topology to keep, but it can thin a wall to nothing and
//! turn a neighbouring triangle around. The result is therefore made manifold and consistently
//! oriented, and refused (`Thin`) when it keeps too little of the surface.
use std::collections::{BTreeMap, HashMap, HashSet};
use surfacecraft_geom::edit::remove_non_manifold;
use surfacecraft_geom::{Mesh, Point3};

/// Bisection steps of the cell size of the clustering.
const CLUSTER_STEPS: usize = 18;
/// The widest cell is the diagonal of the box; the finest is this fraction of it.
const FINEST_CELL: f64 = 1.0 / 4096.0;
/// The clustered mesh must keep MORE than this share of the area of the mesh it was made from. It is
/// above one half on purpose: a wall thinner than the cell merges its two sheets into one layer and
/// keeps almost exactly half of the area, which must not pass or fail on rounding.
pub(super) const MIN_AREA_SHARE: f64 = 0.55;
/// Six times the volume below which a closed piece is taken to enclose nothing (its sign means nothing).
const NO_VOLUME: f64 = 1e-9;
/// ...and at least this share of the triangles the cap allows (never fewer than four).
pub(super) const MIN_CAP_SHARE: f64 = 0.1;

/// Why no clustered mesh was made.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum ClusterError {
    /// Even the widest cell leaves more triangles than the cap, or the mesh is empty.
    Over,
    /// The clustered mesh lost too much of the surface or has too few triangles.
    Thin,
}

/// The cell of a point: floor of its offset from `origin` in cells of size `cell`.
fn cell_of(p: &Point3, origin: &Point3, cell: f64) -> [i64; 3] {
    let k = |a: f64, o: f64| ((a - o) / cell).floor() as i64;
    [k(p.x, origin.x), k(p.y, origin.y), k(p.z, origin.z)]
}

/// The mesh with the vertices of every grid cell merged into their mean; triangles that lose a
/// corner and repeated triangles are dropped, and the rest is made manifold.
fn cluster(mesh: &Mesh, origin: &Point3, cell: f64) -> Mesh {
    let keys: Vec<[i64; 3]> = mesh.positions.iter().map(|p| cell_of(p, origin, cell)).collect();
    let mut index: BTreeMap<[i64; 3], usize> = BTreeMap::new();
    for k in mesh.triangles.iter().flatten().filter_map(|&v| keys.get(v as usize)) {
        index.entry(*k).or_insert(0);
    }
    for (n, slot) in index.values_mut().enumerate() {
        *slot = n;
    }
    let mut sum = vec![(0.0, 0.0, 0.0, 0usize); index.len()];
    let used: Vec<bool> = {
        let mut u = vec![false; mesh.positions.len()];
        mesh.triangles.iter().flatten().for_each(|&v| u.get_mut(v as usize).map_or((), |x| *x = true));
        u
    };
    for (v, p) in mesh.positions.iter().enumerate() {
        let slot = used.get(v).copied().unwrap_or(false).then(|| keys.get(v).and_then(|k| index.get(k))).flatten();
        if let Some(s) = slot.and_then(|&c| sum.get_mut(c)) {
            *s = (s.0 + p.x, s.1 + p.y, s.2 + p.z, s.3 + 1);
        }
    }
    let positions = sum
        .iter()
        .map(|s| Point3::new(s.0 / s.3.max(1) as f64, s.1 / s.3.max(1) as f64, s.2 / s.3.max(1) as f64))
        .collect();
    let cell_id = |v: u32| keys.get(v as usize).and_then(|k| index.get(k)).map(|&c| c as u32);
    let mut seen = HashSet::new();
    let mut triangles = Vec::new();
    for t in &mesh.triangles {
        let (Some(a), Some(b), Some(c)) = (cell_id(t[0]), cell_id(t[1]), cell_id(t[2])) else { continue };
        let mut sorted = [a, b, c];
        sorted.sort_unstable();
        if a != b && b != c && a != c && seen.insert(sorted) {
            triangles.push([a, b, c]);
        }
    }
    let mut out = Mesh { positions, triangles };
    remove_non_manifold(&mut out);
    orient_consistently(&mut out);
    out
}

/// The three directed edges of a triangle, as stored.
fn directed(t: &[u32; 3]) -> [(u32, u32); 3] {
    [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]
}

/// Triangles on each undirected edge.
fn edge_users(mesh: &Mesh) -> HashMap<(u32, u32), Vec<usize>> {
    let mut by_edge: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (i, t) in mesh.triangles.iter().enumerate() {
        for (a, b) in directed(t) {
            by_edge.entry((a.min(b), a.max(b))).or_default().push(i);
        }
    }
    by_edge
}

/// The connected pieces of the mesh, flipped relative to their first triangle so that neighbours
/// traverse their shared edge in opposite directions: `flip[t]`, the triangles that cannot be made
/// consistent (a twist, a pinch) and the pieces as lists of triangles.
fn propagate(mesh: &Mesh, by_edge: &HashMap<(u32, u32), Vec<usize>>) -> (Vec<bool>, Vec<bool>, Vec<Vec<usize>>) {
    let n = mesh.triangles.len();
    let (mut flip, mut seen, mut bad) = (vec![false; n], vec![false; n], vec![false; n]);
    let mut pieces = Vec::new();
    for seed in 0..n {
        if seen[seed] {
            continue;
        }
        seen[seed] = true;
        let (mut stack, mut piece) = (vec![seed], Vec::new());
        while let Some(t) = stack.pop() {
            piece.push(t);
            let Some(tri) = mesh.triangles.get(t) else { continue };
            for (a, b) in directed(tri) {
                let (from, to) = if flip[t] { (b, a) } else { (a, b) };
                for &o in by_edge.get(&(a.min(b), a.max(b))).map_or(&[][..], Vec::as_slice) {
                    let Some(other) = mesh.triangles.get(o).filter(|_| o != t) else { continue };
                    // `o` must run to -> from; flipped when it is stored from -> to
                    let want = directed(other).contains(&(from, to));
                    if !seen[o] {
                        seen[o] = true;
                        flip[o] = want;
                        stack.push(o);
                    } else if flip[o] != want {
                        bad[o] = true;
                    }
                }
            }
        }
        pieces.push(piece);
    }
    (flip, bad, pieces)
}

/// Six times the signed volume enclosed by the triangles `ids` as oriented by `flip`.
fn signed_volume(mesh: &Mesh, ids: &[usize], flip: &[bool]) -> f64 {
    ids.iter()
        .filter_map(|&i| {
            let t = mesh.triangles.get(i)?;
            let p = |k: u32| mesh.positions.get(k as usize).map(|p| p.coords);
            let (a, b, c) = if flip[i] { (p(t[0])?, p(t[2])?, p(t[1])?) } else { (p(t[0])?, p(t[1])?, p(t[2])?) };
            Some(a.dot(&b.cross(&c)))
        })
        .sum()
}

/// Is every edge of the triangles `ids` used by exactly two of them (a closed piece)?
fn is_closed(mesh: &Mesh, ids: &[usize]) -> bool {
    let mut uses: HashMap<(u32, u32), usize> = HashMap::new();
    for t in ids.iter().filter_map(|&i| mesh.triangles.get(i)) {
        for (a, b) in directed(t) {
            *uses.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    uses.values().all(|&n| n == 2)
}

/// Area of triangle `i`.
fn area_of(mesh: &Mesh, i: usize) -> f64 {
    mesh.triangle(i).map_or(0.0, |t| 0.5 * (t[1] - t[0]).cross(&(t[2] - t[0])).norm())
}

/// Whether to reverse a piece whose triangles are consistent but may be turned the wrong way as a
/// whole (`flip[i]` is relative to the input): the orientation the larger part of the piece had as
/// given, by area, wins; on an exact tie (or an area that is not a number) the input orientation of
/// the largest triangle (the lowest index among equals) is kept. The rule depends on the input only,
/// never on which triangle the propagation started from.
fn input_majority_reverses(mesh: &Mesh, live: &[usize], flip: &[bool]) -> bool {
    let (mut turned, mut kept) = (0.0, 0.0);
    let mut largest: Option<(f64, usize)> = None;
    for &i in live {
        let a = area_of(mesh, i);
        if flip[i] {
            turned += a
        } else {
            kept += a
        }
        if a.is_finite() && largest.is_none_or(|(m, _)| a > m) {
            largest = Some((a, i));
        }
    }
    if turned.is_finite() && kept.is_finite() && turned != kept {
        return turned > kept;
    }
    largest.is_some_and(|(_, i)| flip[i])
}

/// Makes triangles that share an edge traverse it in opposite directions and drops the ones that
/// cannot be made consistent. The sense of each connected piece does not depend on which triangle
/// the propagation started from: a closed piece is turned outward (positive enclosed volume; when
/// that volume is zero or not a number, by the rule below), an open one keeps the orientation most
/// of its area had as given ([`input_majority_reverses`]).
pub(super) fn orient_consistently(mesh: &mut Mesh) {
    let by_edge = edge_users(mesh);
    let (mut flip, bad, pieces) = propagate(mesh, &by_edge);
    for piece in &pieces {
        let live: Vec<usize> = piece.iter().copied().filter(|&i| !bad[i]).collect();
        let volume = signed_volume(mesh, &live, &flip);
        let outward = is_closed(mesh, &live) && volume.is_finite() && volume.abs() > NO_VOLUME;
        let reverse = if outward { volume < 0.0 } else { input_majority_reverses(mesh, &live, &flip) };
        if reverse {
            live.iter().for_each(|&i| flip[i] = !flip[i]);
        }
    }
    let kept: Vec<[u32; 3]> = mesh
        .triangles
        .iter()
        .enumerate()
        .filter(|(i, _)| !bad[*i])
        .map(|(i, t)| if flip[i] { [t[0], t[2], t[1]] } else { *t })
        .collect();
    mesh.triangles = kept;
}

/// The clustering of `mesh` with the finest cell that leaves at most `cap` triangles (bisection of
/// the cell size, [`CLUSTER_STEPS`] steps). `Over` when even the widest cell leaves more; `Thin`
/// when the result keeps less than [`MIN_AREA_SHARE`] of the area of `mesh` or fewer than
/// [`MIN_CAP_SHARE`] of the cap (at least four) triangles.
pub(super) fn cluster_to_cap(mesh: &Mesh, cap: usize) -> Result<Mesh, ClusterError> {
    let bbox = mesh.bbox();
    if bbox.is_empty() {
        return Err(ClusterError::Over);
    }
    let diagonal = (bbox.max - bbox.min).norm().max(1e-9);
    let (mut fine, mut wide) = (diagonal * FINEST_CELL, diagonal);
    let mut best =
        Some(cluster(mesh, &bbox.min, wide)).filter(|m| m.triangle_count() <= cap).ok_or(ClusterError::Over)?;
    for _ in 0..CLUSTER_STEPS {
        let mid = (fine * wide).sqrt();
        let m = cluster(mesh, &bbox.min, mid);
        if m.triangle_count() <= cap {
            wide = mid;
            if m.triangle_count() >= best.triangle_count() {
                best = m;
            }
        } else {
            fine = mid;
        }
    }
    let enough = best.triangle_count() >= ((cap as f64 * MIN_CAP_SHARE) as usize).max(4);
    if enough && best.area() >= MIN_AREA_SHARE * mesh.area() { Ok(best) } else { Err(ClusterError::Thin) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::synth;

    fn sphere(res: usize) -> Mesh {
        synth::sphere(Point3::origin(), 10.0, res, res / 2)
    }

    fn volume(m: &Mesh) -> f64 {
        signed_volume(m, &(0..m.triangles.len()).collect::<Vec<_>>(), &vec![false; m.triangles.len()])
    }

    #[test]
    fn clustering_reaches_a_cap_the_mesh_has_many_times_over() {
        // Arrange
        let mesh = sphere(120);
        assert!(mesh.triangle_count() > 10_000);

        // Act
        let out = cluster_to_cap(&mesh, 300).unwrap();

        // Assert
        assert!(out.triangle_count() <= 300, "{}", out.triangle_count());
        assert!(out.triangle_count() >= 150, "{}", out.triangle_count());
        assert!(out.validate().is_ok());
        assert_eq!(out, cluster_to_cap(&mesh, 300).unwrap());
    }

    #[test]
    fn a_clustered_mesh_is_consistently_oriented() {
        // Arrange: a sphere with every third triangle turned around
        let mut mesh = sphere(60);
        for (i, t) in mesh.triangles.iter_mut().enumerate() {
            if i % 3 == 0 {
                t.swap(1, 2);
            }
        }

        // Act
        let out = cluster_to_cap(&mesh, 400).unwrap();

        // Assert: no directed edge is used twice
        let mut used = HashSet::new();
        for t in &out.triangles {
            for e in directed(t) {
                assert!(used.insert(e), "edge {e:?} is used in the same direction twice");
            }
        }
        assert!(out.triangle_count() > 50);
    }

    #[test]
    fn clustering_has_no_answer_for_an_empty_mesh() {
        assert_eq!(cluster_to_cap(&Mesh::default(), 10), Err(ClusterError::Over));
    }

    #[test]
    fn a_component_keeps_its_orientation_whichever_triangle_the_propagation_starts_from() {
        // Arrange: a closed sphere and an open cap, each with their first triangle turned around
        let mut closed = sphere(40);
        assert!(volume(&closed) > 0.0, "the synthetic sphere is outward");
        let mut open = Mesh {
            positions: closed.positions.clone(),
            triangles: closed.triangles[..closed.triangles.len() / 3].to_vec(),
        };
        closed.triangles[0].swap(1, 2);
        open.triangles[0].swap(1, 2);
        let reference = sphere(40);

        // Act
        orient_consistently(&mut closed);
        orient_consistently(&mut open);

        // Assert: the closed one is outward again, the open one has the orientation of its majority
        assert!(volume(&closed) > 0.0);
        let same = |a: &[u32; 3], b: &[u32; 3]| (0..3).any(|k| [a[k], a[(k + 1) % 3], a[(k + 2) % 3]] == *b);
        let kept: usize = open.triangles.iter().filter(|t| reference.triangles.iter().any(|r| same(r, t))).count();
        assert!(kept * 100 >= open.triangles.len() * 99, "{kept} of {}", open.triangles.len());
    }

    #[test]
    fn a_surface_the_cells_reduce_to_nothing_is_refused_as_thin() {
        // Arrange: 40 x 40 separate small triangles 1 mm apart; any cell wider than a triangle
        // swallows them, and a cell narrower than the spacing leaves them above the cap
        let mut positions = Vec::new();
        let mut triangles = Vec::new();
        for i in 0..40u32 {
            for j in 0..40u32 {
                let k = positions.len() as u32;
                let (x, y) = (f64::from(i), f64::from(j));
                positions.extend([Point3::new(x, y, 0.0), Point3::new(x + 0.3, y, 0.0), Point3::new(x, y + 0.3, 0.0)]);
                triangles.push([k, k + 1, k + 2]);
            }
        }
        let mesh = Mesh { positions, triangles };

        // Act
        let out = cluster_to_cap(&mesh, 300);

        // Assert
        assert_eq!(out, Err(ClusterError::Thin));
    }

    #[test]
    fn a_thin_plate_whose_two_sheets_merge_in_the_cells_is_refused_as_thin() {
        // Arrange: two 40 x 40 grids 0.2 mm apart facing away from each other; a cap of 300 needs cells
        // of a few millimetres, so the sheets fall into one layer and half of the area is gone
        let mut mesh = Mesh::default();
        for (z, up) in [(0.1, true), (-0.1, false)] {
            let base = mesh.positions.len() as u32;
            for i in 0..=40u32 {
                for j in 0..=40u32 {
                    mesh.positions.push(Point3::new(f64::from(i), f64::from(j), z));
                }
            }
            for i in 0..40u32 {
                for j in 0..40u32 {
                    let v = |a: u32, b: u32| base + a * 41 + b;
                    let (p, q, r, t) = (v(i, j), v(i + 1, j), v(i + 1, j + 1), v(i, j + 1));
                    let quad = if up { [[p, q, r], [p, r, t]] } else { [[p, r, q], [p, t, r]] };
                    mesh.triangles.extend(quad);
                }
            }
        }

        // Act
        let out = cluster_to_cap(&mesh, 300);

        // Assert
        assert_eq!(out, Err(ClusterError::Thin));
    }

    #[test]
    fn a_tied_vote_keeps_the_input_orientation_of_the_largest_triangle() {
        // Arrange: a square of two triangles of equal area turned against each other; then the same with
        // the second triangle slightly larger
        let square = |far: f64| Mesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, far, 0.0),
            ],
            // [0,1,2] faces +z; [0,3,2] runs the shared edge the same way, so it must turn
            triangles: vec![[0, 1, 2], [0, 3, 2]],
        };
        let normal_z = |m: &Mesh, i: usize| {
            let t = m.triangle(i).unwrap();
            (t[1] - t[0]).cross(&(t[2] - t[0])).z
        };

        // Act
        let (mut tie, mut bigger) = (square(1.0), square(1.5));
        orient_consistently(&mut tie);
        orient_consistently(&mut bigger);

        // Assert: tie -> the first (lowest index) triangle keeps +z; the larger second one is cw as given
        assert!(normal_z(&tie, 0) > 0.0 && normal_z(&tie, 1) > 0.0);
        assert!(normal_z(&bigger, 0) < 0.0 && normal_z(&bigger, 1) < 0.0);
    }
}

//! Untangling the patch: the sound measure of a patch, the triangles turned over, and the moves that mend them.

use super::hole::*;
use super::sides::*;
use super::*;

/// What a sound patch is measured against: the least area of a triangle, and the triangles
/// between border vertices alone that were turned over as the fairing left them. Those have no
/// vertex that can move, so no move makes or mends them.
pub(super) struct Sound {
    pub(super) min_area: f64,
    pub(super) given: HashSet<[u32; 3]>,
}

impl Sound {
    pub(super) fn new(mesh: &Mesh, hole: &Hole, wedge: &Wedge, min_area: f64) -> Sound {
        let open = Sound { min_area, given: HashSet::new() };
        let given = tangled(mesh, hole, wedge, &open)
            .into_iter()
            .filter_map(|t| mesh.triangles.get(t))
            .filter(|t| t.iter().all(|&v| (v as usize) < hole.first_vertex))
            .map(same_corners)
            .collect();
        Sound { min_area, given }
    }
}

/// The corners of a triangle in an order that does not depend on where it starts.
fn same_corners(t: &[u32; 3]) -> [u32; 3] {
    let mut c = *t;
    c.sort_unstable();
    c
}

/// The patch triangles that are thinner than the least area or turned over. A triangle thinner
/// than the hole's own least area (the fairing leaves such slivers) has no direction worth the
/// name and is judged by its area alone.
pub(super) fn tangled(mesh: &Mesh, hole: &Hole, wedge: &Wedge, sound: &Sound) -> Vec<usize> {
    (hole.first_tri..mesh.triangles.len())
        .filter(|&t| {
            let area = mesh.face_area(t);
            let given = mesh.triangles.get(t).is_some_and(|c| sound.given.contains(&same_corners(c)));
            area < sound.min_area || (area >= hole.min_area() && !given && !faces_out(mesh, hole, wedge, t))
        })
        .collect()
}

/// Untangles the patch after its vertices moved: the patch vertices of every turned-over or
/// squashed triangle go to the middle of their neighbours and back onto the wedge, for up to
/// [`UNTANGLE_ROUNDS`] rounds. Returns whether the patch is sound.
pub(super) fn untangle(mesh: &mut Mesh, hole: &Hole, wedge: &Wedge, sound: &Sound, snap: f64) -> bool {
    let on_side = ON_SIDE * hole.edge;
    for _ in 0..UNTANGLE_ROUNDS {
        let bad = tangled(mesh, hole, wedge, sound);
        if bad.is_empty() {
            return true;
        }
        let mut ring: HashMap<u32, Vec<u32>> = HashMap::new();
        for t in mesh.triangles.iter().skip(hole.first_tri) {
            for k in 0..3 {
                ring.entry(t[k]).or_default().extend([t[(k + 1) % 3], t[(k + 2) % 3]]);
            }
        }
        let mut loose: Vec<u32> = bad.iter().filter_map(|&t| mesh.triangles.get(t)).flatten().copied().collect();
        loose.retain(|&v| v as usize >= hole.first_vertex);
        loose.sort_unstable();
        loose.dedup();
        for v in loose {
            let Some(around) = ring.get_mut(&v) else { continue };
            around.sort_unstable();
            around.dedup();
            let near: Vec<Point3> = around.iter().filter_map(|&w| mesh.positions.get(w as usize).copied()).collect();
            if near.is_empty() {
                continue;
            }
            let middle = Point3::from(near.iter().fold(Vec3::zeros(), |a, p| a + p.coords) / near.len() as f64);
            let (on, at_edge) = wedge.onto(snap, on_side, middle);
            if let Some(p) = mesh.positions.get_mut(v as usize) {
                *p = at_edge.unwrap_or(on);
            }
        }
    }
    tangled(mesh, hole, wedge, sound).is_empty()
}

/// Moves the patch vertices onto the wedge; false when a vertex would leave the hole's reach.
pub(super) fn move_onto(mesh: &mut Mesh, hole: &Hole, wedge: &Wedge, snap: f64) -> bool {
    let on_side = ON_SIDE * hole.edge;
    let moved: Vec<(Point3, Option<Point3>)> =
        mesh.positions.iter().skip(hole.first_vertex).map(|p| wedge.onto(snap, on_side, *p)).collect();
    let reach = 2.0 * hole.radius;
    let sane = mesh
        .positions
        .iter()
        .skip(hole.first_vertex)
        .zip(&moved)
        .all(|(p, (q, e))| [Some(*q), *e].iter().flatten().all(|to| crate::finite(to) && (to - p).norm() <= reach));
    if !sane {
        return false;
    }
    for (p, (q, _)) in mesh.positions.iter_mut().skip(hole.first_vertex).zip(&moved) {
        *p = *q;
    }
    // onto the edges one vertex at a time, so two vertices never land on each other
    for (k, (_, e)) in moved.iter().enumerate() {
        if let Some(e) = e {
            move_keeping_shape(mesh, hole.first_tri, hole.first_vertex + k, *e, hole.min_area());
        }
    }
    true
}

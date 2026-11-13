//! Sharpening a hole patch onto the sides around it, and the reason when it is left as it was.

use super::hole::*;
use super::near::*;
use super::sides::*;
use super::untangle::*;
use super::*;

/// Moves the vertices from `first_vertex` on (the patch of the hole bounded by `lp`, whose
/// triangles start at `first_tri`) onto the sides of the surface around the hole when those meet
/// at sharp edges. `around` indexes the mesh as it was before holes were filled. Returns whether
/// it did; otherwise the patch is as it was.
pub(crate) fn sharpen_patch(
    mesh: &mut Mesh,
    around: &Around,
    first_tri: usize,
    first_vertex: usize,
    lp: &[u32],
) -> bool {
    let done = sharpen(mesh, around, first_tri, first_vertex, lp);
    #[cfg(test)]
    LAST_REASON.with(|r| r.set(done.err()));
    done.is_ok()
}

#[cfg(test)]
thread_local! {
    /// Why the last patch on this thread was left as it was (`None`: it was sharpened).
    pub(super) static LAST_REASON: std::cell::Cell<Option<&'static str>> = const { std::cell::Cell::new(None) };
}

/// [`sharpen_patch`], with the reason when the patch is left as it was.
pub(super) fn sharpen(
    mesh: &mut Mesh,
    around: &Around,
    first_tri: usize,
    first_vertex: usize,
    lp: &[u32],
) -> Result<(), &'static str> {
    let hole = Hole::new(mesh, first_tri, first_vertex, lp).ok_or("no patch vertices")?;
    let wedge = read_wedge(mesh, around, lp, hole.edge)?;
    let before = patch_of(mesh, &hole);
    let min_area = (MIN_AREA_KEPT * thinnest(mesh, first_tri)).min(hole.min_area());
    let sound = Sound::new(mesh, &hole, &wedge, min_area);
    let done = shape_patch(mesh, &hole, &wedge, &sound);
    if done.is_err() {
        restore(mesh, &hole, before);
    }
    done
}

/// The patch as it is: its vertices and triangles.
fn patch_of(mesh: &Mesh, hole: &Hole) -> (Vec<Point3>, Vec<[u32; 3]>) {
    (
        mesh.positions.iter().skip(hole.first_vertex).copied().collect(),
        mesh.triangles.iter().skip(hole.first_tri).copied().collect(),
    )
}

/// Puts the patch back as [`patch_of`] saw it.
fn restore(mesh: &mut Mesh, hole: &Hole, (positions, triangles): (Vec<Point3>, Vec<[u32; 3]>)) {
    for (p, q) in mesh.positions.iter_mut().skip(hole.first_vertex).zip(positions) {
        *p = q;
    }
    for (t, u) in mesh.triangles.iter_mut().skip(hole.first_tri).zip(triangles) {
        *t = u;
    }
}

/// Moves the patch onto the wedge and untangles it, with the vertices next to an edge on the
/// edge or, when that tangles the patch, without. Then pins the corner and aligns the edges,
/// which is undone when it tangles the patch again (the vertices stay on the sides).
fn shape_patch(mesh: &mut Mesh, hole: &Hole, wedge: &Wedge, sound: &Sound) -> Result<(), &'static str> {
    let faired = patch_of(mesh, hole);
    let mut snap = EDGE_SNAP * hole.edge;
    if !move_onto(mesh, hole, wedge, snap) {
        return Err("a vertex would leave the hole");
    }
    if !untangle(mesh, hole, wedge, sound, snap) {
        restore(mesh, hole, faired);
        snap = 0.0;
        if !(move_onto(mesh, hole, wedge, snap) && untangle(mesh, hole, wedge, sound, snap)) {
            return Err("tangled after the move");
        }
    }
    let moved = patch_of(mesh, hole);
    pin_corner(mesh, hole, wedge);
    align_edges(mesh, hole, wedge);
    if !untangle(mesh, hole, wedge, sound, snap) {
        restore(mesh, hole, moved);
    }
    Ok(())
}

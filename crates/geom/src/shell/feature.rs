//! A bound on the offset of every vertex from the local feature size of the scanned sheet: how far
//! the sheet can be offset along a ray before the offset meets another part of the sheet itself.
//!
//! The offset of a set by a distance below its reach is free of self-intersections (Federer, "Curvature
//! measures", Trans. AMS 93, 1959: every point within the reach has one nearest point on the set; the
//! reach is the distance to the medial axis, the local feature size of Amenta and Bern, "Surface
//! reconstruction by Voronoi filtering", Discrete Comput. Geom. 22, 1999). The reach is not known
//! for a mesh, so it is bounded from above by a ray: from a vertex along the direction of the
//! offset to the first triangle of the sheet that does not belong to the vertex's own neighbourhood.
//! Two facing walls at distance `h` meet when both are offset by `h / 2`, so a ray that meets the
//! sheet from its far side bounds the offset by [`BACK_SAFETY`] times `h`; one that meets its near
//! side (a second layer of the scan) by [`FRONT_SAFETY`] times `h`. The bound is a limit, not a
//! proof: the exact crossing test of [`super::relax`] still decides.
//!
//! Only parts of the sheet that lie close along the sheet count (within the reach of
//! [`Surface::near`]): the two flanks of a bead, the walls of a crease. A part of the sheet far
//! along the surface (the opposite wall of a channel narrower than the wall) is real geometry: the
//! offset is not reduced for it, the crossing stays and is reported.
use super::crossing::Surface;
use super::relax::Field;
use super::topology::Csr;
use crate::{Bvh, Vec3, par};
use std::cell::OnceCell;

/// Share of the distance to a part met from its far side the offset may reach (just below one half:
/// the two walls would touch at one half).
const BACK_SAFETY: f64 = 0.45;
/// Share of the distance to a part met from its near side the offset may reach.
const FRONT_SAFETY: f64 = 0.9;
/// Hits on triangles that make a smaller angle than this cosine with the ray are ignored: the ray runs
/// along that sheet, which the offset does not cross.
const MIN_RAY_COS: f64 = 0.3;
/// Triangles with a vertex within this many edges of the vertex belong to its own neighbourhood: the
/// scan's noise lives there, and the smoothing of the normals has already dealt with it.
const OWN_HOPS: usize = 2;

/// The vertices within [`OWN_HOPS`] edges of `v` (ascending, `v` included).
fn neighbourhood(adjacent: &Csr, v: u32) -> Vec<u32> {
    let mut seen = vec![v];
    let mut frontier = vec![v];
    for _ in 0..OWN_HOPS {
        let mut next = Vec::new();
        for &u in &frontier {
            for &w in adjacent.row(u as usize) {
                if !seen.contains(&w) && !next.contains(&w) {
                    next.push(w);
                }
            }
        }
        seen.extend_from_slice(&next);
        frontier = next;
    }
    seen.sort_unstable();
    seen
}

/// The largest share of its offset vertex `v` may keep (1 when no part of the sheet is within reach).
fn cap(field: &Field, sheet: &Bvh, surface: &Surface, v: u32) -> f64 {
    let (Some(p), Some(n)) = (field.positions.get(v as usize), field.normals.get(v as usize)) else { return 1.0 };
    let own = neighbourhood(field.adjacent, v);
    let near: OnceCell<Vec<u32>> = OnceCell::new();
    let keep = |ti: usize| -> bool {
        field.triangles.get(ti).is_some_and(|t| {
            t.iter().all(|w| own.binary_search(w).is_err())
                && near.get_or_init(|| surface.within_reach(v)).iter().any(|w| t.contains(w))
        })
    };
    let mut cap: f64 = 1.0;
    for &o in field.offsets.iter().filter(|o| **o != 0.0) {
        let dir: Vec3 = n * o.signum();
        let reach = o.abs() / BACK_SAFETY;
        let usable =
            |ti: usize| keep(ti) && sheet.triangle_normal(ti).is_some_and(|m| m.dot(&dir).abs() >= MIN_RAY_COS);
        let Some((ti, t)) = sheet.ray_cast_where(p, &dir, reach, &usable) else { continue };
        let safety = match sheet.triangle_normal(ti) {
            Some(m) if m.dot(&dir) < 0.0 => FRONT_SAFETY,
            _ => BACK_SAFETY,
        };
        cap = cap.min(safety * t / o.abs());
    }
    cap
}

/// The cap on the share of the offset every vertex keeps, ascending by vertex: 1 where nothing limits
/// it, never below `floor`.
pub(super) fn caps(field: &Field, sheet: &Bvh, surface: &Surface, floor: f64) -> Vec<f64> {
    par::map(field.positions.len(), |i| cap(field, sheet, surface, i as u32).clamp(floor, 1.0))
}

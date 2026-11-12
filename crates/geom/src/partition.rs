//! Reduction of a large mesh in independent parts, for the guarded decimation.
//!
//! The triangles are split by recursive median cuts of their centroids into parts of at most
//! [`PART_TRIANGLES`] (the split depends on the mesh alone, never on the number of threads). A
//! vertex on triangles of several parts is held in place; every other vertex has all its triangles
//! in one part, so each part is decimated on its own, in parallel, exactly as the whole mesh
//! would be restricted to the edges inside the part. The reduced parts are joined in part order and
//! the caller finishes the seams on the (much smaller) joined mesh: its collapse loop continues from
//! the quadrics and budgets the parts end with, so no error history is lost.
use crate::collapse::{Collapser, Quadric};
use crate::decimate::Guard;
use crate::{Bvh, Point3, Vec3, par};

/// Largest number of triangles of a part.
const PART_TRIANGLES: usize = 40_000;
/// Owner of a vertex that lies on several parts.
const SEAM: u32 = u32::MAX;
/// Owner of a vertex no triangle uses.
const UNUSED: u32 = u32::MAX - 1;

/// A mesh with the per-vertex state of a decimation.
pub(crate) struct Whole {
    pub tris: Vec<[u32; 3]>,
    pub pos: Vec<Point3>,
    pub q: Vec<Quadric>,
    pub origin: Vec<Vec3>,
    pub locked: Vec<bool>,
}

/// A vertex a part ends with: its number in the mesh, position, quadric, original orientation and budget.
struct VertexOut {
    id: u32,
    pos: Point3,
    q: Quadric,
    origin: Vec3,
    budget: f64,
}

/// What one part ends with: triangles in mesh vertex numbers and the state of the vertices left.
struct PartOut {
    tris: Vec<[u32; 3]>,
    vertices: Vec<VertexOut>,
}

/// Splits triangle ids into parts of at most [`PART_TRIANGLES`], each in ascending order.
fn split(ids: &mut [u32], centre: &[[f64; 3]], out: &mut Vec<Vec<u32>>) {
    if ids.len() <= PART_TRIANGLES {
        let mut leaf = ids.to_vec();
        leaf.sort_unstable();
        out.push(leaf);
        return;
    }
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for &i in ids.iter() {
        for a in 0..3 {
            lo[a] = lo[a].min(centre[i as usize][a]);
            hi[a] = hi[a].max(centre[i as usize][a]);
        }
    }
    let axis = (0..3).max_by(|&a, &b| (hi[a] - lo[a]).total_cmp(&(hi[b] - lo[b]))).unwrap_or(0);
    let mid = ids.len() / 2;
    ids.select_nth_unstable_by(mid, |&a, &b| {
        centre[a as usize][axis].total_cmp(&centre[b as usize][axis]).then(a.cmp(&b))
    });
    let (left, right) = ids.split_at_mut(mid);
    split(left, centre, out);
    split(right, centre, out);
}

/// Which part owns each vertex: the part of all its triangles, [`SEAM`] when they differ.
fn owners(nv: usize, tris: &[[u32; 3]], parts: &[Vec<u32>]) -> Vec<u32> {
    let mut owner = vec![UNUSED; nv];
    for (p, part) in parts.iter().enumerate() {
        for &t in part {
            for &v in &tris[t as usize] {
                let o = &mut owner[v as usize];
                *o = if *o == UNUSED || *o == p as u32 { p as u32 } else { SEAM };
            }
        }
    }
    owner
}

/// Decimates one part, its seam vertices held.
fn reduce_part(
    whole: &Whole,
    ids: &[u32],
    owner: &[u32],
    g: (&Bvh, &[f64]),
    floor: usize,
    max_cost: Option<f64>,
) -> PartOut {
    let mut gids: Vec<u32> = ids.iter().flat_map(|&t| whole.tris[t as usize]).collect();
    gids.sort_unstable();
    gids.dedup();
    let local = |v: u32| gids.binary_search(&v).map_or(0, |k| k as u32);
    let ltris: Vec<[u32; 3]> = ids.iter().map(|&t| whole.tris[t as usize].map(local)).collect();
    let lpos: Vec<Point3> = gids.iter().map(|&v| whole.pos[v as usize]).collect();
    let lq: Vec<Quadric> = gids.iter().map(|&v| whole.q[v as usize]).collect();
    let lorigin: Vec<Vec3> = gids.iter().map(|&v| whole.origin[v as usize]).collect();
    let llocked: Vec<bool> = gids.iter().map(|&v| whole.locked[v as usize] || owner[v as usize] == SEAM).collect();
    let mut budget: Vec<f64> = gids.iter().map(|&v| g.1[v as usize]).collect();
    let mut c = Collapser::new(ltris, lpos, lq, lorigin, llocked.clone(), Some(Guard { bvh: g.0, budget: &mut budget }), false);
    c.run(floor, max_cost);
    let (tris, pos, q, origin) = c.finish();
    let mut used = vec![false; gids.len()];
    for t in &tris {
        for &v in t {
            used[v as usize] = true;
        }
    }
    let vertices = (0..gids.len())
        .filter(|&v| used[v] && !llocked[v])
        .map(|v| VertexOut { id: gids[v], pos: pos[v], q: q[v], origin: origin[v], budget: budget[v] })
        .collect();
    PartOut { tris: tris.iter().map(|t| t.map(|v| gids[v as usize])).collect(), vertices }
}

/// Reduces `whole` part by part and returns what is left, compact (vertex numbers by ascending
/// number before), with `guard.budget` rewritten to match. Seam vertices are still in place.
pub(crate) fn reduce_in_parts(
    whole: Whole,
    guard: &mut Guard<'_>,
    opts: &crate::decimate::DecimateOptions,
    max_cost: Option<f64>,
) -> Whole {
    let centre: Vec<[f64; 3]> = whole
        .tris
        .iter()
        .map(|t| {
            let p = t.map(|v| whole.pos[v as usize].coords);
            let c = (p[0] + p[1] + p[2]) / 3.0;
            [c.x, c.y, c.z]
        })
        .collect();
    let mut ids: Vec<u32> = (0..whole.tris.len() as u32).collect();
    let mut parts = Vec::new();
    split(&mut ids, &centre, &mut parts);
    drop((ids, centre));
    let owner = owners(whole.pos.len(), &whole.tris, &parts);
    let total = whole.tris.len();
    let bvh = guard.bvh;
    let outs: Vec<PartOut> = {
        let budget: &[f64] = guard.budget;
        par::map_slice(&parts, |ids| {
            let floor = part_floor(opts.target_triangles, ids.len(), total);
            reduce_part(&whole, ids, &owner, (bvh, budget), floor, max_cost)
        })
    };
    let Whole { tris: _, mut pos, mut q, mut origin, locked } = whole;
    let mut budget = std::mem::take(guard.budget);
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for out in outs {
        tris.extend(out.tris);
        for v in out.vertices {
            let id = v.id as usize;
            (pos[id], q[id], origin[id], budget[id]) = (v.pos, v.q, v.origin, v.budget);
        }
    }
    compact_whole(Whole { tris, pos, q, origin, locked }, budget, guard)
}

/// Renumbers the vertices in use by ascending number; the matching budgets go to `guard`.
fn compact_whole(whole: Whole, budget: Vec<f64>, guard: &mut Guard<'_>) -> Whole {
    let mut used = vec![false; whole.pos.len()];
    for t in &whole.tris {
        for &v in t {
            used[v as usize] = true;
        }
    }
    let mut new_id = vec![0u32; used.len()];
    let kept: Vec<usize> = (0..used.len()).filter(|&v| used[v]).collect();
    for (k, &v) in kept.iter().enumerate() {
        new_id[v] = k as u32;
    }
    *guard.budget = kept.iter().map(|&v| budget[v]).collect();
    Whole {
        tris: whole.tris.iter().map(|t| t.map(|v| new_id[v as usize])).collect(),
        pos: kept.iter().map(|&v| whole.pos[v]).collect(),
        q: kept.iter().map(|&v| whole.q[v]).collect(),
        origin: kept.iter().map(|&v| whole.origin[v]).collect(),
        locked: kept.iter().map(|&v| whole.locked[v]).collect(),
    }
}

/// The share of the triangle floor `target` that falls to a part of `part` of `total` triangles,
/// rounded up. The product is taken in 128 bits: on a 32-bit target (wasm) it does not fit `usize`.
fn part_floor(target: usize, part: usize, total: usize) -> usize {
    let share = (target as u128 * part as u128).div_ceil(total.max(1) as u128);
    usize::try_from(share).unwrap_or(usize::MAX)
}

#[cfg(test)]
mod floor_tests {
    use super::part_floor;

    #[test]
    fn the_floor_of_a_part_does_not_overflow() {
        assert_eq!(part_floor(120_000, 40_000, 160_000), 30_000);
        assert_eq!(part_floor(10, 3, 9), 4);
        assert_eq!(part_floor(usize::MAX, usize::MAX, 1), usize::MAX);
        assert_eq!(part_floor(5, 5, 0), 25);
    }
}

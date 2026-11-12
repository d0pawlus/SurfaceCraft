//! Welding of coincident vertices without one global sort.
//!
//! Every vertex falls into a cell of a grid (the cell edge is the tolerance). The vertices are dealt
//! into [`BUCKETS`] buckets by a hash of their cell, a chunk of vertices at a time in parallel, so
//! all vertices of a cell meet in the same bucket; each bucket is then sorted on its own (cell, then
//! index) and the first vertex of every cell run, the one with the lowest index, becomes the
//! representative of the run. The result is that of one global sort of (cell, index): it does not
//! depend on the number of threads, and the transient memory is a few words per vertex instead of
//! a 32-byte key.
use crate::{Point3, par};

/// Vertices dealt out per task.
const CHUNK: usize = 1 << 16;
/// Number of buckets (a power of two; the bucket is the top bits of the cell hash).
const BUCKETS: usize = 256;
/// Bits of the hash that select the bucket.
const BUCKET_BITS: u32 = BUCKETS.trailing_zeros();

type Cell = [i64; 3];

fn cell_of(p: &Point3, inv: f64) -> Cell {
    [(p.x * inv).round() as i64, (p.y * inv).round() as i64, (p.z * inv).round() as i64]
}

fn bucket_of(c: &Cell) -> usize {
    let h = (c[0] as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15)
        ^ (c[1] as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F).rotate_left(21)
        ^ (c[2] as u64).wrapping_mul(0x1656_67B1_9E37_79F9).rotate_left(42);
    ((h ^ (h >> 29)).wrapping_mul(0xBF58_476D_1CE4_E5B9) >> (64 - BUCKET_BITS)) as usize
}

/// For every point the lowest index of the points in its grid cell (cell edge `1 / inv`).
pub(crate) fn representatives(pos: &[Point3], inv: f64) -> Vec<u32> {
    let n = pos.len();
    let dealt: Vec<Vec<Vec<u32>>> = par::map(n.div_ceil(CHUNK), |c| {
        let (lo, hi) = (c * CHUNK, ((c + 1) * CHUNK).min(n));
        let mut by: Vec<Vec<u32>> = (0..BUCKETS).map(|_| Vec::with_capacity((hi - lo) / BUCKETS * 5 / 4 + 8)).collect();
        for i in lo..hi {
            let Some(p) = pos.get(i) else { continue };
            if let Some(list) = by.get_mut(bucket_of(&cell_of(p, inv))) {
                list.push(i as u32);
            }
        }
        by
    });
    let resolved: Vec<Vec<(u32, u32)>> = par::map(BUCKETS, |b| resolve_bucket(pos, inv, &dealt, b));
    drop(dealt);
    let mut rep = vec![0u32; n];
    for (i, r) in resolved.into_iter().flatten() {
        if let Some(slot) = rep.get_mut(i as usize) {
            *slot = r;
        }
    }
    rep
}

/// (vertex, representative) of every vertex of bucket `b`.
fn resolve_bucket(pos: &[Point3], inv: f64, dealt: &[Vec<Vec<u32>>], b: usize) -> Vec<(u32, u32)> {
    let mut keyed: Vec<(Cell, u32)> = Vec::new();
    for list in dealt.iter().filter_map(|d| d.get(b)) {
        keyed.extend(list.iter().filter_map(|&i| Some((cell_of(pos.get(i as usize)?, inv), i))));
    }
    keyed.sort_unstable();
    let mut out = Vec::with_capacity(keyed.len());
    let mut first = (Cell::default(), u32::MAX);
    for (k, &(cell, i)) in keyed.iter().enumerate() {
        if k == 0 || cell != first.0 {
            first = (cell, i);
        }
        out.push((i, first.1));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Mesh, Rng};
    use std::collections::HashMap;

    /// Soup of triangles over a jittered lattice, so that many vertices coincide exactly and some
    /// differ by less than the tolerance.
    fn soup(count: usize) -> Mesh {
        let mut rng = Rng::new(5);
        let mut m = Mesh::default();
        for _ in 0..count {
            let i = m.positions.len() as u32;
            for _ in 0..3 {
                let p = Point3::new(rng.index(40) as f64, rng.index(40) as f64, rng.index(40) as f64) * 0.5;
                m.positions.push(p + crate::Vec3::new(rng.range(0.0, 1e-4), 0.0, 0.0));
            }
            m.triangles.push([i, i + 1, i + 2]);
        }
        m
    }

    fn reference(pos: &[Point3], inv: f64) -> Vec<u32> {
        let mut first: HashMap<Cell, u32> = HashMap::new();
        pos.iter().enumerate().map(|(i, p)| *first.entry(cell_of(p, inv)).or_insert(i as u32)).collect()
    }

    #[test]
    fn matches_the_first_vertex_of_every_cell() {
        let m = soup(60_000);
        let inv = 1.0 / 1e-3;
        assert_eq!(representatives(&m.positions, inv), reference(&m.positions, inv));
    }

    #[test]
    fn one_thread_and_many_threads_agree() {
        let m = soup(60_000);
        let run = |threads: usize| {
            let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
            pool.install(|| {
                let mut w = m.clone();
                let removed = w.weld(1e-3);
                (removed, w)
            })
        };
        let (r1, m1) = run(1);
        let (r8, m8) = run(8);
        assert_eq!(r1, r8);
        assert_eq!(m1, m8);
        assert!(r1 > 0 && m1.vertex_count() < m.vertex_count());
    }

    #[test]
    fn empty_and_tiny_inputs() {
        assert!(representatives(&[], 1.0).is_empty());
        let p = [Point3::origin(), Point3::origin()];
        assert_eq!(representatives(&p, 1.0), vec![0, 0]);
    }
}

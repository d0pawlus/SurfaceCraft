//! The corners of a binary STL as an indexed mesh, in one pass and little memory: corners with the
//! same coordinates (bit for bit) get one vertex as they are read, through an open-addressing table
//! of vertex ids. Reading every corner as its own point first took 72 bytes per triangle in `f64`
//! points plus the welding of three times as many points as the mesh has; this takes the mesh and
//! a table of a few bytes per vertex. The caller welds what is left (coordinates that differ only
//! below the welding tolerance), so the result is the one of welding every corner.
//!
//! The file is hostile: its slots come from a hash with a random seed, a lookup that probes too far
//! (coordinates chosen to collide) makes the table take another seed, and the number of distinct
//! corners is bounded (a file whose corners are all apart has three vertices per triangle).
use surfacecraft_geom::{Mesh, Point3};

/// Bytes of the header and the triangle count.
const HEADER: usize = 84;
/// Bytes of one facet record.
const FACET: usize = 50;
/// Bytes of the normal before a facet's corners.
const NORMAL: usize = 12;
/// An empty slot of the table.
const EMPTY: u32 = u32::MAX;
/// The table grows when it is more than this share full (numerator over 8).
const MAX_LOAD_EIGHTHS: usize = 5;
/// Slots of the smallest table.
const MIN_SLOTS: usize = 1 << 10;
/// Slots a lookup may probe before the table takes another seed. Far above what a table at most
/// 5/8 full probes by chance (a few slots on average), so only coordinates chosen to collide reach it.
const MAX_PROBE: usize = 1024;
/// Seeds tried before the file is refused as made to collide.
const MAX_SEEDS: usize = 8;

/// Why the corners were refused (the first in file order).
pub(super) enum CornerFault {
    /// The file ends inside a facet.
    Truncated,
    /// Triangle `t` has a coordinate that is not finite.
    NonFinite(usize),
    /// More distinct corners than the limit.
    TooManyVertices,
    /// Every seed met coordinates that collide.
    Colliding,
}

/// A corner's coordinates as the file has them.
type Bits = [u32; 3];

fn bits_at(bytes: &[u8], at: usize) -> Option<Bits> {
    let s = bytes.get(at..at.checked_add(12)?)?;
    let word = |k: usize| s.get(k..k + 4).and_then(|w| <[u8; 4]>::try_from(w).ok()).map(u32::from_le_bytes);
    Some([word(0)?, word(4)?, word(8)?])
}

/// The bits a vertex was read from: its coordinates came from `f32` and convert back exactly.
fn bits_of(p: &Point3) -> Bits {
    [(p.x as f32).to_bits(), (p.y as f32).to_bits(), (p.z as f32).to_bits()]
}

/// The finaliser of SplitMix64: every input bit moves every output bit.
fn mix(mut h: u64) -> u64 {
    h = (h ^ (h >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h = (h ^ (h >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    h ^ (h >> 31)
}

/// A seed from the system's random source; a fixed one where there is none (the probe bound still
/// holds the work of a colliding file).
fn first_seed() -> u64 {
    let mut b = [0u8; 8];
    if getrandom::fill(&mut b).is_ok() { u64::from_le_bytes(b) } else { 0x2545_F491_4F6C_DD1D }
}

/// Vertex ids by coordinate bits, the coordinates kept by the vertices themselves.
struct Table {
    slots: Vec<u32>,
    mask: usize,
    seed: u64,
}

/// Where a lookup ended.
enum Probe {
    Found(u32),
    Free(usize),
    /// Probed [`MAX_PROBE`] slots.
    TooFar,
}

impl Table {
    fn with_slots(slots: usize, seed: u64) -> Self {
        Self { slots: vec![EMPTY; slots], mask: slots - 1, seed }
    }

    fn slot_of(&self, b: &Bits) -> usize {
        let low = u64::from(b[0]) | (u64::from(b[1]) << 32);
        let h = mix(mix(low ^ self.seed) ^ u64::from(b[2]) ^ self.seed.rotate_left(32));
        h as usize & self.mask
    }

    /// The id of the vertex with coordinates `b`, or where to put it.
    fn find(&self, b: &Bits, positions: &[Point3]) -> Probe {
        let mut at = self.slot_of(b);
        for _ in 0..MAX_PROBE {
            match self.slots.get(at).copied() {
                Some(EMPTY) | None => return Probe::Free(at),
                Some(id) if positions.get(id as usize).is_some_and(|p| bits_of(p) == *b) => return Probe::Found(id),
                Some(_) => at = (at + 1) & self.mask,
            }
        }
        Probe::TooFar
    }

    /// A table of `slots` slots holding every vertex of `positions`: with `seed`, or with fresh
    /// seeds while vertices collide. `None` when every seed did.
    fn filled(slots: usize, mut seed: u64, positions: &[Point3]) -> Option<Table> {
        for _ in 0..MAX_SEEDS {
            let mut table = Table::with_slots(slots, seed);
            let placed = positions.iter().enumerate().all(|(id, p)| match table.find(&bits_of(p), positions) {
                Probe::Free(at) => table.slots.get_mut(at).map(|s| *s = id as u32).is_some(),
                Probe::Found(_) | Probe::TooFar => false,
            });
            if placed {
                return Some(table);
            }
            seed = fresh_seed(seed);
        }
        None
    }
}

/// Another seed after `seed`.
fn fresh_seed(seed: u64) -> u64 {
    mix(seed ^ first_seed() ^ 0x9E37_79B9_7F4A_7C15)
}

/// The `n` triangles of the binary STL `bytes` (its count already checked against its length and
/// the limits) with bit-equal corners joined, vertices in the order they first appear; at most
/// `max_vertices` of them.
pub(super) fn indexed(bytes: &[u8], n: usize, max_vertices: usize) -> Result<Mesh, CornerFault> {
    // a closed scan has about half as many vertices as triangles
    let expected = (n / 2 + 1).min(max_vertices);
    let mut positions: Vec<Point3> = Vec::with_capacity(expected);
    let mut triangles: Vec<[u32; 3]> = Vec::with_capacity(n);
    let slots = (expected * 8 / MAX_LOAD_EIGHTHS + 1).next_power_of_two().max(MIN_SLOTS);
    let mut table = Table::with_slots(slots, first_seed());
    for t in 0..n {
        let base = HEADER + t * FACET + NORMAL;
        let mut tri = [0u32; 3];
        for (c, slot) in tri.iter_mut().enumerate() {
            let b = bits_at(bytes, base + c * 12).ok_or(CornerFault::Truncated)?;
            let mut probe = table.find(&b, &positions);
            let mut seeds = 0;
            while matches!(probe, Probe::TooFar) {
                // coordinates chosen to collide under this seed: another seed spreads them
                seeds += 1;
                if seeds > MAX_SEEDS {
                    return Err(CornerFault::Colliding);
                }
                let seed = fresh_seed(table.seed);
                table = Table::filled(table.slots.len(), seed, &positions).ok_or(CornerFault::Colliding)?;
                probe = table.find(&b, &positions);
            }
            *slot = match probe {
                Probe::Found(id) => id,
                Probe::TooFar => return Err(CornerFault::Colliding),
                Probe::Free(at) => {
                    let p = Point3::new(
                        f64::from(f32::from_bits(b[0])),
                        f64::from(f32::from_bits(b[1])),
                        f64::from(f32::from_bits(b[2])),
                    );
                    if !surfacecraft_geom::finite(&p) {
                        return Err(CornerFault::NonFinite(t));
                    }
                    if positions.len() >= max_vertices {
                        return Err(CornerFault::TooManyVertices);
                    }
                    if positions.len() == positions.capacity() {
                        // grows by doubling, but never past the limit
                        let wanted = (positions.len() * 2).max(MIN_SLOTS).min(max_vertices);
                        positions.reserve_exact(wanted.saturating_sub(positions.len()));
                    }
                    let id = positions.len() as u32;
                    positions.push(p);
                    if let Some(s) = table.slots.get_mut(at) {
                        *s = id;
                    }
                    if positions.len() * 8 > table.slots.len() * MAX_LOAD_EIGHTHS {
                        table = Table::filled(table.slots.len() * 2, table.seed, &positions)
                            .ok_or(CornerFault::Colliding)?;
                    }
                    id
                }
            };
        }
        triangles.push(tri);
    }
    Ok(Mesh { positions, triangles })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn points(n: usize) -> Vec<Point3> {
        (0..n).map(|i| Point3::new(i as f64, 0.5, -1.0)).collect()
    }

    #[test]
    fn a_lookup_that_probes_too_far_says_so() {
        // every slot holds a vertex with other coordinates: the lookup gives up, it does not loop
        let positions = points(1);
        let mut table = Table::with_slots(MAX_PROBE * 2, 7);
        table.slots.iter_mut().for_each(|s| *s = 0);
        assert!(matches!(table.find(&bits_of(&Point3::new(9.0, 9.0, 9.0)), &positions), Probe::TooFar));
    }

    #[test]
    fn a_table_is_filled_under_any_seed_and_finds_every_vertex() {
        let positions = points(500);
        for seed in [0, 1, u64::MAX, first_seed()] {
            let table = Table::filled(MIN_SLOTS, seed, &positions).unwrap();
            for (id, p) in positions.iter().enumerate() {
                assert!(matches!(table.find(&bits_of(p), &positions), Probe::Found(f) if f as usize == id));
            }
        }
    }
}

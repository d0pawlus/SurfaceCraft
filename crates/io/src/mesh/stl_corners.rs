//! The corners of a binary STL as an indexed mesh, in one pass and little memory: corners with the
//! same coordinates (bit for bit) get one vertex as they are read, through an open-addressing table
//! of vertex ids. Reading every corner as its own point first took 72 bytes per triangle in `f64`
//! points plus the welding of three times as many points as the mesh has; this takes the mesh and
//! a table of a few bytes per vertex. The caller welds what is left (coordinates that differ only
//! below the welding tolerance), so the result is the one of welding every corner.
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

/// Why the corners were refused (the first in file order).
pub(super) enum CornerFault {
    /// The file ends inside a facet.
    Truncated,
    /// Triangle `t` has a coordinate that is not finite.
    NonFinite(usize),
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

fn slot_of(b: &Bits, mask: usize) -> usize {
    let mut h = u64::from(b[0]) | (u64::from(b[1]) << 32);
    h = (h ^ (h >> 33)).wrapping_mul(0xFF51_AFD7_ED55_8CCD);
    h ^= u64::from(b[2]).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    h = (h ^ (h >> 33)).wrapping_mul(0xC4CE_B9FE_1A85_EC53);
    (h ^ (h >> 33)) as usize & mask
}

/// Vertex ids by coordinate bits, the coordinates kept by the vertices themselves.
struct Table {
    slots: Vec<u32>,
    mask: usize,
}

impl Table {
    fn new(expected: usize) -> Self {
        let slots = (expected * 8 / MAX_LOAD_EIGHTHS + 1).next_power_of_two().max(MIN_SLOTS);
        Self { slots: vec![EMPTY; slots], mask: slots - 1 }
    }

    /// The id of the vertex with coordinates `b`, or where to put it.
    fn find(&self, b: &Bits, positions: &[Point3]) -> Result<u32, usize> {
        let mut at = slot_of(b, self.mask);
        loop {
            match self.slots.get(at).copied() {
                Some(EMPTY) | None => return Err(at),
                Some(id) if positions.get(id as usize).is_some_and(|p| bits_of(p) == *b) => return Ok(id),
                Some(_) => at = (at + 1) & self.mask,
            }
        }
    }

    /// Twice the slots, every vertex placed again.
    fn grow(&mut self, positions: &[Point3]) {
        let mut bigger = Table { slots: vec![EMPTY; self.slots.len() * 2], mask: self.slots.len() * 2 - 1 };
        for (id, p) in positions.iter().enumerate() {
            if let Err(at) = bigger.find(&bits_of(p), positions)
                && let Some(slot) = bigger.slots.get_mut(at)
            {
                *slot = id as u32;
            }
        }
        *self = bigger;
    }
}

/// The `n` triangles of the binary STL `bytes` (its count already checked against its length and
/// the limits) with bit-equal corners joined, vertices in the order they first appear.
pub(super) fn indexed(bytes: &[u8], n: usize) -> Result<Mesh, CornerFault> {
    // a closed scan has about half as many vertices as triangles
    let mut positions: Vec<Point3> = Vec::with_capacity(n / 2 + 1);
    let mut triangles: Vec<[u32; 3]> = Vec::with_capacity(n);
    let mut table = Table::new(n / 2 + 1);
    for t in 0..n {
        let base = HEADER + t * FACET + NORMAL;
        let mut tri = [0u32; 3];
        for (c, slot) in tri.iter_mut().enumerate() {
            let b = bits_at(bytes, base + c * 12).ok_or(CornerFault::Truncated)?;
            *slot = match table.find(&b, &positions) {
                Ok(id) => id,
                Err(at) => {
                    let p = Point3::new(
                        f64::from(f32::from_bits(b[0])),
                        f64::from(f32::from_bits(b[1])),
                        f64::from(f32::from_bits(b[2])),
                    );
                    if !surfacecraft_geom::finite(&p) {
                        return Err(CornerFault::NonFinite(t));
                    }
                    let id = positions.len() as u32;
                    positions.push(p);
                    if let Some(s) = table.slots.get_mut(at) {
                        *s = id;
                    }
                    if positions.len() * 8 > table.slots.len() * MAX_LOAD_EIGHTHS {
                        table.grow(&positions);
                    }
                    id
                }
            };
        }
        triangles.push(tri);
    }
    Ok(Mesh { positions, triangles })
}

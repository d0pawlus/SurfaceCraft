//! Orientation of a Bézier patch network built without the kernel's healer (P9-O3).
//!
//! The healer would notice patches that run against their neighbours and a shell that is inside
//! out; without it the network is taken as it is, so this pass settles what it can before the
//! record is assembled. Faces are oriented by `u × v`; flipping a patch transposes its net, which
//! turns its corners `[c0, c1, c2, c3]` into `[c0, c3, c2, c1]` and its normal around.
//!
//! 1. Consistency. Every edge shared by two patches must be traversed in opposite directions by
//!    them. The patches that disagree are flipped (a parity union-find over the shared edges,
//!    linear in the patch count); a network where no flipping makes every edge agree is not
//!    orientable and is refused. Within a connected component the patch of the lowest index keeps
//!    its orientation, whatever the order of the unions.
//! 2. Outward direction, only when the network has exactly one closed component (every edge used
//!    twice): the signed volume of its patches (`bezier_volume`, exact quadrature of the patches
//!    themselves, not of their control polyhedron) decides, and a component of negative volume is
//!    flipped as a whole. A volume that is not clearly larger than the bound of its rounding
//!    error, or not finite, is an error: the pass does not guess.
//! 3. Several closed components may nest (a cavity is a closed component with its normals
//!    pointing into the void): which of them is outer is not decided here, so each keeps the
//!    orientation it was given after step 1. Open components are not touched beyond step 1.
use super::bezier_volume::{Rule, Sum};
use crate::{KernelError, Result};
use std::collections::HashMap;

/// The volume must exceed the bound of its rounding error (`bezier_volume`, valid before any
/// cancellation) by this factor for its sign to be trusted.
const SIGN_SAFETY: f64 = 4.0;

/// The corners of a patch after a flip.
pub(crate) fn flipped_corners(c: [u32; 4]) -> [u32; 4] {
    [c[0], c[3], c[2], c[1]]
}

/// Union-find where every element also knows its parity (flipped or not) against its root.
struct ParityForest {
    parent: Vec<u32>,
    parity: Vec<bool>,
}

impl ParityForest {
    fn new(n: usize) -> Self {
        Self { parent: (0..n as u32).collect(), parity: vec![false; n] }
    }

    /// The root of `x` and the parity of `x` against it; compresses the path.
    fn find(&mut self, x: usize) -> (usize, bool) {
        let mut path = Vec::new();
        let mut cur = x;
        while self.parent.get(cur).is_some_and(|&p| p as usize != cur) {
            path.push(cur);
            cur = self.parent.get(cur).map_or(cur, |&p| p as usize);
        }
        // from the root downwards: a node's parity against the root is its own against its parent
        // plus the parent's
        let mut acc = false;
        for &node in path.iter().rev() {
            acc ^= self.parity.get(node).copied().unwrap_or(false);
            if let (Some(p), Some(q)) = (self.parent.get_mut(node), self.parity.get_mut(node)) {
                *p = cur as u32;
                *q = acc;
            }
        }
        (cur, if x == cur { false } else { self.parity.get(x).copied().unwrap_or(false) })
    }

    /// Requires `parity(a) ^ parity(b) == differ`; `false` when that contradicts earlier unions.
    fn union(&mut self, a: usize, b: usize, differ: bool) -> bool {
        let ((ra, pa), (rb, pb)) = (self.find(a), self.find(b));
        if ra == rb {
            return pa ^ pb == differ;
        }
        if let (Some(p), Some(q)) = (self.parent.get_mut(rb), self.parity.get_mut(rb)) {
            *p = ra as u32;
            *q = pa ^ pb ^ differ;
        }
        true
    }
}

/// The connected components of a network and how its patches must turn to agree.
pub(crate) struct Components {
    /// Must patch `f` be flipped (against the lowest patch of its component)?
    pub flips: Vec<bool>,
    /// The lowest patch index of the component of each patch.
    pub component: Vec<usize>,
    /// The components (by lowest patch) in which every edge is used by exactly two patches.
    pub closed: Vec<usize>,
}

/// One edge between layout vertices: its first patch, the direction that patch runs it, the uses.
struct EdgeSeen {
    face: usize,
    forward: bool,
    uses: u32,
}

/// Parity unions over the shared edges; the edge table for the closed test.
fn unite(corners: &[[u32; 4]]) -> Result<(ParityForest, HashMap<(u32, u32), EdgeSeen>)> {
    let mut forest = ParityForest::new(corners.len());
    let mut seen: HashMap<(u32, u32), EdgeSeen> = HashMap::with_capacity(corners.len() * 2);
    for (f, c) in corners.iter().enumerate() {
        for s in 0..4 {
            let (a, b) = (c[s], c[(s + 1) % 4]);
            let forward = a < b;
            let entry = seen.entry((a.min(b), a.max(b))).or_insert(EdgeSeen { face: f, forward, uses: 0 });
            entry.uses += 1;
            // the first two uses must run in opposite directions; a third is refused later
            if entry.uses == 2 && !forest.union(entry.face, f, entry.forward == forward) {
                return Err(KernelError::Argument(
                    "the patches cannot be oriented consistently (a twisted or non-orientable network)".into(),
                ));
            }
        }
    }
    Ok((forest, seen))
}

/// The flips that make every shared edge consistent, normalised so that the lowest patch of each
/// component keeps its orientation, and which components are closed.
pub(crate) fn components(corners: &[[u32; 4]]) -> Result<Components> {
    let (mut forest, seen) = unite(corners)?;
    let nf = corners.len();
    let mut lowest_of_root: HashMap<usize, (usize, bool)> = HashMap::new();
    let (mut flips, mut component) = (Vec::with_capacity(nf), Vec::with_capacity(nf));
    for f in 0..nf {
        let (root, parity) = forest.find(f);
        let (lowest, base) = *lowest_of_root.entry(root).or_insert((f, parity));
        component.push(lowest);
        flips.push(parity ^ base);
    }
    let mut open = vec![false; nf];
    for edge in seen.values().filter(|e| e.uses != 2) {
        if let Some(slot) = component.get(edge.face).and_then(|&c| open.get_mut(c)) {
            *slot = true;
        }
    }
    let closed = (0..nf).filter(|&f| component.get(f) == Some(&f) && !open[f]).collect();
    Ok(Components { flips, component, closed })
}

/// Turns a component whose enclosed volume is negative; refuses one whose sign is unknown.
fn orient_closed(patches: &[Vec<Vec<[f64; 3]>>], parts: &Components, which: usize, flips: &mut [bool]) -> Result<()> {
    let members: Vec<usize> = (0..patches.len()).filter(|&f| parts.component.get(f) == Some(&which)).collect();
    let degree = patches.first().map_or(1, |p| p.len().saturating_sub(1)).max(1);
    let points: Vec<[f64; 3]> = members.iter().filter_map(|&f| patches.get(f)).flatten().flatten().copied().collect();
    let first = points.first().copied().unwrap_or([0.0; 3]);
    // the reference point only conditions the sum: the first control point plus the mean offset
    let mean: [f64; 3] =
        std::array::from_fn(|k| points.iter().map(|p| p[k] - first[k]).sum::<f64>() / points.len().max(1) as f64);
    let centre: [f64; 3] = std::array::from_fn(|k| first[k] + mean[k]);
    let rule = Rule::for_degree(degree);
    let mut sum = Sum::default();
    for &f in &members {
        let sign = if flips.get(f).copied().unwrap_or(false) { -1.0 } else { 1.0 };
        if let Some(net) = patches.get(f) {
            rule.add_patch(&mut sum, net, centre, sign);
        }
    }
    let volume = sum.value();
    let uncertainty = sum.uncertainty();
    if !volume.is_finite() || !uncertainty.is_finite() {
        return Err(KernelError::Argument(
            "the volume of the closed network overflows or is not a number, so its outward side is unknown".into(),
        ));
    }
    if volume.abs() <= SIGN_SAFETY * uncertainty {
        return Err(KernelError::Argument(format!(
            "the closed network encloses no volume that can be told from zero ({volume:e} within {uncertainty:e}), so its outward side is unknown"
        )));
    }
    if volume < 0.0 {
        members.iter().for_each(|&f| {
            if let Some(slot) = flips.get_mut(f) {
                *slot = !*slot;
            }
        });
    }
    Ok(())
}

/// For each patch: must it be flipped? `Err` for a network that cannot be oriented.
pub(crate) fn flips(patches: &[Vec<Vec<[f64; 3]>>], corners: &[[u32; 4]]) -> Result<Vec<bool>> {
    let parts = components(corners)?;
    let mut flips = parts.flips.clone();
    if let [only] = parts.closed.as_slice() {
        orient_closed(patches, &parts, *only, &mut flips)?;
    }
    Ok(flips)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mobius_strip_is_not_orientable() {
        let strip = [[0, 1, 4, 3], [1, 2, 5, 4], [2, 3, 0, 5]];
        assert!(components(&strip).is_err());
    }

    #[test]
    fn a_flipped_neighbour_is_found() {
        // two quads sharing the edge 1-2, the second written the wrong way round
        let parts = components(&[[0, 1, 2, 3], [1, 2, 5, 4]]).expect("orientable");
        assert_eq!(parts.flips, [false, true]);
    }

    #[test]
    fn the_lowest_patch_keeps_its_orientation_when_a_later_patch_bridges_two_components() {
        let parts = components(&[[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4]]).expect("orientable");
        assert_eq!(parts.flips, [false, true, true]);
        assert_eq!(parts.component, [0, 0, 0]);
    }

    #[test]
    fn a_consistent_network_has_no_flips() {
        let cube = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]];
        let parts = components(&cube).expect("orientable");
        assert!(parts.flips.iter().all(|f| !f));
        assert_eq!(parts.closed, [0]);
    }

    /// A cube `[lo, hi]^3` as six flat bilinear patches with layout vertex ids from `base`, oriented
    /// outward (`outward`) or into the cube.
    fn cube(lo: f64, hi: f64, base: u32, outward: bool) -> (Vec<Vec<Vec<[f64; 3]>>>, Vec<[u32; 4]>) {
        let v: Vec<[f64; 3]> = (0..8)
            .map(|i| {
                [if i & 1 == 0 { lo } else { hi }, if i & 2 == 0 { lo } else { hi }, if i & 4 == 0 { lo } else { hi }]
            })
            .collect();
        let quads: [[usize; 4]; 6] =
            [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [1, 3, 7, 5], [3, 2, 6, 7], [2, 0, 4, 6]];
        let flat = |q: [usize; 4]| -> Vec<Vec<[f64; 3]>> {
            (0..2)
                .map(|i| {
                    (0..2)
                        .map(|j| {
                            std::array::from_fn(|k| {
                                v[q[0]][k] + (v[q[1]][k] - v[q[0]][k]) * i as f64 + (v[q[3]][k] - v[q[0]][k]) * j as f64
                            })
                        })
                        .collect()
                })
                .collect()
        };
        let quads = quads.map(|q| if outward { q } else { [q[0], q[3], q[2], q[1]] });
        (quads.iter().map(|&q| flat(q)).collect(), quads.iter().map(|q| q.map(|i| base + i as u32)).collect())
    }

    #[test]
    fn an_outward_cube_keeps_its_orientation_and_an_inside_out_one_is_turned() {
        let (patches, corners) = cube(0.0, 1.0, 0, true);
        assert_eq!(flips(&patches, &corners).expect("outward"), [false; 6]);
        let (patches, corners) = cube(0.0, 1.0, 0, false);
        assert_eq!(flips(&patches, &corners).expect("inside out"), [true; 6]);
    }

    #[test]
    fn nested_closed_components_keep_the_orientation_they_were_given() {
        // an outer cube and, inside it, a cube whose normals point into the void it bounds
        let (mut patches, mut corners) = cube(0.0, 10.0, 0, true);
        let (inner, inner_corners) = cube(4.0, 6.0, 8, false);
        patches.extend(inner);
        corners.extend(inner_corners);
        assert_eq!(flips(&patches, &corners).expect("two closed components"), [false; 12]);
    }
}

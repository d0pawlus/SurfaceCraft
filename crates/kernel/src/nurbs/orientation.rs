//! Orientation of a Bézier patch network built without the kernel's healer (P9-O3).
//!
//! The healer would notice patches that run against their neighbours and a shell that is inside
//! out; without it the network is taken as it is, so this pass settles both before the record is
//! assembled. Faces are oriented by `u × v`; flipping a patch transposes its net, which turns its
//! corners `[c0, c1, c2, c3]` into `[c0, c3, c2, c1]` and its normal around.
//!
//! 1. Every edge shared by two patches must be traversed in opposite directions by them. The
//!    patches that disagree are flipped (a parity union-find over the shared edges, linear in the
//!    patch count); a network where no flipping can make every edge agree is not orientable and is
//!    refused. Re-orienting is deterministic: within a connected component the patch of the
//!    lowest index keeps its orientation.
//! 2. A component that is closed (every edge used twice) must enclose a positive volume: the
//!    signed volume of the polyhedron through the control nets (closed when the boundary rows
//!    agree, which the builder checks) decides, and a negative component is flipped as a whole.
use crate::{KernelError, Result};
use std::collections::HashMap;

/// The corners of patch `corners` after a flip.
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
        // from the root downwards: parity against the root is the parity to the parent plus the parent's
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

/// The side `s` of a patch as a directed pair of layout vertices.
fn side(c: [u32; 4], s: usize) -> (u32, u32) {
    (c[s], c[(s + 1) % 4])
}

/// Which patches must be flipped so that every shared edge is traversed in opposite directions
/// (relative to the lowest patch of each component), and each component's root.
fn consistent(corners: &[[u32; 4]]) -> Result<(ParityForest, Vec<(usize, usize)>)> {
    let mut forest = ParityForest::new(corners.len());
    let mut seen: HashMap<(u32, u32), (usize, bool)> = HashMap::with_capacity(corners.len() * 2);
    let mut open: Vec<(usize, usize)> = Vec::new();
    for (f, c) in corners.iter().enumerate() {
        for s in 0..4 {
            let (a, b) = side(*c, s);
            let key = (a.min(b), a.max(b));
            let forward = a < b;
            match seen.get(&key) {
                Some(&(g, other)) => {
                    if !forest.union(g, f, other == forward) {
                        return Err(KernelError::Argument(
                            "the patches cannot be oriented consistently (a twisted or non-orientable network)".into(),
                        ));
                    }
                }
                None => {
                    seen.insert(key, (f, forward));
                    open.push((f, s));
                }
            }
        }
    }
    Ok((forest, open))
}

/// The signed volume (times six) of the polyhedron through the control net of one patch,
/// positive when `u × v` points out of the enclosed region.
fn signed_volume6(net: &[Vec<[f64; 3]>]) -> f64 {
    let at = |i: usize, j: usize| net.get(i).and_then(|r| r.get(j)).copied().unwrap_or([0.0; 3]);
    let det = |a: [f64; 3], b: [f64; 3], c: [f64; 3]| {
        a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])
    };
    let n = net.len().saturating_sub(1);
    let mut sum = 0.0;
    for i in 0..n {
        for j in 0..n {
            let (p00, p10, p11, p01) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
            sum += det(p00, p10, p11) + det(p00, p11, p01);
        }
    }
    sum
}

/// For each patch: must it be flipped? `Err` for a network that cannot be oriented.
pub(crate) fn flips(patches: &[Vec<Vec<[f64; 3]>>], corners: &[[u32; 4]]) -> Result<Vec<bool>> {
    let (mut forest, first_uses) = consistent(corners)?;
    let nf = corners.len();
    // components with an edge that has a single use are open: no outward direction
    let mut uses: HashMap<(u32, u32), u8> = HashMap::with_capacity(nf * 2);
    for c in corners {
        for s in 0..4 {
            let (a, b) = side(*c, s);
            let n = uses.entry((a.min(b), a.max(b))).or_insert(0);
            *n = n.saturating_add(1);
        }
    }
    let mut open = vec![false; nf];
    for (f, s) in first_uses {
        let (a, b) = corners.get(f).map_or((0, 0), |c| side(*c, s));
        if uses.get(&(a.min(b), a.max(b))).copied() != Some(2) {
            let (root, _) = forest.find(f);
            if let Some(o) = open.get_mut(root) {
                *o = true;
            }
        }
    }
    let mut volume = vec![0.0_f64; nf];
    let mut parity = vec![false; nf];
    for (f, net) in patches.iter().enumerate() {
        let (root, p) = forest.find(f);
        if let Some(slot) = parity.get_mut(f) {
            *slot = p;
        }
        if let Some(v) = volume.get_mut(root) {
            *v += if p { -signed_volume6(net) } else { signed_volume6(net) };
        }
    }
    Ok((0..nf)
        .map(|f| {
            let (root, _) = forest.find(f);
            let inside_out = !open.get(root).copied().unwrap_or(true) && volume.get(root).copied().unwrap_or(0.0) < 0.0;
            parity.get(f).copied().unwrap_or(false) ^ inside_out
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mobius_strip_is_not_orientable() {
        let strip = [[0, 1, 4, 3], [1, 2, 5, 4], [2, 3, 0, 5]];
        assert!(consistent(&strip).is_err());
    }

    #[test]
    fn a_flipped_neighbour_is_found() {
        // two quads sharing the edge 1-2, the second written the wrong way round
        let corners = [[0, 1, 2, 3], [1, 2, 5, 4]];
        let (mut forest, _) = consistent(&corners).expect("orientable");
        let (_, p0) = forest.find(0);
        let (_, p1) = forest.find(1);
        assert_ne!(p0, p1);
    }
}

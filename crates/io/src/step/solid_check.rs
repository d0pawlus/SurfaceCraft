//! Topology checks before a shell is written as a CLOSED_SHELL: a MANIFOLD_SOLID_BREP's shell is
//! connected and traverses every edge exactly twice, once in each direction. A record that is
//! not that is not written as a solid (it is `Unsupported`, so the solidcore exporter, which
//! heals or reports, gets its chance); a record that points outside itself is an error.

use super::{bad, unsupported};
use crate::IoError;
use std::collections::HashMap;
use surfacecraft_kernel::record::brep::{Brep, Loop, OrientedFace};

/// How often an edge is traversed in each direction by the shell's faces, as the writer writes them.
#[derive(Clone, Copy, Default)]
struct Traversals {
    along: u32,
    against: u32,
    /// Position (in the shell) of the first face that uses the edge.
    first_face: usize,
}

fn find(parent: &mut [usize], mut x: usize) -> usize {
    while let Some(&p) = parent.get(x) {
        if p == x {
            break;
        }
        let grand = parent.get(p).copied().unwrap_or(p);
        if let Some(slot) = parent.get_mut(x) {
            *slot = grand;
        }
        x = grand;
    }
    x
}

/// Checks that `faces` form one connected shell with every edge traversed once each way.
pub(super) fn check_closed_shell(b: &Brep, faces: &[OrientedFace]) -> Result<(), IoError> {
    let mut edges: HashMap<u32, Traversals> = HashMap::new();
    let mut parent: Vec<usize> = (0..faces.len()).collect();
    for (pos, of) in faces.iter().enumerate() {
        let face = b.faces.get(of.face as usize).ok_or_else(|| bad(format!("face {} missing", of.face)))?;
        for l in &face.loops {
            let Loop::Edges { edges: oriented } = l else { continue };
            for u in oriented {
                if b.edges.get(u.edge as usize).is_none() {
                    return Err(bad(format!("edge {} missing", u.edge)));
                }
                let t = edges.entry(u.edge).or_insert(Traversals { first_face: pos, ..Traversals::default() });
                if u.forward == of.forward {
                    t.along += 1;
                } else {
                    t.against += 1;
                }
                let (a, c) = (find(&mut parent, pos), find(&mut parent, t.first_face));
                if let Some(slot) = parent.get_mut(a) {
                    *slot = c;
                }
            }
        }
    }
    if let Some((e, _)) = edges.iter().find(|(_, t)| t.along != 1 || t.against != 1) {
        return Err(unsupported(format!(
            "STEP solid export of a shell that is not closed and consistently oriented (edge {e})"
        )));
    }
    let root = find(&mut parent, 0);
    if (0..faces.len()).any(|i| find(&mut parent, i) != root) {
        return Err(unsupported("STEP solid export of a shell of several disjoint components"));
    }
    Ok(())
}

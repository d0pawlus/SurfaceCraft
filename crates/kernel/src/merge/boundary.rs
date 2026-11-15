//! The boundary of a group of faces: its one outer loop, and the edges that go.
use crate::hybrid::V;
use crate::record::brep::{Brep, Loop, OrientedEdge, Shell};
use crate::{KernelError, Result};
use std::collections::{BTreeMap, BTreeSet};

/// Samples per edge when the distance of a boundary curve from the surface is measured.
const EDGE_SAMPLES: usize = 24;

/// One edge of the merged face's loop, as the patches traverse it (counter-clockwise seen from
/// their outward side).
#[derive(Clone, Copy)]
pub(super) struct LoopEdge {
    pub edge: u32,
    pub forward: bool,
}

fn refuse(what: &str) -> KernelError {
    KernelError::Argument(what.to_string())
}

/// The shell a face is used by, and whether it is used forward there.
fn shell_use(body: &Brep, face: usize) -> Option<(usize, bool)> {
    body.shells.iter().enumerate().find_map(|(i, s)| match s {
        Shell::Faces { faces, .. } => faces.iter().find(|o| o.face as usize == face).map(|o| (i, o.forward)),
        _ => None,
    })
}

/// The loop of `face` walked counter-clockwise seen from the side the shell faces out of.
fn outward_loop(body: &Brep, face: usize) -> Result<Vec<LoopEdge>> {
    let f = body.faces.get(face).ok_or_else(|| refuse("a face index is out of range"))?;
    let (_, forward) = shell_use(body, face).ok_or_else(|| refuse("a face belongs to no face shell"))?;
    if f.loops.len() != 1 {
        return Err(refuse("a patch has holes or no loop"));
    }
    let Some(Loop::Edges { edges }) = f.loops.first() else {
        return Err(refuse("a patch loop is not an edge loop"));
    };
    let walk = |e: &OrientedEdge| LoopEdge { edge: e.edge, forward: e.forward == forward };
    Ok(if forward { edges.iter().map(walk).collect() } else { edges.iter().rev().map(walk).collect() })
}

/// The boundary loop of the faces in `group` and the shell they lie in.
pub(super) struct Boundary {
    /// The closed loops of the boundary (the outer one and the holes, in no order).
    pub loops: Vec<Vec<LoopEdge>>,
    pub shell: usize,
}

/// The edges of the boundary of the faces in `group` as the faces walk them, and the shell.
fn outer_edges(body: &Brep, group: &[usize]) -> Result<(Vec<LoopEdge>, usize)> {
    let members: BTreeSet<usize> = group.iter().copied().collect();
    if members.is_empty() || members.len() != group.len() {
        return Err(refuse("a group needs distinct faces"));
    }
    let shells: BTreeSet<usize> = members.iter().filter_map(|&f| shell_use(body, f).map(|s| s.0)).collect();
    let (Some(&shell), 1) = (shells.first(), shells.len()) else {
        return Err(refuse("the faces of a group must lie in one face shell"));
    };
    let mut outer: Vec<LoopEdge> = Vec::new();
    for &f in &members {
        for le in outward_loop(body, f)? {
            let e = body.edges.get(le.edge as usize).ok_or_else(|| refuse("an edge index is out of range"))?;
            let inside = e.uses.iter().filter(|u| members.contains(&(u.face as usize))).count();
            let shared = e.uses.len() >= 2 && inside == e.uses.len();
            if shared && e.uses.len() > 2 {
                return Err(refuse("an interior edge is shared by more than two faces"));
            }
            if shared {
                if e.uses.iter().all(|u| u.face == e.uses.first().map_or(0, |x| x.face)) {
                    return Err(refuse("a face shares an edge with itself (a seam)"));
                }
            } else {
                outer.push(le);
            }
        }
    }
    Ok((outer, shell))
}

/// # Errors
/// An empty or repeated or out-of-range group, faces of several shells, an edge shared with
/// itself, or a boundary that is not made of simple loops (a pinch).
pub(super) fn boundary(body: &Brep, group: &[usize]) -> Result<Boundary> {
    let (outer, shell) = outer_edges(body, group)?;
    if super::samples::connected_groups(body, group).len() != 1 {
        return Err(refuse("the faces of a group must touch along edges"));
    }
    Ok(Boundary { loops: chains(body, &outer)?, shell })
}

/// The vertices at which the boundary of `group` touches itself (two lobes of the group meet there).
pub(super) fn pinch_vertices(body: &Brep, group: &[usize]) -> Result<BTreeSet<u32>> {
    let (outer, _) = outer_edges(body, group)?;
    let mut seen = BTreeSet::new();
    let mut pinches = BTreeSet::new();
    for le in outer {
        if let Some(v) = start_vertex(body, le)
            && !seen.insert(v)
        {
            pinches.insert(v);
        }
    }
    Ok(pinches)
}

fn start_vertex(body: &Brep, le: LoopEdge) -> Option<u32> {
    let e = body.edges.get(le.edge as usize)?;
    Some(if le.forward { e.vertices[0] } else { e.vertices[1] })
}

fn end_vertex(body: &Brep, le: LoopEdge) -> Option<u32> {
    start_vertex(body, LoopEdge { forward: !le.forward, ..le })
}

/// Chains the boundary edges into closed loops, the one that holds the lowest edge first.
fn chains(body: &Brep, outer: &[LoopEdge]) -> Result<Vec<Vec<LoopEdge>>> {
    let mut by_start: BTreeMap<u32, LoopEdge> = BTreeMap::new();
    for &le in outer {
        let s = start_vertex(body, le).ok_or_else(|| refuse("an edge index is out of range"))?;
        if by_start.insert(s, le).is_some() {
            return Err(refuse("the group's boundary touches itself (a pinch)"));
        }
    }
    let mut left: BTreeSet<(u32, bool)> = outer.iter().map(|e| (e.edge, e.forward)).collect();
    let mut loops = Vec::new();
    while let Some(&(edge, forward)) = left.first() {
        let first = LoopEdge { edge, forward };
        let mut cycle = vec![first];
        let mut at = end_vertex(body, first).ok_or_else(|| refuse("an edge index is out of range"))?;
        loop {
            let Some(&next) = by_start.get(&at) else { return Err(refuse("the group's boundary is not closed")) };
            if next.edge == first.edge && next.forward == first.forward {
                break;
            }
            if cycle.len() > outer.len() {
                return Err(refuse("the group's boundary does not close"));
            }
            cycle.push(next);
            at = end_vertex(body, next).ok_or_else(|| refuse("an edge index is out of range"))?;
        }
        for e in &cycle {
            left.remove(&(e.edge, e.forward));
        }
        loops.push(cycle);
    }
    if loops.is_empty() {
        return Err(refuse("the group has no boundary"));
    }
    Ok(loops)
}

/// Points along an edge (its end vertices included), evaluated by the kernel.
pub(super) fn edge_points(body: &Brep, edge: u32) -> Result<Vec<V>> {
    let e = body.edges.get(edge as usize).ok_or_else(|| refuse("an edge index is out of range"))?;
    let curve = solidcore_convert::curve(e.nurbs.as_ref().unwrap_or(&e.curve)).map_err(|e| refuse(&e.to_string()))?;
    let eval = solidcore::eval::CurveEval::new(&curve).map_err(|e| refuse(&format!("{e:?}")))?;
    let [lo, hi] = e.interval;
    (0..=EDGE_SAMPLES)
        .map(|i| eval.point(lo + (hi - lo) * i as f64 / EDGE_SAMPLES as f64).map(|p| [p.x, p.y, p.z]))
        .collect::<Option<Vec<V>>>()
        .ok_or_else(|| refuse("a boundary edge cannot be evaluated"))
}

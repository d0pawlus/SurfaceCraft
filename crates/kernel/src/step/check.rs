//! What a record read from a STEP file, or about to be written to one, must pass: a bounded size,
//! the record's structural and geometric checks (`solidcore_ops::record_check::validate_built`),
//! and shells whose faces agree on their sides (every edge of a closed shell used twice, in
//! opposite directions; no edge of any shell used more than twice or twice the same way). The
//! structural checks cannot see a face turned the wrong way round: the edge directions can.
//!
//! Before a file is read, the control points its faces would hold are estimated from the file: a
//! large spline surface that many faces share is copied into each face's record, so a small file
//! could otherwise ask for a great deal of memory.
use crate::record::brep::{Brep, Curve, Loop, Shell, Surface};
use solidcore_step::p21::{Entity, Exchange, Param};
use std::collections::HashMap;

/// Most items (vertices, edges, edge uses, faces, loops, control points) of one record: the
/// record's own input budget (4 million) refuses real closed shells of tens of thousands of
/// patches, so the application's bound is larger, and a record past it is refused at import
/// rather than accepted and unusable later.
pub(super) const MAX_ITEMS: usize = 64_000_000;

/// Most control points the faces of one file may hold, counting a shared surface once per face.
pub(super) const MAX_FACE_POINTS: usize = 64_000_000;

/// Deepest chain of surfaces followed to a face's spline (a trimmed surface of a trimmed one).
const MAX_SURFACE_DEPTH: usize = 4;

fn curve_points(c: &Curve) -> usize {
    match c {
        Curve::Spline { points, .. } | Curve::Polyline { points } => points.len(),
        Curve::RationalSpline { homogeneous, .. } => homogeneous.len(),
        Curve::SurfaceIntersection { surfaces, leader, .. } => {
            surfaces.iter().map(|s| surface_points(s)).sum::<usize>() + curve_points(leader)
        }
        _ => 1,
    }
}

fn surface_points(s: &Surface) -> usize {
    match s {
        Surface::Spline { points, .. } => points.iter().map(Vec::len).sum(),
        Surface::RationalSpline { homogeneous, .. } => homogeneous.iter().map(Vec::len).sum(),
        Surface::PlacedRevolution { profile, .. } => curve_points(profile),
        _ => 1,
    }
}

/// The items of `b`, counted as its record's budget counts them (approximately).
pub(super) fn items(b: &Brep) -> usize {
    let mut n = b.regions.len() + b.shells.len() + b.vertices.len();
    for e in &b.edges {
        n += 1 + e.uses.len() + curve_points(&e.curve) + e.nurbs.as_ref().map_or(0, curve_points);
    }
    for f in &b.faces {
        n += 1 + surface_points(&f.surface) + f.nurbs.as_ref().map_or(0, surface_points);
        for l in &f.loops {
            n += 1 + match l {
                Loop::Edges { edges } => edges.len(),
                Loop::Vertex { .. } => 1,
            };
        }
    }
    n
}

/// Do the faces of every shell agree on their sides? Every edge use's direction is taken with
/// its face's side in the shell; an edge used twice must be used in opposite directions, no edge
/// more than twice, and every edge of a closed shell (one that bounds a region) twice.
pub(super) fn shells_agree(b: &Brep) -> Result<(), String> {
    let bounding: Vec<bool> = (0..b.shells.len())
        .map(|s| b.regions.iter().skip(1).any(|r| r.shells.iter().any(|&x| x as usize == s)))
        .collect();
    for (si, shell) in b.shells.iter().enumerate() {
        let Shell::Faces { faces, .. } = shell else { continue };
        // per edge: how many uses run against it and along it, with the face's side
        let mut uses: HashMap<u32, [u32; 2]> = HashMap::new();
        for of in faces {
            let face = b.faces.get(of.face as usize).ok_or_else(|| format!("shell {si}: face {} missing", of.face))?;
            for l in &face.loops {
                let Loop::Edges { edges } = l else { continue };
                for u in edges {
                    let slot = uses.entry(u.edge).or_default();
                    slot[usize::from(u.forward == of.forward)] += 1;
                }
            }
        }
        let closed = bounding.get(si).copied().unwrap_or(false);
        let mut sorted: Vec<(u32, [u32; 2])> = uses.into_iter().collect();
        sorted.sort_unstable_by_key(|(e, _)| *e);
        for (edge, [against, along]) in sorted {
            if along > 1 || against > 1 {
                return Err(format!(
                    "shell {si}: edge {edge} is used {} times, {} of them in one direction (a face turned the wrong way round, or more than two faces on one edge)",
                    along + against,
                    along.max(against)
                ));
            }
            if closed && along + against != 2 {
                return Err(format!("shell {si} bounds a region, but its edge {edge} has one face only"));
            }
        }
    }
    Ok(())
}

/// Every check a record must pass to be read from or written to a file.
pub(super) fn record(b: &Brep) -> Result<(), String> {
    let n = items(b);
    if n > MAX_ITEMS {
        return Err(format!("{n} items, more than the {MAX_ITEMS} a body may have"));
    }
    solidcore_ops::record_check::validate_built(b).map_err(|e| e.to_string())?;
    shells_agree(b)
}

/// That the record's geometry agrees with its topology (`solidcore_ops::record_check::agreement`:
/// each edge meets its vertices, each pcurve lifts onto its edge): what is written must say what
/// the body is, so a record whose edges have left their faces is refused rather than written.
pub(super) fn agreement(b: &Brep) -> Result<(), String> {
    solidcore_ops::record_check::agreement(b)
        .map_err(|m| format!("the record's geometry does not agree with its topology: {m:?}"))
}

/// The first parameter of `e` that is a list of lists (a spline surface's control points), and
/// how many entries it holds.
fn grid_size(e: &Entity) -> usize {
    for r in &e.records {
        for p in &r.params {
            if let Some(rows) = p.as_list()
                && rows.first().is_some_and(|row| row.as_list().is_some())
            {
                return rows.iter().filter_map(Param::as_list).map(<[Param]>::len).sum();
            }
        }
    }
    0
}

/// The control points of the surface entity `id` (1 for a surface without them), following a
/// trimmed or offset surface to its basis.
fn surface_size(ex: &Exchange, id: u64, memo: &mut HashMap<u64, usize>) -> usize {
    if let Some(&n) = memo.get(&id) {
        return n;
    }
    let mut at = id;
    let mut n = 1;
    for _ in 0..MAX_SURFACE_DEPTH {
        let Some(e) = ex.get(at) else { break };
        if e.records.iter().any(|r| r.name.starts_with("B_SPLINE_SURFACE")) {
            n = grid_size(e).max(1);
            break;
        }
        // a trimmed, offset or rectangular composite surface names its basis first
        let Some(next) = e.params().iter().find_map(Param::as_ref_id) else { break };
        at = next;
    }
    memo.insert(id, n);
    n
}

/// The control points the faces of the file would hold, a surface counted once for each face on
/// it.
pub(super) fn face_points(ex: &Exchange) -> usize {
    let mut memo = HashMap::new();
    let mut total = 0usize;
    for e in ex.entities.values() {
        if !matches!(e.name(), "ADVANCED_FACE" | "FACE_SURFACE") {
            continue;
        }
        if let Some(surface) = e.params().get(2).and_then(Param::as_ref_id) {
            total = total.saturating_add(surface_size(ex, surface, &mut memo));
        }
    }
    total
}

#[cfg(test)]
mod tests {
    use super::*;

    fn boxed() -> Brep {
        crate::solid::make_box([0.0; 3], [2.0, 3.0, 4.0]).unwrap()
    }

    #[test]
    fn a_kernel_box_passes() {
        let b = boxed();
        assert!(record(&b).is_ok());
        assert!(items(&b) > b.faces.len() + b.edges.len());
    }

    #[test]
    fn a_face_turned_the_wrong_way_round_is_refused() {
        let mut b = boxed();
        if let Some(Shell::Faces { faces, .. }) = b.shells.first_mut() {
            faces[0].forward = !faces[0].forward;
        }
        let e = shells_agree(&b).unwrap_err();
        assert!(e.contains("one direction"), "{e}");
    }

    #[test]
    fn a_closed_shell_with_a_face_missing_is_refused() {
        let mut b = boxed();
        if let Some(Shell::Faces { faces, .. }) = b.shells.first_mut() {
            faces.pop();
        }
        let e = shells_agree(&b).unwrap_err();
        assert!(e.contains("one face only"), "{e}");
    }

    #[test]
    fn an_edge_of_three_faces_is_refused() {
        let mut b = boxed();
        let target = match b.faces[1].loops.first() {
            Some(Loop::Edges { edges }) => edges[0].edge,
            _ => panic!("edge loop"),
        };
        if let Some(Loop::Edges { edges }) = b.faces[0].loops.first_mut() {
            edges[0].edge = target;
        }
        assert!(shells_agree(&b).is_err());
    }

    #[test]
    fn a_shared_spline_surface_counts_once_per_face() {
        let mut text = String::from("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n");
        let rows: Vec<String> = (0..8)
            .map(|i| format!("({})", (0..8).map(|j| format!("#{}", 10 + i * 8 + j)).collect::<Vec<_>>().join(",")))
            .collect();
        text += &format!(
            "#2=B_SPLINE_SURFACE_WITH_KNOTS('',3,3,({}),.UNSPECIFIED.,.F.,.F.,.F.,(4,4),(4,4),(0.,1.),(0.,1.),.UNSPECIFIED.);\n",
            rows.join(",")
        );
        for k in 0..64 {
            text += &format!("#{}=CARTESIAN_POINT('',({}.,0.,0.));\n", 10 + k, k);
        }
        text += "#3=RECTANGULAR_TRIMMED_SURFACE('',#2,0.,1.,0.,1.,.T.,.T.);\n";
        for f in 0..5 {
            text += &format!("#{}=ADVANCED_FACE('',(),#{},.T.);\n", 1000 + f, if f == 4 { 3 } else { 2 });
        }
        text += "ENDSEC;\nEND-ISO-10303-21;\n";
        let ex = solidcore_step::p21::parse(&text).unwrap();
        assert_eq!(face_points(&ex), 5 * 64);
    }
}

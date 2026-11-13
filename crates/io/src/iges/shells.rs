//! Shells of the IGES file: loops of a face in (u, v), their order, and the parameters of a shell (514).

use super::curves::*;
use super::format::*;
use super::surfaces::*;
use super::writer::*;
use super::*;

/// Twice the signed area of a loop in (u, v), from its pcurves' control polygons: positive for a
/// counter-clockwise (outer) loop.
fn loop_area(uses: &[OrientedEdge]) -> f64 {
    let mut pts: Vec<[f64; 2]> = Vec::new();
    for u in uses {
        let mut p = pcurve_points(&u.pcurve);
        if !u.forward {
            p.reverse();
        }
        pts.extend(p);
    }
    let n = pts.len();
    (0..n)
        .filter_map(|i| {
            let (a, b) = (pts.get(i)?, pts.get((i + 1) % n)?);
            Some(a[0] * b[1] - b[0] * a[1])
        })
        .sum()
}

/// The order loops are written in and whether the first is the outer one: a lone edge loop is
/// outer; among several, the one loop that runs counter-clockwise in (u, v).
fn loop_order(face: &Face) -> (Vec<usize>, bool) {
    let n = face.loops.len();
    if n == 1 {
        return (vec![0], matches!(face.loops.first(), Some(Loop::Edges { .. })));
    }
    let ccw: Vec<usize> = face
        .loops
        .iter()
        .enumerate()
        .filter_map(|(i, l)| match l {
            Loop::Edges { edges } if loop_area(edges) > 0.0 => Some(i),
            _ => None,
        })
        .collect();
    match ccw.as_slice() {
        [outer] => (std::iter::once(*outer).chain((0..n).filter(|i| i != outer)).collect(), true),
        _ => ((0..n).collect(), false),
    }
}

/// One loop entry: `(is_vertex, list index, along)`.
type Use = (bool, usize, bool);

/// The parameters of a shell (514): its faces, with their vertex list, edge list and loops
/// written first.
pub(super) fn shell(w: &mut W, b: &Brep, si: usize) -> Result<Vec<String>, IoError> {
    let Some(Shell::Faces { faces, .. }) = b.shells.get(si) else {
        return Err(bad(format!("shell {si} is not a face shell")));
    };
    if faces.is_empty() {
        return Err(bad(format!("shell {si} has no faces")));
    }
    // Vertices and edges of the shell, in first-use order.
    let (mut vidx, mut verts): (HashMap<u32, usize>, Vec<V3>) = (HashMap::new(), Vec::new());
    let (mut eidx, mut edges): (HashMap<u32, usize>, Vec<u32>) = (HashMap::new(), Vec::new());
    let vertex = |v: u32, vidx: &mut HashMap<u32, usize>, verts: &mut Vec<V3>| -> Result<usize, IoError> {
        if let Some(k) = vidx.get(&v) {
            return Ok(*k);
        }
        let p = b.vertices.get(v as usize).ok_or_else(|| bad(format!("vertex {v} missing")))?.point;
        vidx.insert(v, verts.len());
        verts.push(p);
        Ok(verts.len() - 1)
    };
    let mut face_data = Vec::with_capacity(faces.len());
    for of in faces {
        let face = b.faces.get(of.face as usize).ok_or_else(|| bad(format!("face {} missing", of.face)))?;
        if face.loops.is_empty() {
            return Err(bad(format!("face {} has no loops", of.face)));
        }
        let (order, has_outer) = loop_order(face);
        let mut loops: Vec<Vec<Use>> = Vec::with_capacity(order.len());
        for li in order {
            match face.loops.get(li) {
                Some(Loop::Edges { edges: uses }) => {
                    let mut l = Vec::with_capacity(uses.len());
                    for u in uses {
                        let k = match eidx.get(&u.edge) {
                            Some(k) => *k,
                            None => {
                                let e = b
                                    .edges
                                    .get(u.edge as usize)
                                    .ok_or_else(|| bad(format!("edge {} missing", u.edge)))?;
                                for v in e.vertices {
                                    vertex(v, &mut vidx, &mut verts)?;
                                }
                                eidx.insert(u.edge, edges.len());
                                edges.push(u.edge);
                                edges.len() - 1
                            }
                        };
                        l.push((false, k, u.forward));
                    }
                    if l.is_empty() {
                        return Err(bad(format!("face {}: empty loop", of.face)));
                    }
                    loops.push(l);
                }
                Some(Loop::Vertex { vertex: v }) => loops.push(vec![(true, vertex(*v, &mut vidx, &mut verts)?, true)]),
                None => return Err(bad(format!("face {}: loop {li} missing", of.face))),
            }
        }
        face_data.push((of.face, face, of.forward, has_outer, loops));
    }
    let mut vp = vec![verts.len().to_string()];
    for v in &verts {
        vp.extend(w.coords(*v)?);
    }
    let vlist = w.add(502, 1, vp);
    // Edges run along their curves, from the record edge's start (interval min) to its end.
    let mut ep = vec![edges.len().to_string()];
    for &e in &edges {
        let edge = b.edges.get(e as usize).ok_or_else(|| bad(format!("edge {e} missing")))?;
        let lookup = |v: u32| vidx.get(&v).copied().ok_or_else(|| bad(format!("edge {e}: vertex {v} not listed")));
        let (sv, tv) = (lookup(edge.vertices[0])?, lookup(edge.vertices[1])?);
        let point = |k: usize| verts.get(k).copied().ok_or_else(|| bad(format!("edge {e}: vertex missing")));
        let (pa, pb) = (point(sv)?, point(tv)?);
        let cd = curve(w, e, edge, pa, pb, sv == tv)?;
        ep.extend([cd.to_string(), vlist.to_string(), (sv + 1).to_string(), vlist.to_string(), (tv + 1).to_string()]);
    }
    let elist = w.add(504, 1, ep);
    let mut sp = vec![face_data.len().to_string()];
    for (fi, face, forward, has_outer, loops) in face_data {
        let sd = surface(w, fi, face)?;
        let mut lids = Vec::with_capacity(loops.len());
        for l in loops {
            let mut lp = vec![l.len().to_string()];
            for (is_vertex, k, along) in l {
                let (kind, list) = if is_vertex { ("1", vlist) } else { ("0", elist) };
                lp.extend([
                    kind.into(),
                    list.to_string(),
                    (k + 1).to_string(),
                    if along { "1" } else { "0" }.into(),
                    "0".into(),
                ]);
            }
            lids.push(w.add(508, 1, lp).to_string());
        }
        let mut fp = vec![sd.to_string(), lids.len().to_string(), if has_outer { "1" } else { "0" }.into()];
        fp.extend(lids);
        let fd = w.add(510, 1, fp);
        // The record's `forward`: the face's normal in the shell is its surface's natural normal.
        sp.extend([fd.to_string(), if forward { "1" } else { "0" }.into()]);
    }
    Ok(sp)
}

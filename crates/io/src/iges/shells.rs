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

/// The vertices and edges of a shell in first-use order, with their positions in those lists.
#[derive(Default)]
struct ShellLists {
    vidx: HashMap<u32, usize>,
    verts: Vec<V3>,
    eidx: HashMap<u32, usize>,
    edges: Vec<u32>,
}

impl ShellLists {
    /// The position of vertex `v` in the list, adding it on first use.
    fn vertex(&mut self, b: &Brep, v: u32) -> Result<usize, IoError> {
        if let Some(k) = self.vidx.get(&v) {
            return Ok(*k);
        }
        let p = b.vertices.get(v as usize).ok_or_else(|| bad(format!("vertex {v} missing")))?.point;
        self.vidx.insert(v, self.verts.len());
        self.verts.push(p);
        Ok(self.verts.len() - 1)
    }

    /// The position of the edge of use `u` in the list, adding it and its vertices on first use.
    fn edge(&mut self, b: &Brep, u: &OrientedEdge) -> Result<usize, IoError> {
        if let Some(k) = self.eidx.get(&u.edge) {
            return Ok(*k);
        }
        let e = b.edges.get(u.edge as usize).ok_or_else(|| bad(format!("edge {} missing", u.edge)))?;
        for v in e.vertices {
            self.vertex(b, v)?;
        }
        self.eidx.insert(u.edge, self.edges.len());
        self.edges.push(u.edge);
        Ok(self.edges.len() - 1)
    }
}

/// A face of a shell with its loops resolved to list positions.
struct FaceData<'a> {
    index: u32,
    face: &'a Face,
    forward: bool,
    has_outer: bool,
    loops: Vec<Vec<Use>>,
}

/// The entries of loop `li` of face `fi`, adding what they use to `lists`.
fn collect_loop(b: &Brep, lists: &mut ShellLists, fi: u32, face: &Face, li: usize) -> Result<Vec<Use>, IoError> {
    match face.loops.get(li) {
        Some(Loop::Edges { edges: uses }) => {
            let mut l = Vec::with_capacity(uses.len());
            for u in uses {
                let k = lists.edge(b, u)?;
                l.push((false, k, u.forward));
            }
            if l.is_empty() {
                return Err(bad(format!("face {fi}: empty loop")));
            }
            Ok(l)
        }
        Some(Loop::Vertex { vertex: v }) => Ok(vec![(true, lists.vertex(b, *v)?, true)]),
        None => Err(bad(format!("face {fi}: loop {li} missing"))),
    }
}

/// Face `fi` of the record with its loops in writing order, adding what they use to `lists`.
fn collect_face<'a>(b: &'a Brep, lists: &mut ShellLists, fi: u32, forward: bool) -> Result<FaceData<'a>, IoError> {
    let face = b.faces.get(fi as usize).ok_or_else(|| bad(format!("face {fi} missing")))?;
    if face.loops.is_empty() {
        return Err(bad(format!("face {fi} has no loops")));
    }
    let (order, has_outer) = loop_order(face);
    let mut loops: Vec<Vec<Use>> = Vec::with_capacity(order.len());
    for li in order {
        loops.push(collect_loop(b, lists, fi, face, li)?);
    }
    Ok(FaceData { index: fi, face, forward, has_outer, loops })
}

/// The vertex list (502) of a shell; the DE pointer.
fn write_vertex_list(w: &mut W, verts: &[V3]) -> Result<i64, IoError> {
    let mut vp = vec![verts.len().to_string()];
    for v in verts {
        vp.extend(w.coords(*v)?);
    }
    Ok(w.add(502, 1, vp))
}

/// The curve (written now) and the end vertices of edge `e` as the fields of an edge list entry.
fn edge_entry(w: &mut W, b: &Brep, lists: &ShellLists, e: u32, vlist: i64) -> Result<[String; 5], IoError> {
    let edge = b.edges.get(e as usize).ok_or_else(|| bad(format!("edge {e} missing")))?;
    let lookup = |v: u32| lists.vidx.get(&v).copied().ok_or_else(|| bad(format!("edge {e}: vertex {v} not listed")));
    let (sv, tv) = (lookup(edge.vertices[0])?, lookup(edge.vertices[1])?);
    let point = |k: usize| lists.verts.get(k).copied().ok_or_else(|| bad(format!("edge {e}: vertex missing")));
    let (pa, pb) = (point(sv)?, point(tv)?);
    let cd = curve(w, e, edge, pa, pb, sv == tv)?;
    Ok([cd.to_string(), vlist.to_string(), (sv + 1).to_string(), vlist.to_string(), (tv + 1).to_string()])
}

/// The edge list (504) of a shell; the DE pointer. Edges run along their curves, from the record
/// edge's start (interval min) to its end.
fn write_edge_list(w: &mut W, b: &Brep, lists: &ShellLists, vlist: i64) -> Result<i64, IoError> {
    let mut ep = vec![lists.edges.len().to_string()];
    for &e in &lists.edges {
        ep.extend(edge_entry(w, b, lists, e, vlist)?);
    }
    Ok(w.add(504, 1, ep))
}

/// The parameters of a loop (508) from its entries in the vertex and edge lists.
fn loop_params(l: Vec<Use>, vlist: i64, elist: i64) -> Vec<String> {
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
    lp
}

/// Writes a face's surface, loops and the face (510); its DE pointer and orientation as the
/// fields of a shell entry.
fn write_face(w: &mut W, fd: FaceData<'_>, vlist: i64, elist: i64) -> Result<[String; 2], IoError> {
    let sd = surface(w, fd.index, fd.face)?;
    let mut lids = Vec::with_capacity(fd.loops.len());
    for l in fd.loops {
        lids.push(w.add(508, 1, loop_params(l, vlist, elist)).to_string());
    }
    let mut fp = vec![sd.to_string(), lids.len().to_string(), if fd.has_outer { "1" } else { "0" }.into()];
    fp.extend(lids);
    let fde = w.add(510, 1, fp);
    // The record's `forward`: the face's normal in the shell is its surface's natural normal.
    Ok([fde.to_string(), if fd.forward { "1" } else { "0" }.into()])
}

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
    let mut lists = ShellLists::default();
    let mut face_data = Vec::with_capacity(faces.len());
    for of in faces {
        face_data.push(collect_face(b, &mut lists, of.face, of.forward)?);
    }
    let vlist = write_vertex_list(w, &lists.verts)?;
    let elist = write_edge_list(w, b, &lists, vlist)?;
    let mut sp = vec![face_data.len().to_string()];
    for fd in face_data {
        sp.extend(write_face(w, fd, vlist, elist)?);
    }
    Ok(sp)
}

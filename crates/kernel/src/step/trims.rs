//! The kernel's pcurves for the edge uses of a face that have no closed form (a spline edge on a
//! cylinder, an intersection curve on a spline surface): the face goes to the kernel alone as a
//! one-face sheet record whose other uses keep their pcurves, the uses without one go "bare"
//! (`solidcore_convert::brep::to_kernel_with_bare_uses`), and the kernel makes their trims as it
//! makes every trim it needs when the body is read back (`to_record`). The pcurves are the
//! kernel's; nothing here fits a curve. (SolidCraft's reader takes the same route through its own
//! wrapper of the kernel.)
use crate::record::brep::{
    Brep, Curve2, Edge, EdgeUseRef, Face, Loop, OrientedEdge, OrientedFace, Region, Shell, VERSION, Vertex,
};
use crate::{KernelError, Result, op_err};
use solidcore::kernel::{Kernel, KernelOptions};
use solidcore_ops::pcurve::{TrimEdge, TrimFace};

fn refuse(what: impl Into<String>) -> KernelError {
    KernelError::Argument(what.into())
}

fn index(i: usize) -> Result<u32> {
    u32::try_from(i).map_err(|_| refuse("too many items"))
}

/// The face as a one-face open sheet in canonical order (edges by first use along the loops,
/// vertices by the edges' start and end), each TrimFace edge's index in it, and the uses that go
/// without a pcurve as `(face, loop, position)`, sorted.
struct Sheet {
    record: Brep,
    edge_of: Vec<Option<u32>>,
    bare: Vec<(u32, u32, u32)>,
}

fn sheet(face: &TrimFace<'_>) -> Result<Sheet> {
    let mut edge_of: Vec<Option<u32>> = vec![None; face.edges.len()];
    let mut vertex_of: Vec<Option<u32>> = vec![None; face.vertices.len()];
    let mut edges: Vec<Edge> = Vec::new();
    let mut vertices: Vec<Vertex> = Vec::new();
    let mut bare = Vec::new();
    let mut loops = Vec::with_capacity(face.loops.len());
    for (li, l) in face.loops.iter().enumerate() {
        let li = index(li)?;
        let mut uses = Vec::with_capacity(l.len());
        for (p, u) in l.iter().enumerate() {
            let p = index(p)?;
            let src: &TrimEdge<'_> = face.edges.get(u.edge as usize).ok_or_else(|| refuse("edge reference"))?;
            let slot = edge_of.get_mut(u.edge as usize).ok_or_else(|| refuse("edge reference"))?;
            let e = match *slot {
                Some(e) => e,
                None => {
                    let e = index(edges.len())?;
                    *slot = Some(e);
                    let mut ends = [0u32; 2];
                    for (end, v) in ends.iter_mut().zip(src.vertices) {
                        let vs = vertex_of.get_mut(v as usize).ok_or_else(|| refuse("vertex reference"))?;
                        *end = match *vs {
                            Some(i) => i,
                            None => {
                                let i = index(vertices.len())?;
                                let point = *face.vertices.get(v as usize).ok_or_else(|| refuse("vertex reference"))?;
                                let given = face.vertex_tolerances.get(v as usize).copied().unwrap_or(0.0);
                                vertices.push(Vertex { point, tolerance: given.max(face.tolerance) });
                                *vs = Some(i);
                                i
                            }
                        };
                    }
                    edges.push(Edge {
                        curve: src.curve.clone(),
                        nurbs: src.nurbs.cloned(),
                        interval: src.interval,
                        vertices: ends,
                        tolerance: src.tolerance,
                        uses: Vec::new(),
                    });
                    e
                }
            };
            let edge = edges.get_mut(e as usize).ok_or_else(|| refuse("edge reference"))?;
            edge.uses.push(EdgeUseRef { face: 0, r#loop: li, position: p, upper: !u.forward });
            let pcurve = match &u.pcurve {
                Some(pc) => pc.clone(),
                None => {
                    bare.push((0, li, p));
                    // a placeholder, never read: the use goes bare
                    let [t0, t1] = edge.interval;
                    Curve2::Spline { knots: vec![t0, t0, t1, t1], points: vec![[0.0; 2]; 2] }
                }
            };
            uses.push(OrientedEdge { edge: e, forward: u.forward, pcurve });
        }
        loops.push(Loop::Edges { edges: uses });
    }
    let record = Brep {
        version: VERSION,
        tolerance: face.tolerance,
        regions: vec![Region { shells: vec![0], void: false }],
        shells: vec![Shell::Faces { faces: vec![OrientedFace { face: 0, forward: true }], wires: Vec::new() }],
        faces: vec![Face { surface: face.surface.clone(), nurbs: face.nurbs.cloned(), domain: face.domain, loops }],
        edges,
        vertices,
    };
    Ok(Sheet { record, edge_of, bare })
}

/// A pcurve the kernel made on its edge's interval `theirs`, on ours (`edge.interval`): the same
/// curve when the intervals agree; otherwise moved by the affine change of parameter (exact for a
/// B-spline: its knots move) when each end the kernel chose lies within the edge's tolerance of
/// ours along the edge's curve. Farther is the kernel trimming the edge, refused with how far.
fn on_our_interval(pc: Curve2, edge: &TrimEdge<'_>, theirs: [f64; 2]) -> Result<Curve2> {
    let [a, b] = edge.interval;
    let [c, d] = theirs;
    if a == c && b == d {
        return Ok(pc);
    }
    let eval = crate::pcurve::curve_eval(edge.nurbs.unwrap_or(edge.curve))?;
    let at = |t: f64| crate::pcurve::point_on(&eval, t);
    let moved = [at(a)?.distance(at(c)?), at(b)?.distance(at(d)?)];
    if !(d > c) || moved.iter().any(|m| !(*m <= edge.tolerance)) {
        return Err(refuse(format!(
            "the kernel moved an edge's interval from {:?} to {theirs:?}, its ends {moved:?} mm along its curve (tolerance {} mm)",
            edge.interval, edge.tolerance
        )));
    }
    let map = |k: f64| {
        if k == c {
            a
        } else if k == d {
            b
        } else {
            a + (k - c) * ((b - a) / (d - c))
        }
    };
    Ok(match pc {
        Curve2::Spline { knots, points } => Curve2::Spline { knots: knots.into_iter().map(map).collect(), points },
        Curve2::RationalSpline { knots, homogeneous } => {
            Curve2::RationalSpline { knots: knots.into_iter().map(map).collect(), homogeneous }
        }
    })
}

/// Every use's pcurve of `face`, the kernel's where the face has none.
pub(super) fn kernel_trims(face: &TrimFace<'_>) -> Result<Vec<Vec<Curve2>>> {
    let Sheet { record, edge_of, bare } = sheet(face)?;
    let mut k = Kernel::new(&KernelOptions::default());
    let (mut body, into) = solidcore_convert::brep::to_kernel_with_bare_uses(&mut k, &record, true, &bare)
        .map_err(|e| op_err("to_kernel_with_bare_uses", e))?;
    let (out, from) = solidcore_convert::brep::to_record(&mut k, &mut body).map_err(|e| op_err("to_record", e))?;
    let [made] = out.faces.as_slice() else {
        return Err(refuse(format!("the kernel made {} faces of one", out.faces.len())));
    };
    let made_uses: Vec<&OrientedEdge> = made
        .loops
        .iter()
        .flat_map(|l| match l {
            Loop::Edges { edges } => edges.as_slice(),
            Loop::Vertex { .. } => &[],
        })
        .collect();
    let back = from.record_edges();
    // the record edge the kernel gave each of our edges
    let out_edge = |e: u32| -> Result<u32> {
        let ours = edge_of.get(e as usize).copied().flatten().ok_or_else(|| refuse("an edge no loop uses"))?;
        let kernel = into.edges.get(ours as usize).ok_or_else(|| refuse("an edge the kernel lost"))?;
        back.get(kernel).copied().ok_or_else(|| refuse("an edge the kernel lost"))
    };
    // A sheet's loops may come back the way the kernel runs them, against ours: a use's pcurve is
    // the same either way (on its edge's interval in its curve's direction); the way round is read
    // from an edge the face uses once.
    let ours = face.loops.iter().flatten();
    let once = ours
        .clone()
        .find(|u| ours.clone().filter(|v| v.edge == u.edge).count() == 1)
        .ok_or_else(|| refuse("a face of seam edges only"))?;
    let at = out_edge(once.edge)?;
    let reversed =
        made_uses.iter().find(|m| m.edge == at).ok_or_else(|| refuse("a use the kernel did not keep"))?.forward
            != once.forward;
    let mut result = Vec::with_capacity(face.loops.len());
    for l in &face.loops {
        let mut pcs = Vec::with_capacity(l.len());
        for u in l {
            let e = out_edge(u.edge)?;
            let theirs = out.edges.get(e as usize).ok_or_else(|| refuse("an edge the kernel lost"))?.interval;
            let edge = face.edges.get(u.edge as usize).ok_or_else(|| refuse("edge reference"))?;
            let pc = match &u.pcurve {
                Some(given) => given.clone(),
                None => {
                    let found = made_uses.iter().find(|m| m.edge == e && m.forward == (u.forward != reversed));
                    let pc = found.ok_or_else(|| refuse("a use the kernel did not keep"))?.pcurve.clone();
                    on_our_interval(pc, edge, theirs)?
                }
            };
            pcs.push(pc);
        }
        result.push(pcs);
    }
    Ok(result)
}

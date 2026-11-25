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
use std::collections::HashMap;

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

/// The work one import may hand the kernel for trims, as the control points it projects: each
/// use without a pcurve costs its edge's curve and its face's surface (the kernel projects the one
/// onto the other). Past it the remaining faces that need trims are refused (their bodies with
/// them), so a hostile file cannot make the import run unbounded (it has no cancellation).
pub(super) const MAX_TRIM_COST: usize = 64_000_000;

/// Faces one kernel makes trims for before a fresh one replaces it: the kernel memoises its curve
/// analyses by content and never forgets them, so its memory would grow with the import.
const FACES_PER_KERNEL: usize = 2000;

/// The kernel that makes the trims of one import, and the work done so far.
pub(super) struct TrimKernel {
    kernel: Kernel,
    faces: usize,
    cost: usize,
}

impl TrimKernel {
    pub(super) fn new() -> Self {
        Self { kernel: Kernel::new(&KernelOptions::default()), faces: 0, cost: 0 }
    }

    #[cfg(test)]
    pub(super) fn cost(&self) -> usize {
        self.cost
    }

    /// Every use's pcurve of `face`, the kernel's where the face has none.
    pub(super) fn trims(&mut self, face: &TrimFace<'_>) -> Result<Vec<Vec<Curve2>>> {
        self.cost = self.cost.saturating_add(cost_of(face));
        if self.cost > MAX_TRIM_COST {
            return Err(refuse(format!(
                "the kernel's trims of this file would project more than {MAX_TRIM_COST} control points; the rest is refused"
            )));
        }
        self.faces += 1;
        if self.faces.is_multiple_of(FACES_PER_KERNEL) {
            self.kernel = Kernel::new(&KernelOptions::default());
        }
        kernel_trims(&mut self.kernel, face)
    }
}

/// The control points the kernel projects for `face`: for each use without a pcurve, its edge's
/// curve and the face's surface.
fn cost_of(face: &TrimFace<'_>) -> usize {
    let surface = super::check::surface_points(face.nurbs.unwrap_or(face.surface));
    face.loops
        .iter()
        .flatten()
        .filter(|u| u.pcurve.is_none())
        .map(|u| {
            let curve =
                face.edges.get(u.edge as usize).map_or(1, |e| super::check::curve_points(e.nurbs.unwrap_or(e.curve)));
            curve.saturating_add(surface)
        })
        .fold(0usize, usize::saturating_add)
}

/// Every use's pcurve of `face`, the kernel's where the face has none.
fn kernel_trims(k: &mut Kernel, face: &TrimFace<'_>) -> Result<Vec<Vec<Curve2>>> {
    let Sheet { record, edge_of, bare } = sheet(face)?;
    let (mut body, into) = solidcore_convert::brep::to_kernel_with_bare_uses(k, &record, true, &bare)
        .map_err(|e| op_err("to_kernel_with_bare_uses", e))?;
    let (out, from) = solidcore_convert::brep::to_record(k, &mut body).map_err(|e| op_err("to_record", e))?;
    let [made] = out.faces.as_slice() else {
        return Err(refuse(format!("the kernel made {} faces of one", out.faces.len())));
    };
    // each use the kernel made, by its edge and direction (a seam's two uses run opposite ways)
    let mut made_uses: HashMap<(u32, bool), &Curve2> = HashMap::new();
    for l in &made.loops {
        if let Loop::Edges { edges } = l {
            for m in edges {
                made_uses.entry((m.edge, m.forward)).or_insert(&m.pcurve);
            }
        }
    }
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
    let mut uses_of: HashMap<u32, usize> = HashMap::new();
    for u in face.loops.iter().flatten() {
        *uses_of.entry(u.edge).or_default() += 1;
    }
    let once = face
        .loops
        .iter()
        .flatten()
        .find(|u| uses_of.get(&u.edge) == Some(&1))
        .ok_or_else(|| refuse("a face of seam edges only"))?;
    let at = out_edge(once.edge)?;
    let reversed = if made_uses.contains_key(&(at, once.forward)) {
        false
    } else if made_uses.contains_key(&(at, !once.forward)) {
        true
    } else {
        return Err(refuse("a use the kernel did not keep"));
    };
    let mut result = Vec::with_capacity(face.loops.len());
    for l in &face.loops {
        let mut pcs = Vec::with_capacity(l.len());
        for u in l {
            let pc = match &u.pcurve {
                Some(given) => given.clone(),
                None => {
                    let e = out_edge(u.edge)?;
                    let theirs = out.edges.get(e as usize).ok_or_else(|| refuse("an edge the kernel lost"))?.interval;
                    let edge = face.edges.get(u.edge as usize).ok_or_else(|| refuse("edge reference"))?;
                    let found = made_uses.get(&(e, u.forward != reversed));
                    let pc = (*found.ok_or_else(|| refuse("a use the kernel did not keep"))?).clone();
                    on_our_interval(pc, edge, theirs)?
                }
            };
            pcs.push(pc);
        }
        result.push(pcs);
    }
    Ok(result)
}

#[cfg(test)]
mod tests;

//! Persistent topology naming (design doc §6.4, stage P5).
//!
//! A feature that names an edge or a face by its position in the record (`edges: [7]`) points at
//! another edge as soon as an earlier feature changes the topology (a through hole made blind,
//! a fillet added upstream, a sketch edited into a different number of loops). Alongside the
//! index a feature therefore keeps a geometric *signature* of every reference it names: edge
//! midpoint, end points, tangent, curve kind and the kinds and normals of the two adjacent
//! faces; face kind, normal (or axis) and centroid. Positions are normalised to the body's
//! bounding box, so scaling the part keeps them stable. On replay each signature is resolved
//! against the rebuilt body by the best weighted score; a reference no candidate matches closely
//! enough is an error (the feature fails, nothing panics).
//!
//! Solidcore's `provenance` module tracks origins through kernel attributes, which do not
//! survive our record round-trips (every operation decodes the record and encodes the result),
//! so the signature is computed from the record itself.
use crate::record::brep::{Brep, Curve, Loop, Shell, Surface};
use crate::{KernelError, Result};
use serde::{Deserialize, Serialize};

type V3 = [f64; 3];

/// A reference whose best candidate scores above this is lost (see [`resolve_edges`]).
pub const MAX_COST: f64 = 1.5;

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}
fn dist(a: V3, b: V3) -> f64 {
    norm(sub(a, b))
}
fn unit(a: V3) -> V3 {
    let n = norm(a);
    if n.is_finite() && n > 1e-12 { [a[0] / n, a[1] / n, a[2] / n] } else { [0.0; 3] }
}
fn mid(a: V3, b: V3) -> V3 {
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0]
}
fn axis_of(f: &crate::record::brep::Frame) -> V3 {
    unit(cross(f.x_axis, f.y_axis))
}

/// The geometric signature of a face (positions normalised to the body's bounding box).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FaceSig {
    /// Surface kind (`plane`, `cylinder`, `cone`, `sphere`, `torus`, `spline`, `revolution`).
    pub kind: String,
    /// Outward unit normal of a plane, unit axis of a cylinder, cone or torus (zero otherwise).
    pub normal: [f64; 3],
    /// Centroid (on the axis for a surface of revolution), normalised.
    pub centroid: [f64; 3],
}

/// The geometric signature of an edge (positions normalised to the body's bounding box).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EdgeSig {
    /// Curve kind (`line`, `circle`, `ellipse`, `polyline`, `spline`, `intersection`).
    pub curve: String,
    /// Start point, normalised.
    pub start: [f64; 3],
    /// End point, normalised.
    pub end: [f64; 3],
    /// Midpoint (circle centre for a closed edge), normalised.
    pub mid: [f64; 3],
    /// Unit direction start → end (the axis of a circle; zero for a closed edge).
    pub tangent: [f64; 3],
    /// Signatures of the faces on either side (at most two).
    pub faces: Vec<FaceSig>,
}

struct Ctx<'a> {
    b: &'a Brep,
    min: V3,
    ext: V3,
    forward: Vec<bool>,
}

impl<'a> Ctx<'a> {
    fn new(b: &'a Brep) -> Ctx<'a> {
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for v in &b.vertices {
            for k in 0..3 {
                min[k] = min[k].min(v.point[k]);
                max[k] = max[k].max(v.point[k]);
            }
        }
        let mut ext = [0.0; 3];
        for k in 0..3 {
            if min[k].is_finite() && max[k].is_finite() {
                ext[k] = max[k] - min[k];
            } else {
                min[k] = 0.0;
            }
        }
        let mut forward = vec![true; b.faces.len()];
        for s in &b.shells {
            if let Shell::Faces { faces, .. } = s {
                for of in faces {
                    if let Some(f) = forward.get_mut(of.face as usize) {
                        *f = of.forward;
                    }
                }
            }
        }
        Ctx { b, min, ext, forward }
    }

    fn norm_pt(&self, p: V3) -> V3 {
        let mut o = [0.5; 3];
        for k in 0..3 {
            if self.ext[k] > 1e-9 {
                o[k] = (p[k] - self.min[k]) / self.ext[k];
            }
        }
        o
    }

    fn vertex(&self, i: u32) -> Option<V3> {
        self.b.vertices.get(i as usize).map(|v| v.point)
    }

    /// Midpoint of an edge in model space (the centre of a full circle).
    fn edge_mid(&self, e: usize) -> Option<V3> {
        let edge = self.b.edges.get(e)?;
        let (p0, p1) = (self.vertex(edge.vertices[0])?, self.vertex(edge.vertices[1])?);
        if edge.vertices[0] == edge.vertices[1]
            && let Curve::Circle { frame, .. } = &edge.curve
        {
            return Some(frame.origin);
        }
        Some(mid(p0, p1))
    }

    fn face_sig(&self, f: usize) -> Option<FaceSig> {
        let face = self.b.faces.get(f)?;
        let fwd = self.forward.get(f).copied().unwrap_or(true);
        // (kind, signed normal, axis (point, direction) if the surface is one of revolution)
        let (kind, normal, axis): (&str, V3, Option<(V3, V3)>) = match &face.surface {
            Surface::Plane { u_axis, v_axis, .. } => {
                let n = unit(cross(*u_axis, *v_axis));
                ("plane", if fwd { n } else { [-n[0], -n[1], -n[2]] }, None)
            }
            Surface::PlaneThrough { origin, u_point, v_point } => {
                let n = unit(cross(sub(*u_point, *origin), sub(*v_point, *origin)));
                ("plane", if fwd { n } else { [-n[0], -n[1], -n[2]] }, None)
            }
            Surface::Cylinder { frame, .. } => ("cylinder", axis_of(frame), Some((frame.origin, axis_of(frame)))),
            Surface::Cone { frame, .. } => ("cone", axis_of(frame), Some((frame.origin, axis_of(frame)))),
            Surface::Torus { frame, .. } => ("torus", axis_of(frame), Some((frame.origin, axis_of(frame)))),
            Surface::Sphere { frame, .. } => ("sphere", [0.0; 3], Some((frame.origin, [0.0; 3]))),
            Surface::BallPatch { center, .. } => ("sphere", [0.0; 3], Some((*center, [0.0; 3]))),
            Surface::PlacedRevolution { origin, axis, .. } => ("revolution", unit(*axis), Some((*origin, unit(*axis)))),
            Surface::Spline { .. } | Surface::RationalSpline { .. } => ("spline", [0.0; 3], None),
        };
        // centroid: mean of the distinct edges of the outer loop
        let mut pts: Vec<V3> = Vec::new();
        if let Some(Loop::Edges { edges }) = face.loops.first() {
            let mut seen: Vec<u32> = Vec::new();
            for oe in edges {
                if seen.contains(&oe.edge) {
                    continue;
                }
                seen.push(oe.edge);
                if let Some(m) = self.edge_mid(oe.edge as usize) {
                    pts.push(m);
                }
            }
        }
        if pts.is_empty() {
            for v in &self.b.vertices {
                pts.push(v.point);
            }
        }
        let n = pts.len().max(1) as f64;
        let mut c = [0.0; 3];
        for p in &pts {
            for k in 0..3 {
                c[k] += p[k] / n;
            }
        }
        if let Some((o, d)) = axis {
            c = if norm(d) > 0.5 {
                let t = dot(sub(c, o), d);
                [o[0] + d[0] * t, o[1] + d[1] * t, o[2] + d[2] * t]
            } else {
                o
            };
        }
        Some(FaceSig { kind: kind.to_string(), normal, centroid: self.norm_pt(c) })
    }

    fn edge_sig(&self, i: usize) -> Option<EdgeSig> {
        let e = self.b.edges.get(i)?;
        let (p0, p1) = (self.vertex(e.vertices[0])?, self.vertex(e.vertices[1])?);
        let (curve, tangent) = match &e.curve {
            Curve::Line { .. } => ("line", unit(sub(p1, p0))),
            Curve::Circle { frame, .. } => ("circle", axis_of(frame)),
            Curve::Ellipse { frame, .. } => ("ellipse", axis_of(frame)),
            Curve::Polyline { .. } => ("polyline", unit(sub(p1, p0))),
            Curve::Spline { .. } | Curve::RationalSpline { .. } => ("spline", unit(sub(p1, p0))),
            Curve::SurfaceIntersection { .. } => ("intersection", unit(sub(p1, p0))),
        };
        let mut faces = Vec::new();
        let mut seen: Vec<u32> = Vec::new();
        for u in &e.uses {
            if faces.len() >= 2 {
                break;
            }
            if seen.contains(&u.face) {
                continue;
            }
            seen.push(u.face);
            if let Some(s) = self.face_sig(u.face as usize) {
                faces.push(s);
            }
        }
        Some(EdgeSig {
            curve: curve.to_string(),
            start: self.norm_pt(p0),
            end: self.norm_pt(p1),
            mid: self.norm_pt(self.edge_mid(i)?),
            tangent,
            faces,
        })
    }
}

fn face_cost(a: &FaceSig, b: &FaceSig, w_pos: f64) -> f64 {
    let mut c = w_pos * dist(a.centroid, b.centroid);
    if a.kind != b.kind {
        c += 2.0;
    } else if norm(a.normal) > 0.5 && norm(b.normal) > 0.5 {
        let d = dot(a.normal, b.normal);
        c += 1.5 * if a.kind == "plane" { 1.0 - d } else { 1.0 - d.abs() };
    }
    c
}

fn edge_cost(a: &EdgeSig, b: &EdgeSig) -> f64 {
    let mut c = 4.0 * dist(a.mid, b.mid);
    let straight = dist(a.start, b.start) + dist(a.end, b.end);
    let flipped = dist(a.start, b.end) + dist(a.end, b.start);
    c += straight.min(flipped);
    if norm(a.tangent) > 0.5 && norm(b.tangent) > 0.5 {
        c += 1.0 - dot(a.tangent, b.tangent).abs();
    }
    if a.curve != b.curve {
        c += 1.5;
    }
    if a.faces.len() != b.faces.len() {
        c += 0.5;
    }
    c += 0.5
        * match (a.faces.as_slice(), b.faces.as_slice()) {
            ([a0, a1], [b0, b1]) => {
                (face_cost(a0, b0, 1.0) + face_cost(a1, b1, 1.0)).min(face_cost(a0, b1, 1.0) + face_cost(a1, b0, 1.0))
            }
            ([a0], [b0]) => face_cost(a0, b0, 1.0),
            _ => 0.0,
        };
    c
}

/// The signature of edge `index` of `body` (`None` outside the record).
#[must_use]
pub fn edge_signature(body: &Brep, index: usize) -> Option<EdgeSig> {
    Ctx::new(body).edge_sig(index)
}

/// The signature of face `index` of `body` (`None` outside the record).
#[must_use]
pub fn face_signature(body: &Brep, index: usize) -> Option<FaceSig> {
    Ctx::new(body).face_sig(index)
}

/// Signatures of the edges `indices` (record indices) of `body`.
pub fn capture_edges(body: &Brep, indices: &[usize]) -> Result<Vec<EdgeSig>> {
    let ctx = Ctx::new(body);
    indices
        .iter()
        .map(|&i| {
            ctx.edge_sig(i)
                .ok_or_else(|| KernelError::Argument(format!("edge index {i} is outside 0..{}", body.edges.len())))
        })
        .collect()
}

/// Signatures of the faces `indices` (record indices) of `body`.
pub fn capture_faces(body: &Brep, indices: &[usize]) -> Result<Vec<FaceSig>> {
    let ctx = Ctx::new(body);
    indices
        .iter()
        .map(|&i| {
            ctx.face_sig(i)
                .ok_or_else(|| KernelError::Argument(format!("face index {i} is outside 0..{}", body.faces.len())))
        })
        .collect()
}

fn describe(p: [f64; 3]) -> String {
    format!("[{:.2}, {:.2}, {:.2}]", p[0], p[1], p[2])
}

fn dedup(mut v: Vec<usize>) -> Vec<usize> {
    let mut seen = Vec::with_capacity(v.len());
    v.retain(|i| {
        if seen.contains(i) {
            false
        } else {
            seen.push(*i);
            true
        }
    });
    v
}

/// The edges of `body` that best match `sigs`, in order (duplicates removed). A signature no edge
/// matches within [`MAX_COST`] is an error naming it.
pub fn resolve_edges(body: &Brep, sigs: &[EdgeSig]) -> Result<Vec<usize>> {
    let ctx = Ctx::new(body);
    let cands: Vec<(usize, EdgeSig)> = (0..body.edges.len()).filter_map(|i| ctx.edge_sig(i).map(|s| (i, s))).collect();
    let mut out = Vec::with_capacity(sigs.len());
    for (k, sig) in sigs.iter().enumerate() {
        let best = cands
            .iter()
            .map(|(i, c)| (*i, edge_cost(sig, c)))
            .filter(|(_, c)| c.is_finite())
            .min_by(|a, b| a.1.total_cmp(&b.1));
        match best {
            Some((i, c)) if c <= MAX_COST => out.push(i),
            Some((_, c)) => {
                return Err(KernelError::Argument(format!(
                    "edge reference {k} ({} near {}) matches no edge of the rebuilt body (best score {c:.2} > {MAX_COST})",
                    sig.curve,
                    describe(sig.mid)
                )));
            }
            None => {
                return Err(KernelError::Argument(format!(
                    "edge reference {k} cannot be resolved: the body has no edges"
                )));
            }
        }
    }
    Ok(dedup(out))
}

/// The faces of `body` that best match `sigs`, in order (duplicates removed).
pub fn resolve_faces(body: &Brep, sigs: &[FaceSig]) -> Result<Vec<usize>> {
    let ctx = Ctx::new(body);
    let cands: Vec<(usize, FaceSig)> = (0..body.faces.len()).filter_map(|i| ctx.face_sig(i).map(|s| (i, s))).collect();
    let mut out = Vec::with_capacity(sigs.len());
    for (k, sig) in sigs.iter().enumerate() {
        let best = cands
            .iter()
            .map(|(i, c)| (*i, face_cost(sig, c, 4.0)))
            .filter(|(_, c)| c.is_finite())
            .min_by(|a, b| a.1.total_cmp(&b.1));
        match best {
            Some((i, c)) if c <= MAX_COST => out.push(i),
            Some((_, c)) => {
                return Err(KernelError::Argument(format!(
                    "face reference {k} ({} near {}) matches no face of the rebuilt body (best score {c:.2} > {MAX_COST})",
                    sig.kind,
                    describe(sig.centroid)
                )));
            }
            None => {
                return Err(KernelError::Argument(format!(
                    "face reference {k} cannot be resolved: the body has no faces"
                )));
            }
        }
    }
    Ok(dedup(out))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid::{fillet, make_box};

    /// The record index of the box edge that runs along z at the corner nearest `(x, y)`.
    fn vertical_edge(b: &Brep, x: f64, y: f64) -> usize {
        let ctx = Ctx::new(b);
        (0..b.edges.len())
            .filter_map(|i| ctx.edge_sig(i).map(|s| (i, s)))
            .find(|(_, s)| {
                s.curve == "line"
                    && s.tangent[2].abs() > 0.99
                    && (s.mid[0] - x).abs() < 0.01
                    && (s.mid[1] - y).abs() < 0.01
            })
            .map(|(i, _)| i)
            .unwrap_or(usize::MAX)
    }

    #[test]
    fn signature_follows_the_edge_when_indices_move() {
        let a = make_box([0.0; 3], [20.0, 10.0, 5.0]).unwrap();
        let e = vertical_edge(&a, 0.0, 0.0);
        assert!(e < a.edges.len());
        let sig = capture_edges(&a, &[e]).unwrap();
        // a bigger box with another corner already rounded: other topology, other indices
        let other =
            fillet(&make_box([0.0; 3], [30.0, 12.0, 6.0]).unwrap(), &[vertical_edge(&a, 1.0, 1.0)], 2.0).unwrap();
        assert!(other.edges.len() > a.edges.len());
        let want = vertical_edge(&other, 0.0, 0.0);
        assert!(want < other.edges.len());
        let got = resolve_edges(&other, &sig).unwrap();
        assert_eq!(got, vec![want]);
        // positional indexing would have named something else
        assert_ne!(e, usize::MAX);
        // the same for a face: the top plane
        let top = {
            let ctx = Ctx::new(&a);
            (0..a.faces.len())
                .find(|&f| ctx.face_sig(f).is_some_and(|s| s.kind == "plane" && s.normal[2] > 0.99))
                .unwrap()
        };
        let fs = capture_faces(&a, &[top]).unwrap();
        let ctx = Ctx::new(&other);
        let want_top = (0..other.faces.len())
            .find(|&f| ctx.face_sig(f).is_some_and(|s| s.kind == "plane" && s.normal[2] > 0.99))
            .unwrap();
        assert_eq!(resolve_faces(&other, &fs).unwrap(), vec![want_top]);
    }

    #[test]
    fn a_lost_reference_is_an_error_not_a_panic() {
        let a = make_box([0.0; 3], [20.0, 10.0, 5.0]).unwrap();
        let mut sig = capture_edges(&a, &[0]).unwrap();
        sig[0].mid = [0.5, 0.5, 0.5];
        sig[0].start = [0.5, 0.5, 0.5];
        sig[0].end = [0.5, 0.5, 0.5];
        sig[0].curve = "circle".into();
        assert!(resolve_edges(&a, &sig).is_err());
        sig[0].mid = [f64::NAN; 3];
        assert!(resolve_edges(&a, &sig).is_err());
        assert!(capture_edges(&a, &[99]).is_err());
        assert!(capture_faces(&a, &[99]).is_err());
        assert!(
            resolve_faces(
                &Brep {
                    version: 4,
                    tolerance: 1e-6,
                    regions: vec![],
                    shells: vec![],
                    faces: vec![],
                    edges: vec![],
                    vertices: vec![]
                },
                &[]
            )
            .unwrap()
            .is_empty()
        );
    }
}

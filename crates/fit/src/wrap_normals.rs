//! Creases and normals of the layout of `surface.wrap`, estimated from the scan.
//!
//! The coarse triangle mesh a wrap is laid on is a reduction of the scan; its own normals are too
//! rough to tell a crease from a strongly curved smooth area. Both are therefore taken from the scan:
//! the normal on each side of a coarse edge is a moving-least-squares fit over the scan vertices
//! of that side only (a thin wall's other face and the far side of a crease are not averaged in),
//! and the edge is a *crease* when the two normals differ by more than the crease angle.
//!
//! Around a layout vertex the faces joined by smooth edges form a *sector*; every sector gets one
//! normal (so a vertex on a crease has one normal per side, a vertex of a smooth area one). The
//! normals are what the G1 construction of [`crate::patch_g1`] makes the patches tangent to.
use crate::mls::{ScanSurface, Support};
use std::collections::{BTreeMap, BTreeSet};
use surfacecraft_geom::{Mesh, Point3, Vec3, normalize};

/// Scan vertices whose normal is within about 60 degrees of the hint are used for a normal.
const NORMAL_MIN_COS: f64 = 0.5;
/// Support radius of a normal, as a share of the length of the edges around the point.
const SUPPORT_SHARE: f64 = 0.35;
/// A quadric normal farther than this (degrees) from the plane normal of the same vertices is not trusted.
const MAX_QUADRIC_DEVIATION: f64 = 20.0;
/// Of the support, how far behind the plane of a side (as a share of the radius) a vertex is still used.
const SIDE_SLACK: f64 = 0.1;

/// Coarse edge key: smaller vertex id first.
pub(crate) type EdgeKey = (u32, u32);

pub(crate) fn key(a: u32, b: u32) -> EdgeKey {
    (a.min(b), a.max(b))
}

/// The faces on each edge of `coarse`.
pub(crate) fn edge_faces(coarse: &Mesh) -> BTreeMap<EdgeKey, Vec<usize>> {
    let mut out: BTreeMap<EdgeKey, Vec<usize>> = BTreeMap::new();
    for (f, t) in coarse.triangles.iter().enumerate() {
        for k in 0..3 {
            out.entry(key(t[k], t[(k + 1) % 3])).or_default().push(f);
        }
    }
    out
}

/// Slot `k` of the triangle whose edge `k` runs between `a` and `b`.
pub(crate) fn edge_slot(t: &[u32; 3], a: u32, b: u32) -> Option<usize> {
    (0..3).find(|&k| key(t[k], t[(k + 1) % 3]) == key(a, b))
}

/// What the scan says about the layout.
#[derive(Clone, Debug, Default)]
pub(crate) struct LayoutNormals {
    /// Coarse edges between two faces whose scan normals differ by more than the crease angle.
    pub(crate) creases: BTreeSet<EdgeKey>,
    /// Coarse edges between two faces that are not creases.
    pub(crate) smooth_edges: usize,
    /// Normal of corner `k` of face `f`: that of the sector of the vertex the face lies in.
    pub(crate) corner: Vec<[Vec3; 3]>,
    /// Normal at the midpoint of edge `k` of face `f`, seen from that face.
    pub(crate) mid: Vec<[Vec3; 3]>,
    /// Normal at the centre of face `f`.
    pub(crate) centre: Vec<Vec3>,
}

/// The inputs shared by every estimate.
pub(crate) struct Layout<'a> {
    pub(crate) coarse: &'a Mesh,
    /// Layout vertices on the scan (the coarse vertices snapped to it).
    pub(crate) vertices: &'a [Vec3],
    pub(crate) face_normal: &'a [Vec3],
    pub(crate) surface: &'a ScanSurface,
    pub(crate) edges: &'a BTreeMap<EdgeKey, Vec<usize>>,
}

impl Layout<'_> {
    fn pos(&self, v: u32) -> Vec3 {
        self.vertices.get(v as usize).copied().unwrap_or_else(Vec3::zeros)
    }

    fn centroid(&self, f: usize) -> Vec3 {
        let t = self.coarse.triangles.get(f).copied().unwrap_or([0; 3]);
        (self.pos(t[0]) + self.pos(t[1]) + self.pos(t[2])) / 3.0
    }

    fn normal_of(&self, f: usize) -> Vec3 {
        self.face_normal.get(f).copied().unwrap_or_else(Vec3::z)
    }

    /// Unit direction from the point `at`, perpendicular to the unit `edge`, toward the face `f`.
    fn inward(&self, f: usize, at: Vec3, edge: &Vec3) -> Vec3 {
        let d = self.centroid(f) - at;
        normalize(d - edge * d.dot(edge)).unwrap_or_else(Vec3::zeros)
    }

    /// The plane normal of the scan near `at` that agrees with `hint`, from the vertices `keep` accepts.
    fn plane(&self, at: Vec3, radius: f64, hint: Vec3, keep: &dyn Fn(&Point3) -> bool) -> Option<Vec3> {
        let filter = Support { min_cos: NORMAL_MIN_COS, keep };
        let n = self.surface.plane_normal(&Point3::from(at), radius, &hint, &filter)?;
        (n.dot(&hint) > NORMAL_MIN_COS).then_some(n)
    }

    /// The normal of the scan near `at` that agrees with `hint` (the hint itself where the scan
    /// gives none), from the vertices `keep` accepts: the surface normal of a moving-least-squares
    /// quadric, unless that is far from the plane through the same vertices (a quadric on a thin
    /// support extrapolates wildly), when the plane's is taken.
    fn estimate(&self, at: Vec3, radius: f64, hint: Vec3, keep: &dyn Fn(&Point3) -> bool) -> Vec3 {
        let filter = Support { min_cos: NORMAL_MIN_COS, keep };
        let quadric = self.surface.project_with(&Point3::from(at), radius, &hint, &filter).map(|(_, n)| n);
        let plane = self.plane(at, radius, hint, keep);
        match (quadric, plane) {
            (Some(q), Some(p)) if angle(&q, &p) > MAX_QUADRIC_DEVIATION.to_radians() => p,
            (Some(q), _) if q.dot(&hint) > NORMAL_MIN_COS => q,
            (_, Some(p)) => p,
            _ => hint,
        }
    }

    /// The support of the side of face `f` of the edge `a`-`b`: its middle, radius and the half space.
    fn side_support(&self, f: usize, a: u32, b: u32) -> (Vec3, f64, Vec3) {
        let (pa, pb) = (self.pos(a), self.pos(b));
        let mid = (pa + pb) * 0.5;
        let edge = normalize(pb - pa).unwrap_or_else(Vec3::x);
        (mid, SUPPORT_SHARE * (pb - pa).norm(), self.inward(f, mid, &edge))
    }

    /// The normal at the middle of the edge `a`-`b` seen from its face `f` alone, and the normal of the
    /// plane through that side of the scan (the one angles are judged by).
    fn one_sided(&self, f: usize, a: u32, b: u32) -> (Vec3, Vec3) {
        let (mid, radius, side) = self.side_support(f, a, b);
        let keep = move |p: &Point3| (p.coords - mid).dot(&side) > -SIDE_SLACK * radius;
        let hint = self.normal_of(f);
        let n = self.estimate(mid, radius, hint, &keep);
        (n, self.plane(mid, radius, hint, &keep).unwrap_or(n))
    }

    /// The normal at the middle of the edge `a`-`b` from both sides.
    fn two_sided(&self, faces: &[usize], a: u32, b: u32) -> Vec3 {
        let mid = (self.pos(a) + self.pos(b)) * 0.5;
        let hint = normalize(faces.iter().map(|&f| self.normal_of(f)).sum::<Vec3>()).unwrap_or_else(Vec3::z);
        self.estimate(mid, SUPPORT_SHARE * (self.pos(b) - self.pos(a)).norm(), hint, &|_| true)
    }
}

fn angle(a: &Vec3, b: &Vec3) -> f64 {
    a.dot(b).clamp(-1.0, 1.0).acos()
}

/// Classifies the edges and gives each its midpoint normals: `mid[f][k]` for every face `f` on edge `k`.
fn classify_edges(l: &Layout, crease_rad: f64, out: &mut LayoutNormals) {
    for (&(a, b), faces) in l.edges {
        let slots: Vec<(usize, usize)> =
            faces.iter().filter_map(|&f| Some((f, edge_slot(l.coarse.triangles.get(f)?, a, b)?))).collect();
        let (sides, planes): (Vec<Vec3>, Vec<Vec3>) = faces.iter().map(|&f| l.one_sided(f, a, b)).unzip();
        let crease = matches!(planes.as_slice(), [n1, n2] if angle(n1, n2) > crease_rad);
        let shared = if crease { None } else { Some(l.two_sided(faces, a, b)) };
        if faces.len() == 2 {
            if crease {
                out.creases.insert((a, b));
            } else {
                out.smooth_edges += 1;
            }
        }
        for (i, &(f, k)) in slots.iter().enumerate() {
            if let Some(m) = out.mid.get_mut(f).and_then(|m| m.get_mut(k)) {
                *m = shared.or_else(|| sides.get(i).copied()).unwrap_or_else(Vec3::z);
            }
        }
    }
}

/// An edge of the face `f` at the vertex `v` that bounds a sector (a crease, a free edge): the unit
/// direction perpendicular to it into the face.
fn boundary_sides(l: &Layout, bounds: &BTreeSet<EdgeKey>, v: u32, group: &[usize]) -> Vec<Vec3> {
    let mut out = Vec::new();
    for &f in group {
        let Some(t) = l.coarse.triangles.get(f) else { continue };
        for &u in t.iter().filter(|&&u| u != v) {
            if !bounds.contains(&key(u, v)) {
                continue;
            }
            let edge = normalize(l.pos(u) - l.pos(v)).unwrap_or_else(Vec3::x);
            out.push(l.inward(f, l.pos(v), &edge));
        }
    }
    out
}

/// Edges that divide the faces around a vertex: creases, free edges and edges of three or more faces.
fn sector_bounds(l: &Layout, creases: &BTreeSet<EdgeKey>) -> BTreeSet<EdgeKey> {
    l.edges.iter().filter(|(k, f)| f.len() != 2 || creases.contains(*k)).map(|(k, _)| *k).collect()
}

/// Groups the faces around `v` (listed in `faces`) that are joined by an edge at `v` that is not a bound.
fn sectors(l: &Layout, bounds: &BTreeSet<EdgeKey>, v: u32, faces: &[usize]) -> Vec<Vec<usize>> {
    let mut label: Vec<usize> = (0..faces.len()).collect();
    for (i, &f) in faces.iter().enumerate() {
        for (j, &g) in faces.iter().enumerate().skip(i + 1) {
            let shared =
                l.coarse.triangles.get(f).zip(l.coarse.triangles.get(g)).is_some_and(|(tf, tg)| {
                    tf.iter().any(|&u| u != v && tg.contains(&u) && !bounds.contains(&key(u, v)))
                });
            if shared {
                let (from, to) = (label[j].max(label[i]), label[j].min(label[i]));
                label.iter_mut().filter(|x| **x == from).for_each(|x| *x = to);
            }
        }
    }
    let ids: BTreeSet<usize> = label.iter().copied().collect();
    ids.into_iter().map(|id| faces.iter().zip(&label).filter(|(_, l)| **l == id).map(|(f, _)| *f).collect()).collect()
}

/// One normal per sector of every vertex, stored in `corner[f][k]`.
fn vertex_sectors(l: &Layout, out: &mut LayoutNormals) {
    let mut around: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
    for (f, t) in l.coarse.triangles.iter().enumerate() {
        for &v in t {
            around.entry(v).or_default().push(f);
        }
    }
    let bounds = sector_bounds(l, &out.creases);
    for (v, faces) in around {
        let at = l.pos(v);
        for group in sectors(l, &bounds, v, &faces) {
            let reach: Vec<f64> = group
                .iter()
                .filter_map(|&f| l.coarse.triangles.get(f))
                .flat_map(|t| t.iter().filter(|&&u| u != v).map(|&u| (l.pos(u) - at).norm()))
                .collect();
            let radius = SUPPORT_SHARE * reach.iter().sum::<f64>() / reach.len().max(1) as f64;
            let hint = normalize(group.iter().map(|&f| l.normal_of(f)).sum::<Vec3>()).unwrap_or_else(Vec3::z);
            let walls = boundary_sides(l, &bounds, v, &group);
            let keep = move |p: &Point3| walls.iter().all(|w| (p.coords - at).dot(w) > -SIDE_SLACK * radius);
            let n = l.estimate(at, radius, hint, &keep);
            for &f in &group {
                let slot = l.coarse.triangles.get(f).and_then(|t| t.iter().position(|&u| u == v));
                if let Some(c) = slot.and_then(|k| out.corner.get_mut(f).and_then(|c| c.get_mut(k))) {
                    *c = n;
                }
            }
        }
    }
}

fn face_centres(l: &Layout, out: &mut LayoutNormals) {
    for (f, t) in l.coarse.triangles.iter().enumerate() {
        let at = l.centroid(f);
        let len = (0..3).map(|k| (l.pos(t[k]) - l.pos(t[(k + 1) % 3])).norm()).sum::<f64>() / 3.0;
        if let Some(c) = out.centre.get_mut(f) {
            *c = l.estimate(at, SUPPORT_SHARE * len, l.normal_of(f), &|_| true);
        }
    }
}

/// Classifies every edge of the coarse mesh and estimates the normals the patches are made tangent to.
pub(crate) fn layout_normals(l: &Layout, crease_deg: f64) -> LayoutNormals {
    let faces = l.coarse.triangles.len();
    let mut out = LayoutNormals {
        corner: vec![[Vec3::z(); 3]; faces],
        mid: vec![[Vec3::z(); 3]; faces],
        centre: vec![Vec3::z(); faces],
        ..LayoutNormals::default()
    };
    classify_edges(l, crease_deg.to_radians(), &mut out);
    vertex_sectors(l, &mut out);
    face_centres(l, &mut out);
    out
}

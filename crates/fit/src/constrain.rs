//! Relations between fitted primitives that fix a position or a size, not only a direction:
//! tangency, symmetry about a plane and equal distances, plus the coaxial and equal-radius
//! relations that must keep holding while those are imposed (design doc §6.2).
//!
//! The relations are found within tolerances and then imposed by the smallest change of the
//! parameters that satisfies all of them (the usual approach of constraint recovery in reverse
//! engineering). Once the directions are fixed, every relation here is linear in the changes of
//! the parameters (a signed distance along a fixed direction, a radius, a coordinate), so
//! [`apply`] solves one small linear system per group of coupled primitives: the least-squares
//! change `x = Aᵀ(AAᵀ + λI)⁻¹ r`, refined iteratively. Relations that conflict leave a residual
//! and are reported as a [`Failure`], never forced.
//!
//! Variables: five per primitive, `[dx, dy, dz, d(radius 0), d(radius 1)]` (a shift of the anchor
//! point and the changes of its radii: cylinder and sphere radius; torus major and minor radius).
use crate::primitive::Primitive;
use nalgebra::{Matrix3, SymmetricEigen};
use std::cmp::Ordering;
use std::collections::BTreeMap;
use surfacecraft_geom::{Point3, Vec3, any_perpendicular, normalize};

/// Tangency and equal distance need directions that agree to this many degrees: by then the
/// angular snap has made them exact (or the angular relation was rejected, and then these
/// relations have no meaning).
const EXACT_DEG: f64 = 1e-4;
/// Largest family of parallel primitives searched for equally spaced chains.
const MAX_FAMILY: usize = 200;
/// Longest chain of equally spaced primitives.
const MAX_CHAIN: usize = 64;
/// Most chains reported.
const MAX_CHAINS: usize = 400;
/// Most tangent and symmetric relations reported.
const MAX_RELATIONS: usize = 3000;
/// Most rows in one coupled group (dense Gram matrix of that size).
const MAX_COMPONENT_ROWS: usize = 1200;
/// Most rows in one system.
const MAX_ROWS: usize = 6000;
/// Mirror planes are only searched among at most this many primitives.
const MAX_MIRROR_PRIMS: usize = 300;
/// Most points used for the principal planes.
const MAX_SAMPLE: usize = 20_000;
/// A solution leaves at most this residual (mm) for the relations to count as consistent.
const CONSISTENT: f64 = 1e-7;

/// Kinds of relation [`apply`] can impose.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RelKind {
    /// A plane touches a sphere, a cylinder or a torus tube; two parallel cylinders touch;
    /// a cylinder touches a coaxial torus.
    Tangent,
    /// `a` is the mirror image of `b` about `mirror`.
    Symmetric,
    /// `b` lies halfway between `a` and `c` (planes along their normal; parallel axes).
    EqualDistance,
    /// Parallel axes on one line (carried along so that it keeps holding).
    Coaxial,
    /// Equal radii (carried along so that it keeps holding).
    EqualRadius,
}

/// A relation between primitives, by index into the slice given to the functions here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rel {
    /// Kind.
    pub kind: RelKind,
    /// First primitive.
    pub a: usize,
    /// Second primitive (the middle one of an equal distance).
    pub b: usize,
    /// Third primitive (equal distance only).
    pub c: Option<usize>,
    /// Index of the mirror plane (symmetric only).
    pub mirror: Option<usize>,
}

/// A mirror plane: through `point`, with unit `normal`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mirror {
    /// A point on the plane.
    pub point: Vec3,
    /// Unit normal.
    pub normal: Vec3,
}

impl Mirror {
    /// A mirror through `point` with `normal` (normalised); `None` when the normal is zero.
    #[must_use]
    pub fn new(point: Vec3, normal: Vec3) -> Option<Mirror> {
        normalize(normal).map(|n| Mirror { point, normal: n })
    }

    /// The mirror image of a direction.
    fn dir(&self, d: &Vec3) -> Vec3 {
        d - self.normal * (2.0 * d.dot(&self.normal))
    }

    /// The mirror image of a point (as a vector).
    fn pt(&self, x: &Vec3) -> Vec3 {
        x - self.normal * (2.0 * (x - self.point).dot(&self.normal))
    }
}

/// Why [`apply`] gave up.
#[derive(Clone, Debug, PartialEq)]
pub struct Failure {
    /// What went wrong, for the user.
    pub message: String,
    /// The relation (index into the list given to [`apply`]) that conflicts most, if known.
    pub worst: Option<usize>,
}

// ------------------------------------------------------------------------------------------
// small geometry helpers

fn v3(a: &[f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

fn arr(v: &Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

/// Unit direction of a primitive (normal or axis); none for a sphere.
fn direction(p: &Primitive) -> Option<Vec3> {
    let d = match p {
        Primitive::Plane { normal, .. } => v3(normal),
        Primitive::Cylinder { axis, .. } | Primitive::Cone { axis, .. } | Primitive::Torus { axis, .. } => v3(axis),
        Primitive::Sphere { .. } => return None,
    };
    normalize(d)
}

/// The point that locates the primitive (plane point, sphere or torus centre, axis point, apex).
fn anchor(p: &Primitive) -> Vec3 {
    match p {
        Primitive::Plane { point, .. } | Primitive::Cylinder { point, .. } => v3(point),
        Primitive::Sphere { center, .. } | Primitive::Torus { center, .. } => v3(center),
        Primitive::Cone { apex, .. } => v3(apex),
    }
}

fn move_anchor(p: &mut Primitive, d: &Vec3) {
    let q = arr(&(anchor(p) + d));
    match p {
        Primitive::Plane { point, .. } | Primitive::Cylinder { point, .. } => *point = q,
        Primitive::Sphere { center, .. } | Primitive::Torus { center, .. } => *center = q,
        Primitive::Cone { apex, .. } => *apex = q,
    }
}

fn shift_radii(p: &mut Primitive, d: [f64; 2]) {
    match p {
        Primitive::Cylinder { radius, .. } | Primitive::Sphere { radius, .. } => *radius += d[0],
        Primitive::Torus { major_radius, minor_radius, .. } => {
            *major_radius += d[0];
            *minor_radius += d[1];
        }
        _ => {}
    }
}

fn set_direction(p: &mut Primitive, d: &Vec3) {
    match p {
        Primitive::Plane { normal: v, .. }
        | Primitive::Cylinder { axis: v, .. }
        | Primitive::Cone { axis: v, .. }
        | Primitive::Torus { axis: v, .. } => *v = arr(d),
        Primitive::Sphere { .. } => {}
    }
}

/// Angle between two directions ignoring orientation (degrees, 0..=90).
fn angle_unsigned(a: &Vec3, b: &Vec3) -> f64 {
    a.dot(b).abs().clamp(0.0, 1.0).acos().to_degrees()
}

/// Angle between two oriented directions (degrees, 0..=180).
fn angle_signed(a: &Vec3, b: &Vec3) -> f64 {
    a.dot(b).clamp(-1.0, 1.0).acos().to_degrees()
}

/// Two unit vectors perpendicular to the unit vector `d` and to each other.
fn perp_basis(d: &Vec3) -> [Vec3; 2] {
    let e1 = any_perpendicular(d);
    [e1, d.cross(&e1)]
}

fn xyz() -> [Vec3; 3] {
    [Vec3::x(), Vec3::y(), Vec3::z()]
}

/// Distance of the axes of two axial primitives measured perpendicular to the first axis.
fn axis_separation(pa: &Primitive, pb: &Primitive) -> Option<f64> {
    let da = direction(pa)?;
    let w = anchor(pb) - anchor(pa);
    Some((w - da * w.dot(&da)).norm())
}

// ------------------------------------------------------------------------------------------
// linear rows

/// One linear equation `Σ coef · x = rhs` over the variables of the primitives.
#[derive(Clone, Debug)]
struct Row {
    coef: Vec<(usize, f64)>,
    rhs: f64,
}

fn var(prim: usize, k: usize) -> usize {
    prim * 5 + k
}

fn push_vec(coef: &mut Vec<(usize, f64)>, prim: usize, v: &Vec3, scale: f64) {
    coef.push((var(prim, 0), v.x * scale));
    coef.push((var(prim, 1), v.y * scale));
    coef.push((var(prim, 2), v.z * scale));
}

/// Sorted, duplicate-free coefficients.
fn merge_row(row: &Row) -> Row {
    let mut c = row.coef.clone();
    c.sort_by(|a, b| a.0.cmp(&b.0));
    let mut out: Vec<(usize, f64)> = Vec::with_capacity(c.len());
    for (v, x) in c {
        if out.last().is_some_and(|l| l.0 == v) {
            if let Some(l) = out.last_mut() {
                l.1 += x;
            }
        } else {
            out.push((v, x));
        }
    }
    out.retain(|e| e.1 != 0.0);
    Row { coef: out, rhs: row.rhs }
}

fn radius_row(a: usize, b: usize, slot: usize, ra: f64, rb: f64) -> Row {
    Row { coef: vec![(var(a, 3 + slot), 1.0), (var(b, 3 + slot), -1.0)], rhs: rb - ra }
}

/// The row that makes `e · (mirror(xa + da) − (xb + db)) = 0`.
fn mirror_row(a: usize, b: usize, m: &Mirror, e: &Vec3, xa: &Vec3, xb: &Vec3) -> Row {
    let mut coef = Vec::new();
    push_vec(&mut coef, a, &m.dir(e), 1.0);
    push_vec(&mut coef, b, e, -1.0);
    Row { coef, rhs: e.dot(&(xb - m.pt(xa))) }
}

/// Rows, length error (mm) and angle error (degrees) of a plane tangent to a sphere, a cylinder
/// or a torus tube. The plane is on the side it is on now.
fn plane_round(ip: usize, ir: usize, plane: &Primitive, round: &Primitive) -> Option<(Vec<Row>, f64, f64)> {
    let n = direction(plane)?;
    let sigma = n.dot(&(anchor(round) - anchor(plane)));
    let s = if sigma >= 0.0 { 1.0 } else { -1.0 };
    let (slot, r, angle) = match round {
        Primitive::Sphere { radius, .. } => (0usize, *radius, 0.0),
        Primitive::Cylinder { radius, .. } => (0, *radius, 90.0 - angle_unsigned(&n, &direction(round)?)),
        Primitive::Torus { minor_radius, .. } => (1, *minor_radius, angle_unsigned(&n, &direction(round)?)),
        _ => return None,
    };
    let mut coef = Vec::new();
    push_vec(&mut coef, ir, &n, 1.0);
    push_vec(&mut coef, ip, &n, -1.0);
    coef.push((var(ir, 3 + slot), -s));
    Some((vec![Row { coef, rhs: s * r - sigma }], sigma.abs() - r, angle))
}

/// Two parallel cylinders touching on the outside (distance = sum of radii) or the inside
/// (distance = difference), whichever is nearer.
fn cyl_cyl(ia: usize, ib: usize, pa: &Primitive, pb: &Primitive) -> Option<(Vec<Row>, f64, f64)> {
    let (Primitive::Cylinder { radius: ra, .. }, Primitive::Cylinder { radius: rb, .. }) = (pa, pb) else {
        return None;
    };
    let (ra, rb) = (*ra, *rb);
    let (da, db) = (direction(pa)?, direction(pb)?);
    let w = anchor(pb) - anchor(pa);
    let wp = w - da * w.dot(&da);
    let dist = wp.norm();
    if dist <= 1e-12 {
        return None;
    }
    let u = wp / dist;
    let ext = dist - (ra + rb);
    let int = dist - (ra - rb).abs();
    let mut coef = Vec::new();
    push_vec(&mut coef, ib, &u, 1.0);
    push_vec(&mut coef, ia, &u, -1.0);
    let (gap, rhs) = if ext.abs() <= int.abs() {
        coef.push((var(ia, 3), -1.0));
        coef.push((var(ib, 3), -1.0));
        (ext, ra + rb - dist)
    } else {
        let sign = if ra >= rb { 1.0 } else { -1.0 };
        coef.push((var(ia, 3), -sign));
        coef.push((var(ib, 3), sign));
        (int, sign * (ra - rb) - dist)
    };
    Some((vec![Row { coef, rhs }], gap, angle_unsigned(&da, &db)))
}

/// A cylinder coaxial with a torus whose radius is the torus's outer or inner equator.
fn cyl_torus(ic: usize, it: usize, cyl: &Primitive, tor: &Primitive) -> Option<(Vec<Row>, f64, f64)> {
    let (Primitive::Cylinder { radius: rc, .. }, Primitive::Torus { major_radius: big, minor_radius: small, .. }) =
        (cyl, tor)
    else {
        return None;
    };
    let (rc, big, small) = (*rc, *big, *small);
    let (dc, dt) = (direction(cyl)?, direction(tor)?);
    let w = anchor(tor) - anchor(cyl);
    let off = (w - dc * w.dot(&dc)).norm();
    let outer = rc - (big + small);
    let inner = rc - (big - small);
    let (so, gap) = if outer.abs() <= inner.abs() { (1.0, outer) } else { (-1.0, inner) };
    let mut rows = Vec::new();
    for e in perp_basis(&dc) {
        let mut coef = Vec::new();
        push_vec(&mut coef, it, &e, 1.0);
        push_vec(&mut coef, ic, &e, -1.0);
        rows.push(Row { coef, rhs: -e.dot(&w) });
    }
    rows.push(Row { coef: vec![(var(ic, 3), 1.0), (var(it, 3), -1.0), (var(it, 4), -so)], rhs: big + so * small - rc });
    Some((rows, gap.hypot(off), angle_unsigned(&dc, &dt)))
}

fn tangent_rows(prims: &[Primitive], a: usize, b: usize) -> Option<(Vec<Row>, f64, f64)> {
    let (pa, pb) = (prims.get(a)?, prims.get(b)?);
    match (pa, pb) {
        (Primitive::Plane { .. }, _) => plane_round(a, b, pa, pb),
        (_, Primitive::Plane { .. }) => plane_round(b, a, pb, pa),
        (Primitive::Cylinder { .. }, Primitive::Cylinder { .. }) => cyl_cyl(a, b, pa, pb),
        (Primitive::Cylinder { .. }, Primitive::Torus { .. }) => cyl_torus(a, b, pa, pb),
        (Primitive::Torus { .. }, Primitive::Cylinder { .. }) => cyl_torus(b, a, pb, pa),
        _ => None,
    }
}

fn symmetric_rows(
    prims: &[Primitive],
    mirrors: &[Mirror],
    a: usize,
    b: usize,
    mi: usize,
) -> Option<(Vec<Row>, f64, f64)> {
    let m = mirrors.get(mi)?;
    let (pa, pb) = (prims.get(a)?, prims.get(b)?);
    let (xa, xb) = (anchor(pa), anchor(pb));
    let mut rows: Vec<Row> = Vec::new();
    let (len, angle) = match (pa, pb) {
        (Primitive::Plane { .. }, Primitive::Plane { .. }) => {
            let (na, nb) = (direction(pa)?, direction(pb)?);
            rows.push(mirror_row(a, b, m, &nb, &xa, &xb));
            (nb.dot(&(xb - m.pt(&xa))).abs(), angle_unsigned(&m.dir(&na), &nb))
        }
        (Primitive::Cylinder { radius: ra, .. }, Primitive::Cylinder { radius: rb, .. }) => {
            let (da, db) = (direction(pa)?, direction(pb)?);
            for e in perp_basis(&db) {
                rows.push(mirror_row(a, b, m, &e, &xa, &xb));
            }
            rows.push(radius_row(a, b, 0, *ra, *rb));
            let w = m.pt(&xa) - xb;
            ((w - db * w.dot(&db)).norm().max((ra - rb).abs()), angle_unsigned(&m.dir(&da), &db))
        }
        (Primitive::Sphere { radius: ra, .. }, Primitive::Sphere { radius: rb, .. }) => {
            for e in xyz() {
                rows.push(mirror_row(a, b, m, &e, &xa, &xb));
            }
            rows.push(radius_row(a, b, 0, *ra, *rb));
            ((m.pt(&xa) - xb).norm().max((ra - rb).abs()), 0.0)
        }
        (Primitive::Cone { half_angle: ha, .. }, Primitive::Cone { half_angle: hb, .. }) => {
            let (da, db) = (direction(pa)?, direction(pb)?);
            for e in xyz() {
                rows.push(mirror_row(a, b, m, &e, &xa, &xb));
            }
            ((m.pt(&xa) - xb).norm(), angle_signed(&m.dir(&da), &db).max((ha - hb).abs().to_degrees()))
        }
        (
            Primitive::Torus { major_radius: ma, minor_radius: na, .. },
            Primitive::Torus { major_radius: mb, minor_radius: nb, .. },
        ) => {
            let (da, db) = (direction(pa)?, direction(pb)?);
            for e in xyz() {
                rows.push(mirror_row(a, b, m, &e, &xa, &xb));
            }
            rows.push(radius_row(a, b, 0, *ma, *mb));
            rows.push(radius_row(a, b, 1, *na, *nb));
            let len = (m.pt(&xa) - xb).norm().max((ma - mb).abs()).max((na - nb).abs());
            (len, angle_unsigned(&m.dir(&da), &db))
        }
        _ => return None,
    };
    Some((rows, len, angle))
}

fn equal_distance_rows(prims: &[Primitive], a: usize, b: usize, c: usize) -> Option<(Vec<Row>, f64, f64)> {
    let (pa, pb, pc) = (prims.get(a)?, prims.get(b)?, prims.get(c)?);
    let (da, db, dc) = (direction(pa)?, direction(pb)?, direction(pc)?);
    let plane = matches!(pb, Primitive::Plane { .. });
    if matches!(pa, Primitive::Plane { .. }) != plane || matches!(pc, Primitive::Plane { .. }) != plane {
        return None;
    }
    let angle = angle_unsigned(&da, &db).max(angle_unsigned(&db, &dc));
    let basis: Vec<Vec3> = if plane { vec![db] } else { perp_basis(&db).to_vec() };
    let (xa, xb, xc) = (anchor(pa), anchor(pb), anchor(pc));
    let mut rows = Vec::new();
    let mut second = Vec3::zeros();
    for e in &basis {
        let mut coef = Vec::new();
        push_vec(&mut coef, a, e, 1.0);
        push_vec(&mut coef, b, e, -2.0);
        push_vec(&mut coef, c, e, 1.0);
        let value = e.dot(&xa) - 2.0 * e.dot(&xb) + e.dot(&xc);
        second += e * value;
        rows.push(Row { coef, rhs: -value });
    }
    Some((rows, second.norm(), angle))
}

fn coaxial_rows(prims: &[Primitive], a: usize, b: usize) -> Option<(Vec<Row>, f64, f64)> {
    let (pa, pb) = (prims.get(a)?, prims.get(b)?);
    if matches!(pa, Primitive::Plane { .. }) || matches!(pb, Primitive::Plane { .. }) {
        return None;
    }
    let (da, db) = (direction(pa)?, direction(pb)?);
    let w = anchor(pb) - anchor(pa);
    let mut rows = Vec::new();
    for e in perp_basis(&da) {
        let mut coef = Vec::new();
        push_vec(&mut coef, b, &e, 1.0);
        push_vec(&mut coef, a, &e, -1.0);
        rows.push(Row { coef, rhs: -e.dot(&w) });
    }
    Some((rows, (w - da * w.dot(&da)).norm(), angle_unsigned(&da, &db)))
}

fn equal_radius_rows(prims: &[Primitive], a: usize, b: usize) -> Option<(Vec<Row>, f64, f64)> {
    match (prims.get(a)?, prims.get(b)?) {
        (Primitive::Cylinder { radius: ra, .. }, Primitive::Cylinder { radius: rb, .. })
        | (Primitive::Sphere { radius: ra, .. }, Primitive::Sphere { radius: rb, .. }) => {
            Some((vec![radius_row(a, b, 0, *ra, *rb)], (ra - rb).abs(), 0.0))
        }
        _ => None,
    }
}

fn rows_of(prims: &[Primitive], mirrors: &[Mirror], r: &Rel) -> Option<(Vec<Row>, f64, f64)> {
    match r.kind {
        RelKind::Tangent => tangent_rows(prims, r.a, r.b),
        RelKind::Symmetric => symmetric_rows(prims, mirrors, r.a, r.b, r.mirror?),
        RelKind::EqualDistance => equal_distance_rows(prims, r.a, r.b, r.c?),
        RelKind::Coaxial => coaxial_rows(prims, r.a, r.b),
        RelKind::EqualRadius => equal_radius_rows(prims, r.a, r.b),
    }
}

/// How far the relation is from holding on the primitives as they are: the largest length error
/// (mm) and the largest angle error (degrees). `None` when the primitives do not fit the relation.
#[must_use]
pub fn measure(prims: &[Primitive], mirrors: &[Mirror], r: &Rel) -> Option<(f64, f64)> {
    rows_of(prims, mirrors, r).map(|(_, len, angle)| (len, angle))
}

// ------------------------------------------------------------------------------------------
// finding relations

/// Are the two primitives (of one kind) the same infinite surface within the tolerances?
fn same_surface(pa: &Primitive, pb: &Primitive, angle_tol: f64, length_tol: f64) -> bool {
    let (xa, xb) = (anchor(pa), anchor(pb));
    match (pa, pb) {
        (Primitive::Plane { .. }, Primitive::Plane { .. }) => match (direction(pa), direction(pb)) {
            (Some(na), Some(nb)) => angle_unsigned(&na, &nb) <= angle_tol && nb.dot(&(xa - xb)).abs() <= length_tol,
            _ => false,
        },
        (Primitive::Cylinder { radius: ra, .. }, Primitive::Cylinder { radius: rb, .. }) => {
            match (direction(pa), direction(pb)) {
                (Some(da), Some(db)) => {
                    let w = xa - xb;
                    angle_unsigned(&da, &db) <= angle_tol
                        && (w - db * w.dot(&db)).norm() <= length_tol
                        && (ra - rb).abs() <= length_tol
                }
                _ => false,
            }
        }
        (Primitive::Sphere { radius: ra, .. }, Primitive::Sphere { radius: rb, .. }) => {
            (xa - xb).norm() <= length_tol && (ra - rb).abs() <= length_tol
        }
        (Primitive::Cone { half_angle: ha, .. }, Primitive::Cone { half_angle: hb, .. }) => {
            match (direction(pa), direction(pb)) {
                (Some(da), Some(db)) => {
                    angle_signed(&da, &db) <= angle_tol
                        && (xa - xb).norm() <= length_tol
                        && (ha - hb).abs().to_degrees() <= angle_tol
                }
                _ => false,
            }
        }
        (
            Primitive::Torus { major_radius: ma, minor_radius: na, .. },
            Primitive::Torus { major_radius: mb, minor_radius: nb, .. },
        ) => match (direction(pa), direction(pb)) {
            (Some(da), Some(db)) => {
                angle_unsigned(&da, &db) <= angle_tol
                    && (xa - xb).norm() <= length_tol
                    && (ma - mb).abs() <= length_tol
                    && (na - nb).abs() <= length_tol
            }
            _ => false,
        },
        _ => false,
    }
}

/// Chains of positions with equal steps: `pos[k+1] − pos[k]` is the same within `tol` along the
/// chain. At least three members; sub-chains of a longer chain are left out.
fn chains(pos: &[Vec3], tol: f64) -> Vec<Vec<usize>> {
    let n = pos.len();
    let min_step = (2.0 * tol).max(1e-9);
    let mut found: Vec<(Vec<usize>, f64)> = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let (Some(pi), Some(pj)) = (pos.get(i), pos.get(j)) else { continue };
            if i == j {
                continue;
            }
            let w = pj - pi;
            if w.norm() <= min_step || !canonical(&w) {
                continue;
            }
            // a chain that goes on backwards from `i` starts elsewhere and is found from there
            let back = pi - w;
            if pos.iter().enumerate().any(|(k, q)| k != i && (q - back).norm() <= tol) {
                continue;
            }
            let mut idx = vec![i, j];
            let mut err = 0.0_f64;
            while idx.len() < MAX_CHAIN {
                let next = pi + w * (idx.len() as f64);
                let hit = pos
                    .iter()
                    .enumerate()
                    .filter(|(k, _)| !idx.contains(k))
                    .map(|(k, q)| (k, (q - next).norm()))
                    .filter(|(_, e)| *e <= tol)
                    .min_by(|a, b| a.1.total_cmp(&b.1));
                match hit {
                    Some((k, e)) => {
                        idx.push(k);
                        err = err.max(e);
                    }
                    None => break,
                }
            }
            if idx.len() >= 3 {
                found.push((idx, err));
            }
        }
    }
    found.sort_by(|a, b| b.0.len().cmp(&a.0.len()).then(a.1.total_cmp(&b.1)));
    let mut chosen: Vec<Vec<usize>> = Vec::new();
    for (idx, _) in found {
        let covered = chosen.iter().any(|c| idx.iter().all(|x| c.contains(x)));
        if covered {
            continue;
        }
        chosen.push(idx);
        if chosen.len() >= MAX_CHAINS {
            break;
        }
    }
    chosen
}

/// A step vector is canonical when its first non-zero component is positive: each chain is then
/// found in one direction only.
fn canonical(w: &Vec3) -> bool {
    for c in [w.x, w.y, w.z] {
        if c.abs() > 1e-9 {
            return c > 0.0;
        }
    }
    false
}

/// Equally spaced parallel planes (along the normal) and parallel axes (perpendicular to them):
/// one relation per consecutive triple of a chain.
fn equal_distance_rels(prims: &[Primitive], length_tol: f64) -> Vec<Rel> {
    let mut fams: Vec<(bool, Vec3, Vec<usize>)> = Vec::new();
    for (i, p) in prims.iter().enumerate() {
        let Some(d) = direction(p) else { continue };
        let plane = matches!(p, Primitive::Plane { .. });
        match fams.iter_mut().find(|f| f.0 == plane && angle_unsigned(&f.1, &d) <= EXACT_DEG) {
            Some(f) => f.2.push(i),
            None => fams.push((plane, d, vec![i])),
        }
    }
    let mut out = Vec::new();
    for (plane, d, members) in &fams {
        if members.len() < 3 || members.len() > MAX_FAMILY {
            continue;
        }
        let pos: Vec<(usize, Vec3)> = members
            .iter()
            .filter_map(|&m| {
                let x = anchor(prims.get(m)?);
                let along = d * d.dot(&x);
                Some((m, if *plane { along } else { x - along }))
            })
            .collect();
        let vecs: Vec<Vec3> = pos.iter().map(|p| p.1).collect();
        for chain in chains(&vecs, length_tol) {
            let ids: Vec<usize> = chain.iter().filter_map(|&k| pos.get(k).map(|p| p.0)).collect();
            for w in ids.windows(3) {
                if let &[a, b, c] = w {
                    out.push(Rel { kind: RelKind::EqualDistance, a, b, c: Some(c), mirror: None });
                }
            }
        }
    }
    out
}

/// Tangency, symmetry about each of `mirrors`, and equal distances among `prims`, within
/// `angle_tol` (degrees) and `length_tol` (mm). Tangency and equal distance additionally need
/// exact directions (see [`EXACT_DEG`]): call this on primitives whose parallel and perpendicular
/// relations have been snapped.
#[must_use]
pub fn find_relations(prims: &[Primitive], mirrors: &[Mirror], angle_tol: f64, length_tol: f64) -> Vec<Rel> {
    let n = prims.len();
    let mut out: Vec<Rel> = Vec::new();
    if n > 2000 {
        return out;
    }
    for a in 0..n {
        for b in (a + 1)..n {
            let (Some(pa), Some(pb)) = (prims.get(a), prims.get(b)) else { continue };
            if let Some((_, gap, ang)) = tangent_rows(prims, a, b)
                && gap.abs() <= length_tol
                && ang <= EXACT_DEG
                && !(axis_separation(pa, pb).is_some_and(|s| s <= 2.0 * length_tol)
                    && matches!((pa, pb), (Primitive::Cylinder { .. }, Primitive::Cylinder { .. })))
            {
                out.push(Rel { kind: RelKind::Tangent, a, b, c: None, mirror: None });
            }
            for mi in 0..mirrors.len() {
                let r = Rel { kind: RelKind::Symmetric, a, b, c: None, mirror: Some(mi) };
                if let Some((len, ang)) = measure(prims, mirrors, &r)
                    && len <= length_tol
                    && ang <= angle_tol
                    && !same_surface(pa, pb, angle_tol, length_tol)
                {
                    out.push(r);
                }
            }
            if out.len() >= MAX_RELATIONS {
                return out;
            }
        }
    }
    out.extend(equal_distance_rels(prims, length_tol));
    out
}

/// Unit directions the model is built from: those of the primitives and the cross products of
/// pairs of them, without repeats.
fn frame_directions(prims: &[Primitive]) -> Vec<Vec3> {
    let mut dirs: Vec<Vec3> = Vec::new();
    for p in prims {
        if let Some(d) = direction(p)
            && !dirs.iter().any(|e| angle_unsigned(e, &d) <= 1e-3)
        {
            dirs.push(d);
            if dirs.len() >= 24 {
                break;
            }
        }
    }
    let base = dirs.clone();
    for (i, a) in base.iter().enumerate() {
        for b in base.iter().skip(i + 1) {
            if let Some(c) = normalize(a.cross(b))
                && !dirs.iter().any(|e| angle_unsigned(e, &c) <= 1e-3)
            {
                dirs.push(c);
            }
        }
    }
    dirs
}

/// Does the model look symmetric about `m`? Every primitive must be its own mirror image (on or
/// perpendicular to the plane) or have a partner, with at most one in ten left over, and there
/// must be at least one pair.
fn mirror_supported(prims: &[Primitive], m: &Mirror, angle_tol: f64, length_tol: f64) -> bool {
    let mirrors = [*m];
    let n = prims.len();
    let (mut paired_count, mut unmatched) = (0usize, 0usize);
    for a in 0..n {
        let Some(pa) = prims.get(a) else { continue };
        let own = Rel { kind: RelKind::Symmetric, a, b: a, c: None, mirror: Some(0) };
        let selfsym = measure(prims, &mirrors, &own).is_some_and(|(l, g)| l <= length_tol && g <= angle_tol);
        let paired = (0..n).filter(|&b| b != a).any(|b| {
            let r = Rel { b, ..own };
            prims.get(b).is_some_and(|pb| {
                measure(prims, &mirrors, &r).is_some_and(|(l, g)| l <= length_tol && g <= angle_tol)
                    && !same_surface(pa, pb, angle_tol, length_tol)
            })
        });
        if paired {
            paired_count += 1;
        } else if !selfsym {
            unmatched += 1;
        }
    }
    paired_count >= 2 && unmatched * 10 <= n
}

/// Mirror planes of the model: its principal planes (through the centroid of `points`, or of the
/// primitives' anchors when there are no points), each turned to the nearest direction the model
/// is built from when that is within `angle_tol`, and kept when the model looks symmetric about it.
#[must_use]
pub fn detect_mirrors(prims: &[Primitive], points: &[Point3], angle_tol: f64, length_tol: f64) -> Vec<Mirror> {
    let n = prims.len();
    if !(2..=MAX_MIRROR_PRIMS).contains(&n) {
        return Vec::new();
    }
    let finite = |p: &Vec3| p.iter().all(|c| c.is_finite());
    let step = points.len() / MAX_SAMPLE + 1;
    let mut sample: Vec<Vec3> = points.iter().step_by(step).map(|p| p.coords).filter(finite).collect();
    if sample.len() < 3 {
        sample = prims.iter().map(anchor).filter(finite).collect();
    }
    if sample.len() < 3 {
        return Vec::new();
    }
    let count = sample.len() as f64;
    let centroid = sample.iter().fold(Vec3::zeros(), |s, p| s + p) / count;
    let mut cov = Matrix3::<f64>::zeros();
    for p in &sample {
        let d = p - centroid;
        cov += d * d.transpose();
    }
    cov /= count;
    if !cov.iter().all(|c| c.is_finite()) {
        return Vec::new();
    }
    let Some(eig) = SymmetricEigen::try_new(cov, f64::EPSILON, 1000) else { return Vec::new() };
    let frame = frame_directions(prims);
    let mut out = Vec::new();
    for k in 0..3 {
        let Some(v) = normalize(eig.eigenvectors.column(k).into_owned()) else { continue };
        let normal = frame
            .iter()
            .map(|f| (f, angle_unsigned(f, &v)))
            .filter(|(_, a)| *a <= angle_tol)
            .min_by(|x, y| x.1.total_cmp(&y.1))
            .map_or(v, |(f, _)| *f);
        let m = Mirror { point: centroid, normal };
        if mirror_supported(prims, &m, angle_tol, length_tol) {
            out.push(m);
        }
    }
    out
}

// ------------------------------------------------------------------------------------------
// imposing relations

/// Makes the directions of every symmetric pair mirror images of each other (their mean): a
/// no-op when they already are, as for axis-aligned designs.
fn symmetrise_directions(prims: &mut [Primitive], mirrors: &[Mirror], rels: &[Rel]) {
    for r in rels.iter().filter(|r| r.kind == RelKind::Symmetric) {
        let Some(m) = r.mirror.and_then(|k| mirrors.get(k)) else { continue };
        let (Some(da), Some(db)) = (prims.get(r.a).and_then(direction), prims.get(r.b).and_then(direction)) else {
            continue;
        };
        let signed = matches!(prims.get(r.a), Some(Primitive::Cone { .. }));
        let mut mb = m.dir(&db);
        if !signed && da.dot(&mb) < 0.0 {
            mb = -mb;
        }
        let Some(mean) = normalize(da + mb) else { continue };
        let mut new_b = m.dir(&mean);
        if !signed && db.dot(&new_b) < 0.0 {
            new_b = -new_b;
        }
        if (mean - da).norm() < 1e-14 && (new_b - db).norm() < 1e-14 {
            continue;
        }
        if let Some(p) = prims.get_mut(r.a) {
            set_direction(p, &mean);
        }
        if let Some(p) = prims.get_mut(r.b) {
            set_direction(p, &new_b);
        }
    }
}

fn find_root(parent: &[usize], i: usize) -> usize {
    let mut r = i;
    while parent.get(r).copied().unwrap_or(r) != r {
        r = parent.get(r).copied().unwrap_or(r);
    }
    r
}

fn join(parent: &mut [usize], a: usize, b: usize) {
    let (ra, rb) = (find_root(parent, a), find_root(parent, b));
    if ra != rb
        && let Some(p) = parent.get_mut(ra.max(rb))
    {
        *p = ra.min(rb);
    }
}

fn sparse_dot(a: &[(usize, f64)], b: &[(usize, f64)]) -> f64 {
    let (mut i, mut j, mut sum) = (0usize, 0usize, 0.0);
    while let (Some(x), Some(y)) = (a.get(i), b.get(j)) {
        match x.0.cmp(&y.0) {
            Ordering::Less => i += 1,
            Ordering::Greater => j += 1,
            Ordering::Equal => {
                sum += x.1 * y.1;
                i += 1;
                j += 1;
            }
        }
    }
    sum
}

/// LU factorisation with partial pivoting of a dense `n × n` matrix (row-major).
struct Lu {
    a: Vec<f64>,
    piv: Vec<usize>,
    n: usize,
}

impl Lu {
    fn new(mut a: Vec<f64>, n: usize) -> Option<Lu> {
        if a.len() != n * n {
            return None;
        }
        let mut piv: Vec<usize> = (0..n).collect();
        for k in 0..n {
            let mut p = k;
            let mut best = a.get(k * n + k)?.abs();
            for i in (k + 1)..n {
                let v = a.get(i * n + k)?.abs();
                if v > best {
                    best = v;
                    p = i;
                }
            }
            if !(best > 1e-300) {
                return None;
            }
            if p != k {
                for j in 0..n {
                    a.swap(k * n + j, p * n + j);
                }
                piv.swap(k, p);
            }
            let akk = *a.get(k * n + k)?;
            for i in (k + 1)..n {
                let f = *a.get(i * n + k)? / akk;
                *a.get_mut(i * n + k)? = f;
                if f != 0.0 {
                    for j in (k + 1)..n {
                        let akj = *a.get(k * n + j)?;
                        *a.get_mut(i * n + j)? -= f * akj;
                    }
                }
            }
        }
        Some(Lu { a, piv, n })
    }

    fn at(&self, i: usize, j: usize) -> f64 {
        self.a.get(i * self.n + j).copied().unwrap_or(0.0)
    }

    fn solve(&self, b: &[f64]) -> Vec<f64> {
        let n = self.n;
        let mut x: Vec<f64> = self.piv.iter().map(|&p| b.get(p).copied().unwrap_or(0.0)).collect();
        for i in 0..n {
            let mut sum = x.get(i).copied().unwrap_or(0.0);
            for j in 0..i {
                sum -= self.at(i, j) * x.get(j).copied().unwrap_or(0.0);
            }
            if let Some(v) = x.get_mut(i) {
                *v = sum;
            }
        }
        for i in (0..n).rev() {
            let mut sum = x.get(i).copied().unwrap_or(0.0);
            for j in (i + 1)..n {
                sum -= self.at(i, j) * x.get(j).copied().unwrap_or(0.0);
            }
            if let Some(v) = x.get_mut(i) {
                *v = sum / self.at(i, i);
            }
        }
        x
    }
}

/// The smallest change `x` of the variables with `A x = rhs` for every row, and the residual of
/// each row. Rows are grouped by the primitives they couple; each group is solved on its own.
/// Errors carry a message and the row to blame.
fn solve_rows(rows: &[Row], nvars: usize) -> Result<(Vec<f64>, Vec<f64>), (String, usize)> {
    let mut parent: Vec<usize> = (0..nvars / 5 + 1).collect();
    for row in rows {
        let mut prims = row.coef.iter().map(|(v, _)| *v / 5);
        if let Some(first) = prims.next() {
            for other in prims {
                join(&mut parent, first, other);
            }
        }
    }
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (ri, row) in rows.iter().enumerate() {
        let root = row.coef.first().map_or(0, |(v, _)| find_root(&parent, *v / 5));
        groups.entry(root).or_default().push(ri);
    }
    let mut x = vec![0.0; nvars];
    let mut resid = vec![0.0; rows.len()];
    for idx in groups.into_values() {
        let first = idx.first().copied().unwrap_or(0);
        let rs: Vec<&Row> = idx.iter().filter_map(|&i| rows.get(i)).collect();
        let k = rs.len();
        if k > MAX_COMPONENT_ROWS {
            return Err(("too many relations are coupled to solve them together".into(), first));
        }
        let mut g = vec![0.0; k * k];
        for a in 0..k {
            for b in a..k {
                let (Some(ra), Some(rb)) = (rs.get(a), rs.get(b)) else { continue };
                let d = sparse_dot(&ra.coef, &rb.coef);
                if let Some(v) = g.get_mut(a * k + b) {
                    *v = d;
                }
                if let Some(v) = g.get_mut(b * k + a) {
                    *v = d;
                }
            }
        }
        let mean_diag = (0..k).map(|a| g.get(a * k + a).copied().unwrap_or(0.0)).sum::<f64>() / (k.max(1) as f64);
        let lambda = 1e-10 * mean_diag.max(1e-12);
        for a in 0..k {
            if let Some(v) = g.get_mut(a * k + a) {
                *v += lambda;
            }
        }
        let Some(lu) = Lu::new(g, k) else {
            return Err(("the relations cannot be solved together".into(), first));
        };
        let mut r: Vec<f64> = rs.iter().map(|row| row.rhs).collect();
        for _ in 0..4 {
            let y = lu.solve(&r);
            for (row, yi) in rs.iter().zip(&y) {
                for (v, c) in &row.coef {
                    if let Some(xv) = x.get_mut(*v) {
                        *xv += c * yi;
                    }
                }
            }
            let mut worst = 0.0_f64;
            for (slot, row) in r.iter_mut().zip(&rs) {
                let ax: f64 = row.coef.iter().map(|(v, c)| c * x.get(*v).copied().unwrap_or(0.0)).sum();
                *slot = row.rhs - ax;
                worst = worst.max(slot.abs());
            }
            if worst < 1e-13 {
                break;
            }
        }
        for (e, &ri) in r.iter().zip(&idx) {
            if let Some(out) = resid.get_mut(ri) {
                *out = e.abs();
            }
        }
    }
    if x.iter().any(|v| !v.is_finite()) {
        return Err(("the relations gave a non-finite solution".into(), 0));
    }
    Ok((x, resid))
}

/// Imposes the relations on the primitives by the smallest change of their positions and radii.
/// Directions are only touched to make symmetric pairs mirror images. The primitives are left
/// unchanged when this fails (conflicting relations, an invalid result).
///
/// # Errors
/// A [`Failure`] naming the relation that conflicts most when the relations cannot all hold.
pub fn apply(prims: &mut [Primitive], mirrors: &[Mirror], rels: &[Rel]) -> Result<(), Failure> {
    let mut work: Vec<Primitive> = prims.to_vec();
    symmetrise_directions(&mut work, mirrors, rels);
    let mut rows: Vec<Row> = Vec::new();
    let mut owner: Vec<usize> = Vec::new();
    for (ri, r) in rels.iter().enumerate() {
        if let Some((rs, _, _)) = rows_of(&work, mirrors, r) {
            for row in rs {
                rows.push(merge_row(&row));
                owner.push(ri);
            }
        }
    }
    if rows.is_empty() {
        for (dst, src) in prims.iter_mut().zip(work) {
            *dst = src;
        }
        return Ok(());
    }
    if rows.len() > MAX_ROWS {
        return Err(Failure { message: "too many relations to solve together".into(), worst: None });
    }
    let (x, resid) = solve_rows(&rows, work.len() * 5)
        .map_err(|(message, row)| Failure { message, worst: owner.get(row).copied() })?;
    let (mut worst_row, mut worst) = (0usize, 0.0_f64);
    for (ri, e) in resid.iter().enumerate() {
        if *e > worst {
            worst = *e;
            worst_row = ri;
        }
    }
    if !(worst <= CONSISTENT) {
        return Err(Failure {
            message: format!("the relations conflict with each other (residual {worst:.2e} mm)"),
            worst: owner.get(worst_row).copied(),
        });
    }
    for (i, p) in work.iter_mut().enumerate() {
        let g = |k: usize| x.get(var(i, k)).copied().unwrap_or(0.0);
        move_anchor(p, &Vec3::new(g(0), g(1), g(2)));
        shift_radii(p, [g(3), g(4)]);
        if !p.is_valid() {
            return Err(Failure { message: "the snapped surface would be degenerate".into(), worst: None });
        }
    }
    for (dst, src) in prims.iter_mut().zip(work) {
        *dst = src;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plane(z: f64) -> Primitive {
        Primitive::Plane { point: [0.0, 0.0, z], normal: [0.0, 0.0, 1.0] }
    }

    fn z_of(p: &Primitive) -> f64 {
        match p {
            Primitive::Plane { point, .. } => point[2],
            _ => f64::NAN,
        }
    }

    #[test]
    fn three_planes_become_equally_spaced() {
        let mut prims = vec![plane(0.0), plane(10.02), plane(19.99)];
        let rels = find_relations(&prims, &[], 3.0, 0.2);
        assert_eq!(rels.iter().filter(|r| r.kind == RelKind::EqualDistance).count(), 1);
        apply(&mut prims, &[], &rels).unwrap();
        let z: Vec<f64> = prims.iter().map(z_of).collect();
        assert!(((z[1] - z[0]) - (z[2] - z[1])).abs() < 1e-9, "{z:?}");
        // the smallest change: the total moves are far below the 0.2 mm tolerance
        assert!((z[0]).abs() < 0.05 && (z[2] - 19.99).abs() < 0.05, "{z:?}");
    }

    #[test]
    fn plane_touches_sphere() {
        let mut prims = vec![plane(0.0), Primitive::Sphere { center: [1.0, 2.0, 5.02], radius: 5.0 }];
        let rels = find_relations(&prims, &[], 3.0, 0.2);
        assert_eq!(rels.iter().filter(|r| r.kind == RelKind::Tangent).count(), 1);
        apply(&mut prims, &[], &rels).unwrap();
        let (m, _) = measure(&prims, &[], &rels[0]).unwrap();
        assert!(m < 1e-9, "{m}");
    }
}

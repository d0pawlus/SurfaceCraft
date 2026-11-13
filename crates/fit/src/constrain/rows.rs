//! Linear rows of the relations and their measured residuals.

use super::geometry::*;
use super::types::*;
use super::*;

// ------------------------------------------------------------------------------------------
// linear rows

/// One linear equation `Σ coef · x = rhs` over the variables of the primitives.
#[derive(Clone, Debug)]
pub(super) struct Row {
    pub(super) coef: Vec<(usize, f64)>,
    pub(super) rhs: f64,
}

pub(super) fn var(prim: usize, k: usize) -> usize {
    prim * 5 + k
}

fn push_vec(coef: &mut Vec<(usize, f64)>, prim: usize, v: &Vec3, scale: f64) {
    coef.push((var(prim, 0), v.x * scale));
    coef.push((var(prim, 1), v.y * scale));
    coef.push((var(prim, 2), v.z * scale));
}

/// Sorted, duplicate-free coefficients.
pub(super) fn merge_row(row: &Row) -> Row {
    let mut c = row.coef.clone();
    c.sort_by_key(|a| a.0);
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
    Some((vec![Row { coef, rhs: s * r - sigma }], (sigma.abs() - r).abs(), angle))
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
    Some((vec![Row { coef, rhs }], gap.abs(), angle_unsigned(&da, &db)))
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

pub(super) fn tangent_rows(prims: &[Primitive], a: usize, b: usize) -> Option<(Vec<Row>, f64, f64)> {
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

pub(super) fn rows_of(prims: &[Primitive], mirrors: &[Mirror], r: &Rel) -> Option<(Vec<Row>, f64, f64)> {
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

//! Geometry of fitted primitives for `primitive.auto_constrain`: directions, axes and radii, the
//! union-find over candidate relations, and the snapping of each relation.
use super::{Relation, RelationKind};
use surfacecraft_doc::ObjectId;
use surfacecraft_fit::Primitive;

pub(super) type V3 = surfacecraft_geom::Vec3;

/// Unit direction of a primitive (normal or axis), if it has one.
pub(super) fn direction(p: &Primitive) -> Option<V3> {
    match p {
        Primitive::Plane { normal, .. } => Some(V3::from(*normal)),
        Primitive::Cylinder { axis, .. } | Primitive::Cone { axis, .. } | Primitive::Torus { axis, .. } => {
            Some(V3::from(*axis))
        }
        Primitive::Sphere { .. } => None,
    }
    .and_then(surfacecraft_geom::normalize)
}

/// A point on the axis of an axial primitive.
fn axis_point(p: &Primitive) -> Option<V3> {
    match p {
        Primitive::Cylinder { point, .. } => Some(V3::from(*point)),
        Primitive::Cone { apex, .. } => Some(V3::from(*apex)),
        Primitive::Torus { center, .. } => Some(V3::from(*center)),
        _ => None,
    }
}

fn radius_of(p: &Primitive) -> Option<(u8, f64)> {
    match p {
        Primitive::Cylinder { radius, .. } => Some((0, *radius)),
        Primitive::Sphere { radius, .. } => Some((1, *radius)),
        _ => None,
    }
}

fn set_direction(p: &mut Primitive, d: V3) {
    match p {
        Primitive::Plane { normal, .. } => *normal = d.into(),
        Primitive::Cylinder { axis, .. } | Primitive::Cone { axis, .. } | Primitive::Torus { axis, .. } => {
            *axis = d.into();
        }
        Primitive::Sphere { .. } => {}
    }
}

fn set_axis_point(p: &mut Primitive, q: V3) {
    match p {
        Primitive::Cylinder { point, .. } => *point = q.into(),
        Primitive::Cone { apex, .. } => *apex = q.into(),
        Primitive::Torus { center, .. } => *center = q.into(),
        _ => {}
    }
}

fn set_radius(p: &mut Primitive, r: f64) {
    match p {
        Primitive::Cylinder { radius, .. } | Primitive::Sphere { radius, .. } => *radius = r,
        _ => {}
    }
}

pub(super) fn find(parent: &mut [usize], i: usize) -> usize {
    let mut r = i;
    while parent.get(r).copied().unwrap_or(r) != r {
        r = parent.get(r).copied().unwrap_or(r);
    }
    // path compression
    let mut c = i;
    while parent.get(c).copied().unwrap_or(c) != c {
        let next = parent.get(c).copied().unwrap_or(c);
        if let Some(p) = parent.get_mut(c) {
            *p = r;
        }
        c = next;
    }
    r
}

pub(super) fn union(parent: &mut [usize], a: usize, b: usize) {
    let (ra, rb) = (find(parent, a), find(parent, b));
    if ra != rb {
        let (lo, hi) = (ra.min(rb), ra.max(rb));
        if let Some(p) = parent.get_mut(hi) {
            *p = lo;
        }
    }
}

/// Relations of a primitive list within the tolerances (`angle_tol` in degrees).
pub(super) fn find_relations(
    prims: &[(ObjectId, Primitive)],
    angle_tol: f64,
    length_tol: f64,
) -> Vec<(usize, usize, Relation)> {
    let mut out = Vec::new();
    for (i, (ida, pa)) in prims.iter().enumerate() {
        for (j, (idb, pb)) in prims.iter().enumerate().skip(i + 1) {
            let rel = |kind, error| Relation { kind, a: *ida, b: *idb, c: None, error };
            if let (Some(da), Some(db)) = (direction(pa), direction(pb)) {
                let theta = da.dot(&db).abs().clamp(0.0, 1.0).acos().to_degrees();
                if theta <= angle_tol {
                    out.push((i, j, rel(RelationKind::Parallel, theta)));
                    if let (Some(oa), Some(ob)) = (axis_point(pa), axis_point(pb)) {
                        let w = ob - oa;
                        let off = (w - da * w.dot(&da)).norm();
                        if off <= length_tol {
                            out.push((i, j, rel(RelationKind::Coaxial, off)));
                        }
                    }
                } else if 90.0 - theta <= angle_tol {
                    out.push((i, j, rel(RelationKind::Perpendicular, 90.0 - theta)));
                }
            }
            if let (Some((ka, ra)), Some((kb, rb))) = (radius_of(pa), radius_of(pb))
                && ka == kb
                && (ra - rb).abs() <= length_tol
            {
                out.push((i, j, rel(RelationKind::EqualRadius, (ra - rb).abs())));
            }
        }
    }
    out
}

/// Snaps the primitives so that every found relation holds exactly.
pub(super) fn snap(prims: &mut [(ObjectId, Primitive)], rels: &[(usize, usize, Relation)]) {
    let n = prims.len();
    // 1. directions: parallel classes get their mean, perpendicular classes are orthogonalised
    let mut parent: Vec<usize> = (0..n).collect();
    for (i, j, r) in rels {
        if r.kind == RelationKind::Parallel {
            union(&mut parent, *i, *j);
        }
    }
    let mut classes: Vec<(usize, Vec<usize>)> = Vec::new();
    for i in 0..n {
        if prims.get(i).and_then(|p| direction(&p.1)).is_none() {
            continue;
        }
        let r = find(&mut parent, i);
        match classes.iter_mut().find(|c| c.0 == r) {
            Some(c) => c.1.push(i),
            None => classes.push((r, vec![i])),
        }
    }
    // largest class first, ties by lowest member
    classes.sort_by(|a, b| b.1.len().cmp(&a.1.len()).then(a.1.first().cmp(&b.1.first())));
    let class_of = |i: usize, classes: &[(usize, Vec<usize>)]| classes.iter().position(|c| c.1.contains(&i));
    let mut fixed: Vec<(usize, V3)> = Vec::new(); // class index → direction
    for (ci, (_, members)) in classes.iter().enumerate() {
        let Some(first) = members.first().and_then(|&m| prims.get(m)).and_then(|p| direction(&p.1)) else { continue };
        let mut mean = V3::zeros();
        for &m in members {
            if let Some(d) = prims.get(m).and_then(|p| direction(&p.1)) {
                mean += if d.dot(&first) < 0.0 { -d } else { d };
            }
        }
        let mut d = surfacecraft_geom::normalize(mean).unwrap_or(first);
        for (fc, fd) in &fixed {
            let perpendicular = rels.iter().any(|(i, j, r)| {
                r.kind == RelationKind::Perpendicular
                    && ((class_of(*i, &classes) == Some(ci) && class_of(*j, &classes) == Some(*fc))
                        || (class_of(*j, &classes) == Some(ci) && class_of(*i, &classes) == Some(*fc)))
            });
            if perpendicular {
                d = surfacecraft_geom::normalize(d - fd * d.dot(fd)).unwrap_or(d);
            }
        }
        fixed.push((ci, d));
        for &m in members {
            if let Some(p) = prims.get_mut(m)
                && let Some(orig) = direction(&p.1)
            {
                set_direction(&mut p.1, if orig.dot(&d) < 0.0 { -d } else { d });
            }
        }
    }
    // 2. coaxial groups share one axis line (the mean of their perpendicular offsets)
    let mut parent: Vec<usize> = (0..n).collect();
    for (i, j, r) in rels {
        if r.kind == RelationKind::Coaxial {
            union(&mut parent, *i, *j);
        }
    }
    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for i in 0..n {
        if prims.get(i).and_then(|p| axis_point(&p.1)).is_none() {
            continue;
        }
        let r = find(&mut parent, i);
        match groups.iter_mut().find(|g| g.0 == r) {
            Some(g) => g.1.push(i),
            None => groups.push((r, vec![i])),
        }
    }
    for (_, members) in groups.iter().filter(|g| g.1.len() > 1) {
        let Some((o, d)) =
            members.first().and_then(|&m| prims.get(m)).and_then(|p| axis_point(&p.1).zip(direction(&p.1)))
        else {
            continue;
        };
        let offs: Vec<(usize, V3)> = members
            .iter()
            .filter_map(|&m| {
                let q = prims.get(m).and_then(|p| axis_point(&p.1))?;
                let w = q - o;
                Some((m, w - d * w.dot(&d)))
            })
            .collect();
        let mean = offs.iter().fold(V3::zeros(), |a, (_, w)| a + w) / offs.len().max(1) as f64;
        for (m, w) in offs {
            if let Some(p) = prims.get_mut(m)
                && let Some(q) = axis_point(&p.1)
            {
                set_axis_point(&mut p.1, q - w + mean);
            }
        }
    }
    // 3. equal radii share their mean
    let mut parent: Vec<usize> = (0..n).collect();
    for (i, j, r) in rels {
        if r.kind == RelationKind::EqualRadius {
            union(&mut parent, *i, *j);
        }
    }
    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for i in 0..n {
        if prims.get(i).and_then(|p| radius_of(&p.1)).is_none() {
            continue;
        }
        let r = find(&mut parent, i);
        match groups.iter_mut().find(|g| g.0 == r) {
            Some(g) => g.1.push(i),
            None => groups.push((r, vec![i])),
        }
    }
    for (_, members) in groups.iter().filter(|g| g.1.len() > 1) {
        let radii: Vec<f64> =
            members.iter().filter_map(|&m| prims.get(m).and_then(|p| radius_of(&p.1)).map(|r| r.1)).collect();
        let mean = radii.iter().sum::<f64>() / radii.len().max(1) as f64;
        for &m in members {
            if let Some(p) = prims.get_mut(m) {
                set_radius(&mut p.1, mean);
            }
        }
    }
}

/// Residual of an angular / coaxial / equal-radius relation on two primitives: (degrees, mm).
pub(super) fn old_residual(kind: RelationKind, pa: &Primitive, pb: &Primitive) -> (f64, f64) {
    let (mut angle, mut len) = (0.0_f64, 0.0_f64);
    match kind {
        RelationKind::Parallel | RelationKind::Perpendicular | RelationKind::Coaxial => {
            if let (Some(da), Some(db)) = (direction(pa), direction(pb)) {
                let theta = da.dot(&db).abs().clamp(0.0, 1.0).acos().to_degrees();
                angle = if kind == RelationKind::Perpendicular { (90.0 - theta).abs() } else { theta };
                if kind == RelationKind::Coaxial
                    && let (Some(oa), Some(ob)) = (axis_point(pa), axis_point(pb))
                {
                    let w = ob - oa;
                    len = (w - da * w.dot(&da)).norm();
                }
            }
        }
        RelationKind::EqualRadius => {
            if let (Some((_, ra)), Some((_, rb))) = (radius_of(pa), radius_of(pb)) {
                len = (ra - rb).abs();
            }
        }
        RelationKind::Tangent | RelationKind::Symmetric | RelationKind::EqualDistance => {}
    }
    (angle, len)
}

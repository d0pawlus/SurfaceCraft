//! Finding the relations and the mirror planes of a set of fitted primitives within tolerances.

use super::geometry::*;
use super::rows::*;
use super::types::*;
use super::*;

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

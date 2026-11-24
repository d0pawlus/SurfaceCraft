//! `primitive.auto_constrain`: finds the relations between fitted primitives and snaps them, unless
//! a snap would raise a primitive RMS above its limit.
use super::relations::{V3, find, find_relations, old_residual, snap, union};
use super::{AutoConstrainIn, AutoConstrainOut, PrimitiveSigma, RejectedRelation, Relation, RelationKind};
use crate::Session;
use crate::cmd::{CmdError, CmdResult};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::{Primitive, constrain};
use surfacecraft_geom::Point3;

/// The most points of a region used for an RMS.
const MAX_RMS_POINTS: usize = 4000;
/// The RMS no primitive may be raised above by snapping is at least this (mm, design doc §11.1).
const RMS_FLOOR: f64 = 0.1;

/// Vertices of the region a primitive was fitted to, evenly thinned to `MAX_RMS_POINTS`.
fn region_sample(s: &Session, id: ObjectId) -> Vec<Point3> {
    let Some(reg) = s.doc.primitive(id).and_then(|p| p.region).and_then(|r| s.doc.region(r)) else {
        return Vec::new();
    };
    let Some(mesh) = s.doc.mesh(reg.mesh) else { return Vec::new() };
    let mut ids: Vec<u32> = reg
        .triangles
        .iter()
        .filter_map(|&t| mesh.mesh.triangles.get(t as usize))
        .flat_map(|t| t.iter().copied())
        .collect();
    ids.sort_unstable();
    ids.dedup();
    let step = ids.len() / MAX_RMS_POINTS + 1;
    ids.iter().step_by(step).filter_map(|&v| mesh.mesh.positions.get(v as usize).copied()).collect()
}

/// RMS and largest deviation of `pts` from the primitive; none without points.
fn rms_of(p: &Primitive, pts: &[Point3]) -> Option<(f64, f64)> {
    if pts.is_empty() {
        return None;
    }
    let (mut sum, mut max) = (0.0_f64, 0.0_f64);
    for q in pts {
        let d = p.distance(q);
        sum += d * d;
        max = max.max(d.abs());
    }
    let rms = (sum / pts.len() as f64).sqrt();
    (rms.is_finite() && max.is_finite()).then_some((rms, max))
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(|a, b| a.total_cmp(b));
    v.get(v.len() / 2).copied().unwrap_or(0.0)
}

/// The RMS check: snapping must not raise the RMS of a primitive above `limit`.
struct Gate<'a> {
    pts: &'a [Vec<Point3>],
    before: &'a [Option<f64>],
    names: &'a [String],
    limit: f64,
}

impl Gate<'_> {
    fn after(&self, idx: usize, p: &Primitive) -> Option<f64> {
        rms_of(p, self.pts.get(idx)?).map(|v| v.0)
    }

    /// A reason when primitive `idx`, as `p`, has been pushed above the limit by snapping.
    fn broken(&self, idx: usize, p: &Primitive) -> Option<String> {
        let after = self.after(idx, p)?;
        let before = (*self.before.get(idx)?)?;
        (after > self.limit && after > before + 1e-9).then(|| {
            format!(
                "snapping would raise the RMS of `{}` from {before:.4} mm to {after:.4} mm, above the limit of {:.4} mm",
                self.names.get(idx).map_or("?", String::as_str),
                self.limit
            )
        })
    }
}

/// A relation under consideration: for the output, the snapping and the RMS check.
struct Cand {
    rel: Relation,
    i: usize,
    j: usize,
    k: Option<usize>,
    /// The relation for the position stage (tangent, symmetric and equal distance only).
    crel: Option<constrain::Rel>,
    /// Largest RMS of its primitives after snapping this relation alone (mm).
    effect: f64,
    /// Why it was rejected.
    verdict: Option<String>,
}

impl Cand {
    fn live(&self) -> bool {
        self.verdict.is_none()
    }

    fn is_new(&self) -> bool {
        self.crel.is_some()
    }

    fn touches(&self, p: usize) -> bool {
        self.i == p || self.j == p || self.k == Some(p)
    }
}

fn live_old(cands: &[Cand]) -> Vec<(usize, usize, Relation)> {
    cands.iter().filter(|c| c.live() && !c.is_new()).map(|c| (c.i, c.j, c.rel.clone())).collect()
}

/// The coaxial and equal-radius relations still standing, as chains from the first member of each
/// group, to keep them holding while the position stage runs.
fn carried(cands: &[Cand], n: usize) -> Vec<constrain::Rel> {
    let mut out = Vec::new();
    for (kind, ck) in [
        (RelationKind::Coaxial, constrain::RelKind::Coaxial),
        (RelationKind::EqualRadius, constrain::RelKind::EqualRadius),
    ] {
        let live: Vec<&Cand> = cands.iter().filter(|c| c.live() && c.rel.kind == kind).collect();
        let mut parent: Vec<usize> = (0..n).collect();
        for c in &live {
            union(&mut parent, c.i, c.j);
        }
        let mut members: Vec<usize> = live.iter().flat_map(|c| [c.i, c.j]).collect();
        members.sort_unstable();
        members.dedup();
        for m in members {
            let root = find(&mut parent, m);
            if root != m {
                out.push(constrain::Rel { kind: ck, a: root, b: m, c: None, mirror: None });
            }
        }
    }
    out
}

/// The primitives after snapping every live relation: directions and radii first (the angular,
/// coaxial and equal-radius relations), then the position stage for the new relations. An error
/// carries the message and the candidate to blame.
fn simulate(
    prims: &[(ObjectId, Primitive)],
    cands: &[Cand],
    mirrors: &[constrain::Mirror],
) -> Result<Vec<Primitive>, (String, Option<usize>)> {
    let mut out = prims.to_vec();
    snap(&mut out, &live_old(cands));
    let mut ps: Vec<Primitive> = out.into_iter().map(|p| p.1).collect();
    let mut rels: Vec<constrain::Rel> = Vec::new();
    let mut owner: Vec<Option<usize>> = Vec::new();
    for (ci, c) in cands.iter().enumerate().filter(|(_, c)| c.live()) {
        if let Some(r) = c.crel {
            rels.push(r);
            owner.push(Some(ci));
        }
    }
    if rels.is_empty() {
        return Ok(ps);
    }
    for r in carried(cands, ps.len()) {
        rels.push(r);
        owner.push(None);
    }
    constrain::apply(&mut ps, mirrors, &rels)
        .map_err(|f| (f.message, f.worst.and_then(|w| owner.get(w).copied().flatten())))?;
    Ok(ps)
}

/// Checks the angular and length tolerances and the noise sigma, in that order.
fn check_tolerances(angle_tol: f64, length_tol: f64, noise_sigma: Option<f64>) -> CmdResult<()> {
    if !(angle_tol.is_finite() && (0.0..=20.0).contains(&angle_tol)) {
        return Err(CmdError::Params("angle_tol_deg must be within 0..=20".into()));
    }
    if !(length_tol.is_finite() && length_tol >= 0.0) {
        return Err(CmdError::Params("length_tol must be ≥ 0".into()));
    }
    if let Some(v) = noise_sigma
        && !(v.is_finite() && v >= 0.0)
    {
        return Err(CmdError::Params("noise_sigma must be ≥ 0".into()));
    }
    Ok(())
}

/// The mirror planes given as `symmetry_planes`: references, never snapped or related to the rest.
fn given_mirrors(s: &Session, i: &AutoConstrainIn) -> CmdResult<Vec<(ObjectId, constrain::Mirror)>> {
    let mut given: Vec<(ObjectId, constrain::Mirror)> = Vec::new();
    for r in i.symmetry_planes.iter().flatten() {
        let id = r.resolve(s, ObjectKind::Primitive)?;
        let p = s.doc.primitive(id).ok_or_else(|| CmdError::Params(format!("no primitive {id}")))?;
        let Primitive::Plane { point, normal } = &p.primitive else {
            return Err(CmdError::Params(format!("symmetry plane {id} is not a plane")));
        };
        let m = constrain::Mirror::new(V3::from(*point), V3::from(*normal))
            .ok_or_else(|| CmdError::Params(format!("symmetry plane {id} has no normal")))?;
        given.push((id, m));
    }
    Ok(given)
}

/// The primitives to relate with their names: the listed objects or all, each once, never a mirror.
fn collect_primitives(
    s: &Session,
    i: &AutoConstrainIn,
    given: &[(ObjectId, constrain::Mirror)],
) -> CmdResult<(Vec<(ObjectId, Primitive)>, Vec<String>)> {
    let ids: Vec<ObjectId> = match &i.objects {
        Some(list) => list.iter().map(|r| r.resolve(s, ObjectKind::Primitive)).collect::<CmdResult<_>>()?,
        None => s.doc.primitives.iter().map(|p| p.id).collect(),
    };
    let mut prims: Vec<(ObjectId, Primitive)> = Vec::new();
    let mut names: Vec<String> = Vec::new();
    for id in ids {
        if prims.iter().any(|p| p.0 == id) || given.iter().any(|g| g.0 == id) {
            continue;
        }
        let p = s.doc.primitive(id).ok_or_else(|| CmdError::Params(format!("no primitive {id}")))?;
        prims.push((id, p.primitive.clone()));
        names.push(p.name.clone());
    }
    if prims.len() < 2 {
        return Err(CmdError::Params("auto-constrain needs at least two primitives".into()));
    }
    Ok((prims, names))
}

/// The sampled points of each primitive's region, and the RMS of each about its primitive.
fn region_samples(s: &Session, prims: &[(ObjectId, Primitive)]) -> (Vec<Vec<Point3>>, Vec<Option<f64>>) {
    let pts: Vec<Vec<Point3>> = prims.iter().map(|(id, _)| region_sample(s, *id)).collect();
    let before: Vec<Option<f64>> = prims.iter().zip(&pts).map(|((_, p), q)| rms_of(p, q).map(|v| v.0)).collect();
    (pts, before)
}

/// Sigma of the scan noise: the value given, else the median RMS of the fitted regions.
fn estimate_sigma(s: &Session, prims: &[(ObjectId, Primitive)], before: &[Option<f64>], given: Option<f64>) -> f64 {
    match given {
        Some(v) => v,
        None => {
            let fitted: Vec<f64> = before.iter().flatten().copied().collect();
            if fitted.is_empty() {
                median(
                    prims
                        .iter()
                        .filter_map(|(id, _)| s.doc.primitive(*id).map(|p| p.rms))
                        .filter(|r| r.is_finite())
                        .collect(),
                )
            } else {
                median(fitted)
            }
        }
    }
}

/// The angular, coaxial and equal-radius candidates, each checked alone against the RMS limit.
fn angular_candidates(prims: &[(ObjectId, Primitive)], angle_tol: f64, length_tol: f64, gate: &Gate<'_>) -> Vec<Cand> {
    let mut cands: Vec<Cand> = find_relations(prims, angle_tol, length_tol)
        .into_iter()
        .map(|(a, b, rel)| Cand { rel, i: a, j: b, k: None, crel: None, effect: 0.0, verdict: None })
        .collect();
    for c in &mut cands {
        check_pair_alone(c, prims, gate);
    }
    cands
}

/// Snaps the two primitives of one angular relation alone and records its RMS effect and verdict.
fn check_pair_alone(c: &mut Cand, prims: &[(ObjectId, Primitive)], gate: &Gate<'_>) {
    let (Some(pa), Some(pb)) = (prims.get(c.i), prims.get(c.j)) else { return };
    let mut local = vec![pa.clone(), pb.clone()];
    snap(&mut local, &[(0, 1, c.rel.clone())]);
    for (slot, idx) in [(0_usize, c.i), (1, c.j)] {
        if let Some((_, p)) = local.get(slot) {
            c.effect = c.effect.max(gate.after(idx, p).unwrap_or(0.0));
            if c.verdict.is_none() {
                c.verdict = gate.broken(idx, p);
            }
        }
    }
}

/// The primitives with the directions and axes of the live angular relations snapped.
fn snapped_directions(prims: &[(ObjectId, Primitive)], cands: &[Cand]) -> Vec<Primitive> {
    let mut s1 = prims.to_vec();
    snap(&mut s1, &live_old(cands));
    s1.into_iter().map(|p| p.1).collect()
}

/// The mirror planes for the symmetry relations: the given ones, or else the detected ones.
fn mirror_planes(
    given: &[(ObjectId, constrain::Mirror)],
    detect: bool,
    s1p: &[Primitive],
    pts: &[Vec<Point3>],
    angle_tol: f64,
    length_tol: f64,
) -> Vec<constrain::Mirror> {
    let mut mirrors: Vec<constrain::Mirror> = given.iter().map(|g| g.1).collect();
    if given.is_empty() && detect {
        let all: Vec<Point3> = pts.iter().flatten().copied().collect();
        mirrors.extend(constrain::detect_mirrors(s1p, &all, angle_tol, length_tol));
    }
    mirrors
}

/// The candidates for the tangent, symmetric and equal-distance relations, found on `s1p`.
fn tangent_candidates(
    prims: &[(ObjectId, Primitive)],
    raw: &[Primitive],
    s1p: &[Primitive],
    mirrors: &[constrain::Mirror],
    angle_tol: f64,
    length_tol: f64,
) -> Vec<Cand> {
    let id_of = |k: usize| prims.get(k).map(|p| p.0);
    let mut out = Vec::new();
    for r in constrain::find_relations(s1p, mirrors, angle_tol, length_tol) {
        let kind = match r.kind {
            constrain::RelKind::Tangent => RelationKind::Tangent,
            constrain::RelKind::Symmetric => RelationKind::Symmetric,
            constrain::RelKind::EqualDistance => RelationKind::EqualDistance,
            constrain::RelKind::Coaxial | constrain::RelKind::EqualRadius => continue,
        };
        let (Some(a), Some(b)) = (id_of(r.a), id_of(r.b)) else { continue };
        let error = constrain::measure(raw, mirrors, &r).map_or(0.0, |m| m.0);
        out.push(Cand {
            rel: Relation { kind, a, b, c: r.c.and_then(id_of), error },
            i: r.a,
            j: r.b,
            k: r.c,
            crel: Some(r),
            effect: 0.0,
            verdict: None,
        });
    }
    out
}

/// Imposes each new relation alone on its primitives and records its RMS effect and verdict.
fn check_new_alone(cands: &mut [Cand], s1p: &[Primitive], mirrors: &[constrain::Mirror], gate: &Gate<'_>) {
    for c in cands.iter_mut().filter(|c| c.is_new()) {
        let Some(r) = c.crel else { continue };
        let idxs: Vec<usize> = [c.i, c.j].into_iter().chain(c.k).collect();
        let mut local: Vec<Primitive> = idxs.iter().filter_map(|&x| s1p.get(x).cloned()).collect();
        let lone = constrain::Rel { a: 0, b: 1, c: c.k.map(|_| 2), ..r };
        match constrain::apply(&mut local, mirrors, &[lone]) {
            Err(f) => c.verdict = Some(f.message),
            Ok(()) => {
                for (slot, &idx) in idxs.iter().enumerate() {
                    if let Some(p) = local.get(slot) {
                        c.effect = c.effect.max(gate.after(idx, p).unwrap_or(0.0));
                        if c.verdict.is_none() {
                            c.verdict = gate.broken(idx, p);
                        }
                    }
                }
            }
        }
    }
}

/// The primitive that snapping pushes furthest above the limit: its index, its RMS and the reason.
fn worst_breach(ps: &[Primitive], gate: &Gate<'_>) -> Option<(usize, f64, String)> {
    let mut worst: Option<(usize, f64, String)> = None;
    for (x, p) in ps.iter().enumerate() {
        if let Some(msg) = gate.broken(x, p) {
            let a = gate.after(x, p).unwrap_or(0.0);
            if worst.as_ref().is_none_or(|w| a > w.1) {
                worst = Some((x, a, msg));
            }
        }
    }
    worst
}

/// The live relation that touches primitive `x` with the largest effect (the last one on a tie).
fn blame_for(cands: &[Cand], x: usize) -> Option<usize> {
    cands
        .iter()
        .enumerate()
        .filter(|(_, c)| c.live() && c.touches(x))
        .max_by(|a, b| a.1.effect.total_cmp(&b.1.effect))
        .map(|(k, _)| k)
}

/// Imposes all live relations together, rejecting the most damaging one per round (at most 64).
fn impose_together(
    prims: &[(ObjectId, Primitive)],
    cands: &mut [Cand],
    mirrors: &[constrain::Mirror],
    gate: &Gate<'_>,
) -> Option<Vec<Primitive>> {
    for _ in 0..64 {
        match simulate(prims, cands, mirrors) {
            Err((msg, culprit)) => {
                let k = culprit.or_else(|| cands.iter().rposition(|c| c.live() && c.is_new()))?;
                if let Some(c) = cands.get_mut(k) {
                    c.verdict = Some(msg);
                }
            }
            Ok(ps) => {
                let Some((x, _, msg)) = worst_breach(&ps, gate) else { return Some(ps) };
                let k = blame_for(cands, x)?;
                let c = cands.get_mut(k)?;
                c.verdict = Some(format!("together with the other relations: {msg}"));
            }
        }
    }
    None
}

/// The primitives snapped with every live relation together, or, when that fails, as fitted.
fn impose_all(
    prims: &[(ObjectId, Primitive)],
    cands: &mut [Cand],
    mirrors: &[constrain::Mirror],
    gate: &Gate<'_>,
    raw: &[Primitive],
) -> Vec<Primitive> {
    if let Some(ps) = impose_together(prims, cands, mirrors, gate) {
        return ps;
    }
    for c in cands.iter_mut().filter(|c| c.live()) {
        c.verdict = Some("the relations could not be imposed together within the RMS limit".into());
    }
    raw.to_vec()
}

/// Writes the snapped primitives back into the document, re-projecting their patches.
fn apply_snapped(s: &mut Session, prims: &[(ObjectId, Primitive)], snapped: &[Primitive]) {
    for ((id, _), new) in prims.iter().zip(snapped) {
        if let Some(obj) = s.doc.primitives.iter_mut().find(|p| p.id == *id)
            && obj.primitive != *new
        {
            let mut patch = (*obj.patch).clone();
            for q in &mut patch.positions {
                *q = new.project(q);
            }
            obj.patch = std::sync::Arc::new(patch);
            obj.primitive = new.clone();
        }
    }
}

/// The largest angular and length residuals of the relations that stand, on the `shown` primitives.
fn largest_residuals(cands: &[Cand], shown: &[Primitive], mirrors: &[constrain::Mirror]) -> (f64, f64) {
    let (mut max_angle, mut max_len) = (0.0_f64, 0.0_f64);
    for c in cands.iter().filter(|c| c.live()) {
        if let Some(r) = c.crel {
            if let Some((len, angle)) = constrain::measure(shown, mirrors, &r) {
                max_len = max_len.max(len);
                max_angle = max_angle.max(angle);
            }
        } else if let (Some(pa), Some(pb)) = (shown.get(c.i), shown.get(c.j)) {
            let (angle, len) = old_residual(c.rel.kind, pa, pb);
            max_angle = max_angle.max(angle);
            max_len = max_len.max(len);
        }
    }
    (max_angle, max_len)
}

/// The RMS of each primitive's region before and after snapping, with its name and point count.
fn primitive_sigmas(
    prims: &[(ObjectId, Primitive)],
    names: &[String],
    pts: &[Vec<Point3>],
    before: &[Option<f64>],
    snapped: &[Primitive],
) -> Vec<PrimitiveSigma> {
    prims
        .iter()
        .enumerate()
        .map(|(x, (id, _))| PrimitiveSigma {
            id: *id,
            name: names.get(x).cloned().unwrap_or_default(),
            points: pts.get(x).map_or(0, Vec::len),
            rms_before: before.get(x).copied().flatten(),
            rms_after: snapped.get(x).zip(pts.get(x)).and_then(|(p, q)| rms_of(p, q)).map(|v| v.0),
        })
        .collect()
}

/// The relations that stand, at most 5000 of them.
fn live_relations(cands: &[Cand]) -> Vec<Relation> {
    let mut relations: Vec<Relation> = cands.iter().filter(|c| c.live()).map(|c| c.rel.clone()).collect();
    relations.truncate(5000);
    relations
}

/// The relations found but not imposed, with their reasons, at most 5000 of them.
fn rejected_relations(cands: &[Cand]) -> Vec<RejectedRelation> {
    let mut rejected: Vec<RejectedRelation> = cands
        .iter()
        .filter_map(|c| {
            c.verdict.as_ref().map(|reason| RejectedRelation { relation: c.rel.clone(), reason: reason.clone() })
        })
        .collect();
    rejected.truncate(5000);
    rejected
}

pub(crate) fn auto_constrain(s: &mut Session, i: AutoConstrainIn) -> CmdResult<AutoConstrainOut> {
    let angle_tol = i.angle_tol_deg.unwrap_or(3.0);
    let length_tol = i.length_tol.unwrap_or(0.2);
    check_tolerances(angle_tol, length_tol, i.noise_sigma)?;
    // mirror planes given by the caller: references, not parts
    let given = given_mirrors(s, &i)?;
    let (prims, names) = collect_primitives(s, &i, &given)?;
    let apply = i.apply.unwrap_or(true);

    // sigma before: the RMS of each primitive's region, and the noise it implies
    let (pts, before) = region_samples(s, &prims);
    let sigma = estimate_sigma(s, &prims, &before, i.noise_sigma);
    let limit = RMS_FLOOR.max(2.0 * sigma);
    let gate = Gate { pts: &pts, before: &before, names: &names, limit };

    // angular, coaxial and equal-radius relations, each checked alone against the RMS limit
    let mut cands = angular_candidates(&prims, angle_tol, length_tol, &gate);
    // tangency, symmetry and equal distances, found on the primitives with their directions snapped
    let raw: Vec<Primitive> = prims.iter().map(|p| p.1.clone()).collect();
    let s1p = snapped_directions(&prims, &cands);
    let detect = i.detect_symmetry.unwrap_or(true);
    let mirrors = mirror_planes(&given, detect, &s1p, &pts, angle_tol, length_tol);
    cands.extend(tangent_candidates(&prims, &raw, &s1p, &mirrors, angle_tol, length_tol));
    check_new_alone(&mut cands, &s1p, &mirrors, &gate);

    // all together: while snapping everything raises a primitive above the limit (or the
    // relations conflict), drop the relation that matters most for it, and try again
    let snapped = impose_all(&prims, &mut cands, &mirrors, &gate, &raw);
    if apply {
        apply_snapped(s, &prims, &snapped);
    }
    // residuals of the relations that stand, on the primitives as they are now
    let shown: &[Primitive] = if apply { &snapped } else { &raw };
    let (max_angle_error_deg, max_length_error) = largest_residuals(&cands, shown, &mirrors);
    Ok(AutoConstrainOut {
        relations: live_relations(&cands),
        rejected: rejected_relations(&cands),
        primitives: primitive_sigmas(&prims, &names, &pts, &before, &snapped),
        sigma,
        rms_limit: limit,
        applied: apply,
        max_angle_error_deg,
        max_length_error,
    })
}

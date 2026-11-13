//! Building the fitted entities of one chain and measuring their deviation from the samples.

use super::primitive::*;
use super::segment::*;
use super::types::*;
use super::*;

// ---------------------------------------------------------------------------------------------
// the sketch of one chain

/// Sketch of one chain and, per entity, the sample range `(s, len)` it stands for.
pub(super) struct Built {
    pub(super) sk: Sketch,
    pub(super) ranges: Vec<(usize, usize)>,
}

pub(super) fn build(sm: &Samples, pieces: &[Piece], plane: &SketchPlane, tol: f64) -> Option<Built> {
    let n = pieces.len();
    let m = sm.q.len();
    let mut sk = Sketch { plane: plane.clone(), ..Sketch::default() };
    let mut ranges = Vec::new();
    if sm.closed && n == 1 {
        let p = pieces.first()?;
        let Prim::Circle { c, r } = p.fit.prim else { return None };
        let center = sk.add_point(c);
        sk.entities.push(Entity::Circle { center, radius: r });
        ranges.push((p.s, m));
        return Some(Built { sk, ranges });
    }
    let limit = (10.0 * tol).max(1.0);
    let mut pts: Vec<P2> = Vec::with_capacity(n + 1);
    for k in 0..n {
        let cur = pieces.get(k)?;
        let p0 = sm.at(cur.s);
        pts.push(if k == 0 && !sm.closed {
            cur.fit.prim.project(p0)
        } else {
            let prev = &pieces.get((k + n - 1) % n)?.fit.prim;
            tangent_point(prev, &cur.fit.prim, tol)
                .filter(|t| dist(*t, p0) <= limit)
                .unwrap_or_else(|| junction(prev, &cur.fit.prim, p0, limit))
        });
    }
    if !sm.closed {
        let last = pieces.last()?;
        pts.push(last.fit.prim.project(sm.at(last.s + last.len)));
    }
    let ids: Vec<usize> = pts.iter().map(|p| sk.add_point(*p)).collect();
    for (k, p) in pieces.iter().enumerate() {
        let k1 = if sm.closed { (k + 1) % n } else { k + 1 };
        let (a, b) = (*ids.get(k)?, *ids.get(k1)?);
        let (pa, pb) = (*pts.get(k)?, *pts.get(k1)?);
        if a == b || dist(pa, pb) < 1e-9 {
            continue;
        }
        match p.fit.prim {
            Prim::Line { .. } => sk.entities.push(Entity::Line { a, b }),
            Prim::Circle { c, .. } => {
                let center = sk.add_point(c);
                let (sa, sb) = if p.fit.sweep >= 0.0 { (a, b) } else { (b, a) };
                sk.entities.push(Entity::Arc { center, a: sa, b: sb });
            }
        }
        ranges.push((p.s, p.len));
    }
    (!sk.entities.is_empty() && sk.validate().is_ok()).then_some(Built { sk, ranges })
}

pub(super) fn entity_prim(sk: &Sketch, e: usize) -> Option<Prim> {
    let pt = |i: usize| sk.points.get(i).copied();
    match sk.entities.get(e)? {
        Entity::Line { a, b } => {
            let (pa, pb) = (pt(*a)?, pt(*b)?);
            Some(Prim::Line { o: pa, d: unit(sub(pb, pa))? })
        }
        Entity::Arc { center, a, b } => {
            let (c, pa, pb) = (pt(*center)?, pt(*a)?, pt(*b)?);
            Some(Prim::Circle { c, r: 0.5 * (dist(c, pa) + dist(c, pb)) })
        }
        Entity::Circle { center, radius } => Some(Prim::Circle { c: pt(*center)?, r: *radius }),
    }
}

/// Distance of every sample from the entity it was fitted to.
pub(super) fn deviation(sk: &Sketch, ranges: &[(usize, usize)], sm: &Samples) -> Deviation {
    let (mut sum, mut max, mut count) = (0.0, 0.0f64, 0usize);
    for (e, (s, len)) in ranges.iter().enumerate() {
        let Some(prim) = entity_prim(sk, e) else { continue };
        for i in *s..=*s + *len {
            let d = prim.dist(sm.at(i));
            sum += d * d;
            max = max.max(d);
            count += 1;
        }
    }
    let rms = if count == 0 { 0.0 } else { (sum / count as f64).sqrt() };
    Deviation { rms: if rms.is_finite() { rms } else { f64::MAX }, max: if max.is_finite() { max } else { f64::MAX } }
}

/// Signed distances of the samples from their entities (at most about 1500 of them), the data
/// term of the final least-squares polish.
fn data_residuals(sk: &Sketch, ranges: &[(usize, usize)], sm: &Samples) -> Vec<f64> {
    let total: usize = ranges.iter().map(|r| r.1 + 1).sum();
    let stride = (total / 1500).max(1);
    let mut out = Vec::with_capacity(total / stride + ranges.len());
    for (e, (s, len)) in ranges.iter().enumerate() {
        let Some(prim) = entity_prim(sk, e) else { continue };
        for i in (0..=*len).step_by(stride) {
            out.push(prim.signed(sm.at(s + i)).0);
        }
    }
    out
}

/// Least-squares polish under the constraints; keeps the sketch when that does not fit better.
pub(super) fn polished(sk: &Sketch, ranges: &[(usize, usize)], sm: &Samples) -> Option<Sketch> {
    let mut p = sk.clone();
    polish(&mut p, &|t| data_residuals(t, ranges, sm)).ok()?;
    let mut check = p.clone();
    let rep = solve(&mut check).ok()?;
    let ok = matches!(rep.status, SolveStatus::FullyDefined | SolveStatus::UnderDefined) && rep.residual < 1e-7;
    let (before, after) = (deviation(sk, ranges, sm), deviation(&p, ranges, sm));
    (ok && after.rms <= before.rms && after.rms.is_finite()).then_some(p)
}

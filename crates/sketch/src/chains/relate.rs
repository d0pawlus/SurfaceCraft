//! Candidate relations between sketch entities and their acceptance into the sketch.

use super::build::*;
use super::primitive::*;
use super::segment::*;
use super::types::*;
use super::*;

// ---------------------------------------------------------------------------------------------
// relations

struct Cand {
    kind: RelationKind,
    entities: Vec<usize>,
    cons: Vec<Constraint>,
    /// Tried when `cons` is redundant (collinear lines that are already parallel).
    fallback: Option<Vec<Constraint>>,
}

fn a1_of(sk: &Sketch, e: usize) -> usize {
    sk.endpoints(e).map_or(0, |(a, _)| a)
}

fn shared_points(sk: &Sketch, e1: usize, e2: usize) -> Vec<usize> {
    let (Some((a1, b1)), Some((a2, b2))) = (sk.endpoints(e1), sk.endpoints(e2)) else { return Vec::new() };
    let mut v = Vec::new();
    for p in [a1, b1] {
        if (p == a2 || p == b2) && !v.contains(&p) {
            v.push(p);
        }
    }
    v
}

/// Relations the fitted geometry suggests, most important first.
fn candidates(sk: &Sketch, tol: f64, opts: &ChainOptions) -> Vec<Cand> {
    let ne = sk.entities.len();
    let prims: Vec<Option<Prim>> = (0..ne).map(|e| entity_prim(sk, e)).collect();
    let ang = opts.angle_tolerance_degrees.to_radians();
    let tan_ang = (3.0 * ang).min(0.5);
    let dir = |e: usize| match prims.get(e) {
        Some(Some(Prim::Line { d, .. })) => Some(*d),
        _ => None,
    };
    let circ = |e: usize| match prims.get(e) {
        Some(Some(Prim::Circle { c, r })) => Some((*c, *r)),
        _ => None,
    };
    let pt = |i: usize| sk.points.get(i).copied();
    let center_of = |e: usize| match sk.entities.get(e) {
        Some(Entity::Arc { center, .. } | Entity::Circle { center, .. }) => Some(*center),
        _ => None,
    };
    let (mut hv, mut tangent, mut concentric, mut collinear) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let (mut equal, mut parallel, mut perpendicular) = (Vec::new(), Vec::new(), Vec::new());
    let single = |kind, entities: Vec<usize>, c: Constraint| Cand { kind, entities, cons: vec![c], fallback: None };
    for e in 0..ne {
        if let Some(d) = dir(e) {
            if d[1].abs().asin() <= ang {
                hv.push(single(RelationKind::Horizontal, vec![e], Constraint::Horizontal { line: e }));
            } else if d[0].abs().asin() <= ang {
                hv.push(single(RelationKind::Vertical, vec![e], Constraint::Vertical { line: e }));
            }
        }
    }
    for e1 in 0..ne {
        for e2 in e1 + 1..ne {
            // tangency at a shared end point
            let shared = shared_points(sk, e1, e2);
            let tangent_at = |j: P2| -> bool {
                match (dir(e1), dir(e2), circ(e1), circ(e2)) {
                    (Some(d), _, _, Some((c, _))) | (_, Some(d), Some((c, _)), _) => {
                        unit(sub(j, c)).is_some_and(|u| dot(d, u).abs().asin() <= tan_ang)
                    }
                    (_, _, Some((c1, r1)), Some((c2, r2))) => {
                        dist(c1, c2) > 4.0 * tol
                            && r1.min(r2) > tol
                            && unit(sub(j, c1))
                                .zip(unit(sub(j, c2)))
                                .is_some_and(|(u1, u2)| cross(u1, u2).abs().asin() <= tan_ang)
                    }
                    _ => false,
                }
            };
            if shared.iter().filter_map(|&i| pt(i)).any(tangent_at) {
                // the line (or, between two arcs, the first arc) goes first
                let (first, second) = if dir(e2).is_some() && dir(e1).is_none() { (e2, e1) } else { (e1, e2) };
                tangent.push(single(
                    RelationKind::Tangent,
                    vec![first, second],
                    Constraint::Tangent { line: first, entity: second },
                ));
            }
            // concentric and equal radius
            if let (Some((c1, r1)), Some((c2, r2))) = (circ(e1), circ(e2)) {
                if let (Some(i1), Some(i2)) = (center_of(e1), center_of(e2))
                    && i1 != i2
                    && dist(c1, c2) <= tol
                {
                    concentric.push(single(
                        RelationKind::Concentric,
                        vec![e1, e2],
                        Constraint::Coincident { a: i1, b: i2 },
                    ));
                }
                if (r1 - r2).abs() <= tol + 0.02 * r1.min(r2) {
                    equal.push(single(RelationKind::EqualRadius, vec![e1, e2], Constraint::Equal { a: e1, b: e2 }));
                }
            }
            // lines
            if let (Some(d1), Some(d2), Some((_, _)), Some((a2, b2))) =
                (dir(e1), dir(e2), sk.endpoints(e1), sk.endpoints(e2))
            {
                if cross(d1, d2).abs().asin() <= ang {
                    let offset = pt(a1_of(sk, e1)).zip(pt(a2)).zip(pt(b2)).map(|((p1, q1), q2)| {
                        let line1 = Prim::Line { o: p1, d: d1 };
                        line1.dist(q1).max(line1.dist(q2))
                    });
                    if offset.is_some_and(|dv| dv <= 2.0 * tol) && shared.is_empty() {
                        collinear.push(Cand {
                            kind: RelationKind::Collinear,
                            entities: vec![e1, e2],
                            cons: vec![
                                Constraint::Parallel { a: e1, b: e2 },
                                Constraint::PointOnLine { point: a2, line: e1 },
                            ],
                            fallback: Some(vec![Constraint::PointOnLine { point: a2, line: e1 }]),
                        });
                    } else {
                        parallel.push(single(
                            RelationKind::Parallel,
                            vec![e1, e2],
                            Constraint::Parallel { a: e1, b: e2 },
                        ));
                    }
                } else if dot(d1, d2).abs().asin() <= ang {
                    perpendicular.push(single(
                        RelationKind::Perpendicular,
                        vec![e1, e2],
                        Constraint::Perpendicular { a: e1, b: e2 },
                    ));
                }
            }
        }
    }
    let mut all = hv;
    all.extend(tangent);
    all.extend(concentric);
    all.extend(collinear);
    all.extend(equal);
    all.extend(parallel);
    all.extend(perpendicular);
    all
}

/// Outcome of the relations of one chain.
pub(super) struct Related {
    pub(super) sk: Sketch,
    pub(super) kept: Vec<Relation>,
    pub(super) rejected: Vec<RejectedRelation>,
    pub(super) rejected_total: usize,
    pub(super) status: Option<SolveStatus>,
    pub(super) dof: usize,
    pub(super) residual: f64,
    pub(super) iterations: usize,
}

fn try_add(
    cur: &Sketch,
    cons: &[Constraint],
    ranges: &[(usize, usize)],
    sm: &Samples,
    limits: (f64, f64),
) -> Result<Sketch, String> {
    let mut trial = cur.clone();
    trial.constraints.extend(cons.iter().cloned());
    let rep = solve(&mut trial).map_err(|e| format!("invalid: {e}"))?;
    match rep.status {
        SolveStatus::OverDefined if rep.conflicting.is_empty() => {
            return Err("implied by the relations already kept (redundant)".into());
        }
        SolveStatus::OverDefined => return Err("contradicts the relations already kept".into()),
        SolveStatus::Failed => return Err("cannot be satisfied together with the relations kept".into()),
        SolveStatus::FullyDefined | SolveStatus::UnderDefined => {}
    }
    let mut d = deviation(&trial, ranges, sm);
    if d.rms > limits.0 || d.max > limits.1 {
        // the least-norm solution may be far from the data although a better one exists
        if let Some(p) = polished(&trial, ranges, sm) {
            trial = p;
            d = deviation(&trial, ranges, sm);
        }
    }
    if d.rms > limits.0 || d.max > limits.1 {
        return Err(format!(
            "moves the chain too far from the section (rms {:.3} mm, max {:.3} mm after solving)",
            d.rms, d.max
        ));
    }
    Ok(trial)
}

pub(super) fn relate(built: &Built, sm: &Samples, tol: f64, opts: &ChainOptions, first_entity: usize) -> Related {
    let mut cur = built.sk.clone();
    let (mut kept, mut rejected, mut rejected_total) = (Vec::new(), Vec::new(), 0usize);
    let base = deviation(&cur, &built.ranges, sm);
    let limits = ((RMS_FACTOR * tol).max(1.25 * base.rms), (MAX_FACTOR * tol).max(1.25 * base.max));
    let global = |e: &[usize]| e.iter().map(|k| k + first_entity).collect::<Vec<_>>();
    for (idx, cand) in candidates(&built.sk, tol, opts).into_iter().enumerate() {
        let result = if idx >= MAX_CANDIDATES {
            Err("not tried: more than 300 relations were proposed for this chain".to_string())
        } else {
            match try_add(&cur, &cand.cons, &built.ranges, sm, limits) {
                Ok(s) => Ok(s),
                Err(first) => match &cand.fallback {
                    Some(fb) if first.contains("redundant") => try_add(&cur, fb, &built.ranges, sm, limits),
                    _ => Err(first),
                },
            }
        };
        match result {
            Ok(s) => {
                cur = s;
                kept.push(Relation { kind: cand.kind, entities: global(&cand.entities) });
            }
            Err(reason) => {
                rejected_total += 1;
                if rejected.len() < MAX_REJECTED_LISTED {
                    rejected.push(RejectedRelation { kind: cand.kind, entities: global(&cand.entities), reason });
                }
            }
        }
    }
    if !kept.is_empty()
        && let Some(p) = polished(&cur, &built.ranges, sm)
    {
        cur = p;
    }
    let (mut status, mut dof, mut residual, mut iterations) = (None, 0, 0.0, 0);
    let mut last = cur.clone();
    if let Ok(rep) = solve(&mut last) {
        status = Some(rep.status);
        dof = rep.dof;
        residual = rep.residual;
        iterations = rep.iterations;
        cur = last;
    }
    Related { sk: cur, kept, rejected, rejected_total, status, dof, residual, iterations }
}

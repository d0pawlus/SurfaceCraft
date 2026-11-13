//! Whole section chains fitted into constrained sketches (design doc §6.4).
//!
//! Each chain (a polyline of a mesh section) becomes a connected sequence of lines and arcs:
//!
//! 1. *Split and merge* (Ramer–Douglas–Peucker splitting, then greedy merging of neighbours whose
//!    union is still one primitive, then absorbing slivers and sliding every boundary to the
//!    least-squares optimum). A piece is a line or an arc when its RMS deviation is within the
//!    tolerance and its maximum within three times the tolerance; an arc must also deviate from its
//!    chord by more than twice the tolerance, so a flat arc stays a line.
//! 2. The sketch frame is turned by the dominant line direction (when the lines lie within
//!    [`MAX_FRAME_OFFSET_DEGREES`] of the axes), so a part scanned a little off the plane axes
//!    still gets horizontal and vertical lines.
//! 3. Neighbouring entities share their junction point (the point nearest to both fitted
//!    primitives); a closed chain closes, an open one stays open.
//! 4. Relations (horizontal, vertical, tangent, concentric, collinear, equal radius, parallel,
//!    perpendicular) are proposed from the fitted geometry in that priority order and added one by
//!    one: a relation is kept only when the solver still converges, the sketch is not
//!    over-defined (no redundant relation is added) and the chain stays within the tolerance of
//!    the section points.

mod build;
mod primitive;
mod relate;
mod segment;
#[cfg(test)]
mod tests;
mod types;

use build::*;
use primitive::*;
use relate::*;
use segment::*;

pub use types::{Chain2, ChainOptions, ChainReport, ChainsFit, Deviation, RejectedRelation, Relation, RelationKind};

use crate::{Constraint, Entity, Sketch, SketchPlane, SolveReport, SolveStatus, polish, solve};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

type P2 = [f64; 2];

/// Largest number of samples per chain that is fitted (longer chains are thinned evenly).
pub const MAX_SAMPLES: usize = 8000;
/// Chains with more entities than this get their lines and arcs but no relations.
pub const MAX_RELATION_ENTITIES: usize = 40;
/// Entities of all chains together; further chains are skipped.
pub const MAX_TOTAL_ENTITIES: usize = 500;
/// Lines whose direction is within this many degrees of an axis of the sketch frame turn the frame.
pub const MAX_FRAME_OFFSET_DEGREES: f64 = 5.0;
const MAX_PIECES: usize = 1500;
const MAX_CANDIDATES: usize = 300;
const MAX_REJECTED_LISTED: usize = 200;
const FIT_SAMPLES: usize = 256;
const RMS_FACTOR: f64 = 1.0;
const MAX_FACTOR: f64 = 3.0;
/// An arc must stand off its chord by more than this many tolerances, else it is a line.
const FLAT: f64 = 2.0;

// ---------------------------------------------------------------------------------------------
// the whole fit

/// Worst-first order of solve outcomes (no outcome counts as failed).
fn rank(s: Option<SolveStatus>) -> u8 {
    match s {
        Some(SolveStatus::FullyDefined) => 0,
        Some(SolveStatus::UnderDefined) => 1,
        Some(SolveStatus::Failed) | None => 2,
        Some(SolveStatus::OverDefined) => 3,
    }
}

fn rotate(p: P2, c: f64, s: f64) -> P2 {
    [c * p[0] + s * p[1], -s * p[0] + c * p[1]]
}

/// The turn of the frame (radians): length-weighted mean offset of the lines from the axes.
fn frame_angle(lines: &[(f64, f64)]) -> f64 {
    let limit = MAX_FRAME_OFFSET_DEGREES.to_radians();
    let (mut sum, mut weight) = (0.0, 0.0);
    for (phi, w) in lines {
        let off = (phi + PI / 4.0).rem_euclid(PI / 2.0) - PI / 4.0;
        if off.abs() <= limit && w.is_finite() && *w > 0.0 {
            sum += w * off;
            weight += w;
        }
    }
    if weight > 0.0 { sum / weight } else { 0.0 }
}

fn skipped(index: usize, closed: bool, samples: usize, first_entity: usize, note: &str) -> ChainReport {
    let zero = Deviation { rms: 0.0, max: 0.0 };
    ChainReport {
        chain: index,
        closed,
        samples,
        first_entity,
        entities: 0,
        lines: 0,
        arcs: 0,
        circles: 0,
        relations_kept: Vec::new(),
        relations_rejected: Vec::new(),
        rejected_total: 0,
        before: zero,
        after: zero,
        status: None,
        dof: 0,
        note: Some(note.to_string()),
    }
}

/// Fits every chain into lines and arcs of one sketch on `plane` (turned to the part's axes when
/// `opts.align_frame` is set) and constrains it with the relations that hold within the tolerance.
#[must_use]
pub fn fit_chains(plane: &SketchPlane, chains: &[Chain2], opts: &ChainOptions) -> ChainsFit {
    let tol = if opts.tolerance.is_finite() && opts.tolerance > 0.0 { opts.tolerance.min(1e9) } else { 0.05 };
    let opts = ChainOptions {
        tolerance: tol,
        angle_tolerance_degrees: if opts.angle_tolerance_degrees.is_finite() {
            opts.angle_tolerance_degrees.clamp(0.001, 20.0)
        } else {
            1.0
        },
        align_frame: opts.align_frame,
    };
    // 1. segment every chain in the input frame
    let mut prepared: Vec<(usize, Result<(Samples, Vec<Piece>), String>)> = Vec::new();
    let mut lines: Vec<(f64, f64)> = Vec::new();
    for (i, c) in chains.iter().enumerate() {
        let r = segment(c, tol);
        if let Ok((sm, pieces)) = &r {
            for p in pieces.iter().filter(|p| !p.arc) {
                if let Prim::Line { d, .. } = p.fit.prim {
                    lines.push((d[1].atan2(d[0]), sm.length(p.s, p.len)));
                }
            }
        }
        prepared.push((i, r));
    }
    // 2. turn the frame
    let theta = if opts.align_frame { frame_angle(&lines) } else { 0.0 };
    let (c, s) = (theta.cos(), theta.sin());
    let mut plane = plane.clone();
    if theta != 0.0 {
        let (x, y) = (plane.x, plane.y);
        plane.x = [0, 1, 2].map(|k| c * x[k] + s * y[k]);
        plane.y = [0, 1, 2].map(|k| -s * x[k] + c * y[k]);
    }
    // 3. build, relate, merge
    let mut sketch = Sketch { plane: plane.clone(), ..Sketch::default() };
    let mut reports: Vec<ChainReport> = Vec::new();
    let (mut worst_rank, mut dof, mut residual, mut iterations) = (0u8, 0usize, 0.0f64, 0usize);
    let mut any = false;
    for (index, r) in prepared {
        let closed = chains.get(index).is_some_and(|c| c.closed);
        let first_entity = sketch.entities.len();
        let (mut sm, mut pieces) = match r {
            Ok(v) => v,
            Err(note) => {
                reports.push(skipped(index, closed, 0, first_entity, &note));
                continue;
            }
        };
        sm.q = sm.q.iter().map(|p| rotate(*p, c, s)).collect();
        refit_pieces(&sm, &mut pieces);
        let Some(built) = build(&sm, &pieces, &plane, tol) else {
            reports.push(skipped(index, sm.closed, sm.q.len(), first_entity, "no lines or arcs could be fitted"));
            continue;
        };
        if sketch.entities.len() + built.sk.entities.len() > MAX_TOTAL_ENTITIES {
            reports.push(skipped(
                index,
                sm.closed,
                sm.q.len(),
                first_entity,
                "skipped: the sketch would exceed the entity limit",
            ));
            continue;
        }
        let before = deviation(&built.sk, &built.ranges, &sm);
        let (related, note) = if built.sk.entities.len() > MAX_RELATION_ENTITIES {
            let note = format!(
                "{} entities (more than {MAX_RELATION_ENTITIES}): fitted, but no relations were inferred",
                built.sk.entities.len()
            );
            let mut sk = built.sk.clone();
            let rep = solve(&mut sk).ok();
            (
                Related {
                    sk: built.sk.clone(),
                    kept: Vec::new(),
                    rejected: Vec::new(),
                    rejected_total: 0,
                    status: rep.as_ref().map(|r| r.status),
                    dof: rep.as_ref().map_or(0, |r| r.dof),
                    residual: rep.as_ref().map_or(0.0, |r| r.residual),
                    iterations: rep.as_ref().map_or(0, |r| r.iterations),
                },
                Some(note),
            )
        } else {
            (relate(&built, &sm, tol, &opts, first_entity), None)
        };
        let after = deviation(&related.sk, &built.ranges, &sm);
        let count = |f: fn(&Entity) -> bool| related.sk.entities.iter().filter(|e| f(e)).count();
        let (nl, na, nc) = (
            count(|e| matches!(e, Entity::Line { .. })),
            count(|e| matches!(e, Entity::Arc { .. })),
            count(|e| matches!(e, Entity::Circle { .. })),
        );
        reports.push(ChainReport {
            chain: index,
            closed: sm.closed,
            samples: sm.q.len(),
            first_entity,
            entities: related.sk.entities.len(),
            lines: nl,
            arcs: na,
            circles: nc,
            relations_kept: related.kept,
            relations_rejected: related.rejected,
            rejected_total: related.rejected_total,
            before,
            after,
            status: related.status,
            dof: related.dof,
            note,
        });
        // aggregate
        any = true;
        dof += related.dof;
        residual = residual.max(related.residual);
        iterations += related.iterations;
        worst_rank = worst_rank.max(rank(related.status));
        // append to the sketch
        let (dp, de) = (sketch.points.len(), sketch.entities.len());
        sketch.points.extend(related.sk.points.iter().copied());
        for e in &related.sk.entities {
            sketch.entities.push(match e {
                Entity::Line { a, b } => Entity::Line { a: a + dp, b: b + dp },
                Entity::Arc { center, a, b } => Entity::Arc { center: center + dp, a: a + dp, b: b + dp },
                Entity::Circle { center, radius } => Entity::Circle { center: center + dp, radius: *radius },
            });
        }
        sketch.constraints.extend(related.sk.constraints.iter().map(|k| k.shifted(dp, de)));
    }
    let status = if !any {
        SolveStatus::Failed
    } else {
        match worst_rank {
            0 => SolveStatus::FullyDefined,
            1 => SolveStatus::UnderDefined,
            2 => SolveStatus::Failed,
            _ => SolveStatus::OverDefined,
        }
    };
    let n_ok = reports.iter().filter(|r| r.entities > 0).count();
    let report = SolveReport {
        status,
        dof,
        residual,
        iterations,
        redundant: Vec::new(),
        conflicting: Vec::new(),
        message: format!("{n_ok} of {} chains fitted; {dof} degrees of freedom left", reports.len()),
    };
    ChainsFit { sketch, frame_rotation_degrees: theta.to_degrees(), chains: reports, report }
}

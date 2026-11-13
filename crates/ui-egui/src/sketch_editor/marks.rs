//! The marks drawn next to constraints in the viewport: their text, where they sit and which entities
//! they tie.
use super::dimension::*;
use super::snap::{arc_geom, circle_geom};
use surfacecraft_sketch::{Constraint, Entity, Sketch};

/// The short mark drawn next to a constraint in the viewport.
#[must_use]
pub fn constraint_mark(c: &Constraint) -> String {
    match c {
        Constraint::Fix { .. } => "Fix".into(),
        Constraint::Coincident { .. } => "C".into(),
        Constraint::Horizontal { .. } => "H".into(),
        Constraint::Vertical { .. } => "V".into(),
        Constraint::Parallel { .. } => "//".into(),
        Constraint::Perpendicular { .. } => "_|_".into(),
        Constraint::Equal { .. } => "=".into(),
        Constraint::PointOnLine { .. } | Constraint::PointOnCircle { .. } => "On".into(),
        Constraint::Tangent { .. } => "T".into(),
        Constraint::Midpoint { .. } => "Mid".into(),
        Constraint::Length { value, .. } | Constraint::Distance { value, .. } => fmt_value(*value),
        Constraint::Radius { value, .. } => format!("R{}", fmt_value(*value)),
        Constraint::Angle { degrees, .. } => format!("{}\u{b0}", fmt_value(*degrees)),
    }
}

/// Where a constraint's mark is drawn (sketch coordinates): the points or entities it ties,
/// one anchor per thing (a dimension: a single one).
#[must_use]
pub fn constraint_anchors(sk: &Sketch, c: &Constraint) -> Vec<[f64; 2]> {
    let pt = |i: usize| sk.points.get(i).copied();
    let mid = |e: usize| match sk.entities.get(e)? {
        Entity::Line { a, b } => {
            let (pa, pb) = (sk.points.get(*a)?, sk.points.get(*b)?);
            Some([(pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0])
        }
        _ => None,
    };
    let on_circle = |e: usize| {
        let (c, r) = circle_geom(sk, e)?;
        let a = arc_geom(sk, e).map_or(std::f64::consts::FRAC_PI_4, |g| g.a0 + g.sweep / 2.0);
        Some([c[0] + r * a.cos(), c[1] + r * a.sin()])
    };
    let v: Vec<Option<[f64; 2]>> = match *c {
        Constraint::Fix { point, .. } => vec![pt(point)],
        Constraint::Coincident { a, b } => {
            vec![pt(a).zip(pt(b)).map(|(p, q)| [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0])]
        }
        Constraint::Horizontal { line } | Constraint::Vertical { line } | Constraint::Length { line, .. } => {
            vec![mid(line)]
        }
        Constraint::Distance { a, b, .. } => {
            vec![pt(a).zip(pt(b)).map(|(p, q)| [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0])]
        }
        Constraint::Parallel { a, b } | Constraint::Perpendicular { a, b } | Constraint::Equal { a, b } => {
            vec![mid(a), mid(b)]
        }
        Constraint::Angle { a, b, .. } => {
            vec![mid(a).zip(mid(b)).map(|(p, q)| [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0])]
        }
        Constraint::Radius { entity, .. } => vec![on_circle(entity)],
        Constraint::PointOnLine { point, .. }
        | Constraint::PointOnCircle { point, .. }
        | Constraint::Midpoint { point, .. } => {
            vec![pt(point)]
        }
        Constraint::Tangent { line, entity } => vec![mid(line).or_else(|| on_circle(entity))],
    };
    v.into_iter().flatten().collect()
}

/// The entities a constraint ties (points excluded).
#[must_use]
pub fn constraint_entities(c: &Constraint) -> Vec<usize> {
    match *c {
        Constraint::Horizontal { line }
        | Constraint::Vertical { line }
        | Constraint::Length { line, .. }
        | Constraint::PointOnLine { line, .. }
        | Constraint::Midpoint { line, .. } => vec![line],
        Constraint::Parallel { a, b }
        | Constraint::Perpendicular { a, b }
        | Constraint::Equal { a, b }
        | Constraint::Angle { a, b, .. } => {
            vec![a, b]
        }
        Constraint::Radius { entity, .. } | Constraint::PointOnCircle { entity, .. } => vec![entity],
        Constraint::Tangent { line, entity } => vec![line, entity],
        Constraint::Fix { .. } | Constraint::Coincident { .. } | Constraint::Distance { .. } => Vec::new(),
    }
}

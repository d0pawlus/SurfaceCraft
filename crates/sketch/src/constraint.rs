//! Geometric constraints and their residual equations.
use crate::{Entity, Sketch};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// A constraint. Point and entity fields are indices into the sketch.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Constraint {
    /// Point fixed at `at` (2 equations).
    Fix {
        /// Point.
        point: usize,
        /// Position.
        at: [f64; 2],
    },
    /// Two points coincide (2).
    Coincident {
        /// First point.
        a: usize,
        /// Second point.
        b: usize,
    },
    /// Line parallel to x (1).
    Horizontal {
        /// Line entity.
        line: usize,
    },
    /// Line parallel to y (1).
    Vertical {
        /// Line entity.
        line: usize,
    },
    /// Distance between two points (1).
    Distance {
        /// First point.
        a: usize,
        /// Second point.
        b: usize,
        /// Distance, mm.
        value: f64,
    },
    /// Length of a line (1).
    Length {
        /// Line entity.
        line: usize,
        /// Length, mm.
        value: f64,
    },
    /// Two lines parallel (1).
    Parallel {
        /// First line.
        a: usize,
        /// Second line.
        b: usize,
    },
    /// Two lines perpendicular (1).
    Perpendicular {
        /// First line.
        a: usize,
        /// Second line.
        b: usize,
    },
    /// Signed angle from line `a` to line `b`, degrees (1).
    Angle {
        /// First line.
        a: usize,
        /// Second line.
        b: usize,
        /// Angle, degrees.
        degrees: f64,
    },
    /// Two lines of equal length, or two arcs/circles of equal radius (1).
    Equal {
        /// First line, arc or circle.
        a: usize,
        /// Second entity (a line when `a` is a line, else an arc or circle).
        b: usize,
    },
    /// Radius of an arc or circle (1).
    Radius {
        /// Arc or circle entity.
        entity: usize,
        /// Radius, mm.
        value: f64,
    },
    /// Point on the infinite line through a line entity (1).
    PointOnLine {
        /// Point.
        point: usize,
        /// Line entity.
        line: usize,
    },
    /// Point on an arc's or circle's circle (1).
    PointOnCircle {
        /// Point.
        point: usize,
        /// Arc or circle entity.
        entity: usize,
    },
    /// Line tangent to an arc or circle, or (when `line` is an arc or circle) two arcs or circles
    /// tangent to each other, externally or internally, whichever is nearer (1).
    Tangent {
        /// Line entity, or an arc or circle entity.
        line: usize,
        /// Arc or circle entity.
        entity: usize,
    },
    /// Point at the middle of a line (2).
    Midpoint {
        /// Point.
        point: usize,
        /// Line entity.
        line: usize,
    },
}

/// Unknowns: two per point, then one radius per circle (in entity order).
pub(crate) struct Layout {
    pub radius_var: Vec<Option<usize>>,
    pub n: usize,
}

impl Layout {
    pub(crate) fn of(s: &Sketch) -> Self {
        let mut n = s.points.len() * 2;
        let radius_var = s
            .entities
            .iter()
            .map(|e| {
                if matches!(e, Entity::Circle { .. }) {
                    n += 1;
                    Some(n - 1)
                } else {
                    None
                }
            })
            .collect();
        Self { radius_var, n }
    }
}

fn p(x: &[f64], i: usize) -> (f64, f64) {
    (x.get(2 * i).copied().unwrap_or(0.0), x.get(2 * i + 1).copied().unwrap_or(0.0))
}

fn line(s: &Sketch, e: usize) -> Option<(usize, usize)> {
    match s.entities.get(e)? {
        Entity::Line { a, b } => Some((*a, *b)),
        _ => None,
    }
}

/// Centre point and radius (from unknowns) of an arc or circle.
fn circle(s: &Sketch, l: &Layout, x: &[f64], e: usize) -> Option<((f64, f64), f64)> {
    match s.entities.get(e)? {
        Entity::Arc { center, a, .. } => {
            let c = p(x, *center);
            let q = p(x, *a);
            Some((c, (q.0 - c.0).hypot(q.1 - c.1)))
        }
        Entity::Circle { center, .. } => {
            let r = l.radius_var.get(e).copied().flatten().and_then(|k| x.get(k)).copied()?;
            Some((p(x, *center), r))
        }
        Entity::Line { .. } => None,
    }
}

/// A point that is an end point of both entities (lines and arcs).
fn shared_end(s: &Sketch, e1: usize, e2: usize) -> Option<usize> {
    let ((a1, b1), (a2, b2)) = (s.endpoints(e1)?, s.endpoints(e2)?);
    [a1, b1].into_iter().find(|k| *k == a2 || *k == b2)
}

fn dir(x: &[f64], a: usize, b: usize) -> (f64, f64) {
    let (pa, pb) = (p(x, a), p(x, b));
    (pb.0 - pa.0, pb.1 - pa.1)
}

impl Constraint {
    /// Number of equations.
    #[must_use]
    pub fn arity(&self) -> usize {
        match self {
            Constraint::Fix { .. } | Constraint::Coincident { .. } | Constraint::Midpoint { .. } => 2,
            _ => 1,
        }
    }

    pub(crate) fn validate(&self, s: &Sketch) -> Result<(), String> {
        let np = s.points.len();
        let pt = |i: usize| if i < np { Ok(()) } else { Err(format!("point {i} does not exist")) };
        let ln = |e: usize| line(s, e).map(|_| ()).ok_or_else(|| format!("entity {e} is not a line"));
        let cir = |e: usize| match s.entities.get(e) {
            Some(Entity::Arc { .. } | Entity::Circle { .. }) => Ok(()),
            _ => Err(format!("entity {e} is not an arc or circle")),
        };
        let fin = |v: f64| if v.is_finite() { Ok(()) } else { Err("value is not finite".to_string()) };
        match self {
            Constraint::Fix { point, at } => {
                pt(*point)?;
                fin(at[0])?;
                fin(at[1])
            }
            Constraint::Coincident { a, b } => pt(*a).and(pt(*b)),
            Constraint::Horizontal { line } | Constraint::Vertical { line } => ln(*line),
            Constraint::Distance { a, b, value } => {
                pt(*a)?;
                pt(*b)?;
                fin(*value)
            }
            Constraint::Length { line, value } => {
                ln(*line)?;
                fin(*value)
            }
            Constraint::Parallel { a, b } | Constraint::Perpendicular { a, b } => ln(*a).and(ln(*b)),
            Constraint::Equal { a, b } => {
                if line(s, *a).is_some() {
                    ln(*a).and(ln(*b))
                } else {
                    cir(*a).and(cir(*b))
                }
            }
            Constraint::Angle { a, b, degrees } => {
                ln(*a)?;
                ln(*b)?;
                fin(*degrees)
            }
            Constraint::Radius { entity, value } => {
                cir(*entity)?;
                if value.is_finite() && *value > 0.0 { Ok(()) } else { Err("radius must be positive".into()) }
            }
            Constraint::PointOnLine { point, line } | Constraint::Midpoint { point, line } => pt(*point).and(ln(*line)),
            Constraint::PointOnCircle { point, entity } => pt(*point).and(cir(*entity)),
            Constraint::Tangent { line: e, entity } => {
                if line(s, *e).is_some() {
                    cir(*entity)
                } else {
                    cir(*e).and(cir(*entity))?;
                    if e == entity { Err("a circle cannot be tangent to itself".into()) } else { Ok(()) }
                }
            }
        }
    }

    /// The same constraint for a sketch whose points and entities were appended after `dp` points
    /// and `de` entities (indices shifted).
    #[must_use]
    pub fn shifted(&self, dp: usize, de: usize) -> Constraint {
        use Constraint as C;
        match self.clone() {
            C::Fix { point, at } => C::Fix { point: point + dp, at },
            C::Coincident { a, b } => C::Coincident { a: a + dp, b: b + dp },
            C::Horizontal { line } => C::Horizontal { line: line + de },
            C::Vertical { line } => C::Vertical { line: line + de },
            C::Distance { a, b, value } => C::Distance { a: a + dp, b: b + dp, value },
            C::Length { line, value } => C::Length { line: line + de, value },
            C::Parallel { a, b } => C::Parallel { a: a + de, b: b + de },
            C::Perpendicular { a, b } => C::Perpendicular { a: a + de, b: b + de },
            C::Angle { a, b, degrees } => C::Angle { a: a + de, b: b + de, degrees },
            C::Equal { a, b } => C::Equal { a: a + de, b: b + de },
            C::Radius { entity, value } => C::Radius { entity: entity + de, value },
            C::PointOnLine { point, line } => C::PointOnLine { point: point + dp, line: line + de },
            C::PointOnCircle { point, entity } => C::PointOnCircle { point: point + dp, entity: entity + de },
            C::Tangent { line, entity } => C::Tangent { line: line + de, entity: entity + de },
            C::Midpoint { point, line } => C::Midpoint { point: point + dp, line: line + de },
        }
    }

    /// Residuals at unknowns `x` (zero when satisfied), appended to `out`.
    pub(crate) fn residuals(&self, s: &Sketch, l: &Layout, x: &[f64], out: &mut Vec<f64>) {
        let norm = |d: (f64, f64)| d.0.hypot(d.1).max(1e-12);
        match self {
            Constraint::Fix { point, at } => {
                let q = p(x, *point);
                out.push(q.0 - at[0]);
                out.push(q.1 - at[1]);
            }
            Constraint::Coincident { a, b } => {
                let (qa, qb) = (p(x, *a), p(x, *b));
                out.push(qa.0 - qb.0);
                out.push(qa.1 - qb.1);
            }
            Constraint::Horizontal { line: e } => {
                let (a, b) = line(s, *e).unwrap_or((0, 0));
                out.push(dir(x, a, b).1);
            }
            Constraint::Vertical { line: e } => {
                let (a, b) = line(s, *e).unwrap_or((0, 0));
                out.push(dir(x, a, b).0);
            }
            Constraint::Distance { a, b, value } => out.push(norm(dir(x, *a, *b)) - value),
            Constraint::Length { line: e, value } => {
                let (a, b) = line(s, *e).unwrap_or((0, 0));
                out.push(norm(dir(x, a, b)) - value);
            }
            Constraint::Parallel { a, b } | Constraint::Perpendicular { a, b } | Constraint::Angle { a, b, .. } => {
                let (la, lb) = (line(s, *a).unwrap_or((0, 0)), line(s, *b).unwrap_or((0, 0)));
                let (da, db) = (dir(x, la.0, la.1), dir(x, lb.0, lb.1));
                let (na, nb) = (norm(da), norm(db));
                let cross = (da.0 * db.1 - da.1 * db.0) / (na * nb);
                let dot = (da.0 * db.0 + da.1 * db.1) / (na * nb);
                out.push(match self {
                    Constraint::Parallel { .. } => cross,
                    Constraint::Perpendicular { .. } => dot,
                    Constraint::Angle { degrees, .. } => {
                        let t = degrees.to_radians();
                        cross * t.cos() - dot * t.sin()
                    }
                    _ => 0.0,
                });
            }
            Constraint::Equal { a, b } => {
                if let (Some(la), Some(lb)) = (line(s, *a), line(s, *b)) {
                    out.push(norm(dir(x, la.0, la.1)) - norm(dir(x, lb.0, lb.1)));
                } else {
                    out.push(match (circle(s, l, x, *a), circle(s, l, x, *b)) {
                        (Some((_, ra)), Some((_, rb))) => ra - rb,
                        _ => 0.0,
                    });
                }
            }
            Constraint::Radius { entity, value } => {
                out.push(circle(s, l, x, *entity).map_or(0.0, |(_, r)| r - value));
            }
            Constraint::PointOnLine { point, line: e } => {
                let (a, b) = line(s, *e).unwrap_or((0, 0));
                let d = dir(x, a, b);
                let (pa, q) = (p(x, a), p(x, *point));
                out.push((d.0 * (q.1 - pa.1) - d.1 * (q.0 - pa.0)) / norm(d));
            }
            Constraint::PointOnCircle { point, entity } => {
                let q = p(x, *point);
                out.push(circle(s, l, x, *entity).map_or(0.0, |(c, r)| (q.0 - c.0).hypot(q.1 - c.1) - r));
            }
            Constraint::Tangent { line: e, entity } => {
                // entities that share an end point are tangent there when the radius is perpendicular to
                // the line (or the centres are in line with the point); the distance form below is
                // degenerate (a double root) when the end point lies on the line itself
                let joint = shared_end(s, *e, *entity);
                if let Some((a, b)) = line(s, *e) {
                    let d = dir(x, a, b);
                    if let (Some(j), Some(Entity::Arc { center, .. })) = (joint, s.entities.get(*entity)) {
                        let (pj, c) = (p(x, j), p(x, *center));
                        out.push(((pj.0 - c.0) * d.0 + (pj.1 - c.1) * d.1) / norm(d));
                    } else {
                        let pa = p(x, a);
                        out.push(
                            circle(s, l, x, *entity)
                                .map_or(0.0, |(c, r)| ((d.0 * (c.1 - pa.1) - d.1 * (c.0 - pa.0)) / norm(d)).abs() - r),
                        );
                    }
                } else {
                    // two circles: the centre distance is r1 + r2 (outside) or |r1 − r2| (inside)
                    out.push(match (circle(s, l, x, *e), circle(s, l, x, *entity)) {
                        (Some((c1, r1)), Some((c2, r2))) => {
                            let d = (c2.0 - c1.0).hypot(c2.1 - c1.1);
                            if let (Some(j), Some(Entity::Arc { .. })) = (joint, s.entities.get(*entity)) {
                                let pj = p(x, j);
                                (c2.0 - c1.0) * (pj.1 - c1.1) / d.max(1e-12)
                                    - (c2.1 - c1.1) * (pj.0 - c1.0) / d.max(1e-12)
                            } else {
                                let (outside, inside) = (d - (r1 + r2), d - (r1 - r2).abs());
                                if outside.abs() <= inside.abs() { outside } else { inside }
                            }
                        }
                        _ => 0.0,
                    });
                }
            }
            Constraint::Midpoint { point, line: e } => {
                let (a, b) = line(s, *e).unwrap_or((0, 0));
                let (pa, pb, q) = (p(x, a), p(x, b), p(x, *point));
                out.push(q.0 - 0.5 * (pa.0 + pb.0));
                out.push(q.1 - 0.5 * (pa.1 + pb.1));
            }
        }
    }
}

/// Built-in equations of entities: an arc's ends are equally far from its centre.
pub(crate) fn entity_residuals(s: &Sketch, x: &[f64], out: &mut Vec<f64>) {
    for e in &s.entities {
        if let Entity::Arc { center, a, b } = e {
            let (c, qa, qb) = (p(x, *center), p(x, *a), p(x, *b));
            out.push((qa.0 - c.0).hypot(qa.1 - c.1) - (qb.0 - c.0).hypot(qb.1 - c.1));
        }
    }
}

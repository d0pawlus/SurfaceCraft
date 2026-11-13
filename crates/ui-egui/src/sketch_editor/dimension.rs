//! Dimensions: what a dimension measures, its value as typed, the rubber-band readout, and the value
//! formats.
use super::snap::{circle_geom, dist};
use surfacecraft_sketch::{Constraint, Entity, Sketch};

/// A dimension: what it measures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DimKind {
    /// Length of a line.
    Length {
        /// Line entity.
        line: usize,
    },
    /// Radius of an arc or circle.
    Radius {
        /// Arc or circle entity.
        entity: usize,
    },
    /// Distance between two points.
    Distance {
        /// First point.
        a: usize,
        /// Second point.
        b: usize,
    },
    /// Angle from line `a` to line `b`, degrees.
    Angle {
        /// First line.
        a: usize,
        /// Second line.
        b: usize,
    },
}

impl DimKind {
    /// The dimension a constraint is, if it is one.
    #[must_use]
    pub fn of(c: &Constraint) -> Option<(DimKind, f64)> {
        match *c {
            Constraint::Length { line, value } => Some((DimKind::Length { line }, value)),
            Constraint::Radius { entity, value } => Some((DimKind::Radius { entity }, value)),
            Constraint::Distance { a, b, value } => Some((DimKind::Distance { a, b }, value)),
            Constraint::Angle { a, b, degrees } => Some((DimKind::Angle { a, b }, degrees)),
            _ => None,
        }
    }

    /// The constraint with value `v`.
    #[must_use]
    pub fn build(self, v: f64) -> Constraint {
        match self {
            DimKind::Length { line } => Constraint::Length { line, value: v },
            DimKind::Radius { entity } => Constraint::Radius { entity, value: v },
            DimKind::Distance { a, b } => Constraint::Distance { a, b, value: v },
            DimKind::Angle { a, b } => Constraint::Angle { a, b, degrees: v },
        }
    }

    /// The value now in the sketch.
    #[must_use]
    pub fn measure(self, sk: &Sketch) -> Option<f64> {
        let line_dir = |e: usize| match sk.entities.get(e)? {
            Entity::Line { a, b } => {
                let (pa, pb) = (sk.points.get(*a)?, sk.points.get(*b)?);
                Some([pb[0] - pa[0], pb[1] - pa[1]])
            }
            _ => None,
        };
        match self {
            DimKind::Length { line } => line_dir(line).map(|d| d[0].hypot(d[1])),
            DimKind::Radius { entity } => circle_geom(sk, entity).map(|(_, r)| r),
            DimKind::Distance { a, b } => Some(dist(*sk.points.get(a)?, *sk.points.get(b)?)),
            DimKind::Angle { a, b } => {
                let (da, db) = (line_dir(a)?, line_dir(b)?);
                let cross = da[0] * db[1] - da[1] * db[0];
                let dot = da[0] * db[0] + da[1] * db[1];
                Some(cross.atan2(dot).to_degrees())
            }
        }
    }

    /// The catalogue key of the dimension's name.
    #[must_use]
    pub fn label_key(self) -> &'static str {
        match self {
            DimKind::Length { .. } => "sketch.dim.length",
            DimKind::Radius { .. } => "sketch.dim.radius",
            DimKind::Distance { .. } => "sketch.dim.distance",
            DimKind::Angle { .. } => "sketch.dim.angle",
        }
    }

    pub(super) fn positive_only(self) -> bool {
        !matches!(self, DimKind::Angle { .. })
    }

    pub(super) fn indices_exist(self, sk: &Sketch) -> bool {
        match self {
            DimKind::Length { line } => matches!(sk.entities.get(line), Some(Entity::Line { .. })),
            DimKind::Radius { entity } => {
                matches!(sk.entities.get(entity), Some(Entity::Arc { .. } | Entity::Circle { .. }))
            }
            DimKind::Distance { a, b } => a < sk.points.len() && b < sk.points.len(),
            DimKind::Angle { a, b } => {
                matches!(sk.entities.get(a), Some(Entity::Line { .. }))
                    && matches!(sk.entities.get(b), Some(Entity::Line { .. }))
            }
        }
    }
}

/// A dimension whose value is being typed: new (`index` is `None`) or an existing constraint.
#[derive(Clone, Debug, PartialEq)]
pub struct Pending {
    /// What it measures.
    pub kind: DimKind,
    /// The value text being edited.
    pub text: String,
    /// The constraint being edited (`None`: a new dimension).
    pub index: Option<usize>,
}

/// The editor's live read-out while a gesture has an anchor: length and angle of the rubber
/// band, or the radius of a circle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Readout {
    /// Length or radius, sketch units.
    pub length: f64,
    /// Direction angle, degrees (counter-clockwise from x).
    pub angle: f64,
    /// A radius (circle) rather than a length.
    pub radius: bool,
}
/// Formats a value without trailing zeros (`12.5`, `3`).
#[must_use]
pub fn fmt_value(v: f64) -> String {
    let s = format!("{v:.4}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s.is_empty() || s == "-" || s == "-0" { "0".into() } else { s.into() }
}

/// Parses a typed number (a comma works as the decimal point).
#[must_use]
pub fn parse_value(text: &str) -> Option<f64> {
    let v: f64 = text.trim().replace(',', ".").parse().ok()?;
    v.is_finite().then_some(v)
}

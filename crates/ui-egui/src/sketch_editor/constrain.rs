//! Constraint gestures: the clicks of the constraint tools, dimensions placed and edited, and the
//! removal of a constraint.
use super::dimension::*;
use super::snap::pick;
use super::tool::*;
use super::*;
use serde_json::json;
use surfacecraft_sketch::{Constraint, Entity, Sketch};

impl SketchEditor {
    pub(super) fn click_constrain(&mut self, sk: &Sketch, p: [f64; 2], tol: f64) -> Vec<Cmd> {
        let tool = self.tool;
        let Some(hit) = pick(sk, p, tol, tool.want()) else { return Vec::new() };
        match tool {
            Tool::Horizontal | Tool::Vertical => {
                let Hit::Entity(line) = hit else { return Vec::new() };
                let c = if tool == Tool::Horizontal {
                    Constraint::Horizontal { line }
                } else {
                    Constraint::Vertical { line }
                };
                self.constrain_cmd(&c)
            }
            Tool::Fix => {
                let Hit::Point(point) = hit else { return Vec::new() };
                let Some(at) = sk.points.get(point).copied() else { return Vec::new() };
                self.constrain_cmd(&Constraint::Fix { point, at })
            }
            Tool::Parallel | Tool::Perpendicular | Tool::Equal => {
                let Hit::Entity(b) = hit else { return Vec::new() };
                let Some(Hit::Entity(a)) = self.picks.first().copied() else {
                    self.picks = vec![hit];
                    return Vec::new();
                };
                if a == b {
                    return Vec::new();
                }
                self.picks.clear();
                let c = match tool {
                    Tool::Parallel => Constraint::Parallel { a, b },
                    Tool::Perpendicular => Constraint::Perpendicular { a, b },
                    _ => Constraint::Equal { a, b },
                };
                self.constrain_cmd(&c)
            }
            Tool::Tangent => {
                let Hit::Entity(e) = hit else { return Vec::new() };
                let is_line = |i: usize| matches!(sk.entities.get(i), Some(Entity::Line { .. }));
                match self.picks.first().copied() {
                    Some(Hit::Entity(f)) if is_line(f) != is_line(e) => {
                        self.picks.clear();
                        let (line, entity) = if is_line(e) { (e, f) } else { (f, e) };
                        self.constrain_cmd(&Constraint::Tangent { line, entity })
                    }
                    _ => {
                        self.picks = vec![hit];
                        Vec::new()
                    }
                }
            }
            Tool::Coincident => {
                let Hit::Point(b) = hit else { return Vec::new() };
                match self.picks.first().copied() {
                    Some(Hit::Point(a)) if a != b => {
                        self.picks.clear();
                        self.constrain_cmd(&Constraint::Coincident { a, b })
                    }
                    Some(Hit::Point(_)) => Vec::new(),
                    _ => {
                        self.picks = vec![hit];
                        Vec::new()
                    }
                }
            }
            Tool::Dimension => self.click_dimension(sk, hit),
            Tool::Angle => {
                let Hit::Entity(b) = hit else { return Vec::new() };
                match self.picks.first().copied() {
                    Some(Hit::Entity(a)) if a != b => {
                        self.picks.clear();
                        self.begin_pending(sk, DimKind::Angle { a, b }, None);
                        Vec::new()
                    }
                    Some(Hit::Entity(_)) => Vec::new(),
                    _ => {
                        self.picks = vec![hit];
                        Vec::new()
                    }
                }
            }
            _ => Vec::new(),
        }
    }

    fn click_dimension(&mut self, sk: &Sketch, hit: Hit) -> Vec<Cmd> {
        match hit {
            Hit::Point(b) => match self.picks.first().copied() {
                Some(Hit::Point(a)) if a != b => {
                    self.picks.clear();
                    self.begin_pending(sk, DimKind::Distance { a, b }, None);
                }
                Some(Hit::Point(_)) => {}
                _ => self.picks = vec![hit],
            },
            Hit::Entity(e) => {
                self.picks.clear();
                let kind = match sk.entities.get(e) {
                    Some(Entity::Line { .. }) => Some(DimKind::Length { line: e }),
                    Some(Entity::Arc { .. } | Entity::Circle { .. }) => Some(DimKind::Radius { entity: e }),
                    None => None,
                };
                if let Some(k) = kind {
                    self.begin_pending(sk, k, None);
                }
            }
        }
        Vec::new()
    }

    fn begin_pending(&mut self, sk: &Sketch, kind: DimKind, index: Option<usize>) {
        let text = kind.measure(sk).map_or_else(String::new, fmt_value);
        self.pending = Some(Pending { kind, text, index });
    }

    /// Opens constraint `index` for editing when it is a dimension (double click on its label).
    pub fn begin_edit(&mut self, sk: &Sketch, index: usize) -> bool {
        let Some((kind, value)) = sk.constraints.get(index).and_then(DimKind::of) else { return false };
        self.cancel_gesture();
        self.pending = Some(Pending { kind, text: fmt_value(value), index: Some(index) });
        true
    }

    /// Enter in the value field: the new dimension (`sketch.constrain`) or the edited one
    /// (`sketch.edit_constraint`). A value that is not a number (or not positive for a length)
    /// keeps the field open and sets a notice.
    pub fn commit_pending(&mut self) -> Vec<Cmd> {
        let Some(p) = self.pending.clone() else { return Vec::new() };
        let Some(v) = parse_value(&p.text).filter(|v| !p.kind.positive_only() || *v > 0.0) else {
            self.notice = Some("sketch.notice.value");
            return Vec::new();
        };
        self.pending = None;
        self.notice = None;
        let c = p.kind.build(v);
        match p.index {
            None => self.constrain_cmd(&c),
            Some(index) => match (self.sketch_ref(), serde_json::to_value(&c)) {
                (Some(id), Ok(cv)) => vec![Cmd {
                    id: "sketch.edit_constraint",
                    params: json!({"sketch": id, "index": index, "constraint": cv}),
                }],
                _ => Vec::new(),
            },
        }
    }

    /// Removes constraint `index` (`sketch.edit_constraint` without a replacement).
    #[must_use]
    pub fn delete_constraint(&self, index: usize) -> Vec<Cmd> {
        match self.sketch_ref() {
            Some(id) => vec![Cmd { id: "sketch.edit_constraint", params: json!({"sketch": id, "index": index}) }],
            None => Vec::new(),
        }
    }
}

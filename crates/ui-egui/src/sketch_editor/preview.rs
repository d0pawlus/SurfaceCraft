//! The rubber band of a gesture in progress, the live readout, and the dimension label under a point.
use super::dimension::*;
use super::snap;
use super::snap::{arc_from_centre, arc_through, dist};
use super::tool::*;
use super::*;

impl SketchEditor {
    /// The dimension label under screen point `(x, y)`, from the labels of the last frame.
    #[must_use]
    pub fn dimension_at(&self, x: f32, y: f32) -> Option<usize> {
        self.labels.iter().find(|(_, r)| x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3]).map(|(i, _)| *i)
    }

    /// The rubber band of the gesture in progress, as polylines in sketch coordinates, for the
    /// cursor at `hover`.
    #[must_use]
    pub fn preview(&self, hover: [f64; 2]) -> Vec<Vec<[f64; 2]>> {
        let g = &self.gesture;
        match (self.tool, g.len()) {
            (Tool::Line, 1) => vec![vec![g[0], hover]],
            (Tool::Polyline, n) if n >= 1 => {
                let mut pts = g.clone();
                pts.push(hover);
                vec![pts]
            }
            (Tool::Circle, 1) => {
                vec![snap::sample_arc(g[0], dist(g[0], hover), 0.0, std::f64::consts::TAU, 72)]
            }
            (Tool::Rect, 1) => {
                vec![vec![g[0], [hover[0], g[0][1]], hover, [g[0][0], hover[1]], g[0]]]
            }
            (Tool::Arc3, 1) => vec![vec![g[0], hover]],
            (Tool::Arc3, 2) => arc_through(g[0], hover, g[1]).map_or_else(
                || vec![vec![g[0], hover, g[1]]],
                |(c, a, b)| {
                    vec![snap::sample_arc(
                        c,
                        dist(c, a),
                        (a[1] - c[1]).atan2(a[0] - c[0]),
                        snap::ccw_angle(c, a, b),
                        48,
                    )]
                },
            ),
            (Tool::ArcCentre, 1) => vec![vec![g[0], hover]],
            (Tool::ArcCentre, 2) => arc_from_centre(g[0], g[1], hover).map_or_else(
                || vec![vec![g[0], g[1]]],
                |(c, a, b)| {
                    vec![
                        vec![c, a],
                        snap::sample_arc(c, dist(c, a), (a[1] - c[1]).atan2(a[0] - c[0]), snap::ccw_angle(c, a, b), 48),
                    ]
                },
            ),
            _ => Vec::new(),
        }
    }

    /// The live length/angle (or radius) for the cursor at `hover`.
    #[must_use]
    pub fn readout(&self, hover: [f64; 2]) -> Option<Readout> {
        let a = self.anchor()?;
        let l = dist(a, hover);
        let angle = (hover[1] - a[1]).atan2(hover[0] - a[0]).to_degrees();
        Some(Readout { length: l, angle, radius: self.tool == Tool::Circle })
    }
}

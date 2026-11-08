//! Lines and arcs fitted to a section polyline ("fit primitives in 2D", design doc §6.4), with
//! automatic horizontal/vertical constraints, giving an editable sketch.
use crate::{Constraint, Entity, Sketch, SketchPlane};
use std::f64::consts::TAU;

fn line_dev(pts: &[[f64; 2]]) -> f64 {
    let (Some(a), Some(b)) = (pts.first(), pts.last()) else { return 0.0 };
    let d = (b[0] - a[0], b[1] - a[1]);
    let n = d.0.hypot(d.1);
    if n < 1e-12 {
        // a closed chain: no chord, measure from the start point
        return pts.iter().map(|p| (p[0] - a[0]).hypot(p[1] - a[1])).fold(0.0, f64::max);
    }
    pts.iter().map(|p| ((p[0] - a[0]) * d.1 - (p[1] - a[1]) * d.0).abs() / n).fold(0.0, f64::max)
}

/// Algebraic circle through points: centre and radius.
fn circle(pts: &[[f64; 2]]) -> Option<([f64; 2], f64)> {
    let mut a = nalgebra::Matrix3::<f64>::zeros();
    let mut b = nalgebra::Vector3::<f64>::zeros();
    for p in pts {
        let row = nalgebra::Vector3::new(p[0], p[1], 1.0);
        a += row * row.transpose();
        b += row * -(p[0] * p[0] + p[1] * p[1]);
    }
    let s = a.lu().solve(&b)?;
    let c = [-s[0] / 2.0, -s[1] / 2.0];
    let r2 = c[0] * c[0] + c[1] * c[1] - s[2];
    (r2 > 0.0 && r2.is_finite()).then(|| (c, r2.sqrt()))
}

fn circle_dev(pts: &[[f64; 2]], c: [f64; 2], r: f64) -> f64 {
    pts.iter().map(|p| ((p[0] - c[0]).hypot(p[1] - c[1]) - r).abs()).fold(0.0, f64::max)
}

/// A fitted piece: indices `[i, j]` into the polyline, and its shape.
#[derive(Clone, Debug, PartialEq)]
enum Piece {
    Line(usize, usize),
    Arc(usize, usize, [f64; 2], f64),
}

fn split(pts: &[[f64; 2]], i: usize, j: usize, tol: f64, out: &mut Vec<Piece>, depth: usize) {
    let Some(seg) = pts.get(i..=j) else { return };
    if j <= i + 1 || line_dev(seg) <= tol || depth > 40 {
        out.push(Piece::Line(i, j));
        return;
    }
    if seg.len() >= 5
        && let Some((c, r)) = circle(seg)
        && circle_dev(seg, c, r) <= tol
        && r < 1e4
    {
        out.push(Piece::Arc(i, j, c, r));
        return;
    }
    // split at the point farthest from the chord
    let (Some(a), Some(b)) = (seg.first(), seg.last()) else { return };
    let d = (b[0] - a[0], b[1] - a[1]);
    let n = d.0.hypot(d.1);
    let far = seg
        .iter()
        .enumerate()
        .map(|(k, p)| {
            let dist = if n < 1e-12 {
                (p[0] - a[0]).hypot(p[1] - a[1])
            } else {
                ((p[0] - a[0]) * d.1 - (p[1] - a[1]) * d.0).abs() / n
            };
            (k, dist)
        })
        .max_by(|x, y| x.1.total_cmp(&y.1))
        .map_or((i + j) / 2, |(k, _)| i + k);
    let far = far.clamp(i + 1, j - 1);
    split(pts, i, far, tol, out, depth + 1);
    split(pts, far, j, tol, out, depth + 1);
}

/// Index ranges of maximal runs of straight (turning < 1°) and curved samples; neighbouring runs
/// share their boundary sample.
fn runs(poly: &[[f64; 2]], closed: bool) -> Vec<(usize, usize)> {
    let n = poly.len();
    if n < 3 {
        return vec![(0, n.saturating_sub(1))];
    }
    let straight: Vec<bool> = (0..n)
        .map(|k| {
            let (p, q) = match (k, closed) {
                (0, true) => (poly[n - 2], poly[1]),
                (k, true) if k == n - 1 => (poly[n - 2], poly[1]),
                (0, false) => return true,
                (k, false) if k == n - 1 => return true,
                (k, _) => (poly[k - 1], poly[k + 1]),
            };
            let c = poly[k];
            let (u, v) = ((c[0] - p[0], c[1] - p[1]), (q[0] - c[0], q[1] - c[1]));
            (u.0 * v.1 - u.1 * v.0).atan2(u.0 * v.0 + u.1 * v.1).abs() < 1f64.to_radians()
        })
        .collect();
    // a segment between samples k and k+1 is curved only when both its ends turn
    let seg_straight: Vec<bool> = (0..n - 1).map(|k| straight[k] || straight[k + 1]).collect();
    let mut out = Vec::new();
    let mut start = 0;
    for k in 1..seg_straight.len() {
        if seg_straight[k] != seg_straight[k - 1] {
            out.push((start, k));
            start = k;
        }
    }
    out.push((start, n - 1));
    out
}

/// Fits lines and arcs within `tol` to a polyline (sketch coordinates; `closed` joins the ends)
/// and builds a sketch on `plane`: consecutive pieces share their end point; lines within 0.5°
/// of an axis get a horizontal/vertical constraint.
#[must_use]
pub fn fit_polyline(plane: SketchPlane, pts: &[[f64; 2]], closed: bool, tol: f64) -> Sketch {
    let mut s = Sketch { plane, ..Sketch::default() };
    let mut poly: Vec<[f64; 2]> = pts.to_vec();
    if closed
        && poly.len() > 2
        && let (Some(a), Some(b)) = (poly.first().copied(), poly.last().copied())
        && (a[0] - b[0]).hypot(a[1] - b[1]) > 1e-12
    {
        poly.push(a);
    }
    if poly.len() < 2 {
        return s;
    }
    // start a closed loop at a sharp corner, so no piece straddles it
    if closed && poly.len() > 4 {
        let m = poly.len() - 1;
        let turn = |k: usize| {
            let (p, c, q) = (poly[(k + m - 1) % m], poly[k % m], poly[(k + 1) % m]);
            let (u, v) = ((c[0] - p[0], c[1] - p[1]), (q[0] - c[0], q[1] - c[1]));
            (u.0 * v.1 - u.1 * v.0).atan2(u.0 * v.0 + u.1 * v.1).abs()
        };
        // a sharp corner if there is one, else a change between straight and curved
        let sharp = (0..m).max_by(|&a, &b| turn(a).total_cmp(&turn(b))).unwrap_or(0);
        let bent = |k: usize| turn(k) >= 1f64.to_radians();
        let curved_seg = |k: usize| bent(k) && bent((k + 1) % m);
        let corner = if turn(sharp) > 30f64.to_radians() {
            sharp
        } else {
            (0..m).find(|&k| curved_seg((k + m - 1) % m) != curved_seg(k)).unwrap_or(0)
        };
        let mut rot: Vec<[f64; 2]> = (0..m).map(|k| poly[(k + corner) % m]).collect();
        if let Some(f) = rot.first().copied() {
            rot.push(f);
        }
        poly = rot;
    }
    let tol = if tol.is_finite() && tol > 0.0 { tol } else { 0.05 };
    // runs of straight and of curved samples (by turning angle), each then split to the tolerance
    let mut pieces = Vec::new();
    for (i, j) in runs(&poly, closed) {
        split(&poly, i, j, tol, &mut pieces, 0);
    }
    // merge consecutive collinear lines
    let mut merged: Vec<Piece> = Vec::new();
    for p in pieces {
        if let (Some(Piece::Line(a, _)), Piece::Line(_, j)) = (merged.last().cloned(), &p)
            && poly.get(a..=*j).is_some_and(|seg| line_dev(seg) <= tol)
        {
            if let Some(last) = merged.last_mut() {
                *last = Piece::Line(a, *j);
            }
            continue;
        }
        merged.push(p);
    }
    // points shared between consecutive pieces
    let mut index_of = std::collections::HashMap::new();
    let mut pid = |s: &mut Sketch, k: usize| -> usize {
        let key = if closed && k == poly.len() - 1 { 0 } else { k };
        *index_of.entry(key).or_insert_with(|| s.add_point(poly.get(key).copied().unwrap_or([0.0, 0.0])))
    };
    for p in &merged {
        match p {
            Piece::Line(i, j) => {
                let (a, b) = (pid(&mut s, *i), pid(&mut s, *j));
                if a != b {
                    s.entities.push(Entity::Line { a, b });
                    let e = s.entities.len() - 1;
                    let (pa, pb) = (s.points[a], s.points[b]);
                    let ang = (pb[1] - pa[1]).atan2(pb[0] - pa[0]).rem_euclid(TAU / 2.0).to_degrees();
                    if ang < 0.5 || ang > 179.5 {
                        s.constraints.push(Constraint::Horizontal { line: e });
                    } else if (ang - 90.0).abs() < 0.5 {
                        s.constraints.push(Constraint::Vertical { line: e });
                    }
                }
            }
            Piece::Arc(i, j, c, _) => {
                let (a, b) = (pid(&mut s, *i), pid(&mut s, *j));
                let seg = poly.get(*i..=*j).unwrap_or(&[]);
                // orientation: counter-clockwise when the middle sample turns left
                let mid = seg.get(seg.len() / 2).copied().unwrap_or(*c);
                let (pa, pb) = (s.points[a], s.points[b]);
                let ccw = (mid[0] - pa[0]) * (pb[1] - pa[1]) - (mid[1] - pa[1]) * (pb[0] - pa[0]) > 0.0;
                let center = s.add_point(*c);
                let (sa, sb) = if ccw { (a, b) } else { (b, a) };
                if a == b {
                    let rad = (pa[0] - c[0]).hypot(pa[1] - c[1]);
                    s.entities.push(Entity::Circle { center, radius: rad });
                } else {
                    s.entities.push(Entity::Arc { center, a: sa, b: sb });
                }
            }
        }
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profiles;

    #[test]
    fn rounded_rectangle_section() {
        // a 40 × 20 rectangle with r = 3 rounded corners, densely sampled
        let mut pts: Vec<[f64; 2]> = Vec::new();
        let (w, h, r) = (40.0, 20.0, 3.0);
        let corners = [(w - r, r, -90.0f64), (w - r, h - r, 0.0), (r, h - r, 90.0), (r, r, 180.0)];
        for (k, (cx, cy, a0)) in corners.iter().enumerate() {
            for t in 0..=10 {
                let a = (a0 + 9.0 * f64::from(t)).to_radians();
                pts.push([cx + r * a.cos(), cy + r * a.sin()]);
            }
            let next = corners[(k + 1) % 4];
            let (ex, ey) = (cx + r * (a0 + 90.0).to_radians().cos(), cy + r * (a0 + 90.0).to_radians().sin());
            let (nx, ny) = (next.0 + r * next.2.to_radians().cos(), next.1 + r * next.2.to_radians().sin());
            for t in 1..20 {
                let f = f64::from(t) / 20.0;
                pts.push([ex + (nx - ex) * f, ey + (ny - ey) * f]);
            }
        }
        let s = fit_polyline(SketchPlane::default(), &pts, true, 0.01);
        let lines = s.entities.iter().filter(|e| matches!(e, Entity::Line { .. })).count();
        let arcs = s.entities.iter().filter(|e| matches!(e, Entity::Arc { .. })).count();
        assert_eq!((lines, arcs), (4, 4), "{:?}", s.entities);
        assert_eq!(s.constraints.len(), 4, "two horizontal and two vertical lines");
        assert_eq!(profiles(&s, 1e-6).len(), 1, "one closed profile");
    }
}

//! Which knot cells of the domain the outline polygon touches: plane geometry in (u, v) only.

use super::Grid;

/// Does the segment `p0 p1` meet the closed rectangle `[lo, hi]` (Liang–Barsky)?
fn segment_hits_rect(p0: [f64; 2], p1: [f64; 2], lo: [f64; 2], hi: [f64; 2]) -> bool {
    let (mut t0, mut t1) = (0.0_f64, 1.0_f64);
    for axis in 0..2 {
        let d = p1[axis] - p0[axis];
        for (pp, qq) in [(-d, p0[axis] - lo[axis]), (d, hi[axis] - p0[axis])] {
            if pp == 0.0 {
                if qq < 0.0 {
                    return false;
                }
            } else {
                let r = qq / pp;
                if pp < 0.0 {
                    if r > t1 {
                        return false;
                    }
                    t0 = t0.max(r);
                } else {
                    if r < t0 {
                        return false;
                    }
                    t1 = t1.min(r);
                }
            }
        }
    }
    true
}

fn inside_polygon(pt: [f64; 2], poly: &[[f64; 2]]) -> bool {
    let mut inside = false;
    let Some(mut prev) = poly.last().copied() else { return false };
    for &cur in poly {
        if (cur[1] > pt[1]) != (prev[1] > pt[1]) {
            let x = (prev[0] - cur[0]) * (pt[1] - cur[1]) / (prev[1] - cur[1]) + cur[0];
            if pt[0] < x {
                inside = !inside;
            }
        }
        prev = cur;
    }
    inside
}

/// Knot cells (`a·cells_v + b`) that touch or lie inside the outline, plus one ring of cells.
pub(super) fn active_cells(g: &Grid, outline: &[[f64; 2]]) -> Vec<bool> {
    let [cu, cv] = g.cells;
    if outline.is_empty() {
        return vec![true; cu * cv];
    }
    let mut hit = vec![false; cu * cv];
    for a in 0..cu {
        for b in 0..cv {
            let (Some(&ulo), Some(&uhi), Some(&vlo), Some(&vhi)) =
                (g.bounds[0].get(a), g.bounds[0].get(a + 1), g.bounds[1].get(b), g.bounds[1].get(b + 1))
            else {
                continue;
            };
            let (lo, hi) = ([ulo, vlo], [uhi, vhi]);
            let mut touches = false;
            let mut prev = outline.last().copied().unwrap_or([0.0; 2]);
            for &cur in outline {
                if segment_hits_rect(prev, cur, lo, hi) {
                    touches = true;
                    break;
                }
                prev = cur;
            }
            if !touches {
                touches = inside_polygon([0.5 * (ulo + uhi), 0.5 * (vlo + vhi)], outline);
            }
            if let Some(slot) = hit.get_mut(a * cv + b) {
                *slot = touches;
            }
        }
    }
    let mut active = hit.clone();
    for a in 0..cu {
        for b in 0..cv {
            if !hit.get(a * cv + b).copied().unwrap_or(false) {
                continue;
            }
            for da in a.saturating_sub(1)..=(a + 1).min(cu - 1) {
                for db in b.saturating_sub(1)..=(b + 1).min(cv - 1) {
                    if let Some(slot) = active.get_mut(da * cv + db) {
                        *slot = true;
                    }
                }
            }
        }
    }
    active
}

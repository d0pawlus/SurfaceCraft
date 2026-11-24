//! Step 2 of the planner, holes: a concave full cylinder is through (global) or blind (a tool with a bottom).

use super::*;

/// A blind hole candidate: a point `o` on its axis, the unit direction `u` out of the material
/// towards the opening, the radius and the closed end of the wall along `u`.
struct HoleGeom {
    o: Vec3,
    u: Vec3,
    r: f64,
    wall_end: f64,
}

/// The bottom found for a blind hole: its region, its kind, its position along the axis and a
/// description for the report.
struct Bottom {
    node: usize,
    kind: HoleBottom,
    s: f64,
    how: String,
}

/// A plane across the hole's axis that lies at the wall's closed end: the position along the axis
/// and the description of a flat bottom.
fn flat_bottom(bn: &Node, bp: [f64; 3], normal: [f64; 3], g: &HoleGeom) -> Option<(f64, String)> {
    let n = v3(normal);
    if n.dot(&g.u) < 0.99 {
        return None;
    }
    let near = bn.pts.iter().filter(|p| radial_axial(p, g.o, g.u).0 <= g.r + 1.0).count();
    if near < 8 {
        return None;
    }
    let s_f = (v3(bp) - g.o).dot(&n) / n.dot(&g.u);
    ((s_f - g.wall_end).abs() <= 2.0)
        .then(|| (s_f, format!("flat bottom {:.2} mm from the wall's end", s_f - g.wall_end)))
}

/// A concave full cone coaxial with the hole whose rim lies at the wall's closed end (a drill
/// point): its bottom kind, the position of its apex along the axis and the description.
fn cone_bottom(bn: &Node, axis: [f64; 3], g: &HoleGeom) -> Option<(HoleBottom, f64, String)> {
    if v3(axis).dot(&g.u).abs() < 0.99 {
        return None;
    }
    // regress the cone's radius on the position along the hole's axis: radius = slope * (s - apex)
    let pairs: Vec<(f64, f64)> =
        bn.pts.iter().map(|p| ((p.coords - g.o).dot(&g.u), radial_axial(p, g.o, g.u).0)).collect();
    let n = pairs.len() as f64;
    if n < 10.0 {
        return None;
    }
    let (ms, mr) = (pairs.iter().map(|p| p.0).sum::<f64>() / n, pairs.iter().map(|p| p.1).sum::<f64>() / n);
    let sxx: f64 = pairs.iter().map(|p| (p.0 - ms).powi(2)).sum();
    let sxy: f64 = pairs.iter().map(|p| (p.0 - ms) * (p.1 - mr)).sum();
    if sxx < 1e-9 || sxy <= 0.0 {
        return None;
    }
    let slope = sxy / sxx;
    let half = slope.atan();
    let apex_s = ms - mr / slope;
    let rim_s = apex_s + g.r / slope;
    if !(0.3..=1.4).contains(&half) || (rim_s - g.wall_end).abs() > 1.5 {
        return None;
    }
    Some((
        HoleBottom::Cone { half_angle: half },
        apex_s,
        format!(
            "drill point: {:.1} deg included angle, apex {:.2} mm from the wall's end",
            2.0 * half.to_degrees(),
            apex_s - g.wall_end
        ),
    ))
}

impl Planner<'_> {
    /// Step 2, holes: every pending concave full cylinder is a through hole, a blind hole or skipped.
    pub(super) fn plan_holes(&mut self) {
        for hi in 0..self.nodes.len() {
            self.plan_hole(hi);
        }
    }

    /// Decides what the pending concave full cylinder `hi` is, from the scan that closes it.
    fn plan_hole(&mut self, hi: usize) {
        let nodes = self.nodes;
        let Some(nd) = nodes.get(hi) else { return };
        let Shape::Cylinder { point, axis, radius } = &nd.shape else { return };
        if !self.st.pending(hi) || !nd.full || nd.convex {
            return;
        }
        let (o, a0, r) = (v3(*point), v3(*axis), *radius);
        let (closed_low, closed_high) = self.hole_closure(nd, o, a0, r);
        match (closed_low, closed_high) {
            (false, false) => {
                self.st.set(
                    hi,
                    RoleKind::Global,
                    "through hole: nothing of the scan closes the cylinder at either end",
                );
                return;
            }
            (true, true) => {
                self.st.set(hi, RoleKind::Skipped, "a concave cylinder closed at both ends (a cavity) is not built");
                return;
            }
            _ => {}
        }
        // u points out of the material, towards the hole's opening
        let u = snap_direction(if closed_low { a0 } else { -a0 }, self.anchors, 0.5);
        // the wall's closed end: the smallest position along u
        let wall_end = nd.pts.iter().map(|p| (p.coords - o).dot(&u)).fold(f64::INFINITY, f64::min);
        let g = HoleGeom { o, u, r, wall_end };
        let Some(bottom) = self.hole_bottom(&g) else {
            self.st.set(
                hi,
                RoleKind::Skipped,
                "blind hole: its bottom (a plane across the axis or a coaxial cone) was not given",
            );
            return;
        };
        self.commit_blind_hole(hi, nd, &g, bottom);
    }

    /// Whether the scan closes the hole's cylinder `nd` (axis `(o, a0)`, radius `r`) at its low
    /// and at its high end along the axis.
    fn hole_closure(&self, nd: &Node, o: Vec3, a0: Vec3, r: f64) -> (bool, bool) {
        let (t0, t1) = axial_extent(&nd.pts, o, a0);
        let window = 0.5 * r + 1.5;
        let (mut low, mut high) = (0usize, 0usize);
        for p in self.scan {
            let (rho, t) = radial_axial(p, o, a0);
            if rho < 0.6 * r {
                low += usize::from(t > t0 - window && t < t0 + 0.3);
                high += usize::from(t > t1 - 0.3 && t < t1 + window);
            }
        }
        (low >= 3, high >= 3)
    }

    /// The bottom: a pending local plane across the axis, or a coaxial cone (a drill point); the
    /// first region that fits.
    fn hole_bottom(&self, g: &HoleGeom) -> Option<Bottom> {
        for (bi, bn) in self.nodes.iter().enumerate() {
            if !self.st.pending(bi) {
                continue;
            }
            let found = match &bn.shape {
                Shape::Plane { point: bp, normal } => {
                    flat_bottom(bn, *bp, *normal, g).map(|(s, how)| (HoleBottom::Flat, s, how))
                }
                Shape::Cone { axis, .. } if bn.full && !bn.convex => cone_bottom(bn, *axis, g),
                _ => None,
            };
            if let Some((kind, s, how)) = found {
                return Some(Bottom { node: bi, kind, s, how });
            }
        }
        None
    }

    /// Forms the blind hole's tool and gives its wall `hi` and its bottom their roles and planned shapes.
    fn commit_blind_hole(&mut self, hi: usize, nd: &Node, g: &HoleGeom, bottom: Bottom) {
        let Bottom { node: bi, kind, s, how } = bottom;
        let (u, r) = (g.u, g.r);
        let base = g.o + u * s;
        let ti = self.tools.len();
        self.tools.push(PlannedTool {
            tool: LocalTool::BlindHole { base: arr(base), axis: arr(u), radius: r, bottom: kind },
            kind: "blind_hole",
            members: vec![nd.base, self.nodes.get(bi).map_or(nd.base, |b| b.base)],
            detail: format!("Ø{:.3}, {how}", 2.0 * r),
        });
        let st = &mut self.st;
        st.set(
            hi,
            RoleKind::LocalSubtract,
            format!("wall of a blind hole Ø{:.3} (the scan closes it at one end)", 2.0 * r),
        );
        if let Some(t) = st.tools.get_mut(hi) {
            t.push(ti);
        }
        st.set(bi, RoleKind::LocalSubtract, format!("bottom of the blind hole Ø{:.3}: {how}", 2.0 * r));
        if let Some(t) = st.tools.get_mut(bi) {
            t.push(ti);
        }
        // the planned geometry: snapped axis through the tool's base
        if let Some(sh) = st.shape.get_mut(hi) {
            *sh = Shape::Cylinder { point: arr(base), axis: arr(u), radius: r };
        }
        if let (Some(sh), HoleBottom::Cone { half_angle }) = (st.shape.get_mut(bi), kind) {
            *sh = Shape::Cone { apex: arr(base), axis: arr(u), half_angle };
        }
        if let (Some(sh), HoleBottom::Flat) = (st.shape.get_mut(bi), kind) {
            *sh = Shape::Plane { point: arr(base), normal: arr(u) };
        }
    }
}

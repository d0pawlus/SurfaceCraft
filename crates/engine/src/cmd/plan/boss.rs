//! Bosses: fitted cylinders and cones that stand off the envelope, planned as a boss with its explained scan.

use super::nodes::*;
use super::planner::*;
use super::*;

// ---------------------------------------------------------------------------------------------
// bosses

/// Scan that is in front of a plane only because a boss stands on it.
pub(super) struct Explained {
    pub(super) o: Vec3,
    pub(super) u: Vec3,
    pub(super) reach: f64,
    pub(super) s_base: f64,
}

impl Explained {
    /// Does this boss explain the point `p` in front of a plane with normal `n`?
    pub(super) fn covers(&self, p: &Point3, n: &Vec3) -> bool {
        n.dot(&self.u).abs() > 0.99
            && radial_axial(p, self.o, self.u).0 <= self.reach
            && (p.coords - self.o).dot(&self.u) >= self.s_base - 0.3
    }
}

/// The axial position of the plane `pn` along the axis `(o, a)`, and its unit normal.
fn plane_t(pn: &Node, o: Vec3, a: Vec3) -> Option<(f64, Vec3)> {
    let Shape::Plane { point, normal } = &pn.shape else { return None };
    let n = normalize(v3(*normal))?;
    let c = n.dot(&a);
    (c.abs() > 0.99).then(|| ((v3(*point) - o).dot(&n) / c, n))
}

/// A boss: the convex full cylinder `ci`, the plane across its far end (its top) and the plane it
/// stands on, with a chamfer or round on the top edge and a fillet at the base when the scan has
/// them (a coaxial cone, a coaxial torus). On success the regions that form the boss are given
/// their roles in `st`; the base plane stays for the global test (the boss explains the scan in
/// front of it).
pub(super) fn plan_boss(
    nodes: &[Node],
    ci: usize,
    st: &mut State,
    anchors: &[Vec3],
    add_index: usize,
    outside: f64,
) -> Result<(PlannedAdd, Explained), String> {
    let nd = nodes.get(ci).ok_or("index")?;
    let Shape::Cylinder { point, axis, radius } = &nd.shape else { return Err("not a cylinder".into()) };
    let (o, a0, r) = (v3(*point), normalize(v3(*axis)).ok_or("degenerate axis")?, *radius);
    let (mut t0, mut t1) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in &nd.pts {
        let t = (p.coords - o).dot(&a0);
        t0 = t0.min(t);
        t1 = t1.max(t);
    }
    // the two ends: the top is a disc, the base a plane with the boss's foot in it
    let mut found: Option<(Vec3, f64, f64, usize)> = None;
    for sign in [1.0, -1.0] {
        let u = a0 * sign;
        let (s0, s1) = if sign > 0.0 { (t0, t1) } else { (-t1, -t0) };
        let mut top: Option<(usize, f64)> = None;
        let mut base: Option<f64> = None;
        for (pi, pn) in nodes.iter().enumerate() {
            if pi == ci || !st.pending(pi) {
                continue;
            }
            let Some((t, n)) = plane_t(pn, o, a0) else { continue };
            let s = sign * t;
            if n.dot(&u) < 0.9 {
                continue;
            }
            let inside = pn.pts.iter().filter(|p| radial_axial(p, o, a0).0 <= r + 0.6).count() as f64
                / pn.pts.len().max(1) as f64;
            let ring = pn
                .pts
                .iter()
                .filter(|p| {
                    let d = radial_axial(p, o, a0).0;
                    d > r + 0.2 && d < r + 6.0
                })
                .count();
            if inside >= 0.9
                && (s1 - 0.4..=s1 + 4.0).contains(&s)
                && top.is_none_or(|b| (s - s1).abs() < (b.1 - s1).abs())
            {
                top = Some((pi, s));
            }
            if ring >= 8 && (s0 - 4.0..=s0 + 0.4).contains(&s) && base.is_none_or(|b| (s - s0).abs() < (b - s0).abs()) {
                base = Some(s);
            }
        }
        if let (Some((ti, s_top)), Some(s_base)) = (top, base)
            && s_top > s_base + 0.5
        {
            found = Some((u, s_base, s_top, ti));
            break;
        }
    }
    let Some((u0, s_base, s_top, ti)) = found else {
        return Err("no plane across its end for a top, or no plane it stands on".into());
    };
    let u = snap_direction(u0, anchors, 0.5);
    let h = s_top - s_base;
    // the top edge: a chamfer (a coaxial cone) or a round (a coaxial convex torus); the foot: a
    // coaxial concave torus
    let mut members: Vec<usize> = vec![nd.base, nodes.get(ti).map_or(nd.base, |n| n.base)];
    let mut parts: Vec<String> = Vec::new();
    let mut m_base = 0.0;
    for (ni, nn) in nodes.iter().enumerate() {
        if ni == ci || ni == ti || !st.pending(ni) {
            continue;
        }
        match &nn.shape {
            Shape::Cone { .. } if nn.convex => {
                let near_top = nn
                    .pts
                    .iter()
                    .filter(|p| {
                        let (rho, _) = radial_axial(p, o, u);
                        let s = (p.coords - o).dot(&u);
                        rho <= r + 0.5 && s >= s_top - 8.0 && s <= s_top + 0.5
                    })
                    .count();
                if near_top * 10 < nn.pts.len() * 9 {
                    continue;
                }
                // radius against height along the boss axis: rho = mr + k (s - ms)
                let pairs: Vec<(f64, f64)> = nn
                    .pts
                    .iter()
                    .map(|p| {
                        let (rho, _) = radial_axial(p, o, u);
                        ((p.coords - o).dot(&u), rho)
                    })
                    .collect();
                let n = pairs.len() as f64;
                if n < 10.0 {
                    continue;
                }
                let (ms, mr) = (pairs.iter().map(|p| p.0).sum::<f64>() / n, pairs.iter().map(|p| p.1).sum::<f64>() / n);
                let sxx: f64 = pairs.iter().map(|p| (p.0 - ms).powi(2)).sum();
                let sxy: f64 = pairs.iter().map(|p| (p.0 - ms) * (p.1 - mr)).sum();
                if sxx < 1e-9 {
                    continue;
                }
                let k = sxy / sxx;
                if !(-20.0..=-0.1).contains(&k) {
                    continue;
                }
                let rho_top = mr + k * (s_top - ms);
                let s_r = ms + (r - mr) / k;
                let (c_rad, c_ax) = (r - rho_top, s_top - s_r);
                if !(0.1..r).contains(&c_rad) || !(0.1..h).contains(&c_ax) {
                    continue;
                }
                let half = (-k).atan();
                let s_apex = ms - mr / k;
                st.set(
                    ni,
                    RoleKind::Blend,
                    format!("chamfer of the top edge of the boss: {c_rad:.2} mm on the top, {c_ax:.2} mm on the wall"),
                );
                if let Some(sh) = st.shape.get_mut(ni) {
                    *sh = Shape::Cone { apex: arr(o + u * s_apex), axis: arr(-u), half_angle: half };
                }
                members.push(nn.base);
                parts.push(format!("chamfer {c_rad:.2} x {c_ax:.2}"));
            }
            Shape::Torus { center, axis, major_radius, minor_radius } => {
                let m = *minor_radius;
                let Some(ta) = normalize(v3(*axis)) else { continue };
                let tc = v3(*center);
                let s_c = (tc - o).dot(&u);
                let tol = 0.6 + 0.2 * m;
                if ta.dot(&u).abs() < 0.99 || radial_axial(&Point3::from(tc), o, u).0 > 1.0 {
                    continue;
                }
                let (name, center_s, major) = if nn.convex {
                    ("round of the top edge of the boss", s_top - m, r - m)
                } else {
                    ("fillet at the foot of the boss", s_base + m, r + m)
                };
                if (major_radius - major).abs() > tol || (s_c - center_s).abs() > tol {
                    continue;
                }
                st.set(ni, RoleKind::Blend, format!("{name}: torus, minor radius {m:.2}"));
                if let Some(sh) = st.shape.get_mut(ni) {
                    *sh = Shape::Torus {
                        center: arr(o + u * center_s),
                        axis: arr(u),
                        major_radius: major,
                        minor_radius: m,
                    };
                }
                if !nn.convex {
                    m_base = m;
                }
                members.push(nn.base);
                parts.push(format!("{} r = {m:.2}", if nn.convex { "round" } else { "foot fillet" }));
            }
            _ => {}
        }
    }
    let base_pt = o + u * s_base;
    st.set(
        ci,
        RoleKind::LocalAdd,
        format!(
            "boss Ø{:.3}, {h:.2} mm tall ({:.1} % of the scan lies outside this cylinder)",
            2.0 * r,
            outside * 100.0
        ),
    );
    st.set(ti, RoleKind::LocalAdd, format!("top of the boss Ø{:.3}", 2.0 * r));
    for k in [ci, ti] {
        if let Some(v) = st.adds.get_mut(k) {
            v.push(add_index);
        }
    }
    if let Some(sh) = st.shape.get_mut(ci) {
        *sh = Shape::Cylinder { point: arr(base_pt), axis: arr(u), radius: r };
    }
    if let Some(sh) = st.shape.get_mut(ti) {
        *sh = Shape::Plane { point: arr(o + u * s_top), normal: arr(u) };
    }
    Ok((
        PlannedAdd {
            add: LocalAdd::Boss { base: arr(base_pt), axis: arr(u), radius: r, height: h, embed: 0.5 },
            kind: "boss",
            members,
            detail: format!(
                "Ø{:.3}, {h:.2} mm tall{}",
                2.0 * r,
                if parts.is_empty() { String::new() } else { format!(", {}", parts.join(", ")) }
            ),
        },
        Explained { o, u, reach: r + m_base + 0.6, s_base },
    ))
}

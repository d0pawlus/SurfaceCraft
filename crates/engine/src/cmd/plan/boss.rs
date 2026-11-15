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

/// The cylinder of a boss: its axis `(o, a0)`, its radius and the extent of its points along the axis.
struct BossAxis {
    o: Vec3,
    a0: Vec3,
    r: f64,
    t0: f64,
    t1: f64,
}

/// The two planes that close a boss along its axis: the direction `u` from the foot to the top,
/// the positions of the foot and the top along it, and the region of the top.
struct Ends {
    u: Vec3,
    s_base: f64,
    s_top: f64,
    top: usize,
}

/// A boss along its snapped axis `u`: the radius, the positions of the foot and the top and the height.
struct BossFrame {
    o: Vec3,
    u: Vec3,
    r: f64,
    s_base: f64,
    s_top: f64,
    h: f64,
}

impl BossFrame {
    /// The centre of the boss's foot.
    fn base_point(&self) -> Vec3 {
        self.o + self.u * self.s_base
    }
}

/// The regions that form a boss, the report parts of its edge treatments and the minor radius of its foot fillet.
struct Edges {
    members: Vec<usize>,
    parts: Vec<String>,
    m_base: f64,
}

/// How much of the plane `pn` lies within the boss's radius (plus a margin), and how many of its
/// points form a ring just outside the cylinder.
fn end_plane_scores(pn: &Node, ax: &BossAxis) -> (f64, usize) {
    let inside = pn.pts.iter().filter(|p| radial_axial(p, ax.o, ax.a0).0 <= ax.r + 0.6).count() as f64
        / pn.pts.len().max(1) as f64;
    let ring = pn
        .pts
        .iter()
        .filter(|p| {
            let d = radial_axial(p, ax.o, ax.a0).0;
            d > ax.r + 0.2 && d < ax.r + 6.0
        })
        .count();
    (inside, ring)
}

/// The best candidates for the top and the foot of a boss that stands towards `u = a0 * sign`.
fn scan_ends(
    nodes: &[Node],
    ci: usize,
    st: &State,
    ax: &BossAxis,
    sign: f64,
    u: Vec3,
) -> (Option<(usize, f64)>, Option<f64>) {
    let (s0, s1) = if sign > 0.0 { (ax.t0, ax.t1) } else { (-ax.t1, -ax.t0) };
    let mut top: Option<(usize, f64)> = None;
    let mut base: Option<f64> = None;
    for (pi, pn) in nodes.iter().enumerate() {
        if pi == ci || !st.pending(pi) {
            continue;
        }
        let Some((t, n)) = plane_t(pn, ax.o, ax.a0) else { continue };
        let s = sign * t;
        if n.dot(&u) < 0.9 {
            continue;
        }
        let (inside, ring) = end_plane_scores(pn, ax);
        if inside >= 0.9 && (s1 - 0.4..=s1 + 4.0).contains(&s) && top.is_none_or(|b| (s - s1).abs() < (b.1 - s1).abs())
        {
            top = Some((pi, s));
        }
        if ring >= 8 && (s0 - 4.0..=s0 + 0.4).contains(&s) && base.is_none_or(|b| (s - s0).abs() < (b - s0).abs()) {
            base = Some(s);
        }
    }
    (top, base)
}

/// The two ends of a boss: the top is a disc, the base a plane with the boss's foot in it. Both
/// directions along the axis are tried, `+` first.
fn find_ends(nodes: &[Node], ci: usize, st: &State, ax: &BossAxis) -> Option<Ends> {
    for sign in [1.0, -1.0] {
        let u = ax.a0 * sign;
        if let (Some((ti, s_top)), Some(s_base)) = scan_ends(nodes, ci, st, ax, sign, u)
            && s_top > s_base + 0.5
        {
            return Some(Ends { u, s_base, s_top, top: ti });
        }
    }
    None
}

/// The line `rho = mr + k (s - ms)` through the (position along `u`, radius about the axis) pairs
/// of `pts`, as `(ms, mr, k)`; `None` for too few points or no spread along the axis.
fn radius_height_line(pts: &[Point3], o: Vec3, u: Vec3) -> Option<(f64, f64, f64)> {
    let pairs: Vec<(f64, f64)> = pts
        .iter()
        .map(|p| {
            let (rho, _) = radial_axial(p, o, u);
            ((p.coords - o).dot(&u), rho)
        })
        .collect();
    let n = pairs.len() as f64;
    if n < 10.0 {
        return None;
    }
    let (ms, mr) = (pairs.iter().map(|p| p.0).sum::<f64>() / n, pairs.iter().map(|p| p.1).sum::<f64>() / n);
    let sxx: f64 = pairs.iter().map(|p| (p.0 - ms).powi(2)).sum();
    let sxy: f64 = pairs.iter().map(|p| (p.0 - ms) * (p.1 - mr)).sum();
    if sxx < 1e-9 {
        return None;
    }
    Some((ms, mr, sxy / sxx))
}

/// A convex cone that is the chamfer of the top edge of the boss (coaxial, narrowing towards the
/// top): the region gets the role `Blend` and its fitted cone.
fn chamfer_top_edge(ni: usize, nn: &Node, f: &BossFrame, st: &mut State, edges: &mut Edges) {
    let near_top = nn
        .pts
        .iter()
        .filter(|p| {
            let (rho, _) = radial_axial(p, f.o, f.u);
            let s = (p.coords - f.o).dot(&f.u);
            rho <= f.r + 0.5 && s >= f.s_top - 8.0 && s <= f.s_top + 0.5
        })
        .count();
    if near_top * 10 < nn.pts.len() * 9 {
        return;
    }
    // radius against height along the boss axis: rho = mr + k (s - ms)
    let Some((ms, mr, k)) = radius_height_line(&nn.pts, f.o, f.u) else { return };
    if !(-20.0..=-0.1).contains(&k) {
        return;
    }
    let rho_top = mr + k * (f.s_top - ms);
    let s_r = ms + (f.r - mr) / k;
    let (c_rad, c_ax) = (f.r - rho_top, f.s_top - s_r);
    if !(0.1..f.r).contains(&c_rad) || !(0.1..f.h).contains(&c_ax) {
        return;
    }
    let half = (-k).atan();
    let s_apex = ms - mr / k;
    st.set(
        ni,
        RoleKind::Blend,
        format!("chamfer of the top edge of the boss: {c_rad:.2} mm on the top, {c_ax:.2} mm on the wall"),
    );
    if let Some(sh) = st.shape.get_mut(ni) {
        *sh = Shape::Cone { apex: arr(f.o + f.u * s_apex), axis: arr(-f.u), half_angle: half };
    }
    edges.members.push(nn.base);
    edges.parts.push(format!("chamfer {c_rad:.2} x {c_ax:.2}"));
}

/// A coaxial torus that is the round of the top edge (convex) or the fillet at the foot (concave)
/// of the boss: the region gets the role `Blend` and its fitted torus.
fn torus_edge(ni: usize, nn: &Node, f: &BossFrame, st: &mut State, edges: &mut Edges) {
    let Shape::Torus { center, axis, major_radius, minor_radius } = &nn.shape else { return };
    let m = *minor_radius;
    let Some(ta) = normalize(v3(*axis)) else { return };
    let tc = v3(*center);
    let s_c = (tc - f.o).dot(&f.u);
    let tol = 0.6 + 0.2 * m;
    if ta.dot(&f.u).abs() < 0.99 || radial_axial(&Point3::from(tc), f.o, f.u).0 > 1.0 {
        return;
    }
    let (name, center_s, major) = if nn.convex {
        ("round of the top edge of the boss", f.s_top - m, f.r - m)
    } else {
        ("fillet at the foot of the boss", f.s_base + m, f.r + m)
    };
    if (major_radius - major).abs() > tol || (s_c - center_s).abs() > tol {
        return;
    }
    st.set(ni, RoleKind::Blend, format!("{name}: torus, minor radius {m:.2}"));
    if let Some(sh) = st.shape.get_mut(ni) {
        *sh = Shape::Torus { center: arr(f.o + f.u * center_s), axis: arr(f.u), major_radius: major, minor_radius: m };
    }
    if !nn.convex {
        edges.m_base = m;
    }
    edges.members.push(nn.base);
    edges.parts.push(format!("{} r = {m:.2}", if nn.convex { "round" } else { "foot fillet" }));
}

/// Gives the cylinder `ci` and the top region `ti` their roles (a local addition of index
/// `add_index`) and their planned shapes.
fn claim_boss(st: &mut State, f: &BossFrame, (ci, ti): (usize, usize), add_index: usize, outside: f64) {
    let (r, h) = (f.r, f.h);
    let base_pt = f.base_point();
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
        *sh = Shape::Cylinder { point: arr(base_pt), axis: arr(f.u), radius: r };
    }
    if let Some(sh) = st.shape.get_mut(ti) {
        *sh = Shape::Plane { point: arr(f.o + f.u * f.s_top), normal: arr(f.u) };
    }
}

/// The addition for the kernel and the scan the boss explains, from the frame and its edges.
fn boss_add(f: &BossFrame, edges: Edges) -> (PlannedAdd, Explained) {
    let (r, h) = (f.r, f.h);
    (
        PlannedAdd {
            add: LocalAdd::Boss { base: arr(f.base_point()), axis: arr(f.u), radius: r, height: h, embed: 0.5 },
            kind: "boss",
            members: edges.members,
            detail: format!(
                "Ø{:.3}, {h:.2} mm tall{}",
                2.0 * r,
                if edges.parts.is_empty() { String::new() } else { format!(", {}", edges.parts.join(", ")) }
            ),
        },
        Explained { o: f.o, u: f.u, reach: r + edges.m_base + 0.6, s_base: f.s_base },
    )
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
    let (t0, t1) = axial_extent(&nd.pts, o, a0);
    let Some(ends) = find_ends(nodes, ci, st, &BossAxis { o, a0, r, t0, t1 }) else {
        return Err("no plane across its end for a top, or no plane it stands on".into());
    };
    let u = snap_direction(ends.u, anchors, 0.5);
    let f = BossFrame { o, u, r, s_base: ends.s_base, s_top: ends.s_top, h: ends.s_top - ends.s_base };
    let ti = ends.top;
    // the top edge: a chamfer (a coaxial cone) or a round (a coaxial convex torus); the foot: a
    // coaxial concave torus
    let members: Vec<usize> = vec![nd.base, nodes.get(ti).map_or(nd.base, |n| n.base)];
    let mut edges = Edges { members, parts: Vec::new(), m_base: 0.0 };
    for (ni, nn) in nodes.iter().enumerate() {
        if ni == ci || ni == ti || !st.pending(ni) {
            continue;
        }
        match &nn.shape {
            Shape::Cone { .. } if nn.convex => chamfer_top_edge(ni, nn, &f, st, &mut edges),
            Shape::Torus { .. } => torus_edge(ni, nn, &f, st, &mut edges),
            _ => {}
        }
    }
    claim_boss(st, &f, (ci, ti), add_index, outside);
    Ok(boss_add(&f, edges))
}

//! The planner itself: the state of one planning pass and the plan of all regions.

use super::boss::*;
use super::nodes::*;
use super::*;

// ---------------------------------------------------------------------------------------------
// the planner

pub(super) struct State {
    pub(super) kind: Vec<Option<RoleKind>>,
    pub(super) how: Vec<String>,
    /// Why an attempt to use a still-unassigned region failed (the first one is kept).
    pub(super) notes: Vec<String>,
    pub(super) tools: Vec<Vec<usize>>,
    pub(super) adds: Vec<Vec<usize>>,
    pub(super) shape: Vec<Shape>,
}

impl State {
    pub(super) fn set(&mut self, n: usize, kind: RoleKind, how: impl Into<String>) {
        if let (Some(k), Some(h)) = (self.kind.get_mut(n), self.how.get_mut(n)) {
            *k = Some(kind);
            *h = how.into();
        }
    }
    pub(super) fn note(&mut self, n: usize, text: impl Into<String>) {
        if let Some(t) = self.notes.get_mut(n)
            && t.is_empty()
        {
            *t = text.into();
        }
    }
    pub(super) fn pending(&self, n: usize) -> bool {
        self.kind.get(n).is_some_and(Option::is_none)
    }
}

/// Plans the build: classifies every region and forms the local tools. `scan` is the whole
/// scanned cloud the regions come from (the "is there scan in front of it?" test).
#[must_use]
pub fn plan(items: &[Item], cloud: &Scan, square_deg: f64) -> Plan {
    let scan: &[Point3] = &cloud.points;
    let sc = cloud.scale();
    let nodes = make_nodes(items);
    let contacts = contacts(&nodes);
    let mut st = State {
        kind: vec![None; nodes.len()],
        how: vec![String::new(); nodes.len()],
        notes: vec![String::new(); nodes.len()],
        tools: vec![Vec::new(); nodes.len()],
        adds: vec![Vec::new(); nodes.len()],
        shape: nodes.iter().map(|n| n.shape.clone()).collect(),
    };
    let mut tools: Vec<PlannedTool> = Vec::new();
    let diag = {
        let mut lo = Vec3::repeat(f64::INFINITY);
        let mut hi = Vec3::repeat(f64::NEG_INFINITY);
        for p in scan {
            lo = lo.inf(&p.coords);
            hi = hi.sup(&p.coords);
        }
        if lo.iter().all(|v| v.is_finite()) { (hi - lo).norm() } else { 100.0 }
    };
    let total = scan.len().max(1) as f64;
    let mut front_of: Vec<Option<f64>> = vec![None; nodes.len()];

    // Auto-constraint of directions (design doc §6.2): the tools' directions (a pocket's, a hole's
    // axis, a slot's) snap to the model axes when within half a degree. The kernel's booleans fail
    // on a tool whose direction is, to the last digit, the normal of one of the stock's fitted
    // planes (observed on a window in a plate); a model axis is not.
    let global_normals: Vec<Vec3> = vec![Vec3::x(), Vec3::y(), Vec3::z()];

    // --- 0. bosses: a convex full cylinder with scan outside it that stands on a plane. What is in
    // front of that plane because of the boss is not an obstacle to the plane being global.
    let mut adds: Vec<PlannedAdd> = Vec::new();
    let mut explained: Vec<Explained> = Vec::new();
    for ci in 0..nodes.len() {
        let Some(nd) = nodes.get(ci) else { continue };
        let Shape::Cylinder { point, axis, radius } = &nd.shape else { continue };
        if !(nd.full && nd.convex) || !st.pending(ci) {
            continue;
        }
        let (o, a) = (v3(*point), v3(*axis));
        let tol = sc.mm(0.1) + 0.05 * radius.min(sc.mm(2.0));
        let out = scan.iter().filter(|p| radial_axial(p, o, a).0 > radius + tol).count() as f64 / total;
        front_of[ci] = Some(out);
        if out <= ENVELOPE_FRACTION {
            continue;
        }
        let ai = adds.len();
        match plan_boss(&nodes, ci, &mut st, &global_normals, ai, out) {
            Ok((add, ex)) => {
                adds.push(add);
                explained.push(ex);
            }
            Err(why) => st.note(ci, why),
        }
    }

    // --- 1. global or local, from the scan
    for (ni, nd) in nodes.iter().enumerate() {
        if !st.pending(ni) {
            continue;
        }
        match &nd.shape {
            Shape::Plane { point, normal } => {
                let (p0, n) = (v3(*point), v3(*normal));
                let rms = (nd.pts.iter().map(|p| (p.coords - p0).dot(&n).powi(2)).sum::<f64>()
                    / nd.pts.len().max(1) as f64)
                    .sqrt();
                let tol = (sc.mm(0.1) + 3.0 * rms).min(sc.mm(0.4));
                let (mut in_front, mut by_boss) = (0usize, 0usize);
                for p in scan {
                    if (p.coords - p0).dot(&n) > tol {
                        if explained.iter().any(|e| e.covers(p, &n)) {
                            by_boss += 1;
                        } else {
                            in_front += 1;
                        }
                    }
                }
                let front = in_front as f64 / total;
                front_of[ni] = Some(front);
                if front <= ENVELOPE_FRACTION {
                    let boss = if by_boss > 0 {
                        format!(
                            "; {:.1} % of the scan in front of it is the boss standing on it",
                            by_boss as f64 / total * 100.0
                        )
                    } else {
                        String::new()
                    };
                    st.set(
                        ni,
                        RoleKind::Global,
                        format!("global plane: {:.3} % of the scan lies in front of it{boss}", front * 100.0),
                    );
                }
            }
            Shape::Cylinder { point, axis, radius } if nd.full && nd.convex => {
                let (o, a) = (v3(*point), v3(*axis));
                let tol = sc.mm(0.1) + 0.05 * radius.min(sc.mm(2.0));
                let out = scan.iter().filter(|p| radial_axial(p, o, a).0 > radius + tol).count() as f64 / total;
                front_of[ni] = Some(out);
                if out <= ENVELOPE_FRACTION {
                    st.set(
                        ni,
                        RoleKind::Global,
                        format!("global cylinder: {:.3} % of the scan lies outside it", out * 100.0),
                    );
                } // else pending: a post of a pocket (an island), or no tool at all (step 5)
            }
            Shape::Cylinder { .. } if !nd.full && nd.convex => {} // pending: a reflex round of a profile, or a fillet (step 6)
            Shape::Cylinder { .. } | Shape::Cone { .. } if !nd.full && !nd.convex => {} // pending: pocket corner or floor fillet
            Shape::Cylinder { .. } => {} // full concave: hole analysis below
            Shape::Cone { .. } if nd.full && !nd.convex => {} // pending: drill point or countersink
            Shape::Cone { .. } if nd.full => st.set(ni, RoleKind::Global, "global cone (full revolution, convex)"),
            Shape::Cone { .. } => {}  // pending: a chamfer of a circular edge (step 6)
            Shape::Torus { .. } => {} // pending: a floor fillet's corner, or a circular fillet (step 6)
            Shape::Sphere { .. } if nd.full => {
                st.set(ni, RoleKind::Global, "global revolved surface (full revolution)")
            }
            Shape::Sphere { .. } => {} // pending: a corner patch of equal fillets, or a dome (step 6)
        }
    }
    // --- 2. holes: a concave full cylinder is through (global) or blind (a tool with a bottom)
    for hi in 0..nodes.len() {
        let Some(nd) = nodes.get(hi) else { continue };
        let Shape::Cylinder { point, axis, radius } = &nd.shape else { continue };
        if !st.pending(hi) || !nd.full || nd.convex {
            continue;
        }
        let (o, a0, r) = (v3(*point), v3(*axis), *radius);
        let (mut t0, mut t1) = (f64::INFINITY, f64::NEG_INFINITY);
        for p in &nd.pts {
            let t = (p.coords - o).dot(&a0);
            t0 = t0.min(t);
            t1 = t1.max(t);
        }
        let window = 0.5 * r + 1.5;
        let (mut low, mut high) = (0usize, 0usize);
        for p in scan {
            let (rho, t) = radial_axial(p, o, a0);
            if rho < 0.6 * r {
                low += usize::from(t > t0 - window && t < t0 + 0.3);
                high += usize::from(t > t1 - 0.3 && t < t1 + window);
            }
        }
        let closed_low = low >= 3;
        let closed_high = high >= 3;
        match (closed_low, closed_high) {
            (false, false) => {
                st.set(hi, RoleKind::Global, "through hole: nothing of the scan closes the cylinder at either end");
                continue;
            }
            (true, true) => {
                st.set(hi, RoleKind::Skipped, "a concave cylinder closed at both ends (a cavity) is not built");
                continue;
            }
            _ => {}
        }
        // u points out of the material, towards the hole's opening
        let u = snap_direction(if closed_low { a0 } else { -a0 }, &global_normals, 0.5);
        // the wall's closed end: the smallest position along u
        let wall_end = nd.pts.iter().map(|p| (p.coords - o).dot(&u)).fold(f64::INFINITY, f64::min);
        // the bottom: a local plane across the axis, or a coaxial cone (a drill point)
        let mut bottom: Option<(usize, HoleBottom, f64, String)> = None;
        for bi in 0..nodes.len() {
            let Some(bn) = nodes.get(bi) else { continue };
            if !st.pending(bi) {
                continue;
            }
            match &bn.shape {
                Shape::Plane { point: bp, normal } => {
                    let n = v3(*normal);
                    if n.dot(&u) < 0.99 {
                        continue;
                    }
                    let near = bn.pts.iter().filter(|p| radial_axial(p, o, u).0 <= r + 1.0).count();
                    if near < 8 {
                        continue;
                    }
                    let s_f = (v3(*bp) - o).dot(&n) / n.dot(&u);
                    if (s_f - wall_end).abs() <= 2.0 {
                        bottom = Some((
                            bi,
                            HoleBottom::Flat,
                            s_f,
                            format!("flat bottom {:.2} mm from the wall's end", s_f - wall_end),
                        ));
                        break;
                    }
                }
                Shape::Cone { axis, .. } if bn.full && !bn.convex => {
                    if v3(*axis).dot(&u).abs() < 0.99 {
                        continue;
                    }
                    // regress the cone's radius on the position along the hole's axis: radius = slope * (s - apex)
                    let pairs: Vec<(f64, f64)> =
                        bn.pts.iter().map(|p| ((p.coords - o).dot(&u), radial_axial(p, o, u).0)).collect();
                    let n = pairs.len() as f64;
                    if n < 10.0 {
                        continue;
                    }
                    let (ms, mr) =
                        (pairs.iter().map(|p| p.0).sum::<f64>() / n, pairs.iter().map(|p| p.1).sum::<f64>() / n);
                    let sxx: f64 = pairs.iter().map(|p| (p.0 - ms).powi(2)).sum();
                    let sxy: f64 = pairs.iter().map(|p| (p.0 - ms) * (p.1 - mr)).sum();
                    if sxx < 1e-9 || sxy <= 0.0 {
                        continue;
                    }
                    let slope = sxy / sxx;
                    let half = slope.atan();
                    let apex_s = ms - mr / slope;
                    let rim_s = apex_s + r / slope;
                    if !(0.3..=1.4).contains(&half) || (rim_s - wall_end).abs() > 1.5 {
                        continue;
                    }
                    bottom = Some((
                        bi,
                        HoleBottom::Cone { half_angle: half },
                        apex_s,
                        format!(
                            "drill point: {:.1} deg included angle, apex {:.2} mm from the wall's end",
                            2.0 * half.to_degrees(),
                            apex_s - wall_end
                        ),
                    ));
                    break;
                }
                _ => {}
            }
        }
        let Some((bi, kind, s, how)) = bottom else {
            st.set(
                hi,
                RoleKind::Skipped,
                "blind hole: its bottom (a plane across the axis or a coaxial cone) was not given",
            );
            continue;
        };
        let base = o + u * s;
        let ti = tools.len();
        tools.push(PlannedTool {
            tool: LocalTool::BlindHole { base: arr(base), axis: arr(u), radius: r, bottom: kind },
            kind: "blind_hole",
            members: vec![nd.base, nodes.get(bi).map_or(nd.base, |b| b.base)],
            detail: format!("Ø{:.3}, {how}", 2.0 * r),
        });
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

    // --- 3. pockets, steps and ribs: a local floor plane with walls and rounds; 4. through slots
    let env = Env {
        nodes: &nodes,
        cloud,
        diag,
        anchors: &global_normals,
        explained: &explained,
        front_of: &front_of,
        scale: cloud.scale(),
    };
    plan_pockets(&env, &mut st, &mut tools);
    plan_slots(&env, &mut st, &mut tools);

    // --- 6. rounds, chamfers and corner patches: concave partial cylinders between planes, tori,
    // partial cones and spheres (a corner of equal fillets, or a dome)
    let kind_of = |st: &State, i: usize| st.kind.get(i).copied().flatten();
    // (0) a partial convex cylinder no profile took is a fillet if it is tangent to two global planes
    for ci in 0..nodes.len() {
        if st.pending(ci)
            && matches!(nodes.get(ci), Some(Node { shape: Shape::Cylinder { .. }, full: false, convex: true, .. }))
        {
            st.set(ci, RoleKind::Blend, "partial convex cylinder: a fillet if it is tangent to two global planes");
        }
    }
    // (a) a concave partial cylinder tangent to two planes (its centre in front of both by r)
    for ci in 0..nodes.len() {
        let Some(nd) = nodes.get(ci) else { continue };
        let Shape::Cylinder { point, axis, radius } = &nd.shape else { continue };
        if nd.full || nd.convex || !st.pending(ci) {
            continue;
        }
        let Some(a) = normalize(v3(*axis)) else { continue };
        let (c0, r) = (v3(*point), *radius);
        let normals: Vec<Vec3> = nodes
            .iter()
            .enumerate()
            .filter(|(pi, _)| {
                *pi != ci
                    && matches!(
                        kind_of(&st, *pi),
                        Some(RoleKind::Global | RoleKind::LocalSubtract | RoleKind::LocalAdd)
                    )
            })
            .filter_map(|(_, pn)| match &pn.shape {
                Shape::Plane { point: pp, normal } => {
                    let n = normalize(v3(*normal))?;
                    (n.dot(&a).abs() < 0.05 && ((c0 - v3(*pp)).dot(&n) - r).abs() < 0.08 * r + 0.15).then_some(n)
                }
                _ => None,
            })
            .collect();
        if normals.iter().enumerate().any(|(x, n)| normals.iter().skip(x + 1).any(|m| n.dot(m).abs() < 0.95)) {
            st.set(ci, RoleKind::Blend, format!("fillet of a concave edge: r = {r:.2}, tangent to two planes"));
        }
    }
    // (b) a torus tangent to a plane across its axis and to a coaxial cylinder: the circular
    // fillet of the foot of a boss (concave) or of the rim of a disc (convex)
    for ti in 0..nodes.len() {
        let Some(nd) = nodes.get(ti) else { continue };
        let Shape::Torus { center, axis, major_radius, minor_radius } = &nd.shape else { continue };
        if !st.pending(ti) {
            continue;
        }
        let (Some(a), c, mj, m) = (normalize(v3(*axis)), v3(*center), *major_radius, *minor_radius) else { continue };
        let sgn = if nd.convex { -1.0 } else { 1.0 };
        let want = if nd.convex { mj + m } else { mj - m };
        let plane = nodes.iter().enumerate().any(|(pi, pn)| {
            kind_of(&st, pi) == Some(RoleKind::Global)
                && match &pn.shape {
                    Shape::Plane { point: pp, normal } => normalize(v3(*normal)).is_some_and(|n| {
                        n.dot(&a).abs() > 0.99 && ((c - v3(*pp)).dot(&n) - sgn * m).abs() < 0.4 + 0.15 * m
                    }),
                    _ => false,
                }
        });
        let cyl = nodes.iter().enumerate().find_map(|(pi, pn)| {
            if !matches!(kind_of(&st, pi), Some(RoleKind::Global | RoleKind::LocalAdd)) {
                return None;
            }
            match &pn.shape {
                Shape::Cylinder { point, axis, radius } if pn.full => {
                    let ca = normalize(v3(*axis))?;
                    let ok = ca.dot(&a).abs() > 0.99
                        && radial_axial(&Point3::from(c), v3(*point), ca).0 < 1.0
                        && (radius - want).abs() < 0.5 + 0.2 * m;
                    ok.then(|| {
                        // the torus on the cylinder's own axis, with its sign
                        let ca = if ca.dot(&a) < 0.0 { -ca } else { ca };
                        let on = v3(*point) + ca * (c - v3(*point)).dot(&ca);
                        (on, ca, *radius)
                    })
                }
                _ => None,
            }
        });
        match (plane, cyl) {
            (true, Some((on, ca, rc))) => {
                st.set(
                    ti,
                    RoleKind::Blend,
                    format!(
                        "{} circular fillet, minor radius {m:.2}, tangent to a plane and a cylinder of radius {rc:.2}",
                        if nd.convex { "convex (rim)" } else { "concave (foot)" }
                    ),
                );
                if let Some(sh) = st.shape.get_mut(ti) {
                    *sh = Shape::Torus {
                        center: arr(on),
                        axis: arr(ca),
                        major_radius: if nd.convex { rc - m } else { rc + m },
                        minor_radius: m,
                    };
                }
            }
            _ if nd.full => st.set(ti, RoleKind::Global, "global revolved surface (full revolution)"),
            _ => st.note(ti, "no plane tangent to its tube and coaxial cylinder was found"),
        }
    }
    // (c) a partial convex cone: the chamfer of a circular edge between a plane and a coaxial cylinder
    for ci in 0..nodes.len() {
        let Some(nd) = nodes.get(ci) else { continue };
        let Shape::Cone { apex, axis, .. } = &nd.shape else { continue };
        if nd.full || !nd.convex || !st.pending(ci) {
            continue;
        }
        let (Some(a), ap) = (normalize(v3(*axis)), v3(*apex)) else { continue };
        let plane = nodes.iter().enumerate().any(|(pi, pn)| {
            kind_of(&st, pi) == Some(RoleKind::Global)
                && matches!(&pn.shape, Shape::Plane { normal, .. } if normalize(v3(*normal)).is_some_and(|n| n.dot(&a).abs() > 0.99))
        });
        let cyl = nodes.iter().enumerate().any(|(pi, pn)| {
            matches!(kind_of(&st, pi), Some(RoleKind::Global | RoleKind::LocalAdd))
                && matches!(&pn.shape, Shape::Cylinder { point, axis, .. } if pn.full
                    && normalize(v3(*axis)).is_some_and(|ca| ca.dot(&a).abs() > 0.99)
                    && radial_axial(&Point3::from(ap), v3(*point), a).0 < 1.0)
        });
        if plane && cyl {
            st.set(ci, RoleKind::Blend, "chamfer of a circular edge between a plane and a coaxial cylinder");
        } else {
            st.note(ci, "no plane across its axis and coaxial cylinder for it to chamfer");
        }
    }
    // (d) a partial sphere: the corner patch of equal fillets, or a dome standing on a plane
    for si in 0..nodes.len() {
        let Some(nd) = nodes.get(si) else { continue };
        let Shape::Sphere { center, radius } = &nd.shape else { continue };
        if !st.pending(si) {
            continue;
        }
        let (cs, rs) = (v3(*center), *radius);
        if !nd.convex {
            st.set(si, RoleKind::Skipped, "a concave partial sphere (a dimple) is not built");
            continue;
        }
        let fillets = nodes
            .iter()
            .enumerate()
            .filter(|(pi, pn)| {
                *pi != si
                    && kind_of(&st, *pi) == Some(RoleKind::Blend)
                    && matches!(&pn.shape, Shape::Cylinder { point, axis, radius: rr }
                        if (rr - rs).abs() <= 0.1 * rs + 0.15
                            && normalize(v3(*axis)).is_some_and(|ca| radial_axial(&Point3::from(cs), v3(*point), ca).0 <= 0.25 * rs + 0.3))
            })
            .count();
        if fillets >= 2 {
            st.set(
                si,
                RoleKind::Blend,
                format!("corner patch of {fillets} fillets of radius {rs:.2} (the kernel makes it when they meet)"),
            );
            continue;
        }
        // a dome: the plane it stands on has the sphere's lowest points in it and cuts the ball
        let mut best: Option<(f64, Vec3, Vec3)> = None;
        for (pi, pn) in nodes.iter().enumerate() {
            if kind_of(&st, pi) != Some(RoleKind::Global) {
                continue;
            }
            let Shape::Plane { point: pp, normal } = &pn.shape else { continue };
            let (Some(n), p0) = (normalize(v3(*normal)), v3(*pp)) else { continue };
            let hs: Vec<f64> = nd.pts.iter().map(|p| (p.coords - p0).dot(&n)).collect();
            let low = hs.iter().copied().fold(f64::INFINITY, f64::min);
            let below = hs.iter().filter(|&&h| h < -0.5).count();
            if low > 0.8 || below * 50 > hs.len() || ((cs - p0).dot(&n)).abs() > rs - 0.2 {
                continue;
            }
            if best.as_ref().is_none_or(|b| low.abs() < b.0) {
                best = Some((low.abs(), p0, n));
            }
        }
        match best {
            Some((_, p0, n)) => {
                let ai = adds.len();
                adds.push(PlannedAdd {
                    add: LocalAdd::Dome { center: arr(cs), radius: rs, plane_point: arr(p0), normal: arr(n) },
                    kind: "dome",
                    members: vec![nd.base],
                    detail: format!("ball of radius {rs:.3} above a plane"),
                });
                st.set(
                    si,
                    RoleKind::LocalAdd,
                    format!("dome: the cap of a ball of radius {rs:.3} standing on a plane"),
                );
                if let Some(v) = st.adds.get_mut(si) {
                    v.push(ai);
                }
            }
            None => st.set(
                si,
                RoleKind::Skipped,
                "a convex partial sphere that is not the corner of equal fillets and stands on no global plane",
            ),
        }
    }

    // --- 5. what is left
    for ni in 0..nodes.len() {
        if !st.pending(ni) {
            continue;
        }
        let Some(nd) = nodes.get(ni) else { continue };
        let note = st.notes.get(ni).filter(|n| !n.is_empty()).map_or(String::new(), |n| format!("; {n}"));
        match &nd.shape {
            Shape::Cone { .. } if nd.convex => {
                st.set(ni, RoleKind::Skipped, format!("a partial convex cone that is no chamfer of a circular edge{note}"));
            }
            Shape::Cone { .. } => st.set(ni, RoleKind::Global, "concave cone (countersink): subtracted from the stock"),
            Shape::Torus { .. } | Shape::Sphere { .. } => {
                st.set(ni, RoleKind::Skipped, format!("a partial torus or sphere that is no fillet, corner patch or dome{note}"));
            }
            Shape::Cylinder { .. } if nd.full && nd.convex => st.set(
                ni,
                RoleKind::Skipped,
                format!(
                    "boss or step: {:.1} % of the scan lies outside this convex cylinder and it is no boss on a plane or post in a pocket{note}",
                    front_of.get(ni).copied().flatten().unwrap_or(0.0) * 100.0
                ),
            ),
            Shape::Plane { .. } => st.set(
                ni,
                RoleKind::Skipped,
                format!(
                    "local plane ({:.1} % of the scan lies in front of it) that is not the floor or a wall of a recognised pocket, hole or slot{note}",
                    front_of.get(ni).copied().flatten().unwrap_or(0.0) * 100.0
                ),
            ),
            _ => st.set(ni, RoleKind::Skipped, format!("a partial concave surface that is not the wall of a recognised pocket or slot{note}")),
        }
    }

    // --- the plan for the kernel and the report
    let mut cuts: Vec<Cut> = Vec::new();
    let mut cut_members: Vec<Vec<usize>> = Vec::new();
    let mut squared: Vec<SquaredPlane> = Vec::new();
    let mut roles: Vec<Option<Role>> = vec![None; items.len()];
    for (ni, nd) in nodes.iter().enumerate() {
        let kind = st.kind.get(ni).copied().flatten().unwrap_or(RoleKind::Skipped);
        let mut how = st.how.get(ni).cloned().unwrap_or_default();
        let tl = st.tools.get(ni).cloned().unwrap_or_default();
        let al = st.adds.get(ni).cloned().unwrap_or_default();
        let mut shape = st.shape.get(ni).cloned().unwrap_or_else(|| nd.shape.clone());
        // Auto-constraint (design doc §6.2), switchable by `square_deg` (0 disables): a global plane
        // within `square_deg` of a model axis is made square to it, about the fit's own point. A
        // plane off by fit noise would make the circle where a boss's cylinder meets it an
        // ellipse, and the kernel's blend would then see no circular edge. It is not squared when
        // that would move it over its region by more than max(2 sigma, 0.05 mm): a real draft.
        if square_deg > 0.0
            && kind == RoleKind::Global
            && let Shape::Plane { point, normal } = &shape
            && let Some(n) = normalize(v3(*normal))
        {
            let snapped = snap_direction(n, &global_normals, square_deg);
            let angle = n.dot(&snapped).clamp(-1.0, 1.0).acos().to_degrees();
            if angle > 1e-9 {
                let p0 = v3(*point);
                let rms = (nd.pts.iter().map(|p| (p.coords - p0).dot(&n).powi(2)).sum::<f64>()
                    / nd.pts.len().max(1) as f64)
                    .sqrt();
                let limit = (2.0 * rms).max(sc.mm(0.05));
                let shift = nd.pts.iter().map(|p| (p.coords - p0).dot(&(snapped - n)).abs()).fold(0.0, f64::max);
                let applied = shift <= limit;
                squared.push(SquaredPlane { item: nd.base, angle_deg: angle, max_shift: shift, limit, applied });
                if applied {
                    shape = Shape::Plane { point: *point, normal: arr(snapped) };
                    how.push_str(&format!("; squared to a model axis ({angle:.3} deg, up to {shift:.3} mm)"));
                } else {
                    how.push_str(&format!(
                        "; not squared: {angle:.3} deg would move it up to {shift:.3} mm (limit {limit:.3} mm)"
                    ));
                }
            }
        }
        if matches!(kind, RoleKind::Global | RoleKind::Blend) {
            // a blend is a partial surface for the kernel whatever the region's extent round its axis
            cuts.push(Cut { shape: shape.clone(), convex: nd.convex, full: nd.full && kind == RoleKind::Global });
            cut_members.push(nd.members.clone());
        }
        for &m in &nd.members {
            let merged = if nd.members.len() > 1 {
                format!(" (merged with {} coplanar fragments)", nd.members.len() - 1)
            } else {
                String::new()
            };
            if let Some(r) = roles.get_mut(m) {
                *r = Some(Role {
                    kind,
                    how: format!("{how}{merged}"),
                    tools: tl.clone(),
                    adds: al.clone(),
                    shape: shape.clone(),
                });
            }
        }
    }
    let roles = roles
        .into_iter()
        .zip(items)
        .map(|(r, it)| {
            r.unwrap_or(Role {
                kind: RoleKind::Skipped,
                how: "not planned".into(),
                tools: Vec::new(),
                adds: Vec::new(),
                shape: it.cut.shape.clone(),
            })
        })
        .collect();
    // tool members are node bases; every fragment of a merged plane shares the node's tools already
    let _ = (min_dist, pt as fn([f64; 3]) -> Point3);
    Plan { cuts, cut_members, tools, adds, roles, contacts, squared }
}

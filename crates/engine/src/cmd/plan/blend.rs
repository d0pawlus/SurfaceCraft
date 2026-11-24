//! Step 6 of the planner, rounds, chamfers and corner patches: concave partial cylinders between
//! planes, tori, partial cones and spheres (a corner of equal fillets, or a dome).

use super::*;

/// The plane `pn` as a candidate base of a dome on the sphere `(cs, rs)` whose points are `pts`:
/// its lowest sphere point's height above it, a point on it and its unit normal. `None` when the
/// plane does not hold the sphere's lowest points or does not cut the ball.
fn dome_candidate(pn: &Node, pts: &[Point3], cs: Vec3, rs: f64) -> Option<(f64, Vec3, Vec3)> {
    let Shape::Plane { point: pp, normal } = &pn.shape else { return None };
    let (Some(n), p0) = (normalize(v3(*normal)), v3(*pp)) else { return None };
    let hs: Vec<f64> = pts.iter().map(|p| (p.coords - p0).dot(&n)).collect();
    let low = hs.iter().copied().fold(f64::INFINITY, f64::min);
    let below = hs.iter().filter(|&&h| h < -0.5).count();
    if low > 0.8 || below * 50 > hs.len() || ((cs - p0).dot(&n)).abs() > rs - 0.2 {
        return None;
    }
    Some((low, p0, n))
}

impl Planner<'_> {
    /// Step 6: the rounds, chamfers and corner patches among the regions still pending.
    pub(super) fn plan_blends(&mut self) {
        self.mark_convex_cylinders();
        for ci in 0..self.nodes.len() {
            self.concave_cylinder_fillet(ci);
        }
        for ti in 0..self.nodes.len() {
            self.circular_fillet(ti);
        }
        for ci in 0..self.nodes.len() {
            self.circular_chamfer(ci);
        }
        for si in 0..self.nodes.len() {
            self.sphere_patch(si);
        }
    }

    /// (0) a partial convex cylinder no profile took is a fillet if it is tangent to two global planes
    fn mark_convex_cylinders(&mut self) {
        for ci in 0..self.nodes.len() {
            if self.st.pending(ci)
                && matches!(
                    self.nodes.get(ci),
                    Some(Node { shape: Shape::Cylinder { .. }, full: false, convex: true, .. })
                )
            {
                self.st.set(
                    ci,
                    RoleKind::Blend,
                    "partial convex cylinder: a fillet if it is tangent to two global planes",
                );
            }
        }
    }

    /// The unit normals of the planes already placed (not `ci`) that the cylinder with the axis
    /// `a` through `c0` and the radius `r` is tangent to (its centre in front of the plane by `r`).
    fn tangent_plane_normals(&self, ci: usize, c0: Vec3, a: Vec3, r: f64) -> Vec<Vec3> {
        self.nodes
            .iter()
            .enumerate()
            .filter(|(pi, _)| {
                *pi != ci
                    && matches!(
                        self.kind_of(*pi),
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
            .collect()
    }

    /// (a) a concave partial cylinder tangent to two planes (its centre in front of both by r)
    fn concave_cylinder_fillet(&mut self, ci: usize) {
        let nodes = self.nodes;
        let Some(nd) = nodes.get(ci) else { return };
        let Shape::Cylinder { point, axis, radius } = &nd.shape else { return };
        if nd.full || nd.convex || !self.st.pending(ci) {
            return;
        }
        let Some(a) = normalize(v3(*axis)) else { return };
        let (c0, r) = (v3(*point), *radius);
        let normals = self.tangent_plane_normals(ci, c0, a, r);
        if normals.iter().enumerate().any(|(x, n)| normals.iter().skip(x + 1).any(|m| n.dot(m).abs() < 0.95)) {
            self.st.set(ci, RoleKind::Blend, format!("fillet of a concave edge: r = {r:.2}, tangent to two planes"));
        }
    }

    /// Whether a global plane across the axis `a` is tangent to the tube of the torus with the
    /// centre `c` and the minor radius `m` (on the side `sgn`).
    fn plane_tangent_to_tube(&self, a: Vec3, c: Vec3, sgn: f64, m: f64) -> bool {
        self.nodes.iter().enumerate().any(|(pi, pn)| {
            self.kind_of(pi) == Some(RoleKind::Global)
                && match &pn.shape {
                    Shape::Plane { point: pp, normal } => normalize(v3(*normal)).is_some_and(|n| {
                        n.dot(&a).abs() > 0.99 && ((c - v3(*pp)).dot(&n) - sgn * m).abs() < 0.4 + 0.15 * m
                    }),
                    _ => false,
                }
        })
    }

    /// The first global or added full cylinder coaxial with the torus (axis `a`, centre `c`, minor
    /// radius `m`) whose radius is `want`: the point of its axis the torus is centred on, its
    /// direction (towards `a`) and its radius.
    fn coaxial_cylinder(&self, a: Vec3, c: Vec3, want: f64, m: f64) -> Option<(Vec3, Vec3, f64)> {
        self.nodes.iter().enumerate().find_map(|(pi, pn)| {
            if !matches!(self.kind_of(pi), Some(RoleKind::Global | RoleKind::LocalAdd)) {
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
        })
    }

    /// (b) a torus tangent to a plane across its axis and to a coaxial cylinder: the circular
    /// fillet of the foot of a boss (concave) or of the rim of a disc (convex)
    fn circular_fillet(&mut self, ti: usize) {
        let nodes = self.nodes;
        let Some(nd) = nodes.get(ti) else { return };
        let Shape::Torus { center, axis, major_radius, minor_radius } = &nd.shape else { return };
        if !self.st.pending(ti) {
            return;
        }
        let (Some(a), c, mj, m) = (normalize(v3(*axis)), v3(*center), *major_radius, *minor_radius) else { return };
        let sgn = if nd.convex { -1.0 } else { 1.0 };
        let want = if nd.convex { mj + m } else { mj - m };
        let plane = self.plane_tangent_to_tube(a, c, sgn, m);
        let cyl = self.coaxial_cylinder(a, c, want, m);
        match (plane, cyl) {
            (true, Some((on, ca, rc))) => self.set_circular_fillet(ti, nd, m, (on, ca, rc)),
            _ if nd.full => self.st.set(ti, RoleKind::Global, "global revolved surface (full revolution)"),
            _ => self.st.note(ti, "no plane tangent to its tube and coaxial cylinder was found"),
        }
    }

    /// Makes the torus `ti` (minor radius `m`) a circular fillet of the cylinder `(on, ca, rc)`
    /// (a point on its axis, its direction and its radius) and plans its torus on that axis.
    fn set_circular_fillet(&mut self, ti: usize, nd: &Node, m: f64, (on, ca, rc): (Vec3, Vec3, f64)) {
        self.st.set(
            ti,
            RoleKind::Blend,
            format!(
                "{} circular fillet, minor radius {m:.2}, tangent to a plane and a cylinder of radius {rc:.2}",
                if nd.convex { "convex (rim)" } else { "concave (foot)" }
            ),
        );
        if let Some(sh) = self.st.shape.get_mut(ti) {
            *sh = Shape::Torus {
                center: arr(on),
                axis: arr(ca),
                major_radius: if nd.convex { rc - m } else { rc + m },
                minor_radius: m,
            };
        }
    }

    /// Whether a global plane across the axis `a` exists.
    fn plane_across(&self, a: Vec3) -> bool {
        self.nodes.iter().enumerate().any(|(pi, pn)| {
            self.kind_of(pi) == Some(RoleKind::Global)
                && matches!(&pn.shape, Shape::Plane { normal, .. } if normalize(v3(*normal)).is_some_and(|n| n.dot(&a).abs() > 0.99))
        })
    }

    /// Whether a global or added full cylinder coaxial with the axis `a` through the apex `ap` exists.
    fn coaxial_cylinder_exists(&self, a: Vec3, ap: Vec3) -> bool {
        self.nodes.iter().enumerate().any(|(pi, pn)| {
            matches!(self.kind_of(pi), Some(RoleKind::Global | RoleKind::LocalAdd))
                && matches!(&pn.shape, Shape::Cylinder { point, axis, .. } if pn.full
                    && normalize(v3(*axis)).is_some_and(|ca| ca.dot(&a).abs() > 0.99)
                    && radial_axial(&Point3::from(ap), v3(*point), a).0 < 1.0)
        })
    }

    /// (c) a partial convex cone: the chamfer of a circular edge between a plane and a coaxial cylinder
    fn circular_chamfer(&mut self, ci: usize) {
        let nodes = self.nodes;
        let Some(nd) = nodes.get(ci) else { return };
        let Shape::Cone { apex, axis, .. } = &nd.shape else { return };
        if nd.full || !nd.convex || !self.st.pending(ci) {
            return;
        }
        let (Some(a), ap) = (normalize(v3(*axis)), v3(*apex)) else { return };
        let plane = self.plane_across(a);
        let cyl = self.coaxial_cylinder_exists(a, ap);
        if plane && cyl {
            self.st.set(ci, RoleKind::Blend, "chamfer of a circular edge between a plane and a coaxial cylinder");
        } else {
            self.st.note(ci, "no plane across its axis and coaxial cylinder for it to chamfer");
        }
    }

    /// The number of fillets (blends, cylinders of radius `rs` through the centre `cs`) that meet
    /// in the sphere `si`.
    fn corner_fillets(&self, si: usize, cs: Vec3, rs: f64) -> usize {
        self.nodes
            .iter()
            .enumerate()
            .filter(|(pi, pn)| {
                *pi != si
                    && self.kind_of(*pi) == Some(RoleKind::Blend)
                    && matches!(&pn.shape, Shape::Cylinder { point, axis, radius: rr }
                        if (rr - rs).abs() <= 0.1 * rs + 0.15
                            && normalize(v3(*axis)).is_some_and(|ca| radial_axial(&Point3::from(cs), v3(*point), ca).0 <= 0.25 * rs + 0.3))
            })
            .count()
    }

    /// The global plane a dome on the sphere `nd` (centre `cs`, radius `rs`) stands on, as a point
    /// on it and its unit normal: the one holding the sphere's lowest points (the first of equal ones).
    fn dome_plane(&self, nd: &Node, cs: Vec3, rs: f64) -> Option<(Vec3, Vec3)> {
        let mut best: Option<(f64, Vec3, Vec3)> = None;
        for (pi, pn) in self.nodes.iter().enumerate() {
            if self.kind_of(pi) != Some(RoleKind::Global) {
                continue;
            }
            let Some((low, p0, n)) = dome_candidate(pn, &nd.pts, cs, rs) else { continue };
            if best.as_ref().is_none_or(|b| low.abs() < b.0) {
                best = Some((low.abs(), p0, n));
            }
        }
        best.map(|(_, p0, n)| (p0, n))
    }

    /// Forms the dome on the plane `(p0, n)` from the sphere region `si`.
    fn add_dome(&mut self, si: usize, nd: &Node, cs: Vec3, rs: f64, (p0, n): (Vec3, Vec3)) {
        let ai = self.adds.len();
        self.adds.push(PlannedAdd {
            add: LocalAdd::Dome { center: arr(cs), radius: rs, plane_point: arr(p0), normal: arr(n) },
            kind: "dome",
            members: vec![nd.base],
            detail: format!("ball of radius {rs:.3} above a plane"),
        });
        self.st.set(si, RoleKind::LocalAdd, format!("dome: the cap of a ball of radius {rs:.3} standing on a plane"));
        if let Some(v) = self.st.adds.get_mut(si) {
            v.push(ai);
        }
    }

    /// (d) a partial sphere: the corner patch of equal fillets, or a dome standing on a plane
    fn sphere_patch(&mut self, si: usize) {
        let nodes = self.nodes;
        let Some(nd) = nodes.get(si) else { return };
        let Shape::Sphere { center, radius } = &nd.shape else { return };
        if !self.st.pending(si) {
            return;
        }
        let (cs, rs) = (v3(*center), *radius);
        if !nd.convex {
            self.st.set(si, RoleKind::Skipped, "a concave partial sphere (a dimple) is not built");
            return;
        }
        let fillets = self.corner_fillets(si, cs, rs);
        if fillets >= 2 {
            self.st.set(
                si,
                RoleKind::Blend,
                format!("corner patch of {fillets} fillets of radius {rs:.2} (the kernel makes it when they meet)"),
            );
            return;
        }
        // a dome: the plane it stands on has the sphere's lowest points in it and cuts the ball
        match self.dome_plane(nd, cs, rs) {
            Some(plane) => self.add_dome(si, nd, cs, rs, plane),
            None => self.st.set(
                si,
                RoleKind::Skipped,
                "a convex partial sphere that is not the corner of equal fillets and stands on no global plane",
            ),
        }
    }
}

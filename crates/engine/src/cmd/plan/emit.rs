//! The last step of the planner: the plan for the kernel and the report, from the roles of the nodes.

use super::*;

/// What the nodes are turned into: the cuts for the kernel (with the items of each), the planes
/// the planner tried to square and the role of every item.
struct Emitted {
    cuts: Vec<Cut>,
    cut_members: Vec<Vec<usize>>,
    squared: Vec<SquaredPlane>,
    roles: Vec<Option<Role>>,
}

impl Planner<'_> {
    /// The plan for the kernel and the report: every node becomes a role for each of its items, and
    /// the global and blend nodes become cuts.
    pub(super) fn into_plan(self, items: &[Item], contacts: Vec<Contact>, square_deg: f64) -> Plan {
        let mut out =
            Emitted { cuts: Vec::new(), cut_members: Vec::new(), squared: Vec::new(), roles: vec![None; items.len()] };
        for (ni, nd) in self.nodes.iter().enumerate() {
            self.emit_node(&mut out, ni, nd, square_deg);
        }
        let roles = out
            .roles
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
        Plan {
            cuts: out.cuts,
            cut_members: out.cut_members,
            tools: self.tools,
            adds: self.adds,
            roles,
            contacts,
            squared: out.squared,
        }
    }

    /// Turns the node `ni` into its cut (a global or blend node) and into a role for each of its items.
    fn emit_node(&self, out: &mut Emitted, ni: usize, nd: &Node, square_deg: f64) {
        let kind = self.st.kind.get(ni).copied().flatten().unwrap_or(RoleKind::Skipped);
        let mut how = self.st.how.get(ni).cloned().unwrap_or_default();
        let tl = self.st.tools.get(ni).cloned().unwrap_or_default();
        let al = self.st.adds.get(ni).cloned().unwrap_or_default();
        let mut shape = self.st.shape.get(ni).cloned().unwrap_or_else(|| nd.shape.clone());
        self.square_plane(&mut out.squared, nd, kind, &mut shape, &mut how, square_deg);
        if matches!(kind, RoleKind::Global | RoleKind::Blend) {
            // a blend is a partial surface for the kernel whatever the region's extent round its axis
            out.cuts.push(Cut { shape: shape.clone(), convex: nd.convex, full: nd.full && kind == RoleKind::Global });
            out.cut_members.push(nd.members.clone());
        }
        for &m in &nd.members {
            let merged = if nd.members.len() > 1 {
                format!(" (merged with {} coplanar fragments)", nd.members.len() - 1)
            } else {
                String::new()
            };
            if let Some(r) = out.roles.get_mut(m) {
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

    /// How far squaring the plane of `nd` (fitted through `point` with the unit normal `n`) to the
    /// model direction `snapped`, `angle` degrees away, moves it over its region, and whether that
    /// is within the limit.
    fn squaring(&self, nd: &Node, point: [f64; 3], n: Vec3, snapped: Vec3, angle: f64) -> SquaredPlane {
        let p0 = v3(point);
        let rms = plane_rms(&nd.pts, p0, n);
        let limit = (2.0 * rms).max(self.sc.mm(0.05));
        let shift = nd.pts.iter().map(|p| (p.coords - p0).dot(&(snapped - n)).abs()).fold(0.0, f64::max);
        let applied = shift <= limit;
        SquaredPlane { item: nd.base, angle_deg: angle, max_shift: shift, limit, applied }
    }

    /// Auto-constraint (design doc §6.2), switchable by `square_deg` (0 disables): a global plane
    /// within `square_deg` of a model axis is made square to it, about the fit's own point. A
    /// plane off by fit noise would make the circle where a boss's cylinder meets it an
    /// ellipse, and the kernel's blend would then see no circular edge. It is not squared when
    /// that would move it over its region by more than max(2 sigma, 0.05 mm): a real draft.
    fn square_plane(
        &self,
        squared: &mut Vec<SquaredPlane>,
        nd: &Node,
        kind: RoleKind,
        shape: &mut Shape,
        how: &mut String,
        square_deg: f64,
    ) {
        if square_deg > 0.0
            && kind == RoleKind::Global
            && let Shape::Plane { point, normal } = &*shape
            && let Some(n) = normalize(v3(*normal))
        {
            let point = *point;
            let snapped = snap_direction(n, self.anchors, square_deg);
            let angle = n.dot(&snapped).clamp(-1.0, 1.0).acos().to_degrees();
            if angle > 1e-9 {
                let sq = self.squaring(nd, point, n, snapped, angle);
                let (shift, limit, applied) = (sq.max_shift, sq.limit, sq.applied);
                squared.push(sq);
                if applied {
                    *shape = Shape::Plane { point, normal: arr(snapped) };
                    how.push_str(&format!("; squared to a model axis ({angle:.3} deg, up to {shift:.3} mm)"));
                } else {
                    how.push_str(&format!(
                        "; not squared: {angle:.3} deg would move it up to {shift:.3} mm (limit {limit:.3} mm)"
                    ));
                }
            }
        }
    }
}

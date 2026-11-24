//! The planner itself: the state of one planning pass and the plan of all regions.

use super::boss::*;
use super::nodes::*;
use super::scan::Scale;
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
    /// The state of a pass that has planned nothing yet: every node pending, with its fitted shape.
    fn new(nodes: &[Node]) -> State {
        State {
            kind: vec![None; nodes.len()],
            how: vec![String::new(); nodes.len()],
            notes: vec![String::new(); nodes.len()],
            tools: vec![Vec::new(); nodes.len()],
            adds: vec![Vec::new(); nodes.len()],
            shape: nodes.iter().map(|n| n.shape.clone()).collect(),
        }
    }
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

/// The state of one planning pass: the inputs and everything the phases accumulate.
pub(super) struct Planner<'a> {
    /// The regions as nodes.
    pub(super) nodes: &'a [Node],
    /// The whole scanned cloud the regions come from.
    pub(super) cloud: &'a Scan,
    /// The points of the cloud.
    pub(super) scan: &'a [Point3],
    /// The lengths of the planner at the cloud's spacing.
    pub(super) sc: Scale,
    /// The model axes the tools' directions snap to.
    pub(super) anchors: &'a [Vec3],
    /// Diagonal of the box that holds the scan (mm).
    pub(super) diag: f64,
    /// Number of scan points (at least one), as a divisor.
    pub(super) total: f64,
    /// Roles, reasons, tool and addition memberships and planned shapes per node.
    pub(super) st: State,
    /// The local tools formed so far.
    pub(super) tools: Vec<PlannedTool>,
    /// The local additions formed so far.
    pub(super) adds: Vec<PlannedAdd>,
    /// The scan the bosses explain.
    pub(super) explained: Vec<Explained>,
    /// Per node: the share of the scan in front of it (or outside it), once measured.
    pub(super) front_of: Vec<Option<f64>>,
}

/// The diagonal of the box that holds `scan` (100 mm for no usable points).
fn scan_diagonal(scan: &[Point3]) -> f64 {
    let mut lo = Vec3::repeat(f64::INFINITY);
    let mut hi = Vec3::repeat(f64::NEG_INFINITY);
    for p in scan {
        lo = lo.inf(&p.coords);
        hi = hi.sup(&p.coords);
    }
    if lo.iter().all(|v| v.is_finite()) { (hi - lo).norm() } else { 100.0 }
}

/// Root mean square distance of `pts` from the plane through `p0` with normal `n`.
pub(super) fn plane_rms(pts: &[Point3], p0: Vec3, n: Vec3) -> f64 {
    (pts.iter().map(|p| (p.coords - p0).dot(&n).powi(2)).sum::<f64>() / pts.len().max(1) as f64).sqrt()
}

/// Plans the build: classifies every region and forms the local tools. `scan` is the whole
/// scanned cloud the regions come from (the "is there scan in front of it?" test).
#[must_use]
pub fn plan(items: &[Item], cloud: &Scan, square_deg: f64) -> Plan {
    let nodes = make_nodes(items);
    let contacts = contacts(&nodes);
    // Auto-constraint of directions (design doc §6.2): the tools' directions (a pocket's, a hole's
    // axis, a slot's) snap to the model axes when within half a degree. The kernel's booleans fail
    // on a tool whose direction is, to the last digit, the normal of one of the stock's fitted
    // planes (observed on a window in a plate); a model axis is not.
    let global_normals: Vec<Vec3> = vec![Vec3::x(), Vec3::y(), Vec3::z()];
    let mut planner = Planner::new(&nodes, cloud, &global_normals);
    planner.plan_bosses();
    planner.classify_global();
    planner.plan_holes();
    planner.plan_local_tools();
    planner.plan_blends();
    planner.settle_leftovers();
    let plan = planner.into_plan(items, contacts, square_deg);
    // tool members are node bases; every fragment of a merged plane shares the node's tools already
    let _ = (min_dist, pt as fn([f64; 3]) -> Point3);
    plan
}

impl<'a> Planner<'a> {
    /// A planner over `nodes` with every region pending and nothing planned yet.
    pub(super) fn new(nodes: &'a [Node], cloud: &'a Scan, anchors: &'a [Vec3]) -> Planner<'a> {
        let scan: &'a [Point3] = &cloud.points;
        Planner {
            nodes,
            cloud,
            scan,
            sc: cloud.scale(),
            anchors,
            diag: scan_diagonal(scan),
            total: scan.len().max(1) as f64,
            st: State::new(nodes),
            tools: Vec::new(),
            adds: Vec::new(),
            explained: Vec::new(),
            front_of: vec![None; nodes.len()],
        }
    }

    /// The role node `i` has been given, if any.
    pub(super) fn kind_of(&self, i: usize) -> Option<RoleKind> {
        self.st.kind.get(i).copied().flatten()
    }

    /// Records the share of the scan in front of node `ni` (or outside it).
    fn set_front(&mut self, ni: usize, share: f64) {
        if let Some(f) = self.front_of.get_mut(ni) {
            *f = Some(share);
        }
    }

    /// The share of the scan outside the cylinder with the axis `(point, axis)` and the radius
    /// `radius`, beyond a tolerance that scales with the scan's spacing.
    fn outside_cylinder(&self, point: [f64; 3], axis: [f64; 3], radius: f64) -> f64 {
        let (o, a) = (v3(point), v3(axis));
        let tol = self.sc.mm(0.1) + 0.05 * radius.min(self.sc.mm(2.0));
        self.scan.iter().filter(|p| radial_axial(p, o, a).0 > radius + tol).count() as f64 / self.total
    }

    /// Step 0, bosses: a convex full cylinder with scan outside it that stands on a plane. What is
    /// in front of that plane because of the boss is not an obstacle to the plane being global.
    pub(super) fn plan_bosses(&mut self) {
        for ci in 0..self.nodes.len() {
            self.try_boss(ci);
        }
    }

    /// Measures how much scan lies outside the convex full cylinder `ci` and plans a boss from it
    /// when that is more than the envelope allows.
    fn try_boss(&mut self, ci: usize) {
        let nodes = self.nodes;
        let Some(nd) = nodes.get(ci) else { return };
        let Shape::Cylinder { point, axis, radius } = &nd.shape else { return };
        if !(nd.full && nd.convex) || !self.st.pending(ci) {
            return;
        }
        let out = self.outside_cylinder(*point, *axis, *radius);
        self.set_front(ci, out);
        if out <= ENVELOPE_FRACTION {
            return;
        }
        let ai = self.adds.len();
        match plan_boss(nodes, ci, &mut self.st, self.anchors, ai, out) {
            Ok((add, ex)) => {
                self.adds.push(add);
                self.explained.push(ex);
            }
            Err(why) => self.st.note(ci, why),
        }
    }

    /// Step 1, global or local, from the scan: planes and surfaces of revolution with no scan in
    /// front of them are global cuts; the rest stay pending for the later steps.
    pub(super) fn classify_global(&mut self) {
        let nodes = self.nodes;
        for (ni, nd) in nodes.iter().enumerate() {
            if !self.st.pending(ni) {
                continue;
            }
            match &nd.shape {
                Shape::Plane { point, normal } => self.classify_plane(ni, nd, *point, *normal),
                Shape::Cylinder { point, axis, radius } if nd.full && nd.convex => {
                    self.classify_cylinder(ni, *point, *axis, *radius);
                }
                Shape::Cylinder { .. } if !nd.full && nd.convex => {} // pending: a reflex round of a profile, or a fillet (step 6)
                Shape::Cylinder { .. } | Shape::Cone { .. } if !nd.full && !nd.convex => {} // pending: pocket corner or floor fillet
                Shape::Cylinder { .. } => {} // full concave: hole analysis below
                Shape::Cone { .. } if nd.full && !nd.convex => {} // pending: drill point or countersink
                Shape::Cone { .. } if nd.full => {
                    self.st.set(ni, RoleKind::Global, "global cone (full revolution, convex)")
                }
                Shape::Cone { .. } => {}  // pending: a chamfer of a circular edge (step 6)
                Shape::Torus { .. } => {} // pending: a floor fillet's corner, or a circular fillet (step 6)
                Shape::Sphere { .. } if nd.full => {
                    self.st.set(ni, RoleKind::Global, "global revolved surface (full revolution)")
                }
                Shape::Sphere { .. } => {} // pending: a corner patch of equal fillets, or a dome (step 6)
            }
        }
    }

    /// The scan in front of the plane through `p0` with normal `n` beyond `tol`, as the counts of
    /// the points no boss explains and of those a boss explains.
    fn plane_front_counts(&self, p0: Vec3, n: Vec3, tol: f64) -> (usize, usize) {
        let (mut in_front, mut by_boss) = (0usize, 0usize);
        for p in self.scan {
            if (p.coords - p0).dot(&n) > tol {
                if self.explained.iter().any(|e| e.covers(p, &n)) {
                    by_boss += 1;
                } else {
                    in_front += 1;
                }
            }
        }
        (in_front, by_boss)
    }

    /// A plane with (almost) no scan in front of it, not counting what a boss standing on it
    /// explains, is a global cut.
    fn classify_plane(&mut self, ni: usize, nd: &Node, point: [f64; 3], normal: [f64; 3]) {
        let (p0, n) = (v3(point), v3(normal));
        let rms = plane_rms(&nd.pts, p0, n);
        let tol = (self.sc.mm(0.1) + 3.0 * rms).min(self.sc.mm(0.4));
        let (in_front, by_boss) = self.plane_front_counts(p0, n, tol);
        let front = in_front as f64 / self.total;
        self.set_front(ni, front);
        if front <= ENVELOPE_FRACTION {
            let boss = if by_boss > 0 {
                format!(
                    "; {:.1} % of the scan in front of it is the boss standing on it",
                    by_boss as f64 / self.total * 100.0
                )
            } else {
                String::new()
            };
            self.st.set(
                ni,
                RoleKind::Global,
                format!("global plane: {:.3} % of the scan lies in front of it{boss}", front * 100.0),
            );
        }
    }

    /// A convex full cylinder with (almost) no scan outside it is a global cut.
    fn classify_cylinder(&mut self, ni: usize, point: [f64; 3], axis: [f64; 3], radius: f64) {
        let out = self.outside_cylinder(point, axis, radius);
        self.set_front(ni, out);
        if out <= ENVELOPE_FRACTION {
            self.st.set(
                ni,
                RoleKind::Global,
                format!("global cylinder: {:.3} % of the scan lies outside it", out * 100.0),
            );
        } // else pending: a post of a pocket (an island), or no tool at all (step 5)
    }

    /// Steps 3 and 4, pockets, steps and ribs (a local floor plane with walls and rounds) and
    /// through slots.
    pub(super) fn plan_local_tools(&mut self) {
        let env = Env {
            nodes: self.nodes,
            cloud: self.cloud,
            diag: self.diag,
            anchors: self.anchors,
            explained: &self.explained,
            front_of: &self.front_of,
            scale: self.sc,
        };
        plan_pockets(&env, &mut self.st, &mut self.tools);
        plan_slots(&env, &mut self.st, &mut self.tools);
    }

    /// Step 5, what is left: the reason a region still pending is skipped (or, for a countersink,
    /// global).
    pub(super) fn settle_leftovers(&mut self) {
        for ni in 0..self.nodes.len() {
            if !self.st.pending(ni) {
                continue;
            }
            let Some(nd) = self.nodes.get(ni) else { continue };
            let note = self.st.notes.get(ni).filter(|n| !n.is_empty()).map_or(String::new(), |n| format!("; {n}"));
            let (kind, how) = self.leftover_role(ni, nd, &note);
            self.st.set(ni, kind, how);
        }
    }

    /// The role and the reason for the pending node `ni`; `note` is the failed attempt to use it
    /// (with its separator), or empty.
    fn leftover_role(&self, ni: usize, nd: &Node, note: &str) -> (RoleKind, String) {
        let front = self.front_of.get(ni).copied().flatten().unwrap_or(0.0) * 100.0;
        match &nd.shape {
            Shape::Cone { .. } if nd.convex => {
                (RoleKind::Skipped, format!("a partial convex cone that is no chamfer of a circular edge{note}"))
            }
            Shape::Cone { .. } => (RoleKind::Global, "concave cone (countersink): subtracted from the stock".into()),
            Shape::Torus { .. } | Shape::Sphere { .. } => {
                (RoleKind::Skipped, format!("a partial torus or sphere that is no fillet, corner patch or dome{note}"))
            }
            Shape::Cylinder { .. } if nd.full && nd.convex => (
                RoleKind::Skipped,
                format!(
                    "boss or step: {:.1} % of the scan lies outside this convex cylinder and it is no boss on a plane or post in a pocket{note}",
                    front
                ),
            ),
            Shape::Plane { .. } => (
                RoleKind::Skipped,
                format!(
                    "local plane ({:.1} % of the scan lies in front of it) that is not the floor or a wall of a recognised pocket, hole or slot{note}",
                    front
                ),
            ),
            _ => (
                RoleKind::Skipped,
                format!("a partial concave surface that is not the wall of a recognised pocket or slot{note}"),
            ),
        }
    }
}

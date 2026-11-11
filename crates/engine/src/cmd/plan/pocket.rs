//! Tools that start at a floor: a pocket (walls round a floor, with the islands it leaves
//! standing), a step (walls with a floor that run out of the part) and a rib or post (islands on a
//! face that is cut down everywhere else). The floor is a local plane: scan lies in front of it.
use super::loops::{Loop, build_loops};
use super::profile::{Frame, Seg, inside_polygon, polygon_of, reverse_loop, wall_normal, walls_and_corners};
use super::scan::{Scale, Scan};
use super::void::{VOID_SHARE, Void, measure};
use super::{Explained, NEIGHBOUR_GAP, Node, PlannedTool, RoleKind, State, arr, near, snap_direction, v3};
use surfacecraft_geom::{Vec3, normalize};
use surfacecraft_kernel::hybrid::Shape;
use surfacecraft_kernel::local::LocalTool;

/// A scan vertex this high above the floor (mm at the reference spacing) is in the void of a tool
/// that starts at it.
const VOID_HEIGHT: f64 = 0.6;
/// A wall touches its floor when its lowest point is within these heights above it (mm at the
/// reference spacing).
const WALL_HEIGHTS: (f64, f64) = (-0.6, 3.0);
/// Largest radius of a fillet between a floor and its walls (mm at the reference spacing); also the
/// width of the band along the floor and the walls in which unmodelled fillets are not counted.
const FILLET_MAX_RADIUS: f64 = 6.0;
/// How far a fillet's axis may be from one radius above the floor (mm at the reference spacing).
const FILLET_AXIS_TOL: f64 = 0.4;
/// A floor's normal snaps to a model axis within this angle (degrees).
const FLOOR_SNAP_DEG: f64 = 0.3;
/// Two points are the same scan point when they are this close (mm).
const SAME_POINT: f64 = 0.01;
/// Cosine bounds: an axis along the floor's normal, and an axis square to it.
const ALONG: f64 = 0.99;
const ACROSS: f64 = 0.1;

/// What the pocket step knows about the part.
pub(super) struct Env<'a> {
    pub(super) nodes: &'a [Node],
    pub(super) cloud: &'a Scan,
    pub(super) diag: f64,
    pub(super) anchors: &'a [Vec3],
    pub(super) explained: &'a [Explained],
    pub(super) front_of: &'a [Option<f64>],
    pub(super) scale: Scale,
}

/// A floor: a pending local plane, as the base of a tool.
struct Floor {
    node: usize,
    p0: Vec3,
    nf: Vec3,
    frame: Frame,
}

struct Candidates {
    walls: Vec<usize>,
    fillets: Vec<usize>,
}

impl Env<'_> {
    fn node(&self, i: usize) -> Option<&Node> {
        self.nodes.get(i)
    }
    fn ext(&self) -> f64 {
        2.0 * self.diag + 10.0
    }
    /// Two regions this close (mm) are neighbours.
    fn gap(&self) -> f64 {
        self.scale.mm(NEIGHBOUR_GAP)
    }
}

pub(super) fn plan_pockets(env: &Env, st: &mut State, tools: &mut Vec<PlannedTool>) {
    for fi in 0..env.nodes.len() {
        if st.pending(fi)
            && let Some(floor) = floor_of(env, fi)
        {
            plan_floor(env, st, tools, &floor);
        }
    }
}

fn floor_of(env: &Env, fi: usize) -> Option<Floor> {
    let Shape::Plane { point, normal } = &env.node(fi)?.shape else { return None };
    let nf = snap_direction(v3(*normal), env.anchors, FLOOR_SNAP_DEG);
    let p0 = v3(*point);
    Some(Floor { node: fi, p0, nf, frame: Frame::new(p0, nf) })
}

fn plan_floor(env: &Env, st: &mut State, tools: &mut Vec<PlannedTool>, floor: &Floor) {
    let cand = candidates(env, st, floor);
    let loops = floor_loops(env, st, floor, &cand.walls);
    let made = assemble(env, st, tools, floor, &loops, &cand);
    if made == 0 {
        st.note(floor.node, "no pocket, step or rib walls close around it");
    }
}

// ---------------------------------------------------------------------------------------------
// candidates

/// Can the region be a wall (a plane square to the floor), a round of a profile (a cylinder along
/// the floor's normal: the rounded corner of a pocket, a reflex round, a post) or a fillet of the
/// floor?
fn can_touch(c: &Node, nf: &Vec3) -> bool {
    let axis_dot = |axis: &[f64; 3]| normalize(v3(*axis)).map(|a| a.dot(nf).abs());
    match &c.shape {
        Shape::Plane { normal, .. } => normalize(v3(*normal)).is_some_and(|n| n.dot(nf).abs() < 0.05),
        Shape::Cylinder { axis, .. } if c.convex => axis_dot(axis).is_some_and(|d| d > ALONG),
        Shape::Cylinder { axis, .. } if !c.full => axis_dot(axis).is_some_and(|d| d > ALONG || d < ACROSS),
        Shape::Torus { .. } => !c.full && !c.convex,
        _ => false,
    }
}

/// A floor fillet: a cylinder along the walls or a torus round the corners.
fn is_fillet(c: &Node, nf: &Vec3) -> bool {
    match &c.shape {
        Shape::Cylinder { axis, .. } => normalize(v3(*axis)).is_some_and(|a| a.dot(nf).abs() < ACROSS),
        Shape::Torus { axis, .. } => normalize(v3(*axis)).is_some_and(|a| a.dot(nf).abs() > ALONG),
        _ => false,
    }
}

fn candidates(env: &Env, st: &State, floor: &Floor) -> Candidates {
    let Some(fl) = env.node(floor.node) else { return Candidates { walls: vec![], fillets: vec![] } };
    let low = |nd: &Node| nd.pts.iter().map(|p| (p.coords - floor.p0).dot(&floor.nf)).fold(f64::INFINITY, f64::min);
    let found: Vec<usize> = (0..env.nodes.len())
        .filter(|&ci| ci != floor.node && st.pending(ci))
        .filter(|&ci| {
            env.node(ci).is_some_and(|c| {
                can_touch(c, &floor.nf)
                    && (env.scale.mm(WALL_HEIGHTS.0)..=env.scale.mm(WALL_HEIGHTS.1)).contains(&low(c))
                    && near(fl, c, env.gap()).is_some()
            })
        })
        .collect();
    let (fillets, walls) = found.into_iter().partition(|&c| env.node(c).is_some_and(|c| is_fillet(c, &floor.nf)));
    Candidates { walls, fillets }
}

/// Groups of nodes that touch each other.
fn proximity_components(env: &Env, ids: &[usize]) -> Vec<Vec<usize>> {
    let mut label: Vec<usize> = (0..ids.len()).collect();
    for x in 0..ids.len() {
        for y in (x + 1)..ids.len() {
            let touching = match (ids.get(x).and_then(|&i| env.node(i)), ids.get(y).and_then(|&i| env.node(i))) {
                (Some(a), Some(b)) => near(a, b, env.gap()).is_some(),
                _ => false,
            };
            let (lx, ly) = (label.get(x).copied().unwrap_or(x), label.get(y).copied().unwrap_or(y));
            if touching && lx != ly {
                label.iter_mut().filter(|l| **l == ly).for_each(|l| *l = lx);
            }
        }
    }
    let mut firsts: Vec<usize> = Vec::new();
    for &l in &label {
        if !firsts.contains(&l) {
            firsts.push(l);
        }
    }
    firsts.into_iter().map(|f| ids.iter().zip(&label).filter(|(_, l)| **l == f).map(|(&i, _)| i).collect()).collect()
}

/// A group of walls and rounds forms a profile when it has a wall or a post.
fn forms_profile(env: &Env, members: &[usize]) -> bool {
    members.iter().filter_map(|&m| env.node(m)).any(|n| matches!(n.shape, Shape::Plane { .. }) || n.full)
}

fn floor_loops(env: &Env, st: &mut State, floor: &Floor, walls: &[usize]) -> Vec<Loop> {
    let mut out = Vec::new();
    for members in proximity_components(env, walls) {
        if !forms_profile(env, &members) {
            continue;
        }
        let built = walls_and_corners(env.nodes, &members, &floor.frame, env.anchors)
            .and_then(|(w, c)| build_loops(&w, &c, env.ext(), env.scale));
        match built {
            Ok(loops) => out.extend(loops),
            Err(why) => members.iter().for_each(|&u| st.note(u, format!("pocket profile: {why}"))),
        }
    }
    out
}

// ---------------------------------------------------------------------------------------------
// tools from loops

/// The smallest outer loop that holds the island.
fn holder(outers: &[&Loop], island: &Loop) -> Option<usize> {
    outers
        .iter()
        .enumerate()
        .filter(|(_, o)| inside_polygon(&o.polygon, island.probe))
        .min_by(|a, b| a.1.area.total_cmp(&b.1.area))
        .map(|(i, _)| i)
}

/// A square of half-width `half` about the floor's origin: the outline of a face cut down
/// everywhere.
fn far_box(half: f64) -> Vec<Seg> {
    let c = [[-half, -half], [half, -half], [half, half], [-half, half]];
    (0..4).filter_map(|i| Some(Seg::Line(*c.get(i)?, *c.get((i + 1) % 4)?))).collect()
}

/// One tool to build: its outline, the walls that form it and the islands it leaves standing.
struct Spec<'a> {
    kind: &'static str,
    outline: Vec<Seg>,
    /// Nodes of the outline's walls and rounds (none for the box round a rib).
    nodes: Vec<usize>,
    arcs: Vec<(usize, [f64; 2], f64)>,
    /// Closed by its own walls (a pocket); else it runs out of the part.
    closed: bool,
    islands: Vec<&'a Loop>,
    /// Why the tool must not be built (a part of the pocket that was not understood).
    blocked: Option<&'static str>,
}

impl Spec<'_> {
    fn all_nodes(&self) -> Vec<usize> {
        self.nodes.iter().chain(self.islands.iter().flat_map(|l| &l.nodes)).copied().collect()
    }
    fn all_arcs(&self) -> Vec<(usize, [f64; 2], f64)> {
        self.arcs.iter().chain(self.islands.iter().flat_map(|l| &l.arcs)).copied().collect()
    }
}

fn assemble(
    env: &Env,
    st: &mut State,
    tools: &mut Vec<PlannedTool>,
    floor: &Floor,
    loops: &[Loop],
    cand: &Candidates,
) -> usize {
    let (outers, islands): (Vec<&Loop>, Vec<&Loop>) = loops.iter().partition(|l| l.area > 0.0);
    let mut owned: Vec<Vec<&Loop>> = outers.iter().map(|_| Vec::new()).collect();
    let mut blocked: Vec<Option<&'static str>> = outers.iter().map(|_| None).collect();
    let mut loose: Vec<&Loop> = Vec::new();
    for isl in islands {
        let Some(i) = holder(&outers, isl) else {
            loose.push(isl);
            continue;
        };
        if !isl.closed && outers.get(i).is_some_and(|o| o.closed) {
            if let Some(b) = blocked.get_mut(i) {
                *b = Some("walls that run out of the pocket's own walls are no island");
            }
        } else if let Some(list) = owned.get_mut(i) {
            list.push(isl);
        }
    }
    let mut specs: Vec<Spec> = outers
        .iter()
        .zip(owned)
        .zip(blocked)
        .map(|((o, islands), blocked)| Spec {
            kind: if o.closed { "pocket" } else { "step" },
            outline: o.segs.clone(),
            nodes: o.nodes.clone(),
            arcs: o.arcs.clone(),
            closed: o.closed,
            islands,
            blocked,
        })
        .collect();
    if !loose.is_empty() {
        specs.push(Spec {
            kind: "rib",
            outline: far_box(env.ext()),
            nodes: vec![],
            arcs: vec![],
            closed: false,
            islands: loose,
            blocked: None,
        });
    }
    specs.iter().filter(|spec| build_tool(env, st, tools, floor, cand, spec)).count()
}

/// The nodes whose points are the tool's own surfaces.
fn own_nodes(floor: &Floor, cand: &Candidates) -> Vec<usize> {
    std::iter::once(floor.node).chain(cand.walls.iter().copied()).chain(cand.fillets.iter().copied()).collect()
}

/// Scan surface above the floor inside the tool's outline (and outside its islands) would be cut
/// away with the tool. The tool's own walls, floor and fillets and a boss standing on the floor are
/// not that. Measured as area against the footprint of the tool on the part.
fn void_check(env: &Env, floor: &Floor, spec: &Spec, own: &[usize]) -> Result<(), String> {
    let outline = polygon_of(&spec.outline);
    let polys: Vec<&[[f64; 2]]> = spec.islands.iter().map(|l| l.polygon.as_slice()).collect();
    let void = Void {
        scan: env.cloud,
        frame: &floor.frame,
        scale: env.scale,
        above: Some(env.scale.mm(VOID_HEIGHT)),
        outline: &outline,
        islands: &polys,
        band: env.scale.mm(FILLET_MAX_RADIUS),
    };
    let skipped = |i: usize| {
        let Some(p) = env.cloud.points.get(i) else { return true };
        let owned = own
            .iter()
            .filter_map(|&n| env.node(n))
            .any(|n| n.tree.nearest_within(p, SAME_POINT * SAME_POINT).is_some());
        owned || env.explained.iter().any(|e| e.covers(p, &floor.nf))
    };
    let m = measure(&void, skipped);
    if m.share() > VOID_SHARE {
        return Err(format!(
            "{:.1} mm2 of scan surface ({:.1} % of its {:.0} mm2 footprint) lies inside it above the floor",
            m.area,
            100.0 * m.share(),
            m.footprint
        ));
    }
    Ok(())
}

/// The fillet between the floor and the walls: its radius (the median of the cylinders tangent to
/// the floor) and the regions that form it.
fn floor_fillet(env: &Env, floor: &Floor, used: &[usize], fillets: &[usize]) -> (Option<f64>, Vec<usize>) {
    let mut radii: Vec<f64> = Vec::new();
    let mut members: Vec<usize> = Vec::new();
    for &f in fillets {
        let Some(c) = env.node(f) else { continue };
        if !used.iter().any(|&u| env.node(u).is_some_and(|w| near(w, c, env.gap()).is_some())) {
            continue;
        }
        match &c.shape {
            Shape::Cylinder { point, radius, .. } => {
                let h = (v3(*point) - floor.p0).dot(&floor.nf);
                if (h - radius).abs() < env.scale.mm(FILLET_AXIS_TOL) && *radius < env.scale.mm(FILLET_MAX_RADIUS) {
                    radii.push(*radius);
                    members.push(f);
                }
            }
            Shape::Torus { minor_radius, .. } if *minor_radius < env.scale.mm(FILLET_MAX_RADIUS) => members.push(f),
            _ => {}
        }
    }
    radii.sort_by(f64::total_cmp);
    (radii.get(radii.len() / 2).copied(), members)
}

fn build_tool(
    env: &Env,
    st: &mut State,
    tools: &mut Vec<PlannedTool>,
    floor: &Floor,
    cand: &Candidates,
    spec: &Spec,
) -> bool {
    let nodes = spec.all_nodes();
    let checked = match spec.blocked {
        Some(why) => Err(why.to_string()),
        None => void_check(env, floor, spec, &own_nodes(floor, cand)),
    };
    if let Err(why) = checked {
        let note = format!("a {} from its walls would cut into the part: {why}", spec.kind);
        nodes.iter().chain(std::iter::once(&floor.node)).for_each(|&u| st.note(u, note.clone()));
        return false;
    }
    let (fillet, fmembers) =
        if spec.closed { floor_fillet(env, floor, &nodes, &cand.fillets) } else { (None, Vec::new()) };
    let tool = LocalTool::Prism {
        profile: floor.frame.curves(&spec.outline),
        direction: arr(floor.nf),
        through: false,
        floor_fillet: fillet,
        islands: spec.islands.iter().map(|l| floor.frame.curves(&reverse_loop(&l.segs))).collect(),
    };
    claim(env, st, tools, floor, spec, Made { tool, fmembers, fillet });
    true
}

/// A tool built from a spec, with the fillet regions it took.
struct Made {
    tool: LocalTool,
    fmembers: Vec<usize>,
    fillet: Option<f64>,
}

fn claim(env: &Env, st: &mut State, tools: &mut Vec<PlannedTool>, floor: &Floor, spec: &Spec, made: Made) {
    let Made { tool, fmembers, fillet } = made;
    let fmembers = fmembers.as_slice();
    let ti = tools.len();
    let nodes = spec.all_nodes();
    let base = |u: usize| env.node(u).map(|n| n.base);
    let members: Vec<usize> =
        base(floor.node).into_iter().chain(nodes.iter().chain(fmembers).filter_map(|&u| base(u))).collect();
    let planes = nodes.iter().filter_map(|&u| env.node(u)).filter(|n| matches!(n.shape, Shape::Plane { .. })).count();
    let detail = format!(
        "{planes} walls, {} rounds{}{}",
        nodes.len() - planes,
        fillet.map_or(String::new(), |r| format!(", floor fillet r = {r:.2}")),
        if spec.islands.is_empty() { String::new() } else { format!(", {} islands left standing", spec.islands.len()) }
    );
    tools.push(PlannedTool { tool, kind: spec.kind, members, detail });
    let wall_text = match spec.kind {
        "pocket" => "wall or corner of a pocket (concave contact with its floor)",
        "step" => "wall of a step (the riser of its floor)",
        _ => "wall of a rib or post standing on its floor",
    };
    for &u in nodes.iter().chain(fmembers) {
        st.set(u, RoleKind::LocalSubtract, wall_text);
        if let Some(t) = st.tools.get_mut(u) {
            t.push(ti);
        }
        plan_wall_shape(env, st, &floor.nf, u);
    }
    for (u, c, r) in spec.all_arcs() {
        if let Some(sh) = st.shape.get_mut(u) {
            *sh = Shape::Cylinder { point: arr(floor.frame.to3(c)), axis: arr(floor.nf), radius: r };
        }
    }
    let front = env.front_of.get(floor.node).copied().flatten().unwrap_or(0.0) * 100.0;
    st.set(
        floor.node,
        RoleKind::LocalSubtract,
        format!("floor of a {} ({planes} walls), {front:.1} % of the scan lies in front of it", spec.kind),
    );
    if let Some(t) = st.tools.get_mut(floor.node) {
        t.push(ti);
    }
}

/// A wall is exactly square to the floor in the planned shape.
pub(super) fn plan_wall_shape(env: &Env, st: &mut State, nf: &Vec3, u: usize) {
    let Some(Shape::Plane { point, normal }) = env.node(u).map(|n| &n.shape) else { return };
    if let (Some(m), Some(sh)) = (wall_normal(v3(*normal), *nf, env.anchors), st.shape.get_mut(u)) {
        *sh = Shape::Plane { point: *point, normal: arr(m) };
    }
}

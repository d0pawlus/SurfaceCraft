//! Through slots and windows: walls (and rounds) with no floor, all along one direction. The part
//! has no scan inside such a tool, so it runs across the whole stock.
use super::loops::{Loop, build_loops};
use super::pocket::{Env, plan_wall_shape};
use super::profile::{Frame, polygon_of, walls_and_corners};
use super::void::{VOID_SHARE, Void, measure};
use super::{NEIGHBOUR_GAP, Node, PlannedTool, RoleKind, State, arr, near, snap_direction, v3};
use surfacecraft_geom::{Vec3, normalize};
use surfacecraft_kernel::hybrid::Shape;
use surfacecraft_kernel::local::LocalTool;

/// Slot directions tried (each takes one group of walls).
const MAX_ROUNDS: usize = 8;
/// A slot's direction snaps to a model axis within this angle (degrees).
const DIRECTION_SNAP_DEG: f64 = 0.5;
/// Points sampled per wall for the frame's origin.
const SAMPLES_PER_WALL: usize = 200;
/// Cosine bounds: a wall square to the direction, an axis along it.
const SQUARE: f64 = 0.05;
const ALONG: f64 = 0.99;
/// The cross product of two planes' normals must be at least this long to give a direction.
const MIN_CROSS: f64 = 0.3;

/// A round that is not a wall: the convex partial cylinder of a reflex corner.
fn is_reflex_round(n: &Node) -> bool {
    matches!(n.shape, Shape::Cylinder { .. }) && n.convex && !n.full
}

/// The direction of the first slot: a pending concave partial cylinder's axis, else the cross of
/// two pending planes.
fn direction(env: &Env, st: &State) -> Option<Vec3> {
    let pend: Vec<usize> = (0..env.nodes.len()).filter(|&i| st.pending(i)).collect();
    for &i in &pend {
        if let Some(Node { shape: Shape::Cylinder { axis, .. }, full: false, convex: false, .. }) = env.nodes.get(i) {
            return normalize(v3(*axis));
        }
    }
    for &i in &pend {
        for &j in &pend {
            if let (
                Some(Node { shape: Shape::Plane { normal: n1, .. }, .. }),
                Some(Node { shape: Shape::Plane { normal: n2, .. }, .. }),
            ) = (env.nodes.get(i), env.nodes.get(j))
            {
                let c = v3(*n1).cross(&v3(*n2));
                if c.norm() > MIN_CROSS {
                    return normalize(c);
                }
            }
        }
    }
    None
}

/// Pending planes square to `d` and partial cylinders along it.
fn members_along(env: &Env, st: &State, d: Vec3) -> Vec<usize> {
    (0..env.nodes.len())
        .filter(|&i| st.pending(i))
        .filter(|&i| {
            env.nodes.get(i).is_some_and(|n| match &n.shape {
                Shape::Plane { normal, .. } => normalize(v3(*normal)).is_some_and(|m| m.dot(&d).abs() < SQUARE),
                Shape::Cylinder { axis, .. } => {
                    !n.full && normalize(v3(*axis)).is_some_and(|a| a.dot(&d).abs() > ALONG)
                }
                _ => false,
            })
        })
        .collect()
}

/// The group of members that touch the first wall (or concave round), by proximity.
fn first_component(env: &Env, members: &[usize]) -> Vec<usize> {
    let Some(&start) = members.iter().find(|&&i| env.nodes.get(i).is_some_and(|n| !is_reflex_round(n))) else {
        return Vec::new();
    };
    let mut comp = vec![start];
    let mut grew = true;
    while grew {
        grew = false;
        for &m in members {
            let touches = !comp.contains(&m)
                && comp.iter().any(|&c| match (env.nodes.get(c), env.nodes.get(m)) {
                    (Some(a), Some(b)) => near(a, b, env.scale.mm(NEIGHBOUR_GAP)).is_some(),
                    _ => false,
                });
            if touches {
                comp.push(m);
                grew = true;
            }
        }
    }
    comp
}

fn frame_of(env: &Env, comp: &[usize], d: Vec3) -> Frame {
    let (mut sum, mut cnt) = (Vec3::zeros(), 0.0_f64);
    for n in comp.iter().filter_map(|&c| env.nodes.get(c)) {
        for p in n.pts.iter().step_by((n.pts.len() / SAMPLES_PER_WALL).max(1)) {
            sum += p.coords;
            cnt += 1.0;
        }
    }
    Frame::new(sum / cnt.max(1.0), d)
}

/// The scan surface inside the loop's outline (at any height) and the footprint of the loop on the
/// part: a through tool would cut it away.
fn scan_inside(env: &Env, frame: &Frame, l: &Loop) -> super::void::Measure {
    let outline = polygon_of(&l.segs);
    let void =
        Void { scan: env.cloud, frame, scale: env.scale, above: None, outline: &outline, islands: &[], band: 0.0 };
    measure(&void, |_| false)
}

/// Why a loop cannot be a through tool, if it cannot.
fn check_loop(env: &Env, frame: &Frame, l: &Loop) -> Result<(), String> {
    if l.area < 0.0 {
        return Err("walls round an island with no floor under it".into());
    }
    let m = scan_inside(env, frame, l);
    if m.share() > VOID_SHARE {
        return Err(format!(
            "slot walls with scan inside them ({:.1} mm2, {:.1} % of the footprint): a floor that was not given, or an island",
            m.area,
            100.0 * m.share()
        ));
    }
    Ok(())
}

/// Makes the through tool of one loop of the group and gives its regions their roles and shapes.
fn make_tool(env: &Env, st: &mut State, tools: &mut Vec<PlannedTool>, frame: &Frame, l: &Loop) {
    let planes =
        l.nodes.iter().filter_map(|&n| env.nodes.get(n)).filter(|n| matches!(n.shape, Shape::Plane { .. })).count();
    let rounds = l.nodes.len() - planes;
    let ti = tools.len();
    tools.push(PlannedTool {
        tool: LocalTool::Prism {
            profile: frame.curves(&l.segs),
            direction: arr(frame.d),
            through: true,
            floor_fillet: None,
            islands: Vec::new(),
        },
        kind: "through_slot",
        members: l.nodes.iter().filter_map(|&c| env.nodes.get(c).map(|n| n.base)).collect(),
        detail: format!(
            "{planes} walls, {rounds} rounded corners, {}",
            if l.closed { "closed window" } else { "open to the outside" }
        ),
    });
    let what = if l.closed { "window" } else { "slot" };
    for &c in &l.nodes {
        st.set(c, RoleKind::LocalSubtract, format!("wall of a through {what} ({planes} walls)"));
        if let Some(t) = st.tools.get_mut(c) {
            t.push(ti);
        }
        plan_wall_shape(env, st, &frame.d, c);
    }
    for &(u, c, r) in &l.arcs {
        if let Some(sh) = st.shape.get_mut(u) {
            *sh = Shape::Cylinder { point: arr(frame.to3(c)), axis: arr(frame.d), radius: r };
        }
    }
}

fn plan_group(env: &Env, st: &mut State, tools: &mut Vec<PlannedTool>, comp: &[usize], d: Vec3) -> Result<(), String> {
    let frame = frame_of(env, comp, d);
    let (walls, corners) = walls_and_corners(env.nodes, comp, &frame, env.anchors)?;
    let loops = build_loops(&walls, &corners, 2.0 * env.diag + 10.0, env.scale)?;
    loops.iter().try_for_each(|l| check_loop(env, &frame, l))?;
    loops.iter().for_each(|l| make_tool(env, st, tools, &frame, l));
    Ok(())
}

pub(super) fn plan_slots(env: &Env, st: &mut State, tools: &mut Vec<PlannedTool>) {
    for _ in 0..MAX_ROUNDS {
        let Some(d0) = direction(env, st) else { break };
        let d = snap_direction(d0, env.anchors, DIRECTION_SNAP_DEG);
        let members = members_along(env, st, d);
        let walls: Vec<usize> =
            members.iter().copied().filter(|&i| env.nodes.get(i).is_some_and(|n| !is_reflex_round(n))).collect();
        if walls.len() < 2 {
            walls.iter().for_each(|&i| st.set(i, RoleKind::Skipped, "a lone local wall: no slot or pocket around it"));
            if walls.is_empty() {
                break;
            }
            continue;
        }
        let comp = first_component(env, &members);
        if let Err(why) = plan_group(env, st, tools, &comp, d) {
            // a convex round of the group keeps its chance of being a fillet (step 6)
            let skipped = comp.iter().filter(|&&c| env.nodes.get(c).is_some_and(|n| !is_reflex_round(n)));
            skipped.for_each(|&c| st.set(c, RoleKind::Skipped, format!("slot profile: {why}")));
        }
    }
}

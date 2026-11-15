//! The `body.refit_faces` command: associate scan points to faces, move the faces and rebuild the solid.

use super::faces::*;
use super::*;

/// Shapes of the faces of a body: the analytic shape, or why the face is refused.
type Shapes = [Result<FaceShape, String>];

/// Validated numeric options of `body.refit_faces`.
struct FaceOpts {
    mc: f64,
    max_distance: f64,
    max_angle: f64,
    min_move: f64,
    min_points: usize,
    keep_relations: bool,
    default_stiffness: Stiffness,
}

/// Reads and checks the numeric options of the command, in the order of their error messages.
fn face_opts(i: &RefitFacesIn) -> CmdResult<FaceOpts> {
    let mc = positive(i.max_change, 1.0, "max_change")?;
    let max_distance = positive(i.max_distance, mc, "max_distance")?;
    let max_angle = positive(i.max_angle, 60.0, "max_angle")?;
    if max_angle >= 180.0 {
        return Err(CmdError::Params("max_angle must be below 180 degrees".into()));
    }
    let min_move = positive(i.min_move, 0.002, "min_move")?;
    Ok(FaceOpts {
        mc,
        max_distance,
        max_angle,
        min_move,
        min_points: i.min_points.unwrap_or(8).max(1),
        keep_relations: i.keep_relations.unwrap_or(true),
        default_stiffness: i.stiffness.unwrap_or(Stiffness::Loose),
    })
}

/// Refuses a body that is made by a feature history.
fn ensure_no_history(s: &Session, body: ObjectId) -> CmdResult<()> {
    if s.doc.features.iter().any(|f| f.body == body) {
        return Err(CmdError::Failed(
            "this body is made by a feature history (a replay would undo the edit): use body.refit_features".into(),
        ));
    }
    Ok(())
}

/// The geometry of `body`, refused unless it is a single closed shell.
fn single_shell_brep(s: &Session, body: ObjectId) -> CmdResult<Arc<Brep>> {
    let brep = s.doc.body(body).map(|b| b.brep.clone()).ok_or_else(|| CmdError::Params(format!("no body {body}")))?;
    let closed_shells =
        brep.shells.iter().filter(|sh| matches!(sh, surfacecraft_kernel::record::brep::Shell::Faces { .. })).count();
    if closed_shells != 1 {
        return Err(CmdError::Failed("only a body of one shell is refitted (not several solids or cavities)".into()));
    }
    Ok(brep)
}

/// The stiffness of every face: the default, overridden by the faces listed in the parameters.
fn face_stiffness(
    listed: &[FaceStiffness],
    shapes: &Shapes,
    default_stiffness: Stiffness,
) -> CmdResult<Vec<Stiffness>> {
    let nf = shapes.len();
    let mut stiff = vec![default_stiffness; nf];
    let mut explicit = vec![false; nf];
    for fs in listed {
        let Some(slot) = stiff.get_mut(fs.face) else {
            return Err(CmdError::Params(format!("face {} does not exist ({nf} faces)", fs.face)));
        };
        if explicit.get(fs.face).copied().unwrap_or(false) {
            return Err(CmdError::Params(format!("face {} is listed twice", fs.face)));
        }
        if let Some(Err(reason)) = shapes.get(fs.face)
            && fs.stiffness != Stiffness::Fixed
        {
            return Err(CmdError::Params(format!("face {} cannot be refitted: {reason}", fs.face)));
        }
        *slot = fs.stiffness;
        if let Some(e) = explicit.get_mut(fs.face) {
            *e = true;
        }
    }
    Ok(stiff)
}

/// One BVH per face (`None` for a face without triangles) over the meshed body.
fn face_bvhs(brep: &Brep, nf: usize) -> CmdResult<Vec<Option<Bvh>>> {
    let chord = (bbox_diagonal(brep) * 2e-4).clamp(0.005, 0.05);
    let topo = surfacecraft_kernel::tess::tessellate_topo(brep, Some(chord))?;
    if topo.tri_face.is_empty() || topo.mesh.triangle_count() == 0 {
        return Err(CmdError::Failed("the body could not be meshed to associate the scan to its faces".into()));
    }
    let mut tris: Vec<Vec<u32>> = vec![Vec::new(); nf];
    for (t, f) in topo.tri_face.iter().enumerate() {
        if let Some(list) = tris.get_mut(*f as usize) {
            list.push(t as u32);
        }
    }
    Ok(tris.iter().map(|t| (!t.is_empty()).then(|| Bvh::build(&topo.mesh.submesh(t)))).collect())
}

/// The nearest face (within `max_distance`) whose normal faces the scan point `p` with normal `n`.
fn nearest_face(
    p: &Point3,
    n: &Vec3,
    bvhs: &[Option<Bvh>],
    shapes: &Shapes,
    max_distance: f64,
    cos_min: f64,
) -> Option<(usize, f64)> {
    let has_normal = n.norm() > 0.5;
    let mut best: Option<(usize, f64)> = None;
    let mut reach = max_distance;
    for (f, bvh) in bvhs.iter().enumerate() {
        let Some(bvh) = bvh else { continue };
        let Some(hit) = bvh.closest(p, reach) else { continue };
        let facing = match shapes.get(f) {
            Some(Ok(shape)) => {
                let fnm = shape.outward_normal([hit.point.x, hit.point.y, hit.point.z]);
                !has_normal || n.dot(&Vec3::from(fnm)) >= cos_min
            }
            // a refused face: only its triangle's direction, whichever way the mesh winds
            _ => !has_normal || bvh.triangle_normal(hit.triangle).is_none_or(|t| n.dot(&t).abs() >= cos_min),
        };
        if facing {
            reach = hit.distance;
            best = Some((f, hit.distance));
        }
    }
    best
}

/// Association: the face (and distance) every scan point belongs to, if any.
fn associate(
    points: &[Point3],
    normals: &[Vec3],
    bvhs: &[Option<Bvh>],
    shapes: &Shapes,
    o: &FaceOpts,
) -> Vec<Option<(usize, f64)>> {
    let cos_min = o.max_angle.to_radians().cos();
    let max_distance = o.max_distance;
    par::map(points.len(), |k| {
        let (Some(p), Some(n)) = (points.get(k), normals.get(k)) else { return None };
        nearest_face(p, n, bvhs, shapes, max_distance, cos_min)
    })
}

/// Signed offsets of the scan points from the analytic face they belong to, and the number of
/// points that belong to no usable face.
fn point_offsets(points: &[Point3], assigned: &[Option<(usize, f64)>], shapes: &Shapes) -> (Vec<Vec<f64>>, usize) {
    let mut offsets: Vec<Vec<f64>> = vec![Vec::new(); shapes.len()];
    let mut unassigned = 0;
    for (p, a) in points.iter().zip(assigned) {
        match a {
            Some((f, _)) => match shapes.get(*f) {
                Some(Ok(shape)) => {
                    let d = shape.signed_distance([p.x, p.y, p.z]);
                    if d.is_finite()
                        && let Some(list) = offsets.get_mut(*f)
                    {
                        list.push(d);
                    }
                }
                _ => unassigned += 1,
            },
            None => unassigned += 1,
        }
    }
    (offsets, unassigned)
}

/// Unites the faces that must move together (same surface, or equal radius when relations are kept).
fn tie_faces(shapes: &Shapes, keep_relations: bool) -> UnionFind {
    let nf = shapes.len();
    let mut uf = UnionFind::new(nf);
    for a in 0..nf {
        let Some(Ok(sa)) = shapes.get(a) else { continue };
        for b in (a + 1)..nf {
            let Some(Ok(sb)) = shapes.get(b) else { continue };
            let same = sa.same_surface(sb, SAME_SURFACE_TOL);
            let equal = keep_relations
                && matches!((sa.radius(), sb.radius()), (Some(ra), Some(rb)) if (ra - rb).abs() < 1e-5 * (1.0 + ra.abs()));
            if same || equal {
                uf.union(a, b);
            }
        }
    }
    uf
}

/// Groups of faces that must move together, in the order of their first face.
fn face_groups(shapes: &Shapes, keep_relations: bool) -> Vec<Group> {
    let mut uf = tie_faces(shapes, keep_relations);
    let mut groups: Vec<Group> = Vec::new();
    let mut slot_of: BTreeMap<usize, usize> = BTreeMap::new();
    for f in 0..shapes.len() {
        if !matches!(shapes.get(f), Some(Ok(_))) {
            continue;
        }
        let root = uf.find(f);
        let g = *slot_of.entry(root).or_insert_with(|| {
            groups.push(Group { faces: Vec::new() });
            groups.len() - 1
        });
        if let Some(group) = groups.get_mut(g) {
            group.faces.push(f);
        }
    }
    groups
}

/// Inputs of the per-group move solve.
struct MoveCtx<'a> {
    shapes: &'a Shapes,
    offsets: &'a [Vec<f64>],
    stiff: &'a [Stiffness],
    default_stiffness: Stiffness,
    max_change: f64,
    min_points: usize,
}

/// The accumulating outputs of the per-group solve: the move of every face and why it stays.
struct GroupMoves {
    delta: Vec<f64>,
    reason: Vec<Option<String>>,
}

/// What the solve of one group found.
struct GroupSolve {
    theta: f64,
    total: usize,
    fixed_member: Option<usize>,
}

impl MoveCtx<'_> {
    /// θ per group, then the move of every face: δ = sign·θ.
    fn solve(&self, groups: &[Group]) -> GroupMoves {
        let nf = self.shapes.len();
        let mut out = GroupMoves { delta: vec![0.0; nf], reason: vec![None; nf] };
        for g in groups {
            let gs = self.solve_one(g);
            for &f in &g.faces {
                self.place_face(f, g.faces.len(), &gs, &mut out);
            }
        }
        out
    }

    /// The sign of a face's radius direction (1 for a face without a shape).
    fn radius_sign(&self, f: usize) -> f64 {
        self.shapes.get(f).and_then(|s| s.as_ref().ok()).map_or(1.0, FaceShape::radius_sign)
    }

    /// The shared move of one group, its point count and its fixed member, if any.
    fn solve_one(&self, g: &Group) -> GroupSolve {
        let members: Vec<(f64, &[f64], f64)> = g
            .faces
            .iter()
            .map(|&f| {
                let sign = self.radius_sign(f);
                let d: &[f64] = self.offsets.get(f).map_or(&[], Vec::as_slice);
                (sign, d, self.stiff.get(f).copied().unwrap_or(self.default_stiffness).prior())
            })
            .collect();
        let total: usize = members.iter().map(|m| m.1.len()).sum();
        let fixed_member = g.faces.iter().copied().find(|&f| self.stiff.get(f) == Some(&Stiffness::Fixed));
        let theta = solve_group(&members, self.max_change);
        GroupSolve { theta, total, fixed_member }
    }

    /// Why face `f` of a group of `group_len` faces does not move, if it does not.
    fn skip_reason(&self, f: usize, group_len: usize, gs: &GroupSolve) -> Option<String> {
        let (total, min_points) = (gs.total, self.min_points);
        if let Some(fx) = gs.fixed_member
            && self.stiff.get(f) != Some(&Stiffness::Fixed)
        {
            return Some(format!("tied to the fixed face {fx} (same surface or equal radius)"));
        }
        if total < min_points {
            return Some(format!("too few scan points ({total}, at least {min_points})"));
        }
        let own = self.offsets.get(f).map_or(0, Vec::len);
        if own < min_points && group_len == 1 {
            return Some(format!("too few scan points ({own}, at least {min_points})"));
        }
        None
    }

    /// Sets the move of face `f`, or the reason it stays.
    fn place_face(&self, f: usize, group_len: usize, gs: &GroupSolve, out: &mut GroupMoves) {
        let sign = self.radius_sign(f);
        if let Some(why) = self.skip_reason(f, group_len, gs) {
            if let Some(r) = out.reason.get_mut(f) {
                *r = Some(why);
            }
        } else if let Some(d) = out.delta.get_mut(f) {
            *d = sign * gs.theta;
        }
    }
}

/// The faces on the same surface as face `f` (itself first) that are not taken yet.
fn surface_twins(shapes: &Shapes, taken: &[bool], f: usize, sf: &FaceShape) -> Vec<usize> {
    let mut faces = vec![f];
    for h in (f + 1)..shapes.len() {
        if let Some(Ok(sh)) = shapes.get(h)
            && !taken.get(h).copied().unwrap_or(true)
            && sf.same_surface(sh, SAME_SURFACE_TOL)
        {
            faces.push(h);
        }
    }
    faces
}

/// Kernel moves: one per surface (twins together); curved faces before planes.
fn kernel_moves(shapes: &Shapes, delta: &[f64], min_move: f64) -> Vec<FaceMove> {
    let nf = shapes.len();
    let mut by_surface: Vec<(Vec<usize>, f64)> = Vec::new();
    let mut taken = vec![false; nf];
    for f in 0..nf {
        if taken.get(f).copied().unwrap_or(true) {
            continue;
        }
        let d = delta.get(f).copied().unwrap_or(0.0);
        let Some(Ok(sf)) = shapes.get(f) else { continue };
        let faces = surface_twins(shapes, &taken, f, sf);
        for &h in &faces {
            if let Some(t) = taken.get_mut(h) {
                *t = true;
            }
        }
        if d.abs() >= min_move {
            by_surface.push((faces, d));
        }
    }
    by_surface.sort_by_key(|(faces, _)| {
        matches!(faces.first().and_then(|f| shapes.get(*f)), Some(Ok(FaceShape::Plane { .. })))
    });
    by_surface.iter().map(|(faces, d)| FaceMove { faces: faces.clone(), distance: *d }).collect()
}

/// The planned kernel moves and the reason each face that does not move stays.
struct MovePlan {
    moves: Vec<FaceMove>,
    reason: Vec<Option<String>>,
}

/// Groups the faces, solves the move of every group and plans the kernel moves.
fn plan_moves(shapes: &Shapes, offsets: &[Vec<f64>], stiff: &[Stiffness], o: &FaceOpts) -> MovePlan {
    let groups = face_groups(shapes, o.keep_relations);
    let ctx = MoveCtx {
        shapes,
        offsets,
        stiff,
        default_stiffness: o.default_stiffness,
        max_change: o.mc,
        min_points: o.min_points,
    };
    let GroupMoves { delta, reason } = ctx.solve(&groups);
    MovePlan { moves: kernel_moves(shapes, &delta, o.min_move), reason }
}

/// The rebuilt body and, per face of the original, the move applied or why the kernel refused it.
struct Rebuilt {
    brep: Brep,
    applied_offset: Vec<f64>,
    failed_reason: Vec<Option<String>>,
}

/// Applies the planned moves in the kernel and records the outcome of each face.
fn rebuild(brep: &Brep, moves: &[FaceMove], nf: usize) -> CmdResult<Rebuilt> {
    let mut applied_offset = vec![0.0; nf];
    let mut failed_reason: Vec<Option<String>> = vec![None; nf];
    if moves.is_empty() {
        return Ok(Rebuilt { brep: brep.clone(), applied_offset, failed_reason });
    }
    let (nb, outcomes) = offset_face_groups(brep, moves)?;
    if outcomes.iter().all(|o| !o.applied) {
        let why = outcomes.iter().filter_map(|o| o.message.clone()).next().unwrap_or_default();
        return Err(CmdError::Failed(format!("the kernel could not move any face: {why}")));
    }
    for (m, o) in moves.iter().zip(&outcomes) {
        for &f in &m.faces {
            if o.applied {
                if let Some(a) = applied_offset.get_mut(f) {
                    *a = m.distance;
                }
            } else if let Some(r) = failed_reason.get_mut(f) {
                *r = o.message.clone();
            }
        }
    }
    Ok(Rebuilt { brep: nb, applied_offset, failed_reason })
}

/// Refuses a rebuilt body without a positive finite volume.
fn check_volume(new_brep: &Brep) -> CmdResult<()> {
    let volume = surfacecraft_kernel::solid::volume(new_brep).map_err(|e| CmdError::Failed(e.to_string()))?;
    if !(volume.is_finite() && volume > 0.0) {
        return Err(CmdError::Failed("the rebuilt body has no volume".into()));
    }
    Ok(())
}

/// The per-face report of the refit, accumulated over all faces.
struct ReportAcc {
    faces: Vec<FaceOut>,
    sq_before: f64,
    sq_after: f64,
    n_pts: usize,
    moved: usize,
}

/// Inputs of the per-face report.
struct Report<'a> {
    shapes: &'a Shapes,
    new_shapes: &'a Shapes,
    stiff: &'a [Stiffness],
    default_stiffness: Stiffness,
    offsets: &'a [Vec<f64>],
    applied_offset: &'a [f64],
    failed_reason: &'a [Option<String>],
    reason: &'a [Option<String>],
    points: &'a [Point3],
    assigned: &'a [Option<(usize, f64)>],
    min_points: usize,
}

/// The report line of a face whose shape is refused.
fn refused_face(f: usize, st: Stiffness, why: &str) -> FaceOut {
    FaceOut {
        face: f,
        kind: "freeform".into(),
        stiffness: st.name().into(),
        status: "refused".into(),
        points: 0,
        offset: 0.0,
        rms_before: None,
        rms_after: None,
        old: BTreeMap::new(),
        new: BTreeMap::new(),
        reason: Some(why.to_string()),
    }
}

impl Report<'_> {
    /// The report line of every face, with the sums behind the overall RMS.
    fn run(&self) -> ReportAcc {
        let nf = self.shapes.len();
        let mut acc = ReportAcc { faces: Vec::with_capacity(nf), sq_before: 0.0, sq_after: 0.0, n_pts: 0, moved: 0 };
        for f in 0..nf {
            let st = self.stiff.get(f).copied().unwrap_or(self.default_stiffness);
            match self.shapes.get(f) {
                Some(Err(why)) => acc.faces.push(refused_face(f, st, why)),
                Some(Ok(shape)) => self.add_face(f, st, shape, &mut acc),
                None => {}
            }
        }
        acc
    }

    /// The shape of `shape` after a move of `off`: read back from the rebuilt body, else predicted.
    fn after_shape(&self, shape: &FaceShape, off: f64) -> Option<FaceShape> {
        let predicted = if off != 0.0 { shape.offset(off) } else { Some(shape.clone()) };
        let found = predicted
            .as_ref()
            .and_then(|p| self.new_shapes.iter().flatten().find(|n| n.same_surface(p, 1e-4)))
            .cloned();
        found.or(predicted)
    }

    /// Sum of squared distances of the scan points of face `f` from the rebuilt face `ns`.
    fn sq_after(&self, f: usize, ns: &FaceShape) -> f64 {
        // the same scan points measured against the rebuilt face
        let mut acc = 0.0;
        let mut cnt = 0;
        for (p, a) in self.points.iter().zip(self.assigned) {
            if matches!(a, Some((g, _)) if *g == f) {
                let d = ns.signed_distance([p.x, p.y, p.z]);
                if d.is_finite() {
                    acc += d * d;
                    cnt += 1;
                }
            }
        }
        if cnt == 0 { 0.0 } else { acc }
    }

    /// The reason text and status of face `f`, which has `own_len` scan points and moved by `off`.
    fn status(&self, f: usize, st: Stiffness, off: f64, own_len: usize) -> (String, &'static str) {
        if let Some(m) = self.failed_reason.get(f).and_then(Clone::clone) {
            (m, "failed")
        } else if st == Stiffness::Fixed {
            ("fixed by request".to_string(), "fixed")
        } else if off != 0.0 {
            (String::new(), "moved")
        } else if let Some(r) = self.reason.get(f).and_then(Clone::clone) {
            (r, if own_len < self.min_points { "no_data" } else { "unchanged" })
        } else {
            ("the move is below min_move".to_string(), "unchanged")
        }
    }

    /// Adds the report line of the analytic face `f` and its sums to `acc`.
    fn add_face(&self, f: usize, st: Stiffness, shape: &FaceShape, acc: &mut ReportAcc) {
        let own = self.offsets.get(f).map_or(&[][..], Vec::as_slice);
        let off = self.applied_offset.get(f).copied().unwrap_or(0.0);
        let after_shape = self.after_shape(shape, off);
        let (b2, a2): (f64, f64) =
            (own.iter().map(|d| d * d).sum(), after_shape.as_ref().map_or(f64::NAN, |ns| self.sq_after(f, ns)));
        acc.sq_before += b2;
        if a2.is_finite() {
            acc.sq_after += a2;
        }
        acc.n_pts += own.len();
        let status = self.status(f, st, off, own.len());
        if status.1 == "moved" {
            acc.moved += 1;
        }
        let n = own.len().max(1) as f64;
        acc.faces.push(FaceOut {
            face: f,
            kind: shape.kind().into(),
            stiffness: st.name().into(),
            status: status.1.into(),
            points: own.len(),
            offset: off,
            rms_before: (!own.is_empty()).then(|| (b2 / n).sqrt()),
            rms_after: (!own.is_empty() && a2.is_finite()).then(|| (a2 / n).sqrt()),
            old: shape_params(shape),
            new: after_shape.as_ref().map(shape_params).unwrap_or_default(),
            reason: (!status.0.is_empty()).then_some(status.0),
        });
    }
}

/// The command's result from the per-face report and the shapes before and after.
fn faces_result(
    body: ObjectId,
    rep: ReportAcc,
    unassigned: usize,
    before: &Shapes,
    after: &Shapes,
    faces_after: usize,
    apply: bool,
) -> RefitFacesOut {
    let denom = rep.n_pts.max(1) as f64;
    RefitFacesOut {
        body,
        faces: rep.faces,
        rms_before: (rep.sq_before / denom).sqrt(),
        rms_after: (rep.sq_after / denom).sqrt(),
        points: rep.n_pts,
        unassigned,
        moved: rep.moved,
        faces_before: before.len(),
        faces_after,
        relations_before: relations_of(before),
        relations_after: relations_of(after),
        applied: apply,
    }
}

pub(super) fn refit_faces(s: &mut Session, i: RefitFacesIn) -> CmdResult<RefitFacesOut> {
    let body = i.body.resolve(s, ObjectKind::Body)?;
    ensure_no_history(s, body)?;
    let o = face_opts(&i)?;
    let (points, normals) = scan_points(s, &i.mesh, i.max_points.unwrap_or(50_000))?;
    let brep = single_shell_brep(s, body)?;
    let shapes = face_shapes(&brep)?;
    let nf = shapes.len();
    let stiff = face_stiffness(&i.faces, &shapes, o.default_stiffness)?;
    // faces with their triangles, then the association of the scan points to them
    let bvhs = face_bvhs(&brep, nf)?;
    let assigned = associate(&points, &normals, &bvhs, &shapes, &o);
    let (offsets, unassigned) = point_offsets(&points, &assigned, &shapes);
    let plan = plan_moves(&shapes, &offsets, &stiff, &o);
    let rebuilt = rebuild(&brep, &plan.moves, nf)?;
    check_volume(&rebuilt.brep)?;
    let new_shapes = face_shapes(&rebuilt.brep)?;
    let rep = Report {
        shapes: &shapes,
        new_shapes: &new_shapes,
        stiff: &stiff,
        default_stiffness: o.default_stiffness,
        offsets: &offsets,
        applied_offset: &rebuilt.applied_offset,
        failed_reason: &rebuilt.failed_reason,
        reason: &plan.reason,
        points: &points,
        assigned: &assigned,
        min_points: o.min_points,
    }
    .run();
    let moved = rep.moved;
    let apply = i.apply.unwrap_or(true);
    let out = faces_result(body, rep, unassigned, &shapes, &new_shapes, rebuilt.brep.faces.len(), apply);
    if apply
        && moved > 0
        && let Some(b) = s.doc.body_mut(body)
    {
        b.brep = Arc::new(rebuilt.brep);
    }
    Ok(out)
}

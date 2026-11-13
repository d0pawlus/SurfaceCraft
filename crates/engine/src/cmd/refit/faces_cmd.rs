//! The `body.refit_faces` command: associate scan points to faces, move the faces and rebuild the solid.

use super::faces::*;
use super::*;

pub(super) fn refit_faces(s: &mut Session, i: RefitFacesIn) -> CmdResult<RefitFacesOut> {
    let body = i.body.resolve(s, ObjectKind::Body)?;
    if s.doc.features.iter().any(|f| f.body == body) {
        return Err(CmdError::Failed(
            "this body is made by a feature history (a replay would undo the edit): use body.refit_features".into(),
        ));
    }
    let mc = positive(i.max_change, 1.0, "max_change")?;
    let max_distance = positive(i.max_distance, mc, "max_distance")?;
    let max_angle = positive(i.max_angle, 60.0, "max_angle")?;
    if max_angle >= 180.0 {
        return Err(CmdError::Params("max_angle must be below 180 degrees".into()));
    }
    let min_move = positive(i.min_move, 0.002, "min_move")?;
    let min_points = i.min_points.unwrap_or(8).max(1);
    let keep_relations = i.keep_relations.unwrap_or(true);
    let default_stiffness = i.stiffness.unwrap_or(Stiffness::Loose);
    let (points, normals) = scan_points(s, &i.mesh, i.max_points.unwrap_or(50_000))?;
    let brep = s.doc.body(body).map(|b| b.brep.clone()).ok_or_else(|| CmdError::Params(format!("no body {body}")))?;
    let closed_shells =
        brep.shells.iter().filter(|sh| matches!(sh, surfacecraft_kernel::record::brep::Shell::Faces { .. })).count();
    if closed_shells != 1 {
        return Err(CmdError::Failed("only a body of one shell is refitted (not several solids or cavities)".into()));
    }
    let shapes = face_shapes(&brep)?;
    let nf = shapes.len();
    // stiffness per face
    let mut stiff = vec![default_stiffness; nf];
    let mut explicit = vec![false; nf];
    for fs in &i.faces {
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
    // faces with their triangles
    let chord = (bbox_diagonal(&brep) * 2e-4).clamp(0.005, 0.05);
    let topo = surfacecraft_kernel::tess::tessellate_topo(&brep, Some(chord))?;
    if topo.tri_face.is_empty() || topo.mesh.triangle_count() == 0 {
        return Err(CmdError::Failed("the body could not be meshed to associate the scan to its faces".into()));
    }
    let mut tris: Vec<Vec<u32>> = vec![Vec::new(); nf];
    for (t, f) in topo.tri_face.iter().enumerate() {
        if let Some(list) = tris.get_mut(*f as usize) {
            list.push(t as u32);
        }
    }
    let bvhs: Vec<Option<Bvh>> =
        tris.iter().map(|t| (!t.is_empty()).then(|| Bvh::build(&topo.mesh.submesh(t)))).collect();
    // association: the nearest face (within max_distance) whose normal faces the scan point's
    let cos_min = max_angle.to_radians().cos();
    let assigned: Vec<Option<(usize, f64)>> = par::map(points.len(), |k| {
        let (Some(p), Some(n)) = (points.get(k), normals.get(k)) else { return None };
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
    });
    // signed offsets of the scan points from the analytic face they belong to
    let mut offsets: Vec<Vec<f64>> = vec![Vec::new(); nf];
    let mut unassigned = 0;
    for (p, a) in points.iter().zip(&assigned) {
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
    // groups of faces that must move together
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
    let mut groups: Vec<Group> = Vec::new();
    let mut slot_of: BTreeMap<usize, usize> = BTreeMap::new();
    for f in 0..nf {
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
    // θ per group, then the move of every face: δ = sign·θ
    let mut delta = vec![0.0; nf];
    let mut reason: Vec<Option<String>> = vec![None; nf];
    for g in &groups {
        let members: Vec<(f64, &[f64], f64)> = g
            .faces
            .iter()
            .map(|&f| {
                let sign = shapes.get(f).and_then(|s| s.as_ref().ok()).map_or(1.0, FaceShape::radius_sign);
                let d: &[f64] = offsets.get(f).map_or(&[], Vec::as_slice);
                (sign, d, stiff.get(f).copied().unwrap_or(default_stiffness).prior())
            })
            .collect();
        let total: usize = members.iter().map(|m| m.1.len()).sum();
        let fixed_member = g.faces.iter().copied().find(|&f| stiff.get(f) == Some(&Stiffness::Fixed));
        let theta = solve_group(&members, mc);
        for &f in &g.faces {
            let sign = shapes.get(f).and_then(|s| s.as_ref().ok()).map_or(1.0, FaceShape::radius_sign);
            let own = offsets.get(f).map_or(0, Vec::len);
            if let Some(fx) = fixed_member
                && stiff.get(f) != Some(&Stiffness::Fixed)
            {
                if let Some(r) = reason.get_mut(f) {
                    *r = Some(format!("tied to the fixed face {fx} (same surface or equal radius)"));
                }
                continue;
            }
            if total < min_points {
                if let Some(r) = reason.get_mut(f) {
                    *r = Some(format!("too few scan points ({total}, at least {min_points})"));
                }
                continue;
            }
            if own < min_points && g.faces.len() == 1 {
                if let Some(r) = reason.get_mut(f) {
                    *r = Some(format!("too few scan points ({own}, at least {min_points})"));
                }
                continue;
            }
            if let Some(d) = delta.get_mut(f) {
                *d = sign * theta;
            }
        }
    }
    // kernel moves: one per surface (twins together); curved faces before planes
    let mut by_surface: Vec<(Vec<usize>, f64)> = Vec::new();
    let mut taken = vec![false; nf];
    for f in 0..nf {
        if taken.get(f).copied().unwrap_or(true) {
            continue;
        }
        let d = delta.get(f).copied().unwrap_or(0.0);
        let Some(Ok(sf)) = shapes.get(f) else { continue };
        let mut faces = vec![f];
        for h in (f + 1)..nf {
            if let Some(Ok(sh)) = shapes.get(h)
                && !taken.get(h).copied().unwrap_or(true)
                && sf.same_surface(sh, SAME_SURFACE_TOL)
            {
                faces.push(h);
            }
        }
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
    let moves: Vec<FaceMove> =
        by_surface.iter().map(|(faces, d)| FaceMove { faces: faces.clone(), distance: *d }).collect();
    let mut applied_offset = vec![0.0; nf];
    let mut failed_reason: Vec<Option<String>> = vec![None; nf];
    let new_brep = if moves.is_empty() {
        (*brep).clone()
    } else {
        let (nb, outcomes) = offset_face_groups(&brep, &moves)?;
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
        nb
    };
    let volume = surfacecraft_kernel::solid::volume(&new_brep).map_err(|e| CmdError::Failed(e.to_string()))?;
    if !(volume.is_finite() && volume > 0.0) {
        return Err(CmdError::Failed("the rebuilt body has no volume".into()));
    }
    let new_shapes = face_shapes(&new_brep)?;
    // report
    let mut faces_out = Vec::with_capacity(nf);
    let (mut sq_before, mut sq_after, mut n_pts) = (0.0, 0.0, 0usize);
    let mut moved = 0;
    for f in 0..nf {
        let st = stiff.get(f).copied().unwrap_or(default_stiffness);
        let own = offsets.get(f).map_or(&[][..], Vec::as_slice);
        match shapes.get(f) {
            Some(Err(why)) => faces_out.push(FaceOut {
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
                reason: Some(why.clone()),
            }),
            Some(Ok(shape)) => {
                let off = applied_offset.get(f).copied().unwrap_or(0.0);
                let predicted = if off != 0.0 { shape.offset(off) } else { Some(shape.clone()) };
                let found = predicted
                    .as_ref()
                    .and_then(|p| new_shapes.iter().flatten().find(|n| n.same_surface(p, 1e-4)))
                    .cloned();
                let after_shape = found.clone().or(predicted.clone());
                let (b2, a2): (f64, f64) = (
                    own.iter().map(|d| d * d).sum(),
                    after_shape.as_ref().map_or(f64::NAN, |ns| {
                        // the same scan points measured against the rebuilt face
                        let mut acc = 0.0;
                        let mut cnt = 0;
                        for (p, a) in points.iter().zip(&assigned) {
                            if matches!(a, Some((g, _)) if *g == f) {
                                let d = ns.signed_distance([p.x, p.y, p.z]);
                                if d.is_finite() {
                                    acc += d * d;
                                    cnt += 1;
                                }
                            }
                        }
                        if cnt == 0 { 0.0 } else { acc }
                    }),
                );
                sq_before += b2;
                if a2.is_finite() {
                    sq_after += a2;
                }
                n_pts += own.len();
                let status = if let Some(m) = failed_reason.get(f).and_then(Clone::clone) {
                    (m, "failed")
                } else if st == Stiffness::Fixed {
                    ("fixed by request".to_string(), "fixed")
                } else if off != 0.0 {
                    (String::new(), "moved")
                } else if let Some(r) = reason.get(f).and_then(Clone::clone) {
                    (r, if own.len() < min_points { "no_data" } else { "unchanged" })
                } else {
                    ("the move is below min_move".to_string(), "unchanged")
                };
                if status.1 == "moved" {
                    moved += 1;
                }
                let n = own.len().max(1) as f64;
                faces_out.push(FaceOut {
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
            None => {}
        }
    }
    let denom = n_pts.max(1) as f64;
    let apply = i.apply.unwrap_or(true);
    let out = RefitFacesOut {
        body,
        faces: faces_out,
        rms_before: (sq_before / denom).sqrt(),
        rms_after: (sq_after / denom).sqrt(),
        points: n_pts,
        unassigned,
        moved,
        faces_before: nf,
        faces_after: new_brep.faces.len(),
        relations_before: relations_of(&shapes),
        relations_after: relations_of(&new_shapes),
        applied: apply,
    };
    if apply
        && moved > 0
        && let Some(b) = s.doc.body_mut(body)
    {
        b.brep = Arc::new(new_brep);
    }
    Ok(out)
}

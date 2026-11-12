//! Unstructured layouts (P4): `quad.retopo`, the layout helpers shared with `quad.split` and
//! `quad.snap`, and the unstructured branch of `surface.auto`.
use super::{AutoBlock, AutoIn, AutoOut, CmdError, CmdResult, ObjRef, QuadInfo};
use crate::Session;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind, QuadObject, SurfaceObject};
use surfacecraft_fit::mls::ScanSurface;
use surfacecraft_fit::patch::{build_patches, deviation, edge_normal_jumps};
use surfacecraft_fit::retopo::{QuadMesh, RetopoOptions, retopo};
use surfacecraft_fit::segment::region_points;
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3};
use surfacecraft_kernel::nurbs::{is_closed, sheet_from_bezier_network};

/// Mean triangle edge length of `tris` (the scan's sampling distance), mm.
pub(super) fn mean_edge(m: &Mesh, tris: &[u32]) -> f64 {
    let (mut sum, mut n) = (0.0, 0usize);
    for t in tris.iter().step_by(tris.len().div_ceil(20_000).max(1)) {
        if let Some([a, b, c]) = m.triangle(*t as usize) {
            sum += (b - a).norm() + (c - b).norm() + (a - c).norm();
            n += 3;
        }
    }
    sum / n.max(1) as f64
}

/// The projection used for layout vertices and patch edges: moving least squares over the scan.
pub(super) fn projector(scan: &ScanSurface, radius: f64) -> impl Fn(&Point3, &Vec3) -> Option<(Point3, Vec3)> + '_ {
    move |p, n| scan.project(p, radius, n)
}

/// Retopology parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RetopoIn {
    /// Region to cover (wrapped or closed regions are fine).
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Mesh to cover whole (when no region).
    #[serde(default)]
    pub mesh: Option<ObjRef>,
    /// Faces wanted in the field layout (default 100). Faces that are not quads force one
    /// 4-to-1 split of the whole layout into quads, so the result can hold several times more.
    #[serde(default)]
    pub target_quads: Option<usize>,
    /// Orientation-field smoothing sweeps per level (default 40, at most 500).
    #[serde(default)]
    pub orientation_sweeps: Option<usize>,
    /// Position-field smoothing sweeps per level (default 40, at most 500).
    #[serde(default)]
    pub position_sweeps: Option<usize>,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

/// Retopology result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct RetopoOut {
    /// New unstructured layout.
    pub id: ObjectId,
    /// Quads in the layout.
    pub quads: usize,
    /// Layout vertices.
    pub vertices: usize,
    /// `field` (orientation/position field extraction) or `fallback` (decimation split into quads).
    pub layout: String,
    /// Faces the field extraction gave.
    pub field_faces: usize,
    /// Quads among the field faces.
    pub field_quads: usize,
    /// Share of interior vertices whose valence is not 4 (after the split to quads).
    pub irregular_fraction: f64,
    /// Edge length asked of the field (mm).
    pub edge_length: f64,
    /// Euler characteristic V − E + F of the layout.
    pub euler: i64,
    /// Closed (no free boundary)?
    pub closed: bool,
    /// Quads whose orientation is inverted against the scan normals (should be 0).
    pub folded: usize,
    /// Largest distance of a layout vertex from the scan (mm).
    pub snap_max: f64,
    /// Why the fallback was used.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

pub(super) fn retopo_cmd(s: &mut Session, i: RetopoIn) -> CmdResult<RetopoOut> {
    let (mesh, region) = match (&i.region, &i.mesh) {
        (Some(r), None) => {
            let rid = r.resolve(s, ObjectKind::Region)?;
            (s.doc.region(rid).map(|r| r.mesh).ok_or_else(|| CmdError::Params("no region".into()))?, Some(rid))
        }
        (None, Some(m)) => (m.resolve(s, ObjectKind::Mesh)?, None),
        _ => return Err(CmdError::Params("give `region` or `mesh`".into())),
    };
    let target = i.target_quads.unwrap_or(100);
    if !(8..=20_000).contains(&target) {
        return Err(CmdError::Params("target_quads must be 8..=20000".into()));
    }
    let sweeps = |v: Option<usize>, what: &str| -> CmdResult<usize> {
        let v = v.unwrap_or(40);
        if (1..=500).contains(&v) { Ok(v) } else { Err(CmdError::Params(format!("{what} must be 1..=500"))) }
    };
    let opts = RetopoOptions {
        target_faces: target,
        orientation_sweeps: sweeps(i.orientation_sweeps, "orientation_sweeps")?,
        position_sweeps: sweeps(i.position_sweeps, "position_sweeps")?,
        force_fallback: false,
    };
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let tris: Vec<u32> = match region {
        Some(r) => s.doc.region(r).map(|r| r.triangles.to_vec()).ok_or_else(|| CmdError::Params("no region".into()))?,
        None => (0..m.triangle_count() as u32).collect(),
    };
    if tris.len() < 20 {
        return Err(CmdError::Params("too few triangles to retopologise (need 20)".into()));
    }
    let sub = m.submesh(&tris);
    let normals = m.vertex_normals();
    let (pts, point_normals) = region_points(&m, &normals, &tris);
    let scan = ScanSurface::oriented(&pts, &point_normals);
    // layout from the fields; folded quads (rare, at field defects) trigger the fallback layout
    let mut attempt = 0;
    let (quads, rep, field_faces, field_quads, folded) = loop {
        let (mut poly, rep) =
            retopo(&sub, &RetopoOptions { force_fallback: attempt > 0, ..opts.clone() }).map_err(CmdError::Failed)?;
        let radius = (0.6 * rep.edge_length).max(3.0 * mean_edge(&m, &tris));
        let project = projector(&scan, radius);
        for (p, n) in poly.vertices.iter_mut().zip(poly.normals.iter_mut()) {
            if let Some((q, nn)) = project(p, n) {
                *p = q;
                *n = nn;
            }
        }
        poly.relax(6, &project);
        let field_faces = poly.faces.len();
        let field_quads = poly.faces.iter().filter(|f| f.len() == 4).count();
        let mut quads = match poly.as_quads() {
            Some(q) => q,
            None => poly.split_to_quads(&project),
        };
        quads.relax(8, &project);
        let folded = quads.folded();
        if folded == 0 || attempt > 0 {
            break (quads, rep, field_faces, field_quads, folded);
        }
        attempt += 1;
    };
    let bvh = Bvh::build(&sub);
    let snap_max =
        quads.vertices.iter().filter_map(|p| bvh.closest(p, f64::INFINITY)).map(|h| h.distance).fold(0.0, f64::max);
    let val = quads.valences();
    let boundary = quads.boundary_edges();
    let mut on_b = vec![false; quads.vertices.len()];
    for (a, b) in &boundary {
        if let Some(x) = on_b.get_mut(*a as usize) {
            *x = true;
        }
        if let Some(x) = on_b.get_mut(*b as usize) {
            *x = true;
        }
    }
    let inner: Vec<usize> = (0..val.len()).filter(|&k| !on_b[k] && val[k] > 0).collect();
    let irregular = inner.iter().filter(|&&k| val[k] != 4).count() as f64 / inner.len().max(1) as f64;
    let id = s.doc.add_quad(QuadObject {
        id: ObjectId(0),
        name: i.name.unwrap_or_else(|| "retopo".into()),
        mesh,
        region,
        frame: [[0.0; 3]; 4],
        domain: [[0.0; 2]; 2],
        nu: 0,
        nv: 0,
        vertices: quads.vertices.iter().map(|p| [p.x, p.y, p.z]).collect(),
        crease_u: Vec::new(),
        crease_v: Vec::new(),
        faces: quads.faces.clone(),
        normals: quads.normals.iter().map(|n| [n.x, n.y, n.z]).collect(),
        visible: true,
    });
    Ok(RetopoOut {
        id,
        quads: quads.faces.len(),
        vertices: quads.vertices.len(),
        layout: match rep.layout {
            surfacecraft_fit::retopo::Layout::Field => "field".into(),
            surfacecraft_fit::retopo::Layout::Fallback => "fallback".into(),
        },
        field_faces,
        field_quads,
        irregular_fraction: irregular,
        edge_length: rep.edge_length,
        euler: quads.euler(),
        closed: boundary.is_empty(),
        folded,
        snap_max,
        note: if attempt > 0 {
            Some("the field layout had folded quads; the fallback layout was used".into())
        } else {
            rep.note
        },
    })
}

/// The unstructured layout of a quad object.
pub(super) fn layout_of(q: &QuadObject) -> QuadMesh {
    let v3 = |p: &[f64; 3]| Point3::new(p[0], p[1], p[2]);
    QuadMesh {
        vertices: q.vertices.iter().map(v3).collect(),
        normals: q.normals.iter().map(|n| Vec3::new(n[0], n[1], n[2])).collect(),
        faces: q.faces.clone(),
    }
}

/// One layout.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct QuadIn {
    /// Quad layout id or name.
    pub object: ObjRef,
}

/// Largest unstructured layout (quads) a split or an automatic refinement produces.
pub(super) const MAX_LAYOUT_QUADS: usize = 12_000;

/// Mean quad edge length of a layout (mm).
pub(super) fn layout_edge(l: &QuadMesh) -> f64 {
    let (mut sum, mut n) = (0.0, 0usize);
    for f in &l.faces {
        for k in 0..4 {
            if let (Some(a), Some(b)) = (l.vertices.get(f[k] as usize), l.vertices.get(f[(k + 1) % 4] as usize)) {
                sum += (b - a).norm();
                n += 1;
            }
        }
    }
    sum / n.max(1) as f64
}

/// MLS support for a layout: wide enough to average the scan noise, narrow enough to follow it.
pub(super) fn layout_radius(l: &QuadMesh, spacing: f64) -> f64 {
    (0.8 * layout_edge(l)).max(3.0 * spacing)
}

/// Checks an unstructured layout read from a document (hostile files).
pub(super) fn check_layout(q: &QuadObject) -> CmdResult<()> {
    let n = q.vertices.len();
    if q.normals.len() != n || q.faces.iter().flatten().any(|&i| i as usize >= n) {
        return Err(CmdError::Failed("the layout is inconsistent (normals or face ids)".into()));
    }
    if q.vertices.iter().flatten().chain(q.normals.iter().flatten()).any(|v| !v.is_finite()) {
        return Err(CmdError::Failed("the layout has non-finite coordinates".into()));
    }
    Ok(())
}

/// The scan points under layout `q`, their normals and the mean edge length of the scan there.
pub(super) fn scan_for(s: &Session, q: &QuadObject) -> CmdResult<(Vec<Point3>, Vec<Vec3>, f64)> {
    let m =
        s.doc.mesh(q.mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {}", q.mesh)))?;
    let tris: Vec<u32> = match q.region {
        Some(r) => s.doc.region(r).map(|r| r.triangles.to_vec()).ok_or_else(|| CmdError::Params("no region".into()))?,
        None => (0..m.triangle_count() as u32).collect(),
    };
    let normals = m.vertex_normals();
    let (pts, point_normals) = region_points(&m, &normals, &tris);
    Ok((pts, point_normals, mean_edge(&m, &tris)))
}

pub(super) fn info_of(q: &QuadObject, snap_max: f64) -> QuadInfo {
    QuadInfo { id: q.id, nu: q.nu, nv: q.nv, creases: (q.crease_u.clone(), q.crease_v.clone()), snap_max }
}

pub(super) fn split_unstructured(s: &mut Session, id: ObjectId) -> CmdResult<QuadInfo> {
    let q = s.doc.quad(id).cloned().ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    check_layout(&q)?;
    if q.faces.len() * 4 > MAX_LAYOUT_QUADS {
        return Err(CmdError::Params(format!("the layout would exceed {MAX_LAYOUT_QUADS} quads")));
    }
    let (pts, point_normals, spacing) = scan_for(s, &q)?;
    let scan = ScanSurface::oriented(&pts, &point_normals);
    let layout = layout_of(&q);
    let split = layout.split(&projector(&scan, layout_radius(&layout, spacing)));
    let snap_max = {
        let bvh_pts = ScanSurface::new(&pts);
        split.vertices.iter().filter_map(|p| bvh_pts.nearest(p)).map(|x| x.1).fold(0.0, f64::max)
    };
    if let Some(m) = s.doc.quad_mut(id) {
        m.vertices = split.vertices.iter().map(|p| [p.x, p.y, p.z]).collect();
        m.normals = split.normals.iter().map(|n| [n.x, n.y, n.z]).collect();
        m.faces = split.faces;
    }
    let q = s.doc.quad(id).ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    Ok(info_of(q, snap_max))
}

/// AutoSurface of an unstructured layout: bicubic patches, refined until within `tol`.
pub(super) fn auto_unstructured(s: &mut Session, q: &QuadObject, tol: f64, i: &AutoIn) -> CmdResult<AutoOut> {
    check_layout(q)?;
    let (pts, point_normals, spacing) = scan_for(s, q)?;
    if pts.len() < 20 {
        return Err(CmdError::Failed("the region has too few points".into()));
    }
    let scan = ScanSurface::oriented(&pts, &point_normals);
    let max_patches = i.max_patches.unwrap_or(6000).clamp(1, MAX_LAYOUT_QUADS);
    let mut layout = layout_of(q);
    if layout.faces.len() > max_patches {
        return Err(CmdError::Params(format!("the layout has more than max_patches = {max_patches} quads")));
    }
    let mut refinements = 0usize;
    let mut flipped = false;
    loop {
        let radius = layout_radius(&layout, spacing);
        let set = build_patches(&layout, &scan, radius);
        if set.nets.len() != layout.faces.len() {
            return Err(CmdError::Failed("the layout has quads with missing edges".into()));
        }
        let dev = deviation(&set, &pts, 12);
        let can_split = layout.faces.len() * 4 <= max_patches && refinements < 3;
        if dev.max > tol && can_split {
            layout = layout.split(&projector(&scan, radius));
            refinements += 1;
            continue;
        }
        let nets: Vec<Vec<Vec<[f64; 3]>>> = set.nets.iter().map(|n| n.iter().map(|r| r.to_vec()).collect()).collect();
        let sheet = sheet_from_bezier_network(&nets, &set.corners)?;
        let closed = is_closed(&sheet);
        let mut volume = None;
        if closed {
            match surfacecraft_kernel::solid::volume(&sheet) {
                Ok(v) if v < 0.0 && !flipped => {
                    // the layout runs against its normals: turn every quad around and build again
                    for f in &mut layout.faces {
                        f.swap(1, 3);
                    }
                    for n in &mut layout.normals {
                        *n = -*n;
                    }
                    flipped = true;
                    continue;
                }
                Ok(v) => volume = Some(v.abs()),
                Err(_) => {}
            }
        }
        let jumps = edge_normal_jumps(&set);
        let jmax = jumps.iter().copied().fold(0.0, f64::max).to_degrees();
        let jmean = (jumps.iter().sum::<f64>() / jumps.len().max(1) as f64).to_degrees();
        let display = surfacecraft_kernel::tess::tessellate(&sheet, None).unwrap_or_default();
        let name = i.name.clone().unwrap_or_else(|| "autosurface".into());
        let patches = sheet.faces.len();
        let solid = if closed && i.make_solid.unwrap_or(true) {
            Some(s.doc.add_body(&format!("{name}_solid"), sheet.clone()))
        } else {
            None
        };
        let sid = s.doc.add_surface(SurfaceObject {
            id: ObjectId(0),
            name,
            sheet: Arc::new(sheet),
            region: q.region,
            control: [4, 4],
            degree: 3,
            rms: dev.rms,
            max: dev.max,
            display: Arc::new(display),
            parent: None,
            visible: true,
        });
        let within = dev.max <= tol;
        let warning = (!within).then(|| {
            format!("max deviation {:.4} mm exceeds {tol} mm after {refinements} refinements; raise max_patches or retopologise finer", dev.max)
        });
        return Ok(AutoOut {
            blocks: vec![AutoBlock { id: sid, patches, rms: dev.rms, max: dev.max }],
            max: dev.max,
            within_tolerance: within,
            warning,
            normal_jump_max_deg: Some(jmax),
            normal_jump_mean_deg: Some(jmean),
            closed: Some(closed),
            volume,
            solid,
            refinements: Some(refinements),
            uncovered: Some(dev.uncovered),
            folded: Some(layout.folded()),
        });
    }
}

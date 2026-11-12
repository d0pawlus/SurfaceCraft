//! Unstructured layouts (P4): `quad.retopo`, the layout helpers shared with `quad.split` and
//! `quad.snap`, and the unstructured branch of `surface.auto`.
use super::{AutoBlock, AutoIn, AutoOut, CmdError, CmdResult, ObjRef, QuadInfo};
use crate::Session;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind, QuadObject, SurfaceObject};
use surfacecraft_fit::boundary::{Outline, cleaned_outline};
use surfacecraft_fit::mls::ScanSurface;
use surfacecraft_fit::orient::{FlipReport, repair_orientation, repair_patches, same_side_normal};
use surfacecraft_fit::patch::{Deviation, PatchSet, deviation, edge_normal_jumps};
use surfacecraft_fit::retopo::{
    CheckedSplit, QuadMesh, RetopoOptions, RetopoReport, checked_split, invalid_quads, retopo,
};
use surfacecraft_fit::segment::region_points;
use surfacecraft_geom::{Bvh, KdTree, Mesh, Point3, Vec3};
use surfacecraft_kernel::nurbs::{is_closed, sheet_from_bezier_network};
use surfacecraft_kernel::record::brep::Brep;

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
    /// Experimental: conform the layout of an open region to a smoothed outline of the region and put split boundary vertices back onto it (default false). Falls back to the plain layout when any quad would be improper.
    #[serde(default)]
    pub conform_boundary: Option<bool>,
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
    /// Open regions: is every free boundary loop of the layout conformed to the region outline (false
    /// when any loop is left as it was, or when the final check reverted the conform)?
    pub outline_conformed: bool,
    /// Open regions: free boundary loops of the layout conformed to the region outline.
    pub outline_loops_conformed: usize,
    /// Open regions: free boundary loops left exactly as they were (no matching outline loop, order
    /// mismatch, a span too short for its vertices, or an invalid face after snapping).
    pub outline_loops_skipped: usize,
    /// Why the first such loop was left as it was.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outline_unconformed_reason: Option<String>,
    /// Open regions: faces added between the layout and the outline.
    pub outline_ring_faces: usize,
    /// Quads whose patch was turned against the scan and that were turned around (repaired).
    pub repaired_patches: usize,
    /// Quads whose patch still runs against the scan (should be 0; `surface.auto` then refuses the layout).
    pub flipped_patches: usize,
    /// Quads whose patch could not be checked against the scan orientation (no samples on its side).
    pub unverified_patches: usize,
    /// Largest distance of a layout vertex from the scan (mm).
    pub snap_max: f64,
    /// Why the fallback was used.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

/// The mesh and region that `quad.retopo` covers: a region with its mesh, or a whole mesh.
fn retopo_source(s: &Session, i: &RetopoIn) -> CmdResult<(ObjectId, Option<ObjectId>)> {
    match (&i.region, &i.mesh) {
        (Some(r), None) => {
            let rid = r.resolve(s, ObjectKind::Region)?;
            let mesh = s.doc.region(rid).map(|r| r.mesh).ok_or_else(|| CmdError::Params("no region".into()))?;
            Ok((mesh, Some(rid)))
        }
        (None, Some(m)) => Ok((m.resolve(s, ObjectKind::Mesh)?, None)),
        _ => Err(CmdError::Params("give `region` or `mesh`".into())),
    }
}

/// A sweep count of the input: default 40, at most 500.
fn sweeps_in(v: Option<usize>, what: &str) -> CmdResult<usize> {
    let v = v.unwrap_or(40);
    if (1..=500).contains(&v) { Ok(v) } else { Err(CmdError::Params(format!("{what} must be 1..=500"))) }
}

/// The retopology options of the input, with the defaults and limits of `quad.retopo`.
fn retopo_options(i: &RetopoIn) -> CmdResult<RetopoOptions> {
    let target = i.target_quads.unwrap_or(100);
    if !(8..=20_000).contains(&target) {
        return Err(CmdError::Params("target_quads must be 8..=20000".into()));
    }
    Ok(RetopoOptions {
        target_faces: target,
        orientation_sweeps: sweeps_in(i.orientation_sweeps, "orientation_sweeps")?,
        position_sweeps: sweeps_in(i.position_sweeps, "position_sweeps")?,
        force_fallback: false,
        conform_boundary: i.conform_boundary.unwrap_or(false),
    })
}

/// The triangles that `quad.retopo` covers: the region's, or every triangle of `m`.
fn retopo_triangles(s: &Session, m: &Mesh, region: Option<ObjectId>) -> CmdResult<Vec<u32>> {
    let tris: Vec<u32> = match region {
        Some(r) => s.doc.region(r).map(|r| r.triangles.to_vec()).ok_or_else(|| CmdError::Params("no region".into()))?,
        None => (0..m.triangle_count() as u32).collect(),
    };
    if tris.len() < 20 {
        return Err(CmdError::Params("too few triangles to retopologise (need 20)".into()));
    }
    Ok(tris)
}

/// The scan a layout is fitted to: its points with their oriented normals, and the sampling distance.
struct LayoutScan {
    pts: Vec<Point3>,
    point_normals: Vec<Vec3>,
    scan: ScanSurface,
    spacing: f64,
}

impl LayoutScan {
    fn from_parts(pts: Vec<Point3>, point_normals: Vec<Vec3>, spacing: f64) -> Self {
        let scan = ScanSurface::oriented(&pts, &point_normals);
        Self { pts, point_normals, scan, spacing }
    }

    /// The scan of the triangles `tris` of `m`.
    fn of_region(m: &Mesh, tris: &[u32]) -> Self {
        let normals = m.vertex_normals();
        let (pts, point_normals) = region_points(m, &normals, tris);
        Self::from_parts(pts, point_normals, mean_edge(m, tris))
    }
}

/// One layout attempt: field extraction (or the decimation fallback when `force_fallback`),
/// projected onto the scan, relaxed, and checked for folded and flipped quads.
struct FieldLayout {
    quads: QuadMesh,
    rep: RetopoReport,
    field_faces: usize,
    field_quads: usize,
    folded: usize,
    flips: FlipReport,
    /// Quads of the split of the field polygons that are improper although their parent polygon is proper.
    split_defects: usize,
}

fn field_layout(sub: &Mesh, scan: &LayoutScan, opts: &RetopoOptions, force_fallback: bool) -> CmdResult<FieldLayout> {
    let (mut poly, rep) = retopo(sub, &RetopoOptions { force_fallback, ..opts.clone() }).map_err(CmdError::Failed)?;
    let radius = (0.6 * rep.edge_length).max(3.0 * scan.spacing);
    let project = projector(&scan.scan, radius);
    for (p, n) in poly.vertices.iter_mut().zip(poly.normals.iter_mut()) {
        if let Some((q, nn)) = project(p, n) {
            *p = q;
            *n = nn;
        }
    }
    poly.relax(6, &project);
    let field_faces = poly.faces.len();
    let field_quads = poly.faces.iter().filter(|f| f.len() == 4).count();
    let (mut quads, was_split) = match poly.as_quads() {
        Some(q) => (q, false),
        None => (poly.split_to_quads(&project), true),
    };
    quads.relax(8, &project);
    // rule: a split is accepted only if no proper polygon turns into improper quads (real geometry)
    let split_defects = if was_split { surfacecraft_fit::retopo::split_defects(&poly, &quads) } else { 0 };
    let flips = orientation_check(&mut quads, &scan.scan, (&scan.pts, &scan.point_normals), scan.spacing);
    let folded = quads.folded();
    Ok(FieldLayout { quads, rep, field_faces, field_quads, folded, flips, split_defects })
}

/// The first layout attempt without folded or flipped quads and without split defects. The fallback (attempt 1) is accepted
/// whatever it holds. Returns the layout and the attempt that gave it.
fn settled_field_layout(sub: &Mesh, scan: &LayoutScan, opts: &RetopoOptions) -> CmdResult<(FieldLayout, usize)> {
    let mut attempt = 0;
    loop {
        let layout = field_layout(sub, scan, opts, attempt > 0)?;
        if (layout.folded == 0 && layout.flips.failed == 0 && layout.split_defects == 0) || attempt > 0 {
            return Ok((layout, attempt));
        }
        attempt += 1;
    }
}

/// Share of the interior vertices of `quads` whose valence is not 4; `boundary` are its free edges.
fn interior_irregularity(quads: &QuadMesh, boundary: &[(u32, u32)]) -> f64 {
    let val = quads.valences();
    let mut on_b = vec![false; quads.vertices.len()];
    for (a, b) in boundary {
        if let Some(x) = on_b.get_mut(*a as usize) {
            *x = true;
        }
        if let Some(x) = on_b.get_mut(*b as usize) {
            *x = true;
        }
    }
    let inner: Vec<usize> = (0..val.len()).filter(|&k| !on_b[k] && val[k] > 0).collect();
    inner.iter().filter(|&&k| val[k] != 4).count() as f64 / inner.len().max(1) as f64
}

/// Largest distance of a layout vertex from the mesh `sub` (mm).
fn layout_snap_max(quads: &QuadMesh, sub: &Mesh) -> f64 {
    let bvh = Bvh::build(sub);
    quads.vertices.iter().filter_map(|p| bvh.closest(p, f64::INFINITY)).map(|h| h.distance).fold(0.0, f64::max)
}

/// Stores `quads` as a new quad object of `mesh` (and `region`) and returns its id.
fn add_retopo_layout(
    s: &mut Session,
    mesh: ObjectId,
    region: Option<ObjectId>,
    name: String,
    quads: &QuadMesh,
) -> ObjectId {
    s.doc.add_quad(QuadObject {
        id: ObjectId(0),
        name,
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
    })
}

/// Why the final net gave up conforming.
const NET_REASON: &str = "the conformed layout has an improper quad; the layout without conforming is returned";

/// The final net: a conformed layout is accepted only if every quad of its final geometry (after
/// all splits, projections and relaxations) is proper. Otherwise `plain` (the same pipeline with
/// conforming off) is returned and the reason says why. No counts are compared.
fn net_conformed(
    conformed: (FieldLayout, usize),
    plain: impl FnOnce() -> CmdResult<(FieldLayout, usize)>,
) -> CmdResult<(FieldLayout, usize, Option<&'static str>)> {
    let (fit, attempt) = conformed;
    if fit.rep.boundary.conformed == 0 || invalid_quads(&fit.quads) == 0 {
        return Ok((fit, attempt, None));
    }
    let (plain, plain_attempt) = plain()?;
    Ok((plain, plain_attempt, Some(NET_REASON)))
}

/// The settled layout through the final net.
fn netted_layout(
    sub: &Mesh,
    scan: &LayoutScan,
    opts: &RetopoOptions,
) -> CmdResult<(FieldLayout, usize, Option<&'static str>)> {
    let conformed = settled_field_layout(sub, scan, opts)?;
    net_conformed(conformed, || {
        settled_field_layout(sub, scan, &RetopoOptions { conform_boundary: false, ..opts.clone() })
    })
}

pub(super) fn retopo_cmd(s: &mut Session, i: RetopoIn) -> CmdResult<RetopoOut> {
    let (mesh, region) = retopo_source(s, &i)?;
    let opts = retopo_options(&i)?;
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let tris = retopo_triangles(s, &m, region)?;
    let sub = m.submesh(&tris);
    let scan = LayoutScan::of_region(&m, &tris);
    let (fit, attempt, reverted) = netted_layout(&sub, &scan, &opts)?;
    let boundary = fit.quads.boundary_edges();
    let irregular = interior_irregularity(&fit.quads, &boundary);
    let snap_max = layout_snap_max(&fit.quads, &sub);
    let id = add_retopo_layout(s, mesh, region, i.name.unwrap_or_else(|| "retopo".into()), &fit.quads);
    Ok(RetopoOut {
        id,
        quads: fit.quads.faces.len(),
        vertices: fit.quads.vertices.len(),
        layout: match fit.rep.layout {
            surfacecraft_fit::retopo::Layout::Field => "field".into(),
            surfacecraft_fit::retopo::Layout::Fallback => "fallback".into(),
        },
        field_faces: fit.field_faces,
        field_quads: fit.field_quads,
        irregular_fraction: irregular,
        edge_length: fit.rep.edge_length,
        euler: fit.quads.euler(),
        closed: boundary.is_empty(),
        folded: fit.folded,
        outline_loops_conformed: fit.rep.boundary.conformed,
        outline_loops_skipped: fit.rep.boundary.skipped + fit.rep.boundary.rolled_back + fit.rep.boundary.unchanged,
        outline_conformed: fit.rep.boundary.all_conformed(),
        outline_unconformed_reason: reverted.or(fit.rep.boundary.reason).map(str::to_string),
        outline_ring_faces: fit.rep.boundary.ring_faces,
        repaired_patches: fit.flips.repaired,
        flipped_patches: fit.flips.failed,
        unverified_patches: fit.flips.unverified,
        snap_max,
        note: if attempt > 0 {
            Some("the field layout had folded or flipped quads; the fallback layout was used".into())
        } else {
            fit.rep.note
        },
    })
}

/// Detects quads whose patch runs against the scan and turns them around (the change persists in
/// `layout`); `scan_points` are the scan points with their oriented normals.
pub(super) fn orientation_check(
    layout: &mut QuadMesh,
    scan: &ScanSurface,
    scan_points: (&[Point3], &[Vec3]),
    spacing: f64,
) -> FlipReport {
    let (pts, normals) = scan_points;
    let tree = KdTree::build(pts);
    let reference = |p: &Point3, hint: &Vec3| same_side_normal(&tree, normals, 1.0, p, hint);
    let radius = layout_radius(layout, spacing);
    repair_orientation(layout, scan, radius, &reference)
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
    /// Experimental: conform the layout of an open region to a smoothed outline of the region and put split boundary vertices back onto it (default false). Falls back to the plain layout when any quad would be improper.
    #[serde(default)]
    pub conform_boundary: Option<bool>,
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

/// New boundary vertices of a split move onto the outline when within this fraction of the old edge length.
const OUTLINE_SNAP_FRACTION: f64 = 0.4;

/// The outline of the region under layout `q` (empty for a closed mesh).
pub(super) fn outline_for(s: &Session, q: &QuadObject) -> CmdResult<Outline> {
    let m =
        s.doc.mesh(q.mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {}", q.mesh)))?;
    let sub = match q.region {
        Some(r) => {
            let tris =
                s.doc.region(r).map(|r| r.triangles.to_vec()).ok_or_else(|| CmdError::Params("no region".into()))?;
            m.submesh(&tris)
        }
        None => (*m).clone(),
    };
    Ok(cleaned_outline(&sub).0)
}

/// One split of the layout with its new free-boundary vertices put back onto the outline, accepted
/// only if no proper quad turns improper on the real (projected) geometry.
pub(super) fn split_conformed(layout: &QuadMesh, scan: &ScanSurface, outline: &Outline, radius: f64) -> CheckedSplit {
    let edge = layout_edge(layout);
    checked_split(layout, &projector(scan, radius), outline, (edge, OUTLINE_SNAP_FRACTION * edge))
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
    QuadInfo {
        id: q.id,
        nu: q.nu,
        nv: q.nv,
        creases: (q.crease_u.clone(), q.crease_v.clone()),
        snap_max,
        repaired_patches: None,
        flipped_patches: None,
        unverified_patches: None,
        outline_loops_unsnapped: None,
        outline_unsnapped_reason: None,
        split_rejected: None,
        improper_before: None,
        improper_after: None,
        snap_rejected: None,
    }
}

pub(super) fn split_unstructured(s: &mut Session, id: ObjectId, conform: bool) -> CmdResult<QuadInfo> {
    let q = s.doc.quad(id).cloned().ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    check_layout(&q)?;
    if q.faces.len() * 4 > MAX_LAYOUT_QUADS {
        return Err(CmdError::Params(format!("the layout would exceed {MAX_LAYOUT_QUADS} quads")));
    }
    let (pts, point_normals, spacing) = scan_for(s, &q)?;
    let scan = ScanSurface::oriented(&pts, &point_normals);
    let layout = layout_of(&q);
    let outline = if conform { outline_for(s, &q)? } else { Outline::default() };
    let step = split_conformed(&layout, &scan, &outline, layout_radius(&layout, spacing));
    let (resnap, rejected) = (step.report, step.rejected);
    let mut split = step.layout;
    let flips = if rejected {
        FlipReport::default() // the layout is as it was: nothing to check again
    } else {
        orientation_check(&mut split, &scan, (&pts, &point_normals), spacing)
    };
    let snap_max = {
        let bvh_pts = ScanSurface::new(&pts);
        split.vertices.iter().filter_map(|p| bvh_pts.nearest(p)).map(|x| x.1).fold(0.0, f64::max)
    };
    if let (false, Some(m)) = (rejected, s.doc.quad_mut(id)) {
        m.vertices = split.vertices.iter().map(|p| [p.x, p.y, p.z]).collect();
        m.normals = split.normals.iter().map(|n| [n.x, n.y, n.z]).collect();
        m.faces = split.faces;
    }
    let q = s.doc.quad(id).ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    Ok(QuadInfo {
        repaired_patches: Some(flips.repaired),
        flipped_patches: Some(flips.failed),
        unverified_patches: Some(flips.unverified),
        outline_loops_unsnapped: Some(resnap.unsnapped()),
        improper_before: Some(step.improper_before),
        improper_after: Some(step.improper_after),
        outline_unsnapped_reason: resnap.reason.map(str::to_string),
        split_rejected: Some(rejected),
        ..info_of(q, snap_max)
    })
}

/// Refinements of an AutoSurface layout allowed before it is accepted as it is.
const MAX_AUTO_REFINEMENTS: usize = 3;

/// The scan an AutoSurface layout is fitted to, with the lookup tree for the reference normals.
struct AutoContext<'a> {
    q: &'a QuadObject,
    i: &'a AutoIn,
    tol: f64,
    max_patches: usize,
    scan: LayoutScan,
    tree: KdTree,
    outline: Outline,
}

/// The patches of a layout after repair, with their deviation from the scan.
struct Measured {
    radius: f64,
    set: PatchSet,
    dev: Deviation,
    repaired: usize,
    unverified: usize,
    /// Patches still flipped after the repair (0: otherwise the fit fails).
    flipped: usize,
}

/// Facts about the accepted fit that the report needs besides the surface.
struct Accepted {
    closed: bool,
    signed_volume: Option<f64>,
    refinements: usize,
    repaired: usize,
    unsnapped: usize,
    splits_rejected: usize,
    folded: usize,
    flipped_patches: usize,
}

/// The scan normal at a point, the reference of the orientation checks. It is reversed when the
/// layout has been turned around as a whole.
fn scan_reference<'a>(
    tree: &'a KdTree,
    normals: &'a [Vec3],
    flipped: bool,
) -> impl Fn(&Point3, &Vec3) -> Option<Vec3> + 'a {
    let sign = if flipped { -1.0 } else { 1.0 };
    move |p: &Point3, hint: &Vec3| same_side_normal(tree, normals, sign, p, hint)
}

/// Turns the patches of `layout` that run against the scan around, and measures the fit.
fn repair_and_measure(layout: &mut QuadMesh, scan: &LayoutScan, tree: &KdTree, flipped: bool) -> CmdResult<Measured> {
    let radius = layout_radius(layout, scan.spacing);
    let reference = scan_reference(tree, &scan.point_normals, flipped);
    let (set, report) = repair_patches(layout, &scan.scan, radius, &reference);
    if report.failed > 0 {
        return Err(CmdError::Failed(format!(
            "{} patches stay flipped against the scan after repair; retopologise again (fewer quads) or clean the region",
            report.failed
        )));
    }
    if set.nets.len() != layout.faces.len() {
        return Err(CmdError::Failed("the layout has quads with missing edges".into()));
    }
    let dev = deviation(&set, &scan.pts, 12);
    Ok(Measured { radius, set, dev, repaired: report.repaired, unverified: report.unverified, flipped: report.failed })
}

/// Turns every quad of `layout` around, together with its normal.
fn reverse_layout(layout: &mut QuadMesh) {
    for f in &mut layout.faces {
        f.swap(1, 3);
    }
    for n in &mut layout.normals {
        *n = -*n;
    }
}

/// The control nets of the patches of `set`, as the kernel builds them.
fn bezier_nets(set: &PatchSet) -> Vec<Vec<Vec<[f64; 3]>>> {
    set.nets.iter().map(|n| n.iter().map(|r| r.to_vec()).collect()).collect()
}

/// Adds the surface of an accepted fit (and its solid when closed) and returns their ids and the
/// number of patches.
fn store_auto_surface(
    s: &mut Session,
    q: &QuadObject,
    i: &AutoIn,
    sheet: Brep,
    dev: &Deviation,
    closed: bool,
) -> (ObjectId, Option<ObjectId>, usize) {
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
    (sid, solid, patches)
}

/// The AutoSurface report of an accepted fit; stores its surface first.
fn finish_auto(s: &mut Session, ctx: &AutoContext<'_>, sheet: Brep, step: &Measured, accepted: &Accepted) -> AutoOut {
    let jumps = edge_normal_jumps(&step.set);
    let jmax = jumps.iter().copied().fold(0.0, f64::max).to_degrees();
    let jmean = (jumps.iter().sum::<f64>() / jumps.len().max(1) as f64).to_degrees();
    let within = step.dev.max <= ctx.tol;
    let mut notes: Vec<String> = Vec::new();
    if !within {
        notes.push(format!(
            "max deviation {:.4} mm exceeds {} mm after {} refinements; raise max_patches or retopologise finer",
            step.dev.max, ctx.tol, accepted.refinements
        ));
    }
    if step.unverified > 0 {
        notes.push(format!(
            "{} patches could not be checked against the scan orientation (no scan samples on their side); inspect them",
            step.unverified
        ));
    }
    if accepted.splits_rejected > 0 {
        notes.push(
            "a split of the refinement would have broken a proper quad and was rejected; the layout was not refined further"
                .to_string(),
        );
    }
    if accepted.unsnapped > 0 {
        notes.push(format!(
            "{} boundary loop splits could not be snapped back onto the region outline (kept as they were)",
            accepted.unsnapped
        ));
    }
    let warning = (!notes.is_empty()).then(|| notes.join("; "));
    let (sid, solid, patches) = store_auto_surface(s, ctx.q, ctx.i, sheet, &step.dev, accepted.closed);
    AutoOut {
        blocks: vec![AutoBlock { id: sid, patches, rms: step.dev.rms, max: step.dev.max }],
        max: step.dev.max,
        within_tolerance: within,
        warning,
        normal_jump_max_deg: Some(jmax),
        normal_jump_mean_deg: Some(jmean),
        closed: Some(accepted.closed),
        volume: accepted.signed_volume.map(f64::abs),
        solid,
        refinements: Some(accepted.refinements),
        uncovered: Some(step.dev.uncovered),
        folded: Some(accepted.folded),
        repaired_patches: Some(accepted.repaired),
        flipped_patches: Some(accepted.flipped_patches),
        unverified_patches: Some(step.unverified),
        outline_loops_unsnapped: Some(accepted.unsnapped),
        splits_rejected: Some(accepted.splits_rejected),
    }
}

/// Refines `layout` until its patches are within the tolerance or it can be split no further,
/// then stores the surface.
fn refine_and_fit(s: &mut Session, ctx: &AutoContext<'_>, mut layout: QuadMesh) -> CmdResult<AutoOut> {
    let mut refinements = 0usize;
    let mut flipped = false;
    let mut repaired = 0usize;
    let mut unsnapped = 0usize;
    let mut splits_rejected = 0usize;
    loop {
        let step = repair_and_measure(&mut layout, &ctx.scan, &ctx.tree, flipped)?;
        repaired += step.repaired;
        let can_split =
            layout.faces.len() * 4 <= ctx.max_patches && refinements < MAX_AUTO_REFINEMENTS && splits_rejected == 0;
        if step.dev.max > ctx.tol && can_split {
            let split = split_conformed(&layout, &ctx.scan.scan, &ctx.outline, step.radius);
            if split.rejected {
                // a rejected split stops the refinement: the layout stays as it is
                splits_rejected += 1;
                continue;
            }
            unsnapped += split.report.unsnapped();
            layout = split.layout;
            refinements += 1;
            continue;
        }
        let sheet = sheet_from_bezier_network(&bezier_nets(&step.set), &step.set.corners)?;
        let closed = is_closed(&sheet);
        let signed_volume = if closed { surfacecraft_kernel::solid::volume(&sheet).ok() } else { None };
        if signed_volume.is_some_and(|v| v < 0.0) && !flipped {
            // the layout runs against its normals: turn every quad around and build again
            reverse_layout(&mut layout);
            flipped = true;
            continue;
        }
        let accepted = Accepted {
            closed,
            signed_volume,
            refinements,
            repaired,
            unsnapped,
            splits_rejected,
            folded: layout.folded(),
            flipped_patches: step.flipped,
        };
        return Ok(finish_auto(s, ctx, sheet, &step, &accepted));
    }
}

/// AutoSurface of an unstructured layout: bicubic patches, refined until within `tol`.
pub(super) fn auto_unstructured(s: &mut Session, q: &QuadObject, tol: f64, i: &AutoIn) -> CmdResult<AutoOut> {
    check_layout(q)?;
    let (pts, point_normals, spacing) = scan_for(s, q)?;
    if pts.len() < 20 {
        return Err(CmdError::Failed("the region has too few points".into()));
    }
    let max_patches = i.max_patches.unwrap_or(6000).clamp(1, MAX_LAYOUT_QUADS);
    let layout = layout_of(q);
    if layout.faces.len() > max_patches {
        return Err(CmdError::Params(format!("the layout has more than max_patches = {max_patches} quads")));
    }
    let scan = LayoutScan::from_parts(pts, point_normals, spacing);
    let tree = KdTree::build(&scan.pts);
    let outline = if i.conform_boundary.unwrap_or(false) { outline_for(s, q)? } else { Outline::default() };
    let ctx = AutoContext { q, i, tol, max_patches, scan, tree, outline };
    refine_and_fit(s, &ctx, layout)
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_fit::retopo::{ConformReport, Layout};

    /// A layout of `good` proper unit quads and `bad` reversed ones.
    fn layout(good: usize, bad: usize, conformed: usize) -> FieldLayout {
        let mut quads = QuadMesh::default();
        for k in 0..good + bad {
            let o = 3.0 * k as f64;
            let mut q = [[o, 0.0], [o + 1.0, 0.0], [o + 1.0, 1.0], [o, 1.0]];
            if k >= good {
                q.reverse();
            }
            let base = quads.vertices.len() as u32;
            for p in q {
                quads.vertices.push(Point3::new(p[0], p[1], 0.0));
                quads.normals.push(Vec3::z());
            }
            quads.faces.push([base, base + 1, base + 2, base + 3]);
        }
        let boundary = ConformReport { loops: 1, conformed, ..ConformReport::default() };
        let rep = RetopoReport {
            layout: Layout::Field,
            edge_length: 1.0,
            work_triangles: 0,
            quads: good + bad,
            other_polygons: 0,
            irregular_fraction: 0.0,
            euler_ok: true,
            note: None,
            boundary,
        };
        FieldLayout {
            quads,
            rep,
            field_faces: good + bad,
            field_quads: good + bad,
            folded: 0,
            flips: FlipReport::default(),
            split_defects: 0,
        }
    }

    /// Conforming left one improper quad and the plain layout has one too (a tie): the plain layout
    /// is returned, with the reason; no counts are compared.
    #[test]
    fn a_tie_of_improper_quads_returns_the_plain_layout() {
        let (out, _, reason) = net_conformed((layout(3, 1, 1), 0), || Ok((layout(3, 1, 0), 0))).unwrap();
        assert_eq!(reason, Some(NET_REASON));
        assert_eq!(out.rep.boundary.conformed, 0);
        assert!(!out.rep.boundary.all_conformed());
    }

    /// No improper quad: the conformed layout is kept and the plain pipeline is not even run.
    #[test]
    fn a_conformed_layout_without_improper_quads_is_kept() {
        let plain = || -> CmdResult<(FieldLayout, usize)> { Err(CmdError::Failed("must not run".into())) };
        let (out, _, reason) = net_conformed((layout(4, 0, 1), 0), plain).unwrap();
        assert_eq!(reason, None);
        assert!(out.rep.boundary.all_conformed());
    }
}

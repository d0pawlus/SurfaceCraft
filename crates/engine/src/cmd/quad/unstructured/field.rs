//! Layout attempts: the field layout projected onto the scan, the orientation fallback, and the final net that keeps a conformed layout only when it is proper.
use super::super::{CmdError, CmdResult};
use super::layout::*;
use super::*;
use crate::Session;
use surfacecraft_doc::{ObjectId, QuadObject};
use surfacecraft_fit::mls::ScanSurface;
use surfacecraft_fit::orient::FlipReport;
use surfacecraft_fit::retopo::{QuadMesh, RetopoOptions, RetopoReport, invalid_quads, retopo};
use surfacecraft_fit::segment::region_points;
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3};

/// The scan a layout is fitted to: its points with their oriented normals, and the sampling distance.
pub(super) struct LayoutScan {
    pub(super) pts: Vec<Point3>,
    pub(super) point_normals: Vec<Vec3>,
    pub(super) scan: ScanSurface,
    pub(super) spacing: f64,
}

impl LayoutScan {
    pub(super) fn from_parts(pts: Vec<Point3>, point_normals: Vec<Vec3>, spacing: f64) -> Self {
        let scan = ScanSurface::oriented(&pts, &point_normals);
        Self { pts, point_normals, scan, spacing }
    }

    /// The scan of the triangles `tris` of `m`.
    pub(super) fn of_region(m: &Mesh, tris: &[u32]) -> Self {
        let normals = m.vertex_normals();
        let (pts, point_normals) = region_points(m, &normals, tris);
        Self::from_parts(pts, point_normals, mean_edge(m, tris))
    }
}

/// One layout attempt: field extraction (or the decimation fallback when `force_fallback`),
/// projected onto the scan, relaxed, and checked for folded and flipped quads.
pub(super) struct FieldLayout {
    pub(super) quads: QuadMesh,
    pub(super) rep: RetopoReport,
    pub(super) field_faces: usize,
    pub(super) field_quads: usize,
    pub(super) folded: usize,
    pub(super) flips: FlipReport,
    /// Quads of the split of the field polygons that are improper although their parent polygon is proper.
    pub(super) split_defects: usize,
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
pub(super) fn interior_irregularity(quads: &QuadMesh, boundary: &[(u32, u32)]) -> f64 {
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
pub(super) fn layout_snap_max(quads: &QuadMesh, sub: &Mesh) -> f64 {
    let bvh = Bvh::build(sub);
    quads.vertices.iter().filter_map(|p| bvh.closest(p, f64::INFINITY)).map(|h| h.distance).fold(0.0, f64::max)
}

/// Stores `quads` as a new quad object of `mesh` (and `region`) and returns its id.
pub(super) fn add_retopo_layout(
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
pub(super) const NET_REASON: &str =
    "the conformed layout has an improper quad; the layout without conforming is returned";

/// The final net: a conformed layout is accepted only if every quad of its final geometry (after
/// all splits, projections and relaxations) is proper. Otherwise `plain` (the same pipeline with
/// conforming off) is returned and the reason says why. No counts are compared.
pub(super) fn net_conformed(
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
pub(super) fn netted_layout(
    sub: &Mesh,
    scan: &LayoutScan,
    opts: &RetopoOptions,
) -> CmdResult<(FieldLayout, usize, Option<&'static str>)> {
    let conformed = settled_field_layout(sub, scan, opts)?;
    net_conformed(conformed, || {
        settled_field_layout(sub, scan, &RetopoOptions { conform_boundary: false, ..opts.clone() })
    })
}

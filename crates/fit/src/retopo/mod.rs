//! Unstructured, field-aligned retopology of a triangle mesh into a quad-dominant layout
//! (design doc §6.3 and §10, milestone P4): wrapped and closed regions the height-field
//! layout of M4 cannot cover.
//!
//! The method is that of Jakob, Tarini, Panozzo and Sorkine-Hornung, "Instant Field-Aligned
//! Meshes" (SIGGRAPH Asia 2015; reference implementation "Instant Meshes", permissive licence):
//!
//! 1. an *orientation field* (4-RoSy: a tangent direction per vertex defined up to 90° turns),
//!    smoothed by Gauss–Seidel sweeps over the one-ring with the best 90° match per edge;
//! 2. a *position field* (2-PoSy: a lattice point of spacing ρ per vertex, aligned with the
//!    orientation) smoothed the same way: every neighbour proposes the point of its own lattice
//!    closest to the shared edge's middle, the average is rounded to the lattice point nearest to
//!    the vertex;
//! 3. extraction: mesh vertices whose lattice points coincide collapse into one layout vertex,
//!    lattice neighbours become layout edges and the faces are the cycles of the resulting
//!    embedded graph.
//!
//! This is written from the paper, not from the reference source (no code was copied; the
//! reference and its licence are credited in ATTRIBUTION.md and NOTICE). Differences from the
//! reference: a coarse-to-fine hierarchy built with quadric decimation instead of its graph
//! coarsening; single-threaded; extraction by cell adjacency (no edge-tag tracing and no
//! post-clean-up beyond pruning dangling vertices); open-boundary alignment by pinning the
//! orientation to the boundary tangent and the lattice to pass through boundary vertices.
//! Where the field cannot give a clean polygon mesh, [`retopo`] falls back to subdividing a
//! coarse decimation (every triangle into three quads) and says so in the report.
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3};

mod checked;
mod conform;
mod extract;
mod field;
mod polymesh;
#[cfg(test)]
mod tests;
mod util;

pub use checked::{CheckedSplit, checked_split};
pub use conform::{ConformReport, conform_to_outline, improper_quads, invalid_quads, snap_to_outline, split_defects};
use extract::extract;
use field::{Level, init_orientation, init_positions, level_of, prolong, smooth_orientation, smooth_positions};
use util::{
    build_levels, conform_layout, euler_of, euler_of_mesh, fallback, irregular_fraction, is_closed, working_mesh,
};

/// Largest polygon the extraction keeps.
pub(super) const MAX_POLY: usize = 8;
/// Most triangles of the working mesh.
pub(super) const MAX_WORK_TRIANGLES: usize = 120_000;

/// A polygon mesh with per-vertex unit normals; faces run counter-clockwise seen from the side
/// the normals point to.
#[derive(Clone, Debug, Default)]
pub struct PolyMesh {
    /// Vertices.
    pub vertices: Vec<Point3>,
    /// Unit normals.
    pub normals: Vec<Vec3>,
    /// Faces (3 or more vertices).
    pub faces: Vec<Vec<u32>>,
}

/// An all-quad mesh.
#[derive(Clone, Debug, Default)]
pub struct QuadMesh {
    /// Vertices.
    pub vertices: Vec<Point3>,
    /// Unit normals.
    pub normals: Vec<Vec3>,
    /// Quads, counter-clockwise seen from the normals' side.
    pub faces: Vec<[u32; 4]>,
}

/// How the layout was obtained.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Layout {
    /// Field-aligned extraction.
    Field,
    /// Coarse decimation split into quads (the field gave no clean layout).
    Fallback,
}

/// Retopology parameters.
#[derive(Clone, Debug)]
pub struct RetopoOptions {
    /// Approximate number of faces wanted.
    pub target_faces: usize,
    /// Smoothing sweeps of the orientation field per level.
    pub orientation_sweeps: usize,
    /// Smoothing sweeps of the position field per level.
    pub position_sweeps: usize,
    /// Skip the fields and use the fallback layout (decimation split into quads).
    pub force_fallback: bool,
    /// Conform the layout of an open region to its outline (default true; the engine turns it off
    /// when conforming would leave an improper quad).
    pub conform_boundary: bool,
}

impl Default for RetopoOptions {
    fn default() -> Self {
        Self {
            target_faces: 200,
            orientation_sweeps: 40,
            position_sweeps: 40,
            force_fallback: false,
            conform_boundary: true,
        }
    }
}

/// What [`retopo`] did.
#[derive(Clone, Debug)]
pub struct RetopoReport {
    /// Field extraction or fallback.
    pub layout: Layout,
    /// Edge length ρ asked of the field (mm).
    pub edge_length: f64,
    /// Triangles of the working mesh the fields were solved on.
    pub work_triangles: usize,
    /// Quads extracted (before any splitting of other polygons).
    pub quads: usize,
    /// Triangles, pentagons … extracted.
    pub other_polygons: usize,
    /// Share of vertices whose valence is not 4 (interior vertices only).
    pub irregular_fraction: f64,
    /// Does the Euler characteristic equal that of the input (closed inputs; true otherwise)?
    pub euler_ok: bool,
    /// Why the fallback was used.
    pub note: Option<String>,
    /// How the layout was conformed to the outline of an open region.
    pub boundary: ConformReport,
}

/// Retopologises `mesh` into a quad-dominant polygon mesh of about `opts.target_faces` faces.
///
/// # Errors
/// An error message when the mesh is empty or invalid.
pub fn retopo(mesh: &Mesh, opts: &RetopoOptions) -> Result<(PolyMesh, RetopoReport), String> {
    let (mut poly, mut report) = lay_out(mesh, opts)?;
    if opts.conform_boundary {
        report.boundary = conform_layout(mesh, &mut poly, report.edge_length);
    }
    Ok((poly, report))
}

/// The layout of the fields (or the fallback), before it is conformed to the region outline.
fn lay_out(mesh: &Mesh, opts: &RetopoOptions) -> Result<(PolyMesh, RetopoReport), String> {
    mesh.validate().map_err(|e| e.to_string())?;
    if mesh.triangle_count() < 4 {
        return Err("the mesh has too few triangles".into());
    }
    let target = opts.target_faces.clamp(8, 100_000);
    let area = mesh.area();
    if !(area.is_finite() && area > 0.0) {
        return Err("the mesh has no area".into());
    }
    let rho = (area / target as f64).sqrt();
    let work = working_mesh(mesh, rho, target);
    if work.triangle_count() < 4 {
        return Err("the mesh collapsed while preparing it".into());
    }
    let closed = is_closed(&work);
    let chi = euler_of_mesh(&work);
    let bvh = Bvh::build(&work);
    if opts.force_fallback {
        let f = fallback(&work, target, &bvh);
        let report = RetopoReport {
            layout: Layout::Fallback,
            edge_length: rho,
            work_triangles: work.triangle_count(),
            quads: 0,
            other_polygons: f.faces.len(),
            irregular_fraction: irregular_fraction(&f),
            euler_ok: !closed || euler_of(&f) == chi,
            note: Some("fallback layout requested".into()),
            boundary: ConformReport::default(),
        };
        return Ok((f, report));
    }
    let meshes = build_levels(&work);
    let levels: Vec<Level> = meshes.iter().map(level_of).collect();
    // solve coarse to fine
    let mut o: Vec<Vec3> = Vec::new();
    let mut p: Vec<Vec3> = Vec::new();
    for k in (0..levels.len()).rev() {
        let lv = &levels[k];
        if k == levels.len() - 1 {
            o = init_orientation(lv);
            smooth_orientation(lv, &mut o, opts.orientation_sweeps * 4);
            p = init_positions(lv, &o, rho);
            smooth_positions(lv, &o, &mut p, rho, opts.position_sweeps * 4);
        } else {
            let (no, np) = prolong(&levels[k + 1], &o, &p, lv, rho);
            o = no;
            p = np;
            let fine = if k == 0 { 3 } else { 1 };
            smooth_orientation(lv, &mut o, opts.orientation_sweeps / fine);
            smooth_positions(lv, &o, &mut p, rho, opts.position_sweeps / fine);
        }
    }
    let finest = levels.first().ok_or("no level")?;
    let mut report = RetopoReport {
        layout: Layout::Field,
        edge_length: rho,
        work_triangles: work.triangle_count(),
        quads: 0,
        other_polygons: 0,
        irregular_fraction: 0.0,
        euler_ok: true,
        note: None,
        boundary: ConformReport::default(),
    };
    let extracted = extract(finest, &o, &p, rho, &bvh);
    let mut chosen: Option<PolyMesh> = None;
    let mut why = String::new();
    match extracted {
        Some(poly) => {
            let quads = poly.faces.iter().filter(|f| f.len() == 4).count();
            let share = quads as f64 / poly.faces.len().max(1) as f64;
            let euler_ok = !closed || euler_of(&poly) == chi;
            let size_ok = poly.faces.len() * 3 > target && poly.faces.len() < target * 3;
            if share >= 0.5 && euler_ok && size_ok {
                report.quads = quads;
                report.other_polygons = poly.faces.len() - quads;
                report.irregular_fraction = irregular_fraction(&poly);
                chosen = Some(poly);
            } else {
                why = format!(
                    "field extraction gave {} faces, {:.0} % quads, Euler {}",
                    poly.faces.len(),
                    share * 100.0,
                    if euler_ok { "ok" } else { "wrong" }
                );
            }
        }
        None => why = "field extraction found no faces".into(),
    }
    let poly = match chosen {
        Some(p) => p,
        None => {
            report.layout = Layout::Fallback;
            let f = fallback(&work, target, &bvh);
            report.quads = 0;
            report.other_polygons = f.faces.len();
            report.irregular_fraction = irregular_fraction(&f);
            report.euler_ok = !closed || euler_of(&f) == chi;
            report.note = Some(why);
            f
        }
    };
    Ok((poly, report))
}

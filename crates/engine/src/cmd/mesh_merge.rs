//! `mesh.merge`: aligned scans of one part joined into one mesh (`geom::merge`). Where scans
//! overlap one layer is kept (the scan in which the surface lies farthest from its own border);
//! the seam between the kept regions is closed by ball pivoting on surface the scans saw. The
//! scans stay as they are; the new mesh is one undo step. Scans that disagree in their overlap by
//! more than the tolerance, or whose normals oppose, are refused with the numbers (no mesh).
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::Mesh;
use surfacecraft_geom::merge::{Issue, MergeError, Params, Report, Stage, Topology, merge};

/// Wall-clock time since a stage started; zero where the clock is not available (the browser build).
struct Stopwatch(#[cfg(not(target_arch = "wasm32"))] std::time::Instant);

impl Stopwatch {
    fn start() -> Self {
        #[cfg(not(target_arch = "wasm32"))]
        {
            Self(std::time::Instant::now())
        }
        #[cfg(target_arch = "wasm32")]
        {
            Self()
        }
    }

    fn ms(&self) -> u64 {
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.0.elapsed().as_millis() as u64
        }
        #[cfg(target_arch = "wasm32")]
        {
            0
        }
    }
}

/// Scans merged at most in one call.
pub const MAX_OBJECTS: usize = 64;
/// The default tolerance: this many standard deviations of the noise of the two noisiest scans
/// together (two independent noisy copies of one surface differ by √2 σ) ...
pub const NOISE_SIGMAS: f64 = 4.0;
/// ... plus this share of the median edge of the coarsest scan (two tessellations of one curved
/// surface differ by a part of an edge even without noise).
pub const CHORD_SHARE: f64 = 0.05;

/// Parameters of `mesh.merge`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MergeIn {
    /// Scan meshes to merge (ids or names, 2 to 64, each once), already aligned in one frame (`align.global`).
    pub objects: Vec<ObjRef>,
    /// Surface of two scans closer than this is the same surface (mm). Default: 4 standard deviations of the noise of the two noisiest scans together plus 5 % of the median edge of the coarsest scan.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Name of the new mesh (default "merged").
    #[serde(default)]
    pub name: Option<String>,
}

/// One scan in the merge.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct MergeScanOut {
    /// The mesh.
    pub object: ObjectId,
    /// Noise estimate of the scan (σ, mm; 0 when it could not be measured; absent when the tolerance was given and no estimate was made).
    pub noise: Option<f64>,
    /// Triangles of the scan.
    pub triangles: usize,
    /// Kept in the result as they were.
    pub kept: usize,
    /// Removed (another scan keeps that surface), including the triangles left out (`dropped`).
    pub removed: usize,
    /// Removed, then put back because the result had all their corners at exactly their places (pieces of one scan).
    pub restored: usize,
    /// Triangles left out: exact duplicates, on an edge of three or more triangles, or wound against the rest of their piece.
    pub dropped: usize,
    /// Share of the scan's area within the tolerance of facing surface of another scan (0..1).
    pub overlap_share: f64,
    /// Area of the scan and the area removed (mm²).
    pub area: f64,
    /// Area removed (mm²).
    pub removed_area: f64,
    /// Median edge of the scan (mm).
    pub median_edge: f64,
}

/// How two scans agree where one lies on the other.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct MergePairOut {
    /// The scan sampled (index into `objects`).
    pub a: usize,
    /// The scan measured against.
    pub b: usize,
    /// Points sampled over `a` by area.
    pub samples: usize,
    /// Of them, on facing surface of `b` (not past its border) within the search reach.
    pub shared: usize,
    /// Points with no surface of `b` within the reach that lie over facing surface of `b` farther away (along their normal, within the size of `b`): `b` is there, displaced.
    pub over: usize,
    /// Of those, within the tolerance.
    pub within: usize,
    /// Points on opposed surface of `b` within the tolerance (one of the two is turned inside out).
    pub opposed: usize,
    /// Root mean square of the distances of the shared points (mm).
    pub rms: f64,
    /// 95th percentile of those distances (mm).
    pub p95: f64,
    /// Largest of those distances (mm; near the borders of the scans it can reach the search reach).
    pub worst: f64,
    /// Share of the shared points farther than the tolerance.
    pub beyond_share: f64,
}

/// What the seam step did.
#[derive(Clone, Debug, Default, Deserialize, Serialize, JsonSchema)]
pub struct MergeSeamOut {
    /// Border edges the removal cut, both sides of the seam.
    pub cut_edges: usize,
    /// Length of the seam (mm): half the length of the cut edges that were closed.
    pub length: f64,
    /// Ball radii of the pivoting (mm).
    pub radii: Vec<f64>,
    /// Triangles created to close the seam (pivoting and small holes).
    pub triangles_created: usize,
    /// Removed triangles put back (their corners were all in the result).
    pub restored: usize,
    /// Small holes of the seam filled.
    pub holes_filled: usize,
    /// Candidate triangles refused: not on surface any scan saw.
    pub refused_unsupported: usize,
    /// Candidate triangles refused: they crossed a triangle there.
    pub refused_crossing: usize,
    /// Seam triangles taken back because they met another fan at one vertex only, before the hole was filled again.
    pub unpinched: usize,
    /// Removed triangles whose centre the seam left without result surface within the tolerance.
    pub uncovered_after_seam: usize,
    /// Of them, put back on their own scan's vertices (nothing moves, no scans are joined by them).
    pub recovered: usize,
    /// Triangles of small stray pieces dropped because they lay wholly on the rest of the result.
    pub strays_dropped: usize,
    /// Cut edges left open.
    pub open_edges: usize,
    /// Their length (mm).
    pub open_length: f64,
}

/// Pieces, boundary loops and genus.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, JsonSchema)]
pub struct MergeTopologyOut {
    /// Connected pieces.
    pub pieces: usize,
    /// Boundary loops (the rims of open pieces and their holes).
    pub loops: usize,
    /// Genus (handles), sum over the pieces; absent when a piece has none (pinched, non-manifold, not oriented).
    pub genus: Option<usize>,
    /// Edges of three or more triangles.
    pub non_manifold_edges: usize,
    /// Edges whose two triangles run the same way (inconsistent orientation).
    pub unoriented_edges: usize,
    /// Vertices whose triangles form more than one fan.
    pub pinched_vertices: usize,
}

/// The honesty checks, measured on the result.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, JsonSchema)]
pub struct MergeChecksOut {
    /// Points sampled over the removed triangles by area (not only their corners).
    pub removed_samples: usize,
    /// Area removed (mm²).
    pub removed_area: f64,
    /// Share of the removed area within the tolerance of facing surface of the result (1 when nothing was removed).
    pub removed_within_share: f64,
    /// Largest distance of a removed sample from the result (mm).
    pub removed_worst: f64,
    /// Vertices of the result; every one is a vertex of a scan.
    pub kept_vertices: usize,
    /// Of them, moved by a weld of coincident seam vertices.
    pub moved_vertices: usize,
    /// Largest move (mm; at most a quarter of the tolerance and a tenth of the median edge).
    pub max_vertex_move: f64,
    /// Triangles the merge added (seam, restored) that cross another triangle of the result.
    pub added_crossings: usize,
    /// Vertices of the result closer than 0.01 median edges to a vertex of another scan with no edge between them (copies of one point left unwelded).
    pub unwelded_vertices: usize,
    /// Share of the area of the scans' triangles in the result whose centre lies within the tolerance of facing surface of another scan's triangles: two layers left where the scans overlapped (near 0 when the overlaps were removed).
    pub stacked_share: f64,
}

/// Something to know, with its numbers.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct MergeIssueOut {
    /// `scans_misaligned`, `opposed_normals`, `inconsistent_winding`, `inverted_scan` (these refuse), `non_manifold_input`, `flipped_triangles_dropped`, `inner_skins`, `scans_disjoint`, `scan_fully_covered`, `seam_open`, `removed_not_covered`, `layers_stacked`, `unwelded_vertices`, `unoriented_edges`, `pinched_vertices`.
    pub code: String,
    /// `refusal`, `warning` or `info`.
    pub severity: String,
    /// The scans concerned (indices into `objects`).
    pub scans: Vec<usize>,
    /// What happened, with the numbers.
    pub message: String,
}

/// Time of one stage.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct MergeStageOut {
    /// `prepare`, `overlap`, `ownership`, `union`, `seam` or `check`.
    pub stage: String,
    /// Milliseconds (0 in the browser).
    pub ms: u64,
}

/// Result of `mesh.merge`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct MergeOut {
    /// The new mesh; absent when the merge refused.
    pub id: Option<ObjectId>,
    /// The merge refused (see the issues of severity `refusal`); nothing was made.
    pub refused: bool,
    /// Tolerance used (mm).
    pub tolerance: f64,
    /// `given` or `noise` (the default).
    pub tolerance_source: String,
    /// How far apart two scans may lie and still be compared as one surface (mm, 8 tolerances).
    pub reach: f64,
    /// The scans, in the order of `objects`.
    pub scans: Vec<MergeScanOut>,
    /// Every ordered pair of scans with surface near each other.
    pub pairs: Vec<MergePairOut>,
    /// Redundant triangles eaten back after the ownership.
    pub eaten: usize,
    /// The seam.
    pub seam: MergeSeamOut,
    /// Topology of the scans together (sums).
    pub before: MergeTopologyOut,
    /// Topology of the result.
    pub after: MergeTopologyOut,
    /// The honesty checks.
    pub checks: MergeChecksOut,
    /// Issues; those of severity `refusal` stopped the merge.
    pub issues: Vec<MergeIssueOut>,
    /// Triangles of the result.
    pub triangles: usize,
    /// Stage times.
    pub stages: Vec<MergeStageOut>,
    /// In words.
    pub note: String,
}

fn topology_out(t: &Topology) -> MergeTopologyOut {
    MergeTopologyOut {
        pieces: t.pieces,
        loops: t.loops,
        genus: t.genus,
        non_manifold_edges: t.non_manifold_edges,
        unoriented_edges: t.unoriented_edges,
        pinched_vertices: t.pinched_vertices,
    }
}

fn issue(code: &str, severity: &str, scans: Vec<usize>, message: String) -> MergeIssueOut {
    MergeIssueOut { code: code.into(), severity: severity.into(), scans, message }
}

/// The issue in the form of the command.
fn issue_out(i: &Issue) -> MergeIssueOut {
    refusal_out(i).or_else(|| scan_notice(i)).unwrap_or_else(|| notice_out(i))
}

/// An issue that refuses, in the form of the command.
fn refusal_out(i: &Issue) -> Option<MergeIssueOut> {
    Some(match i {
        Issue::Misaligned { a, b, near, over, beyond_share, rms, p95 } => issue(
            "scans_misaligned",
            "refusal",
            vec![*a, *b],
            format!(
                "scans {a} and {b} disagree where they overlap: {:.1} % of {near} shared points lie farther apart than the tolerance (rms {rms:.4} mm, 95th percentile {p95:.4} mm), and {over} points lie over the other scan farther away than 8 tolerances; they are not averaged or stacked: align them better (align.icp, align.global) or give a larger tolerance",
                100.0 * beyond_share
            ),
        ),
        Issue::Opposed { a, b, opposed, facing } => issue(
            "opposed_normals",
            "refusal",
            vec![*a, *b],
            format!(
                "the normals of scans {a} and {b} oppose where they meet ({opposed} points on opposed surface, {facing} on facing surface): one of them is turned inside out; orient it outward and merge again"
            ),
        ),
        Issue::Winding { scan, flipped, share } => issue(
            "inconsistent_winding",
            "refusal",
            vec![*scan],
            format!(
                "{flipped} triangles of scan {scan} ({:.2} % of its area) are wound against the rest of their piece: a patch is turned inside out; repair its orientation first",
                100.0 * share
            ),
        ),
        Issue::Inverted { scan, pieces } => issue(
            "inverted_scan",
            "refusal",
            vec![*scan],
            format!(
                "{pieces} closed piece(s) of scan {scan} have a negative volume: turned inside out; orient it outward and merge again"
            ),
        ),
        _ => return None,
    })
}

/// An issue about the scans given that does not refuse, in the form of the command.
fn scan_notice(i: &Issue) -> Option<MergeIssueOut> {
    Some(match i {
        Issue::NonManifold { scan, dropped } => issue(
            "non_manifold_input",
            "warning",
            vec![*scan],
            format!(
                "{dropped} triangles of scan {scan} are exact duplicates or lie on edges of three or more triangles and were left out (they count as removed in the checks)"
            ),
        ),
        Issue::InnerSkins { scan, skins, debris } => issue(
            "inner_skins",
            "info",
            vec![*scan],
            format!(
                "scan {scan} has {skins} closed inner skin(s) (inside another piece, facing inward as the inside of a hollow part does) and {debris} closed debris piece(s) below 1 % of its area with a negative volume; they are not taken as turned inside out"
            ),
        ),
        Issue::Disjoint { groups } => issue(
            "scans_disjoint",
            "info",
            groups.iter().flatten().copied().collect(),
            format!(
                "the scans fall into {} groups that share no surface ({groups:?}): the result is a union of separate pieces",
                groups.len()
            ),
        ),
        _ => return None,
    })
}

/// An issue about the result, in the form of the command.
fn notice_out(i: &Issue) -> MergeIssueOut {
    match i {
        Issue::FullyCovered { scan } => issue(
            "scan_fully_covered",
            "info",
            vec![*scan],
            format!("scan {scan} lies wholly on the others: nothing of it is in the result"),
        ),
        Issue::SeamOpen { edges, length } => issue(
            "seam_open",
            "warning",
            Vec::new(),
            format!(
                "{edges} cut edges ({length:.1} mm) stayed open: no triangle there lay on scanned surface without crossing another"
            ),
        ),
        Issue::RemovedUncovered { share, worst } => issue(
            "removed_not_covered",
            "warning",
            Vec::new(),
            format!(
                "only {:.2} % of the removed area lies within the tolerance of the result (worst {worst:.3} mm)",
                100.0 * share
            ),
        ),
        Issue::Pinched { before, after } => issue(
            "pinched_vertices",
            "warning",
            Vec::new(),
            format!("the result has {after} pinched vertices (the scans had {before})"),
        ),
        other => check_notice(other),
    }
}

/// An issue of the result's own checks (and the scans' flipped triangles), in the form of the command.
fn check_notice(i: &Issue) -> MergeIssueOut {
    match i {
        Issue::FlippedDropped { scan, flipped, share } => issue(
            "flipped_triangles_dropped",
            "warning",
            vec![*scan],
            format!(
                "{flipped} triangles of scan {scan} ({:.3} % of its area) were wound against the rest of their piece and were left out (they count as removed in the checks)",
                100.0 * share
            ),
        ),
        Issue::Unoriented { edges } => issue(
            "unoriented_edges",
            "warning",
            Vec::new(),
            format!("{edges} edges of the result have two triangles that run the same way"),
        ),
        Issue::Unwelded { vertices } => issue(
            "unwelded_vertices",
            "warning",
            Vec::new(),
            format!(
                "{vertices} vertices of the result lie at (nearly) the point of a vertex of another scan without an edge between them"
            ),
        ),
        Issue::Stacked { share } => issue(
            "layers_stacked",
            "warning",
            Vec::new(),
            format!(
                "{:.2} % of the scans' area in the result lies on another scan's surface: two layers were left there",
                100.0 * share
            ),
        ),
        other => issue("other", "warning", Vec::new(), format!("{other:?}")),
    }
}

/// Noise estimate (σ) and median edge of every scan, and the default tolerance from them.
fn default_tolerance(meshes: &[Mesh]) -> (Vec<f64>, f64) {
    let noise: Vec<f64> = meshes.iter().map(|m| surfacecraft_deviation::noise_estimate(m).sigma).collect();
    let mut sorted = noise.clone();
    sorted.sort_by(|a, b| b.total_cmp(a));
    let pair = sorted.iter().take(2).map(|s| s * s).sum::<f64>().sqrt();
    let coarse = meshes.iter().map(|m| surfacecraft_geom::quality::edge_census(m).lengths.median).fold(0.0, f64::max);
    (noise, NOISE_SIGMAS * pair + CHORD_SHARE * coarse)
}

fn scan_out(object: ObjectId, noise: Option<f64>, r: &surfacecraft_geom::merge::ScanRow) -> MergeScanOut {
    MergeScanOut {
        object,
        noise,
        triangles: r.triangles,
        kept: r.kept,
        removed: r.removed,
        restored: r.restored,
        dropped: r.dropped,
        overlap_share: r.overlap_share,
        area: r.area,
        removed_area: r.removed_area,
        median_edge: r.median_edge,
    }
}

fn pair_out(p: &surfacecraft_geom::merge::PairResidual) -> MergePairOut {
    MergePairOut {
        a: p.a,
        b: p.b,
        samples: p.samples,
        shared: p.near,
        over: p.over,
        within: p.within,
        opposed: p.opposed,
        rms: p.rms,
        p95: p.p95,
        worst: p.worst,
        beyond_share: p.beyond_share(),
    }
}

fn seam_out(r: &Report) -> MergeSeamOut {
    let s = &r.seam;
    MergeSeamOut {
        cut_edges: s.front_edges,
        length: 0.5 * s.closed_length,
        radii: s.radii.clone(),
        triangles_created: s.built + s.fill_triangles,
        restored: s.restored,
        holes_filled: s.filled_loops,
        refused_unsupported: s.unsupported,
        refused_crossing: s.crossing,
        unpinched: s.unpinched,
        uncovered_after_seam: s.recovered.uncovered,
        recovered: s.recovered.triangles.saturating_sub(s.recovered.specks_dropped),
        strays_dropped: s.islands_dropped,
        open_edges: s.open_edges,
        open_length: s.open_length,
    }
}

fn checks_out(r: &Report) -> MergeChecksOut {
    let c = &r.checks;
    MergeChecksOut {
        removed_samples: c.removed.samples,
        removed_area: c.removed.area,
        removed_within_share: c.removed.share(),
        removed_worst: c.removed.worst,
        kept_vertices: c.kept_vertices,
        moved_vertices: c.moved_vertices,
        max_vertex_move: c.max_vertex_move,
        added_crossings: c.added_crossings,
        unwelded_vertices: c.unwelded_vertices,
        stacked_share: c.stacked_share,
    }
}

fn stage_name(s: Stage) -> &'static str {
    match s {
        Stage::Prepare => "prepare",
        Stage::Overlap => "overlap",
        Stage::Ownership => "ownership",
        Stage::Union => "union",
        Stage::Seam => "seam",
        Stage::Check => "check",
    }
}

/// The meshes of `objects`, each once.
fn meshes_of(s: &Session, objects: &[ObjRef]) -> CmdResult<(Vec<ObjectId>, Vec<Mesh>)> {
    if !(2..=MAX_OBJECTS).contains(&objects.len()) {
        return Err(CmdError::Params(format!("objects must list 2 to {MAX_OBJECTS} meshes, not {}", objects.len())));
    }
    let mut ids: Vec<ObjectId> = Vec::with_capacity(objects.len());
    for o in objects {
        let id = o.resolve(s, ObjectKind::Mesh)?;
        if ids.contains(&id) {
            return Err(CmdError::Params(format!("mesh {id} is listed twice: list each scan once")));
        }
        ids.push(id);
    }
    let meshes = ids
        .iter()
        .map(|&id| {
            s.doc.mesh(id).map(|m| m.mesh.as_ref().clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))
        })
        .collect::<CmdResult<Vec<Mesh>>>()?;
    Ok((ids, meshes))
}

/// Runs the merge with stage times; the error of a cancellation is the command's.
fn run(meshes: &[Mesh], tolerance: f64, s: &Session) -> CmdResult<(Option<Mesh>, Report, Vec<MergeStageOut>)> {
    let mut stages: Vec<(Stage, Stopwatch)> = Vec::new();
    let mut on_stage = |st: Stage| stages.push((st, Stopwatch::start()));
    let result = merge(meshes, &Params { tolerance }, &s.cancel, &mut on_stage);
    let done = Stopwatch::start();
    let mut times: Vec<MergeStageOut> = Vec::with_capacity(stages.len());
    for (k, (st, w)) in stages.iter().enumerate() {
        let next =
            stages.get(k + 1).map_or_else(|| w.ms().saturating_sub(done.ms()), |(_, n)| w.ms().saturating_sub(n.ms()));
        times.push(MergeStageOut { stage: stage_name(*st).into(), ms: next });
    }
    let (mesh, rep) = result.map_err(|e| match e {
        MergeError::Cancelled => super::cancelled(),
        MergeError::Params(p) => CmdError::Params(p),
        MergeError::Invalid(k, p) => CmdError::Failed(format!("scan {k} is invalid: {p}")),
    })?;
    Ok((mesh, rep, times))
}

fn note_of(rep: &Report, triangles: usize) -> String {
    if rep.refused {
        return "refused: see the issues of severity refusal; nothing was made".into();
    }
    format!(
        "{} scans merged into {triangles} triangles in {} piece(s) with {} boundary loop(s); {:.2} % of the removed area lies within the tolerance of the result; the seam has {} open edge(s)",
        rep.scans.len(),
        rep.after.pieces,
        rep.after.loops,
        100.0 * rep.checks.removed.share(),
        rep.seam.open_edges
    )
}

/// Checks the meshes, then the tolerance: given, or (only then is the noise estimated) from the scans.
fn tolerance_of(meshes: &[Mesh], given: Option<f64>) -> CmdResult<(Vec<Option<f64>>, f64, &'static str)> {
    for (k, m) in meshes.iter().enumerate() {
        m.validate().map_err(|e| CmdError::Params(format!("scan {k} is invalid: {e}")))?;
        if m.triangles.is_empty() {
            return Err(CmdError::Params(format!("scan {k} has no triangles")));
        }
    }
    if let Some(t) = given {
        return Ok((vec![None; meshes.len()], t, "given"));
    }
    let (noise, by_noise) = default_tolerance(meshes);
    if !(by_noise.is_finite() && by_noise > 0.0) {
        return Err(CmdError::Failed("no tolerance can be taken from the scans (no noise, no edges): give one".into()));
    }
    Ok((noise.into_iter().map(Some).collect(), by_noise, "noise"))
}

pub(crate) fn merge_cmd(s: &mut Session, i: MergeIn) -> CmdResult<MergeOut> {
    if s.cancel.is_cancelled() {
        return Err(super::cancelled());
    }
    let given = i.tolerance.map(|t| super::deviation::positive(t, "tolerance")).transpose()?;
    let (ids, meshes) = meshes_of(s, &i.objects)?;
    let (noise, tolerance, source) = tolerance_of(&meshes, given)?;
    let (mesh, rep, stages) = run(&meshes, tolerance, s)?;
    let triangles = mesh.as_ref().map_or(0, Mesh::triangle_count);
    let out = MergeOut {
        id: None,
        refused: rep.refused,
        tolerance,
        tolerance_source: source.into(),
        reach: rep.reach,
        scans: ids.iter().zip(&noise).zip(&rep.scans).map(|((id, n), r)| scan_out(*id, *n, r)).collect(),
        pairs: rep.pairs.iter().map(pair_out).collect(),
        eaten: rep.eaten,
        seam: seam_out(&rep),
        before: topology_out(&rep.before),
        after: topology_out(&rep.after),
        checks: checks_out(&rep),
        issues: rep.issues.iter().map(issue_out).collect(),
        triangles,
        stages,
        note: note_of(&rep, triangles),
    };
    let Some(mesh) = mesh else {
        s.mark_unchanged();
        return Ok(out);
    };
    let new = s.doc.add_mesh(i.name.as_deref().unwrap_or("merged"), mesh);
    Ok(MergeOut { id: Some(new), ..out })
}

/// The `mesh.merge` command.
pub static COMMANDS: &[CommandSpec] = &[command!(
    "mesh.merge",
    "Merge scans",
    "Mesh",
    true,
    "Merges aligned scans (`objects`, 2 to 64 meshes in one frame, e.g. after `align.global`) into one new mesh. Where scans overlap within `tolerance` one layer is kept: the scan in which that surface lies farthest from its own border keeps it (borders are the least reliable part of a scan), the others lose it. The seam between the kept regions is closed by ball pivoting, only with triangles that lie on surface a scan saw, so no surface is made up over an opening nobody scanned. Kept vertices stay where they are, except that seam vertices of later scans move onto coinciding seam vertices of earlier scans, by at most a quarter of the tolerance (and a tenth of the median edge). Scans that disagree in their overlap by more than the tolerance (also when they lie over each other farther apart than 8 tolerances), or whose normals oppose, are refused with the numbers (nothing is made). Reports per scan what was kept and removed, the residual of every pair, the seam, the topology before and after, and how far the removed surface lies from the result.",
    MergeIn,
    MergeOut,
    merge_cmd
)];

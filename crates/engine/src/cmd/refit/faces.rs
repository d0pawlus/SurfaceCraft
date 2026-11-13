//! Face groups and shapes of `body.refit_faces`: relations between faces, union-find and the solve per group.

use super::*;

// ---------------------------------------------------------------------------------------------
// body.refit_faces
// ---------------------------------------------------------------------------------------------

/// How firmly a face is held to its original place.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Stiffness {
    /// Never moves.
    Fixed,
    /// The original counts twice as much as the scan evidence: a third of the measured offset.
    Tight,
    /// The original counts a twentieth as much as the scan: almost all of the measured offset.
    Loose,
    /// Follows the scan.
    Free,
}

impl Stiffness {
    /// Weight of the original position against the scan evidence of the face.
    pub(super) fn prior(self) -> f64 {
        match self {
            Stiffness::Fixed => f64::INFINITY,
            Stiffness::Tight => 2.0,
            Stiffness::Loose => 0.05,
            Stiffness::Free => 0.0,
        }
    }

    pub(super) fn name(self) -> &'static str {
        match self {
            Stiffness::Fixed => "fixed",
            Stiffness::Tight => "tight",
            Stiffness::Loose => "loose",
            Stiffness::Free => "free",
        }
    }
}

/// Stiffness of one face.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FaceStiffness {
    /// Face index (record order of the body).
    pub face: usize,
    /// Its stiffness.
    pub stiffness: Stiffness,
}

/// Parameters of `body.refit_faces`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RefitFacesIn {
    /// Body without feature history (e.g. imported from STEP).
    pub body: ObjRef,
    /// The scan (a mesh, aligned to the body).
    pub mesh: ObjRef,
    /// Stiffness of every face not listed in `faces` (default `loose`).
    #[serde(default)]
    pub stiffness: Option<Stiffness>,
    /// Per-face stiffness.
    #[serde(default)]
    pub faces: Vec<FaceStiffness>,
    /// Largest move of a face (mm, default 1).
    #[serde(default)]
    pub max_change: Option<f64>,
    /// Scan points farther than this from every face are ignored (mm, default `max_change`).
    #[serde(default)]
    pub max_distance: Option<f64>,
    /// A scan point belongs to a face only when its normal is within this angle of the face's
    /// (degrees, default 60).
    #[serde(default)]
    pub max_angle: Option<f64>,
    /// Faces with fewer scan points are not moved (default 8).
    #[serde(default)]
    pub min_points: Option<usize>,
    /// Moves smaller than this are not made (mm, default 0.002).
    #[serde(default)]
    pub min_move: Option<f64>,
    /// Scan points measured at most (default 50000).
    #[serde(default)]
    pub max_points: Option<usize>,
    /// Keep equal radii equal: cylinders, spheres and torus tubes of one radius change together
    /// (default true).
    #[serde(default)]
    pub keep_relations: Option<bool>,
    /// Apply the result to the document (default true); false only reports it.
    #[serde(default)]
    pub apply: Option<bool>,
}

/// One face in the result of `body.refit_faces`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FaceOut {
    /// Face index in the original body.
    pub face: usize,
    /// `plane`, `cylinder`, `cone`, `sphere`, `torus` or `freeform`.
    pub kind: String,
    /// The stiffness it was given.
    pub stiffness: String,
    /// `moved`, `unchanged`, `fixed`, `no_data`, `refused` or `failed`.
    pub status: String,
    /// Scan points associated to the face.
    pub points: usize,
    /// Applied move along the outward normal (mm; for a hole a positive move shrinks it).
    pub offset: f64,
    /// RMS distance of its scan points from the face before (mm).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rms_before: Option<f64>,
    /// RMS distance of the same points from the rebuilt face (mm).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rms_after: Option<f64>,
    /// Surface parameters before.
    pub old: BTreeMap<String, f64>,
    /// Surface parameters after (read back from the rebuilt body).
    pub new: BTreeMap<String, f64>,
    /// Why it did not move, or what went wrong.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

/// Relation counts of a body.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct RelationsOut {
    /// Pairs of parallel normals or axes.
    pub parallel: usize,
    /// Pairs of perpendicular normals or axes.
    pub perpendicular: usize,
    /// Pairs of faces on one axis line.
    pub coaxial: usize,
    /// Pairs of equal radius.
    pub equal_radius: usize,
}

/// Result of `body.refit_faces`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct RefitFacesOut {
    /// The body.
    pub body: ObjectId,
    /// Per face of the original body.
    pub faces: Vec<FaceOut>,
    /// RMS distance of the associated scan points before (mm).
    pub rms_before: f64,
    /// RMS distance of the same points from the rebuilt faces (mm).
    pub rms_after: f64,
    /// Scan points associated to an analytic face.
    pub points: usize,
    /// Scan points that belong to no face (too far or facing away) or to a refused face.
    pub unassigned: usize,
    /// Faces moved.
    pub moved: usize,
    /// Faces of the body before.
    pub faces_before: usize,
    /// Faces of the body after.
    pub faces_after: usize,
    /// Relations among the faces before.
    pub relations_before: RelationsOut,
    /// Relations among the faces after (equal to `relations_before` when they are all kept).
    pub relations_after: RelationsOut,
    /// Was the result applied?
    pub applied: bool,
}

pub(super) fn relations_of(shapes: &[Result<FaceShape, String>]) -> RelationsOut {
    let list: Vec<Option<&FaceShape>> = shapes.iter().map(|s| s.as_ref().ok()).collect();
    let r = relation_counts(&list);
    RelationsOut {
        parallel: r.parallel,
        perpendicular: r.perpendicular,
        coaxial: r.coaxial,
        equal_radius: r.equal_radius,
    }
}

pub(super) fn shape_params(s: &FaceShape) -> BTreeMap<String, f64> {
    s.params().into_iter().map(|(k, v)| (k.to_string(), v)).collect()
}

pub(super) struct UnionFind(Vec<usize>);

impl UnionFind {
    pub(super) fn new(n: usize) -> Self {
        UnionFind((0..n).collect())
    }
    pub(super) fn find(&mut self, mut x: usize) -> usize {
        while let Some(&p) = self.0.get(x) {
            if p == x {
                break;
            }
            let gp = self.0.get(p).copied().unwrap_or(p);
            if let Some(slot) = self.0.get_mut(x) {
                *slot = gp;
            }
            x = gp;
        }
        x
    }
    pub(super) fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb
            && let Some(slot) = self.0.get_mut(ra)
        {
            *slot = rb;
        }
    }
}

fn median(v: &mut [f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(|a, b| a.total_cmp(b));
    let n = v.len();
    let hi = v.get(n / 2).copied().unwrap_or(0.0);
    if n % 2 == 1 { hi } else { (hi + v.get(n / 2 - 1).copied().unwrap_or(hi)) / 2.0 }
}

/// One group of faces that move together: on the same surface or tied by an equal radius. Member
/// `k` moves by `signs[k]·θ`.
pub(super) struct Group {
    pub(super) faces: Vec<usize>,
}

/// The shared move θ of a group: a Huber-weighted mean of the members' signed offsets, held
/// towards zero by the members' priors; zero when a member is fixed.
pub(super) fn solve_group(members: &[(f64, &[f64], f64)], max_change: f64) -> f64 {
    // member = (sign, offsets d of its scan points, prior weight ω)
    let mut prior_total = 0.0;
    let mut signed: Vec<f64> = Vec::new();
    for (sign, d, omega) in members {
        if !omega.is_finite() {
            return 0.0;
        }
        prior_total += omega * d.len() as f64;
        signed.extend(d.iter().map(|x| sign * x));
    }
    if signed.is_empty() {
        return 0.0;
    }
    let mut theta = median(&mut signed.clone());
    for _ in 0..12 {
        let mut resid: Vec<f64> = signed.iter().map(|x| (x - theta).abs()).collect();
        let mad = median(&mut resid);
        let k = (1.4826 * 1.5 * mad).max(0.01);
        let (mut num, mut den) = (0.0, 0.0);
        for x in &signed {
            let r = (x - theta).abs();
            let w = if r <= k { 1.0 } else { k / r };
            num += w * x;
            den += w;
        }
        let next = num / (den + prior_total);
        let done = (next - theta).abs() < 1e-9;
        theta = next;
        if done {
            break;
        }
    }
    theta.clamp(-max_change, max_change)
}

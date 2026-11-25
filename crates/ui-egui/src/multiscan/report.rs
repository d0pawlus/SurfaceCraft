//! What the wizard shows of a command's outcome: the few numbers and sentences of the JSON the
//! engine answers with, read into plain structs. Nothing here decides anything; a field the
//! outcome does not have reads as its empty value, so a newer engine never breaks the wizard.
use serde_json::Value;

/// A row-major 4x4 matrix, as the align commands report their motion.
pub type Matrix = [[f64; 4]; 4];

/// One pose a pair of scans can be put in: the best one, or an alternative.
#[derive(Clone, Debug, PartialEq)]
pub struct Pose {
    /// The motion of the moving scan.
    pub matrix: Matrix,
    /// Share (0..1) of the moving scan that lies on the other.
    pub overlap_moving: f64,
    /// Share (0..1) of the other scan that lies on the moving one.
    pub overlap_fixed: f64,
    /// How far this pose explains the scan better than any different one (0..1).
    pub confidence: f64,
}

/// What `align.pair_coarse` proposed for one pair.
#[derive(Clone, Debug, PartialEq)]
pub struct PairProposal {
    /// `unique`, `ambiguous` or `no_match`.
    pub verdict: String,
    /// Is the verdict `unique`?
    pub unique: bool,
    /// The best pose first, then the alternatives in the order the command lists them.
    pub poses: Vec<Pose>,
    /// The pose the user picked (index into `poses`; 0 is the best).
    pub chosen: usize,
    /// The command's own sentence about the outcome (an English engine message).
    pub note: String,
}

impl PairProposal {
    /// The pose shown and applied.
    #[must_use]
    pub fn chosen_pose(&self) -> Option<&Pose> {
        self.poses.get(self.chosen)
    }

    /// May the pair be accepted? A `no_match` has no pose that fits.
    #[must_use]
    pub fn acceptable(&self) -> bool {
        self.verdict != "no_match" && self.chosen_pose().is_some()
    }
}

fn number(v: &Value, key: &str) -> f64 {
    v.get(key).and_then(Value::as_f64).unwrap_or(0.0)
}

fn text(v: &Value, key: &str) -> String {
    v.get(key).and_then(Value::as_str).unwrap_or_default().to_string()
}

fn count(v: &Value, key: &str) -> usize {
    v.get(key).and_then(Value::as_u64).and_then(|n| usize::try_from(n).ok()).unwrap_or(0)
}

fn flag(v: &Value, key: &str) -> bool {
    v.get(key).and_then(Value::as_bool).unwrap_or(false)
}

fn matrix(v: &Value) -> Option<Matrix> {
    serde_json::from_value(v.clone()).ok()
}

/// The proposal in the outcome of an `align.pair_coarse` run; `None` when it has no pose.
#[must_use]
pub fn pair_proposal(outcome: &Value) -> Option<PairProposal> {
    let best = Pose {
        matrix: matrix(outcome.get("matrix")?)?,
        overlap_moving: number(outcome, "overlap_moving"),
        overlap_fixed: number(outcome, "overlap_fixed"),
        confidence: number(outcome, "confidence"),
    };
    let alternatives = outcome.get("alternatives").and_then(Value::as_array).into_iter().flatten().filter_map(|a| {
        Some(Pose {
            matrix: matrix(a.get("transform")?)?,
            overlap_moving: number(a, "overlap_moving"),
            overlap_fixed: number(a, "overlap_fixed"),
            confidence: number(a, "confidence"),
        })
    });
    Some(PairProposal {
        verdict: text(outcome, "verdict"),
        unique: flag(outcome, "confident"),
        poses: std::iter::once(best).chain(alternatives).collect(),
        chosen: 0,
        note: text(outcome, "note"),
    })
}

/// Did the outcome of an apply run move the scan?
#[must_use]
pub fn applied(outcome: &Value) -> bool {
    flag(outcome, "applied")
}

/// One scan's place in the proposed common frame.
#[derive(Clone, Debug, PartialEq)]
pub struct GlobalScan {
    /// Position in the `objects` the command was given.
    pub index: usize,
    /// The scan that stays where it is.
    pub fixed: bool,
    /// Was it connected to the fixed scan?
    pub placed: bool,
    /// Is it held by a closed loop of strong pairs? False: it hangs on one pair.
    pub held_by_loop: bool,
    /// The motion that brings it into the common frame.
    pub matrix: Matrix,
}

impl GlobalScan {
    /// Does it hang on a single pair that nothing confirms (it moves only when accepted)?
    #[must_use]
    pub fn weakly_held(&self) -> bool {
        self.placed && !self.fixed && !self.held_by_loop
    }
}

/// The evidence of one pair the global alignment tried.
#[derive(Clone, Debug, PartialEq)]
pub struct GlobalEdge {
    /// Position of the scan that stays.
    pub fixed: usize,
    /// Position of the scan that moves.
    pub moving: usize,
    /// `used`, `rejected`, `ambiguous`, `no_match`, ...
    pub status: String,
    /// Residual where the scans meet, mm.
    pub rms: f64,
    /// Share (0..1) of the moving scan that lies on the other.
    pub overlap_moving: f64,
    /// The pair's evidence is strong (small residual).
    pub strong: bool,
}

/// The loop closure error of the global alignment, before and after the relaxation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LoopError {
    /// Independent cycles of the pose graph.
    pub cycles: usize,
    /// Largest edge misfit before the relaxation, mm.
    pub worst_before_mm: f64,
    /// Mean edge misfit before the relaxation, mm.
    pub mean_before_mm: f64,
    /// Largest edge misfit after the relaxation, mm.
    pub worst_after_mm: f64,
    /// Mean edge misfit after the relaxation, mm.
    pub mean_after_mm: f64,
}

/// What `align.global` reported.
#[derive(Clone, Debug, PartialEq)]
pub struct GlobalReport {
    /// `aligned`, `partial` or `failed`.
    pub verdict: String,
    /// The reasons of a verdict other than `aligned` (the engine's own words, snake case).
    pub issues: Vec<String>,
    /// One entry per scan, in the order of `objects`.
    pub scans: Vec<GlobalScan>,
    /// Every pair tried.
    pub edges: Vec<GlobalEdge>,
    /// The loop closure error.
    pub loops: LoopError,
    /// Pairs used as edges of the pose graph.
    pub used_edges: usize,
    /// Pairs left out because they contradicted the others.
    pub rejected_edges: usize,
    /// Indices of the scans an apply moved.
    pub moved: Vec<usize>,
    /// The command's own sentence about the outcome.
    pub note: String,
}

impl GlobalReport {
    /// Is there nothing the apply could move (no scan could be connected)?
    #[must_use]
    pub fn failed(&self) -> bool {
        self.verdict == "failed"
    }
}

fn loop_error(v: &Value) -> LoopError {
    let part = |key: &str, field: &str| v.get(key).map_or(0.0, |s| number(s, field));
    LoopError {
        cycles: count(v, "cycles"),
        worst_before_mm: part("before", "worst_mm"),
        mean_before_mm: part("before", "mean_mm"),
        worst_after_mm: part("after", "worst_mm"),
        mean_after_mm: part("after", "mean_mm"),
    }
}

/// The report in the outcome of an `align.global` run; `None` when it is not one.
#[must_use]
pub fn global_report(outcome: &Value) -> Option<GlobalReport> {
    let scans = outcome
        .get("scans")?
        .as_array()?
        .iter()
        .map(|s| {
            Some(GlobalScan {
                index: count(s, "index"),
                fixed: flag(s, "fixed"),
                placed: flag(s, "placed"),
                held_by_loop: flag(s, "held_by_loop"),
                matrix: matrix(s.get("matrix")?)?,
            })
        })
        .collect::<Option<Vec<_>>>()?;
    let edges = outcome.get("edges").and_then(Value::as_array).into_iter().flatten().map(|e| GlobalEdge {
        fixed: count(e, "fixed"),
        moving: count(e, "moving"),
        status: text(e, "status"),
        rms: number(e, "rms"),
        overlap_moving: number(e, "overlap_moving"),
        strong: flag(e, "strong_evidence"),
    });
    let strings = |key: &str| -> Vec<String> {
        outcome
            .get(key)
            .and_then(Value::as_array)
            .into_iter()
            .flatten()
            .filter_map(Value::as_str)
            .map(str::to_string)
            .collect()
    };
    let moved = outcome
        .get("moved")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|n| n.as_u64().and_then(|n| usize::try_from(n).ok()))
        .collect();
    Some(GlobalReport {
        verdict: text(outcome, "verdict"),
        issues: strings("issues"),
        scans,
        edges: edges.collect(),
        loops: outcome.get("loop").map(loop_error).unwrap_or_default(),
        used_edges: count(outcome, "used_edges"),
        rejected_edges: count(outcome, "rejected_edges"),
        moved,
        note: text(outcome, "note"),
    })
}

/// A finding of `mesh.merge`.
#[derive(Clone, Debug, PartialEq)]
pub struct MergeIssue {
    /// The stable code (`scans_misaligned`, `inverted_scan`, ...).
    pub code: String,
    /// `refusal`, `warning` or `notice`.
    pub severity: String,
    /// Positions of the scans it is about.
    pub scans: Vec<usize>,
    /// The engine's message with the numbers (English; the panel translates it).
    pub message: String,
}

/// The code of the findings that `alignment: "warn"` turns into warnings.
pub const MISALIGNED: &str = "scans_misaligned";

/// What `mesh.merge` reported.
#[derive(Clone, Debug, PartialEq)]
pub struct MergeReport {
    /// The merged mesh; `None` when the merge refused.
    pub id: Option<u64>,
    /// Was the merge refused (nothing made)?
    pub refused: bool,
    /// Triangles of the result.
    pub triangles: usize,
    /// The tolerance used, mm.
    pub tolerance: f64,
    /// Seam edges no triangle could close.
    pub open_edges: usize,
    /// Pieces of the result.
    pub pieces: usize,
    /// Everything the merge found.
    pub issues: Vec<MergeIssue>,
    /// The command's own sentence about the outcome.
    pub note: String,
}

impl MergeReport {
    /// The findings that refused the merge.
    pub fn refusals(&self) -> impl Iterator<Item = &MergeIssue> {
        self.issues.iter().filter(|i| i.severity == "refusal")
    }

    /// Would `alignment: "warn"` let the merge through: it refused, and only for the
    /// misalignment of scans (the other refusals protect the merge itself)?
    #[must_use]
    pub fn warn_would_help(&self) -> bool {
        self.refused && self.refusals().next().is_some() && self.refusals().all(|i| i.code == MISALIGNED)
    }
}

/// The report in the outcome of a `mesh.merge` run; `None` when it is not one.
#[must_use]
pub fn merge_report(outcome: &Value) -> Option<MergeReport> {
    let issues = outcome
        .get("issues")?
        .as_array()?
        .iter()
        .map(|i| MergeIssue {
            code: text(i, "code"),
            severity: text(i, "severity"),
            scans: i
                .get("scans")
                .and_then(Value::as_array)
                .into_iter()
                .flatten()
                .filter_map(|n| n.as_u64().and_then(|n| usize::try_from(n).ok()))
                .collect(),
            message: text(i, "message"),
        })
        .collect();
    let after = outcome.get("after").cloned().unwrap_or(Value::Null);
    Some(MergeReport {
        id: outcome.get("id").and_then(Value::as_u64),
        refused: flag(outcome, "refused"),
        triangles: count(outcome, "triangles"),
        tolerance: number(outcome, "tolerance"),
        open_edges: outcome.get("seam").map_or(0, |s| count(s, "open_edges")),
        pieces: count(&after, "pieces"),
        issues,
        note: text(outcome, "note"),
    })
}

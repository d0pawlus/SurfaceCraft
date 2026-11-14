//! The confusion table of `align.pair_coarse` against a known truth: every pair `(i, j)` of the
//! views of a fixture (moving `j`, fixed `i`), a verdict and the error of the pose against the true
//! motion `P_i P_j^-1`. Shared by the gate `pair_coarse_wrong_unique` and the local corpus tests.
#![allow(dead_code)]
use crate::global_common::*;
use crate::pair_common::*;
use serde_json::{Value, json};
use surfacecraft_align::{PairOptions, PoseFit, pair_coarse, pose_fit};
use surfacecraft_geom::{Cancel, Mesh, Transform};

/// A pose is right when it is within this many mm (largest displacement of a vertex of the moving
/// scan) and degrees of the truth: the bound of the `align.global` gates, which hold views of a part.
pub const RIGHT_MM: f64 = BOUND_MM;
pub const RIGHT_DEG: f64 = BOUND_DEGREES;
/// A pose beyond this is wrong whatever the verdict; between the two it is neither (printed).
pub const WRONG_MM: f64 = 1.0;
/// Pairs whose scans share at least this much (larger of the two shares) are the ones that
/// can be expected to be found.
pub const SHARING: f64 = 0.4;
/// Pairs whose scans share less than this share nothing.
pub const NOTHING: f64 = 0.05;

pub struct Case {
    pub fixture: &'static str,
    pub series: u64,
    pub i: usize,
    pub j: usize,
    /// True shares of the moving and the fixed scan that lie on the other (tolerance 0.3 mm).
    pub truth: (f64, f64),
    pub verdict: String,
    pub error_mm: f64,
    pub angle_deg: f64,
    pub out: Value,
    /// The evidence at the true pose, measured the way the command measures its own.
    pub at_truth: Option<PoseFit>,
    /// For a `unique` pose that is not right: what the command says when the true pose is handed
    /// to it as a candidate (verdict, error mm, confidence, alternatives).
    pub given: Option<String>,
}

impl Case {
    pub fn overlap(&self) -> f64 {
        self.truth.0.max(self.truth.1)
    }
    pub fn is_right(&self) -> bool {
        self.error_mm <= RIGHT_MM && self.angle_deg <= RIGHT_DEG
    }
    pub fn is_wrong(&self) -> bool {
        self.error_mm > WRONG_MM
    }
    pub fn is_unique(&self) -> bool {
        self.verdict == "unique"
    }
    pub fn field(&self, key: &str) -> f64 {
        self.out[key].as_f64().unwrap_or(f64::NAN)
    }
}

fn wedge(k: usize, series: u64) -> Mesh {
    let whole = noisy(blob_at(48 + 4 * k), 900 + k as u64 + 1000 * series);
    cut(&whole, |p| {
        let d = (p.y.atan2(p.x).to_degrees() - 60.0 * k as f64 + 540.0).rem_euclid(360.0) - 180.0;
        d.abs() <= 70.0
    })
}

/// The four fixtures of series `series`: views of the bracket, of the blob, wedges of the blob, arcs
/// of the lumpy ring (all with sigma 0.05 and their own tessellation and noise).
pub fn fixtures(series: u64) -> Vec<(&'static str, Vec<Mesh>)> {
    vec![
        ("bracket", bracket_views_s(6, series)),
        ("blob", blob_views_s(series)),
        ("wedges", (0..6).map(|k| wedge(k, series)).collect()),
        ("ring", (0..8).map(|k| ring_arc_s(k, 45.0 * k as f64, 58.0, series)).collect()),
    ]
}

/// Every pair of the views through the command.
pub fn run(fixture: &'static str, views: &[Mesh], series: u64) -> Vec<Case> {
    let poses = poses_s(views.len(), series);
    let mut s = session_of(views, &poses);
    let mut cases = Vec::new();
    for i in 0..views.len() {
        for j in i + 1..views.len() {
            let params = json!({"object": format!("v{j}"), "target": format!("v{i}")});
            let truth = (true_share(&views[j], &views[i], 0.3), true_share(&views[i], &views[j], 0.3));
            let Ok(out) = s.execute("align.pair_coarse", params) else {
                let verdict = "error".to_string();
                cases.push(Case {
                    fixture,
                    series,
                    i,
                    j,
                    truth,
                    verdict,
                    error_mm: f64::NAN,
                    angle_deg: f64::NAN,
                    out: Value::Null,
                    at_truth: None,
                    given: None,
                });
                continue;
            };
            let (error_mm, angle_deg) = error_against_truth(&out, views, &poses, i, j);
            let verdict = out["verdict"].as_str().unwrap_or("?").to_string();
            let truth_pose = poses[i].then_after(&poses[j].inverse());
            let at_truth = pose_fit(
                &views[j].transformed(&poses[j]),
                &views[i].transformed(&poses[i]),
                &truth_pose,
                None,
                &Cancel::new(),
            )
            .ok();
            let given = (verdict == "unique" && error_mm > RIGHT_MM).then(|| {
                with_truth_given(&views[j].transformed(&poses[j]), &views[i].transformed(&poses[i]), &truth_pose)
            });
            cases.push(Case { fixture, series, i, j, truth, verdict, error_mm, angle_deg, out, at_truth, given });
        }
    }
    cases
}

pub fn error_against_truth(out: &Value, views: &[Mesh], poses: &[Transform], i: usize, j: usize) -> (f64, f64) {
    let truth = poses[i].then_after(&poses[j].inverse());
    let found = matrix_of(out);
    let on = views[j].transformed(&poses[j]);
    let q = truth.inverse().then_after(&found).quaternion();
    let angle = (2.0 * q.vector().norm().atan2(q.scalar().abs())).to_degrees();
    (found.max_displacement(&truth, &on.positions), angle)
}

/// The cases of series `series_list` of every fixture.
pub fn all_cases(series_list: &[u64]) -> Vec<Case> {
    let mut all = Vec::new();
    for &series in series_list {
        for (name, views) in fixtures(series) {
            all.extend(run(name, &views, series));
        }
    }
    all
}

type Class = (&'static str, fn(&Case) -> bool);

const CLASSES: [Class; 3] = [
    ("true overlap < 0.05", |c| c.overlap() < NOTHING),
    ("0.05 .. 0.4", |c| (NOTHING..SHARING).contains(&c.overlap())),
    (">= 0.4", |c| c.overlap() >= SHARING),
];

/// The confusion table: per class of true overlap, the verdicts, `unique` and `ambiguous` split by
/// whether the pose is right (within the bound), wrong (beyond 1 mm) or in between.
pub fn table(cases: &[Case]) -> String {
    let mut s = String::from(
        "class | pairs | unique right | unique wrong | unique between | ambiguous (best pose right) | ambiguous (wrong) | no_match | error\n",
    );
    for (name, in_class) in CLASSES {
        let of: Vec<&Case> = cases.iter().filter(|c| in_class(c)).collect();
        let n = |f: &dyn Fn(&&Case) -> bool| of.iter().filter(|c| f(c)).count();
        let (right, wrong) = (n(&|c| c.is_unique() && c.is_right()), n(&|c| c.is_unique() && c.is_wrong()));
        let between = n(&|c| c.is_unique()) - right - wrong;
        let (amb_right, amb_wrong) =
            (n(&|c| c.verdict == "ambiguous" && c.is_right()), n(&|c| c.verdict == "ambiguous" && !c.is_right()));
        let (none, err) = (n(&|c| c.verdict == "no_match"), n(&|c| c.verdict == "error"));
        s += &format!(
            "{name} | {} | {right} | {wrong} | {between} | {amb_right} | {amb_wrong} | {none} | {err}\n",
            of.len()
        );
    }
    s
}

/// One line per case that is `unique` and not right, with the command's own evidence.
pub fn wrong_unique_lines(cases: &[Case]) -> Vec<String> {
    cases
        .iter()
        .filter(|c| c.is_unique() && !c.is_right())
        .map(|c| {
            format!(
                "{} s{} {}-{}: true overlap {:.2}/{:.2}, error {:.1} mm {:.2} deg, confidence {:.2}, overlap {:.2}/{:.2}, rms {:.3}, tol {:.3}, stability {:.3}, alternatives {}",
                c.fixture,
                c.series,
                c.i,
                c.j,
                c.truth.0,
                c.truth.1,
                c.error_mm,
                c.angle_deg,
                c.field("confidence"),
                c.field("overlap_moving"),
                c.field("overlap_fixed"),
                c.field("rms"),
                c.field("tolerance"),
                c.field("stability"),
                c.out["alternatives"].as_array().map_or(0, Vec::len),
            )
        })
        .collect()
}

/// One line per case with the evidence at the pose found and at the true pose: noise, tolerance,
/// rms in units of the noise, conflicts of both scans, overlaps.
pub fn evidence_lines(cases: &[Case]) -> Vec<String> {
    let mut lines = vec!["fixture seed pair | true overlap m/f | verdict conf | error mm deg | noise tol | rms/noise | found: overlap m/f conflict m/f | truth: overlap m/f conflict m/f rms/noise".to_string()];
    for c in cases {
        let t = c.at_truth.map_or(String::from("-"), |t| {
            format!(
                "{:.2}/{:.2} {:.3}/{:.3} {:.1}",
                t.overlap_moving,
                t.overlap_fixed,
                t.conflict_moving,
                t.conflict_fixed,
                t.rms / t.noise.max(1e-12)
            )
        });
        lines.push(format!(
            "{} s{} {}-{} | {:.2}/{:.2} | {} {:.2} | {:.1} {:.2} | {:.3} {:.3} | {:.1} | {:.2}/{:.2} {:.3}/{:.3} | {}",
            c.fixture,
            c.series,
            c.i,
            c.j,
            c.truth.0,
            c.truth.1,
            c.verdict,
            c.field("confidence"),
            c.error_mm,
            c.angle_deg,
            c.field("noise"),
            c.field("tolerance"),
            c.field("rms") / c.field("noise").max(1e-12),
            c.field("overlap_moving"),
            c.field("overlap_fixed"),
            c.field("conflict_moving"),
            c.field("conflict_fixed"),
            t
        ));
        if let Some(g) = &c.given {
            lines.push(format!("    with the true pose handed over as a candidate: {g}"));
        }
    }
    lines
}

fn with_truth_given(moving: &Mesh, fixed: &Mesh, truth: &Transform) -> String {
    let opts = PairOptions { given: Some(*truth), ..PairOptions::default() };
    match pair_coarse(moving, fixed, &opts, &Cancel::new()) {
        Ok(r) => format!(
            "{:?} conf {:.2} error {:.2} mm, {} alternatives",
            r.verdict,
            r.confidence,
            r.transform.max_displacement(truth, &moving.positions),
            r.alternatives.len()
        ),
        Err(e) => format!("error {e}"),
    }
}

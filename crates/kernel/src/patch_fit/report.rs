//! The report of a fit: what the kept round achieved, which targets were missed and why, and the
//! warnings the caller should read.
use super::measure::Measure;
use super::rounds::Outcome;
use super::start::Loop;
use super::{
    CornerReport, G0_TARGET, G1_TARGET_DEG, G2_TARGET_REL, HARD_RESIDUAL_SHARE, Report, SideReport, WARN_FEW_DATA,
    WARN_MAX_CONTROL, WARN_NO_DATA, WARN_TRIM_START,
};
use crate::vpatch;

/// What the report is built from.
pub(super) struct Inputs<'a> {
    pub lp: &'a Loop,
    pub outcome: &'a Outcome,
    pub tolerance: f64,
    pub max_control: usize,
    pub start_net: [usize; 2],
    pub corners: Vec<CornerReport>,
    /// Warnings of the preparation.
    pub notes: Vec<String>,
    /// The trimmer needed the sides' own parameters.
    pub trim_fallback: bool,
}

/// The targets of one hard side that were missed, in words; empty for a side that met them (and
/// for a soft side, which has none).
fn unmet(s: &SideReport) -> Vec<String> {
    let mut out = Vec::new();
    if s.soft {
        return out;
    }
    if s.gap_max > G0_TARGET {
        out.push(format!("gap {:.4} mm (target {G0_TARGET} mm)", s.gap_max));
    }
    if let Some(a) = s.angle_max_deg.filter(|a| *a > G1_TARGET_DEG) {
        out.push(format!("angle {a:.3}° (target {G1_TARGET_DEG}°)"));
    }
    if let Some(c) = s.curvature_error_max.filter(|c| *c > G2_TARGET_REL) {
        out.push(format!("normal curvature error {:.1} % (target {:.0} %)", 100.0 * c, 100.0 * G2_TARGET_REL));
    }
    if s.folded {
        out.push("the patch folds back over the neighbouring face".to_string());
    }
    out
}

fn side_warnings(lp: &Loop, m: &Measure, frozen: &[bool], out: &mut Vec<String>) {
    for (k, (s, sd)) in m.sides.iter().zip(&lp.sides).enumerate() {
        let missed = unmet(s);
        if !missed.is_empty() {
            out.push(format!("side {}: {:?} not met: {}", sd.label, s.continuity, missed.join(", ")));
        }
        if frozen.get(k).copied().unwrap_or(false) && !s.soft && s.gap_max > G0_TARGET {
            out.push(format!("side {}: the gap of {:.4} mm is the accuracy of the side itself", sd.label, s.gap_max));
        }
    }
}

/// What the verdict was, for the warnings.
struct Verdict {
    within_tolerance: bool,
    residual_ok: bool,
    within_targets: bool,
}

/// The warnings of the report: the preparation's notes, then what the kept round says.
fn warnings(i: &Inputs, v: &Verdict) -> Vec<String> {
    let best = &i.outcome.best;
    let m = &best.measure;
    let max = i.max_control;
    let (within_tolerance, within_targets, residual_ok) = (v.within_tolerance, v.within_targets, v.residual_ok);
    let mut warnings = i.notes.clone();
    if i.trim_fallback {
        warnings.push(WARN_TRIM_START.to_string());
    }
    if m.points == 0 {
        warnings.push(WARN_NO_DATA.to_string());
    } else if m.points < 3 * best.net[0] * best.net[1] {
        warnings.push(WARN_FEW_DATA.to_string());
    }
    if (best.net[0] >= max || best.net[1] >= max) && (best.net != i.start_net || !(within_tolerance && within_targets))
    {
        warnings.push(WARN_MAX_CONTROL.to_string());
    }
    if let Some(stop) = i.outcome.stop {
        warnings.push(format!(
            "the control net stopped at {} × {}: a net of {} × {} needs about {:.1} s per solve (limit {:.1} s); \
             the report shows what that net achieved",
            best.net[0],
            best.net[1],
            stop.net[0],
            stop.net[1],
            stop.seconds,
            vpatch::MAX_SOLVE_SECONDS
        ));
    }
    if m.outside > 0 {
        warnings.push(format!("{} data points lie outside the outline and were ignored", m.outside));
    }
    if m.hard_max > G0_TARGET {
        warnings.push(format!("interior points or curves miss the patch by up to {:.4} mm", m.hard_max));
    }
    if m.interior_folded {
        warnings
            .push("the patch folds over inside the outline (its normal turns past the outline's plane)".to_string());
    }
    if !residual_ok {
        warnings.push(format!(
            "the hard conditions contradict each other: the solved patch misses one by {:.4} mm (limit {:.4} mm)",
            best.hard_residual,
            HARD_RESIDUAL_SHARE * G0_TARGET
        ));
    }
    side_warnings(i.lp, m, &i.outcome.frozen, &mut warnings);
    warnings
}

pub(super) fn build(i: Inputs) -> Report {
    let best = &i.outcome.best;
    let m = &best.measure;
    let within_tolerance = m.points == 0 || m.max <= i.tolerance;
    let residual_ok = best.hard_residual <= HARD_RESIDUAL_SHARE * G0_TARGET;
    let within_targets = m.targets_met() && within_tolerance && i.corners.is_empty() && residual_ok;
    let warnings = warnings(&i, &Verdict { within_tolerance, residual_ok, within_targets });
    Report {
        control: best.net,
        rounds: i.outcome.rounds,
        rms: m.rms,
        p99: m.p99,
        max: m.max,
        points: m.points,
        outside: m.outside,
        within_tolerance,
        within_targets,
        folded: m.interior_folded || m.sides.iter().any(|s| s.folded),
        sides: m.sides.clone(),
        corners: i.corners,
        soft_points_max: m.soft_max,
        dropped: best.dropped,
        hard_residual: best.hard_residual,
        cost_limited: i.outcome.stop.is_some(),
        warnings,
    }
}

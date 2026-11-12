//! `scan.to_solid` with `method: auto`: the choice of method is by outcome. The share of the scan in
//! regions a primitive fits only orders the attempts; when the first does not end in a pass the
//! other is run too, and the better result is returned. The losing attempt leaves nothing in the
//! document, and the whole run is still one undo step.
use super::{
    CmdResult, MIN_MODELLED_SHARE, Method, ScanSummary, Settings, ToSolidIn, ToSolidOut, Verdict, freeform,
    primitives_solid, scan_primitive_share, segment_scan,
};
use crate::Session;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// The share of the scan in primitive regions below which a primitives attempt is not tried after a
/// failed freeform one (half the share that makes it the first choice).
const PLAUSIBLE_SHARE: f64 = MIN_MODELLED_SHARE / 2.0;

/// One method tried by `scan.to_solid`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct Attempt {
    /// The method tried.
    pub method: Method,
    /// How it ended.
    pub verdict: Verdict,
    /// The codes of the issues that kept it from a pass.
    pub issues: Vec<String>,
    /// Wall time of the attempt (s; 0 where there is no clock).
    pub seconds: f64,
    /// This attempt's result is the one returned.
    pub returned: bool,
}

/// Result of `scan.to_solid`: the result of the method returned, and the attempts.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct ToSolidReport {
    /// The returned attempt's result.
    #[serde(flatten)]
    pub result: ToSolidOut,
    /// Every method tried, in order; one when the first passed or the method was given.
    pub attempts: Vec<Attempt>,
}

/// A clock that is absent on wasm.
struct Timer(#[cfg(not(target_arch = "wasm32"))] std::time::Instant);

impl Timer {
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

    fn seconds(&self) -> f64 {
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.0.elapsed().as_secs_f64()
        }
        #[cfg(target_arch = "wasm32")]
        {
            0.0
        }
    }
}

/// Runs one method and records the attempt.
fn attempt(
    s: &mut Session,
    i: &ToSolidIn,
    set: &Settings,
    report: &ScanSummary,
    method: Method,
) -> CmdResult<(ToSolidOut, Attempt)> {
    let timer = Timer::start();
    let result = match method {
        Method::Freeform => freeform::freeform_solid(s, set, report.clone())?,
        _ => primitives_solid(s, i, set, report.clone())?,
    };
    let record = Attempt {
        method,
        verdict: result.verdict,
        issues: result.issues.iter().map(|x| x.code.clone()).collect(),
        seconds: timer.seconds(),
        returned: false,
    };
    Ok((result, record))
}

fn rank(v: Verdict) -> u8 {
    match v {
        Verdict::Pass => 2,
        Verdict::Review => 1,
        Verdict::Fail => 0,
    }
}

/// Is `b` a better result than `a`: a better verdict; at the same verdict the larger share of the
/// points within the tolerance; then a closed solid over a sheet. A tie keeps `a`.
fn is_better(b: &ToSolidOut, a: &ToSolidOut) -> bool {
    let within = |r: &ToSolidOut| r.deviation.as_ref().map_or(0.0, |d| d.within);
    let solid = |r: &ToSolidOut| r.body.as_ref().is_some_and(|b| b.volume.is_some());
    (rank(b.verdict), within(b), solid(b)) > (rank(a.verdict), within(a), solid(a))
}

fn single(result: ToSolidOut, mut record: Attempt) -> ToSolidReport {
    record.returned = true;
    ToSolidReport { result, attempts: vec![record] }
}

/// `scan.to_solid` after cleaning: the given method once, or `auto`'s ordered attempts.
pub(super) fn run(s: &mut Session, i: &ToSolidIn, set: &Settings, report: ScanSummary) -> CmdResult<ToSolidReport> {
    if i.method == Method::Freeform {
        let (result, record) = attempt(s, i, set, &report, Method::Freeform)?;
        return Ok(single(result, record));
    }
    segment_scan(s, i, set)?;
    if i.method == Method::Primitives || i.reuse_regions {
        let (result, record) = attempt(s, i, set, &report, Method::Primitives)?;
        return Ok(single(result, record));
    }
    let share = scan_primitive_share(s, set, &report);
    let (first, second) = if share >= MIN_MODELLED_SHARE {
        (Method::Primitives, Method::Freeform)
    } else {
        (Method::Freeform, Method::Primitives)
    };
    let base = s.doc.clone();
    let (first_result, first_record) = attempt(s, i, set, &report, first)?;
    let second_plausible = first == Method::Primitives || share >= PLAUSIBLE_SHARE;
    if first_result.verdict == Verdict::Pass || !second_plausible {
        return Ok(single(first_result, first_record));
    }
    let after_first = std::mem::replace(&mut s.doc, base);
    let (second_result, second_record) = match attempt(s, i, set, &report, second) {
        Ok(x) => x,
        Err(_) => {
            // a second method that errors is no better than the first result: keep that
            s.doc = after_first;
            let failed = Attempt {
                method: second,
                verdict: Verdict::Fail,
                issues: vec!["error".into()],
                seconds: 0.0,
                returned: false,
            };
            let mut first_record = first_record;
            first_record.returned = true;
            return Ok(ToSolidReport { result: first_result, attempts: vec![first_record, failed] });
        }
    };
    let second_wins = is_better(&second_result, &first_result);
    if !second_wins {
        s.doc = after_first;
    }
    let (mut a, mut b) = (first_record, second_record);
    (a.returned, b.returned) = (!second_wins, second_wins);
    let result = if second_wins { second_result } else { first_result };
    Ok(ToSolidReport { result, attempts: vec![a, b] })
}

//! `cargo xtask test --partition <kind>:K/N`: one of N parts of the test suite, so CI can run the
//! parts on separate machines at the same time. Every test runs in exactly one part; the doc tests
//! run with part 1.
//!
//! `count`, `hash` and `slice` are nextest's own partitions (equal numbers of tests). `balanced`
//! (what CI uses) first spreads the heaviest tests of [`HEAVY_TESTS`] over the parts by their
//! measured time, then runs `slice:K/N` of every other test: a few tests hold most of the time,
//! so equal numbers of tests gave parts of very unequal length.

use crate::heavy_tests::HEAVY_TESTS;

/// Longest filter expression passed to nextest (the Windows command line holds 32767 characters).
const MAX_EXPRESSION_CHARS: usize = 24_000;

/// One part of the suite: `<kind>:K/N` with 1 <= K <= N.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Partition {
    kind: Kind,
    shard: u32,
    total: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Count,
    Hash,
    Slice,
    Balanced,
}

impl Kind {
    fn name(self) -> &'static str {
        match self {
            Kind::Count => "count",
            Kind::Hash => "hash",
            Kind::Slice => "slice",
            Kind::Balanced => "balanced",
        }
    }
}

/// One `cargo nextest run` of a part: what it is called and the arguments it adds.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct NextestRun {
    pub(crate) label: String,
    pub(crate) args: Vec<String>,
}

impl Partition {
    /// `<kind>:K/N` as given (leading zeros removed).
    pub(crate) fn spec(&self) -> String {
        format!("{}:{}/{}", self.kind.name(), self.shard, self.total)
    }

    /// Whether this part also runs the doc tests (nextest does not run them).
    pub(crate) fn runs_doc_tests(&self) -> bool {
        self.shard == 1
    }

    /// The nextest runs of this part, in order. A `balanced` part has up to two: its share of the
    /// heavy tests (absent when it has none; a stale entry matching nothing only warns), then its
    /// slice of every other test (which must match at least one test).
    pub(crate) fn nextest_runs(&self) -> Result<Vec<NextestRun>, String> {
        let spec = self.spec();
        if self.kind != Kind::Balanced {
            return Ok(vec![NextestRun {
                label: format!("tests (nextest, part {spec})"),
                args: vec!["--partition".into(), spec],
            }]);
        }
        let mut runs = Vec::new();
        let mine: Vec<(&str, &str)> =
            heavy_bins(self.total).into_iter().nth(self.shard.saturating_sub(1) as usize).unwrap_or_default();
        if !mine.is_empty() {
            runs.push(NextestRun {
                label: format!("heavy tests (nextest, part {spec}: {} of {})", mine.len(), HEAVY_TESTS.len()),
                args: vec!["-E".into(), checked(any_of(&mine))?, "--no-tests=warn".into()],
            });
        }
        let all: Vec<(&str, &str)> = HEAVY_TESTS.iter().map(|&(b, t, _)| (b, t)).collect();
        runs.push(NextestRun {
            label: format!("other tests (nextest, part {spec})"),
            args: vec![
                "-E".into(),
                checked(format!("not ({})", any_of(&all)))?,
                "--partition".into(),
                format!("slice:{}/{}", self.shard, self.total),
            ],
        });
        Ok(runs)
    }
}

/// The heavy tests spread over `total` parts: heaviest first, each to the part with the least
/// time so far (ties to the lower part), so every machine computes the same split.
fn heavy_bins(total: u32) -> Vec<Vec<(&'static str, &'static str)>> {
    let mut order: Vec<&(&str, &str, u32)> = HEAVY_TESTS.iter().collect();
    order.sort_by(|a, b| b.2.cmp(&a.2).then(a.0.cmp(b.0)).then(a.1.cmp(b.1)));
    let mut bins: Vec<(u64, Vec<(&str, &str)>)> = (0..total).map(|_| (0, Vec::new())).collect();
    for &(binary, test, seconds) in order {
        let lightest = bins.iter_mut().min_by_key(|(load, _)| *load);
        if let Some((load, tests)) = lightest {
            *load += u64::from(seconds);
            tests.push((binary, test));
        }
    }
    bins.into_iter().map(|(_, tests)| tests).collect()
}

/// A nextest filter expression matching exactly the listed tests.
fn any_of(tests: &[(&str, &str)]) -> String {
    tests
        .iter()
        .map(|(binary, test)| format!("(binary_id(={binary}) and test(={test}))"))
        .collect::<Vec<_>>()
        .join(" or ")
}

fn checked(expression: String) -> Result<String, String> {
    if expression.len() > MAX_EXPRESSION_CHARS {
        return Err(format!(
            "the filter expression of the heavy tests has {} characters (limit {MAX_EXPRESSION_CHARS}): shorten HEAVY_TESTS",
            expression.len()
        ));
    }
    Ok(expression)
}

/// The partition named by `--partition <spec>` or `--partition=<spec>` in `args` (the arguments
/// after `test`), if any; any other argument is refused, so a misspelt flag does not silently run
/// the whole suite.
pub(crate) fn from_args(args: &[String]) -> Result<Option<Partition>, String> {
    let mut found = None;
    let mut it = args.iter();
    while let Some(a) = it.next() {
        let value = if a == "--partition" {
            it.next().ok_or("--partition needs a value such as balanced:1/3")?.as_str()
        } else if let Some(v) = a.strip_prefix("--partition=") {
            v
        } else {
            return Err(format!("test: unknown argument {a:?} (expected --partition balanced:K/N)"));
        };
        if found.is_some() {
            return Err("--partition given twice".into());
        }
        found = Some(parse(value)?);
    }
    Ok(found)
}

fn parse(spec: &str) -> Result<Partition, String> {
    let bad = || format!("--partition {spec:?}: expected balanced, slice, count or hash:K/N with 1 <= K <= N");
    let (kind, part) = spec.split_once(':').ok_or_else(bad)?;
    let kind = match kind {
        "count" => Kind::Count,
        "hash" => Kind::Hash,
        "slice" => Kind::Slice,
        "balanced" => Kind::Balanced,
        _ => return Err(bad()),
    };
    let (k, n) = part.split_once('/').ok_or_else(bad)?;
    let digits = |s: &str| !s.is_empty() && s.len() <= 3 && s.bytes().all(|b| b.is_ascii_digit());
    if !digits(k) || !digits(n) {
        return Err(bad());
    }
    let shard: u32 = k.parse().map_err(|_| bad())?;
    let total: u32 = n.parse().map_err(|_| bad())?;
    if shard == 0 || shard > total {
        return Err(bad());
    }
    Ok(Partition { kind, shard, total })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| (*s).to_string()).collect()
    }

    #[test]
    fn no_flag_means_the_whole_suite() {
        assert_eq!(from_args(&args(&[])), Ok(None));
    }

    #[test]
    fn a_misspelt_flag_is_refused_instead_of_running_everything() {
        assert!(from_args(&args(&["--partiton", "count:1/2"])).is_err());
        assert!(from_args(&args(&["count:1/2"])).is_err());
    }

    #[test]
    fn both_spellings_are_read() {
        let a = from_args(&args(&["--partition", "count:2/3"])).unwrap().unwrap();
        let b = from_args(&args(&["--partition=hash:2/3"])).unwrap().unwrap();
        assert_eq!(a.spec(), "count:2/3");
        assert_eq!(b.spec(), "hash:2/3");
    }

    #[test]
    fn only_the_first_part_runs_the_doc_tests() {
        let first = from_args(&args(&["--partition", "count:1/2"])).unwrap().unwrap();
        let second = from_args(&args(&["--partition", "count:2/2"])).unwrap().unwrap();
        assert!(first.runs_doc_tests());
        assert!(!second.runs_doc_tests());
    }

    #[test]
    fn leading_zeros_are_normalised() {
        let p = from_args(&args(&["--partition", "count:01/03"])).unwrap().unwrap();
        assert_eq!(p.spec(), "count:1/3");
    }

    #[test]
    fn hostile_values_are_refused() {
        for v in [
            "",
            "count",
            "count:",
            "count:1",
            "count:0/3",
            "count:4/3",
            "count:-1/3",
            "count:1/",
            "count:/3",
            "weighted:1/3",
            "balanced:0/3",
            "count:1/3/4",
            "count: 1/3",
            "count:1000/1000",
            "count:a/b",
        ] {
            assert!(from_args(&args(&["--partition", v])).is_err(), "{v:?} accepted");
        }
        assert!(from_args(&args(&["--partition"])).is_err(), "missing value accepted");
        assert!(from_args(&args(&["--partition", "count:1/2", "--partition", "count:2/2"])).is_err());
    }

    fn part(spec: &str) -> Partition {
        from_args(&args(&["--partition", spec])).unwrap().unwrap()
    }

    #[test]
    fn nextest_partitions_are_passed_through_as_one_run() {
        for spec in ["count:2/3", "hash:1/2", "slice:3/3"] {
            let runs = part(spec).nextest_runs().unwrap();
            assert_eq!(runs.len(), 1);
            assert_eq!(runs[0].args, vec!["--partition".to_string(), spec.to_string()]);
        }
    }

    /// The filter expressions of every part of a balanced split of `total`.
    fn balanced_runs(total: u32) -> Vec<Vec<NextestRun>> {
        (1..=total).map(|k| part(&format!("balanced:{k}/{total}")).nextest_runs().unwrap()).collect()
    }

    fn expression(run: &NextestRun) -> &str {
        let at = run.args.iter().position(|a| a == "-E").unwrap();
        &run.args[at + 1]
    }

    #[test]
    fn every_heavy_test_is_in_exactly_one_part() {
        for total in [1, 2, 3, 5, 40] {
            let parts = balanced_runs(total);
            for &(binary, test, _) in HEAVY_TESTS {
                let one = format!("(binary_id(={binary}) and test(={test}))");
                let holders = parts
                    .iter()
                    .filter(|runs| runs.len() == 2 && expression(&runs[0]).split(" or ").any(|e| e == one))
                    .count();
                assert_eq!(holders, 1, "{binary} {test} in {holders} of {total} parts");
            }
        }
    }

    #[test]
    fn the_other_tests_are_one_slice_per_part_of_everything_not_heavy() {
        let parts = balanced_runs(3);
        let others: Vec<&NextestRun> = parts.iter().map(|runs| runs.last().unwrap()).collect();
        let not_heavy = expression(others[0]);
        assert!(not_heavy.starts_with("not ("));
        for &(binary, test, _) in HEAVY_TESTS {
            assert!(not_heavy.contains(&format!("(binary_id(={binary}) and test(={test}))")));
        }
        for (k, run) in others.iter().enumerate() {
            assert_eq!(expression(run), not_heavy, "every part excludes the same heavy tests");
            let slice = format!("slice:{}/3", k + 1);
            assert!(run.args.ends_with(&["--partition".to_string(), slice]));
            assert!(!run.args.iter().any(|a| a.starts_with("--no-tests")), "an empty slice must fail");
        }
        assert!(parts.iter().all(|runs| runs.len() == 2 && runs[0].args.contains(&"--no-tests=warn".to_string())));
    }

    #[test]
    fn a_part_without_heavy_tests_runs_only_its_slice() {
        let total = HEAVY_TESTS.len() as u32 + 1;
        let runs = part(&format!("balanced:{total}/{total}")).nextest_runs().unwrap();
        assert_eq!(runs.len(), 1);
        assert!(runs[0].args.contains(&format!("slice:{total}/{total}")));
    }

    #[test]
    fn the_heavy_tests_are_spread_by_time() {
        let weight = |b: &str, t: &str| HEAVY_TESTS.iter().find(|e| e.0 == b && e.1 == t).unwrap().2;
        let heaviest = HEAVY_TESTS.iter().map(|e| e.2).max().unwrap();
        for total in [2, 3] {
            let loads: Vec<u32> =
                heavy_bins(total).iter().map(|bin| bin.iter().map(|(b, t)| weight(b, t)).sum()).collect();
            let (lo, hi) = (*loads.iter().min().unwrap(), *loads.iter().max().unwrap());
            assert!(hi - lo <= heaviest, "{total} parts: loads {loads:?}");
        }
    }

    #[test]
    fn expressions_name_library_and_integration_tests_exactly() {
        assert_eq!(
            any_of(&[("surfacecraft-geom", "recon::tests::hostile_clouds_never_panic")]),
            "(binary_id(=surfacecraft-geom) and test(=recon::tests::hostile_clouds_never_panic))"
        );
        assert_eq!(
            any_of(&[
                ("surfacecraft-engine::pair_coarse_wrong_unique", "gate_m2_no_wrong_pose_is_unique_series_0"),
                ("surfacecraft-engine::hybrid", "p9_block_with_wavy_top_has_one_freeform_top"),
            ]),
            "(binary_id(=surfacecraft-engine::pair_coarse_wrong_unique) and test(=gate_m2_no_wrong_pose_is_unique_series_0)) \
             or (binary_id(=surfacecraft-engine::hybrid) and test(=p9_block_with_wavy_top_has_one_freeform_top))"
        );
    }

    #[test]
    fn the_heavy_list_is_well_formed() {
        let plain = |s: &str| !s.is_empty() && s.bytes().all(|b| b.is_ascii_alphanumeric() || b"_:-/".contains(&b));
        for (i, &(binary, test, seconds)) in HEAVY_TESTS.iter().enumerate() {
            assert!(plain(binary) && plain(test), "entry {i} has characters a filter expression would misread");
            assert!(seconds > 0);
            assert!(!HEAVY_TESTS[..i].iter().any(|e| e.0 == binary && e.1 == test), "{binary} {test} listed twice");
        }
        let all: Vec<(&str, &str)> = HEAVY_TESTS.iter().map(|&(b, t, _)| (b, t)).collect();
        assert!(checked(format!("not ({})", any_of(&all))).is_ok());
        assert!(checked("x".repeat(MAX_EXPRESSION_CHARS + 1)).is_err());
    }
}

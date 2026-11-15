//! `cargo xtask test --partition count:K/N`: one of N parts of the test suite, so CI can run the
//! parts on separate machines at the same time. Every test runs in exactly one part (nextest
//! assigns them); the doc tests run with part 1.

/// One part of the suite: `count:K/N` or `hash:K/N` with 1 <= K <= N, as nextest takes it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Partition {
    spec: String,
    shard: u32,
}

impl Partition {
    /// The argument nextest takes after `--partition`.
    pub(crate) fn spec(&self) -> &str {
        &self.spec
    }

    /// Whether this part also runs the doc tests (nextest does not partition them).
    pub(crate) fn runs_doc_tests(&self) -> bool {
        self.shard == 1
    }
}

/// The partition named by `--partition <spec>` or `--partition=<spec>` in `args` (the arguments
/// after `test`), if any; any other argument is refused, so a misspelt flag does not silently run
/// the whole suite.
pub(crate) fn from_args(args: &[String]) -> Result<Option<Partition>, String> {
    let mut found = None;
    let mut it = args.iter();
    while let Some(a) = it.next() {
        let value = if a == "--partition" {
            it.next().ok_or("--partition needs a value such as count:1/3")?.as_str()
        } else if let Some(v) = a.strip_prefix("--partition=") {
            v
        } else {
            return Err(format!("test: unknown argument {a:?} (expected --partition count:K/N)"));
        };
        if found.is_some() {
            return Err("--partition given twice".into());
        }
        found = Some(parse(value)?);
    }
    Ok(found)
}

fn parse(spec: &str) -> Result<Partition, String> {
    let bad = || format!("--partition {spec:?}: expected count:K/N or hash:K/N with 1 <= K <= N");
    let (kind, part) = spec.split_once(':').ok_or_else(bad)?;
    if kind != "count" && kind != "hash" {
        return Err(bad());
    }
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
    Ok(Partition { spec: format!("{kind}:{shard}/{total}"), shard })
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
            "slice:1/3",
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
}

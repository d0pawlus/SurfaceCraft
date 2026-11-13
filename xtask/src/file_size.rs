//! `cargo xtask file-size`: no Rust file under `crates/`, `apps/` or `xtask/` has more than 800
//! lines, unless it is on the allowlist `xtask/file-size.txt` with its current line count.
//! The list only shrinks: a listed file may not grow, and a listed file that is now 800 lines or
//! fewer must leave the list.
use std::collections::BTreeMap;

/// The most lines a Rust file may have unless it is on the allowlist.
const MAX_LINES: usize = 800;
/// The allowlist: one `<path> <lines>` pair per line, `#` starts a comment.
const ALLOWLIST: &str = "xtask/file-size.txt";
/// Directories whose `*.rs` files are checked.
const DIRS: &[&str] = &["crates", "apps", "xtask"];

/// Lines of a file, counted as `wc -l` counts them (newlines).
fn line_count(source: &str) -> usize {
    source.bytes().filter(|b| *b == b'\n').count()
}

/// Reads the allowlist into path → recorded line count.
fn parse_allowlist(text: &str) -> Result<BTreeMap<String, usize>, String> {
    let mut listed = BTreeMap::new();
    for (i, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut parts = line.split_whitespace();
        let (Some(path), Some(count), None) = (parts.next(), parts.next(), parts.next()) else {
            return Err(format!("{ALLOWLIST}:{}: expected `<path> <lines>`", i + 1));
        };
        let count = count.parse::<usize>().map_err(|e| format!("{ALLOWLIST}:{}: {e}", i + 1))?;
        if listed.insert(path.to_string(), count).is_some() {
            return Err(format!("{ALLOWLIST}:{}: {path} is listed twice", i + 1));
        }
    }
    Ok(listed)
}

/// What to do about one file, given its line count and its recorded count (if listed):
/// `None` when it is fine, otherwise the reason it fails.
fn verdict(lines: usize, recorded: Option<usize>) -> Option<String> {
    match recorded {
        None if lines > MAX_LINES => Some(format!(
            "{lines} lines, over {MAX_LINES}: split it into modules (or, if it cannot be split yet, list it in {ALLOWLIST})"
        )),
        None => None,
        Some(_) if lines <= MAX_LINES => {
            Some(format!("{lines} lines, {MAX_LINES} or fewer: remove it from {ALLOWLIST}"))
        }
        Some(recorded) if lines > recorded => {
            Some(format!("grew from {recorded} to {lines} lines: the list only shrinks, split the file instead"))
        }
        Some(recorded) if lines < recorded => {
            Some(format!("shrank from {recorded} to {lines} lines: update its count in {ALLOWLIST}"))
        }
        Some(_) => None,
    }
}

/// Checks every Rust file under the checked directories against the allowlist.
pub(crate) fn check() -> Result<(), String> {
    eprintln!("==> file-size");
    let root = crate::root();
    let text = std::fs::read_to_string(root.join(ALLOWLIST)).map_err(|e| format!("read {ALLOWLIST}: {e}"))?;
    let listed = parse_allowlist(&text)?;
    let mut errors = Vec::new();
    let mut seen = Vec::new();
    for dir in DIRS {
        crate::walk(&root.join(dir), &mut |p| {
            if !p.extension().is_some_and(|e| e == "rs") {
                return;
            }
            let rel = crate::rel(p);
            match std::fs::read_to_string(p) {
                Ok(source) => {
                    if let Some(reason) = verdict(line_count(&source), listed.get(&rel).copied()) {
                        errors.push(format!("{rel}: {reason}"));
                    }
                }
                Err(e) => errors.push(format!("{rel}: cannot read: {e}")),
            }
            seen.push(rel);
        });
    }
    for path in listed.keys() {
        if !seen.contains(path) {
            errors.push(format!("{ALLOWLIST} lists {path}, which does not exist: remove the entry"));
        }
    }
    if errors.is_empty() { Ok(()) } else { Err(errors.join("\n")) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unlisted_file_over_the_limit_fails() {
        assert!(verdict(801, None).is_some());
    }

    #[test]
    fn unlisted_file_at_the_limit_passes() {
        assert!(verdict(800, None).is_none());
    }

    #[test]
    fn listed_file_with_its_recorded_count_passes() {
        assert!(verdict(1045, Some(1045)).is_none());
    }

    #[test]
    fn listed_file_that_grew_fails() {
        let reason = verdict(1046, Some(1045));
        assert!(reason.is_some_and(|r| r.contains("grew from 1045 to 1046")));
    }

    #[test]
    fn listed_file_that_is_now_small_fails_with_the_removal_message() {
        let reason = verdict(790, Some(1045));
        assert!(reason.is_some_and(|r| r.contains("remove it from xtask/file-size.txt")));
    }

    #[test]
    fn listed_file_that_shrank_but_is_still_over_fails_until_its_count_is_updated() {
        assert!(verdict(900, Some(1045)).is_some_and(|r| r.contains("update its count")));
    }

    #[test]
    fn allowlist_parses_pairs_and_skips_comments() {
        let parsed = parse_allowlist("# note\n\ncrates/a.rs 812\n").expect("valid list");
        assert_eq!(parsed.get("crates/a.rs"), Some(&812));
    }

    #[test]
    fn allowlist_rejects_a_line_without_a_count() {
        assert!(parse_allowlist("crates/a.rs\n").is_err());
    }
}

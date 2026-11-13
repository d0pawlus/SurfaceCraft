//! `cargo xtask file-size`: no Rust file under `crates/`, `apps/` or `xtask/` has more than 800
//! lines, unless it is on the allowlist `xtask/file-size.txt` with its current line count.
//! The list only shrinks: a listed file may not grow, and a listed file that is now 800 lines or
//! fewer must leave the list. A listed file that shrank but is still over the limit passes with a
//! note, so a change that deletes a few lines of it need not touch the list.
use std::collections::{BTreeMap, BTreeSet};

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

/// What the check says about one file.
#[derive(Debug, PartialEq)]
enum Verdict {
    Fine,
    /// Passes; the text is printed as a note.
    Note(String),
    /// Fails for this reason.
    Fail(String),
}

/// What to do about one file, given its line count and its recorded count (if listed).
fn verdict(lines: usize, recorded: Option<usize>) -> Verdict {
    match recorded {
        None if lines > MAX_LINES => Verdict::Fail(format!(
            "{lines} lines, over {MAX_LINES}: split it into modules (or, if it cannot be split yet, list it in {ALLOWLIST})"
        )),
        None => Verdict::Fine,
        Some(_) if lines <= MAX_LINES => {
            Verdict::Fail(format!("{lines} lines, {MAX_LINES} or fewer: remove it from {ALLOWLIST}"))
        }
        Some(recorded) if lines > recorded => Verdict::Fail(format!(
            "grew from {recorded} to {lines} lines: the list only shrinks, split the file instead"
        )),
        Some(recorded) if lines < recorded => {
            Verdict::Note(format!("shrank from {recorded} to {lines} lines: lower its count in {ALLOWLIST}"))
        }
        Some(_) => Verdict::Fine,
    }
}

/// Checks every Rust file under the checked directories against the allowlist.
pub(crate) fn check() -> Result<(), String> {
    eprintln!("==> file-size");
    let root = crate::root();
    let text = std::fs::read_to_string(root.join(ALLOWLIST)).map_err(|e| format!("read {ALLOWLIST}: {e}"))?;
    let listed = parse_allowlist(&text)?;
    let mut errors = Vec::new();
    let mut notes = Vec::new();
    let mut seen = BTreeSet::new();
    for dir in DIRS {
        crate::walk(&root.join(dir), &mut |p| {
            if !p.extension().is_some_and(|e| e == "rs") {
                return;
            }
            let rel = crate::rel(p);
            match std::fs::read_to_string(p) {
                Ok(source) => match verdict(line_count(&source), listed.get(&rel).copied()) {
                    Verdict::Fine => {}
                    Verdict::Note(note) => notes.push(format!("{rel}: {note}")),
                    Verdict::Fail(reason) => errors.push(format!("{rel}: {reason}")),
                },
                Err(e) => errors.push(format!("{rel}: cannot read: {e}")),
            }
            seen.insert(rel);
        });
    }
    let missing = listed.keys().filter(|path| !seen.contains(*path));
    errors.extend(missing.map(|path| format!("{ALLOWLIST} lists {path}, which does not exist: remove the entry")));
    // The directory walk has no stable order; the report does.
    notes.sort();
    errors.sort();
    notes.iter().for_each(|note| eprintln!("note: {note}"));
    if errors.is_empty() { Ok(()) } else { Err(errors.join("\n")) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fails_with(v: &Verdict, text: &str) -> bool {
        matches!(v, Verdict::Fail(reason) if reason.contains(text))
    }

    #[test]
    fn unlisted_file_over_the_limit_fails() {
        assert!(fails_with(&verdict(801, None), "over 800"));
    }

    #[test]
    fn unlisted_file_at_the_limit_passes() {
        assert_eq!(verdict(800, None), Verdict::Fine);
    }

    #[test]
    fn listed_file_with_its_recorded_count_passes() {
        assert_eq!(verdict(1045, Some(1045)), Verdict::Fine);
    }

    #[test]
    fn listed_file_that_grew_fails() {
        assert!(fails_with(&verdict(1046, Some(1045)), "grew from 1045 to 1046"));
    }

    #[test]
    fn listed_file_that_is_now_small_fails_with_the_removal_message() {
        assert!(fails_with(&verdict(790, Some(1045)), "remove it from xtask/file-size.txt"));
        assert!(fails_with(&verdict(800, Some(1045)), "remove it from xtask/file-size.txt"));
    }

    #[test]
    fn listed_file_that_shrank_but_is_still_over_passes_with_a_note() {
        let v = verdict(900, Some(1045));
        assert!(matches!(&v, Verdict::Note(note) if note.contains("shrank from 1045 to 900")), "{v:?}");
        assert!(matches!(verdict(801, Some(802)), Verdict::Note(_)));
    }

    #[test]
    fn a_line_without_a_final_newline_is_not_counted() {
        assert_eq!(line_count("a\nb\n"), 2);
        assert_eq!(line_count("a\r\nb"), 1);
        assert_eq!(line_count(""), 0);
    }

    #[test]
    fn allowlist_rejects_hostile_lines() {
        for text in
            ["crates/a.rs many\n", "crates/a.rs -3\n", "crates/a.rs 900 extra\n", "crates/a.rs 900\ncrates/a.rs 901\n"]
        {
            assert!(parse_allowlist(text).is_err(), "{text:?} was accepted");
        }
        let odd = parse_allowlist("/etc/passwd 900\n../x.rs 900\n").expect("paths are only names here");
        assert_eq!(odd.len(), 2, "such entries match no checked file and are reported as not existing");
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

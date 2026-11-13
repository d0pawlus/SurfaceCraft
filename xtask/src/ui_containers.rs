//! `cargo xtask ui-containers`: the UI conventions of the Craft family that a compiler cannot see.
//!
//! - Colours come from the theme tokens: outside `theme.rs` (and test code) `crates/ui-egui/src`
//!   has no raw colour: no `Color32::from_*`, no colour constant such as `Color32::RED`, no array
//!   of three or four floats in 0..=1 (a colour in shader or buffer form).
//! - One outer container: the window has exactly one `egui::CentralPanel` and one `DockArea`
//!   (panels live inside the dock layout, not in competing panels).
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

const SRC: &str = "crates/ui-egui/src";
/// Files that may spell colours: the theme resolves tokens to colours.
const COLOUR_FILES: &[&str] = &["theme.rs"];
/// The outer containers and the most of each the shell may have.
const CONTAINERS: &[(&str, usize)] = &[("CentralPanel::", 1), ("DockArea::new(", 1)];

/// Whether the attribute line `t` gates its item to test builds.
fn is_test_cfg(t: &str) -> bool {
    t.starts_with("#[cfg(test)]") || t.starts_with("#[cfg(all(test")
}

/// The name of the out-of-line module that the line `t` declares (`mod name;`, any visibility).
fn declared_module(t: &str) -> Option<&str> {
    let at = t.find("mod ")?;
    let before = t.get(..at)?.trim();
    if !before.is_empty() && !before.starts_with("pub") {
        return None;
    }
    t.get(at + "mod ".len()..)?.strip_suffix(';').map(str::trim)
}

/// The names of the modules a file declares out of line under a test `cfg` (`#[cfg(test)]`, further
/// attributes, then `mod name;`).
fn test_module_names(source: &str) -> Vec<String> {
    let mut names = Vec::new();
    let mut lines = source.lines().map(str::trim).peekable();
    while let Some(t) = lines.next() {
        if !is_test_cfg(t) {
            continue;
        }
        while lines.peek().is_some_and(|l| l.starts_with("#[")) {
            lines.next();
        }
        if let Some(name) = lines.peek().and_then(|l| declared_module(l)) {
            names.push(name.to_string());
        }
    }
    names
}

/// The directory in which the child modules of `file` live.
fn module_dir(file: &Path) -> PathBuf {
    let stem = file.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
    let parent = file.parent().map(Path::to_path_buf).unwrap_or_default();
    if matches!(stem.as_str(), "mod" | "lib" | "main") { parent } else { parent.join(stem) }
}

/// The files that are test code as a whole: modules their parent declares under a test `cfg`. A
/// file is not test code because of its name.
fn test_modules(files: &[PathBuf]) -> BTreeSet<PathBuf> {
    let mut out = BTreeSet::new();
    for f in files {
        let Ok(source) = std::fs::read_to_string(f) else { continue };
        let dir = module_dir(f);
        for name in test_module_names(&source) {
            out.insert(dir.join(format!("{name}.rs")));
            out.insert(dir.join(&name).join("mod.rs"));
        }
    }
    out
}

/// The code of a source file: comments removed, the declaration of an out-of-line test module
/// skipped, and everything from an inline test item on cut off (test code may use any colour).
fn code_of(source: &str) -> String {
    let mut out = String::new();
    let mut lines = source.lines().peekable();
    while let Some(line) = lines.next() {
        let t = line.trim_start();
        if is_test_cfg(t) {
            while lines.peek().is_some_and(|l| l.trim_start().starts_with("#[")) {
                lines.next();
            }
            if lines.peek().is_some_and(|l| declared_module(l.trim()).is_some()) {
                // only the declaration is test code here; what follows it is checked
                lines.next();
                continue;
            }
            break;
        }
        // a vector that only looks like a colour (a unit direction) says so
        if line.contains("// not a colour") {
            continue;
        }
        let code = line.split("//").next().unwrap_or("");
        out.push_str(code);
        out.push('\n');
    }
    out
}

/// Whether `line` holds a colour literal.
fn raw_colour(line: &str) -> Option<String> {
    let mut rest = line;
    while let Some(i) = rest.find("Color32::") {
        let after = &rest[i + "Color32::".len()..];
        let ident: String = after.chars().take_while(|c| c.is_alphanumeric() || *c == '_').collect();
        if ident.starts_with("from_")
            || (!ident.is_empty() && ident.chars().all(|c| c.is_ascii_uppercase() || c == '_' || c.is_ascii_digit()))
        {
            return Some(format!("Color32::{ident}"));
        }
        rest = after;
    }
    if line.contains("hex_color!") {
        return Some("hex_color!".into());
    }
    // `[0.2, 0.8, 0.2, 1.0]`: three or four floats in 0..=1
    let mut s = line;
    while let Some(open) = s.find('[') {
        let after = &s[open + 1..];
        if let Some(close) = after.find(']') {
            let items: Vec<&str> = after[..close].split(',').map(str::trim).collect();
            let floats: Vec<f32> = items
                .iter()
                .filter(|t| t.contains('.'))
                .filter_map(|t| t.trim_end_matches("f32").parse::<f32>().ok())
                .collect();
            if (items.len() == 3 || items.len() == 4)
                && floats.len() == items.len()
                && floats.iter().all(|f| (0.0..=1.0).contains(f))
            {
                return Some(format!("[{}]", after[..close].trim()));
            }
            s = &after[close..];
        } else {
            break;
        }
    }
    None
}

fn sources(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            sources(&p, out);
        } else if p.extension().is_some_and(|x| x == "rs") {
            out.push(p);
        }
    }
}

pub(crate) fn check() -> Result<(), String> {
    eprintln!("==> ui-containers");
    let root = crate::root();
    let mut files = Vec::new();
    sources(&root.join(SRC), &mut files);
    files.sort();
    let tests = test_modules(&files);
    let mut errors = Vec::new();
    let mut counts = vec![0usize; CONTAINERS.len()];
    for f in files.iter().filter(|f| !tests.contains(*f)) {
        let rel = crate::rel(f);
        let name = f.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        let source = std::fs::read_to_string(f).map_err(|e| format!("{rel}: {e}"))?;
        let code = code_of(&source);
        for (n, line) in code.lines().enumerate() {
            if !COLOUR_FILES.contains(&name.as_str())
                && let Some(c) = raw_colour(line)
            {
                errors.push(format!("{rel}:{}: raw colour {c} (use a theme token)", n + 1));
            }
            for (k, (pat, _)) in CONTAINERS.iter().enumerate() {
                counts[k] += line.matches(pat).count();
            }
        }
    }
    for (k, (pat, max)) in CONTAINERS.iter().enumerate() {
        if counts[k] > *max {
            errors.push(format!("{} uses of `{pat}` in {SRC} (at most {max}: one outer container)", counts[k]));
        }
    }
    if errors.is_empty() { Ok(()) } else { Err(errors.join("\n")) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_raw_colours_and_ignores_types() {
        assert!(raw_colour("let c = Color32::from_rgb(1, 2, 3);").is_some());
        assert!(raw_colour("let c = egui::Color32::RED;").is_some());
        assert!(raw_colour("let c = [0.2, 0.8, 0.2, 1.0];").is_some());
        assert!(raw_colour("fn f(tint: egui::Color32) {}").is_none());
        assert!(raw_colour("let v = [1.0, 2.0, 3.0];").is_none());
        assert!(raw_colour("let v = [0, 1, 2];").is_none());
    }

    #[test]
    fn code_skips_comments_and_tests() {
        let c = code_of("a // Color32::RED\n#[cfg(test)]\nmod t { Color32::RED }");
        assert!(!c.contains("RED"));
    }

    #[test]
    fn code_after_the_declaration_of_a_test_module_is_still_checked() {
        let source = "mod a;\n#[cfg(test)]\n#[allow(clippy::unwrap_used)]\nmod tests;\nlet c = Color32::RED;\n";
        let c = code_of(source);
        assert!(c.contains("Color32::RED"), "{c}");
        assert!(!c.contains("mod tests"), "{c}");
        let gated = code_of("#[cfg(all(test, not(target_arch = \"wasm32\")))]\nmod tests;\nlet c = Color32::RED;\n");
        assert!(gated.contains("Color32::RED"), "{gated}");
    }

    #[test]
    fn a_file_is_test_code_only_when_its_parent_declares_it_under_a_test_cfg() {
        let parent = "mod rows;\n#[cfg(test)]\n#[allow(clippy::panic)]\nmod tests;\npub(crate) mod widgets;\n";
        assert_eq!(test_module_names(parent), vec!["tests".to_string()]);
        assert!(test_module_names("mod tests;\n#[cfg(test)]\nfn helper() {}\n").is_empty(), "no cfg on the module");
        assert_eq!(declared_module("pub(crate) mod a;"), Some("a"));
        assert_eq!(declared_module("mod a {"), None);
        assert_eq!(declared_module("let model = 1;"), None);
        assert_eq!(module_dir(Path::new("src/forms.rs")), Path::new("src/forms"));
        assert_eq!(module_dir(Path::new("src/sketch_editor/mod.rs")), Path::new("src/sketch_editor"));
    }
}

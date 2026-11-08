//! `cargo xtask assets`: every non-code file has a row in ATTRIBUTION.md.
use std::process::Command;

/// Extensions and names that are code, configuration or documentation, not assets.
const NOT_ASSETS: &[&str] = &["rs", "toml", "lock", "md", "yml", "yaml", "gitignore", "gitattributes"];
const ROOT_FILES: &[&str] = &["LICENSE-APACHE", "LICENSE-MIT", "NOTICE", "THIRD-PARTY-LICENSES.txt"];

fn glob_match(pat: &str, path: &str) -> bool {
    if let Some(prefix) = pat.strip_suffix("/**") {
        return path.starts_with(&format!("{prefix}/"));
    }
    if let Some((dir, rest)) = pat.rsplit_once('/')
        && let Some(ext) = rest.strip_prefix("*.")
    {
        return path.rsplit_once('/').is_some_and(|(d, f)| d == dir && f.ends_with(&format!(".{ext}")));
    }
    pat == path
}

pub(crate) fn check() -> Result<(), String> {
    eprintln!("==> assets");
    let root = crate::root();
    let attribution =
        std::fs::read_to_string(root.join("ATTRIBUTION.md")).map_err(|e| format!("ATTRIBUTION.md: {e}"))?;
    let patterns: Vec<String> = attribution
        .lines()
        .filter(|l| l.starts_with('|'))
        .filter_map(|l| l.split('|').nth(1))
        .map(|c| c.trim().trim_matches('`').to_string())
        .filter(|c| !c.is_empty() && c != "File" && !c.starts_with("---"))
        .collect();
    let out = Command::new("git")
        .current_dir(&root)
        .args(["ls-files", "--cached", "--others", "--exclude-standard"])
        .output()
        .map_err(|e| format!("git ls-files: {e}"))?;
    let mut missing = Vec::new();
    for f in String::from_utf8_lossy(&out.stdout).lines() {
        let f = f.trim();
        if f.is_empty() || f.starts_with(".github/") || f.starts_with(".claude/") || f.starts_with(".cargo/") {
            continue;
        }
        let name = f.rsplit('/').next().unwrap_or(f);
        let ext = name.rsplit_once('.').map(|(_, e)| e).unwrap_or(name);
        if NOT_ASSETS.contains(&ext) || ROOT_FILES.contains(&f) || name.starts_with('.') {
            continue;
        }
        if !root.join(f).exists() {
            continue;
        }
        if !patterns.iter().any(|p| glob_match(p, f)) {
            missing.push(f.to_string());
        }
    }
    if missing.is_empty() {
        Ok(())
    } else {
        Err(format!("files without an ATTRIBUTION.md row:\n  {}", missing.join("\n  ")))
    }
}

#[cfg(test)]
mod tests {
    use super::glob_match;

    #[test]
    fn globs() {
        assert!(glob_match("docs/**", "docs/a/b.png"));
        assert!(glob_match("crates/ui-egui/src/shaders/*.wgsl", "crates/ui-egui/src/shaders/mesh.wgsl"));
        assert!(!glob_match("crates/ui-egui/src/shaders/*.wgsl", "crates/ui-egui/src/x/mesh.wgsl"));
        assert!(glob_match("NOTICE", "NOTICE"));
    }
}

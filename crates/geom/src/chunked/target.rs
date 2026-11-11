//! Where a store is built and how it is installed: the build writes into a sibling work directory
//! (`<dir>.building`) and moves the finished files into the target only when everything succeeded, so
//! a failed import never touches an earlier store. Only files named like the store's own are ever
//! deleted.
use super::StoreError;
use super::format::INDEX_FILE;
use std::path::{Path, PathBuf};

/// Suffix of the work directory next to the target.
const WORK_SUFFIX: &str = ".building";

/// Is `name` a file the store build writes?
pub(crate) fn is_store_file(name: &str) -> bool {
    let numbered = |prefix: &str, suffix: &str| {
        name.strip_prefix(prefix)
            .and_then(|r| r.strip_suffix(suffix))
            .is_some_and(|n| !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()))
    };
    name == INDEX_FILE || numbered("mesh-", ".dat") || numbered("map-", ".dat") || numbered("spill-", ".tmp")
}

fn io(dir: &Path, e: &std::io::Error) -> StoreError {
    StoreError::Io(format!("{}: {e}", dir.display()))
}

fn names_in(dir: &Path) -> Result<Vec<String>, StoreError> {
    Ok(std::fs::read_dir(dir)
        .map_err(|e| io(dir, &e))?
        .filter_map(|e| e.ok().map(|e| e.file_name().to_string_lossy().into_owned()))
        .collect())
}

/// Checks that `dir` can receive a store (a path, not a file; empty, or holding nothing but a store
/// that `overwrite` may replace) and returns the work directory, freshly created and empty.
pub(crate) fn prepare(dir: &Path, overwrite: bool) -> Result<PathBuf, StoreError> {
    if dir.as_os_str().is_empty() {
        return Err(StoreError::Params("the store directory path is empty".into()));
    }
    let name =
        dir.file_name().ok_or_else(|| StoreError::Params(format!("{} has no final directory name", dir.display())))?;
    if dir.exists() {
        if !dir.is_dir() {
            return Err(StoreError::Params(format!("{} is a file, not a directory", dir.display())));
        }
        check_replaceable(dir, overwrite)?;
    }
    let work = dir.with_file_name(format!("{}{WORK_SUFFIX}", name.to_string_lossy()));
    if work.exists() {
        // left by a crashed build: reuse it when it holds nothing but store files
        if !work.is_dir() || names_in(&work)?.iter().any(|n| !is_store_file(n)) {
            return Err(StoreError::Params(format!("{} exists and is not a store work directory", work.display())));
        }
        remove_store_files(&work);
    } else {
        std::fs::create_dir_all(&work).map_err(|e| io(&work, &e))?;
    }
    Ok(work)
}

fn check_replaceable(dir: &Path, overwrite: bool) -> Result<(), StoreError> {
    let names = names_in(dir)?;
    if names.is_empty() {
        return Ok(());
    }
    if !overwrite {
        return Err(StoreError::Params(format!(
            "{} is not empty; pass overwrite to replace a store there",
            dir.display()
        )));
    }
    match names.iter().find(|n| !is_store_file(n)) {
        Some(other) => Err(StoreError::Params(format!(
            "{} holds {other}, which is not a store file; nothing was deleted",
            dir.display()
        ))),
        None => Ok(()),
    }
}

/// Moves the finished store from `work` into `dir` (data files first, the index last) and removes
/// `work`. The earlier store in `dir`, if any, is removed just before.
pub(crate) fn install(work: &Path, dir: &Path) -> Result<(), StoreError> {
    if dir.exists() {
        remove_store_files(dir);
    } else {
        std::fs::create_dir_all(dir).map_err(|e| io(dir, &e))?;
    }
    let mut names = names_in(work)?;
    names.sort_by_key(|n| n == INDEX_FILE);
    for n in names {
        std::fs::rename(work.join(&n), dir.join(&n)).map_err(|e| io(dir, &e))?;
    }
    std::fs::remove_dir(work).map_err(|e| io(work, &e))
}

/// Throws the work directory away after a failure (best effort).
pub(crate) fn discard(work: &Path) {
    remove_store_files(work);
    let _ = std::fs::remove_dir(work);
}

/// Removes the files of a store (best effort), nothing else.
pub(crate) fn remove_store_files(dir: &Path) {
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    for e in rd.flatten() {
        if is_store_file(&e.file_name().to_string_lossy()) {
            let _ = std::fs::remove_file(e.path());
        }
    }
}

/// Bytes of the store files in `dir`.
pub(crate) fn dir_bytes(dir: &Path) -> u64 {
    let Ok(rd) = std::fs::read_dir(dir) else { return 0 };
    rd.flatten()
        .filter(|e| is_store_file(&e.file_name().to_string_lossy()))
        .filter_map(|e| e.metadata().ok())
        .map(|m| m.len())
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chunked::spill::spill_name;

    #[test]
    fn only_our_own_file_names_count_as_store_files() {
        assert!(is_store_file("index.scx") && is_store_file("mesh-0.dat") && is_store_file("map-12.dat"));
        assert!(is_store_file(&spill_name(3)));
        for other in ["notes.txt", "mesh-.dat", "mesh-x.dat", "mesh-1.dat.bak", "index.scx.old", "spill-1.tmp.x"] {
            assert!(!is_store_file(other), "{other}");
        }
    }

    #[test]
    fn empty_and_file_paths_are_refused() {
        assert!(matches!(prepare(Path::new(""), false), Err(StoreError::Params(_))));
        assert!(matches!(prepare(Path::new("/"), false), Err(StoreError::Params(_))));
        let f = std::env::temp_dir().join(format!("sc-target-file-{}", std::process::id()));
        std::fs::write(&f, "x").unwrap();
        assert!(matches!(prepare(&f, true), Err(StoreError::Params(_))));
        let _ = std::fs::remove_file(&f);
    }
}

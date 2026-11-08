//! `cargo xtask layers`: crate dependencies only point down the layer table.
use serde_json::Value;
use std::process::Command;

/// (crate directory name, layer). Apps are the top layer.
const LAYERS: &[(&str, u8)] = &[
    ("surfacecraft-geom", 0),
    ("surfacecraft-kernel", 1),
    ("surfacecraft-fit", 1),
    ("surfacecraft-align", 1),
    ("surfacecraft-sketch", 1),
    ("surfacecraft-deviation", 2),
    ("surfacecraft-doc", 2),
    ("surfacecraft-io", 3),
    ("surfacecraft-engine", 4),
    ("surfacecraft-mcp", 5),
    ("surfacecraft-ai", 5),
    ("surfacecraft-ui-egui", 5),
    ("surfacecraft", 6),
    ("surfacecraft-cli", 6),
    ("surfacecraft-web", 6),
];

/// Within a layer, a crate may depend only on crates listed before it here.
const INTRA_LAYER_ORDER: &[&str] = &[
    "surfacecraft-kernel",
    "surfacecraft-fit",
    "surfacecraft-align",
    "surfacecraft-sketch",
    "surfacecraft-deviation",
    "surfacecraft-doc",
    "surfacecraft-mcp",
    "surfacecraft-ai",
    "surfacecraft-ui-egui",
];

/// UI crates, allowed only from layer 5 up.
const UI_CRATES: &[&str] = &["egui", "eframe", "egui-wgpu", "wgpu", "winit", "egui_kittest", "rfd"];

/// solidcore crates, allowed only in the kernel adapter.
const SOLIDCORE: &[&str] = &[
    "solidcore",
    "solidcore-ops",
    "solidcore-record",
    "solidcore-convert",
    "solidcore-tess",
    "step-brep",
    "step-export",
    "step-import",
];

fn layer(name: &str) -> Option<u8> {
    LAYERS.iter().find(|(n, _)| *n == name).map(|(_, l)| *l)
}

fn order(name: &str) -> Option<usize> {
    INTRA_LAYER_ORDER.iter().position(|n| *n == name)
}

pub(crate) fn check() -> Result<(), String> {
    eprintln!("==> layers");
    let out = Command::new(std::env::var("CARGO").unwrap_or_else(|_| "cargo".into()))
        .current_dir(crate::root())
        .args(["metadata", "--format-version", "1", "--no-deps"])
        .output()
        .map_err(|e| format!("cargo metadata: {e}"))?;
    let meta: Value = serde_json::from_slice(&out.stdout).map_err(|e| format!("cargo metadata output: {e}"))?;
    let mut errors = Vec::new();
    for pkg in meta["packages"].as_array().into_iter().flatten() {
        let name = pkg["name"].as_str().unwrap_or("");
        if name == "xtask" {
            continue;
        }
        let Some(my) = layer(name) else {
            errors.push(format!("{name} is not in the layer table (xtask/src/layers.rs)"));
            continue;
        };
        for dep in pkg["dependencies"].as_array().into_iter().flatten() {
            let dn = dep["name"].as_str().unwrap_or("");
            let dev = dep["kind"].as_str() == Some("dev");
            if UI_CRATES.contains(&dn) && my < 5 {
                errors.push(format!("{name} (layer {my}) depends on UI crate {dn}"));
            }
            if SOLIDCORE.contains(&dn) && name != "surfacecraft-kernel" {
                errors.push(format!("{name} depends on {dn}; only surfacecraft-kernel may use solidcore"));
            }
            if let Some(dl) = layer(dn) {
                if dev {
                    continue;
                }
                let ok = dl < my || (dl == my && matches!((order(dn), order(name)), (Some(a), Some(b)) if a < b));
                if !ok {
                    errors.push(format!("{name} (layer {my}) must not depend on {dn} (layer {dl})"));
                }
            }
        }
    }
    if errors.is_empty() { Ok(()) } else { Err(errors.join("\n")) }
}

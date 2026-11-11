//! MCP prompts and the server instructions: the workflow an agent should follow, in words.
use serde_json::{Value, json};

/// The prompt that walks an agent from a scan to an exported solid.
pub const SCAN_TO_SOLID: &str = "scan_to_solid";

/// URI prefix of a guide resource (`surfacecraft://guide/<id>`).
pub const GUIDE_URI: &str = "surfacecraft://guide/";

/// What `initialize` tells the agent. `files` says whether path parameters work in this session
/// (`None`: decided by the app the server is bridged to).
#[must_use]
pub fn instructions(files: Option<bool>) -> String {
    let files = match files {
        Some(true) => "File paths are allowed: pass `path` to mesh_import and to the exports.",
        Some(false) => {
            "File paths are off: pass base64 `data` to mesh_import and read `data` from the exports (start the server with --allow-files to use paths)."
        }
        None => "Bridged to the running app: the user sees every change; file paths follow the app's setting.",
    };
    format!(
        "SurfaceCraft turns 3D scans into CAD models. Units are mm. \
The short path is three calls: mesh_import, then scan_to_solid (cleans, segments, fits, builds the solid and measures it against the scan), then export_step. \
Read `verdict` of scan_to_solid: on `pass` export; on `review` or `fail` follow its `next_steps` (each has a command and parameters) and guide_read topic scan-to-solid. \
Never report a model as done without a `pass` or a body_compare you have read. \
{files} \
Every other command is a verb of an area tool (e.g. tool `region`, verb `merge`, `params`); a wrong parameter returns the command's input schema. \
Several steps: app_batch. A long call: `queued` true, then job_wait."
    )
}

/// `prompts/list`.
#[must_use]
pub fn list() -> Value {
    json!({"prompts": [{
        "name": SCAN_TO_SOLID,
        "title": "Scan to solid",
        "description": "Turn a 3D scan into a verified CAD solid and export it as STEP.",
        "arguments": [
            {"name": "scan", "description": "Path of the scan file (STL, OBJ, PLY, XYZ), or the name of a mesh already in the design.", "required": false},
            {"name": "tolerance", "description": "Tolerance the solid is held to against the scan, in mm (default 0.2).", "required": false}
        ]
    }]})
}

/// `prompts/get`: the messages of prompt `name`, or `None` for an unknown prompt.
#[must_use]
pub fn get(name: &str, args: &Value) -> Option<Value> {
    if name != SCAN_TO_SOLID {
        return None;
    }
    let arg = |k: &str| args.get(k).and_then(Value::as_str).map(str::trim).filter(|v| !v.is_empty());
    let scan = arg("scan").map_or_else(
        || {
            "the scan the user names (ask for it if there is none in the design; inspect_design lists the meshes)"
                .to_string()
        },
        |s| format!("`{s}`"),
    );
    let tolerance = arg("tolerance").unwrap_or("0.2");
    let text = format!(
        "Rebuild {scan} as a CAD solid with SurfaceCraft, held to {tolerance} mm against the scan.\n\
1. If it is a file, load it with mesh_import (`path`, or base64 `data` when paths are off).\n\
2. Call scan_to_solid with `scan` and `tolerance`. Use `queued: true` and job_wait for a scan of more than a million triangles.\n\
3. On `verdict: pass`, call export_step with the parameters of `next_steps[0]` and report the volume, the face count, and the deviation (`within`, `rms`, `max_abs`).\n\
4. On `review` or `fail`, read `issues` and run `next_steps` in order; guide_read topic scan-to-solid explains each issue. Correct the regions (tool `region`), then call scan_to_solid with `reuse_regions: true`. Stop after three attempts and report what is left.\n\
5. Do not call the model done without a `pass` verdict or a body_compare result you have read; say plainly when it is a `review`."
    );
    Some(json!({
        "description": "Scan to a verified solid",
        "messages": [{"role": "user", "content": {"type": "text", "text": text}}]
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_prompt_names_the_scan_and_the_tolerance() {
        let p = get(SCAN_TO_SOLID, &json!({"scan": "part.stl", "tolerance": "0.1"})).expect("a prompt");
        let text = p["messages"][0]["content"]["text"].as_str().expect("text");
        assert!(text.contains("`part.stl`") && text.contains("0.1 mm"), "{text}");
        assert!(get("nope", &Value::Null).is_none());
        assert!(get(SCAN_TO_SOLID, &json!({"scan": 5})).is_some(), "a hostile argument falls back");
    }

    #[test]
    fn the_instructions_say_how_files_are_passed() {
        assert!(instructions(Some(true)).contains("pass `path`"));
        assert!(instructions(Some(false)).contains("--allow-files"));
        assert!(instructions(None).contains("Bridged"));
    }
}

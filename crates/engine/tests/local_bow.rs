//! The "bow" of large regions on real scans that stay outside the repository: for each large region
//! of the plain segmentation (`freeform: false`) the residual to its primitive (`r_p`), the residual
//! to second-order patches (`r_q`) and their difference in quadrature (bow), all in numbers.
//!
//! `SURFACECRAFT_LOCAL_CORPUS` names the directory (STL/OBJ/PLY, never committed); scans are
//! reported by number, `SURFACECRAFT_LOCAL_CORPUS_NUMBERS=1,4` picks some of them. Run:
//! `SURFACECRAFT_LOCAL_CORPUS=… cargo test --profile ci -p surfacecraft-engine --test local_bow -- --ignored --nocapture`
use serde_json::json;
use surfacecraft_engine::Session;
use surfacecraft_fit::Primitive;
use surfacecraft_fit::segment::{protected_by_tolerance, region_bow, residual_spread};

fn corpus() -> Vec<(usize, std::path::PathBuf)> {
    let Ok(dir) = std::env::var("SURFACECRAFT_LOCAL_CORPUS") else { return Vec::new() };
    let mut files: Vec<_> =
        std::fs::read_dir(dir).map(|rd| rd.filter_map(Result::ok).map(|e| e.path()).collect()).unwrap_or_default();
    files.retain(|p| {
        p.extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| matches!(e.to_ascii_lowercase().as_str(), "stl" | "obj" | "ply"))
    });
    files.sort();
    let wanted: Option<Vec<usize>> = std::env::var("SURFACECRAFT_LOCAL_CORPUS_NUMBERS")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect());
    files
        .into_iter()
        .enumerate()
        .map(|(k, f)| (k + 1, f))
        .filter(|(k, _)| wanted.as_ref().is_none_or(|w| w.contains(k)))
        .collect()
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_bow_table() {
    for (number, path) in corpus() {
        let mut s = Session::new();
        s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).expect("import");
        s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).expect("clean");
        let out =
            s.execute("region.auto_segment", json!({"mesh": "scan", "options": {"freeform": false}})).expect("segment");
        let mesh = s.doc.find_by_name("scan").and_then(|(id, _)| s.doc.mesh(id)).expect("mesh").mesh.clone();
        let area: f64 = (0..mesh.triangle_count()).map(|t| mesh.face_area(t)).sum();
        let mut listed = 0;
        for r in out["regions"].as_array().into_iter().flatten() {
            let id = surfacecraft_doc::ObjectId(r["id"].as_u64().unwrap_or(0));
            let Some(triangles) = s.doc.region(id).map(|r| r.triangles.clone()) else { continue };
            let region_area: f64 = triangles.iter().map(|&t| mesh.face_area(t as usize)).sum();
            if region_area < 0.01 * area || listed >= 12 {
                continue;
            }
            let fit = s.execute("primitive.fit", json!({"region": r["id"]}));
            let Ok(fit) = fit else { continue };
            let Ok(prim) = serde_json::from_value::<Primitive>(fit["primitive"].clone()) else { continue };
            let bow = region_bow(&mesh, &triangles, &prim);
            let seg = surfacecraft_fit::segment::Segment {
                triangles: triangles.to_vec(),
                primitive: prim.clone(),
                rms: r["rms"].as_f64().unwrap_or(0.0),
                freeform: false,
            };
            let spread = residual_spread(&mesh, &triangles, &prim);
            let prot = protected_by_tolerance(&mesh, &seg, 0.2, 0.0);
            listed += 1;
            eprintln!(
                "B{number:02} {:?} area {:.2} % rms {:.4} spread {:.4} r_p {:.4} r_q {:.4} bow {:.4} extent {:.1} points {} protected {prot}",
                prim.kind(),
                100.0 * region_area / area,
                seg.rms,
                spread,
                bow.map_or(f64::NAN, |b| b.primitive_rms),
                bow.map_or(f64::NAN, |b| b.quadric_rms),
                bow.map_or(f64::NAN, |b| b.bow()),
                bow.map_or(f64::NAN, |b| b.extent),
                bow.map_or(0, |b| b.points),
            );
        }
        eprintln!(
            "B{number:02} triangles {} area {area:.0} regions {}",
            mesh.triangle_count(),
            out["regions"].as_array().map_or(0, Vec::len)
        );
        // the default segmentation: which large regions survive, and how much is freeform
        let new = s.execute("region.auto_segment", json!({"mesh": "scan"})).expect("segment");
        let (mut free, mut kept) = (0.0, Vec::new());
        for r in new["regions"].as_array().into_iter().flatten() {
            let id = surfacecraft_doc::ObjectId(r["id"].as_u64().unwrap_or(0));
            let Some(triangles) = s.doc.region(id).map(|r| r.triangles.clone()) else { continue };
            let region_area: f64 = triangles.iter().map(|&t| mesh.face_area(t as usize)).sum();
            if r["freeform"] == true {
                free += region_area / area;
            } else if region_area >= 0.01 * area {
                kept.push(format!(
                    "{} {:.2} % rms {:.4}",
                    r["kind"],
                    100.0 * region_area / area,
                    r["rms"].as_f64().unwrap_or(0.0)
                ));
            }
        }
        eprintln!(
            "B{number:02} default mode: {} regions, freeform share {free:.3}, noise {} {}; large primitives kept: {kept:?}",
            new["regions"].as_array().map_or(0, Vec::len),
            new["noise_used"],
            new["noise_source"]
        );
    }
}

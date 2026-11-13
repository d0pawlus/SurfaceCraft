//! P9-CLEAN: `mesh.clean` and `mesh.fill_holes` never raise the number of edges shared by three
//! or more triangles, whatever the mesh (random triangle soups with repeated and pinched
//! triangles, out-of-range indices, NaN coordinates), and never panic.
use serde_json::json;
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::Session;
use surfacecraft_geom::quality::edge_census;
use surfacecraft_geom::{Mesh, Point3, Rng};

fn soup(rng: &mut Rng, hostile: bool) -> Mesh {
    let nv = 6 + rng.index(24);
    let mut positions: Vec<Point3> = (0..nv)
        .map(|_| Point3::new(rng.range(0.0, 4.0).floor(), rng.range(0.0, 4.0).floor(), rng.range(0.0, 2.0).floor()))
        .collect();
    let mut triangles: Vec<[u32; 3]> =
        (0..8 + rng.index(80)).map(|_| [rng.index(nv) as u32, rng.index(nv) as u32, rng.index(nv) as u32]).collect();
    if let Some(&dup) = triangles.first() {
        triangles.push(dup);
        triangles.push([dup[1], dup[0], dup[2]]);
    }
    if hostile && rng.index(2) == 0 {
        triangles.push([0, 1, nv as u32 + 7]);
    }
    if hostile
        && rng.index(2) == 0
        && let Some(p) = positions.get_mut(rng.index(nv))
    {
        *p = Point3::new(f64::NAN, 0.0, 0.0);
    }
    Mesh { positions, triangles }
}

/// The non-manifold edges the triangles that can be read at all (corners in range and finite)
/// have: the census of an invalid mesh is empty, so this is the fair figure to start from.
fn readable_non_manifold(mesh: &Mesh) -> usize {
    let ok = |&v: &u32| mesh.positions.get(v as usize).is_some_and(|p| p.coords.iter().all(|c| c.is_finite()));
    let triangles = mesh.triangles.iter().filter(|t| t.iter().all(ok)).copied().collect();
    edge_census(&Mesh {
        positions: mesh
            .positions
            .iter()
            .map(|p| if p.coords.iter().all(|c| c.is_finite()) { *p } else { Point3::origin() })
            .collect(),
        triangles,
    })
    .non_manifold
}

fn non_manifold(s: &Session, id: ObjectId) -> usize {
    s.doc.mesh(id).map_or(0, |m| edge_census(&m.mesh).non_manifold)
}

#[test]
fn clean_and_fill_holes_never_add_non_manifold_edges_to_random_meshes() {
    let mut rng = Rng::new(9_000_017);
    for round in 0..150 {
        let mesh = soup(&mut rng, round % 3 == 2);
        let before = readable_non_manifold(&mesh);
        let mut s = Session::new();
        let id = s.doc.add_mesh("scan", mesh);
        let params = if round % 2 == 0 {
            json!({"object": "scan", "fill_holes_max_edges": 300, "min_component_triangles": 0,
                   "min_component_fraction": 0.0, "outlier_ratio": 0.0})
        } else {
            json!({"object": "scan", "fill_holes_max_edges": 300})
        };
        // an error is acceptable (a hostile mesh), a panic or a worse mesh is not
        let _ = s.execute("mesh.clean", params);
        let after = non_manifold(&s, id);
        assert!(after <= before, "round {round}: mesh.clean {before} -> {after}");
        let before = after;
        let smooth = round % 4 < 2;
        let _ = s.execute("mesh.fill_holes", json!({"object": "scan", "max_edges": 300, "smooth": smooth}));
        let after = non_manifold(&s, id);
        assert!(after <= before, "round {round}: mesh.fill_holes {before} -> {after}");
    }
}

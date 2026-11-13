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

/// `n x n` flat grid of unit squares; vertex `(i, j)` is `j * (n + 1) + i`.
fn grid(n: u32) -> Mesh {
    let w = n + 1;
    let positions = (0..w * w).map(|k| Point3::new(f64::from(k % w), f64::from(k / w), 0.0)).collect();
    let mut triangles = Vec::new();
    for j in 0..n {
        for i in 0..n {
            let (a, b, c, d) = (j * w + i, j * w + i + 1, (j + 1) * w + i + 1, (j + 1) * w + i);
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    Mesh { positions, triangles }
}

/// The square hole of vertices 6, 7, 12, 11 in a 4x4 grid with a flap hanging on the diagonal
/// 6-12. The old fill clipped the ear on that diagonal (an edge of the mesh already) and made it
/// non-manifold: the geom test `a_hole_with_one_free_diagonal_is_filled_with_it` measured 3
/// non-manifold edges on this very mesh before the fix.
fn hole_next_to_a_flap() -> Mesh {
    let mut m = grid(4);
    let cell = 5;
    let mut k = 0;
    m.triangles.retain(|_| {
        k += 1;
        k - 1 != 2 * cell && k - 1 != 2 * cell + 1
    });
    m.positions.push(Point3::new(1.5, 1.5, 3.0));
    let x = m.positions.len() as u32 - 1;
    m.triangles.push([6, 12, x]);
    m
}

#[test]
fn fill_holes_closes_a_hole_beside_an_existing_diagonal_without_a_non_manifold_edge() {
    for smooth in [false, true] {
        let mut s = Session::new();
        let id = s.doc.add_mesh("scan", hole_next_to_a_flap());
        let out =
            s.execute("mesh.fill_holes", json!({"object": "scan", "max_edges": 100, "smooth": smooth})).expect("fill");
        assert!(out["holes_filled"].as_u64().unwrap_or(0) >= 2, "{out}");
        assert_eq!(non_manifold(&s, id), 0, "smooth {smooth}");
    }
}

#[test]
fn random_meshes_do_get_holes_filled_by_mesh_fill_holes_alone() {
    let mut rng = Rng::new(9_000_031);
    let mut filled = 0;
    for round in 0..150 {
        let mesh = soup(&mut rng, round % 3 == 2);
        let before = readable_non_manifold(&mesh);
        let mut s = Session::new();
        let id = s.doc.add_mesh("scan", mesh);
        if let Ok(out) =
            s.execute("mesh.fill_holes", json!({"object": "scan", "max_edges": 300, "smooth": round % 2 == 0}))
        {
            filled += out["holes_filled"].as_u64().unwrap_or(0);
        }
        assert!(non_manifold(&s, id) <= before, "round {round}");
    }
    assert!(filled > 0, "no round filled a hole: the test would pass on any fill");
}

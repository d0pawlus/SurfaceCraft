//! P8: region operations (enlarge, shrink, separate, insert, resegment) on a synthetic box scan.
//! The box is 40 × 30 × 20 mm with every face cut into 8 × 8 squares, so each face is 128
//! triangles. Faces are selected by the centroids of their triangles.
use serde_json::{Value, json};
use std::collections::HashSet;
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::Session;
use surfacecraft_fit::PrimitiveKind;
use surfacecraft_geom::{Point3, Vec3, synth};

fn box_scan(s: &mut Session) {
    s.doc.add_mesh("scan", synth::cuboid(Point3::origin(), Vec3::new(40.0, 30.0, 20.0), 8));
}

/// Triangles of the scan whose centroid satisfies `pred` (in index order).
fn tris_where(s: &Session, pred: impl Fn(Point3) -> bool) -> Vec<u32> {
    let m = &s.doc.meshes[0].mesh;
    (0..m.triangle_count()).filter(|&t| m.face_centroid(t).is_some_and(&pred)).map(|t| t as u32).collect()
}

/// The id of the region in a command's output.
fn id_of(v: &Value) -> u64 {
    v["id"].as_u64().unwrap_or_else(|| panic!("no region id in {v}"))
}

/// The triangles of region `id` (sorted).
fn tris(s: &Session, id: u64) -> Vec<u32> {
    s.doc.region(ObjectId(id)).map(|r| r.triangles.to_vec()).unwrap_or_default()
}

/// Enlarge by 2 rings, then shrink by 2: the top face comes back exactly. Shrink reports the
/// triangles it released (all unassigned here), and one undo brings the enlarged region back.
#[test]
fn enlarge_then_shrink_gives_the_face_back() {
    let mut s = Session::new();
    box_scan(&mut s);
    let top = tris_where(&s, |c| c.z > 19.999);
    assert_eq!(top.len(), 128);
    let id = id_of(&s.execute("region.create", json!({"mesh": "scan", "triangles": top, "name": "top"})).unwrap());
    let grown = s.execute("region.enlarge", json!({"region": id, "rings": 2})).unwrap();
    let bigger = tris(&s, id);
    assert!(bigger.len() > top.len());
    assert_eq!(grown["regions"][0]["triangles"].as_u64(), Some(bigger.len() as u64));
    let shrunk = s.execute("region.shrink", json!({"region": id, "rings": 2})).unwrap();
    assert_eq!(tris(&s, id), top, "enlarge then shrink by 2 rings must give back the face");
    assert_eq!(shrunk["unassigned"].as_u64(), Some((bigger.len() - top.len()) as u64));
    // one undo step reverts the shrink only
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(tris(&s, id), bigger);
}

/// Two opposite faces share no edge: separating their region gives two regions, nothing lost.
#[test]
fn separate_splits_opposite_faces_in_two() {
    let mut s = Session::new();
    box_scan(&mut s);
    let mut both = tris_where(&s, |c| c.z > 19.999);
    both.extend(tris_where(&s, |c| c.z < 0.001));
    let id = id_of(&s.execute("region.create", json!({"mesh": "scan", "triangles": both, "name": "pair"})).unwrap());
    let out = s.execute("region.separate", json!({"region": id})).unwrap();
    let parts = out["regions"].as_array().unwrap();
    assert_eq!(parts.len(), 2, "{out}");
    let total: u64 = parts.iter().filter_map(|p| p["triangles"].as_u64()).sum();
    assert_eq!(total as usize, both.len());
    assert_eq!(out["unassigned"], 0);
    assert_eq!(parts.iter().filter(|p| p["created"] == json!(true)).count(), 1);
    let held: usize = s.doc.regions.iter().map(|r| r.triangles.len()).sum();
    assert_eq!(held, both.len());
}

/// Pieces below `min_triangles` are dropped: their triangles become unassigned.
#[test]
fn separate_drops_pieces_below_the_minimum() {
    let mut s = Session::new();
    box_scan(&mut s);
    let mut all = tris_where(&s, |c| c.z > 19.999);
    all.extend(tris_where(&s, |c| c.z < 0.001));
    // one triangle of the front face, in the middle: a piece of its own
    let lone = tris_where(&s, |c| c.y < 0.001 && (17.5..=22.5).contains(&c.x) && (7.5..=12.5).contains(&c.z));
    assert!(!lone.is_empty());
    all.push(lone[0]);
    let id = id_of(&s.execute("region.create", json!({"mesh": "scan", "triangles": all, "name": "pair"})).unwrap());
    let out = s.execute("region.separate", json!({"region": id, "min_triangles": 2})).unwrap();
    assert_eq!(out["regions"].as_array().unwrap().len(), 2, "{out}");
    assert_eq!(out["unassigned"], 1);
    assert!(s.doc.regions.iter().all(|r| r.triangles.len() >= 2));
}

/// A box selection becomes a new region and leaves the regions that held its triangles: no
/// triangle is in two regions afterwards.
#[test]
fn insert_takes_its_triangles_out_of_the_regions_that_held_them() {
    let mut s = Session::new();
    box_scan(&mut s);
    let top = tris_where(&s, |c| c.z > 19.999);
    let front = tris_where(&s, |c| c.y < 0.001);
    let a = id_of(&s.execute("region.create", json!({"mesh": "scan", "triangles": top, "name": "top"})).unwrap());
    let b = id_of(&s.execute("region.create", json!({"mesh": "scan", "triangles": front, "name": "front"})).unwrap());
    let inside =
        |c: Point3| (30.0..=41.0).contains(&c.x) && (-1.0..=31.0).contains(&c.y) && (12.0..=21.0).contains(&c.z);
    let picked = tris_where(&s, inside);
    assert!(!picked.is_empty());
    let out = s
        .execute(
            "region.insert",
            json!({"mesh": "scan", "box": {"min": [30.0, -1.0, 12.0], "max": [41.0, 31.0, 21.0]}, "name": "corner"}),
        )
        .unwrap();
    let n = id_of(&out["regions"][0]);
    assert_eq!(tris(&s, n), picked);
    let chosen: HashSet<u32> = picked.iter().copied().collect();
    assert_eq!(tris(&s, a), top.iter().copied().filter(|t| !chosen.contains(t)).collect::<Vec<u32>>());
    assert_eq!(tris(&s, b), front.iter().copied().filter(|t| !chosen.contains(t)).collect::<Vec<u32>>());
    let mut seen: HashSet<u32> = HashSet::new();
    for r in &s.doc.regions {
        for &t in r.triangles.iter() {
            assert!(seen.insert(t), "triangle {t} is in two regions");
        }
    }
    assert_eq!(out["unassigned"], 0);
}

/// Top and front merged, then resegmented: two planes come back, the merged region is replaced,
/// and the other four face regions keep their ids and triangles.
#[test]
fn resegment_of_two_merged_faces_gives_two_planes() {
    let mut s = Session::new();
    box_scan(&mut s);
    let faces: [(&str, Vec<u32>); 6] = [
        ("top", tris_where(&s, |c| c.z > 19.999)),
        ("bottom", tris_where(&s, |c| c.z < 0.001)),
        ("front", tris_where(&s, |c| c.y < 0.001)),
        ("back", tris_where(&s, |c| c.y > 29.999)),
        ("left", tris_where(&s, |c| c.x < 0.001)),
        ("right", tris_where(&s, |c| c.x > 39.999)),
    ];
    let mut ids: Vec<u64> = Vec::new();
    for (name, t) in &faces {
        ids.push(id_of(&s.execute("region.create", json!({"mesh": "scan", "triangles": t, "name": name})).unwrap()));
    }
    let merged = id_of(&s.execute("region.merge", json!({"regions": [ids[0], ids[2]]})).unwrap());
    let others = [ids[1], ids[3], ids[4], ids[5]];
    let before: Vec<Vec<u32>> = others.iter().map(|&id| tris(&s, id)).collect();
    let union = tris(&s, merged);
    assert_eq!(union.len(), 256);

    let out = s.execute("region.resegment", json!({"regions": [merged]})).unwrap();
    let regions = out["regions"].as_array().unwrap();
    let created: Vec<u64> = regions.iter().filter(|r| r["created"] == json!(true)).map(id_of).collect();
    assert_eq!(created.len(), 2, "{out}");
    for &id in &created {
        assert_eq!(s.doc.region(ObjectId(id)).and_then(|r| r.kind), Some(PrimitiveKind::Plane));
    }
    // every triangle of the merged region is either in a new plane or unassigned
    let made: u64 = created.iter().map(|&id| tris(&s, id).len() as u64).sum();
    assert_eq!(made + out["unassigned"].as_u64().unwrap(), union.len() as u64);
    assert!(s.doc.region(ObjectId(merged)).is_none());
    for (id, t) in others.iter().zip(&before) {
        assert_eq!(&tris(&s, *id), t);
    }
}

/// Bad parameters are errors, never panics; a huge ring count stops where the mesh ends.
#[test]
fn region_ops_refuse_bad_parameters() {
    let mut s = Session::new();
    box_scan(&mut s);
    let top = tris_where(&s, |c| c.z > 19.999);
    let id = id_of(&s.execute("region.create", json!({"mesh": "scan", "triangles": top, "name": "top"})).unwrap());
    assert!(s.execute("region.enlarge", json!({"region": 999_999})).is_err());
    assert!(s.execute("region.enlarge", json!({"region": id, "rings": 0})).is_err());
    assert!(s.execute("region.shrink", json!({"region": id, "rings": 0})).is_err());
    assert!(s.execute("region.separate", json!({"region": "no such region"})).is_err());
    assert!(s.execute("region.insert", json!({"mesh": "scan"})).is_err());
    assert!(
        s.execute(
            "region.insert",
            json!({"mesh": "scan", "triangles": [1], "sphere": {"center": [0, 0, 0], "radius": 1}})
        )
        .is_err()
    );
    assert!(
        s.execute("region.insert", json!({"mesh": "scan", "sphere": {"center": [0, 0, 0], "radius": -1}})).is_err()
    );
    assert!(s.execute("region.insert", json!({"mesh": "scan", "triangles": []})).is_err());
    assert!(s.execute("region.resegment", json!({"regions": []})).is_err());
    // a region with an edge erodes away completely, however many rings are asked for
    assert!(s.execute("region.shrink", json!({"region": id, "rings": u32::MAX})).is_ok());
    assert!(s.doc.region(ObjectId(id)).is_none(), "shrinking by every ring leaves nothing");
    // one triangle grown by every ring covers the closed box (6 × 128 triangles)
    let seed =
        id_of(&s.execute("region.create", json!({"mesh": "scan", "triangles": [top[0]], "name": "seed"})).unwrap());
    assert!(s.execute("region.enlarge", json!({"region": seed, "rings": u32::MAX, "steal": true})).is_ok());
    assert_eq!(tris(&s, seed).len(), 768);
}

//! P8-G1 gate tests of the chunk store: the surface is reproduced, level errors stay within their
//! stated bounds, the build is bounded and deterministic, and damaged or hostile stores are errors.
#![cfg(not(target_arch = "wasm32"))]
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use surfacecraft_geom::chunked::build::{Sink, build_store};
use surfacecraft_geom::chunked::format::{FileKind, StoreIndex, checksum};
use surfacecraft_geom::chunked::levels::finalize_soup;
use surfacecraft_geom::chunked::spill::SPILL_BUFFER_TRIANGLES;
use surfacecraft_geom::chunked::{
    BlockStore, ChunkLevels, Chunked, MemBlocks, StoreError, StoreParams, chunk_in_memory, disk,
};
use surfacecraft_geom::mesh::{source_area, source_bbox};
use surfacecraft_geom::{Bvh, Mesh, Point3, PointSource, Rng, TriangleSource, Vec3, synth};

struct TempDir(PathBuf);

impl TempDir {
    fn new(name: &str) -> Self {
        let p = std::env::temp_dir().join(format!("sc-chunked-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&p);
        TempDir(p)
    }
    fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn torus(nu: usize, nv: usize) -> Mesh {
    synth::torus(Point3::new(3.0, -2.0, 1.0), Vec3::new(0.3, 0.2, 1.0).normalize(), 40.0, 9.0, nu, nv)
}

fn soup_of(m: &Mesh) -> Vec<[Point3; 3]> {
    (0..m.triangle_count()).filter_map(|i| m.triangle(i)).collect()
}

fn soup_mesh(soup: &[[Point3; 3]]) -> Mesh {
    Mesh {
        positions: soup.iter().flatten().copied().collect(),
        triangles: (0..soup.len() as u32).map(|i| [3 * i, 3 * i + 1, 3 * i + 2]).collect(),
    }
}

fn replay_of(soup: &[[Point3; 3]]) -> impl FnMut(Sink<'_>) -> Result<(), StoreError> + '_ {
    move |sink| soup.chunks(4096).try_for_each(sink)
}

fn build(dir: &Path, soup: &[[Point3; 3]], params: &StoreParams) -> surfacecraft_geom::chunked::build::BuildReport {
    build_store(dir, params, false, &mut replay_of(soup)).unwrap()
}

fn splitmix(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9e37_79b9_7f4a_7c15);
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    x ^ (x >> 31)
}

/// Order-independent fingerprint of the triangles of a mesh by their corner coordinates (each
/// triangle rotated so its smallest corner comes first, orientation kept).
fn fingerprint(m: &Mesh) -> (usize, u64) {
    let mut sum = 0u64;
    for i in 0..m.triangle_count() {
        let t = m.triangle(i).unwrap();
        let key = |p: &Point3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        let k = [key(&t[0]), key(&t[1]), key(&t[2])];
        let r = (0..3).min_by_key(|&j| k[j]).unwrap();
        let mut h = 0u64;
        for j in 0..3 {
            for w in k[(r + j) % 3] {
                h = splitmix(h ^ w);
            }
        }
        sum = sum.wrapping_add(h);
    }
    (m.triangle_count(), sum)
}

#[test]
fn gate_g1_store_reproduces_the_surface() {
    let soup = soup_of(&torus(300, 200));
    let dir = TempDir::new("surface");
    let params = StoreParams { chunk_triangles: 4096, levels: 4, ..StoreParams::default() };
    let report = build(dir.path(), &soup, &params);
    let reference = finalize_soup(soup_mesh(&soup));
    let store = disk::open(dir.path()).unwrap();
    let rep = store.verify().unwrap();
    assert_eq!(rep.chunks, store.chunk_count());
    assert!(store.chunk_count() >= 120_000 / 4096, "{} chunks", store.chunk_count());
    let s = &report.summary;
    assert_eq!(s.source_triangles, soup.len() as u64);
    assert_eq!(s.levels[0].triangles as usize, reference.triangle_count());
    let (rb, sb) = (reference.bbox(), s.bbox);
    for i in 0..3 {
        assert!((rb.min[i] - sb.min[i]).abs() < 1e-12 && (rb.max[i] - sb.max[i]).abs() < 1e-12);
    }
    assert!((s.area - reference.area()).abs() < 1e-9 * reference.area());
    // through the traits, level 0
    assert_eq!(store.triangle_count_of(), reference.triangle_count());
    assert_eq!(source_bbox(&store), sb);
    assert!((source_area(&store) - reference.area()).abs() < 1e-9 * reference.area());
    // the union of the chunks is the surface: same multiset of triangles
    let all = store.to_mesh(0, usize::MAX).unwrap();
    assert_eq!(fingerprint(&all), fingerprint(&reference));
    // every chunk is within capacity, and its box is tight around its vertices
    for (c, e) in store.index().chunks.iter().enumerate() {
        assert!(e.levels[0].triangles <= 4096);
        assert_eq!(store.read_mesh(c, 0).unwrap().bbox(), e.bbox);
    }
    assert!(store.take_error().is_none());
    // vertices welded inside chunks only: seam duplicates exist, but far fewer than a soup's 3 per triangle
    assert!(s.levels[0].vertices as usize > reference.vertex_count());
    assert!((s.levels[0].vertices as usize) < reference.vertex_count() * 3 / 2);
}

#[test]
fn gate_g1_levels_stay_within_their_stated_error() {
    let soup = soup_of(&torus(300, 200));
    let reference = finalize_soup(soup_mesh(&soup));
    let bvh = Bvh::build(&reference);
    let dir = TempDir::new("levels");
    let params = StoreParams { chunk_triangles: 4096, levels: 4, ..StoreParams::default() };
    let report = build(dir.path(), &soup, &params);
    let store = disk::open(dir.path()).unwrap();
    let mut previous = report.summary.levels[0].triangles;
    for l in 1..=4 {
        let lv = report.summary.levels[l];
        assert!(lv.triangles < previous && lv.triangles > 0, "level {l}: {} after {previous}", lv.triangles);
        assert!(lv.max_error > 0.0 && lv.max_error <= lv.error_bound + 1e-9, "level {l}: {lv:?}");
        assert!((lv.error_bound - 3f64.sqrt() * lv.cell_size).abs() < 1e-12);
        previous = lv.triangles;
        for c in 0..store.chunk_count() {
            // (a) every level-0 vertex is within the bound of its coarse vertex (through the stored map)
            let (m0, mk) = (store.read_mesh(c, 0).unwrap(), store.read_mesh(c, l).unwrap());
            let maps = store.read_maps(c, l).unwrap();
            let worst = m0
                .positions
                .iter()
                .zip(&maps.vertex_map)
                .map(|(p, &v)| (p - mk.positions[v as usize]).norm())
                .fold(0.0, f64::max);
            assert!((worst - store.index().chunks[c].levels[l].error).abs() < 1e-12);
            assert!(worst <= lv.error_bound + 1e-9, "level {l} chunk {c}: {worst}");
            // a surviving triangle maps to a coarse triangle of this level
            for &ct in maps.triangle_map.iter().filter(|&&t| t != u32::MAX) {
                assert!((ct as usize) < mk.triangle_count());
            }
            // (b) coarse corners and centroids lie within the bound of the full surface
            for (i, t) in mk.triangles.iter().enumerate().filter(|(i, _)| i % 5 == 0) {
                let [a, b, d] = t.map(|k| mk.positions[k as usize]);
                let mid = Point3::from((a.coords + b.coords + d.coords) / 3.0);
                for q in [a, b, d, mid] {
                    let dist = bvh.closest(&q, f64::INFINITY).unwrap().distance;
                    assert!(dist <= lv.error_bound + 1e-9, "level {l} chunk {c} tri {i}: {dist} > {}", lv.error_bound);
                }
            }
        }
        // the level as a whole is a source too, with global numbering
        let view = store.level(l).unwrap();
        assert_eq!(view.triangle_count_of() as u64, lv.triangles);
        assert_eq!(view.point_count() as u64, lv.vertices);
        let last = view.triangle_count_of() - 1;
        let t = view.triangle_indices_at(last).unwrap();
        assert!(t.iter().all(|&k| (k as usize) < view.point_count()));
        assert!(view.triangle_points_at(last).is_some() && view.triangle_points_at(last + 1).is_none());
    }
}

#[test]
fn gate_g1_build_is_bounded_and_deterministic() {
    // more triangles than the spill buffer holds, so buffers are flushed and merged from files
    let soup = soup_of(&torus(520, 300));
    assert!(soup.len() > SPILL_BUFFER_TRIANGLES);
    let (d1, d2) = (TempDir::new("det1"), TempDir::new("det2"));
    let params = StoreParams { chunk_triangles: 2048, levels: 3, ..StoreParams::default() };
    let r1 = build(d1.path(), &soup, &params);
    let r2 = build(d2.path(), &soup, &params);
    assert_eq!(r1.summary, r2.summary);
    let st = r1.stats;
    assert!(st.peak_buffered_triangles <= SPILL_BUFFER_TRIANGLES, "{st:?}");
    assert!(st.spill_flushes >= 1 && st.max_chunk_triangles <= 2048 && st.leaves >= soup.len() / 2048, "{st:?}");
    assert!(
        surfacecraft_geom::chunked::build::memory_bound_bytes(&params) < 128 << 20,
        "the documented memory bound is a constant: it depends on the chunk size, not on the input"
    );
    let mut names: Vec<String> =
        std::fs::read_dir(d1.path()).unwrap().map(|e| e.unwrap().file_name().to_string_lossy().into_owned()).collect();
    names.sort();
    assert_eq!(
        names,
        ["index.scx", "map-1.dat", "map-2.dat", "map-3.dat", "mesh-0.dat", "mesh-1.dat", "mesh-2.dat", "mesh-3.dat"]
    );
    for n in &names {
        assert_eq!(std::fs::read(d1.path().join(n)).unwrap(), std::fs::read(d2.path().join(n)).unwrap(), "{n}");
    }
}

#[test]
fn a_store_without_maps_and_the_in_memory_chunking_agree_with_the_disk_build() {
    let soup = soup_of(&torus(120, 80));
    let dir = TempDir::new("nomaps");
    let params = StoreParams { chunk_triangles: 512, levels: 2, keep_maps: false, ..StoreParams::default() };
    let rep = build(dir.path(), &soup, &params);
    assert!(!dir.path().join("map-1.dat").exists());
    let disk_store = disk::open(dir.path()).unwrap();
    assert!(matches!(disk_store.read_maps(0, 1), Err(StoreError::Params(_))));
    let mem = chunk_in_memory(&soup_mesh(&soup), &params).unwrap();
    assert_eq!(mem.summary(), rep.summary);
    assert_eq!(
        fingerprint(&mem.to_mesh(0, usize::MAX).unwrap()),
        fingerprint(&disk_store.to_mesh(0, usize::MAX).unwrap())
    );
}

#[test]
fn a_hostile_pile_of_identical_triangles_is_cut_by_input_order() {
    let t = [Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];
    let mut soup = vec![t; 3000];
    soup.push([Point3::new(50.0, 50.0, 50.0), Point3::new(51.0, 50.0, 50.0), Point3::new(50.0, 51.0, 50.0)]);
    let dir = TempDir::new("heavy");
    let params = StoreParams { chunk_triangles: 256, levels: 1, ..StoreParams::default() };
    let rep = build(dir.path(), &soup, &params);
    assert_eq!(rep.stats.heavy_cells, 1);
    assert!(rep.stats.max_chunk_triangles <= 256);
    assert_eq!(rep.summary.levels[0].triangles, 3001);
    disk::open(dir.path()).unwrap().verify().unwrap();
}

#[test]
fn directories_are_never_clobbered_and_failures_leave_nothing_behind() {
    let soup = soup_of(&torus(40, 30));
    let params = StoreParams { chunk_triangles: 256, levels: 1, ..StoreParams::default() };
    let dir = TempDir::new("dirs");
    std::fs::create_dir_all(dir.path()).unwrap();
    std::fs::write(dir.path().join("precious.txt"), "keep").unwrap();
    for overwrite in [false, true] {
        let r = build_store(dir.path(), &params, overwrite, &mut replay_of(&soup));
        assert!(matches!(r, Err(StoreError::Params(_))), "overwrite {overwrite}: {r:?}");
        assert_eq!(std::fs::read_to_string(dir.path().join("precious.txt")).unwrap(), "keep");
    }
    std::fs::remove_file(dir.path().join("precious.txt")).unwrap();
    build(dir.path(), &soup, &params);
    // a second build into a finished store needs `overwrite`, and then replaces it
    assert!(build_store(dir.path(), &params, false, &mut replay_of(&soup)).is_err());
    build_store(dir.path(), &params, true, &mut replay_of(&soup)).unwrap();
    // bad input: nothing of the store is left, and the error says what is wrong
    let mut bad = soup.clone();
    bad[7][1].y = f64::NAN;
    let fail = TempDir::new("fail");
    let r = build_store(fail.path(), &params, false, &mut replay_of(&bad));
    assert!(matches!(r, Err(StoreError::Input(_))), "{r:?}");
    assert_eq!(std::fs::read_dir(fail.path()).unwrap().count(), 0);
    // an input that differs between passes is detected
    let mut calls = 0;
    let mut shifting = |sink: Sink<'_>| -> Result<(), StoreError> {
        calls += 1;
        let n = if calls == 3 { soup.len() - 5 } else { soup.len() };
        soup[..n].chunks(1000).try_for_each(sink)
    };
    let r = build_store(fail.path(), &params, false, &mut shifting);
    assert!(matches!(r, Err(StoreError::Input(_))), "{r:?}");
    assert_eq!(std::fs::read_dir(fail.path()).unwrap().count(), 0);
    // parameters and empty input
    let tiny = StoreParams { chunk_triangles: 3, ..params };
    assert!(matches!(build_store(fail.path(), &tiny, false, &mut replay_of(&soup)), Err(StoreError::Params(_))));
    assert!(matches!(build_store(fail.path(), &params, false, &mut replay_of(&[])), Err(StoreError::Input(_))));
    let flat = [[Point3::origin(), Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)]];
    assert!(matches!(build_store(fail.path(), &params, false, &mut replay_of(&flat)), Err(StoreError::Input(_))));
}

#[test]
fn damage_on_disk_is_an_error_never_a_wrong_surface() {
    let soup = soup_of(&torus(100, 60));
    let params = StoreParams { chunk_triangles: 512, levels: 2, ..StoreParams::default() };
    let base = TempDir::new("dmg-base");
    build(base.path(), &soup, &params);
    let copy = |name: &str| {
        let d = TempDir::new(name);
        std::fs::create_dir_all(d.path()).unwrap();
        for e in std::fs::read_dir(base.path()).unwrap() {
            let e = e.unwrap();
            std::fs::copy(e.path(), d.path().join(e.file_name())).unwrap();
        }
        d
    };
    // a flipped bit in a data file: open is fine, verify and the affected read fail
    let d = copy("dmg-flip");
    let f = d.path().join("mesh-0.dat");
    let mut bytes = std::fs::read(&f).unwrap();
    let at = bytes.len() / 2;
    bytes[at] ^= 0x10;
    std::fs::write(&f, &bytes).unwrap();
    let store = disk::open(d.path()).unwrap();
    assert!(matches!(store.verify(), Err(StoreError::Corrupt(_))));
    let hit = (0..store.chunk_count()).filter(|&c| store.read_mesh(c, 0).is_err()).count();
    assert_eq!(hit, 1);
    // through the trait the failed chunk is None and the error is remembered
    let n = store.triangle_count_of();
    assert!((0..n).any(|i| store.triangle_points_at(i).is_none()));
    assert!(matches!(store.take_error(), Some(StoreError::Corrupt(_))));
    // truncated and padded data files, missing files, a bad index
    type Edit = Box<dyn Fn(&mut Vec<u8>)>;
    let edits: Vec<(&str, Edit)> = vec![
        ("mesh-1.dat", Box::new(|b| b.truncate(b.len() - 1))),
        ("map-2.dat", Box::new(|b| b.push(0))),
        ("index.scx", Box::new(|b| b[40] ^= 1)),
        ("index.scx", Box::new(|b| b.truncate(10))),
        ("index.scx", Box::new(Vec::clear)),
    ];
    for (name, edit) in edits {
        let d = copy("dmg-edit");
        let p = d.path().join(name);
        let mut b = std::fs::read(&p).unwrap();
        edit(&mut b);
        std::fs::write(&p, b).unwrap();
        assert!(disk::open(d.path()).is_err(), "{name} damaged");
    }
    for name in ["index.scx", "mesh-0.dat", "map-1.dat"] {
        let d = copy("dmg-missing");
        std::fs::remove_file(d.path().join(name)).unwrap();
        assert!(disk::open(d.path()).is_err(), "{name} missing");
    }
    assert!(disk::open(&base.path().join("nope")).is_err());
}

fn memory_store() -> Chunked<MemBlocks> {
    let params = StoreParams { chunk_triangles: 256, levels: 3, ..StoreParams::default() };
    chunk_in_memory(&torus(60, 40), &params).unwrap()
}

fn kinds(index: &StoreIndex) -> Vec<FileKind> {
    let mut k = vec![FileKind::Mesh(0)];
    for l in 1..=index.levels.len() {
        k.extend([FileKind::Mesh(l), FileKind::Map(l)]);
    }
    k
}

/// Touches everything a reader can reach; nothing may panic, and what is returned is valid.
fn exercise(store: &Chunked<MemBlocks>) {
    let _ = store.verify();
    for c in 0..store.chunk_count() {
        for l in 0..store.level_count() {
            if let Ok(m) = store.read_mesh(c, l) {
                assert!(m.validate().is_ok());
            }
            let _ = store.read_maps(c, l);
        }
    }
    for l in 0..store.level_count() {
        let v = store.level(l).unwrap();
        for i in (0..v.triangle_count_of().min(200)).chain([usize::MAX]) {
            let _ = v.triangle_points_at(i);
            let _ = v.point_at(i);
        }
    }
    let _ = store.take_error();
}

#[test]
fn hostile_store_bytes_never_panic() {
    let (index, blocks) = memory_store().into_parts();
    let mut rng = Rng::new(7);
    for round in 0..600 {
        let (mut idx, mut blk) = (index.clone(), blocks.clone());
        let fix_checksums = round % 2 == 0;
        // damage the data files
        let k = kinds(&idx);
        let kind = k[rng.index(k.len())];
        let file = blk.file_mut(kind).unwrap();
        for _ in 0..1 + rng.index(4) {
            let at = rng.index(file.len());
            file[at] ^= 1 + (rng.next_u64() % 255) as u8;
        }
        if fix_checksums {
            // make the index agree with the damaged bytes, so only the structural checks stand in the way
            for c in &mut idx.chunks {
                for (l, b) in c.levels.iter_mut().enumerate() {
                    for (kind, span) in [(FileKind::Mesh(l), &mut b.mesh), (FileKind::Map(l), &mut b.map)] {
                        if let Ok(bytes) = blk.read(kind, span.offset, span.len as usize) {
                            span.checksum = checksum(&bytes);
                        }
                    }
                }
            }
        }
        let store = Chunked::new(idx, blk).unwrap();
        if !fix_checksums {
            assert!(store.verify().is_err(), "round {round}: damage went unnoticed");
        }
        exercise(&store);
    }
}

#[test]
fn hostile_index_bytes_never_panic() {
    let (index, blocks) = memory_store().into_parts();
    let bytes = index.encode();
    assert_eq!(StoreIndex::decode(&bytes).unwrap(), index);
    let mut rng = Rng::new(11);
    for round in 0..3000 {
        let mut b = bytes.clone();
        match round % 5 {
            0 => b.truncate(rng.index(b.len() + 1)),
            1 => b.extend((0..rng.index(64)).map(|_| rng.next_u64() as u8)),
            _ => {
                for _ in 0..1 + rng.index(6) {
                    let at = rng.index(b.len());
                    b[at] ^= 1 + (rng.next_u64() % 255) as u8;
                }
            }
        }
        let fix = round % 2 == 0 && b.len() > 8;
        if fix {
            let n = b.len() - 8;
            let sum = checksum(&b[..n]);
            b[n..].copy_from_slice(&sum.to_le_bytes());
        }
        let decoded = StoreIndex::decode(&b);
        if !fix && b != bytes {
            assert!(decoded.is_err(), "round {round}: a damaged index was accepted");
        }
        if let Ok(idx) = decoded
            && let Ok(store) = Chunked::new(idx, blocks.clone())
        {
            exercise(&store);
        }
    }
    // random garbage
    for _ in 0..500 {
        let n = rng.index(300);
        let b: Vec<u8> = (0..n).map(|_| rng.next_u64() as u8).collect();
        let _ = StoreIndex::decode(&b);
    }
}

#[test]
fn levels_built_directly_are_deterministic_and_shrink() {
    let m = finalize_soup(soup_mesh(&soup_of(&torus(50, 30))));
    let lp = surfacecraft_geom::chunked::LevelParams {
        grid_origin: m.bbox().min,
        base_cell: 2.0,
        levels: 3,
        keep_maps: true,
    };
    let a: ChunkLevels = surfacecraft_geom::chunked::levels::build_levels(m.clone(), &lp);
    let b = surfacecraft_geom::chunked::levels::build_levels(m, &lp);
    assert_eq!(a, b);
    assert!(a.levels.iter().all(|l| l.mesh.validate().is_ok()));
    let counts: HashMap<usize, usize> =
        a.levels.iter().enumerate().map(|(i, l)| (i, l.mesh.triangle_count())).collect();
    assert!(counts[&0] > counts[&1] && counts[&1] > counts[&2] && counts[&2] > counts[&3], "{counts:?}");
}

//! P8-G1 gate tests of the out-of-core import: a generated mesh is streamed from a file into a chunk
//! store and compared with the in-memory import; the stream readers are fuzzed as hostile input.
#![cfg(not(target_arch = "wasm32"))]
use std::fs::File;
use std::io::{BufWriter, Cursor, Write};
use std::path::{Path, PathBuf};
use surfacecraft_geom::chunked::{StoreParams, disk};
use surfacecraft_geom::{Bvh, Mesh, Point3, Rng};
use surfacecraft_io::stream::import::{StreamImportOptions, import_to_store};
use surfacecraft_io::stream::{StreamFormat, StreamOptions, detect, for_each_batch};
use surfacecraft_io::{IoError, Limits, MeshFormat, read_mesh};

struct TempDir(PathBuf);

impl TempDir {
    fn new(name: &str) -> Self {
        let p = std::env::temp_dir().join(format!("sc-stream-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).unwrap();
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

const BIG_R: f64 = 40.0;
const SMALL_R: f64 = 9.0;

/// Vertex `(i, j)` of a torus grid of `nu × nv` (indices wrap, so seams share exact coordinates).
fn torus_vertex(i: usize, j: usize, nu: usize, nv: usize) -> [f32; 3] {
    let u = std::f64::consts::TAU * (i % nu) as f64 / nu as f64;
    let v = std::f64::consts::TAU * (j % nv) as f64 / nv as f64;
    let ring = BIG_R + SMALL_R * v.cos();
    [(ring * u.cos()) as f32, (ring * u.sin()) as f32, (SMALL_R * v.sin()) as f32]
}

/// Writes a binary STL of the torus without ever building the mesh; returns the triangle count.
fn write_torus_stl(path: &Path, nu: usize, nv: usize) -> usize {
    let n = 2 * nu * nv;
    let mut w = BufWriter::new(File::create(path).unwrap());
    let mut header = [0u8; 80];
    header[..4].copy_from_slice(b"test");
    w.write_all(&header).unwrap();
    w.write_all(&(n as u32).to_le_bytes()).unwrap();
    for i in 0..nu {
        for j in 0..nv {
            let (a, b, c, d) = (
                torus_vertex(i, j, nu, nv),
                torus_vertex(i + 1, j, nu, nv),
                torus_vertex(i + 1, j + 1, nu, nv),
                torus_vertex(i, j + 1, nu, nv),
            );
            for t in [[a, b, c], [a, c, d]] {
                w.write_all(&[0u8; 12]).unwrap();
                for p in t {
                    for x in p {
                        w.write_all(&x.to_le_bytes()).unwrap();
                    }
                }
                w.write_all(&[0, 0]).unwrap();
            }
        }
    }
    w.flush().unwrap();
    n
}

/// Writes an indexed binary PLY of the torus (little endian, float vertices); returns the triangle count.
fn write_torus_ply(path: &Path, nu: usize, nv: usize) -> usize {
    let mut w = BufWriter::new(File::create(path).unwrap());
    write!(
        w,
        "ply\nformat binary_little_endian 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nelement face {}\nproperty list uchar int vertex_indices\nend_header\n",
        nu * nv,
        2 * nu * nv
    )
    .unwrap();
    for i in 0..nu {
        for j in 0..nv {
            for x in torus_vertex(i, j, nu, nv) {
                w.write_all(&x.to_le_bytes()).unwrap();
            }
        }
    }
    let id = |i: usize, j: usize| ((i % nu) * nv + (j % nv)) as i32;
    for i in 0..nu {
        for j in 0..nv {
            let (a, b, c, d) = (id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1));
            for t in [[a, b, c], [a, c, d]] {
                w.write_all(&[3]).unwrap();
                for k in t {
                    w.write_all(&k.to_le_bytes()).unwrap();
                }
            }
        }
    }
    w.flush().unwrap();
    2 * nu * nv
}

fn splitmix(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9e37_79b9_7f4a_7c15);
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    x ^ (x >> 31)
}

/// Order-independent fingerprint of the triangles of a mesh by their corner coordinates.
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

fn gate_size() -> (usize, usize) {
    // about 3 M triangles by default; SURFACECRAFT_G1_GATE_TRIANGLES raises it (a few million locally)
    let target: usize =
        std::env::var("SURFACECRAFT_G1_GATE_TRIANGLES").ok().and_then(|v| v.parse().ok()).unwrap_or(3_000_000);
    let nv = ((target as f64 / 4.0).sqrt()) as usize;
    (2 * nv, nv)
}

/// Streams the file into a store and checks it against the in-memory import of the same bytes.
fn gate(path: &Path, format: MeshFormat, tag: &str, expected_triangles: usize) {
    let dir = TempDir::new(tag);
    let opts = StreamImportOptions {
        format: Some(format),
        store: StoreParams { chunk_triangles: 65_536, levels: 4, ..StoreParams::default() },
        ..StreamImportOptions::default()
    };
    let t = std::time::Instant::now();
    let report = import_to_store(path, dir.path(), &opts).unwrap();
    let stream_s = t.elapsed().as_secs_f64();
    let bytes = std::fs::read(path).unwrap();
    let reference = read_mesh(&bytes, format, &Limits::default()).unwrap();
    drop(bytes);
    let s = &report.summary;
    assert_eq!(s.source_triangles as usize, expected_triangles);
    assert_eq!(s.levels[0].triangles as usize, reference.triangle_count());
    let (rb, sb) = (reference.bbox(), s.bbox);
    for i in 0..3 {
        assert!((rb.min[i] - sb.min[i]).abs() < 1e-12 && (rb.max[i] - sb.max[i]).abs() < 1e-12);
    }
    assert!((s.area - reference.area()).abs() < 1e-9 * reference.area());
    // the memory of the import is a function of the chunk size, not of the file
    let st = &report.stats;
    assert!(st.peak_buffered_triangles <= surfacecraft_geom::chunked::spill::SPILL_BUFFER_TRIANGLES);
    assert!(st.max_chunk_triangles <= 65_536 && st.leaves >= expected_triangles / 65_536, "{st:?}");
    let store = disk::open(dir.path()).unwrap();
    store.verify().unwrap();
    // reading back all chunks reproduces the surface
    assert_eq!(fingerprint(&store.to_mesh(0, usize::MAX).unwrap()), fingerprint(&reference));
    // every level stays within its stated error of the full mesh
    let bvh = Bvh::build(&reference);
    for l in 1..=4 {
        let lv = s.levels[l];
        assert!(lv.max_error > 0.0 && lv.max_error <= lv.error_bound + 1e-9, "level {l}: {lv:?}");
        assert!(lv.triangles < s.levels[l - 1].triangles);
        let mut worst = 0.0f64;
        for c in 0..store.chunk_count() {
            let mk = store.read_mesh(c, l).unwrap();
            for t in mk.triangles.iter().step_by(37) {
                let [a, b, d] = t.map(|k| mk.positions[k as usize]);
                let mid = Point3::from((a.coords + b.coords + d.coords) / 3.0);
                for q in [a, b, d, mid] {
                    worst = worst.max(bvh.closest(&q, f64::INFINITY).unwrap().distance);
                }
            }
        }
        assert!(
            worst <= lv.error_bound + 1e-9,
            "level {l}: coarse surface {worst} from the full one, bound {}",
            lv.error_bound
        );
        eprintln!(
            "g1 {tag} level {l}: {} tris, cell {:.3} mm, bound {:.3} mm, max vertex error {:.3} mm, sampled coarse-to-full {:.3} mm",
            lv.triangles, lv.cell_size, lv.error_bound, lv.max_error, worst
        );
    }
    eprintln!(
        "g1 {tag}: {expected_triangles} tris -> {} chunks, {} bytes on disk, import {stream_s:.1} s, peak buffered {} tris, memory bound {} MiB",
        s.chunks,
        st.bytes_on_disk,
        st.peak_buffered_triangles,
        surfacecraft_geom::chunked::build::memory_bound_bytes(&opts.store) >> 20
    );
}

#[test]
fn gate_g1_streamed_stl_matches_the_in_memory_import() {
    let dir = TempDir::new("gate-stl-src");
    let (nu, nv) = gate_size();
    let path = dir.path().join("torus.stl");
    let n = write_torus_stl(&path, nu, nv);
    gate(&path, MeshFormat::Stl, "gate-stl", n);
}

#[test]
fn gate_g1_streamed_ply_matches_the_in_memory_import() {
    let dir = TempDir::new("gate-ply-src");
    let (nu, nv) = gate_size();
    let path = dir.path().join("torus.ply");
    let n = write_torus_ply(&path, nu, nv);
    gate(&path, MeshFormat::Ply, "gate-ply", n);
}

/// Larger runs for the integration pass: `SURFACECRAFT_G1_PERF_TRIANGLES=4000000,10000000`, and the
/// resident memory of the process is printed after each (macOS and Linux `ps`).
#[test]
#[ignore = "minutes and gigabytes of disk for 10 M triangles; run explicitly in release"]
fn perf_store_scale() {
    let sizes: Vec<usize> = std::env::var("SURFACECRAFT_G1_PERF_TRIANGLES")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .unwrap_or_else(|| vec![4_000_000]);
    for target in sizes {
        let nv = ((target as f64 / 4.0).sqrt()) as usize;
        let src = TempDir::new("perf-src");
        let path = src.path().join("torus.stl");
        let n = write_torus_stl(&path, 2 * nv, nv);
        let dir = TempDir::new("perf");
        let t = std::time::Instant::now();
        let opts = StreamImportOptions {
            store: StoreParams { levels: 5, ..StoreParams::default() },
            ..StreamImportOptions::default()
        };
        let report = import_to_store(&path, dir.path(), &opts).unwrap();
        let secs = t.elapsed().as_secs_f64();
        let rss = std::process::Command::new("ps")
            .args(["-o", "rss=", "-p", &std::process::id().to_string()])
            .output()
            .ok()
            .and_then(|o| String::from_utf8(o.stdout).ok())
            .and_then(|s| s.trim().parse::<u64>().ok())
            .unwrap_or(0);
        eprintln!(
            "perf {n} tris: import {secs:.1} s, {} chunks, {} MB on disk, resident set of the test process after import {} MB, memory bound {} MiB, {:?}",
            report.summary.chunks,
            report.stats.bytes_on_disk >> 20,
            rss / 1024,
            surfacecraft_geom::chunked::build::memory_bound_bytes(&opts.store) >> 20,
            report.stats
        );
        let bound_mb = (surfacecraft_geom::chunked::build::memory_bound_bytes(&opts.store) >> 20) as u64;
        assert!(rss == 0 || rss / 1024 <= bound_mb, "resident {} MB over the bound of {bound_mb} MB", rss / 1024);
        let st = disk::open(dir.path()).unwrap();
        let v = st.verify().unwrap();
        eprintln!("perf {n} tris: verify read {} blocks, {} MB", v.blocks, v.bytes >> 20);
    }
}

fn collect(bytes: &[u8], format: StreamFormat, scale: f64) -> Result<Vec<[Point3; 3]>, IoError> {
    let opts = StreamOptions { limits: Limits::default(), scale };
    let mut out = Vec::new();
    for_each_batch(&|| Ok(Cursor::new(bytes)), format, &opts, &mut |b| {
        out.extend_from_slice(b);
        Ok(())
    })?;
    Ok(out)
}

fn triangles_of(m: &Mesh) -> Vec<[Point3; 3]> {
    (0..m.triangle_count()).filter_map(|i| m.triangle(i)).collect()
}

/// A binary PLY with extra properties, quads, big endian and double coordinates.
fn tricky_ply(big_endian: bool) -> Vec<u8> {
    let enc = if big_endian { "binary_big_endian" } else { "binary_little_endian" };
    let mut b = format!(
        "ply\nformat {enc} 1.0\ncomment tricky\nelement vertex 6\nproperty double x\nproperty double y\nproperty double z\nproperty uchar red\nproperty float quality\nelement face 3\nproperty uchar flags\nproperty list ushort uint vertex_indices\nelement edge 0\nproperty int a\nend_header\n"
    )
    .into_bytes();
    let pts = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 1.0]];
    let put = |b: &mut Vec<u8>, bytes: [u8; 8], n: usize| {
        let mut s = bytes[..n].to_vec();
        if big_endian {
            s.reverse();
        }
        b.extend(s);
    };
    for p in pts {
        for c in p {
            put(&mut b, f64::to_le_bytes(c), 8);
        }
        b.push(200);
        put(
            &mut b,
            [
                f32::to_le_bytes(0.5)[0],
                f32::to_le_bytes(0.5)[1],
                f32::to_le_bytes(0.5)[2],
                f32::to_le_bytes(0.5)[3],
                0,
                0,
                0,
                0,
            ],
            4,
        );
    }
    // a quad, a triangle, a pentagon-less triangle fan of four
    for face in [vec![0u32, 1, 2, 3], vec![0, 1, 5], vec![0, 3, 4, 5]] {
        b.push(7);
        put(&mut b, [face.len() as u8, 0, 0, 0, 0, 0, 0, 0], 2);
        for i in face {
            put(
                &mut b,
                [i.to_le_bytes()[0], i.to_le_bytes()[1], i.to_le_bytes()[2], i.to_le_bytes()[3], 0, 0, 0, 0],
                4,
            );
        }
    }
    b
}

#[test]
fn the_stream_readers_give_the_triangles_of_the_in_memory_readers() {
    let m = surfacecraft_geom::synth::torus(Point3::origin(), surfacecraft_geom::Vec3::z(), 30.0, 7.0, 40, 24);
    let stl = surfacecraft_io::write_mesh(&m, MeshFormat::Stl);
    let ply = surfacecraft_io::write_mesh(&m, MeshFormat::Ply);
    for (bytes, fmt, stream) in
        [(&stl, MeshFormat::Stl, StreamFormat::StlBinary), (&ply, MeshFormat::Ply, StreamFormat::PlyBinary)]
    {
        assert_eq!(detect(&mut Cursor::new(bytes.as_slice()), None).unwrap(), stream);
        let reference = triangles_of(&read_mesh(bytes, fmt, &Limits::default()).unwrap());
        assert_eq!(collect(bytes, stream, 1.0).unwrap(), reference, "{fmt:?}");
        // scale
        let scaled = collect(bytes, stream, 25.4).unwrap();
        assert!((scaled[0][1].coords - reference[0][1].coords * 25.4).norm() < 1e-9);
    }
    // ASCII STL
    let ascii = "solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nfacet normal 0 0 1\nouter loop\nvertex 1 1 0\nvertex 0 1 0\nvertex 1 0 0\nendloop\nendfacet\nendsolid t\n";
    assert_eq!(detect(&mut Cursor::new(ascii.as_bytes()), None).unwrap(), StreamFormat::StlAscii);
    let got = collect(ascii.as_bytes(), StreamFormat::StlAscii, 2.0).unwrap();
    assert_eq!(got.len(), 2);
    assert_eq!(got[1][0], Point3::new(2.0, 2.0, 0.0));
    // tricky PLY: both byte orders give the same fan-triangulated polygons
    let le = collect(&tricky_ply(false), StreamFormat::PlyBinary, 1.0).unwrap();
    let be = collect(&tricky_ply(true), StreamFormat::PlyBinary, 1.0).unwrap();
    assert_eq!(le.len(), 2 + 1 + 2);
    assert_eq!(le, be);
    let reference = triangles_of(&read_mesh(&tricky_ply(false), MeshFormat::Ply, &Limits::default()).unwrap());
    assert_eq!(le, reference);
}

#[test]
fn unsupported_and_hostile_inputs_are_errors() {
    let o = StreamOptions { limits: Limits::default(), scale: 1.0 };
    let run = |bytes: &[u8], f: StreamFormat| for_each_batch(&|| Ok(Cursor::new(bytes)), f, &o, &mut |_| Ok(()));
    // ASCII PLY, OBJ and unknown data are refused with a pointer to mesh.import
    let ascii_ply = "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n1 0 0\n0 1 0\n3 0 1 2\n";
    assert!(matches!(run(ascii_ply.as_bytes(), StreamFormat::PlyBinary), Err(IoError::Unsupported(_))));
    assert!(matches!(
        detect(&mut Cursor::new(&b"v 0 0 0\nv 1 0 0\nf 1 2 3\n"[..]), Some(MeshFormat::Obj)),
        Err(IoError::Unsupported(_))
    ));
    assert!(
        detect(&mut Cursor::new(&b"\x00\x01"[..]), None).is_err() || run(b"\x00\x01", StreamFormat::StlBinary).is_err()
    );
    // declared counts larger than the file, in every format
    let mut stl = vec![0u8; 84];
    stl[80..84].copy_from_slice(&u32::MAX.to_le_bytes());
    assert!(matches!(run(&stl, StreamFormat::StlBinary), Err(IoError::Malformed { .. })));
    let huge = "ply\nformat binary_little_endian 1.0\nelement vertex 4000000000\nproperty float x\nproperty float y\nproperty float z\nelement face 4000000000\nproperty list uchar int vertex_indices\nend_header\n";
    assert!(run(huge.as_bytes(), StreamFormat::PlyBinary).is_err());
    // limits: a tiny triangle limit
    let m = surfacecraft_geom::synth::cuboid(Point3::origin(), surfacecraft_geom::Vec3::new(1.0, 1.0, 1.0), 3);
    let stl = surfacecraft_io::write_mesh(&m, MeshFormat::Stl);
    let tight = StreamOptions { limits: Limits { max_triangles: 5, ..Limits::default() }, scale: 1.0 };
    let r = for_each_batch(&|| Ok(Cursor::new(stl.as_slice())), StreamFormat::StlBinary, &tight, &mut |_| Ok(()));
    assert!(matches!(r, Err(IoError::Limit(_))));
    // scaling can overflow to infinity
    let big = StreamOptions { limits: Limits::default(), scale: 1e308 };
    let wide = surfacecraft_io::write_mesh(
        &surfacecraft_geom::synth::cuboid(Point3::origin(), surfacecraft_geom::Vec3::new(3.0, 3.0, 3.0), 1),
        MeshFormat::Stl,
    );
    let r = for_each_batch(&|| Ok(Cursor::new(wide.as_slice())), StreamFormat::StlBinary, &big, &mut |_| Ok(()));
    assert!(r.is_err());
    // a PLY face that points past the vertices, a NaN vertex that is used
    let mut ply = tricky_ply(false);
    let at = ply.len() - 4;
    ply[at..].copy_from_slice(&99u32.to_le_bytes());
    assert!(matches!(run(&ply, StreamFormat::PlyBinary), Err(IoError::Malformed { .. })));
    // a very long ASCII line
    let long = format!("solid x\nfacet normal 0 0 0\nouter loop\nvertex {}\n", "1".repeat(10_000));
    assert!(run(long.as_bytes(), StreamFormat::StlAscii).is_err());
}

/// Random bytes and random corruptions of valid files never panic, and what comes out is finite.
#[test]
fn hostile_stream_bytes_never_panic() {
    let m = surfacecraft_geom::synth::cuboid(Point3::origin(), surfacecraft_geom::Vec3::new(1.0, 2.0, 3.0), 2);
    let ascii = "solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid t\n".as_bytes().to_vec();
    let valid: Vec<(StreamFormat, Vec<u8>)> = vec![
        (StreamFormat::StlBinary, surfacecraft_io::write_mesh(&m, MeshFormat::Stl)),
        (StreamFormat::StlAscii, ascii),
        (StreamFormat::PlyBinary, surfacecraft_io::write_mesh(&m, MeshFormat::Ply)),
        (StreamFormat::PlyBinary, tricky_ply(false)),
        (StreamFormat::PlyBinary, tricky_ply(true)),
    ];
    let mut rng = Rng::new(99);
    for round in 0..6000 {
        let (f, base) = &valid[round % valid.len()];
        let mut bytes = if round % 4 == 0 {
            (0..rng.index(300)).map(|_| rng.next_u64() as u8).collect::<Vec<u8>>()
        } else {
            base.clone()
        };
        for _ in 0..1 + rng.index(8) {
            if bytes.is_empty() {
                break;
            }
            let i = rng.index(bytes.len());
            bytes[i] = rng.next_u64() as u8;
        }
        if round % 3 == 0 {
            let cut = rng.index(bytes.len() + 1);
            bytes.truncate(cut);
        }
        let _ = detect(&mut Cursor::new(bytes.as_slice()), None);
        for fmt in [*f, StreamFormat::StlBinary, StreamFormat::StlAscii, StreamFormat::PlyBinary] {
            if let Ok(tris) = collect(&bytes, fmt, 1.0) {
                assert!(tris.iter().flatten().all(surfacecraft_geom::finite));
            }
        }
    }
}

#[test]
fn import_to_store_reports_missing_files_formats_and_scale() {
    let dir = TempDir::new("import-errors");
    let store = dir.path().join("s");
    let opts = StreamImportOptions::default();
    assert!(import_to_store(&dir.path().join("missing.stl"), &store, &opts).is_err());
    let obj = dir.path().join("a.obj");
    std::fs::write(&obj, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n").unwrap();
    assert!(matches!(import_to_store(&obj, &store, &opts), Err(IoError::Unsupported(_))));
    let stl = dir.path().join("t.stl");
    write_torus_stl(&stl, 16, 8);
    let bad = StreamImportOptions { scale: -1.0, ..StreamImportOptions::default() };
    assert!(import_to_store(&stl, &store, &bad).is_err());
    let small = StreamImportOptions {
        store: StoreParams { chunk_triangles: 64, levels: 2, ..StoreParams::default() },
        scale: 2.0,
        ..StreamImportOptions::default()
    };
    let r = import_to_store(&stl, &store, &small).unwrap();
    assert_eq!(r.summary.source_triangles, 2 * 16 * 8);
    assert!(r.summary.chunks >= 4);
    // scaled by 2: the torus (R = 40, r = 9) spans 2 * (40 + 9) in x
    assert!((r.summary.bbox.max.x - r.summary.bbox.min.x - 4.0 * 49.0).abs() < 4.0 * 49.0 * 0.05);
    assert!(matches!(import_to_store(&stl, &store, &small), Err(IoError::Store(_))));
    assert!(import_to_store(&stl, &store, &StreamImportOptions { overwrite: true, ..small }).is_ok());
}

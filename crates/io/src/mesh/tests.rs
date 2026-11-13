//! Tests of the mesh and point cloud readers and writers, and of hostile input.
use super::*;
use surfacecraft_geom::Vec3;

use surfacecraft_geom::{Rng, synth};

fn sample() -> Mesh {
    synth::cuboid(Point3::new(1.0, 2.0, 3.0), surfacecraft_geom::Vec3::new(4.0, 5.0, 6.0), 2)
}

#[test]
fn roundtrip_every_format() {
    let m = sample();
    for f in [MeshFormat::Stl, MeshFormat::Obj, MeshFormat::Ply] {
        let bytes = write_mesh(&m, f);
        assert_eq!(MeshFormat::sniff(&bytes), Some(f), "{f:?}");
        let back = read_mesh(&bytes, f, &Limits::default()).unwrap();
        assert_eq!(back.triangle_count(), m.triangle_count(), "{f:?}");
        assert_eq!(back.vertex_count(), m.vertex_count(), "{f:?}");
        assert!((back.area() - m.area()).abs() < 1e-4, "{f:?}");
    }
}

#[test]
fn ascii_stl_and_ply_variants() {
    let stl = "solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid t\n";
    let m = read_mesh(stl.as_bytes(), MeshFormat::Stl, &Limits::default()).unwrap();
    assert_eq!(m.triangle_count(), 1);
    let ply = "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0 1\n1 0 0 1\n1 1 0 1\n0 1 0 1\n4 0 1 2 3\n";
    let m = read_mesh(ply.as_bytes(), MeshFormat::Ply, &Limits::default()).unwrap();
    assert_eq!(m.triangle_count(), 2);
    // big endian float
    let mut be = b"ply\nformat binary_big_endian 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n".to_vec();
    for v in [0f32, 0., 0., 1., 0., 0., 0., 1., 0.] {
        be.extend_from_slice(&v.to_be_bytes());
    }
    be.push(3);
    for i in [0i32, 1, 2] {
        be.extend_from_slice(&i.to_be_bytes());
    }
    let m = read_mesh(&be, MeshFormat::Ply, &Limits::default()).unwrap();
    assert!((m.area() - 0.5).abs() < 1e-12);
    let obj = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf -4/1/1 -3/2/2 -2 -1\n";
    let m = read_mesh(obj.as_bytes(), MeshFormat::Obj, &Limits::default()).unwrap();
    assert_eq!(m.triangle_count(), 2);
}

/// A binary STL of a bumpy strip, more triangles than one parse chunk holds.
fn big_stl() -> (Vec<u8>, Mesh) {
    let strip = surfacecraft_geom::synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 30.0, 20.0, 150, 100);
    let wavy = Mesh {
        positions: strip.positions.iter().map(|p| Point3::new(p.x, p.y, (p.x * 0.7).sin())).collect(),
        triangles: strip.triangles,
    };
    assert!(wavy.triangle_count() > 3 * STL_CHUNK / 2);
    (write_mesh(&wavy, MeshFormat::Stl), wavy)
}

fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

#[test]
fn binary_stl_is_the_same_on_one_and_many_threads() {
    let (bytes, wavy) = big_stl();
    let read = || read_mesh(&bytes, MeshFormat::Stl, &Limits::default()).unwrap();
    let (one, many) = (in_pool(1, read), in_pool(8, read));
    assert_eq!(one, many);
    assert_eq!(one.triangle_count(), wavy.triangle_count());
    assert_eq!(one.vertex_count(), wavy.vertex_count());
}

#[test]
fn the_first_bad_triangle_of_a_binary_stl_is_reported_on_any_thread_count() {
    let (mut bytes, _) = big_stl();
    for t in [STL_CHUNK + 5, 5000] {
        let at = 84 + t * 50 + 12;
        bytes[at..at + 4].copy_from_slice(&f32::NAN.to_le_bytes());
    }
    let msg =
        |threads| in_pool(threads, || read_mesh(&bytes, MeshFormat::Stl, &Limits::default()).unwrap_err().to_string());
    let (a, b) = (msg(1), msg(8));
    assert_eq!(a, b);
    assert!(a.contains("triangle 5000"), "{a}");
}

#[test]
fn hostile_headers_are_refused() {
    // STL declaring a billion triangles in a tiny file
    let mut stl = vec![0u8; 84];
    stl[80..84].copy_from_slice(&1_000_000_000u32.to_le_bytes());
    assert!(read_mesh(&stl, MeshFormat::Stl, &Limits::default()).is_err());
    // PLY declaring huge element counts
    let ply = b"ply\nformat binary_little_endian 1.0\nelement vertex 4000000000\nproperty float x\nend_header\n";
    assert!(read_mesh(ply, MeshFormat::Ply, &Limits::default()).is_err());
    let ply = b"ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n3 0 0 7\n";
    assert!(read_mesh(ply, MeshFormat::Ply, &Limits::default()).is_err());
    let obj = b"v 0 0 0\nf 1 2 3\n";
    assert!(read_mesh(obj, MeshFormat::Obj, &Limits::default()).is_err());
    let obj = b"v nan 0 0\n";
    assert!(read_mesh(obj, MeshFormat::Obj, &Limits::default()).is_err());
    let limits = Limits { max_triangles: 1, ..Limits::default() };
    assert!(matches!(
        read_mesh(&write_mesh(&sample(), MeshFormat::Stl), MeshFormat::Stl, &limits),
        Err(IoError::Limit(_))
    ));
}

fn cloud_samples() -> Vec<(MeshFormat, Vec<u8>)> {
    let ascii = "ply
format ascii 1.0
element vertex 4
property float x
property float y
property float z
property float nx
property float ny
property float nz
end_header
0 0 0 0 0 1
1 0 0 0 0 1
1 1 0 0 0 1
0 1 0 0 0 1
";
    let mut bin = b"ply
format binary_little_endian 1.0
element vertex 3
property double x
property double y
property double z
property uchar red
end_header
"
    .to_vec();
    for v in [0f64, 0., 0., 1., 0., 0., 0., 1., 0.] {
        bin.extend_from_slice(&v.to_le_bytes());
    }
    bin.extend_from_slice(&[1, 2, 3]);
    vec![
        (MeshFormat::Ply, ascii.as_bytes().to_vec()),
        (MeshFormat::Ply, bin),
        (
            MeshFormat::Obj,
            b"v 0 0 0
v 1 0 0
v 1 1 0
"
            .to_vec(),
        ),
        (
            MeshFormat::Xyz,
            b"# scan
3
0 0 0 0 0 1
1,0,0,0,0,1
1;1;0;0;0;1
"
            .to_vec(),
        ),
    ]
}

#[test]
fn point_clouds_are_read_from_every_format() {
    let lim = Limits::default();
    let samples = cloud_samples();
    // PLY ascii with normals
    let Scan::Points(c) = read_scan(&samples[0].1, MeshFormat::Ply, &lim).unwrap() else { panic!("points") };
    assert_eq!(c.len(), 4);
    assert_eq!(c.normals.as_ref().map(Vec::len), Some(4));
    // PLY binary without normals, extra property ignored
    let Scan::Points(c) = read_scan(&samples[1].1, MeshFormat::Ply, &lim).unwrap() else { panic!("points") };
    assert_eq!((c.len(), c.normals.is_none()), (3, true));
    // OBJ with only vertices
    let Scan::Points(c) = read_scan(&samples[2].1, MeshFormat::Obj, &lim).unwrap() else { panic!("points") };
    assert_eq!(c.len(), 3);
    // XYZ: a count header, comments, three separators, normals in columns 4-6
    let Scan::Points(c) = read_scan(&samples[3].1, MeshFormat::Xyz, &lim).unwrap() else { panic!("points") };
    assert_eq!(c.len(), 3);
    assert_eq!(c.normals.as_ref().and_then(|n| n.first()).copied(), Some(Vec3::z()));
    assert_eq!(MeshFormat::sniff(&samples[3].1), Some(MeshFormat::Xyz));
    assert_eq!(MeshFormat::from_extension("ASC"), Some(MeshFormat::Xyz));
    // four columns (intensity) carry no normals
    let Scan::Points(c) = read_scan(
        b"1 2 3 0.5
4 5 6 0.7
",
        MeshFormat::Xyz,
        &lim,
    )
    .unwrap() else {
        panic!("points")
    };
    assert!(c.normals.is_none());
    // meshes still come back as meshes, and `read_mesh` still refuses a cloud
    let bytes = write_mesh(&sample(), MeshFormat::Ply);
    assert!(matches!(read_scan(&bytes, MeshFormat::Ply, &lim).unwrap(), Scan::Mesh(_)));
    assert!(read_mesh(&samples[0].1, MeshFormat::Ply, &lim).is_err());
    assert!(read_mesh(&samples[3].1, MeshFormat::Xyz, &lim).is_err());
    // a mesh read as points gives its vertices; a mesh written as XYZ reads back as points
    assert_eq!(read_points(&bytes, MeshFormat::Ply, &lim).unwrap().len(), sample().vertex_count());
    let xyz = write_mesh(&sample(), MeshFormat::Xyz);
    let c = read_points(&xyz, MeshFormat::Xyz, &lim).unwrap();
    assert_eq!((c.len(), c.normals.is_some()), (sample().vertex_count(), true));
}

#[test]
fn hostile_point_clouds_are_refused() {
    let lim = Limits::default();
    for bad in [
        &b"1 2 3
4 five 6
"[..],
        b"nan 0 0
",
        b"1 2
",
        b"",
        b"# only a comment
",
        b"1 2 3
x y z
",
    ] {
        assert!(read_scan(bad, MeshFormat::Xyz, &lim).is_err(), "{:?}", String::from_utf8_lossy(bad));
    }
    let small = Limits { max_vertices: 2, ..Limits::default() };
    assert!(matches!(
        read_scan(
            b"0 0 0
1 0 0
2 0 0
",
            MeshFormat::Xyz,
            &small
        ),
        Err(IoError::Limit(_))
    ));
    assert!(matches!(
        read_scan(&cloud_samples()[0].1, MeshFormat::Ply, &Limits { max_vertices: 3, ..Limits::default() }),
        Err(IoError::Limit(_))
    ));
    // a vertex with a NaN normal is "unknown", one with a NaN position is refused
    let ply = "ply
format ascii 1.0
element vertex 1
property float x
property float y
property float z
property float nx
property float ny
property float nz
end_header
0 0 0 nan 0 0
";
    let Scan::Points(c) = read_scan(ply.as_bytes(), MeshFormat::Ply, &lim).unwrap() else { panic!("points") };
    assert_eq!(c.normals.as_ref().and_then(|n| n.first()).copied(), Some(Vec3::zeros()));
}

/// Random bytes and random corruptions of valid files never panic (spike §3.2 item 7).
#[test]
fn hostile_bytes_never_panic() {
    let mut rng = Rng::new(42);
    let mut valid: Vec<(MeshFormat, Vec<u8>)> = [MeshFormat::Stl, MeshFormat::Obj, MeshFormat::Ply, MeshFormat::Xyz]
        .iter()
        .map(|&f| (f, write_mesh(&sample(), f)))
        .collect();
    valid.extend(cloud_samples());
    for round in 0..6000 {
        let (f, base) = &valid[round % valid.len()];
        let mut bytes = if round % 4 == 0 {
            (0..rng.index(400)).map(|_| rng.next_u64() as u8).collect::<Vec<u8>>()
        } else {
            base.clone()
        };
        let flips = 1 + rng.index(8);
        for _ in 0..flips {
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
        for fmt in [*f, MeshFormat::Stl, MeshFormat::Obj, MeshFormat::Ply, MeshFormat::Xyz] {
            if let Ok(m) = read_mesh(&bytes, fmt, &Limits::default()) {
                assert!(m.validate().is_ok());
            }
            match read_scan(&bytes, fmt, &Limits::default()) {
                Ok(Scan::Mesh(m)) => assert!(m.validate().is_ok()),
                Ok(Scan::Points(c)) => assert!(c.validate().is_ok() && !c.is_empty()),
                Err(_) => {}
            }
            if let Ok(c) = read_points(&bytes, fmt, &Limits::default()) {
                assert!(c.validate().is_ok());
            }
        }
        let _ = MeshFormat::sniff(&bytes);
    }
}

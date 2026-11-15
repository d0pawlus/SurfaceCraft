//! A mesh file read reduced (`stream::reduce`): a file above a triangle limit comes in as a mesh
//! within a stated number of triangles, close to the surface of the file, whatever the limit of
//! the ordinary reader.
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3, synth};
use surfacecraft_io::stream::StreamOptions;
use surfacecraft_io::stream::reduce::{declared_triangles, read_reduced};
use surfacecraft_io::{IoError, Limits, MeshFormat, read_mesh, write_mesh};

const BIG_R: f64 = 30.0;
const SMALL_R: f64 = 8.0;

fn torus() -> Mesh {
    let mut m = synth::torus(Point3::origin(), Vec3::z(), BIG_R, SMALL_R, 400, 200);
    m.weld(1e-9);
    m
}

fn options() -> StreamOptions {
    StreamOptions { limits: Limits::default(), scale: 1.0 }
}

/// Distance of `p` from the torus surface.
fn off_torus(p: &Point3) -> f64 {
    let ring = (p.x * p.x + p.y * p.y).sqrt() - BIG_R;
    ((ring * ring + p.z * p.z).sqrt() - SMALL_R).abs()
}

#[test]
fn a_file_above_the_target_is_reduced_within_it_and_stays_on_the_surface() {
    let m = torus();
    let n = m.triangle_count();
    for format in [MeshFormat::Stl, MeshFormat::Ply] {
        let bytes = write_mesh(&m, format);
        assert_eq!(declared_triangles(&bytes, Some(format)), Some(n as u64), "{format:?}");
        let target = n / 8;
        let r = read_reduced(&bytes, Some(format), &options(), target).unwrap().unwrap();
        assert_eq!(r.source_triangles, n as u64);
        let got = r.mesh.triangle_count();
        // within the target, and not wastefully far below it
        assert!(got <= target, "{format:?}: {got} > {target}");
        assert!(got * 4 >= target, "{format:?}: {got} is far below {target}");
        r.mesh.validate().unwrap();
        // the stated bound holds against the exact surface (√3·h, plus the chord error of the file)
        let bound = 3f64.sqrt() * r.cell_size + 0.05;
        let worst = r.mesh.positions.iter().map(off_torus).fold(0.0, f64::max);
        assert!(worst <= bound, "{format:?}: {worst} vs {bound}");
        // and the file's surface is covered: its vertices are near the reduced mesh
        let bvh = Bvh::build(&r.mesh);
        let far = m.positions.iter().step_by(37).map(|p| bvh.closest(p, f64::INFINITY).unwrap().distance);
        assert!(far.fold(0.0, f64::max) <= 2.0 * bound);
    }
}

#[test]
fn the_reduced_read_ignores_the_limit_that_stops_the_ordinary_reader() {
    let m = torus();
    let bytes = write_mesh(&m, MeshFormat::Stl);
    let tight = Limits { max_triangles: 10_000, max_vertices: 10_000, ..Limits::default() };
    assert!(matches!(read_mesh(&bytes, MeshFormat::Stl, &tight), Err(IoError::Limit(_))));
    let r = read_reduced(&bytes, Some(MeshFormat::Stl), &options(), tight.max_triangles).unwrap().unwrap();
    assert!(r.mesh.triangle_count() <= tight.max_triangles);
    assert!(r.mesh.vertex_count() <= tight.max_vertices);
}

#[test]
fn a_file_within_the_target_is_left_to_the_ordinary_reader() {
    let m = synth::sphere(Point3::origin(), 10.0, 32, 16);
    let bytes = write_mesh(&m, MeshFormat::Stl);
    assert_eq!(read_reduced(&bytes, Some(MeshFormat::Stl), &options(), 1_000_000).unwrap(), None);
}

#[test]
fn the_scale_applies_to_the_reduced_mesh() {
    let m = torus();
    let bytes = write_mesh(&m, MeshFormat::Stl);
    let opts = StreamOptions { scale: 25.4, ..options() };
    let r = read_reduced(&bytes, None, &opts, 20_000).unwrap().unwrap();
    let size = r.mesh.bbox().size();
    assert!((size.x - 2.0 * (BIG_R + SMALL_R) * 25.4).abs() < 25.4, "{size:?}");
}

#[test]
fn formats_that_cannot_be_streamed_and_hostile_files_are_errors_not_panics() {
    let m = synth::sphere(Point3::origin(), 10.0, 32, 16);
    let obj = write_mesh(&m, MeshFormat::Obj);
    assert!(matches!(read_reduced(&obj, Some(MeshFormat::Obj), &options(), 10), Err(IoError::Unsupported(_))));
    assert_eq!(declared_triangles(&obj, Some(MeshFormat::Obj)), None);
    let bad_scale = StreamOptions { scale: f64::NAN, ..options() };
    assert!(read_reduced(&write_mesh(&m, MeshFormat::Stl), None, &bad_scale, 10).is_err());
    // every triangle degenerate (no area): still an answer, never a panic or an endless loop
    let flat = Mesh { positions: vec![Point3::origin(); 3], triangles: vec![[0, 1, 2]; 500] };
    let _ = read_reduced(&write_mesh(&flat, MeshFormat::Stl), Some(MeshFormat::Stl), &options(), 10);
    // truncated and random bytes
    let stl = write_mesh(&m, MeshFormat::Stl);
    for cut in [0, 10, 83, 84, 100, stl.len() / 2, stl.len() - 1] {
        let _ = read_reduced(&stl[..cut], Some(MeshFormat::Stl), &options(), 10);
        let _ = declared_triangles(&stl[..cut], None);
    }
}

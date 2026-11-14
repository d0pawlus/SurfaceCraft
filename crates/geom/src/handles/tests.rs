use super::*;
use crate::synth_wall::{Wall, wall};
use crate::{KdTree, Rng, synth};

/// A curved thin wall, 80 mm square in cells of 1 mm, sheets 0.6 mm apart, with one-cell tunnels every
/// three cells over the upper half (`y` from 41 mm) and the `extra` tubes.
fn tunnelled(extra: &[(usize, usize, usize)]) -> (Wall, Mesh) {
    let mut tubes: Vec<(usize, usize, usize)> = extra.to_vec();
    for j in (41..77).step_by(3) {
        for i in (2..77).step_by(3) {
            tubes.push((i, j, 1));
        }
    }
    let w = Wall { size: 80.0, cells: 80, gap: 0.6, bend: 5.0, tubes };
    let m = wall(&w);
    (w, m)
}

fn params(max_size: f64) -> Params {
    Params { max_size, tolerance: 0.2 }
}

fn rim(mesh: &Mesh) -> Vec<[u64; 3]> {
    let mut r: Vec<[u64; 3]> = edit::boundary_loops(mesh)
        .iter()
        .flatten()
        .map(|&v| {
            let p = mesh.positions[v as usize];
            [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]
        })
        .collect();
    r.sort_unstable();
    r
}

#[test]
fn a_wall_with_hundreds_of_small_tunnels_loses_them_and_keeps_its_rim_and_its_smooth_half() {
    // Arrange
    let (w, mut m) = tunnelled(&[]);
    synth::add_noise(&mut m, 0.02, &mut Rng::new(3));
    let before = m.clone();
    let tunnels = w.built_tubes().len();

    // Act
    let rep = close_handles(&mut m, &params(8.0), &Cancel::new()).unwrap();

    // Assert
    let within = rep.removed_within as f64 / rep.removed_vertices.max(1) as f64;
    eprintln!(
        "tunnels {tunnels}: genus {:?} -> {:?}, closed {} in {} parts, refused {:?}, area cut out {:.3}, removed within 0.2 mm {within:.3} (max {:.3})",
        rep.genus_before,
        rep.genus_after,
        rep.handles_closed,
        rep.closed.len(),
        rep.refused,
        rep.removed_area_share,
        rep.removed_max_distance
    );
    assert!(tunnels >= 300, "{tunnels}");
    assert_eq!(rep.genus_before, Some(tunnels));
    assert!(3 * rep.handles_closed >= 2 * tunnels, "at least two thirds closed: {}", rep.handles_closed);
    assert_eq!(rep.genus_after, Some(tunnels - rep.handles_closed), "none made");
    assert_eq!((rep.pieces_before, rep.pieces_after), (1, 1));
    assert_eq!((rep.loops_before, rep.loops_after), (1, 1));
    assert_eq!(rim(&before), rim(&m), "the rim is where it was, vertex for vertex");
    // the smooth half (below y = 33 mm, eight cells, the largest size, from the first tunnel) is untouched
    let tree = KdTree::build(&m.positions);
    let smooth: Vec<&Point3> = before.positions.iter().filter(|p| p.y < 33.0).collect();
    assert!(smooth.iter().all(|p| tree.nearest(p).is_some_and(|(_, d2)| d2 == 0.0)));
    assert!(within > 0.5, "most of what was cut out lies on the result: {within}");
    assert!(m.validate().is_ok() && pieces::pieces(&m).iter().all(|p| p.non_manifold_edges == 0));
}

#[test]
fn real_holes_wider_than_max_size_are_kept() {
    // Arrange: four 10 mm holes through the wall below the tunnels
    let holes = [(4, 4, 10), (20, 4, 10), (36, 4, 10), (4, 20, 10)];
    let (w, mut m) = tunnelled(&holes);
    let side = w.cells + 1;
    let hole_vertices: Vec<Point3> = holes
        .iter()
        .flat_map(|&(i, j, k)| {
            (j..=j + k)
                .flat_map(move |y| (i..=i + k).map(move |x| (x, y)))
                .filter(move |&(x, y)| x == i || x == i + k || y == j || y == j + k)
        })
        .flat_map(|(x, y)| [y * side + x, side * side + y * side + x])
        .map(|v| m.positions[v])
        .collect();

    // Act
    let rep = close_handles(&mut m, &params(8.0), &Cancel::new()).unwrap();

    // Assert
    assert!(rep.handles_closed > 0);
    assert!(rep.genus_after.is_some_and(|g| g >= holes.len()), "{:?}", rep.genus_after);
    let tree = KdTree::build(&m.positions);
    assert!(
        hole_vertices.iter().all(|p| tree.nearest(p).is_some_and(|(_, d2)| d2 == 0.0)),
        "the holes are as they were"
    );
    assert_eq!(rep.pieces_after, 1);
}

#[test]
fn meshes_without_small_handles_come_back_bit_for_bit() {
    let open = synth::wave(Point3::origin(), 40.0, 30.0, 2.0, 40, 30);
    let closed = synth::sphere(Point3::origin(), 10.0, 48, 24);
    let torus = synth::torus(Point3::origin(), crate::Vec3::z(), 20.0, 5.0, 160, 40);
    let big_holes = wall(&Wall { size: 40.0, cells: 40, gap: 0.6, bend: 2.0, tubes: vec![(5, 5, 10), (20, 20, 10)] });
    for (name, mesh) in [("open", open), ("closed", closed), ("torus", torus), ("big holes", big_holes)] {
        let mut m = mesh.clone();
        let rep = close_handles(&mut m, &params(8.0), &Cancel::new()).unwrap();
        assert!(!rep.changed && rep.closed.is_empty(), "{name}: {:?}", rep.refused);
        assert_eq!(m.positions, mesh.positions, "{name}");
        assert_eq!(m.triangles, mesh.triangles, "{name}");
    }
}

#[test]
fn hostile_input_never_panics() {
    let cancel = Cancel::new();
    let mut empty = Mesh::default();
    let rep = close_handles(&mut empty, &params(5.0), &cancel).unwrap();
    assert!(!rep.changed);
    let (_, base) = tunnelled(&[]);
    for bad in [f64::NAN, -1.0, 0.0, f64::INFINITY] {
        assert!(matches!(close_handles(&mut base.clone(), &params(bad), &cancel), Err(HandleError::Params(_))));
        let p = Params { max_size: 5.0, tolerance: bad };
        assert!(matches!(close_handles(&mut base.clone(), &p, &cancel), Err(HandleError::Params(_))));
    }
    let mut nan = base.clone();
    nan.positions[7].x = f64::NAN;
    assert!(matches!(close_handles(&mut nan, &params(5.0), &cancel), Err(HandleError::Invalid(_))));
    let mut index = base.clone();
    index.triangles.push([0, 1, u32::MAX]);
    assert!(matches!(close_handles(&mut index, &params(5.0), &cancel), Err(HandleError::Invalid(_))));
    // a fin on an edge of a tunnelled region: its vertices are left alone, the rest still closes
    let mut fin = base.clone();
    let t = fin.triangles[fin.triangles.len() / 2];
    fin.positions.push(Point3::new(30.0, 60.0, 12.0));
    fin.triangles.push([t[0], t[1], (fin.positions.len() - 1) as u32]);
    let rep = close_handles(&mut fin, &params(8.0), &cancel).unwrap();
    assert!(rep.handles_closed > 0 && fin.validate().is_ok());
    // a mesh that is all tunnels, every other cell
    let tubes: Vec<(usize, usize, usize)> =
        (1..39).step_by(2).flat_map(|j| (1..39).step_by(2).map(move |i| (i, j, 1))).collect();
    let mut sponge = wall(&Wall { size: 40.0, cells: 40, gap: 0.4, bend: 1.0, tubes });
    let rep = close_handles(&mut sponge, &params(8.0), &cancel).unwrap();
    assert!(
        sponge.validate().is_ok() && rep.genus_after <= rep.genus_before,
        "{:?}",
        (rep.genus_before, rep.genus_after)
    );
    assert_eq!(rep.pieces_after, rep.pieces_before);
    // a max_size below four median edges does nothing
    let mut tiny = base.clone();
    let rep = close_handles(&mut tiny, &params(1.0), &cancel).unwrap();
    assert!(rep.scales.is_empty() && !rep.changed && tiny.triangles == base.triangles);
}

#[test]
fn a_cancelled_call_changes_nothing() {
    let (_, mut m) = tunnelled(&[]);
    let before = m.clone();
    let cancel = Cancel::new();
    cancel.cancel();
    assert!(matches!(close_handles(&mut m, &params(8.0), &cancel), Err(HandleError::Cancelled)));
    assert_eq!(m.triangles, before.triangles);
}

#[test]
fn the_result_does_not_depend_on_the_threads() {
    let (_, base) = tunnelled(&[(4, 4, 10)]);
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let mut m = base.clone();
            close_handles(&mut m, &params(8.0), &Cancel::new()).unwrap();
            m
        })
    };
    let (one, four) = (run(1), run(4));
    assert_eq!(one.positions, four.positions);
    assert_eq!(one.triangles, four.triangles);
}

/// A million triangles: 500 x 500 cells per sheet with 2000 tunnels. Prints the time.
#[test]
#[ignore = "performance: a million triangles (run with --profile ci)"]
fn a_million_triangles() {
    let tubes: Vec<(usize, usize, usize)> = (0..2000).map(|k| (2 + (k % 50) * 9, 2 + (k / 50) * 9, 1)).collect();
    let mut m = wall(&Wall { size: 500.0, cells: 500, gap: 0.6, bend: 20.0, tubes });
    let t0 = std::time::Instant::now();
    let rep = close_handles(&mut m, &params(8.0), &Cancel::new()).unwrap();
    eprintln!(
        "{} triangles: genus {:?} -> {:?}, {} closed in {:.1} s",
        m.triangle_count(),
        rep.genus_before,
        rep.genus_after,
        rep.handles_closed,
        t0.elapsed().as_secs_f64()
    );
    assert_eq!(rep.genus_after, Some(0));
}

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
    let within = rep.removed_within as f64 / rep.removed_samples.max(1) as f64;
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

fn single_tunnel() -> Mesh {
    wall(&Wall { size: 20.0, cells: 20, gap: 0.6, bend: 0.0, tubes: vec![(10, 10, 1)] })
}

/// A bore of 2 x 2 mm through a wall 8 mm thick, whose walls are long triangles without a vertex inside:
/// the cut-out wall is not covered by the caps, so it is refused (it would have read as 0 by vertices).
#[test]
fn a_deep_bore_through_a_thick_wall_is_not_capped() {
    let mut m = wall(&Wall { size: 30.0, cells: 30, gap: 8.0, bend: 0.0, tubes: vec![(13, 13, 2)] });
    let before = m.clone();
    let rep = close_handles(&mut m, &params(14.0), &Cancel::new()).unwrap();
    eprintln!("{:?} closed {} {:?} far {}", rep.refused, rep.handles_closed, rep.closed, rep.removed_max_distance);
    assert_eq!(rep.handles_closed, 0);
    assert!(rep.refused.uncovered + rep.refused.fill_far > 0, "{:?}", rep.refused);
    assert_eq!(m.triangles, before.triangles);
}

/// Two sheets joined only by tunnels: the last tunnel holds them together and is never cut.
#[test]
fn the_surface_is_never_cut_in_two() {
    let mut m = wall(&Wall { size: 30.0, cells: 30, gap: 0.6, bend: 0.0, tubes: vec![(8, 8, 1), (20, 20, 1)] });
    let positions = m.positions.clone();
    m.triangles.retain(|t| !t.iter().all(|&v| positions[v as usize].x == 0.0));
    assert_eq!(pieces::pieces(&m)[0].genus(), Some(1));
    let rep = close_handles(&mut m, &params(8.0), &Cancel::new()).unwrap();
    assert_eq!(pieces::pieces(&m).len(), 1, "{:?}", rep.refused);
    assert!(rep.handles_closed <= 1);
    assert!(rep.refused.splits > 0 || rep.handles_closed == 1);
}

/// A small closed torus that a ball covers whole: cutting it would delete it, so every candidate is
/// refused explicitly and the mesh comes back bit for bit.
#[test]
fn a_whole_closed_piece_with_genus_is_refused_not_emptied() {
    let mut torus = synth::torus(Point3::origin(), crate::Vec3::z(), 1.5, 0.5, 60, 20);
    // a large sheet far away, so that the balls may be as wide as the torus
    torus.append(&synth::wave(Point3::new(20.0, 0.0, 0.0), 40.0, 30.0, 1.0, 40, 30));
    let mut m = torus.clone();
    let rep = close_handles(&mut m, &params(50.0), &Cancel::new()).unwrap();
    assert!(rep.refused.whole_pieces > 0, "{rep:?}");
    assert!(!rep.changed);
    assert_eq!((m.positions, m.triangles), (torus.positions, torus.triangles));
}

#[test]
fn a_piece_that_vanishes_whole_is_a_loose_piece() {
    let mut both = synth::sphere(Point3::origin(), 1.0, 12, 6);
    let far = synth::sphere(Point3::new(5.0, 0.0, 0.0), 1.0, 12, 6);
    let n = both.triangles.len();
    both.append(&far);
    let mut out = both.clone();
    out.triangles.truncate(n);
    let verts: Vec<u32> = (both.positions.len() as u32 / 2..both.positions.len() as u32).collect();
    let tris: Vec<u32> = (n as u32..both.triangles.len() as u32).collect();
    let c = cells::Candidate { triangles: tris, vertices: verts, genus: 1, loops: vec![0] };
    assert_eq!(cut::loosened(&both, &out, &[c], &[false]), vec![0]);
}

#[test]
fn a_fill_far_from_what_it_replaced_is_refused() {
    let m = single_tunnel();
    let topo = Topo::new(&m);
    let (found, _) = cells::candidates(&m, &topo, 6.0, 0);
    let cands = cells::pick(&topo, found, m.positions.len());
    assert!(!cands.is_empty());
    let tight = cut::Bounds { tolerance: 1e-9, far_limit: 10.0 };
    let a = cut::attempt(&m, &cands, &vec![false; cands.len()], tight, &Cancel::new()).unwrap();
    assert!(a.failed.iter().any(|f| f.1 == Refusal::FillFar), "{:?}", a.failed);
    let fair = cut::Bounds { tolerance: 0.2, far_limit: 1.5 };
    let a = cut::attempt(&m, &cands, &vec![false; cands.len()], fair, &Cancel::new()).unwrap();
    assert!(a.failed.is_empty(), "{:?}", a.failed);
    let cancel = Cancel::new();
    cancel.cancel();
    assert!(cut::attempt(&m, &cands, &vec![false; cands.len()], fair, &cancel).is_none());
}

#[test]
fn a_hole_that_stays_open_is_unfillable() {
    let mut m = synth::sphere(Point3::origin(), 1.0, 12, 6);
    let t = m.triangles.remove(20);
    let mut owner = vec![u32::MAX; m.positions.len()];
    for v in t {
        owner[v as usize] = 0;
    }
    assert!(cut::unfilled(&m, &owner).contains(&(0, Refusal::Unfillable)));
}

#[test]
fn a_failed_candidate_grows_by_a_ring_and_keeps_its_handle() {
    let m = single_tunnel();
    let topo = Topo::new(&m);
    let (found, _) = cells::candidates(&m, &topo, 6.0, 0);
    let c = cells::pick(&topo, found, m.positions.len()).remove(0);
    let g = cells::grow(&m, &topo, &c).unwrap();
    assert!(g.triangles.len() > c.triangles.len());
    assert!(c.triangles.iter().all(|t| g.triangles.binary_search(t).is_ok()));
    assert_eq!(g.handles(), c.handles());
}

/// One vertex with 50 000 triangles around it: the fans are counted at once (a quadratic count would hang).
#[test]
fn fans_of_a_vertex_of_huge_valence_are_counted() {
    let k = 50_000u32;
    let mut positions = vec![Point3::origin()];
    for i in 0..k {
        let a = std::f64::consts::TAU * f64::from(i) / f64::from(k);
        positions.push(Point3::new(a.cos(), a.sin(), 0.0));
    }
    let mut triangles: Vec<[u32; 3]> = (0..k).map(|i| [0, 1 + i, 1 + (i + 1) % k]).collect();
    let fans = cut::Fans::new(&Mesh { positions: positions.clone(), triangles: triangles.clone() });
    assert_eq!(fans.count(&Mesh { positions: positions.clone(), triangles: triangles.clone() }, 0), 1);
    // a second disk on the same centre vertex: two fans
    let base = positions.len() as u32;
    for i in 0..8u32 {
        let a = std::f64::consts::TAU * f64::from(i) / 8.0;
        positions.push(Point3::new(a.cos(), 0.0, a.sin() + 3.0));
    }
    triangles.extend((0..8).map(|i| [0, base + i, base + (i + 1) % 8]));
    let m = Mesh { positions, triangles };
    assert_eq!(cut::Fans::new(&m).count(&m, 0), 2);
}

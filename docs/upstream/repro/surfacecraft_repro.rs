//! Kernel-only reproductions for solidcore (public API of solidcore-ops and solidcore-record only).
//! Copy to `crates/solidcore-ops/tests/surfacecraft_repro.rs` of a solidcore checkout and run:
//!   cargo test -p solidcore-ops --test surfacecraft_repro -- --skip healer_time --skip trusted_from_record_time
//! (the two skipped tests time records of up to 240 000 faces and take minutes).
//! `SC_REPRO_FIXES=1` turns every upstream fix on; unset, the kernel runs as an application that
//! never sets fixes.
//!
//! Result on solidcore `f02d13e9ce38f45a8cd4b5a794fa09ae81d2221e` (2026-11-15, dev profile,
//! Windows x86_64); every test asserts the expected, correct behaviour:
//!   fillet_of_box_edge_is_a_cylinder_face             fails, fixes off and on (cone with half angle 0)
//!   near_coaxial_cylinders_intersect                  fails: status 1001 (fixes off), NotSolid (fixes on)
//!   turned_box_reads_back_from_its_record             fails, fixes off and on ("the plane's axes are not square")
//!   union_of_three_chained_rotated_copies_reads_back  fails, fixes off and on (same refusal)
//!   window_along_a_plane_normal_reads_back            fails, fixes off and on (same refusal)
//!   shell_of_filleted_pocketed_plate_reads_back       fails with fixes off (same refusal), passes with fixes on
//!   record_of_a_large_closed_bspline_shell_validates  fails, fixes off and on ("portable B-rep: item budget")
//!   healer_time_*, trusted_from_record_time_*         not run (timing; needs a quiet machine)
//!
//! Minimal reproductions of kernel problems found on solidcore `4f2da85` while building a
//! scan-to-CAD application on it (hybrid solids, parametric features, large B-spline shells),
//! ported to solidcore's own public API so one run says, for each, whether it still fails.
//!
//! Every test asserts the EXPECTED, correct behaviour and none is ignored: a test that fails
//! names what the kernel did instead (its status, the surface kinds, the volume). The "Actual"
//! lines of the doc comments are what `4f2da85` did. The STEP problem (a) is in
//! `crates/solidcore-step/tests/surfacecraft_repro_step.rs`, the one test that needs the STEP
//! reader and writer. Problems 9 and 9b time large records and take minutes.
//!
//! The application's thin wrappers are written out: a body handed over as a record (`to_record`)
//! and read again by the healer (`from_record(.., false)`) before every operation, and its
//! row-major column-vector matrices handed to the kernel transposed.
#![allow(
    clippy::disallowed_methods, // the tests' own closed forms, not ops geometry
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing,
    clippy::float_arithmetic
)] // test helpers

use std::collections::BTreeMap;

use solidcore_ops as ops;
use solidcore_record::brep::{
    Brep, Curve, Curve2, Edge, EdgeUseRef, Face, Loop, OrientedEdge, OrientedFace, Region, Shell, Surface,
    Vertex, VERSION,
};

/// The value of an ops result, or a panic naming the call and the kernel's own words.
/// Turns every upstream fix on for this thread when `SC_REPRO_FIXES=1` (off by default, as an
/// application that never sets fixes runs).
fn fixes() {
    if std::env::var("SC_REPRO_FIXES").as_deref() == Ok("1") {
        ops::session::set_thread_fixes(solidcore::kernel::UpstreamFixes::NAMES.iter().copied()).unwrap();
    }
}

fn ok<T>(what: &str, result: ops::Result<T>) -> T {
    match result {
        Ok(value) => value,
        Err(e) => panic!("{what} failed: {e} ({e:?})"),
    }
}

/// The body as its record: what the application's `solid::make_*` and every operation of it
/// returned.
fn record_of(what: &str, body: &ops::Body) -> Brep {
    ok(what, body.to_record()).0
}

/// The record as a body read by the healer: what the application did before each operation (its
/// records stay under 2000 faces, so never the trusted conversion).
fn decode(what: &str, record: &Brep) -> ops::Body {
    ok(what, ops::Body::from_record(record, false)).0
}

/// The application's row-major column-vector matrix (`p' = M p`, translation in the last column)
/// as the kernel's row-vector form (`p' = p A + t`, translation in the last row): the transpose.
fn to_kernel_matrix(m: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
}

/// Problem b: filleting one straight edge of a box gives a cylindrical face.
///
/// Input: box 20 × 10 × 5, `solid::fillet` on edge 0 with radius 2.
/// Expected: exactly one `Surface::Cylinder` face (the round).
/// Actual: the round comes back as a `Surface::Cone` with half angle 0 (SMLib's own form of a
/// cylinder from `ops::blend`), so every consumer must special-case it.
#[test]
fn fillet_of_box_edge_is_a_cylinder_face() {
    fixes();
    let made = ok("make_box", ops::make_box([0.0; 3], [20.0, 10.0, 5.0]));
    let b = record_of("make_box: to_record", &made);
    // `solid::fillet`: the record read by the healer, the round of edge 0, the record again
    let body = decode("fillet: from_record", &b);
    let _blended = ok("blend", ops::blend(&body, ops::Blend::Round, &[0], &[2.0], &[], ops::Corners::Filled));
    let f = record_of("fillet: to_record", &body);
    let kinds: Vec<String> = f
        .faces
        .iter()
        .map(|face| match &face.surface {
            Surface::Plane { .. } | Surface::PlaneThrough { .. } => "plane".to_string(),
            Surface::Cylinder { .. } => "cylinder".to_string(),
            Surface::Cone { half_angle, .. } => format!("cone(half_angle {half_angle})"),
            _ => "other".to_string(),
        })
        .collect();
    let cylinders = f.faces.iter().filter(|face| matches!(face.surface, Surface::Cylinder { .. })).count();
    assert_eq!(cylinders, 1, "one cylindrical round expected, face kinds were {kinds:?}");
}

/// Problem c: two cylinders whose axes are 0.01° apart (same radius, both based at the origin)
/// intersect in a solid.
///
/// Input: cylinder r 5 h 20 at the origin, and the same cylinder turned 0.01° about x.
/// Expected: `ops::boolean(.., Intersect)` returns a solid of volume between 1400 and π·25·20.
/// Actual: the boolean fails with kernel status 1001 (`SM_ERR`), so no lens solid is built.
/// (The same pair turned 0.5° intersects fine, volume ≈ 1551.9 mm³, so the failure needs a
/// near-coincident axis, as the fit noise of scanned revolved surfaces gives.)
#[test]
fn near_coaxial_cylinders_intersect() {
    fixes();
    let a = record_of("make_cylinder a: to_record", &ok("make_cylinder a", ops::make_cylinder([0.0; 3], 5.0, 20.0)));
    let c = 0.01_f64.to_radians().cos();
    let s = 0.01_f64.to_radians().sin();
    let turn = [[1.0, 0.0, 0.0, 0.0], [0.0, c, -s, 0.0], [0.0, s, c, 0.0], [0.0, 0.0, 0.0, 1.0]];
    // `solid::transform(&solid::make_cylinder(..), &turn)`: made, read, moved, written
    let second = record_of("make_cylinder b: to_record", &ok("make_cylinder b", ops::make_cylinder([0.0; 3], 5.0, 20.0)));
    let moving = decode("transform: from_record", &second);
    ok("transform", moving.transform(&to_kernel_matrix(&turn)));
    let tilted = record_of("transform: to_record", &moving);
    let ba = decode("from_record a", &a);
    let bb = decode("from_record b", &tilted);
    let out = ok("boolean(Intersect)", ops::boolean::boolean(ba, bb, ops::BooleanOp::Intersect));
    let body = out.unwrap_or_else(|| panic!("the intersection came back empty (None), a lens solid was expected"));
    let v = ok("volume", body.volume(1e-6));
    assert!((1400.0..=std::f64::consts::PI * 25.0 * 20.0).contains(&v), "lens volume {v}");
}

type V = [f64; 3];

fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn unit(a: V) -> V {
    let l = dot(a, a).sqrt();
    [a[0] / l, a[1] / l, a[2] / l]
}

fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(a: V, s: f64) -> V {
    [a[0] * s, a[1] * s, a[2] * s]
}

/// The turn of `angle` radians about the unit `axis` through the origin, as `Body::transform`
/// takes it: rows 0 to 2 the images of x, y and z, row 3 the translation (`p' = p·A + t`).
fn turn(axis: V, angle: f64) -> [[f64; 4]; 4] {
    let a = unit(axis);
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    [
        [t * a[0] * a[0] + c, t * a[0] * a[1] + s * a[2], t * a[0] * a[2] - s * a[1], 0.0],
        [t * a[0] * a[1] - s * a[2], t * a[1] * a[1] + c, t * a[1] * a[2] + s * a[0], 0.0],
        [t * a[0] * a[2] + s * a[1], t * a[1] * a[2] - s * a[0], t * a[2] * a[2] + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// `to_record` then `from_record`: the round trip every consumer of the record format makes.
fn round_trip(body: &ops::Body, what: &str) -> Result<ops::Body, String> {
    let (rec, _) = body.to_record().map_err(|e| format!("{what}: to_record: {e:?}"))?;
    ops::Body::from_record(&rec, false).map(|(b, _)| b).map_err(|e| format!("{what}: from_record: {e:?}"))
}

/// Problem d: a box turned about a general axis is written as a record that `from_record` refuses.
///
/// Input: `ops::make_box([0;3], [50, 40, 10])`, `Body::transform` with a pure rotation (angle
/// 2.4184066553326042 rad about [-0.3902044387637371, 0.4445854841071728, 0.26358792709436607]),
/// `to_record`, `from_record`.
/// Expected: the record reads back as a body.
/// Actual: `from_record` fails with kernel status 1007 (`SM_ERR_INVALID_INPUT`), `trusted` or
/// not. Of 200 pseudo-random rotations of this box 27 fail; the same box turned about z in whole
/// degrees (360 of them) or translated (100 of them) never does.
#[test]
fn turned_box_reads_back_from_its_record() {
    fixes();
    let b = ok("make_box", ops::make_box([0.0; 3], [50.0, 40.0, 10.0]));
    ok(
        "transform",
        b.transform(&turn([-0.3902044387637371, 0.4445854841071728, 0.26358792709436607], 2.4184066553326042)),
    );
    if let Err(e) = round_trip(&b, "turned box") {
        panic!("{e}");
    }
}

/// Problem e: the application recipe for a pattern (copy, move the copy by one step, copy again,
/// `union_all`) with three bars turned 60 degrees apart about z gives a body whose record does
/// not read back.
///
/// Input: three copies of the box 20 x 2 x 2 centred on the origin, the k-th one turned `60 k`
/// degrees about z by chaining the step (copy of the previous copy, `transform`, copy), united.
/// Expected: the union (volume 219.215...) writes a record that `from_record` reads.
/// Actual: the volume is right, `to_record` succeeds, `from_record` fails with status 1007
/// (`SM_ERR_INVALID_INPUT`). Turning each copy independently from the original, or uniting one
/// copy after the other with `boolean`, reads back; four bars at 60 degrees and four bars at 30
/// degrees fail the same way when chained.
#[test]
fn union_of_three_chained_rotated_copies_reads_back() {
    fixes();
    let (s, c) = 60.0_f64.to_radians().sin_cos();
    let step = [[c, s, 0.0, 0.0], [-s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]];
    let base = ok("make_box", ops::make_box([-10.0, -1.0, -1.0], [20.0, 2.0, 2.0]));
    let mut bodies = vec![ok("copy base", base.copy())];
    let mut prev = base;
    for _ in 1..3 {
        let next = ok("copy previous", prev.copy());
        ok("transform", next.transform(&step));
        bodies.push(ok("copy next", next.copy()));
        prev = next;
    }
    let united = ok("union_all", ops::boolean::union_all(bodies));
    let v = ok("volume of the union", united.volume(1e-6));
    assert!((v - 219.215).abs() < 0.01, "volume {v}");
    if let Err(e) = round_trip(&united, "union of three bars") {
        panic!("{e}");
    }
}

/// Edge of `rec` along z at the corner `(x, y)`.
fn vertical_edge_at(rec: &Brep, x: f64, y: f64) -> Option<usize> {
    rec.edges.iter().position(|e| {
        let p = rec.vertices.get(e.vertices[0] as usize).map(|v| v.point);
        let q = rec.vertices.get(e.vertices[1] as usize).map(|v| v.point);
        let (Some(p), Some(q)) = (p, q) else { return false };
        (p[0] - q[0]).abs() < 1e-9
            && (p[1] - q[1]).abs() < 1e-9
            && (p[2] - q[2]).abs() > 1.0
            && (p[0] - x).abs() < 1e-6
            && (p[1] - y).abs() < 1e-6
    })
}

/// The plane face of `rec` with a normal along z whose plane is the highest.
fn top_face(rec: &Brep) -> Option<usize> {
    let mut best: Option<(usize, f64)> = None;
    for (i, f) in rec.faces.iter().enumerate() {
        let Surface::Plane { origin, u_axis, v_axis } = &f.surface else { continue };
        let n = unit(cross(*u_axis, *v_axis));
        if n[2].abs() > 0.99 && best.is_none_or(|(_, z)| origin[2] > z) {
            best = Some((i, origin[2]));
        }
    }
    best.map(|b| b.0)
}

/// Problem f: a shell of a filleted plate with a pocket gives a body whose record does not read
/// back.
///
/// Input: a 40 x 20 x 10 plate (outline from a constraint solver, so its corners carry
/// round-off of 1e-14 and 1e-15), a through hole of radius 4 at (15, 8) (a circle extruded 12
/// from z = 0, subtracted), two vertical edges at opposite corners rounded with radius 1.5, the
/// top face opened and a wall of 1.0 (`ops::shell`, thickness positive). Every step goes
/// through the record (`to_record`, `from_record`) as an application keeps its bodies.
/// Expected: the shell's record reads back and the body has a volume.
/// Actual: `ops::shell` succeeds, `to_record` succeeds, `from_record` fails with status 1007
/// (`SM_ERR_INVALID_INPUT`). The same plate with outline coordinates 40.0 and 20.0 exactly reads
/// back (volume 2335.8); the failure is sensitive to round-off at that level.
#[test]
fn shell_of_filleted_pocketed_plate_reads_back() {
    fixes();
    let corner = [
        [2.787812552530594e-16, 7.107075885548428e-15, 0.0],
        [40.0, 1.3354580823781616e-14, 0.0],
        [40.0, 20.000000000000018, 0.0],
        [3.119522659134018e-16, 20.00000000000002, 0.0],
    ];
    let outline: Vec<ops::sweep::Curve> =
        (0..4).map(|i| ops::sweep::Curve::Line { start: corner[i], end: corner[(i + 1) % 4] }).collect();
    let z = [0.0, 0.0, 1.0];
    let plate = ok("extrude plate", ops::sweep::extrude(&outline, z, 10.0, 0.0, Default::default(), true));
    let hole = ops::sweep::Curve::Arc {
        center: [15.0, 8.0, 0.0],
        axis: z,
        reference: [1.0, 0.0, 0.0],
        radius: 4.0,
        start: 0.0,
        end: std::f64::consts::TAU,
    };
    let cutter = ok("extrude hole", ops::sweep::extrude(&[hole], z, 12.0, 0.0, Default::default(), true));
    let plate = round_trip(&plate, "plate").unwrap_or_else(|e| panic!("{e}"));
    let cutter = round_trip(&cutter, "cutter").unwrap_or_else(|e| panic!("{e}"));
    let pocketed = ok("subtract", ops::boolean::boolean(plate, cutter, ops::BooleanOp::Subtract))
        .unwrap_or_else(|| panic!("subtract: the plate came back empty (None)"));
    let (rec, _) = ok("pocketed plate: to_record", pocketed.to_record());
    let e0 = vertical_edge_at(&rec, 0.0, 0.0).unwrap_or_else(|| panic!("no vertical edge at (0, 0) in the pocketed plate"));
    let e1 =
        vertical_edge_at(&rec, 40.0, 20.0).unwrap_or_else(|| panic!("no vertical edge at (40, 20) in the pocketed plate"));
    let body = decode("pocketed plate: from_record", &rec);
    let _blended = ok("blend", ops::blend(&body, ops::Blend::Round, &[e0, e1], &[1.5], &[], ops::Corners::Filled));
    let (rec, _) = ok("filleted plate: to_record", body.to_record());
    let top = top_face(&rec).unwrap_or_else(|| panic!("no top plane face in the filleted plate"));
    let body = decode("filleted plate: from_record", &rec);
    let (_, order) = ok("filleted plate: to_record (order)", body.to_record());
    ok("shell", ops::shell(&body, &order, &[top], 1.0));
    match round_trip(&body, "shelled plate") {
        Ok(b) => assert!(b.volume(1e-6).is_ok(), "no volume"),
        Err(e) => panic!("{e}"),
    }
}

/// Problem g: subtracting a prism whose direction is exactly the normal of one of the body's
/// planes gives a body whose record does not read back.
///
/// Input: the box 50 x 40 x 10 turned by 1.38734577002826 rad about
/// [0.26162006230833446, -0.20491162934367935, -0.3720612130285178] (so its planes are general),
/// round-tripped through its record; a 12 x 8 window of the prism (`ops::sweep::extrude`) with
/// the direction `unit(u_axis x v_axis)` of the top plane read from that record, centred on the
/// plate and reaching 3 mm out of both faces; `ops::boolean::boolean(.., Subtract)`.
/// Expected: the plate with the window reads back from its record.
/// Actual: the boolean succeeds, `to_record` succeeds, `from_record` fails with status 1007
/// (`SM_ERR_INVALID_INPUT`). Axis-aligned plates and prisms work. A direction a hair off the
/// normal is not a fix: with the direction tilted by 1e-12 about 65 of 98 pseudo-random turned
/// plates (where the plate itself reads back) fail, with 1e-3 still 6 of 98.
#[test]
fn window_along_a_plane_normal_reads_back() {
    fixes();
    let axis = [0.26162006230833446, -0.20491162934367935, -0.3720612130285178];
    let m = turn(axis, 1.38734577002826);
    let plate = ok("make_box", ops::make_box([0.0; 3], [50.0, 40.0, 10.0]));
    ok("transform", plate.transform(&m));
    let (rec, _) = ok("turned plate: to_record", plate.to_record());
    let turned_z = [m[2][0], m[2][1], m[2][2]];
    let (origin, n) = rec
        .faces
        .iter()
        .filter_map(|f| match &f.surface {
            Surface::Plane { origin, u_axis, v_axis } => Some((*origin, unit(cross(*u_axis, *v_axis)))),
            _ => None,
        })
        .find(|(_, n)| dot(*n, turned_z) > 0.999)
        .unwrap_or_else(|| panic!("no Surface::Plane face of the turned plate has the turned z as its normal"));
    let (copy, _) = ok("the turned plate reads back", ops::Body::from_record(&rec, false));
    // the centre of the plate: (25, 20, 5) turned
    let centre = add(
        add(scale([m[0][0], m[0][1], m[0][2]], 25.0), scale([m[1][0], m[1][1], m[1][2]], 20.0)),
        scale(turned_z, 5.0),
    );
    let to_top = dot([origin[0] - centre[0], origin[1] - centre[1], origin[2] - centre[2]], n);
    let e1 = unit(cross(n, [1.0, 0.0, 0.0]));
    let e2 = cross(n, e1);
    let base = add(centre, scale(n, -to_top - 3.0));
    let pts: Vec<V> = [(-6.0, -4.0), (6.0, -4.0), (6.0, 4.0), (-6.0, 4.0)]
        .iter()
        .map(|(a, b)| add(base, add(scale(e1, *a), scale(e2, *b))))
        .collect();
    let window: Vec<ops::sweep::Curve> =
        (0..4).map(|i| ops::sweep::Curve::Line { start: pts[i], end: pts[(i + 1) % 4] }).collect();
    let tool = ok("extrude", ops::sweep::extrude(&window, n, 2.0 * to_top + 6.0, 0.0, Default::default(), true));
    let cut = ok("subtract", ops::boolean::boolean(copy, tool, ops::BooleanOp::Subtract))
        .unwrap_or_else(|| panic!("subtract: the plate came back empty (None)"));
    match round_trip(&cut, "plate with window") {
        Ok(b) => assert!(b.volume(1e-6).is_ok(), "no volume"),
        Err(e) => panic!("{e}"),
    }
}

// Problems 8 and 9 (P9-O3): large B-spline shells. The record is assembled here with
// `solidcore-record` types only: a cube sphere of `6 n²` bicubic patches, closed, edges shared.

/// Control nets and corner ids of a closed network, as `sheet_from_bezier_network` takes them.
struct Network {
    patches: Vec<Vec<Vec<[f64; 3]>>>,
    corners: Vec<[u32; 4]>,
}

/// Patches of a cube sphere with `n` quads along each cube edge: `6 n²` patches. Every quad is
/// one patch whose control points lie on the sphere; shared boundary control points are computed
/// from the same lattice point, so neighbouring patches agree exactly.
fn cube_sphere(n: usize, radius: f64) -> Network {
    let steps = 3 * n;
    let point = |lat: [usize; 3]| -> [f64; 3] {
        let c = lat.map(|a| 2.0 * a as f64 / steps as f64 - 1.0);
        let len = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
        c.map(|x| x / len * radius)
    };
    let mut ids: BTreeMap<[usize; 3], u32> = BTreeMap::new();
    let mut patches = Vec::with_capacity(6 * n * n);
    let mut corners = Vec::with_capacity(6 * n * n);
    for axis in 0..3 {
        for side in 0..2 {
            let (p, q) = ((axis + 1) % 3, (axis + 2) % 3);
            // u x v must point out of the cube: swap the tangent axes on the low side
            let (au, av) = if side == 1 { (p, q) } else { (q, p) };
            for i in 0..n {
                for j in 0..n {
                    let lattice = |m: usize, k: usize| -> [usize; 3] {
                        let mut l = [0; 3];
                        l[axis] = side * steps;
                        l[au] = 3 * i + m;
                        l[av] = 3 * j + k;
                        l
                    };
                    let net: Vec<Vec<[f64; 3]>> =
                        (0..4).map(|m| (0..4).map(|k| point(lattice(m, k))).collect()).collect();
                    let mut id = |l: [usize; 3]| {
                        let next = ids.len() as u32;
                        *ids.entry(l).or_insert(next)
                    };
                    corners.push([id(lattice(0, 0)), id(lattice(3, 0)), id(lattice(3, 3)), id(lattice(0, 3))]);
                    patches.push(net);
                }
            }
        }
    }
    Network { patches, corners }
}

/// The network as a solidcore record, edges and vertices numbered in the canonical order of
/// first use.
fn cube_sphere_record(net: &Network) -> Brep {
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let uv = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
    let mut vertex_of: BTreeMap<u32, u32> = BTreeMap::new();
    let mut vertices: Vec<Vertex> = Vec::new();
    let mut edge_of: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    let mut edges: Vec<Edge> = Vec::new();
    let mut faces: Vec<Face> = Vec::new();
    for (f, (net3, c)) in net.patches.iter().zip(&net.corners).enumerate() {
        let side = |s: usize| -> Vec<[f64; 3]> {
            (0..4)
                .map(|k| match s {
                    0 => net3[k][0],
                    1 => net3[3][k],
                    2 => net3[3 - k][3],
                    _ => net3[0][3 - k],
                })
                .collect()
        };
        let mut loop_edges = Vec::new();
        for s in 0..4 {
            let (a, b) = (c[s], c[(s + 1) % 4]);
            let (lo, hi, forward) = (a.min(b), a.max(b), a < b);
            let eid = *edge_of.entry((lo, hi)).or_insert_with(|| {
                let mut pts = side(s);
                if !forward {
                    pts.reverse();
                }
                let mut ids = [0u32; 2];
                for (slot, v) in ids.iter_mut().zip([lo, hi]) {
                    let next = vertices.len() as u32;
                    *slot = *vertex_of.entry(v).or_insert_with(|| {
                        vertices.push(Vertex { point: if v == lo { pts[0] } else { pts[3] }, tolerance: 1e-6 });
                        next
                    });
                }
                edges.push(Edge {
                    curve: Curve::Spline { knots: knots.clone(), points: pts },
                    nurbs: None,
                    interval: [0.0, 1.0],
                    vertices: ids,
                    tolerance: 1e-6,
                    uses: Vec::new(),
                });
                (edges.len() - 1) as u32
            });
            edges[eid as usize].uses.push(EdgeUseRef {
                face: f as u32,
                r#loop: 0,
                position: s as u32,
                upper: !forward,
            });
            let (p0, p1) = (uv[s], uv[(s + 1) % 4]);
            let (start, end) = if forward { (p0, p1) } else { (p1, p0) };
            let pcurve = Curve2::Spline { knots: vec![0.0, 0.0, 1.0, 1.0], points: vec![start, end] };
            loop_edges.push(OrientedEdge { edge: eid, forward, pcurve });
        }
        faces.push(Face {
            surface: Surface::Spline { u_knots: knots.clone(), v_knots: knots.clone(), points: net3.clone() },
            nurbs: None,
            domain: [[0.0, 1.0], [0.0, 1.0]],
            loops: vec![Loop::Edges { edges: loop_edges }],
        });
    }
    let oriented = (0..faces.len() as u32).map(|face| OrientedFace { face, forward: true }).collect();
    Brep {
        version: VERSION,
        tolerance: 1e-6,
        regions: vec![Region { shells: vec![0], void: false }, Region { shells: vec![0], void: false }],
        shells: vec![Shell::Faces { faces: oriented, wires: vec![] }],
        faces,
        edges,
        vertices,
    }
}

/// Problem 8: a closed B-spline shell of 45 000 bicubic patches (about 4.5 million items) is
/// refused by `Brep::validate` with `item budget`, although the body reads back and the same
/// record passes `validate_built`.
///
/// Input: the cube sphere record of 45 414 patches (n = 87), `Body::from_record`, `to_record`.
/// Expected: the record `to_record` writes passes `Brep::validate`, or the item budget scales
/// with the number of faces.
/// Actual: `validate()` returns `item budget` between 39 366 (ok) and 40 344 (refused) bicubic patches (MAX_ITEMS = 4 000 000,
/// about 100 items per patch); `validate_built()` and `Body::from_record` accept the record.
#[test]
fn record_of_a_large_closed_bspline_shell_validates() {
    fixes();
    let record = cube_sphere_record(&cube_sphere(87, 50.0));
    if let Err(e) = record.validate_built() {
        panic!("validate_built must accept the record: {e:?}");
    }
    let (body, _) = ok("from_record", ops::Body::from_record(&record, true));
    let (written, _) = ok("to_record", body.to_record());
    let verdict = written.validate().map_err(|e| e.to_string());
    assert!(verdict.is_ok(), "{} faces: {verdict:?}", written.faces.len());
}

/// Problem 9: the healer of `Body::from_record(.., false)` takes more than linear time in the
/// number of faces.
///
/// Input: cube sphere records of 1 014 and 4 056 bicubic patches (n = 13 and 26).
/// Expected: four times the faces take at most six times as long (linear is four).
/// Actual: 2.7 s and 30.0 s, a factor of 11 (measured: 600 faces 1.4 s, 2 166 faces 8.9 s,
/// 8 214 faces 91.4 s, `ci` profile).
#[test]
fn healer_time_is_close_to_linear_in_the_face_count() {
    fixes();
    let time = |n: usize| {
        let record = cube_sphere_record(&cube_sphere(n, 50.0));
        let start = std::time::Instant::now();
        ok("from_record with the healer", ops::Body::from_record(&record, false));
        start.elapsed().as_secs_f64()
    };
    let (small, large) = (time(13), time(26));
    assert!(large <= 6.0 * small, "1 014 faces took {small:.2} s, 4 056 faces {large:.2} s (x{:.1})", large / small);
}

/// Problem 9b: the trusted conversion (no healer) of a large record is not linear either.
///
/// Input: cube sphere records of 60 000 and 240 000 bicubic patches (n = 100 and 200),
/// `Body::from_record(.., true)`.
/// Expected: four times the faces take at most six times as long.
/// Actual: 2.0 s and 21.5 s, a factor of 10.7 (9.8 in a repeat; also 0.59 s at 16 224, 1.04 s at 30 246,
/// 6.4 s at 120 984 faces). Read in the sources, not profiled: `fill_shells` inserts every face with
/// `post_insert_faceuse`, which checks `shell.faces.contains(&f)` before each push (a linear scan).
#[test]
fn trusted_from_record_time_is_close_to_linear_in_the_face_count() {
    fixes();
    let time = |n: usize| {
        let record = cube_sphere_record(&cube_sphere(n, 50.0));
        let start = std::time::Instant::now();
        ok("from_record without the healer", ops::Body::from_record(&record, true));
        start.elapsed().as_secs_f64()
    };
    let (small, large) = (time(100), time(200));
    assert!(large <= 6.0 * small, "60 000 faces took {small:.2} s, 240 000 faces {large:.2} s (x{:.1})", large / small);
}

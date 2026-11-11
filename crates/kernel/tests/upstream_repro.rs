//! Minimal reproductions of solidcore problems found while building `kernel::hybrid` (plan P3b Z5)
//! and the parametric features (P5).
//!
//! Every test asserts the EXPECTED, correct behaviour and is `#[ignore]`d: it fails on the pinned
//! solidcore (`4f2da85`) and is meant to pass once upstream fixes the problem. Run them with
//! `cargo test -p surfacecraft-kernel --profile ci --test upstream_repro -- --ignored`.
//! The draft issue texts are in `docs/upstream/` (`solidcore-p3.md` and the files it names).
//! Problems 1 to 3 were first seen on `b9c02da`; all of them were re-checked on `4f2da85`.
//! The tests below use the public API of `solidcore-ops` and `solidcore-record` only.
use solidcore_ops as ops;
use solidcore_record::brep::{Brep, Surface};
use surfacecraft_kernel::{solid, step};

/// Problem a: a cone that narrows along +z (radius 5 at the base, 1 at the top, height 10)
/// is exported to STEP and read back as ONE body.
///
/// Input: `ops::make_cone([0;3], 5, 1, 10)` as a record, `step::export_solids`, `step::import`.
/// Expected: one body back (the same solid as before).
/// Actual: the STEP import splits the solid into several bodies, one per face (the negative
/// half-angle conical face does not survive `step_brep::import`).
#[test]
#[ignore = "solidcore 4f2da85: narrowing cone (negative half angle) splits into several bodies after STEP round trip"]
fn cone_narrowing_along_z_survives_step_round_trip() {
    let cone = ops::make_cone([0.0; 3], 5.0, 1.0, 10.0).expect("make_cone");
    let (rec, _) = cone.to_record().expect("to_record");
    let text = step::export_solids(&[("cone", &rec)]).expect("export_solids");
    let bodies = step::import(text.as_bytes()).expect("import");
    assert_eq!(bodies.len(), 1, "a single solid must come back as one body, got {} bodies", bodies.len());
}

/// Problem b: filleting one straight edge of a box gives a cylindrical face.
///
/// Input: box 20 × 10 × 5, `solid::fillet` on edge 0 with radius 2.
/// Expected: exactly one `Surface::Cylinder` face (the round).
/// Actual: the round comes back as a `Surface::Cone` with half angle 0 (SMLib's own form of a
/// cylinder from `ops::blend`), so every consumer must special-case it.
#[test]
#[ignore = "solidcore 4f2da85: ops::blend returns a fillet cylinder as Cone with half angle 0"]
fn fillet_of_box_edge_is_a_cylinder_face() {
    let b = solid::make_box([0.0; 3], [20.0, 10.0, 5.0]).expect("make_box");
    let f = solid::fillet(&b, &[0], 2.0).expect("fillet");
    let kinds: Vec<&str> = f
        .faces
        .iter()
        .map(|face| match &face.surface {
            Surface::Plane { .. } | Surface::PlaneThrough { .. } => "plane",
            Surface::Cylinder { .. } => "cylinder",
            Surface::Cone { .. } => "cone",
            _ => "other",
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
#[ignore = "solidcore 4f2da85: boolean of near-coaxial revolved surfaces (axes 0.01 deg apart) fails with status 1001"]
fn near_coaxial_cylinders_intersect() {
    let a = solid::make_cylinder([0.0; 3], 5.0, 20.0).expect("make_cylinder");
    let c = 0.01_f64.to_radians().cos();
    let s = 0.01_f64.to_radians().sin();
    let turn = [[1.0, 0.0, 0.0, 0.0], [0.0, c, -s, 0.0], [0.0, s, c, 0.0], [0.0, 0.0, 0.0, 1.0]];
    let tilted =
        solid::transform(&solid::make_cylinder([0.0; 3], 5.0, 20.0).expect("make_cylinder"), &turn).expect("transform");
    let (ba, _) = ops::Body::from_record(&a, false).expect("from_record a");
    let (bb, _) = ops::Body::from_record(&tilted, false).expect("from_record b");
    let out = ops::boolean::boolean(ba, bb, ops::BooleanOp::Intersect).expect("intersect must succeed");
    let body = out.expect("the intersection is not empty");
    let v = body.volume(1e-6).expect("volume");
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
#[ignore = "solidcore 4f2da85: a box turned about a general axis does not read back from its own record (status 1007)"]
fn turned_box_reads_back_from_its_record() {
    let b = ops::make_box([0.0; 3], [50.0, 40.0, 10.0]).expect("make_box");
    b.transform(&turn([-0.3902044387637371, 0.4445854841071728, 0.26358792709436607], 2.4184066553326042))
        .expect("transform");
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
#[ignore = "solidcore 4f2da85: union_all of three chained rotated copies does not read back from its record (status 1007)"]
fn union_of_three_chained_rotated_copies_reads_back() {
    let (s, c) = 60.0_f64.to_radians().sin_cos();
    let step = [[c, s, 0.0, 0.0], [-s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]];
    let base = ops::make_box([-10.0, -1.0, -1.0], [20.0, 2.0, 2.0]).expect("make_box");
    let mut bodies = vec![base.copy().expect("copy")];
    let mut prev = base;
    for _ in 1..3 {
        let next = prev.copy().expect("copy");
        next.transform(&step).expect("transform");
        bodies.push(next.copy().expect("copy"));
        prev = next;
    }
    let united = ops::boolean::union_all(bodies).expect("union_all");
    let v = united.volume(1e-6).expect("the union has a volume");
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
#[ignore = "solidcore 4f2da85: shell of a filleted, pocketed plate does not read back from its record (status 1007)"]
fn shell_of_filleted_pocketed_plate_reads_back() {
    let corner = [
        [2.787812552530594e-16, 7.107075885548428e-15, 0.0],
        [40.0, 1.3354580823781616e-14, 0.0],
        [40.0, 20.000000000000018, 0.0],
        [3.119522659134018e-16, 20.00000000000002, 0.0],
    ];
    let outline: Vec<ops::sweep::Curve> =
        (0..4).map(|i| ops::sweep::Curve::Line { start: corner[i], end: corner[(i + 1) % 4] }).collect();
    let z = [0.0, 0.0, 1.0];
    let plate = ops::sweep::extrude(&outline, z, 10.0, 0.0, Default::default(), true).expect("extrude plate");
    let hole = ops::sweep::Curve::Arc {
        center: [15.0, 8.0, 0.0],
        axis: z,
        reference: [1.0, 0.0, 0.0],
        radius: 4.0,
        start: 0.0,
        end: std::f64::consts::TAU,
    };
    let cutter = ops::sweep::extrude(&[hole], z, 12.0, 0.0, Default::default(), true).expect("extrude hole");
    let plate = round_trip(&plate, "plate").expect("plate");
    let cutter = round_trip(&cutter, "cutter").expect("cutter");
    let pocketed = ops::boolean::boolean(plate, cutter, ops::BooleanOp::Subtract)
        .expect("subtract")
        .expect("the plate is not empty");
    let (rec, _) = pocketed.to_record().expect("to_record");
    let e0 = vertical_edge_at(&rec, 0.0, 0.0).expect("edge at (0, 0)");
    let e1 = vertical_edge_at(&rec, 40.0, 20.0).expect("edge at (40, 20)");
    let (body, _) = ops::Body::from_record(&rec, false).expect("from_record pocketed plate");
    ops::blend(&body, ops::Blend::Round, &[e0, e1], &[1.5], &[], ops::Corners::Filled).expect("blend");
    let (rec, _) = body.to_record().expect("to_record");
    let top = top_face(&rec).expect("a top face");
    let (body, _) = ops::Body::from_record(&rec, false).expect("from_record filleted plate");
    let (_, order) = body.to_record().expect("to_record");
    ops::shell(&body, &order, &[top], 1.0).expect("shell");
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
#[ignore = "solidcore 4f2da85: plate turned about a general axis minus a prism along its normal does not read back (status 1007)"]
fn window_along_a_plane_normal_reads_back() {
    let axis = [0.26162006230833446, -0.20491162934367935, -0.3720612130285178];
    let m = turn(axis, 1.38734577002826);
    let plate = ops::make_box([0.0; 3], [50.0, 40.0, 10.0]).expect("make_box");
    plate.transform(&m).expect("transform");
    let (rec, _) = plate.to_record().expect("to_record");
    let turned_z = [m[2][0], m[2][1], m[2][2]];
    let (origin, n) = rec
        .faces
        .iter()
        .filter_map(|f| match &f.surface {
            Surface::Plane { origin, u_axis, v_axis } => Some((*origin, unit(cross(*u_axis, *v_axis)))),
            _ => None,
        })
        .find(|(_, n)| dot(*n, turned_z) > 0.999)
        .expect("the top plane");
    let (copy, _) = ops::Body::from_record(&rec, false).expect("the turned plate reads back");
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
    let tool = ops::sweep::extrude(&window, n, 2.0 * to_top + 6.0, 0.0, Default::default(), true).expect("extrude");
    let cut =
        ops::boolean::boolean(copy, tool, ops::BooleanOp::Subtract).expect("subtract").expect("the plate is not empty");
    match round_trip(&cut, "plate with window") {
        Ok(b) => assert!(b.volume(1e-6).is_ok(), "no volume"),
        Err(e) => panic!("{e}"),
    }
}

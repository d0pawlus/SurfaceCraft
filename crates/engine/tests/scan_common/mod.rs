//! Fixtures shared by the `scan.to_solid` tests: a welded mesh, a session holding a scan, a wavy
//! closed organic part and fins (edges of three triangles).
#![allow(dead_code)]
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, synth};

pub fn weld(mut m: Mesh) -> Mesh {
    m.weld(1e-9);
    m
}

/// A closed organic part: a sphere of radius `r` at `c` with a wavy skin, noise 0.02.
pub fn wavy(c: Point3, r: f64, k: f64, res: usize) -> Mesh {
    let mut m = weld(synth::sphere(Point3::origin(), 1.0, res, res / 2));
    for p in &mut m.positions {
        let d = p.coords;
        p.coords = c.coords + d * r * (1.0 + 0.08 * (k * d.x).sin() * (k * 0.7 * d.y).cos());
    }
    synth::add_noise(&mut m, 0.02, &mut Rng::new(c.x as u64 + 3));
    m
}

pub fn session_with(name: &str, m: Mesh) -> Session {
    let mut s = Session::new();
    s.doc.add_mesh(name, m);
    s
}

/// Fins: a triangle on an existing edge, its tip 0.15 mm off the surface (an edge of three triangles).
pub fn inject_fins(m: &mut Mesh, count: usize, rng: &mut Rng) {
    let normals = m.vertex_normals();
    for _ in 0..count {
        let t = m.triangles[rng.index(m.triangles.len())];
        let (a, b) = (t[0] as usize, t[1] as usize);
        let tip = Point3::from(
            (m.positions[a].coords + m.positions[b].coords) / 2.0 + (normals[a] + normals[b]).normalize() * 0.15,
        );
        m.positions.push(tip);
        let n = (m.positions.len() - 1) as u32;
        m.triangles.push([t[1], t[0], n]);
    }
}

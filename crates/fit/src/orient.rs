//! Orientation control of a patch network (P8-H9): detecting patches whose normal disagrees with
//! the scan and repairing them, so that no flipped patch is ever returned.
//!
//! A patch is *flipped* when its normal, sampled at the centre and at four inner points, points
//! against the reference normal of the scan (the oriented normal of the nearest scan point) at
//! three or more of the five samples, or at the centre. Repair works on the layout, never on the
//! patch alone, because neighbours share their edges:
//!
//! 1. the quad is turned around (vertices 1 and 3 swapped) and the patches are built again; the
//!    turn is kept only when it cured that patch;
//! 2. a patch that is still flipped gets a flat interior (the Coons blend of its four boundary
//!    curves), which keeps its boundary and so the watertightness of the network;
//! 3. whatever is still flipped is reported as failed; callers must not return such a network.
use crate::mls::ScanSurface;
use crate::patch::{Net, PatchSet, build_patches, normal};
use crate::retopo::QuadMesh;
use surfacecraft_geom::{Point3, Vec3};

/// Gives the reference (scan) normal near a point.
pub type NormalAt<'a> = &'a dyn Fn(&Point3) -> Option<Vec3>;

/// Samples of a patch where its normal is compared with the reference.
const SAMPLES: [(f64, f64); 5] = [(0.5, 0.5), (0.25, 0.25), (0.75, 0.25), (0.25, 0.75), (0.75, 0.75)];

/// What the repair did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FlipReport {
    /// Patches that were flipped and no longer are.
    pub repaired: usize,
    /// Patches that are still flipped.
    pub failed: usize,
}

fn point_of(net: &Net, u: f64, v: f64) -> Point3 {
    let (p, _, _) = crate::patch::eval(net, u, v);
    Point3::from(p)
}

/// Is the patch turned against the scan?
fn is_flipped(net: &Net, reference: NormalAt) -> bool {
    let (mut against, mut counted, mut centre_against) = (0usize, 0usize, false);
    for (k, &(u, v)) in SAMPLES.iter().enumerate() {
        let (Some(n), Some(r)) = (normal(net, u, v), reference(&point_of(net, u, v))) else { continue };
        counted += 1;
        if n.dot(&r) < 0.0 {
            against += 1;
            centre_against |= k == 0;
        }
    }
    centre_against || (counted > 0 && against * 2 > counted)
}

/// Indices of the flipped patches of `set`.
#[must_use]
pub fn flipped_patches(set: &PatchSet, reference: NormalAt) -> Vec<usize> {
    set.nets.iter().enumerate().filter(|(_, n)| is_flipped(n, reference)).map(|(i, _)| i).collect()
}

/// The patch with the interior control points of the Coons blend of its boundary curves.
fn flat_interior(net: &Net) -> Net {
    let mut out = *net;
    for i in 1..3 {
        for j in 1..3 {
            let (s, t) = (i as f64 / 3.0, j as f64 / 3.0);
            for c in 0..3 {
                let bilinear = net[0][0][c] * ((1.0 - s) * (1.0 - t))
                    + net[3][0][c] * (s * (1.0 - t))
                    + net[0][3][c] * ((1.0 - s) * t)
                    + net[3][3][c] * (s * t);
                out[i][j][c] =
                    net[i][0][c] * (1.0 - t) + net[i][3][c] * t + net[0][j][c] * (1.0 - s) + net[3][j][c] * s
                        - bilinear;
            }
        }
    }
    out
}

/// Turns the flipped quads of `layout` around where that cures them. Returns the patches built
/// from the repaired layout and the quads that stayed flipped.
fn turn_around(
    layout: &mut QuadMesh,
    scan: &ScanSurface,
    radius: f64,
    reference: NormalAt,
) -> (PatchSet, Vec<usize>, Vec<usize>) {
    let mut set = build_patches(layout, scan, radius);
    let first = flipped_patches(&set, reference);
    if first.is_empty() || set.nets.len() != layout.faces.len() {
        return (set, first.clone(), first);
    }
    let mut turned = layout.clone();
    for &i in &first {
        if let Some(f) = turned.faces.get_mut(i) {
            f.swap(1, 3);
        }
    }
    let trial = build_patches(&turned, scan, radius);
    if trial.nets.len() != layout.faces.len() {
        return (set, first.clone(), first);
    }
    let still: Vec<usize> = flipped_patches(&trial, reference);
    // keep the turn only for the quads it cured
    for &i in first.iter().filter(|i| !still.contains(i)) {
        if let (Some(f), Some(g)) = (layout.faces.get_mut(i), turned.faces.get(i)) {
            *f = *g;
        }
    }
    set = build_patches(layout, scan, radius);
    let left = flipped_patches(&set, reference);
    (set, first, left)
}

/// Detects flipped patches of `layout` and turns the quads around that cause them (the change
/// persists in `layout`). Patches the turn does not cure are counted as failed.
pub fn repair_orientation(layout: &mut QuadMesh, scan: &ScanSurface, radius: f64, reference: NormalAt) -> FlipReport {
    let (_, first, left) = turn_around(layout, scan, radius, reference);
    FlipReport { repaired: first.len().saturating_sub(left.len()), failed: left.len() }
}

/// [`repair_orientation`], then a flat interior for the patches that are still flipped. Returns
/// the patches (shared edges untouched) and the report.
pub fn repair_patches(
    layout: &mut QuadMesh,
    scan: &ScanSurface,
    radius: f64,
    reference: NormalAt,
) -> (PatchSet, FlipReport) {
    let (mut set, first, left) = turn_around(layout, scan, radius, reference);
    let mut failed = 0usize;
    for &i in &left {
        if let Some(net) = set.nets.get(i) {
            let flat = flat_interior(net);
            if is_flipped(&flat, reference) {
                failed += 1;
            } else if let Some(slot) = set.nets.get_mut(i) {
                *slot = flat;
            }
        }
    }
    (set, FlipReport { repaired: first.len().saturating_sub(failed), failed })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 5 × 5 grid of quads over a gently curved cap, counter-clockwise seen from +z.
    fn cap_layout() -> (QuadMesh, ScanSurface) {
        let n = 6usize;
        let height = |x: f64, y: f64| 0.02 * (x * x + y * y);
        let mut layout = QuadMesh::default();
        for j in 0..n {
            for i in 0..n {
                let (x, y) = (i as f64 * 2.0 - 5.0, j as f64 * 2.0 - 5.0);
                layout.vertices.push(Point3::new(x, y, height(x, y)));
                layout.normals.push(Vec3::new(-0.04 * x, -0.04 * y, 1.0).normalize());
            }
        }
        for j in 0..n - 1 {
            for i in 0..n - 1 {
                let a = (j * n + i) as u32;
                layout.faces.push([a, a + 1, a + n as u32 + 1, a + n as u32]);
            }
        }
        let cloud: Vec<Point3> = (0..60)
            .flat_map(|j| (0..60).map(move |i| (i as f64 / 6.0 - 5.0, j as f64 / 6.0 - 5.0)))
            .map(|(x, y)| Point3::new(x, y, height(x, y)))
            .collect();
        (layout, ScanSurface::new(&cloud))
    }

    fn outward(_: &Point3) -> Option<Vec3> {
        Some(Vec3::z())
    }

    #[test]
    fn a_turned_quad_is_detected_and_repaired_once() {
        let (mut layout, scan) = cap_layout();
        assert!(!layout.faces.is_empty());
        let clean = repair_orientation(&mut layout.clone(), &scan, 3.0, &outward);
        assert_eq!(clean, FlipReport::default());
        let k = layout.faces.len() / 2;
        layout.faces[k].swap(1, 3);
        let set = build_patches(&layout, &scan, 3.0);
        assert_eq!(flipped_patches(&set, &outward), vec![k]);
        let report = repair_orientation(&mut layout, &scan, 3.0, &outward);
        assert_eq!(report, FlipReport { repaired: 1, failed: 0 });
        let set = build_patches(&layout, &scan, 3.0);
        assert!(flipped_patches(&set, &outward).is_empty());
    }

    #[test]
    fn the_flat_interior_keeps_the_boundary() {
        let (layout, scan) = cap_layout();
        let set = build_patches(&layout, &scan, 3.0);
        let net = set.nets[0];
        let flat = flat_interior(&net);
        for i in 0..4 {
            for j in 0..4 {
                if i == 0 || j == 0 || i == 3 || j == 3 {
                    assert_eq!(net[i][j], flat[i][j]);
                }
            }
        }
    }
}

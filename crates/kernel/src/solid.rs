//! Solid bodies as records: construction, blends, measurement.
use crate::record::brep::Brep;
use crate::{KernelError, Result, finite_all, op_err};
use serde::{Deserialize, Serialize};
use solidcore_ops as ops;

/// Faces, edges, vertices of a body.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Counts {
    /// Number of faces.
    pub faces: usize,
    /// Number of edges.
    pub edges: usize,
    /// Number of vertices.
    pub vertices: usize,
}

/// A summary of a body for reports and MCP `expect` checks.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BodyInfo {
    /// Topology counts.
    pub counts: Counts,
    /// Enclosed volume, mm³ (`None` for an open sheet or when the kernel cannot measure it).
    pub volume: Option<f64>,
}

/// A record with more faces than this goes to the kernel without its healer: the healer takes
/// 20 s at 4000 scan-fitted patches and a minute at 7000 (measured), and every operation decodes
/// its body again. Such records are patch networks this crate built and checked itself.
const HEAL_FACE_LIMIT: usize = 2000;

pub(crate) fn decode(rec: &Brep) -> Result<ops::Body> {
    let trusted = rec.faces.len() > HEAL_FACE_LIMIT;
    match ops::Body::from_record(rec, trusted) {
        Ok((b, _)) => Ok(b),
        // a large record the kernel does not take as it is still gets the healer's attempt
        Err(_) if trusted => ops::Body::from_record(rec, false).map(|(b, _)| b).map_err(|e| op_err("from_record", e)),
        Err(e) => Err(op_err("from_record", e)),
    }
}

pub(crate) fn encode(body: &ops::Body) -> Result<Brep> {
    body.to_record().map(|(r, _)| r).map_err(|e| op_err("to_record", e))
}

/// An axis-aligned box from `origin` with `size` (all positive).
pub fn make_box(origin: [f64; 3], size: [f64; 3]) -> Result<Brep> {
    if !finite_all(&origin) || !finite_all(&size) || size.iter().any(|s| *s <= 0.0) {
        return Err(KernelError::Argument("box needs finite origin and positive size".into()));
    }
    let b = ops::make_box(origin, size).map_err(|e| op_err("make_box", e))?;
    encode(&b)
}

/// A cylinder along +Z from `origin`.
pub fn make_cylinder(origin: [f64; 3], radius: f64, height: f64) -> Result<Brep> {
    if !finite_all(&origin) || !(radius > 0.0) || !(height > 0.0) || !radius.is_finite() || !height.is_finite() {
        return Err(KernelError::Argument("cylinder needs positive radius and height".into()));
    }
    let b = ops::make_cylinder(origin, radius, height).map_err(|e| op_err("make_cylinder", e))?;
    encode(&b)
}

/// A sphere.
pub fn make_sphere(center: [f64; 3], radius: f64) -> Result<Brep> {
    if !finite_all(&center) || !(radius > 0.0) || !radius.is_finite() {
        return Err(KernelError::Argument("sphere needs a positive radius".into()));
    }
    let b = ops::make_sphere(center, radius).map_err(|e| op_err("make_sphere", e))?;
    encode(&b)
}

/// Rounds the listed edges (record edge indices) with a constant radius.
pub fn fillet(body: &Brep, edges: &[usize], radius: f64) -> Result<Brep> {
    if !(radius > 0.0) || !radius.is_finite() {
        return Err(KernelError::Argument("fillet radius must be positive".into()));
    }
    if edges.is_empty() || edges.iter().any(|&e| e >= body.edges.len()) {
        return Err(KernelError::Argument(format!("edge indices must be in 0..{}", body.edges.len())));
    }
    let b = decode(body)?;
    ops::blend(&b, ops::Blend::Round, edges, &[radius], &[], ops::Corners::Filled).map_err(|e| op_err("blend", e))?;
    encode(&b)
}

/// Bevels the listed edges with a constant distance.
pub fn chamfer(body: &Brep, edges: &[usize], distance: f64) -> Result<Brep> {
    if !(distance > 0.0) || !distance.is_finite() {
        return Err(KernelError::Argument("chamfer distance must be positive".into()));
    }
    if edges.is_empty() || edges.iter().any(|&e| e >= body.edges.len()) {
        return Err(KernelError::Argument(format!("edge indices must be in 0..{}", body.edges.len())));
    }
    let b = decode(body)?;
    ops::blend(&b, ops::Blend::Chamfer, edges, &[distance], &[], ops::Corners::Filled)
        .map_err(|e| op_err("blend", e))?;
    encode(&b)
}

/// Bevels edges with `distance` back along the face `along[i]` (one of edge i's two faces, a
/// record face index) and `second` back along the other face.
pub fn chamfer_sides(body: &Brep, edges: &[usize], along: &[usize], distance: f64, second: f64) -> Result<Brep> {
    if !(distance > 0.0) || !distance.is_finite() || !(second > 0.0) || !second.is_finite() {
        return Err(KernelError::Argument("chamfer distances must be positive".into()));
    }
    if edges.is_empty()
        || along.len() != edges.len()
        || edges.iter().any(|&e| e >= body.edges.len())
        || along.iter().any(|&f| f >= body.faces.len())
    {
        return Err(KernelError::Argument("edge or face indices out of range".into()));
    }
    let b = decode(body)?;
    ops::chamfer_sides(&b, edges, along, distance, ops::ChamferSecond::Distance(second))
        .map_err(|e| op_err("chamfer_sides", e))?;
    encode(&b)
}

/// Thickens a sheet into a solid (`ops::offset::thicken`, design doc §8 M4): `thickness` along
/// the face normals, or half each way when `symmetric`.
pub fn thicken(sheet: &Brep, thickness: f64, symmetric: bool) -> Result<Brep> {
    if !thickness.is_finite() || thickness == 0.0 {
        return Err(KernelError::Argument("thickness must be finite and non-zero".into()));
    }
    let b = decode(sheet)?;
    ops::offset::thicken(&b, thickness, symmetric).map_err(|e| op_err("thicken", e))?;
    encode(&b)
}

/// Topology counts straight from the record.
#[must_use]
pub fn counts(body: &Brep) -> Counts {
    Counts { faces: body.faces.len(), edges: body.edges.len(), vertices: body.vertices.len() }
}

/// Enclosed volume (mm³).
pub fn volume(body: &Brep) -> Result<f64> {
    let b = decode(body)?;
    b.volume(1e-6).map_err(|e| op_err("volume", e))
}

/// Counts and volume.
#[must_use]
pub fn info(body: &Brep) -> BodyInfo {
    BodyInfo { counts: counts(body), volume: volume(body).ok() }
}

/// Applies a row-major rigid 4×4 transform.
pub fn transform(body: &Brep, rows: &[[f64; 4]; 4]) -> Result<Brep> {
    if rows.iter().flatten().any(|v| !v.is_finite()) {
        return Err(KernelError::Argument("transform must be finite".into()));
    }
    let b = decode(body)?;
    b.transform(&crate::to_kernel_matrix(rows)).map_err(|e| op_err("transform", e))?;
    encode(&b)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// M0 smoke test of the kernel boundary (design doc §8): box → fillet → to_record.
    #[test]
    fn smoke_box_fillet_record() {
        let b = make_box([0.0; 3], [20.0, 10.0, 5.0]).unwrap();
        assert_eq!(counts(&b), Counts { faces: 6, edges: 12, vertices: 8 });
        assert!((volume(&b).unwrap() - 1000.0).abs() < 1e-6);
        // a round of radius r on a straight edge of length L removes (1 − π/4)·r²·L
        let [v0, v1] = b.edges[0].vertices;
        let (p0, p1) = (b.vertices[v0 as usize].point, b.vertices[v1 as usize].point);
        let len = ((p0[0] - p1[0]).powi(2) + (p0[1] - p1[1]).powi(2) + (p0[2] - p1[2]).powi(2)).sqrt();
        let r = 2.0;
        let f = fillet(&b, &[0], r).unwrap();
        assert_eq!(counts(&f), Counts { faces: 7, edges: 15, vertices: 10 });
        let v = volume(&f).unwrap();
        let exact = 1000.0 - (1.0 - std::f64::consts::FRAC_PI_4) * r * r * len;
        assert!((v - exact).abs() < 1e-3, "volume {v} vs {exact}");
        // the record round-trips through serde and the kernel
        let json = serde_json::to_string(&f).unwrap();
        let back: Brep = serde_json::from_str(&json).unwrap();
        assert_eq!(back, f);
        assert!((volume(&back).unwrap() - v).abs() < 1e-9);
    }

    #[test]
    fn rejects_bad_arguments() {
        assert!(make_box([0.0; 3], [0.0, 1.0, 1.0]).is_err());
        assert!(make_box([f64::NAN, 0.0, 0.0], [1.0, 1.0, 1.0]).is_err());
        let b = make_box([0.0; 3], [1.0, 1.0, 1.0]).unwrap();
        assert!(fillet(&b, &[99], 0.1).is_err());
        assert!(fillet(&b, &[0], -1.0).is_err());
        assert!(fillet(&b, &[], 0.1).is_err());
    }
}

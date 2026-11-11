//! STEP round trip of a trimmed sheet (P8-H4): a spline face bounded by an L-shaped outline with
//! a square hole is written by `write_sheets` and read back by solidcore's importer; the face
//! keeps both loops and its area.
use surfacecraft_io::step::write_sheets;
use surfacecraft_kernel::nurbs::{fit_grid, trimmed_sheet};
use surfacecraft_kernel::{step, tess};

fn trimmed() -> surfacecraft_kernel::record::brep::Brep {
    let grid: Vec<Vec<[f64; 3]>> = (0..30)
        .map(|i| {
            (0..20)
                .map(|j| {
                    let (x, y) = (60.0 * f64::from(i) / 29.0, 40.0 * f64::from(j) / 19.0);
                    [x, y, 3.0 * (x / 12.0).sin() * (y / 9.0).cos()]
                })
                .collect()
        })
        .collect();
    let s = fit_grid(&grid, 12, 10, 3).unwrap();
    let outer = vec![[0.05, 0.05], [0.95, 0.05], [0.95, 0.45], [0.5, 0.45], [0.5, 0.95], [0.05, 0.95]];
    let hole = vec![[0.2, 0.2], [0.2, 0.35], [0.35, 0.35], [0.35, 0.2]];
    trimmed_sheet(&s, &outer, &[hole]).unwrap()
}

#[test]
fn trimmed_sheet_survives_step() {
    let b = trimmed();
    let text = write_sheets("trimmed", &[&b]).unwrap();
    let back = step::import(text.as_bytes()).unwrap();
    assert_eq!(back.len(), 1);
    assert_eq!(back[0].faces.len(), 1);
    assert_eq!(back[0].faces[0].loops.len(), 2, "outer loop and hole");
    let (a0, a1) =
        (tess::tessellate(&b, Some(0.01)).unwrap().area(), tess::tessellate(&back[0], Some(0.01)).unwrap().area());
    eprintln!("trimmed STEP round trip: area {a0:.3} -> {a1:.3} mm2");
    assert!((a0 - a1).abs() / a0 < 0.005, "area {a0} vs {a1}");
}

//! A file that names this application as its writer is read through the topology build when it
//! is a large sheet; whatever else it holds, the answer is a record or an error, never a panic.
use surfacecraft_kernel::step;

const HEADER: &str = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('x','',(''),(''),'SurfaceCraft','SurfaceCraft','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n";

#[test]
fn files_with_the_mark_and_nothing_valid_behind_it_are_errors_or_empty_not_panics() {
    let mut garbage = HEADER.as_bytes().to_vec();
    garbage.extend((0..4096u32).map(|i| (i.wrapping_mul(2_654_435_761) >> 13) as u8));
    for body in [
        &b""[..],
        &b"#1=CARTESIAN_POINT('',(0.,0.,0.));\nENDSEC;\nEND-ISO-10303-21;\n"[..],
        &b"#1=SHELL_BASED_SURFACE_MODEL('',(#2));\n#2=OPEN_SHELL('',(#2));\nENDSEC;\nEND-ISO-10303-21;\n"[..],
        &b"#999999999999999999999=OPEN_SHELL('',(#1,#1,#1));\n"[..],
        &garbage[HEADER.len()..],
    ] {
        let mut file = HEADER.as_bytes().to_vec();
        file.extend_from_slice(body);
        // either outcome is fine; the test is that the call returns
        let _ = step::import(&file);
    }
}

#[test]
fn a_header_alone_and_a_truncated_header_do_not_panic() {
    for n in [0, 1, 20, HEADER.len() - 1, HEADER.len()] {
        let _ = step::import(HEADER.as_bytes().get(..n).unwrap_or_default());
    }
}

/// Reads the file named by `SURFACECRAFT_PERF_DUMP` and imports it (run under `/usr/bin/time -l`).
#[test]
#[ignore = "measurement"]
fn import_dumped_file() {
    let bytes = std::fs::read(std::env::var("SURFACECRAFT_PERF_DUMP").unwrap()).unwrap();
    let t = std::time::Instant::now();
    let r = step::import(&bytes).unwrap();
    eprintln!("IMPORT {} bytes, {} faces, {:.1} s", bytes.len(), r[0].faces.len(), t.elapsed().as_secs_f64());
}

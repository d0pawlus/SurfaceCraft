//! The desktop build must keep eframe's `accesskit` feature. eframe's defaults include it, but the
//! workspace turns the defaults off, so without this line the native window exposes no
//! accessibility tree and a screen reader sees only the title bar.
const MANIFEST: &str = include_str!("../Cargo.toml");

#[test]
fn the_native_app_enables_the_eframe_accesskit_feature() {
    let eframe = MANIFEST
        .lines()
        .find(|line| line.trim_start().starts_with("eframe"))
        .expect("apps/surfacecraft depends on eframe");
    assert!(eframe.contains("\"accesskit\""), "eframe must enable the accesskit feature: {eframe}");
}

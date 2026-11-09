//! `cargo xtask dist [--tag vX.Y.Z]`: release packages of the desktop app and the CLI for the host target.
//!
//! Everything lands in `dist/<host-target>/`: the two release binaries, and `release/`, which holds the
//! packages a GitHub release carries, named the way the Craft launcher picks them (docs/distribution.md,
//! "Release assets"), with `release/SHA256SUMS.txt` (one line per package: sha256 hex, two spaces, file
//! name). Nothing is uploaded here; `.github/workflows/release.yml` does that. With `--tag`, the build
//! does not start unless the tag is the workspace version.
//!
//! The packages are unsigned. The macOS bundle carries an ad-hoc signature, which seals it but is not a
//! Developer ID signature. Archives are made with the host's own tools: `tar` (Windows: the system
//! `tar.exe`, which writes zip with `-a`; Linux: `tar`), and `ditto`, `ln` and `hdiutil` on macOS.
use std::path::{Path, PathBuf};
use std::process::Command;

/// Files every package carries: the licences, the notices and the attribution table.
const LICENSE_FILES: &[&str] =
    &["LICENSE-MIT", "LICENSE-APACHE", "NOTICE", "THIRD-PARTY-LICENSES.txt", "ATTRIBUTION.md"];

/// Product metadata: `[package.metadata.surfacecraft]` of `apps/surfacecraft/Cargo.toml`, and the version.
struct Meta {
    /// The workspace version, e.g. `0.1.0` or `0.2.0-beta.1`.
    version: String,
    /// The product name, shown to users and used for the `.app` bundle: `SurfaceCraft`.
    product: String,
    /// The reverse-DNS bundle id: `dev.pawlus.surfacecraft`.
    bundle_id: String,
    /// The desktop binary: `surfacecraft`.
    binary: String,
    /// The command-line binary: `surfacecraft-cli`.
    cli_binary: String,
}

/// Reads the metadata through `cargo metadata --no-deps`, which resolves and builds nothing else.
fn read_meta() -> Result<Meta, String> {
    let out = super::cargo()
        .args(["metadata", "--no-deps", "--format-version", "1"])
        .output()
        .map_err(|e| format!("cargo metadata: cannot start: {e}"))?;
    if !out.status.success() {
        return Err(format!("cargo metadata failed: {}", String::from_utf8_lossy(&out.stderr)));
    }
    let json: serde_json::Value =
        serde_json::from_slice(&out.stdout).map_err(|e| format!("cargo metadata output: {e}"))?;
    let app = json["packages"]
        .as_array()
        .and_then(|list| list.iter().find(|p| p["name"] == "surfacecraft"))
        .ok_or("cargo metadata lists no package `surfacecraft`")?;
    let field = |key: &str| {
        app["metadata"]["surfacecraft"][key]
            .as_str()
            .map(str::to_string)
            .ok_or_else(|| format!("apps/surfacecraft/Cargo.toml: [package.metadata.surfacecraft] has no `{key}`"))
    };
    Ok(Meta {
        version: app["version"].as_str().ok_or("cargo metadata: the surfacecraft package has no version")?.to_string(),
        product: field("product-name")?,
        bundle_id: field("bundle-id")?,
        binary: field("binary")?,
        cli_binary: field("cli-binary")?,
    })
}

/// The value after `--tag`, when it is given.
fn tag_arg(args: &[String]) -> Result<Option<String>, String> {
    match args.iter().position(|a| a == "--tag") {
        None => Ok(None),
        Some(i) => match args.get(i + 1) {
            Some(tag) => Ok(Some(tag.clone())),
            None => Err("dist: --tag needs a value, e.g. --tag v0.1.0".into()),
        },
    }
}

/// The release tag must be the workspace version with a `v` in front (`v0.1.0` for `0.1.0`).
fn check_tag(tag: &str, version: &str) -> Result<(), String> {
    if tag == format!("v{version}") {
        Ok(())
    } else {
        Err(format!("dist: tag {tag} does not match the workspace version {version} (expected v{version})"))
    }
}

/// The host target triple as reported by `rustc -vV`.
fn host_triple() -> Result<String, String> {
    let out = Command::new(std::env::var("RUSTC").unwrap_or_else(|_| "rustc".into()))
        .arg("-vV")
        .output()
        .map_err(|e| format!("rustc -vV: cannot start: {e}"))?;
    let text = String::from_utf8_lossy(&out.stdout);
    text.lines()
        .find_map(|l| l.strip_prefix("host: "))
        .map(|h| h.trim().to_string())
        .ok_or_else(|| "rustc -vV printed no host line".to_string())
}

/// The platform part of a package name for `host`, spelled the way the Craft launcher parses it
/// (`crates/core/src/platform.rs`): `windows-x64`, `linux-x86_64`, `macos-aarch64`.
fn platform_token(host: &str) -> Result<&'static str, String> {
    Ok(match host {
        "x86_64-pc-windows-msvc" => "windows-x64",
        "aarch64-pc-windows-msvc" => "windows-arm64",
        "x86_64-unknown-linux-gnu" => "linux-x86_64",
        "aarch64-unknown-linux-gnu" => "linux-aarch64",
        "aarch64-apple-darwin" => "macos-aarch64",
        "x86_64-apple-darwin" => "macos-x86_64",
        other => return Err(format!("dist: no package naming for the host target {other}")),
    })
}

/// Where cargo puts host-target release artifacts (honours CARGO_TARGET_DIR).
fn target_dir() -> PathBuf {
    match std::env::var_os("CARGO_TARGET_DIR") {
        Some(d) => PathBuf::from(d),
        None => crate::root().join("target"),
    }
}

/// The release binaries that ship: the desktop app, then the CLI.
fn shipped(meta: &Meta) -> [&str; 2] {
    [meta.binary.as_str(), meta.cli_binary.as_str()]
}

/// Where one host's packages are staged and written.
struct Layout<'a> {
    meta: &'a Meta,
    /// Cargo's release folder, where the binaries are.
    bins: PathBuf,
    /// Scratch folder for the package trees; removed at the end.
    stage: PathBuf,
    /// `dist/<host-target>/release`: the packages and their checksums.
    out: PathBuf,
}

pub(crate) fn run(args: &[String]) -> Result<(), String> {
    let meta = read_meta()?;
    if let Some(tag) = tag_arg(args)? {
        check_tag(&tag, &meta.version)?;
    }
    let host = host_triple()?;
    let token = platform_token(&host)?;
    let mut c = super::cargo();
    c.args(["build", "--release"]);
    for name in shipped(&meta) {
        c.args(["-p", name]);
    }
    super::run(c, "dist: release build (host target)")?;

    let dist = crate::root().join("dist").join(&host);
    fresh_dir(&dist)?;
    let out = dist.join("release");
    let stage = dist.join("stage");
    fresh_dir(&out)?;
    fresh_dir(&stage)?;
    let bins = target_dir().join("release");
    for name in shipped(&meta) {
        let file = format!("{name}{}", std::env::consts::EXE_SUFFIX);
        copy(&bins.join(&file), &dist.join(&file))?;
    }
    let base = format!("{}-{}-{token}", meta.binary, meta.version);
    let layout = Layout { meta: &meta, bins, stage: stage.clone(), out: out.clone() };
    if host.contains("windows") {
        package_windows(&layout, &base)?;
    } else if host.contains("apple-darwin") {
        package_macos(&layout, &base)?;
    } else if host.contains("linux") {
        package_linux(&layout, &base)?;
    } else {
        return Err(format!("dist: no package layout for the host target {host}"));
    }
    let sums = write_sums(&out)?;
    let _ = std::fs::remove_dir_all(&stage);
    for (digest, name) in &sums {
        eprintln!("==> {digest}  {name}");
    }
    eprintln!("==> release packages in {}", out.display());
    Ok(())
}

/// Windows: `<app>-<version>-windows-x64-portable.zip`, with one folder inside that holds the desktop
/// app and the CLI, the layout the launcher's portable installer takes (`crates/native/src/portable.rs`).
fn package_windows(l: &Layout<'_>, base: &str) -> Result<(), String> {
    let top = format!("{base}-portable");
    let folder = l.stage.join(&top);
    create_dir(&folder)?;
    for name in shipped(l.meta) {
        copy(&l.bins.join(format!("{name}.exe")), &folder.join(format!("{name}.exe")))?;
    }
    copy_licences(&folder)?;
    let mut c = Command::new(tar_program());
    c.arg("-a").arg("-c").arg("-f").arg(l.out.join(format!("{top}.zip"))).arg("-C").arg(&l.stage).arg(&top);
    super::run(c, "dist: zip (Windows portable package)")
}

/// The `tar` to run. On Windows that is the system `tar.exe` (bsdtar, which writes zip), not a GNU tar
/// that Git for Windows may put earlier on PATH.
fn tar_program() -> PathBuf {
    match std::env::var_os("SystemRoot") {
        Some(root) if cfg!(windows) => PathBuf::from(root).join("System32").join("tar.exe"),
        _ => PathBuf::from("tar"),
    }
}

/// Linux: `<app>-<version>-linux-x86_64.tar.gz`, laid out the way the launcher unpacks it
/// (`crates/native/src/install.rs`, `tree::unpack`): `bin/` with both binaries, and
/// `share/applications/<bundle id>.desktop`.
fn package_linux(l: &Layout<'_>, base: &str) -> Result<(), String> {
    let folder = l.stage.join(base);
    let bin = folder.join("bin");
    let apps = folder.join("share").join("applications");
    create_dir(&bin)?;
    create_dir(&apps)?;
    for name in shipped(l.meta) {
        copy(&l.bins.join(name), &bin.join(name))?;
    }
    write_file(&apps.join(format!("{}.desktop", l.meta.bundle_id)), &desktop_entry(l.meta))?;
    copy_licences(&folder)?;
    let mut c = Command::new("tar");
    c.arg("-czf").arg(l.out.join(format!("{base}.tar.gz"))).arg("-C").arg(&l.stage).arg(base);
    super::run(c, "dist: tarball (Linux)")
}

/// macOS: `<app>-<version>-macos-aarch64.dmg`, the disk image the launcher installs macOS apps from, and
/// `<app>-<version>-macos-aarch64.app.zip`, the same folder zipped. Both hold `SurfaceCraft.app` with the
/// CLI in `Contents/MacOS`, and the licences.
fn package_macos(l: &Layout<'_>, base: &str) -> Result<(), String> {
    let product = l.meta.product.as_str();
    let folder = l.stage.join(base);
    let app = folder.join(format!("{product}.app"));
    let macos = app.join("Contents").join("MacOS");
    create_dir(&macos)?;
    for name in shipped(l.meta) {
        copy(&l.bins.join(name), &macos.join(name))?;
    }
    write_file(&app.join("Contents").join("Info.plist"), &info_plist(l.meta))?;
    copy_licences(&folder)?;
    // An ad-hoc signature seals the bundle, so Gatekeeper reports an unidentified developer instead of a
    // damaged bundle. It is not a Developer ID signature and does not replace notarisation.
    let mut sign = Command::new("codesign");
    sign.args(["--force", "--deep", "--sign", "-"]).arg(&app);
    super::run(sign, "dist: ad-hoc signature (not Developer ID)")?;

    let mut zip = Command::new("ditto");
    zip.args(["-c", "-k", "--sequesterRsrc", "--keepParent"]).arg(&folder).arg(l.out.join(format!("{base}.app.zip")));
    super::run(zip, "dist: zip (macOS app bundle)")?;

    let volume = l.stage.join("volume");
    create_dir(&volume)?;
    let mut copy_app = Command::new("ditto");
    copy_app.arg(&app).arg(volume.join(format!("{product}.app")));
    super::run(copy_app, "dist: copy the bundle into the disk image")?;
    copy_licences(&volume)?;
    let mut link = Command::new("ln");
    link.args(["-s", "/Applications"]).arg(volume.join("Applications"));
    super::run(link, "dist: Applications link")?;
    let mut image = Command::new("hdiutil");
    image
        .args(["create", "-volname", product, "-srcfolder"])
        .arg(&volume)
        .args(["-ov", "-format", "UDZO"])
        .arg(l.out.join(format!("{base}.dmg")));
    super::run(image, "dist: disk image (macOS)")
}

/// `Info.plist` of the bundle. Apple wants dotted numbers in `CFBundleShortVersionString` and
/// `CFBundleVersion`, so they take the numeric part of the version. The full version, pre-release suffix
/// included, is under `<product>Version`, the key the Craft launcher reads first.
fn info_plist(meta: &Meta) -> String {
    let numeric = meta.version.split(['-', '+']).next().unwrap_or(meta.version.as_str());
    format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleDevelopmentRegion</key>
  <string>en</string>
  <key>CFBundleDisplayName</key>
  <string>{product}</string>
  <key>CFBundleExecutable</key>
  <string>{binary}</string>
  <key>CFBundleIdentifier</key>
  <string>{bundle}</string>
  <key>CFBundleInfoDictionaryVersion</key>
  <string>6.0</string>
  <key>CFBundleName</key>
  <string>{product}</string>
  <key>CFBundlePackageType</key>
  <string>APPL</string>
  <key>CFBundleShortVersionString</key>
  <string>{numeric}</string>
  <key>CFBundleVersion</key>
  <string>{numeric}</string>
  <key>{product}Version</key>
  <string>{version}</string>
  <key>LSApplicationCategoryType</key>
  <string>public.app-category.graphics-design</string>
  <key>LSMinimumSystemVersion</key>
  <string>11.0</string>
  <key>NSHighResolutionCapable</key>
  <true/>
</dict>
</plist>
"#,
        product = meta.product,
        binary = meta.binary,
        bundle = meta.bundle_id,
        version = meta.version,
    )
}

/// The freedesktop menu entry shipped in the Linux tarball. `Exec` names the binary only, so the entry
/// works once `bin/` is on PATH, or after `Exec` is edited to the unpacked path.
fn desktop_entry(meta: &Meta) -> String {
    format!(
        "[Desktop Entry]\nType=Application\nName={product}\nComment=Scan to CAD: mesh alignment, primitive extraction and surface fitting\nExec={binary}\nTryExec={binary}\nTerminal=false\nCategories=Graphics;3DGraphics;\n",
        product = meta.product,
        binary = meta.binary,
    )
}

/// Copies the licences, notices and attribution table into `dir`.
fn copy_licences(dir: &Path) -> Result<(), String> {
    for name in LICENSE_FILES {
        copy(&crate::root().join(name), &dir.join(name))?;
    }
    Ok(())
}

/// Writes `SHA256SUMS.txt` for the packages in `dir` (every file except the checksum file itself) and
/// returns the lines as `(digest, name)`, sorted by name.
fn write_sums(dir: &Path) -> Result<Vec<(String, String)>, String> {
    let entries = std::fs::read_dir(dir).map_err(|e| format!("read {}: {e}", dir.display()))?;
    let mut names: Vec<String> = entries
        .flatten()
        .filter(|e| e.file_type().is_ok_and(|t| t.is_file()))
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|name| name != "SHA256SUMS.txt")
        .collect();
    names.sort();
    let mut sums = Vec::new();
    for name in names {
        let bytes = std::fs::read(dir.join(&name)).map_err(|e| format!("read {name}: {e}"))?;
        sums.push((hex(&sha256(&bytes)), name));
    }
    let text: String = sums.iter().map(|(digest, name)| format!("{digest}  {name}\n")).collect();
    write_file(&dir.join("SHA256SUMS.txt"), &text)?;
    Ok(sums)
}

/// Removes `path` if it exists, then creates it empty.
fn fresh_dir(path: &Path) -> Result<(), String> {
    let _ = std::fs::remove_dir_all(path);
    create_dir(path)
}

fn create_dir(path: &Path) -> Result<(), String> {
    std::fs::create_dir_all(path).map_err(|e| format!("create {}: {e}", path.display()))
}

fn copy(from: &Path, to: &Path) -> Result<(), String> {
    std::fs::copy(from, to).map(|_| ()).map_err(|e| format!("copy {} -> {}: {e}", from.display(), to.display()))
}

fn write_file(path: &Path, text: &str) -> Result<(), String> {
    std::fs::write(path, text).map_err(|e| format!("write {}: {e}", path.display()))
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

const K: [u32; 64] = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98,
    0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
    0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8,
    0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819,
    0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
    0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
    0xc67178f2,
];

const H0: [u32; 8] = [0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19];

/// SHA-256 (FIPS 180-4) of `data`.
pub(crate) fn sha256(data: &[u8]) -> [u8; 32] {
    let mut h = H0;
    let (full, rem) = data.as_chunks::<64>();
    for block in full {
        compress(&mut h, block);
    }
    // Final one or two blocks: the remainder, 0x80, zero padding, then the bit length (big-endian).
    let mut tail = [0u8; 128];
    tail[..rem.len()].copy_from_slice(rem);
    tail[rem.len()] = 0x80;
    let tail_len = if rem.len() < 56 { 64 } else { 128 };
    let bit_len = (data.len() as u64).wrapping_mul(8);
    tail[tail_len - 8..tail_len].copy_from_slice(&bit_len.to_be_bytes());
    for block in tail[..tail_len].as_chunks::<64>().0 {
        compress(&mut h, block);
    }
    let mut out = [0u8; 32];
    for (dst, word) in out.as_chunks_mut::<4>().0.iter_mut().zip(h.iter()) {
        dst.copy_from_slice(&word.to_be_bytes());
    }
    out
}

fn compress(h: &mut [u32; 8], block: &[u8]) {
    let mut w = [0u32; 64];
    for (word, bytes) in w.iter_mut().zip(block.as_chunks::<4>().0) {
        *word = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    }
    for i in 16..64 {
        let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
        let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16].wrapping_add(s0).wrapping_add(w[i - 7]).wrapping_add(s1);
    }
    let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut hh] = *h;
    for (k, wi) in K.iter().zip(w.iter()) {
        let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
        let ch = (e & f) ^ (!e & g);
        let t1 = hh.wrapping_add(s1).wrapping_add(ch).wrapping_add(*k).wrapping_add(*wi);
        let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
        let maj = (a & b) ^ (a & c) ^ (b & c);
        let t2 = s0.wrapping_add(maj);
        hh = g;
        g = f;
        f = e;
        e = d.wrapping_add(t1);
        d = c;
        c = b;
        b = a;
        a = t1.wrapping_add(t2);
    }
    for (state, v) in h.iter_mut().zip([a, b, c, d, e, f, g, hh]) {
        *state = state.wrapping_add(v);
    }
}

#[cfg(test)]
mod tests {
    use super::{Meta, check_tag, desktop_entry, hex, info_plist, platform_token, sha256, tag_arg};

    fn meta(version: &str) -> Meta {
        Meta {
            version: version.into(),
            product: "SurfaceCraft".into(),
            bundle_id: "dev.pawlus.surfacecraft".into(),
            binary: "surfacecraft".into(),
            cli_binary: "surfacecraft-cli".into(),
        }
    }

    #[test]
    fn sha256_known_vectors() {
        assert_eq!(hex(&sha256(b"")), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
        assert_eq!(hex(&sha256(b"abc")), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
        // 56 bytes: the padding spills into a second block.
        assert_eq!(
            hex(&sha256(b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq")),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"
        );
    }

    #[test]
    fn sha256_multi_block_input() {
        let data = vec![b'a'; 1_000];
        assert_eq!(hex(&sha256(&data)), "41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3");
    }

    #[test]
    fn tag_must_be_the_workspace_version() {
        assert!(check_tag("v0.1.0", "0.1.0").is_ok());
        assert!(check_tag("0.1.0", "0.1.0").is_err());
        assert!(check_tag("v0.1.1", "0.1.0").is_err());
    }

    #[test]
    fn packages_use_the_launcher_platform_tokens() {
        assert_eq!(platform_token("x86_64-pc-windows-msvc").ok(), Some("windows-x64"));
        assert_eq!(platform_token("x86_64-unknown-linux-gnu").ok(), Some("linux-x86_64"));
        assert_eq!(platform_token("aarch64-apple-darwin").ok(), Some("macos-aarch64"));
        assert!(platform_token("x86_64-unknown-linux-musl").is_err());
    }

    #[test]
    fn tag_argument_needs_a_value() {
        let args = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        assert_eq!(tag_arg(&args(&["dist"])).ok(), Some(None));
        assert_eq!(tag_arg(&args(&["dist", "--tag", "v0.1.0"])).ok(), Some(Some("v0.1.0".to_string())));
        assert!(tag_arg(&args(&["dist", "--tag"])).is_err());
    }

    #[test]
    fn info_plist_has_the_bundle_id_and_a_numeric_short_version() {
        let plist = info_plist(&meta("0.2.0-beta.1"));
        assert!(plist.contains("<string>dev.pawlus.surfacecraft</string>"));
        assert!(plist.contains("<key>CFBundleShortVersionString</key>\n  <string>0.2.0</string>"));
        assert!(plist.contains("<key>SurfaceCraftVersion</key>\n  <string>0.2.0-beta.1</string>"));
    }

    #[test]
    fn desktop_entry_names_the_binary() {
        let entry = desktop_entry(&meta("0.1.0"));
        assert!(entry.starts_with("[Desktop Entry]\n"));
        assert!(entry.contains("\nExec=surfacecraft\n"));
    }
}

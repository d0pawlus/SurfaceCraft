# Distribution (stage P7)

This document covers what the repository produces for distribution, how a release is cut, and what
still needs the maintainer's or the user's consent. A release is published only when a tag is pushed
or the release workflow is started by hand. Nothing here touches another repository.

## What the repository produces

| Output | Command | Where |
|---|---|---|
| Release packages (desktop app and CLI), per host target | `cargo xtask dist [--tag vX.Y.Z]` | `dist/<host-target>/release/` (see "Release assets") |
| Checksums of those packages, one line per file (`<sha256 hex>  <file name>`) | `cargo xtask dist` | `dist/<host-target>/release/SHA256SUMS.txt` |
| Release binaries (desktop app and CLI) | `cargo xtask dist` | `dist/<host-target>/surfacecraft[.exe]`, `dist/<host-target>/surfacecraft-cli[.exe]` |
| Browser bundle, optionally shrunk with `wasm-opt -Oz` | `cargo xtask web` | `apps/surfacecraft-web/dist/` |

`dist/` is git-ignored. `cargo xtask dist` is not part of `cargo xtask ci`: a release build of the
desktop app is slow and needs no CI gate. With `--tag`, it stops before the build unless the tag is
`v` followed by the workspace version. The SHA-256 implementation lives in `xtask/src/dist.rs`
(FIPS 180-4, checked against the standard vectors in unit tests); it adds no dependency.

The product metadata that packages read is in `apps/surfacecraft/Cargo.toml`, section
`[package.metadata.surfacecraft]`: product name, bundle id, and the two binary names. The version is
the workspace version (`[workspace.package] version` in the root `Cargo.toml`).

## Release assets (Craft launcher convention)

The packages are named and laid out so that the Craft launcher (`storytold/craft-launcher`, public)
can pick and verify them. The convention below was read from its sources on 2026-11-09:
`crates/core/src/platform.rs` (names, tokens, package kinds), `crates/core/src/release.rs` (valid names,
`SHA256SUMS.txt`, tags, download links), `crates/native/src/install.rs` (install per kind),
`crates/native/src/portable.rs` (portable ZIP layout), `crates/native/src/install/macos_verify.rs` and
`crates/core/src/catalog.rs` (macOS signing team), and the release fixtures in
`crates/core/tests/fixtures/`.

- **Name:** `<app>-<version>-<os>-<arch>[-portable].<ext>`. `<app>` is the app id (`surfacecraft`),
  `<version>` the tag without `v`. Only `A-Z a-z 0-9 . _ + -`, at most 200 characters.
- **Platform tokens, best first:** Windows x64 `windows-x64` (then `windows-x86`); Linux x86_64
  `linux-x86_64`; Apple silicon `macos-universal`, then `macos-aarch64`, `macos-arm64`,
  `macos-x86_64`; Intel macOS `macos-universal`, `macos-x86_64`.
- **Package kinds per OS, in the launcher's order:** Windows MSI, then portable ZIP (`-portable.zip`);
  macOS disk image (`.dmg`) only; Linux AppImage, then tarball (`.tar.gz`).
- **Checksums:** `SHA256SUMS.txt` in `sha256sum` format (`<hex>  <name>`), listing every release asset
  (the checksum contract, see the spike, §3.7 item 22). The launcher checks each download against it,
  unless GitHub's own per-file digest is present. A file with no checksum is not installed.
- **Tags:** `vX.Y.Z`. A tag with a `-pre` suffix is published as a GitHub pre-release, which the
  launcher's stable track skips.

What the workflow produces:

| Platform | Assets | Layout inside |
|---|---|---|
| Windows x64 | `surfacecraft-<v>-windows-x64-portable.zip` | one folder `surfacecraft-<v>-windows-x64-portable/` with `surfacecraft.exe`, `surfacecraft-cli.exe` |
| Linux x86_64 | `surfacecraft-<v>-linux-x86_64.tar.gz` | one folder `surfacecraft-<v>-linux-x86_64/` with `bin/surfacecraft`, `bin/surfacecraft-cli`, `share/applications/dev.pawlus.surfacecraft.desktop` |
| macOS Apple silicon | `surfacecraft-<v>-macos-aarch64.dmg` (the launcher's kind), `surfacecraft-<v>-macos-aarch64.app.zip` (the same folder zipped) | `SurfaceCraft.app` (Info.plist from the metadata, CLI in `Contents/MacOS`) and an `Applications` link in the disk image |
| all | `SHA256SUMS.txt` | one line per asset above |

Every package also carries `LICENSE-MIT`, `LICENSE-APACHE`, `NOTICE`, `THIRD-PARTY-LICENSES.txt` and
`ATTRIBUTION.md`. Packages are unsigned. The macOS bundle has an ad-hoc signature only, which seals it
but is not a Developer ID signature. The Linux `.desktop` entry runs `surfacecraft` from PATH: put
`bin/` on PATH or edit `Exec=`.

Whether the launcher installs these today:

- **Linux tarball:** yes in principle. It unpacks `bin/` and `share/`, checks `bin/surfacecraft` and needs
  only the published SHA-256. Not yet tried against a real release.
- **Windows portable ZIP:** not yet. After unpacking, the launcher compares the exe's ProductVersion
  (VERSIONINFO) with the release version and rolls back on a mismatch (`install.rs`, the `PortableZip`
  branch). Rust builds carry no VERSIONINFO, so the check fails. Open item: embed it in both binaries
  (a build script with a resource crate, reviewed and built on Windows).
- **macOS disk image:** not yet. The launcher installs a DMG only for an app whose catalog entry records
  an Apple Developer team (`mac_team`), and only when the bundle verifies against that team's Developer ID
  signature and notarisation (`catalog.rs`, `macos_verify.rs`). Unsigned or ad-hoc builds are refused.
  Needs the owner's certificate (see "Needs consent").
- **Catalog entry:** the launcher does not list SurfaceCraft. Adding it is the maintainer's decision
  (item 2 below).

Intel Macs get no asset yet. An `x86_64-apple-darwin` build would be a second macOS release build; the
packager already maps its token (`macos-x86_64`).

## Release workflow

`.github/workflows/release.yml` builds the packages above and publishes them. It starts on a pushed tag
`v*`, or by hand (Actions > release > Run workflow, with an existing tag such as `v0.1.0`).

1. **prepare:** checks the tag form (`vX.Y.Z` or `vX.Y.Z-pre`, at most 64 characters).
2. **package (matrix):** `windows-latest` (`x86_64-pc-windows-msvc`), `ubuntu-latest`
   (`x86_64-unknown-linux-gnu`), `macos-latest` (`aarch64-apple-darwin`). Each runs
   `cargo xtask dist --tag <tag>` and uploads `dist/<target>/release/`. The build fails when the tag
   is not the workspace version.
3. **release:** merges the three `SHA256SUMS.txt` files into one, checks every listed file and that no
   name repeats, writes release notes that say the builds are not code-signed, and runs
   `gh release create`. A tag with a `-` suffix is marked as a pre-release. It uses only `GITHUB_TOKEN`
   (`contents: write`), no other secret.

### Cutting a release

1. Set `version` under `[workspace.package]` in `Cargo.toml` to the new version, and merge it to `main`.
2. From the commit to release: `git tag v0.1.0 && git push origin v0.1.0`.
3. Wait for the run in the Actions tab. The release page then lists the packages and `SHA256SUMS.txt`.

A run refuses to overwrite a release. If a run fails after its release was created, delete that
release first, then start the run again.

## Bundle id

`dev.pawlus.surfacecraft`: reverse-DNS of the owner's domain `pawlus.dev` (decided by the owner,
2026-11-08). It identifies the app on macOS (`CFBundleIdentifier`), in the Linux desktop entry and as
the Windows application id.

## Needs the user's or maintainer's consent (not done)

1. **Signing:** a Windows code-signing certificate (SmartScreen), and an Apple Developer ID with
   notarisation (and its team recorded in the launcher catalog). These need accounts and secrets the
   repository must not contain. Until then the packages are unsigned, and the release notes say so. The
   VERSIONINFO item above is needed for the launcher on Windows whether or not the binaries are signed.
2. **Launcher entry:** adding SurfaceCraft as a community app in `storytold/craft-launcher`. This is
   the maintainer's decision; the launcher's agent rules forbid landing a third-party app without
   approval.
3. **Releases:** tags and GitHub releases on `d0pawlus/surfacecraft` are created by the owner, by
   pushing a tag or starting the workflow. A push to `main` does not start the release workflow.

## Web build size

`cargo xtask web` runs `wasm-opt -Oz` on the bundle when `wasm-opt` (binaryen) is on PATH and prints
the size before and after. Without it the bundle is left unoptimised and the build still succeeds.
CI does not install binaryen.

## Web deployment (surfacecraft.pawlus.dev)

The browser app is served by the Vercel project `surfacecraft` (team of the `pawlus.dev` domain)
at <https://surfacecraft.pawlus.dev>. The workflow `.github/workflows/deploy-web.yml` builds the
bundle with `cargo xtask web` after `ci` passes on `main` and uploads it prebuilt
(`vercel build` + `vercel deploy --prebuilt --prod`); it can also be started by hand
(workflow_dispatch). Vercel never builds the repository itself: the project's Ignored Build Step is
`exit 0`, so the Git connection cannot publish the source tree. The workflow needs the repository
secret `VERCEL_TOKEN` (created by the owner in Vercel); without it the bundle is built and the
deploy is skipped with a warning. A manual deploy from a workstation: `cargo xtask web`, then in
`apps/surfacecraft-web/dist` `npx vercel link` and `npx vercel deploy --prod`, after adding a
`.vercelignore` with `.env*` (the CLI writes a token file there). The root `vercel.json` sets `git.deploymentEnabled` to `false` so that Git pushes never trigger a Vercel deployment, while the prebuilt CLI deploy from `apps/surfacecraft-web/dist` is unaffected; the Vercel bot's PR comments are silenced with the dashboard toggles under Settings > Git > "Connected Git Repository", since the `github.silent` key is deprecated.

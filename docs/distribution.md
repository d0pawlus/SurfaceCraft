# Distribution (stage P7)

This document covers what the repository produces for distribution and what still needs the
maintainer's or the user's consent. Nothing here publishes a release, creates a tag, or touches
another repository.

## What the repository produces

| Output | Command | Where |
|---|---|---|
| Release binaries (desktop app and CLI) for the host target | `cargo xtask dist` | `dist/<host-target>/surfacecraft[.exe]`, `dist/<host-target>/surfacecraft-cli[.exe]` |
| Checksums, one line per file (`<sha256 hex>  <file name>`) | `cargo xtask dist` | `dist/<host-target>/SHA256SUMS.txt` |
| Browser bundle, optionally shrunk with `wasm-opt -Oz` | `cargo xtask web` | `apps/surfacecraft-web/dist/` |

`dist/` is git-ignored. `cargo xtask dist` is not part of `cargo xtask ci`: a release build of the
desktop app is slow and needs no CI gate. The SHA-256 implementation lives in `xtask/src/dist.rs`
(FIPS 180-4, checked against the standard vectors in unit tests); it adds no dependency.

The product metadata that installers read is in `apps/surfacecraft/Cargo.toml`, section
`[package.metadata.surfacecraft]`: product name, bundle id, and binary names. The version is the
workspace version (`version.workspace = true`).

## Bundle id

`dev.pawlus.surfacecraft`: reverse-DNS of the owner's domain `pawlus.dev` (decided by the owner,
2026-11-08). It identifies the app on macOS (`CFBundleIdentifier`), in Linux desktop entries and as
the Windows application id.

## Installers per OS (not built yet)

Installers are not built in the repository. Each one would be produced from `dist/<target>/`:

- **Windows:** an MSI or a signed `.exe` installer around `surfacecraft.exe` and `surfacecraft-cli.exe`.
  Code signing requires a certificate that belongs to the publisher. Not started.
- **macOS:** a `.app` bundle with `Info.plist` built from the metadata above (bundle id, product
  name, version), packaged as `.dmg`. Requires an Apple Developer ID signature and notarisation.
  Not started; needs a macOS build host and the Developer ID certificate.
- **Linux:** a tarball with the binaries and a `.desktop` entry, or an AppImage. Not started.

Each release asset must be listed in `SHA256SUMS.txt`, which is the checksum contract the
launcher expects (see the spike, §3.7 item 22).

## Needs the user's or maintainer's consent (not done)

1. **Signing:** Windows certificate, Apple Developer ID and notarisation. These need accounts and
   secrets that the repository must not contain.
2. **Launcher entry:** adding SurfaceCraft as a community app in `storytold/craft-launcher`. This is
   the maintainer's decision; the launcher's agent rules forbid landing a third-party app without
   approval.
3. **Releases:** creating tags and GitHub releases on `d0pawlus/surfacecraft`, and uploading the
   binaries and `SHA256SUMS.txt`. Nothing is uploaded automatically.

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
`.vercelignore` with `.env*` (the CLI writes a token file there).

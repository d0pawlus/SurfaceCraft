#!/bin/sh
# Downloads the open test corpus: seven 3D scans published by Artec Group inc. (www.artec3d.com)
# under a Creative Commons Attribution licence. The files are large (28 to 90 MB packed, 57 to
# 270 MB unpacked), so they stay outside the repository; only this script and the checksums are
# committed. See ATTRIBUTION.md, "External test models".
#
# Usage: xtask/fetch-artec-corpus.sh <target-dir>
# Then:  SURFACECRAFT_ARTEC_CORPUS=<target-dir> cargo test --profile ci -p surfacecraft-engine \
#          --test artec_corpus -- --ignored --nocapture
# Needs curl, unzip and shasum or sha256sum (on Windows: Git Bash).
set -eu

TARGET="${1:?usage: fetch-artec-corpus.sh <target-dir>}"
BASE="https://cdn.artec3d.com/content-hub-3dmodels"

# name | archive on the server | pinned version | SHA-256 of the archive | file inside | saved as
# "file inside" may be `inner.zip!file` for a model packed in an archive inside the archive.
MODELS='car-rim|car-rim-stl.zip|XUsXWZxiPYumkA5zMCWIrOn1DDeqxOGn|77d94e953183c60714ba3c2e505fa78d5a93cd0efd311b765afeeb5cb5406f44|car-rim-stl/car-rim.stl|car-rim.stl
four-cylinder-engine|four-cylinder-engine.zip|1oixxzc4qoUKKagUTKHe2SBXUN6pm8lB|454cc9303aeb496ecfd8570c8d1dbf77f4f8f30334a08f445f334c6f2e0f5b2a|four-cylinder-engine.ply|four-cylinder-engine.ply
industrial-part|industrialpart.zip|4bMdTU8ZM72EWuOiWvzWrd7V0rEo_wmR|e2f244a817cbd95212660aabffc880a41244b19dffdb121344a6b9bf39b2e6c3|IndustrialPart/Industrial Part.obj|industrial-part.obj
water-pump|water-pump.zip|gIkl6w7q27EOyC9xNUVQ_nci5BOc2bZp|03f01eca815b6cfda259433c595427e277c3b64268323e0399a349cc3e3b9e39|Water pump SP2.ply|water-pump.ply
motorcycle-crankcase|motorcycle-crankcase-ply.zip|Fn6soq6bEUvVEFyQ8i9gL1R1b_0Yb0_H|a366a181c88b80373ce3eb5bf17cb82b8f7ebc59f9dfa724ba66a2ef89641876|Motorcycle Crankcase.ply|motorcycle-crankcase.ply
suspension-bracket|metal-part-2_0.zip|tQ0eRCMPFqF87.9mhJ1ihjjsjZ8t.KYq|2f4fa54cd72989253bb3ba0b63b17c543d41a330ade7206c5ffbdbbe3fcd0f58|source/Metal part_2.zip!Metal part_2.obj|suspension-bracket.obj
motorcycle-upper-triple-clamp|motorcycle-upper-triple-clamp-ply.zip|r.cFf6Tp.1PLWFuIaUg2cxWw.1gYDu37|30cdbb1d1a4abf75463fb2169bacc2b13a073f9bb96f02b30c8a5448c2685cab|Motorcycle Upper Triple Clamp.ply|motorcycle-upper-triple-clamp.ply'

sha256_of() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$1" | cut -d' ' -f1
    else
        shasum -a 256 "$1" | cut -d' ' -f1
    fi
}

for tool in curl unzip; do
    command -v "$tool" >/dev/null 2>&1 || { echo "error: $tool is not installed" >&2; exit 1; }
done
command -v sha256sum >/dev/null 2>&1 || command -v shasum >/dev/null 2>&1 || {
    echo "error: neither sha256sum nor shasum is installed" >&2
    exit 1
}

mkdir -p "$TARGET/archives"

echo "$MODELS" | while IFS='|' read -r name archive version sum inner saved; do
    zip="$TARGET/archives/$name.zip"
    if [ ! -f "$zip" ] || [ "$(sha256_of "$zip")" != "$sum" ]; then
        echo "downloading $name"
        curl -fL --retry 3 -o "$zip.part" "$BASE/$archive?versionId=$version"
        mv "$zip.part" "$zip"
    fi
    got="$(sha256_of "$zip")"
    if [ "$got" != "$sum" ]; then
        echo "error: $name: SHA-256 is $got, expected $sum (the file on the server changed)" >&2
        exit 1
    fi
    case "$inner" in
    *'!'*)
        # an archive inside the archive: unpack the inner one first
        nested="$TARGET/archives/$name.inner.zip"
        unzip -p "$zip" "${inner%%!*}" >"$nested"
        unzip -p "$nested" "${inner#*!}" >"$TARGET/$saved.part"
        rm -f "$nested"
        ;;
    *)
        unzip -p "$zip" "$inner" >"$TARGET/$saved.part"
        ;;
    esac
    mv "$TARGET/$saved.part" "$TARGET/$saved"
    # the licence text is the same file in every archive that has one; keep a copy per model
    # beside it. An archive without one gets the statement of the model's page instead.
    licence="$(dirname "${inner%%!*}")/license.txt"
    if unzip -l "$zip" "${licence#./}" >/dev/null 2>&1; then
        unzip -p "$zip" "${licence#./}" >"$TARGET/$name.license.txt"
    else
        printf '%s
'             "The archive of this model holds no licence file. Its page,"             "https://www.artec3d.com/3d-models/$name, states (read 2026-11-15):"             "\"Under Creative Commons license v4, attribution to Artec 3D and a link back to our"             "website is all you need to use this 3D model for free.\""             "https://creativecommons.org/licenses/by/4.0/" >"$TARGET/$name.license.txt"
    fi
    echo "ok $saved"
done

echo "corpus ready in $TARGET (3D models by Artec Group inc., www.artec3d.com, CC BY)"

## Summary

<!-- What changed and why; the stage of docs/superpowers/plans this belongs to. -->

## Checks run before pushing

<!-- `cargo xtask quick <packages>` (fmt, static checks, clippy). GitHub runs the full `cargo xtask ci`. -->

## Do sprawdzenia lokalnie

<!-- Checks GitHub Actions cannot run, for the next batched integration pass (docs/integration.md).
     One line each: what to run, on which data, what result is expected. Examples:
     - local corpus QS: `local_corpus_full_pipeline`, expect 10/10 files and 10/10 regions in threshold
     - perf: `perf_scan_scale` at 10 M, segmentation < 60 s
     - GUI: open a scan, run X from the palette, screenshot of panel Y
     Write "none" when there is nothing. -->

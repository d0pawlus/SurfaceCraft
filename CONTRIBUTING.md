# Contributing

- Read [AGENTS.md](AGENTS.md) (hard rules) and the skill for your area under `skills/`.
- One topic per branch and pull request; run `cargo xtask quick <changed packages>` before every commit;
  the full `cargo xtask ci` runs on every pull request and must be green before merging.
- New behaviour is a command (`crates/engine/src/cmd/`) with input and output schemas and tests.
- Never add code, assets or data from QuickSurface, Geomagic Design X or any closed product.
- Code, comments and commit messages are in English.

# Draft 20: the STEP reader indexes every EDGE_CURVE of the file once per body

Status: not filed (2026-11-25). Read from the code of the pin `c02c5c51c`; not measured, no reproduction run yet. Nothing was
posted.

## What happens

`solidcore_step::topology` builds each body with a `Builder` of its own (`read_with`, `topology.rs` around line 445; one per
solid item of `read_solids_in`). When two distinct vertices of that body coincide within their tolerances, `Builder::meet` asks
`Builder::edge_between` (around line 779) whether an EDGE_CURVE of the file joins them. The first such call of a builder fills
`edge_partners` by walking **every entity of the file** (`self.ex.entities.values()`, around line 782) and collecting the ends
of every EDGE_CURVE. The map lives in the builder, so the next body walks the whole file again.

A file of `B` bodies, each with one pair of coincident vertices, and `N` entities costs `B × N` steps before any geometry is
read. Within the reader's own caps (`MAX_SOLIDS` 100 000 solid items, the parser's 20 000 000 entities) that is up to 2·10¹²
steps: a file the parser accepts in seconds can keep the reader busy for hours. The application's pre-read estimate bounds what
the records would hold, not this time.

## Expected

The EDGE_CURVE ends are indexed once per file (for example in `Index::new` or the `Reader` shared by the builders of one
context) and every builder looks pairs up in that index.

## Reproduction to write first

A file of a few thousand MANIFOLD_SOLID_BREPs, each a box whose corner is written as two VERTEX_POINTs at the same point
(no EDGE_CURVE between them), padded with a large number of unrelated entities; time `topology::read_solids` against the
padding. Expected: linear in the file's size; seen (from the code): the product of the bodies and the entities.

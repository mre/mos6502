# NMOS single-step tests

Fixtures from [SingleStepTests/65x02](https://github.com/SingleStepTests/65x02),
pinned by `REVISION` and distributed under the included MIT license.

`cargo test` runs the first eight cases for each of the 151 documented opcodes.
The harness checks registers, the six physical status flags, memory, and cycle
counts. It does not compare bus-access order or dummy accesses. Status bytes
written to the stack are checked in full.

To run all 1,510,000 cases (about 618 MB of downloaded JSON):

```sh
cargo test --release --test single_step_nmos nmos_single_step_full -- --ignored
```

CI runs the full suite. Locally it is explicitly ignored unless requested.
The full test uses `curl` (required on PATH) to download missing or invalid files
and caches validated downloads for later runs. Normal `cargo test` stays offline.
Wrong-revision fixtures, or JSON files without a revision marker, fail rather
than being reused. Set `NMOS6502_TESTS` to use a fresh fixture directory instead
of `target/nmos6502/v1`.

To regenerate the bundled sample from the pinned data:

```sh
NMOS6502_SAMPLE_OUTPUT=tests/assets/nmos6502/sample.json cargo test --release --test single_step_nmos nmos_single_step_full -- --ignored
```

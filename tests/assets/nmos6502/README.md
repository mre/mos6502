# NMOS single-step tests

Fixtures from [SingleStepTests/65x02](https://github.com/SingleStepTests/65x02),
pinned by `REVISION` and distributed under the included MIT license.

`cargo test` runs the first eight cases for each of the 151 documented opcodes.
The harness checks registers, the six physical status flags, memory, and cycle
counts. It does not compare bus-access order or dummy accesses. Status bytes
written to the stack are checked in full.

To run all 1,510,000 cases (about 618 MB of downloaded JSON):

```sh
cargo xtask fetch-nmos
cargo test --release --test single_step_nmos nmos_single_step_full -- --ignored
```

CI runs the full suite. Locally it is explicitly ignored unless requested.
The xtask downloads missing or invalid files over HTTPS using a Rust HTTP client
and caches validated downloads for later runs. No external downloader is needed.
Both the sample and full tests stay offline and never modify fixtures; missing or
invalid files fail the test.
Wrong-revision fixtures, or JSON files without a revision marker, fail rather
than being reused. Set `NMOS6502_TESTS` to use a fresh fixture directory instead
of `target/nmos6502/v1`.

To regenerate the bundled sample from the pinned data:

```sh
cargo xtask fetch-nmos --sample-output tests/assets/nmos6502/sample.json
```

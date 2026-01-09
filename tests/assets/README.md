# 6502 Functional Test

This is a test suite for 6502/65C02/65C816 processors. It contains a plethora of
tests, which covers all documented opcodes.

Note that the binary was not built from the source code in the repository, but
pre-built binaries were used instead. That is because the source code is not
compatible with the assembler used by the [cc65](https://cc65.github.io/cc65/)
toolchain.

## Building

```bash
make build
```

This will create a `6502_functional_test.bin` file, which the emulator will
load.

## Running

Then, from the root of the repository, run:

```bash
cargo run --release --example functional
```

## Credits

Taken from 
https://github.com/amb5l/6502_65C02_functional_tests
which is a CA65-compatible port of
https://github.com/Klaus2m5/6502_65C02_functional_tests

The original source code was written by Klaus Dormann, and is licensed under
the GPL-3.0 license.

---

# 6502 Timing Test

The `6502_timing_test.bin` file is from the dp111/6502Timing project:
https://github.com/dp111/6502Timing

This test ROM validates instruction cycle timing for the 6502 processor.

The ROM includes undocumented/illegal 6502 instructions which are not
implemented in this emulator. Therefore, it cannot run successfully yet. 

The timing test ROM is kept here for reference and potential future use if
undocumented instruction support is added.

## Credits

Created by dp111, licensed under the 3-Clause BSD License.
```

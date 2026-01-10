# 6502 Instruction Cycle Timing Reference

This document provides cycle timing information for all 6502 variants supported by this emulator.

## Key Timing Differences Between Variants

### NMOS 6502 vs 65C02

The primary timing difference between NMOS 6502 and 65C02 affects decimal mode:

- ADC/SBC in Decimal Mode: 65C02 adds +1 cycle to all ADC and SBC instructions when the D (decimal) flag is set
  - Reason: The 65C02 fixes the N, V, and Z flag behavior in decimal mode (these flags were unreliable/undefined on NMOS)
  - Impact: ALL addressing modes of ADC/SBC take one extra cycle when D=1 on 65C02
  - Example: `ADC #$01` takes 2 cycles in binary mode, but 3 cycles in decimal mode on 65C02
  - Example: `SBC $1234,X` with page crossing and D=1 takes 6 cycles on 65C02 (4 base + 1 page crossing + 1 decimal)

### Ricoh 2A03 Timing

The Ricoh 2A03 has identical cycle timing to the NMOS 6502 because:
- It uses the same 6502 core
- Decimal mode circuitry was removed (SED/CLD are NOPs, D flag does nothing)
- No timing optimizations were made

### Revision A Timing

Revision A has identical cycle timing to NMOS 6502 for all implemented instructions:
- The only difference is missing ROR opcodes (which decode as illegal)
- All other instructions have the same cycle counts as NMOS 6502

## Instruction Timing Table

### Notation

- Base cycle count is shown
- +p: Add 1 cycle if page boundary is crossed
- +b: Add 1 cycle if branch is taken
- +b2: Add 2 cycles if branch is taken to different page
- +d: Add 1 cycle if D=1 (decimal mode) - 65C02 only
- —: Instruction/addressing mode combination not available

### ADC - Add with Carry

| Variant | imm | zp | zp,X | abs | abs,X | abs,Y | (zp,X) | (zp),Y | (zp)† |
|---------|-----|----|----- |-----|-------|-------|--------|--------|-------|
| NMOS 6502 | 2 | 3 | 4 | 4 | 4+p | 4+p | 6 | 5+p | — |
| Revision A | 2 | 3 | 4 | 4 | 4+p | 4+p | 6 | 5+p | — |
| Ricoh 2A03 | 2 | 3 | 4 | 4 | 4+p | 4+p | 6 | 5+p | — |
| 65C02 | 2+d | 3+d | 4+d | 4+d | 4+p+d | 4+p+d | 6+d | 5+p+d | 5+d |

† Zero page indirect addressing mode is 65C02 only

Note: On 65C02, add +1 cycle when D flag is set (decimal mode).

### AND - Logical AND

| Variant | imm | zp | zp,X | abs | abs,X | abs,Y | (zp,X) | (zp),Y | (zp)† |
|---------|-----|----|----- |-----|-------|-------|--------|--------|-------|
| All | 2 | 3 | 4 | 4 | 4+p | 4+p | 6 | 5+p | 5‡ |

‡ 65C02 only

### ASL - Arithmetic Shift Left

| Variant | A | zp | zp,X | abs | abs,X |
|---------|---|----|----- |-----|-------|
| All | 2 | 5 | 6 | 6 | 7 |

### Branch Instructions - BCC, BCS, BEQ, BMI, BNE, BPL, BVC, BVS

| Variant | rel |
|---------|-----|
| All | 2+b+b2 |

Breakdown:
- 2 cycles if branch not taken
- 3 cycles if branch taken, same page
- 4 cycles if branch taken, different page

### BRA - Branch Always (65C02 only)

| Variant | rel |
|---------|-----|
| 65C02 | 3+b2 |

Breakdown:
- 3 cycles same page
- 4 cycles different page

### BIT - Bit Test

| Variant | zp | abs | zp,X† | abs,X† | imm† |
|---------|----|----- |-------|--------|------|
| NMOS 6502 | 3 | 4 | — | — | — |
| Revision A | 3 | 4 | — | — | — |
| Ricoh 2A03 | 3 | 4 | — | — | — |
| 65C02 | 3 | 4 | 4 | 4+p | 2 |

† 65C02 only

### BRK - Break

| Variant | impl |
|---------|------|
| NMOS 6502 | 7 |
| Revision A | 7 |
| Ricoh 2A03 | 7 |
| 65C02 | 7 |

Note: 65C02 clears the D flag during BRK (BRKcld in this emulator).

### CMP - Compare Accumulator

| Variant | imm | zp | zp,X | abs | abs,X | abs,Y | (zp,X) | (zp),Y | (zp)† |
|---------|-----|----|----- |-----|-------|-------|--------|--------|-------|
| All | 2 | 3 | 4 | 4 | 4+p | 4+p | 6 | 5+p | 5‡ |

‡ 65C02 only

### CPX - Compare X Register

| Variant | imm | zp | abs |
|---------|-----|----|----- |
| All | 2 | 3 | 4 |

### CPY - Compare Y Register

| Variant | imm | zp | abs |
|---------|-----|----|----- |
| All | 2 | 3 | 4 |

### DEC - Decrement Memory

| Variant | A† | zp | zp,X | abs | abs,X |
|---------|----|----|------|-----|-------|
| NMOS 6502 | — | 5 | 6 | 6 | 7 |
| Revision A | — | 5 | 6 | 6 | 7 |
| Ricoh 2A03 | — | 5 | 6 | 6 | 7 |
| 65C02 | 2 | 5 | 6 | 6 | 7 |

† Accumulator mode is 65C02 only

### DEX, DEY - Decrement X/Y

| Variant | impl |
|---------|------|
| All | 2 |

### EOR - Exclusive OR

| Variant | imm | zp | zp,X | abs | abs,X | abs,Y | (zp,X) | (zp),Y | (zp)† |
|---------|-----|----|----- |-----|-------|-------|--------|--------|-------|
| All | 2 | 3 | 4 | 4 | 4+p | 4+p | 6 | 5+p | 5‡ |

‡ 65C02 only

### Flag Instructions - CLC, CLD, CLI, CLV, SEC, SED, SEI

| Variant | impl |
|---------|------|
| All | 2 |

### INC - Increment Memory

| Variant | A† | zp | zp,X | abs | abs,X |
|---------|----|----|------|-----|-------|
| NMOS 6502 | — | 5 | 6 | 6 | 7 |
| Revision A | — | 5 | 6 | 6 | 7 |
| Ricoh 2A03 | — | 5 | 6 | 6 | 7 |
| 65C02 | 2 | 5 | 6 | 6 | 7 |

† Accumulator mode is 65C02 only

### INX, INY - Increment X/Y

| Variant | impl |
|---------|------|
| All | 2 |

### JMP - Jump

| Variant | abs | (abs) | (abs,X)† |
|---------|-----|-------|----------|
| NMOS 6502 | 3 | 5* | — |
| Revision A | 3 | 5* | — |
| Ricoh 2A03 | 3 | 5* | — |
| 65C02 | 3 | 6 | 6 |

* NMOS has a bug when indirect address crosses page boundary (uses buggy indirect addressing mode)
† 65C02 only

### JSR - Jump to Subroutine

| Variant | abs |
|---------|-----|
| All | 6 |

### LDA - Load Accumulator

| Variant | imm | zp | zp,X | abs | abs,X | abs,Y | (zp,X) | (zp),Y | (zp)† |
|---------|-----|----|----- |-----|-------|-------|--------|--------|-------|
| All | 2 | 3 | 4 | 4 | 4+p | 4+p | 6 | 5+p | 5‡ |

‡ 65C02 only

### LDX - Load X Register

| Variant | imm | zp | zp,Y | abs | abs,Y |
|---------|-----|----|----- |-----|-------|
| All | 2 | 3 | 4 | 4 | 4+p |

### LDY - Load Y Register

| Variant | imm | zp | zp,X | abs | abs,X |
|---------|-----|----|----- |-----|-------|
| All | 2 | 3 | 4 | 4 | 4+p |

### LSR - Logical Shift Right

| Variant | A | zp | zp,X | abs | abs,X |
|---------|---|----|----- |-----|-------|
| All | 2 | 5 | 6 | 6 | 7 |

### NOP - No Operation

| Variant | impl |
|---------|------|
| All | 2 |

### ORA - Logical OR

| Variant | imm | zp | zp,X | abs | abs,X | abs,Y | (zp,X) | (zp),Y | (zp)† |
|---------|-----|----|----- |-----|-------|-------|--------|--------|-------|
| All | 2 | 3 | 4 | 4 | 4+p | 4+p | 6 | 5+p | 5‡ |

‡ 65C02 only

### PHA, PHP - Push Accumulator/Processor Status

| Variant | impl |
|---------|------|
| All | 3 |

### PHX, PHY - Push X/Y (65C02 only)

| Variant | impl |
|---------|------|
| 65C02 | 3 |

### PLA, PLP - Pull Accumulator/Processor Status

| Variant | impl |
|---------|------|
| All | 4 |

### PLX, PLY - Pull X/Y (65C02 only)

| Variant | impl |
|---------|------|
| 65C02 | 4 |

### ROL - Rotate Left

| Variant | A | zp | zp,X | abs | abs,X |
|---------|---|----|----- |-----|-------|
| All | 2 | 5 | 6 | 6 | 7 |

### ROR - Rotate Right

| Variant | A | zp | zp,X | abs | abs,X |
|---------|---|----|----- |-----|-------|
| NMOS 6502 | 2 | 5 | 6 | 6 | 7 |
| Revision A | — | — | — | — | — |
| Ricoh 2A03 | 2 | 5 | 6 | 6 | 7 |
| 65C02 | 2 | 5 | 6 | 6 | 7 |

Note: Revision A does not have ROR instruction (missing or buggy).

### RTI - Return from Interrupt

| Variant | impl |
|---------|------|
| All | 6 |

### RTS - Return from Subroutine

| Variant | impl |
|---------|------|
| All | 6 |

### SBC - Subtract with Carry

| Variant | imm | zp | zp,X | abs | abs,X | abs,Y | (zp,X) | (zp),Y | (zp)† |
|---------|-----|----|----- |-----|-------|-------|--------|--------|-------|
| NMOS 6502 | 2 | 3 | 4 | 4 | 4+p | 4+p | 6 | 5+p | — |
| Revision A | 2 | 3 | 4 | 4 | 4+p | 4+p | 6 | 5+p | — |
| Ricoh 2A03 | 2 | 3 | 4 | 4 | 4+p | 4+p | 6 | 5+p | — |
| 65C02 | 2+d | 3+d | 4+d | 4+d | 4+p+d | 4+p+d | 6+d | 5+p+d | 5+d |

† Zero page indirect addressing mode is 65C02 only
Note: On 65C02, add +1 cycle when D flag is set (decimal mode).

### STA - Store Accumulator

| Variant | zp | zp,X | abs | abs,X | abs,Y | (zp,X) | (zp),Y | (zp)† |
|---------|----|----- |-----|-------|-------|--------|--------|-------|
| All | 3 | 4 | 4 | 5 | 5 | 6 | 6 | 5‡ |

‡ 65C02 only

Note: Store instructions do NOT have page crossing penalties (always take the stated cycles).

### STX - Store X Register

| Variant | zp | zp,Y | abs |
|---------|----|----- |-----|
| All | 3 | 4 | 4 |

### STY - Store Y Register

| Variant | zp | zp,X | abs |
|---------|----|----- |-----|
| All | 3 | 4 | 4 |

### STZ - Store Zero (65C02 only)

| Variant | zp | zp,X | abs | abs,X |
|---------|----|----- |-----|-------|
| 65C02 | 3 | 4 | 4 | 5 |

### TAX, TAY, TSX, TXA, TXS, TYA - Register Transfers

| Variant | impl |
|---------|------|
| All | 2 |

### TRB, TSB - Test and Reset/Set Bits (65C02 only)

| Variant | zp | abs |
|---------|----|----- |
| 65C02 | 5 | 6 |

### STP - Stop (65C02 only)

| Variant | impl |
|---------|------|
| 65C02 | 3 |

Note: STP halts the processor until reset. Cycle count represents the cycles before halting.

### WAI - Wait for Interrupt (65C02 only)

| Variant | impl |
|---------|------|
| 65C02 | 3 |

Note: WAI enters low-power mode until interrupt. Cycle count represents the cycles before waiting.

## Timing Rules and Special Cases

### Page Boundary Crossing (+p)

A page boundary is crossed when adding an index register (X or Y) to a base address causes the high byte to change.

Example:
- Base address: `$10FF`
- Add X register: `$02`
- Result: `$1101` (page boundary crossed: `$10` → `$11`)

Rules:
- Load instructions (LDA, LDX, LDY, EOR, AND, ORA, ADC, SBC, CMP): +1 cycle if page crossed
- Store instructions (STA, STX, STY, STZ): NO extra cycle (always take base cycles)
- Read-Modify-Write (INC, DEC, ASL, LSR, ROL, ROR): Always take the stated cycles (includes the extra work)

Addressing modes affected:
- Absolute indexed: `abs,X` and `abs,Y`
- Indirect indexed: `(zp),Y`

### Branch Instructions (+b, +b2)

Branch instruction timing depends on whether the branch is taken and whether it crosses a page boundary:

1. Branch not taken: 2 cycles
2. Branch taken, same page: 3 cycles (+1)
3. Branch taken, different page: 4 cycles (+2)

Page crossing for branches: Occurs when the branch target is on a different memory page than the instruction following the branch instruction.

### Decimal Mode Timing (65C02 only)

On the 65C02, when the D (decimal) flag is set:
- ADC: +1 cycle for ALL addressing modes
- SBC: +1 cycle for ALL addressing modes

Example calculations:
- `ADC #$10` with D=0: 2 cycles
- `ADC #$10` with D=1: 3 cycles (2 + 1 decimal)
- `ADC $1234,X` with D=0, page crossed: 5 cycles (4 + 1 page)
- `ADC $1234,X` with D=1, page crossed: 6 cycles (4 + 1 page + 1 decimal)

### Store Instructions - No Page Crossing Penalty

Unlike load instructions, store instructions (STA, STX, STY, STZ) do not have page crossing penalties. They always perform the write cycle regardless of page crossing.

Example:
- `LDA $1000,X` with page crossing: 5 cycles (4+1)
- `STA $1000,X` with page crossing: 5 cycles (always 5, no penalty)

### Indirect JMP Bug (NMOS 6502, Revision A, Ricoh 2A03)

On NMOS variants, `JMP ($xxFF)` has a bug: when the indirect address is at the end of a page (low byte = $FF), the high byte is fetched from the beginning of the same page instead of the next page.

Example:
```
JMP ($10FF)
```
- Correct behavior (65C02): Fetch low byte from `$10FF`, high byte from `$1100`
- Buggy behavior (NMOS): Fetch low byte from `$10FF`, high byte from `$1000`

The cycle count is still 5 for NMOS (using BuggyIndirect addressing mode in this emulator), but 65C02 uses normal Indirect mode.

Update: The 65C02 takes 6 cycles for `JMP (abs)`, not 5. This is one cycle longer than NMOS.

## Discrepancies and Recommendations

### ADC/SBC Decimal Mode Timing

Sources generally agree: The 65C02 adds +1 cycle to ADC and SBC in decimal mode.

Discrepancy found: Some older references do not mention this timing difference.

Recommendation: Use +1 cycle for ADC/SBC when D=1 on 65C02. This is confirmed by:
- [Oxyron 65C02 reference](https://www.oxyron.de/html/opcodesc02.html)
- [Wilson Minesco's NMOS-CMOS differences](https://wilsonminesco.com/NMOS-CMOSdif/)
- Multiple forum discussions on 6502.org

Reason: The 65C02 fixes the N, V, and Z flags in decimal mode (which were unreliable on NMOS), requiring extra logic and one additional cycle.

### JMP (abs) Timing - NMOS vs 65C02

Discrepancy: Different sources report different cycle counts for `JMP (abs)` on 65C02.

NMOS 6502: 5 cycles (with page-crossing bug)
65C02: Most sources say 6 cycles, some say 5

Recommendation: Use 6 cycles for `JMP (abs)` on 65C02, 5 cycles for NMOS variants. The extra cycle is needed to fix the indirect JMP bug.

Sources:
- [WDC W65C02S datasheet](https://www.westerndesigncenter.com/wdc/documentation/w65c02s.pdf) (official specification)
- [NESdev 6502 cycle times](https://www.nesdev.org/wiki/6502_cycle_times)

### ROR on Revision A

Discrepancy: Some sources say ROR is "buggy" on early 6502s, others say it's "missing."

Recommendation: Treat ROR as not implemented on Revision A (decode returns None). According to historical documentation, the ROR instruction was intentionally unimplemented in very early production runs (pre-June 1976), not buggy.

Sources:
- [Masswerk 6502 reference](https://www.masswerk.at/6502/6502_instruction_set.html#ror-bug)
- Various KIM-1 documentation

### Read-Modify-Write Cycle Counts

Note: Some documentation discusses internal cycle-level differences between NMOS and 65C02 for RMW instructions (NMOS does "one read, two writes" while 65C02 does "two reads, one write"), but the total cycle counts remain the same.

Recommendation: Use the same cycle counts for RMW instructions on both NMOS and 65C02. The internal behavior differs but the total cycles do not.

### Ricoh 2A03 vs NMOS 6502

Sources agree: The Ricoh 2A03 has identical cycle timing to NMOS 6502.

Recommendation: Use NMOS 6502 cycle counts for Ricoh 2A03. The only difference is that decimal mode is disabled (SED/CLD are NOPs, ADC/SBC always use binary arithmetic).

## Sources

This document was compiled from multiple authoritative sources:

### Primary References

1. [NESdev Wiki - 6502 Cycle Times](https://www.nesdev.org/wiki/6502_cycle_times) - Cycle timing tables
2. [NESdev Wiki - Visual6502 All 256 Opcodes](https://www.nesdev.org/wiki/Visual6502wiki/6502_all_256_Opcodes) - Opcode table with transistor-level simulation data
3. [Masswerk 6502 Instruction Set](https://www.masswerk.at/6502/6502_instruction_set.html) - Instruction reference with cycle counts
4. [WDC W65C02S Datasheet](https://www.westerndesigncenter.com/wdc/documentation/w65c02s.pdf) - Official 65C02 specification (PDF)
5. [6502.org Tutorials](http://www.6502.org/tutorials/6502opcodes.html) - Historical 6502 documentation
6. [Oxyron 65C02 Opcodes](https://www.oxyron.de/html/opcodesc02.html) - 65C02 timing reference

### Variant-Specific References

7. [Wilson Minesco - NMOS/CMOS Differences](https://wilsonminesco.com/NMOS-CMOSdif/) - Comparison of timing differences
8. [NESdev Wiki - 2A03](https://www.nesdev.org/wiki/2A03) - Ricoh 2A03 documentation
9. [NESdev Wiki - CPU Variants](https://www.nesdev.org/wiki/CPU_variants) - Comparison of different 6502 variants

### Additional Resources

10. [6502.org Forums - 65C02 Instruction Timings](http://forum.6502.org/viewtopic.php?f=1&t=6618) - Community discussions on timing
11. [Matt Godbolt - jsbeeb CPU Timings](https://xania.org/201405/jsbeeb-getting-the-timings-right-cpu) - Cycle-accurate emulation discussion
12. [C64 OS - 6502/6510 Instruction Set](https://c64os.com/post/6502instructions) - Commodore 64 perspective

### Test Suites

13. [GitHub - dp111/6502Timing](https://github.com/dp111/6502Timing) - Instruction timing test program

## Implementation Notes for Cycle-Accurate Emulation

When implementing cycle-accurate emulation:

1. Track the D (decimal) flag and add +1 cycle for ADC/SBC on 65C02 when D=1
2. Detect page boundary crossings for indexed and indirect indexed addressing modes
3. Branch timing requires checking if branch is taken and if it crosses a page
4. Store instructions never have page crossing penalties (unlike loads)
5. JMP (abs) is 6 cycles on 65C02, 5 on NMOS (with bug in NMOS)
6. ROR does not exist on Revision A

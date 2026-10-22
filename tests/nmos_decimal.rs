use mos6502::Variant;
use mos6502::cpu::CPU;
use mos6502::instruction::{Cmos6502, Nmos6502, Ricoh2a03};
use mos6502::memory::{Bus, Memory};
use mos6502::registers::Status;

// Each vector is (A, operand, carry in, result, NVZC).
fn check(variant: impl Variant, opcode: u8, cases: &[(u8, u8, bool, u8, u8)]) {
    let mut cpu = CPU::new(Memory::new(), variant);
    for &(a, operand, carry, result, flags) in cases {
        // Both initial N/V/Z patterns ensure the instruction sets and clears flags.
        for initial_flags in [0x00, 0xc2] {
            cpu.registers.program_counter = 0x0200;
            cpu.registers.accumulator = a;
            cpu.registers.status = Status::from_bits_retain(0x2c | initial_flags | u8::from(carry));
            cpu.memory.set_bytes(0x0200, &[opcode, operand]);
            let decoded = cpu.fetch_next_and_decode().unwrap();
            cpu.execute_instruction(decoded);
            assert_eq!(
                (
                    cpu.registers.accumulator,
                    cpu.registers.status.bits() & 0xcf
                ),
                (result, flags | 0x0c),
                "opcode {opcode:02x}, A={a:02x}, operand={operand:02x}, C={carry}"
            );
        }
    }
}

#[test]
fn adc_samples_flags_before_high_digit_correction() {
    // Visual6502's difficult cases, including invalid digits and binary zero:
    // https://www.nesdev.org/wiki/Visual6502wiki/6502DecimalMode
    check(
        Nmos6502,
        0x69,
        &[
            (0x00, 0x00, false, 0x00, 0x02),
            (0x79, 0x00, true, 0x80, 0xc0),
            (0x24, 0x56, false, 0x80, 0xc0),
            (0x93, 0x82, false, 0x75, 0x41),
            (0x89, 0x76, false, 0x65, 0x01),
            (0x89, 0x76, true, 0x66, 0x03),
            (0x80, 0xf0, false, 0xd0, 0x41),
            (0x80, 0xfa, false, 0xe0, 0x81),
            (0x2f, 0x4f, false, 0x74, 0x00),
            (0x6f, 0x00, true, 0x76, 0x00),
            (0x99, 0x01, false, 0x00, 0x81),
            // SingleStepTests 2f6980a2d95757486c7bee24355c360e40e2a224:
            // immediate cases "69 8f b3" and "69 38 17".
            (0xe3, 0x8f, true, 0xd9, 0x41),
            (0x61, 0x38, true, 0x00, 0xc1),
        ],
    );
}

#[test]
fn sbc_samples_flags_from_binary_difference() {
    // Bruce Clark, Appendix A: SBC flags are binary even for invalid BCD.
    // https://6502.org/tutorials/decimal_mode.html
    check(
        Nmos6502,
        0xe9,
        &[
            (0x00, 0x00, false, 0x99, 0x80),
            (0x00, 0x00, true, 0x00, 0x03),
            (0x00, 0x01, true, 0x99, 0x80),
            (0x0a, 0x00, true, 0x0a, 0x01),
            (0x0b, 0x00, false, 0x0a, 0x01),
            (0x9a, 0x00, true, 0x9a, 0x81),
            (0x9b, 0x00, false, 0x9a, 0x81),
            (0x90, 0x0f, true, 0x8b, 0x81),
            (0x00, 0x80, true, 0x20, 0xc0),
            // Same fixture revision: "e9 c4 08" and "e9 eb e7".
            (0x9c, 0xc4, true, 0x78, 0x80),
            (0x4c, 0xeb, false, 0x00, 0x00),
        ],
    );
}

#[test]
fn cmos_and_ricoh_keep_their_flag_behavior() {
    check(Cmos6502, 0x69, &[(0x99, 0x01, false, 0x00, 0x03)]);
    check(Cmos6502, 0xe9, &[(0x00, 0x80, true, 0x20, 0x40)]);
    check(Ricoh2a03, 0x69, &[(0x99, 0x01, false, 0x9a, 0x80)]);
}

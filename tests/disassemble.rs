// Tests for the non-mutating single-instruction disassembler,
// `mos6502::instruction::disassemble_one`.
//
// The disassembler decodes the full HuC6280 superset, resolves branch targets
// to absolute addresses, and renders static operands exactly as encoded (no
// register offsets, no pointer dereferencing, no zero-page relocation). It is
// driven by a side-effect-free `read` closure that is only allowed to touch the
// instruction's own bytes.

use mos6502::Variant;
use mos6502::instruction::{AddressingMode, Huc6280, Instruction, OpInput, disassemble_one};

/// Build a `read` closure over `program` placed at `base`. Reading outside the
/// program returns `0`.
fn reader_at(base: u16, program: &'static [u8]) -> impl Fn(u16) -> u8 {
    move |addr: u16| {
        let offset = addr.wrapping_sub(base);
        program.get(offset as usize).copied().unwrap_or(0)
    }
}

#[test]
fn zero_page_keeps_literal_address() {
    // LDA $69 (opcode 0xA5). The operand is rendered as the literal encoded
    // address ($0069), not relocated through the HuC6280 zero-page base.
    let d = disassemble_one(0x1000, reader_at(0x1000, &[0xA5, 0x69]));

    assert_eq!(d.length, 2);
    assert_eq!(d.opcode, 0xA5);
    assert_eq!(d.instruction, Instruction::LDA);
    assert_eq!(d.mode, AddressingMode::ZeroPage);
    assert!(matches!(
        d.operand,
        OpInput::UseAddress {
            address: 0x0069,
            page_crossed: false
        }
    ));
    assert_eq!(d.text, "LDA $0069");
}

#[test]
fn immediate_store_to_vdc_port() {
    // ST1 #$00 (opcode 0x13) - a HuC6280 immediate VDC write.
    let d = disassemble_one(0x2000, reader_at(0x2000, &[0x13, 0x00]));

    assert_eq!(d.length, 2);
    assert_eq!(d.instruction, Instruction::ST1);
    assert_eq!(d.mode, AddressingMode::Immediate);
    assert!(matches!(d.operand, OpInput::UseImmediate(0x00)));
    assert_eq!(d.text, "ST1 #$00");
}

#[test]
fn relative_branch_forward_target_is_absolute() {
    // BNE +5 (opcode 0xD0) at $1000: target = $1000 + 2 + 5 = $1007.
    let d = disassemble_one(0x1000, reader_at(0x1000, &[0xD0, 0x05]));

    assert_eq!(d.length, 2);
    assert_eq!(d.instruction, Instruction::BNE);
    assert_eq!(d.mode, AddressingMode::Relative);
    assert!(matches!(d.operand, OpInput::UseRelative(0x1007)));
    assert_eq!(d.text, "BNE $1007");
}

#[test]
fn relative_branch_backward_target_is_absolute() {
    // BNE -5 (offset 0xFB) at $1000: target = $1000 + 2 - 5 = $0FFD.
    let d = disassemble_one(0x1000, reader_at(0x1000, &[0xD0, 0xFB]));

    assert!(matches!(d.operand, OpInput::UseRelative(0x0FFD)));
    assert_eq!(d.text, "BNE $0FFD");
}

#[test]
fn bsr_renders_as_relative_branch() {
    // BSR +5 (HuC6280 opcode 0x44, relative branch-to-subroutine) at $1000:
    // target = $1000 + 2 + 5 = $1007, rendered like any other relative branch.
    let d = disassemble_one(0x1000, reader_at(0x1000, &[0x44, 0x05]));

    assert_eq!(d.length, 2);
    assert_eq!(d.instruction, Instruction::BSR);
    assert_eq!(d.mode, AddressingMode::Relative);
    assert!(matches!(d.operand, OpInput::UseRelative(0x1007)));
    assert_eq!(d.text, "BSR $1007");
}

#[test]
fn bit_branch_renders_bit_index_and_absolute_target() {
    // BBR3 $12, +12 (opcode 0x3F) at $D940: target = $D940 + 3 + 12 = $D94F.
    let d = disassemble_one(0xD940, reader_at(0xD940, &[0x3F, 0x12, 0x0C]));

    assert_eq!(d.length, 3);
    assert_eq!(d.instruction, Instruction::BBR(3));
    assert_eq!(d.mode, AddressingMode::ZeroPageRelative);
    assert!(matches!(
        d.operand,
        OpInput::UseBitBranch {
            zp_address: 0x12,
            relative: 0xD94F
        }
    ));
    assert_eq!(d.text, "BBR3 $12,$D94F");
}

#[test]
fn indexed_operand_is_base_address_only() {
    // LDA $2000,X (opcode 0xBD). The disassembler reports the base address with
    // no X applied; the caller adds the register offset itself.
    let d = disassemble_one(0x1000, reader_at(0x1000, &[0xBD, 0x00, 0x20]));

    assert_eq!(d.length, 3);
    assert_eq!(d.mode, AddressingMode::AbsoluteX);
    assert!(matches!(
        d.operand,
        OpInput::UseAddress {
            address: 0x2000,
            page_crossed: false
        }
    ));
    assert_eq!(d.text, "LDA $2000");
}

#[test]
fn indirect_operand_is_not_dereferenced() {
    // LDA ($10),Y (opcode 0xB1). The operand is the zero-page pointer byte as
    // encoded; the pointer is never followed.
    let d = disassemble_one(0x1000, reader_at(0x1000, &[0xB1, 0x10]));

    assert_eq!(d.length, 2);
    assert_eq!(d.mode, AddressingMode::IndirectIndexedY);
    assert!(matches!(
        d.operand,
        OpInput::UseAddress {
            address: 0x0010,
            page_crossed: false
        }
    ));
    assert_eq!(d.text, "LDA $0010");
}

#[test]
fn block_transfer_is_seven_bytes() {
    // TII $1000, $2000, $0004 (opcode 0x73): the 7-byte block-transfer form.
    let d = disassemble_one(
        0x4000,
        reader_at(0x4000, &[0x73, 0x00, 0x10, 0x00, 0x20, 0x04, 0x00]),
    );

    assert_eq!(d.length, 7);
    assert_eq!(d.instruction, Instruction::TII);
    assert_eq!(d.mode, AddressingMode::BlockTransfer);
    assert!(matches!(
        d.operand,
        OpInput::UseBlockTransfer {
            source: 0x1000,
            dest: 0x2000,
            length: 0x0004
        }
    ));
    assert_eq!(d.text, "TII $1000,$2000,$0004");
}

#[test]
fn tst_immediate_zero_page() {
    // TST #$AA, $10 (opcode 0x83): immediate mask plus a memory operand.
    let d = disassemble_one(0x1000, reader_at(0x1000, &[0x83, 0xAA, 0x10]));

    assert_eq!(d.length, 3);
    assert_eq!(d.instruction, Instruction::TST);
    assert_eq!(d.mode, AddressingMode::ImmediateZeroPage);
    assert!(matches!(
        d.operand,
        OpInput::UseImmediateAddress {
            value: 0xAA,
            address: 0x0010
        }
    ));
    assert_eq!(d.text, "TST #$AA,$0010");
}

#[test]
fn implied_instruction_has_no_operand_text() {
    // CLA (opcode 0x62): a HuC6280 implied clear with no operand.
    let d = disassemble_one(0x1000, reader_at(0x1000, &[0x62]));

    assert_eq!(d.length, 1);
    assert_eq!(d.instruction, Instruction::CLA);
    assert!(matches!(d.operand, OpInput::UseImplied));
    assert_eq!(d.text, "CLA");
}

#[test]
fn length_always_matches_extra_bytes_and_never_panics() {
    // Every opcode must decode without panicking, and `length` must equal
    // `1 + mode.extra_bytes()`. The HuC6280 table is exhaustive, so the `.db`
    // fallback never triggers here - but the invariants still hold for all 256.
    for opcode in 0u16..=0xFF {
        let opcode = opcode as u8;
        // Fill operand bytes with a recognizable pattern.
        let program: &'static [u8] = &[0xAB; 8];
        let d = disassemble_one(0x8000, move |addr| {
            let offset = addr.wrapping_sub(0x8000) as usize;
            if offset == 0 {
                opcode
            } else {
                program[offset.min(program.len() - 1)]
            }
        });

        let (_, mode) = Huc6280::decode(opcode).expect("HuC6280 table is exhaustive");
        assert_eq!(
            d.length,
            1 + mode.extra_bytes(),
            "length mismatch for opcode {opcode:#04X}"
        );
        assert!(d.length >= 1);
        assert!(!d.text.is_empty());
    }
}

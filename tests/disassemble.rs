// Tests for the non-mutating single-instruction disassembler,
// `mos6502::instruction::disassemble_one`.
//
// The disassembler uses the selected variant decode table, resolves branch targets
// to absolute addresses, and renders static operands exactly as encoded (no
// register offsets, no pointer dereferencing, no zero-page relocation). It is
// driven by a side-effect-free `read` closure that is only allowed to touch the
// instruction's own bytes.

#![cfg(feature = "alloc")]

use mos6502::Variant;
use mos6502::instruction::{
    AddressingMode, Huc6280, Instruction, Nmos6502, OpInput, RevisionA, disassemble_one,
};

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
    // Keep the encoded $69 address instead of applying the HuC6280 zero-page base.
    let d = disassemble_one::<Huc6280>(0x1000, reader_at(0x1000, &[0xA5, 0x69]));

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
    assert_eq!(d.text, "lda  $69");
}

#[test]
fn immediate_store_to_vdc_port() {
    // ST1 #$00 (opcode 0x13) - a HuC6280 immediate VDC write.
    let d = disassemble_one::<Huc6280>(0x2000, reader_at(0x2000, &[0x13, 0x00]));

    assert_eq!(d.length, 2);
    assert_eq!(d.instruction, Instruction::ST1);
    assert_eq!(d.mode, AddressingMode::Immediate);
    assert!(matches!(d.operand, OpInput::UseImmediate(0x00)));
    assert_eq!(d.text, "st1  #$00");
}

#[test]
fn relative_branch_forward_target_is_absolute() {
    // BNE +5 (opcode 0xD0) at $1000: target = $1000 + 2 + 5 = $1007.
    let d = disassemble_one::<Huc6280>(0x1000, reader_at(0x1000, &[0xD0, 0x05]));

    assert_eq!(d.length, 2);
    assert_eq!(d.instruction, Instruction::BNE);
    assert_eq!(d.mode, AddressingMode::Relative);
    assert!(matches!(d.operand, OpInput::UseRelative(0x1007)));
    assert_eq!(d.text, "bne  $1007");
}

#[test]
fn relative_branch_backward_target_is_absolute() {
    // BNE -5 (offset 0xFB) at $1000: target = $1000 + 2 - 5 = $0FFD.
    let d = disassemble_one::<Huc6280>(0x1000, reader_at(0x1000, &[0xD0, 0xFB]));

    assert!(matches!(d.operand, OpInput::UseRelative(0x0FFD)));
    assert_eq!(d.text, "bne  $0FFD");
}

#[test]
fn bsr_renders_as_relative_branch() {
    // BSR +5 (HuC6280 opcode 0x44, relative branch-to-subroutine) at $1000:
    // target = $1000 + 2 + 5 = $1007, rendered like any other relative branch.
    let d = disassemble_one::<Huc6280>(0x1000, reader_at(0x1000, &[0x44, 0x05]));

    assert_eq!(d.length, 2);
    assert_eq!(d.instruction, Instruction::BSR);
    assert_eq!(d.mode, AddressingMode::Relative);
    assert!(matches!(d.operand, OpInput::UseRelative(0x1007)));
    assert_eq!(d.text, "bsr  $1007");
}

#[test]
fn bit_branch_renders_bit_index_and_absolute_target() {
    // BBR3 $12, +12 (opcode 0x3F) at $D940: target = $D940 + 3 + 12 = $D94F.
    let d = disassemble_one::<Huc6280>(0xD940, reader_at(0xD940, &[0x3F, 0x12, 0x0C]));

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
    assert_eq!(d.text, "bbr3 $12,$D94F");
}

#[test]
fn indexed_operand_is_base_address_only() {
    // LDA $2000,X (opcode 0xBD). The disassembler reports the base address with
    // no X applied; the caller adds the register offset itself.
    let d = disassemble_one::<Huc6280>(0x1000, reader_at(0x1000, &[0xBD, 0x00, 0x20]));

    assert_eq!(d.length, 3);
    assert_eq!(d.mode, AddressingMode::AbsoluteX);
    assert!(matches!(
        d.operand,
        OpInput::UseAddress {
            address: 0x2000,
            page_crossed: false
        }
    ));
    assert_eq!(d.text, "lda  $2000,x");
}

#[test]
fn indirect_operand_is_not_dereferenced() {
    // LDA ($10),Y (opcode 0xB1). The operand is the zero-page pointer byte as
    // encoded; the pointer is never followed.
    let d = disassemble_one::<Huc6280>(0x1000, reader_at(0x1000, &[0xB1, 0x10]));

    assert_eq!(d.length, 2);
    assert_eq!(d.mode, AddressingMode::IndirectIndexedY);
    assert!(matches!(
        d.operand,
        OpInput::UseAddress {
            address: 0x0010,
            page_crossed: false
        }
    ));
    assert_eq!(d.text, "lda  ($10),y");
}

#[test]
fn block_transfer_is_seven_bytes() {
    // TII $1000, $2000, $0004 (opcode 0x73): the 7-byte block-transfer form.
    let d = disassemble_one::<Huc6280>(
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
    assert_eq!(d.text, "tii  $1000,$2000,$0004");
}

#[test]
fn tst_immediate_zero_page() {
    // TST #$AA, $10 (opcode 0x83): immediate mask plus a memory operand.
    let d = disassemble_one::<Huc6280>(0x1000, reader_at(0x1000, &[0x83, 0xAA, 0x10]));

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
    assert_eq!(d.text, "tst  #$AA,$10");
}

#[test]
fn implied_instruction_has_no_operand_text() {
    // CLA (opcode 0x62): a HuC6280 implied clear with no operand.
    let d = disassemble_one::<Huc6280>(0x1000, reader_at(0x1000, &[0x62]));

    assert_eq!(d.length, 1);
    assert_eq!(d.instruction, Instruction::CLA);
    assert!(matches!(d.operand, OpInput::UseImplied));
    assert_eq!(d.text, "cla");
}

#[test]
fn selected_variant_controls_instruction_and_length() {
    // The same opcode is a two-byte NMOS RRA or a seven-byte HuC6280 transfer.
    let nmos = disassemble_one::<Nmos6502>(0x1000, |addr| {
        [0x73, 0x42][usize::from(addr.wrapping_sub(0x1000))]
    });
    let huc = disassemble_one::<Huc6280>(
        0x1000,
        reader_at(0x1000, &[0x73, 0x42, 0x10, 0x00, 0x20, 0x04, 0x00]),
    );

    assert_eq!(nmos.instruction, Instruction::RRA);
    assert_eq!(nmos.mode, AddressingMode::IndirectIndexedY);
    assert_eq!(nmos.length, 2);
    assert_eq!(nmos.text, "rra  ($42),y");
    assert_eq!(huc.instruction, Instruction::TII);
    assert_eq!(huc.mode, AddressingMode::BlockTransfer);
    assert_eq!(huc.length, 7);
    assert_eq!(huc.text, "tii  $1042,$2000,$0004");
}

#[test]
fn unknown_opcode_is_a_one_byte_data_directive() {
    // Revision A lacks ROR, including the absolute form with two operand bytes.
    let d = disassemble_one::<RevisionA>(0xFFFF, |addr| {
        assert_eq!(addr, 0xFFFF, "unknown opcodes must not read operands");
        0x6E
    });

    assert_eq!(d.opcode, 0x6E);
    assert_eq!(d.instruction, Instruction::JAM);
    assert_eq!(d.mode, AddressingMode::Implied);
    assert_eq!(d.length, 1);
    assert!(matches!(d.operand, OpInput::UseImplied));
    assert_eq!(d.text, ".db  $6E");
}

#[test]
fn mnemonic_and_display_use_lowercase() {
    for (instruction, expected) in [
        (Instruction::ADCnd, "adc"),
        (Instruction::BBR(3), "bbr3"),
        (Instruction::SMB(7), "smb7"),
        (Instruction::SLO, "slo"),
        (Instruction::SWAPAX, "sax"),
    ] {
        assert_eq!(instruction.mnemonic(), expected);
        assert_eq!(instruction.to_string(), expected);
    }
}

#[test]
fn rendering_preserves_addressing_syntax() {
    let cases: &[(&[u8], &str)] = &[
        (&[0x0A], "asl  a"),
        (&[0xB5, 0x10], "lda  $10,x"),
        (&[0xB6, 0x10], "ldx  $10,y"),
        (&[0xAD, 0x10, 0x00], "lda  $0010"),
        (&[0xB9, 0x10, 0x00], "lda  $0010,y"),
        (&[0x6C, 0x10, 0x00], "jmp  ($0010)"),
        (&[0xA1, 0x10], "lda  ($10,x)"),
        (&[0xB2, 0x10], "lda  ($10)"),
        (&[0x7C, 0x10, 0x00], "jmp  ($0010,x)"),
        (&[0xA3, 0xAA, 0x10], "tst  #$AA,$10,x"),
        (&[0x93, 0xAA, 0x10, 0x00], "tst  #$AA,$0010"),
        (&[0xB3, 0xAA, 0x10, 0x00], "tst  #$AA,$0010,x"),
    ];
    for &(bytes, text) in cases {
        let d = disassemble_one::<Huc6280>(0, |addr| bytes[usize::from(addr)]);
        assert_eq!(d.text, text);
    }
}

#[test]
fn nmos_indirect_jump_preserves_parentheses() {
    let d = disassemble_one::<Nmos6502>(0, |addr| [0x6C, 0x10, 0x00][usize::from(addr)]);
    assert_eq!(d.text, "jmp  ($0010)");
}

#[test]
fn instruction_bytes_wrap_at_end_of_address_space() {
    let d = disassemble_one::<Nmos6502>(0xFFFF, |addr| match addr {
        0xFFFF => 0xBD,
        0x0000 => 0x34,
        0x0001 => 0x12,
        _ => panic!("unexpected read at {addr:04X}"),
    });
    assert_eq!(d.text, "lda  $1234,x");
}

#[test]
fn branch_targets_wrap_at_end_of_address_space() {
    for (pc, offset, target) in [(0xFFFF_u16, 0x01, 0x0002), (0x0000, 0xFD, 0xFFFF)] {
        let d = disassemble_one::<Nmos6502>(pc, |addr| {
            [0xD0, offset][usize::from(addr.wrapping_sub(pc))]
        });
        assert_eq!(d.text, format!("bne  ${target:04X}"));
    }
}

#[test]
fn bit_branch_target_wraps_at_end_of_address_space() {
    let d = disassemble_one::<Huc6280>(0xFFFF, |addr| {
        [0x3F, 0x12, 0xFD][usize::from(addr.wrapping_sub(0xFFFF))]
    });
    assert_eq!(d.text, "bbr3 $12,$FFFF");
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
        let d = disassemble_one::<Huc6280>(0x8000, move |addr| {
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

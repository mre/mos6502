// Copyright (C) 2014 The 6502-rs Developers
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the names of the copyright holders nor the names of any
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Instruction {
    // ADd with Carry
    ADC,

    // ADd with Carry. This one has now decimal mode.
    ADCnd,

    // logical AND (bitwise)
    AND,

    // Arithmetic Shift Left
    ASL,

    // Branch if Carry Clear
    BCC,

    // Branch if Carry Set
    BCS,

    // Branch if Equal (to zero?)
    BEQ,

    // BIT test
    BIT,

    // Branch if Minus
    BMI,

    // Branch if Not Equal
    BNE,

    // Branch if Positive
    BPL,

    // Unconditional BRAnch
    BRA,

    // BReaK
    BRK,

    // BReaK, clearing decimal flag
    BRKcld,

    // Branch if oVerflow Clear
    BVC,

    // Branch if oVerflow Set
    BVS,

    // CLear Carry flag
    CLC,

    // Clear Decimal Mode
    CLD,

    // Clear Interrupt Disable
    CLI,

    // Clear oVerflow flag
    CLV,

    // Compare
    CMP,

    // Compare X register
    CPX,

    // Compare Y register
    CPY,

    // DECrement memory
    DEC,

    // DEcrement X register
    DEX,

    // DEcrement Y register
    DEY,

    // Exclusive OR (bitwise)
    EOR,

    // INCrement memory
    INC,

    // INcrement X register
    INX,

    // INcrement Y register
    INY,

    // JuMP
    JMP,

    // Jump to SubRoutine
    JSR,

    // LoaD Accumulator
    LDA,

    // LoaD X register
    LDX,

    // LoaD Y register
    LDY,

    // Logical Shift Right
    LSR,

    // No OPeration
    NOP,

    // inclusive OR (bitwise)
    ORA,

    // PusH Accumulator
    PHA,

    // PusH X
    PHX,

    // PusH Y
    PHY,

    // PusH Processor status
    PHP,

    // PuLl Accumulator
    PLA,

    // PuLl X
    PLX,

    // PuLl Y
    PLY,

    // PuLl Processor status
    PLP,

    // ROtate Left
    ROL,

    // ROtate Right
    ROR,

    // ReTurn from Interrupt
    RTI,

    // ReTurn from Subroutine
    RTS,

    // SuBtract with Carry
    SBC,

    // SuBtract with Carry. This one has now decimal mode.
    SBCnd,

    // SEt Carry flag
    SEC,

    // SEt Decimal flag
    SED,

    // SEt Interrupt disable
    SEI,

    // STore Accumulator
    STA,

    // STore X register
    STX,

    // STore Y register
    STY,

    // STore Zero
    STZ,

    // Transfer Accumulator to X
    TAX,

    // Transfer Accumulator to Y
    TAY,

    // Test and Reset Bits
    TRB,

    // Test and Set Bits
    TSB,

    // Transfer Stack pointer to X
    TSX,

    // Transfer X to Accumulator
    TXA,

    // Transfer X to Stack pointer
    TXS,

    // Transfer Y to Accumulator
    TYA,
}

#[derive(Copy, Clone, Debug)]
pub enum OpInput {
    UseImplied,
    UseImmediate(u8),
    UseRelative(u16),
    UseAddress(u16),
}

#[derive(Copy, Clone, Debug)]
pub enum AddressingMode {
    // work directly on accumulator, e. g. `lsr a`.
    Accumulator,

    // BRK
    Implied,

    // 8-bit constant in instruction, e. g. `lda #10`.
    Immediate,

    // zero-page address, e. g. `lda $00`.
    ZeroPage,

    // address is X register + 8-bit constant, e. g. `lda $80,x`.
    ZeroPageX,

    // address is Y register + 8-bit constant, e. g. `ldx $10,y`.
    ZeroPageY,

    // branch target as signed relative offset, e. g. `bne label`.
    Relative,

    // full 16-bit address, e. g. `jmp $1000`.
    Absolute,

    // full 16-bit address plus X register, e. g. `sta $1000,X`.
    AbsoluteX,

    // full 16-bit address plus Y register, e. g. `sta $1000,Y`.
    AbsoluteY,

    // jump to address stored at address, with the page-crossing bug found in NMOS chips, e. g. `jmp ($1000)`.
    BuggyIndirect,

    // jump to address stored at address, e. g. `jmp ($1000)`.
    Indirect,

    // load from address stored at (constant zero page address plus X register), e. g. `lda ($10,X)`.
    IndexedIndirectX,

    // load from (address stored at constant zero page address) plus Y register, e. g. `lda ($10),Y`.
    IndirectIndexedY,
}

impl AddressingMode {
    #[must_use]
    pub const fn extra_bytes(self) -> u16 {
        match self {
            AddressingMode::Accumulator => 0,
            AddressingMode::Implied => 0,
            AddressingMode::Immediate => 1,
            AddressingMode::ZeroPage => 1,
            AddressingMode::ZeroPageX => 1,
            AddressingMode::ZeroPageY => 1,
            AddressingMode::Relative => 1,
            AddressingMode::Absolute => 2,
            AddressingMode::AbsoluteX => 2,
            AddressingMode::AbsoluteY => 2,
            AddressingMode::Indirect => 2,
            AddressingMode::BuggyIndirect => 2,
            AddressingMode::IndexedIndirectX => 1,
            AddressingMode::IndirectIndexedY => 1,
        }
    }
}

pub type DecodedInstr = (Instruction, OpInput);

/// The NMOS 6502 variant. This one is present in the Commodore 64, early Apple IIs, etc.
#[derive(Copy, Clone, Debug)]
pub struct Nmos6502;

impl crate::Variant for Nmos6502 {
    fn decode(opcode: u8) -> Option<(Instruction, AddressingMode)> {
        match opcode {
            0x00 => Some((Instruction::BRK, AddressingMode::Implied)),
            0x01 => Some((Instruction::ORA, AddressingMode::IndexedIndirectX)),
            0x02 => None,
            0x03 => None,
            0x04 => None,
            0x05 => Some((Instruction::ORA, AddressingMode::ZeroPage)),
            0x06 => Some((Instruction::ASL, AddressingMode::ZeroPage)),
            0x07 => None,
            0x08 => Some((Instruction::PHP, AddressingMode::Implied)),
            0x09 => Some((Instruction::ORA, AddressingMode::Immediate)),
            0x0a => Some((Instruction::ASL, AddressingMode::Accumulator)),
            0x0b => None,
            0x0c => None,
            0x0d => Some((Instruction::ORA, AddressingMode::Absolute)),
            0x0e => Some((Instruction::ASL, AddressingMode::Absolute)),
            0x0f => None,
            0x10 => Some((Instruction::BPL, AddressingMode::Relative)),
            0x11 => Some((Instruction::ORA, AddressingMode::IndirectIndexedY)),
            0x12 => None,
            0x13 => None,
            0x14 => None,
            0x15 => Some((Instruction::ORA, AddressingMode::ZeroPageX)),
            0x16 => Some((Instruction::ASL, AddressingMode::ZeroPageX)),
            0x17 => None,
            0x18 => Some((Instruction::CLC, AddressingMode::Implied)),
            0x19 => Some((Instruction::ORA, AddressingMode::AbsoluteY)),
            0x1a => None,
            0x1b => None,
            0x1c => None,
            0x1d => Some((Instruction::ORA, AddressingMode::AbsoluteX)),
            0x1e => Some((Instruction::ASL, AddressingMode::AbsoluteX)),
            0x1f => None,
            0x20 => Some((Instruction::JSR, AddressingMode::Absolute)),
            0x21 => Some((Instruction::AND, AddressingMode::IndexedIndirectX)),
            0x22 => None,
            0x23 => None,
            0x24 => Some((Instruction::BIT, AddressingMode::ZeroPage)),
            0x25 => Some((Instruction::AND, AddressingMode::ZeroPage)),
            0x26 => Some((Instruction::ROL, AddressingMode::ZeroPage)),
            0x27 => None,
            0x28 => Some((Instruction::PLP, AddressingMode::Implied)),
            0x29 => Some((Instruction::AND, AddressingMode::Immediate)),
            0x2a => Some((Instruction::ROL, AddressingMode::Accumulator)),
            0x2b => None,
            0x2c => Some((Instruction::BIT, AddressingMode::Absolute)),
            0x2d => Some((Instruction::AND, AddressingMode::Absolute)),
            0x2e => Some((Instruction::ROL, AddressingMode::Absolute)),
            0x2f => None,
            0x30 => Some((Instruction::BMI, AddressingMode::Relative)),
            0x31 => Some((Instruction::AND, AddressingMode::IndirectIndexedY)),
            0x32 => None,
            0x33 => None,
            0x34 => None,
            0x35 => Some((Instruction::AND, AddressingMode::ZeroPageX)),
            0x36 => Some((Instruction::ROL, AddressingMode::ZeroPageX)),
            0x37 => None,
            0x38 => Some((Instruction::SEC, AddressingMode::Implied)),
            0x39 => Some((Instruction::AND, AddressingMode::AbsoluteY)),
            0x3a => None,
            0x3b => None,
            0x3c => None,
            0x3d => Some((Instruction::AND, AddressingMode::AbsoluteX)),
            0x3e => Some((Instruction::ROL, AddressingMode::AbsoluteX)),
            0x3f => None,
            0x40 => Some((Instruction::RTI, AddressingMode::Implied)),
            0x41 => Some((Instruction::EOR, AddressingMode::IndexedIndirectX)),
            0x42 => None,
            0x43 => None,
            0x44 => None,
            0x45 => Some((Instruction::EOR, AddressingMode::ZeroPage)),
            0x46 => Some((Instruction::LSR, AddressingMode::ZeroPage)),
            0x47 => None,
            0x48 => Some((Instruction::PHA, AddressingMode::Implied)),
            0x49 => Some((Instruction::EOR, AddressingMode::Immediate)),
            0x4a => Some((Instruction::LSR, AddressingMode::Accumulator)),
            0x4b => None,
            0x4c => Some((Instruction::JMP, AddressingMode::Absolute)),
            0x4d => Some((Instruction::EOR, AddressingMode::Absolute)),
            0x4e => Some((Instruction::LSR, AddressingMode::Absolute)),
            0x4f => None,
            0x50 => Some((Instruction::BVC, AddressingMode::Relative)),
            0x51 => Some((Instruction::EOR, AddressingMode::IndirectIndexedY)),
            0x52 => None,
            0x53 => None,
            0x54 => None,
            0x55 => Some((Instruction::EOR, AddressingMode::ZeroPageX)),
            0x56 => Some((Instruction::LSR, AddressingMode::ZeroPageX)),
            0x57 => None,
            0x58 => Some((Instruction::CLI, AddressingMode::Implied)),
            0x59 => Some((Instruction::EOR, AddressingMode::AbsoluteY)),
            0x5a => None,
            0x5b => None,
            0x5c => None,
            0x5d => Some((Instruction::EOR, AddressingMode::AbsoluteX)),
            0x5e => Some((Instruction::LSR, AddressingMode::AbsoluteX)),
            0x5f => None,
            0x60 => Some((Instruction::RTS, AddressingMode::Implied)),
            0x61 => Some((Instruction::ADC, AddressingMode::IndexedIndirectX)),
            0x62 => None,
            0x63 => None,
            0x64 => None,
            0x65 => Some((Instruction::ADC, AddressingMode::ZeroPage)),
            0x66 => Some((Instruction::ROR, AddressingMode::ZeroPage)),
            0x67 => None,
            0x68 => Some((Instruction::PLA, AddressingMode::Implied)),
            0x69 => Some((Instruction::ADC, AddressingMode::Immediate)),
            0x6a => Some((Instruction::ROR, AddressingMode::Accumulator)),
            0x6b => None,
            0x6c => Some((Instruction::JMP, AddressingMode::BuggyIndirect)),
            0x6d => Some((Instruction::ADC, AddressingMode::Absolute)),
            0x6e => Some((Instruction::ROR, AddressingMode::Absolute)),
            0x6f => None,
            0x70 => Some((Instruction::BVS, AddressingMode::Relative)),
            0x71 => Some((Instruction::ADC, AddressingMode::IndirectIndexedY)),
            0x72 => None,
            0x73 => None,
            0x74 => None,
            0x75 => Some((Instruction::ADC, AddressingMode::ZeroPageX)),
            0x76 => Some((Instruction::ROR, AddressingMode::ZeroPageX)),
            0x77 => None,
            0x78 => Some((Instruction::SEI, AddressingMode::Implied)),
            0x79 => Some((Instruction::ADC, AddressingMode::AbsoluteY)),
            0x7a => None,
            0x7b => None,
            0x7c => None,
            0x7d => Some((Instruction::ADC, AddressingMode::AbsoluteX)),
            0x7e => Some((Instruction::ROR, AddressingMode::AbsoluteX)),
            0x7f => None,
            0x80 => None,
            0x81 => Some((Instruction::STA, AddressingMode::IndexedIndirectX)),
            0x82 => None,
            0x83 => None,
            0x84 => Some((Instruction::STY, AddressingMode::ZeroPage)),
            0x85 => Some((Instruction::STA, AddressingMode::ZeroPage)),
            0x86 => Some((Instruction::STX, AddressingMode::ZeroPage)),
            0x87 => None,
            0x88 => Some((Instruction::DEY, AddressingMode::Implied)),
            0x89 => None,
            0x8a => Some((Instruction::TXA, AddressingMode::Implied)),
            0x8b => None,
            0x8c => Some((Instruction::STY, AddressingMode::Absolute)),
            0x8d => Some((Instruction::STA, AddressingMode::Absolute)),
            0x8e => Some((Instruction::STX, AddressingMode::Absolute)),
            0x8f => None,
            0x90 => Some((Instruction::BCC, AddressingMode::Relative)),
            0x91 => Some((Instruction::STA, AddressingMode::IndirectIndexedY)),
            0x92 => None,
            0x93 => None,
            0x94 => Some((Instruction::STY, AddressingMode::ZeroPageX)),
            0x95 => Some((Instruction::STA, AddressingMode::ZeroPageX)),
            0x96 => Some((Instruction::STX, AddressingMode::ZeroPageY)),
            0x97 => None,
            0x98 => Some((Instruction::TYA, AddressingMode::Implied)),
            0x99 => Some((Instruction::STA, AddressingMode::AbsoluteY)),
            0x9a => Some((Instruction::TXS, AddressingMode::Implied)),
            0x9b => None,
            0x9c => None,
            0x9d => Some((Instruction::STA, AddressingMode::AbsoluteX)),
            0x9e => None,
            0x9f => None,
            0xa0 => Some((Instruction::LDY, AddressingMode::Immediate)),
            0xa1 => Some((Instruction::LDA, AddressingMode::IndexedIndirectX)),
            0xa2 => Some((Instruction::LDX, AddressingMode::Immediate)),
            0xa3 => None,
            0xa4 => Some((Instruction::LDY, AddressingMode::ZeroPage)),
            0xa5 => Some((Instruction::LDA, AddressingMode::ZeroPage)),
            0xa6 => Some((Instruction::LDX, AddressingMode::ZeroPage)),
            0xa7 => None,
            0xa8 => Some((Instruction::TAY, AddressingMode::Implied)),
            0xa9 => Some((Instruction::LDA, AddressingMode::Immediate)),
            0xaa => Some((Instruction::TAX, AddressingMode::Implied)),
            0xab => None,
            0xac => Some((Instruction::LDY, AddressingMode::Absolute)),
            0xad => Some((Instruction::LDA, AddressingMode::Absolute)),
            0xae => Some((Instruction::LDX, AddressingMode::Absolute)),
            0xaf => None,
            0xb0 => Some((Instruction::BCS, AddressingMode::Relative)),
            0xb1 => Some((Instruction::LDA, AddressingMode::IndirectIndexedY)),
            0xb2 => None,
            0xb3 => None,
            0xb4 => Some((Instruction::LDY, AddressingMode::ZeroPageX)),
            0xb5 => Some((Instruction::LDA, AddressingMode::ZeroPageX)),
            0xb6 => Some((Instruction::LDX, AddressingMode::ZeroPageY)),
            0xb7 => None,
            0xb8 => Some((Instruction::CLV, AddressingMode::Implied)),
            0xb9 => Some((Instruction::LDA, AddressingMode::AbsoluteY)),
            0xba => Some((Instruction::TSX, AddressingMode::Implied)),
            0xbb => None,
            0xbc => Some((Instruction::LDY, AddressingMode::AbsoluteX)),
            0xbd => Some((Instruction::LDA, AddressingMode::AbsoluteX)),
            0xbe => Some((Instruction::LDX, AddressingMode::AbsoluteY)),
            0xbf => None,
            0xc0 => Some((Instruction::CPY, AddressingMode::Immediate)),
            0xc1 => Some((Instruction::CMP, AddressingMode::IndexedIndirectX)),
            0xc2 => None,
            0xc3 => None,
            0xc4 => Some((Instruction::CPY, AddressingMode::ZeroPage)),
            0xc5 => Some((Instruction::CMP, AddressingMode::ZeroPage)),
            0xc6 => Some((Instruction::DEC, AddressingMode::ZeroPage)),
            0xc7 => None,
            0xc8 => Some((Instruction::INY, AddressingMode::Implied)),
            0xc9 => Some((Instruction::CMP, AddressingMode::Immediate)),
            0xca => Some((Instruction::DEX, AddressingMode::Implied)),
            0xcb => None,
            0xcc => Some((Instruction::CPY, AddressingMode::Absolute)),
            0xcd => Some((Instruction::CMP, AddressingMode::Absolute)),
            0xce => Some((Instruction::DEC, AddressingMode::Absolute)),
            0xcf => None,
            0xd0 => Some((Instruction::BNE, AddressingMode::Relative)),
            0xd1 => Some((Instruction::CMP, AddressingMode::IndirectIndexedY)),
            0xd2 => None,
            0xd3 => None,
            0xd4 => None,
            0xd5 => Some((Instruction::CMP, AddressingMode::ZeroPageX)),
            0xd6 => Some((Instruction::DEC, AddressingMode::ZeroPageX)),
            0xd7 => None,
            0xd8 => Some((Instruction::CLD, AddressingMode::Implied)),
            0xd9 => Some((Instruction::CMP, AddressingMode::AbsoluteY)),
            0xda => None,
            0xdb => None,
            0xdc => None,
            0xdd => Some((Instruction::CMP, AddressingMode::AbsoluteX)),
            0xde => Some((Instruction::DEC, AddressingMode::AbsoluteX)),
            0xdf => None,
            0xe0 => Some((Instruction::CPX, AddressingMode::Immediate)),
            0xe1 => Some((Instruction::SBC, AddressingMode::IndexedIndirectX)),
            0xe2 => None,
            0xe3 => None,
            0xe4 => Some((Instruction::CPX, AddressingMode::ZeroPage)),
            0xe5 => Some((Instruction::SBC, AddressingMode::ZeroPage)),
            0xe6 => Some((Instruction::INC, AddressingMode::ZeroPage)),
            0xe7 => None,
            0xe8 => Some((Instruction::INX, AddressingMode::Implied)),
            0xe9 => Some((Instruction::SBC, AddressingMode::Immediate)),
            0xea => Some((Instruction::NOP, AddressingMode::Implied)),
            0xeb => None,
            0xec => Some((Instruction::CPX, AddressingMode::Absolute)),
            0xed => Some((Instruction::SBC, AddressingMode::Absolute)),
            0xee => Some((Instruction::INC, AddressingMode::Absolute)),
            0xef => None,
            0xf0 => Some((Instruction::BEQ, AddressingMode::Relative)),
            0xf1 => Some((Instruction::SBC, AddressingMode::IndirectIndexedY)),
            0xf2 => None,
            0xf3 => None,
            0xf4 => None,
            0xf5 => Some((Instruction::SBC, AddressingMode::ZeroPageX)),
            0xf6 => Some((Instruction::INC, AddressingMode::ZeroPageX)),
            0xf7 => None,
            0xf8 => Some((Instruction::SED, AddressingMode::Implied)),
            0xf9 => Some((Instruction::SBC, AddressingMode::AbsoluteY)),
            0xfa => None,
            0xfb => None,
            0xfc => None,
            0xfd => Some((Instruction::SBC, AddressingMode::AbsoluteX)),
            0xfe => Some((Instruction::INC, AddressingMode::AbsoluteX)),
            0xff => None,
        }
    }
}

/// The Ricoh variant which has no decimal mode. This is what to use if you want
/// to emulate the NES.
#[derive(Copy, Clone, Debug)]
pub struct Ricoh2a03;

impl crate::Variant for Ricoh2a03 {
    fn decode(opcode: u8) -> Option<(Instruction, AddressingMode)> {
        // It's the same as on NMOS, but doesn't support decimal mode.
        match Nmos6502::decode(opcode) {
            Some((Instruction::ADC, addressing_mode)) => {
                Some((Instruction::ADCnd, addressing_mode))
            }
            Some((Instruction::SBC, addressing_mode)) => {
                Some((Instruction::SBCnd, addressing_mode))
            }
            something_else => something_else,
        }
    }
}

/// Emulates some very early 6502s which have no ROR instruction. This one is used in very early
/// KIM-1s.
#[derive(Copy, Clone, Debug)]
pub struct RevisionA;

impl crate::Variant for RevisionA {
    fn decode(opcode: u8) -> Option<(Instruction, AddressingMode)> {
        // It's the same as on NMOS, but has no ROR instruction.
        match Nmos6502::decode(opcode) {
            Some((Instruction::ROR, _)) => None,
            something_else => something_else,
        }
    }
}

/// Emulates the 65C02, which has a few bugfixes, and another addressing mode
#[derive(Copy, Clone, Debug)]
pub struct Cmos6502;

impl crate::Variant for Cmos6502 {
    fn decode(opcode: u8) -> Option<(Instruction, AddressingMode)> {
        // TODO: We obviously need to add the other CMOS instructions here.
        match opcode {
            0x00 => Some((Instruction::BRKcld, AddressingMode::Implied)),
            0x1a => Some((Instruction::INC, AddressingMode::Accumulator)),
            0x3a => Some((Instruction::DEC, AddressingMode::Accumulator)),
            0x6c => Some((Instruction::JMP, AddressingMode::Indirect)),
            0x80 => Some((Instruction::BRA, AddressingMode::Relative)),
            0x64 => Some((Instruction::STZ, AddressingMode::ZeroPage)),
            0x74 => Some((Instruction::STZ, AddressingMode::ZeroPageX)),
            0x9c => Some((Instruction::STZ, AddressingMode::Absolute)),
            0x9e => Some((Instruction::STZ, AddressingMode::AbsoluteX)),
            0x7a => Some((Instruction::PLY, AddressingMode::Implied)),
            0xfa => Some((Instruction::PLX, AddressingMode::Implied)),
            0x5a => Some((Instruction::PHY, AddressingMode::Implied)),
            0xda => Some((Instruction::PHX, AddressingMode::Implied)),
            0x04 => Some((Instruction::TSB, AddressingMode::ZeroPage)),
            0x14 => Some((Instruction::TRB, AddressingMode::ZeroPage)),
            0x0c => Some((Instruction::TSB, AddressingMode::Absolute)),
            0x1c => Some((Instruction::TRB, AddressingMode::Absolute)),
            _ => Nmos6502::decode(opcode),
        }
    }
}

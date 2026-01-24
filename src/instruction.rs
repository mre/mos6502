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

use crate::ArithmeticOutput;

use core::fmt::{Display, Error, Formatter};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Instruction {
    // ADd with Carry
    ADC,

    // ADd with Carry. This one has no decimal mode.
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

    // SuBtract with Carry. This one has no decimal mode.
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

    // STore A & X
    SAX,

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

    // Wait for Interrupt (65C02 only)
    WAI,

    // SToP processor (65C02 only)
    STP,

    // XAA, (transfer X to A, and then ANDs the accumulator with an immediate value)
    XAA,

    // ALR, (ANDs the accumulator with an immediate value, and then does LSR)
    ALR,
}

impl Instruction {
    /// Returns the base cycle count for this instruction with the given addressing mode.
    ///
    /// This returns the minimum number of cycles required. Additional cycles may be added for:
    /// - Page boundary crossings (+1 for certain addressing modes)
    /// - Decimal mode on 65C02 (+1 for ADC/SBC when D flag is set)
    /// - Branch instructions (+1 if taken, +2 if taken and crosses page)
    #[must_use]
    pub fn base_cycles(self, mode: AddressingMode) -> u8 {
        #[allow(clippy::enum_glob_use)]
        use AddressingMode::*;
        #[allow(clippy::enum_glob_use)]
        use Instruction::*;

        match (self, mode) {
            // ADC - Add with Carry (2-6 cycles, +1 for page crossing on some modes, +1 for decimal on 65C02)
            (ADC | ADCnd, Immediate) => 2,
            (ADC | ADCnd, ZeroPage) => 3,
            (ADC | ADCnd, ZeroPageX) => 4,
            (ADC | ADCnd, Absolute) => 4,
            (ADC | ADCnd, AbsoluteX) => 4, // +1 if page crossed
            (ADC | ADCnd, AbsoluteY) => 4, // +1 if page crossed
            (ADC | ADCnd, IndexedIndirectX) => 6,
            (ADC | ADCnd, IndirectIndexedY) => 5, // +1 if page crossed
            (ADC | ADCnd, ZeroPageIndirect) => 5, // 65C02 only

            // AND - Logical AND (2-6 cycles)
            (AND, Immediate) => 2,
            (AND, ZeroPage) => 3,
            (AND, ZeroPageX) => 4,
            (AND, Absolute) => 4,
            (AND, AbsoluteX) => 4, // +1 if page crossed
            (AND, AbsoluteY) => 4, // +1 if page crossed
            (AND, IndexedIndirectX) => 6,
            (AND, IndirectIndexedY) => 5, // +1 if page crossed
            (AND, ZeroPageIndirect) => 5, // 65C02 only

            // ASL - Arithmetic Shift Left (2-7 cycles)
            (ASL, Accumulator) => 2,
            (ASL, ZeroPage) => 5,
            (ASL, ZeroPageX) => 6,
            (ASL, Absolute) => 6,
            (ASL, AbsoluteX) => 7,

            // Branch instructions - 2 cycles base, +1 if taken, +1 more if page crossed
            (BCC | BCS | BEQ | BMI | BNE | BPL | BVC | BVS, Relative) => 2,

            // BRA - Branch Always (65C02 only) - 3 cycles, +1 if page crossed
            (BRA, Relative) => 3,

            // BIT - Bit Test (2-4 cycles)
            (BIT, ZeroPage) => 3,
            (BIT, Absolute) => 4,
            (BIT, ZeroPageX) => 4, // 65C02 only
            (BIT, AbsoluteX) => 4, // 65C02 only, +1 if page crossed
            (BIT, Immediate) => 2, // 65C02 only

            // BRK - Break (7 cycles)
            (BRK | BRKcld, Implied) => 7,

            // CMP - Compare Accumulator (2-6 cycles)
            (CMP, Immediate) => 2,
            (CMP, ZeroPage) => 3,
            (CMP, ZeroPageX) => 4,
            (CMP, Absolute) => 4,
            (CMP, AbsoluteX) => 4, // +1 if page crossed
            (CMP, AbsoluteY) => 4, // +1 if page crossed
            (CMP, IndexedIndirectX) => 6,
            (CMP, IndirectIndexedY) => 5, // +1 if page crossed
            (CMP, ZeroPageIndirect) => 5, // 65C02 only

            // CPX - Compare X Register (2-4 cycles)
            (CPX, Immediate) => 2,
            (CPX, ZeroPage) => 3,
            (CPX, Absolute) => 4,

            // CPY - Compare Y Register (2-4 cycles)
            (CPY, Immediate) => 2,
            (CPY, ZeroPage) => 3,
            (CPY, Absolute) => 4,

            // DEC - Decrement Memory (2-7 cycles)
            (DEC, Accumulator) => 2, // 65C02 only
            (DEC, ZeroPage) => 5,
            (DEC, ZeroPageX) => 6,
            (DEC, Absolute) => 6,
            (DEC, AbsoluteX) => 7,

            // DEX/DEY - Decrement X/Y (2 cycles)
            (DEX | DEY, Implied) => 2,

            // EOR - Exclusive OR (2-6 cycles)
            (EOR, Immediate) => 2,
            (EOR, ZeroPage) => 3,
            (EOR, ZeroPageX) => 4,
            (EOR, Absolute) => 4,
            (EOR, AbsoluteX) => 4, // +1 if page crossed
            (EOR, AbsoluteY) => 4, // +1 if page crossed
            (EOR, IndexedIndirectX) => 6,
            (EOR, IndirectIndexedY) => 5, // +1 if page crossed
            (EOR, ZeroPageIndirect) => 5, // 65C02 only

            // Flag instructions (2 cycles)
            (CLC | CLD | CLI | CLV | SEC | SED | SEI, Implied) => 2,

            // INC - Increment Memory (2-7 cycles)
            (INC, Accumulator) => 2, // 65C02 only
            (INC, ZeroPage) => 5,
            (INC, ZeroPageX) => 6,
            (INC, Absolute) => 6,
            (INC, AbsoluteX) => 7,

            // INX/INY - Increment X/Y (2 cycles)
            (INX | INY, Implied) => 2,

            // JMP - Jump (3-6 cycles)
            (JMP, Absolute) => 3,
            (JMP, BuggyIndirect) => 5,           // NMOS with bug
            (JMP, Indirect) => 6,                // 65C02 fixed version
            (JMP, AbsoluteIndexedIndirect) => 6, // 65C02 only

            // JSR - Jump to Subroutine (6 cycles)
            (JSR, Absolute) => 6,

            // LDA - Load Accumulator (2-6 cycles)
            (LDA, Immediate) => 2,
            (LDA, ZeroPage) => 3,
            (LDA, ZeroPageX) => 4,
            (LDA, Absolute) => 4,
            (LDA, AbsoluteX) => 4, // +1 if page crossed
            (LDA, AbsoluteY) => 4, // +1 if page crossed
            (LDA, IndexedIndirectX) => 6,
            (LDA, IndirectIndexedY) => 5, // +1 if page crossed
            (LDA, ZeroPageIndirect) => 5, // 65C02 only

            // LDX - Load X Register (2-4 cycles)
            (LDX, Immediate) => 2,
            (LDX, ZeroPage) => 3,
            (LDX, ZeroPageY) => 4,
            (LDX, Absolute) => 4,
            (LDX, AbsoluteY) => 4, // +1 if page crossed

            // LDY - Load Y Register (2-4 cycles)
            (LDY, Immediate) => 2,
            (LDY, ZeroPage) => 3,
            (LDY, ZeroPageX) => 4,
            (LDY, Absolute) => 4,
            (LDY, AbsoluteX) => 4, // +1 if page crossed

            // LSR - Logical Shift Right (2-7 cycles)
            (LSR, Accumulator) => 2,
            (LSR, ZeroPage) => 5,
            (LSR, ZeroPageX) => 6,
            (LSR, Absolute) => 6,
            (LSR, AbsoluteX) => 7,

            // NOP - No Operation (2 cycles)
            (NOP, Implied) => 2,

            // ORA - Logical OR (2-6 cycles)
            (ORA, Immediate) => 2,
            (ORA, ZeroPage) => 3,
            (ORA, ZeroPageX) => 4,
            (ORA, Absolute) => 4,
            (ORA, AbsoluteX) => 4, // +1 if page crossed
            (ORA, AbsoluteY) => 4, // +1 if page crossed
            (ORA, IndexedIndirectX) => 6,
            (ORA, IndirectIndexedY) => 5, // +1 if page crossed
            (ORA, ZeroPageIndirect) => 5, // 65C02 only

            // PHA/PHP - Push Accumulator/Processor Status (3 cycles)
            (PHA | PHP, Implied) => 3,

            // PHX/PHY - Push X/Y (65C02 only, 3 cycles)
            (PHX | PHY, Implied) => 3,

            // PLA/PLP - Pull Accumulator/Processor Status (4 cycles)
            (PLA | PLP, Implied) => 4,

            // PLX/PLY - Pull X/Y (65C02 only, 4 cycles)
            (PLX | PLY, Implied) => 4,

            // ROL - Rotate Left (2-7 cycles)
            (ROL, Accumulator) => 2,
            (ROL, ZeroPage) => 5,
            (ROL, ZeroPageX) => 6,
            (ROL, Absolute) => 6,
            (ROL, AbsoluteX) => 7,

            // ROR - Rotate Right (2-7 cycles)
            (ROR, Accumulator) => 2,
            (ROR, ZeroPage) => 5,
            (ROR, ZeroPageX) => 6,
            (ROR, Absolute) => 6,
            (ROR, AbsoluteX) => 7,

            // RTI - Return from Interrupt (6 cycles)
            (RTI, Implied) => 6,

            // RTS - Return from Subroutine (6 cycles)
            (RTS, Implied) => 6,

            // SBC - Subtract with Carry (2-6 cycles, +1 for page crossing, +1 for decimal on 65C02)
            (SBC | SBCnd, Immediate) => 2,
            (SBC | SBCnd, ZeroPage) => 3,
            (SBC | SBCnd, ZeroPageX) => 4,
            (SBC | SBCnd, Absolute) => 4,
            (SBC | SBCnd, AbsoluteX) => 4, // +1 if page crossed
            (SBC | SBCnd, AbsoluteY) => 4, // +1 if page crossed
            (SBC | SBCnd, IndexedIndirectX) => 6,
            (SBC | SBCnd, IndirectIndexedY) => 5, // +1 if page crossed
            (SBC | SBCnd, ZeroPageIndirect) => 5, // 65C02 only

            // STA - Store Accumulator (3-6 cycles, NO page crossing penalty)
            (STA, ZeroPage) => 3,
            (STA, ZeroPageX) => 4,
            (STA, Absolute) => 4,
            (STA, AbsoluteX) => 5,
            (STA, AbsoluteY) => 5,
            (STA, IndexedIndirectX) => 6,
            (STA, IndirectIndexedY) => 6,
            (STA, ZeroPageIndirect) => 5, // 65C02 only

            // STX - Store X Register (3-4 cycles)
            (STX, ZeroPage) => 3,
            (STX, ZeroPageY) => 4,
            (STX, Absolute) => 4,

            // STY - Store Y Register (3-4 cycles)
            (STY, ZeroPage) => 3,
            (STY, ZeroPageX) => 4,
            (STY, Absolute) => 4,

            // STZ - Store Zero (65C02 only, 3-5 cycles)
            (STZ, ZeroPage) => 3,
            (STZ, ZeroPageX) => 4,
            (STZ, Absolute) => 4,
            (STZ, AbsoluteX) => 5,

            // Transfer instructions (2 cycles)
            (TAX | TAY | TSX | TXA | TXS | TYA, Implied) => 2,

            // TRB/TSB - Test and Reset/Set Bits (65C02 only, 5-6 cycles)
            (TRB | TSB, ZeroPage) => 5,
            (TRB | TSB, Absolute) => 6,

            // STP - Stop (65C02 only, 3 cycles before halting)
            (STP, Implied) => 3,

            // WAI - Wait for Interrupt (65C02 only, 3 cycles before waiting)
            (WAI, Implied) => 3,

            // Invalid combinations cause a panic to indicate a bug in the decoder
            _ => unreachable!("undecoded instruction"),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum OpInput {
    UseImplied,
    UseImmediate(u8),
    UseRelative(u16),
    UseAddress { address: u16, page_crossed: bool },
}

impl OpInput {
    /// Returns true if a page boundary was crossed during address calculation.
    ///
    /// Only relevant for `UseAddress` - all other addressing modes return false.
    #[must_use]
    pub const fn page_crossed(&self) -> bool {
        match self {
            OpInput::UseAddress { page_crossed, .. } => *page_crossed,
            _ => false,
        }
    }
}

impl Display for OpInput {
    fn fmt(&self, f: &mut Formatter) -> Result<(), Error> {
        match self {
            OpInput::UseImplied => write!(f, ""),
            OpInput::UseImmediate(v) => write!(f, "#${v:02X}"),
            OpInput::UseRelative(v) => write!(f, "${v:04X}"),
            OpInput::UseAddress { address, .. } => write!(f, "${address:04X}"),
        }
    }
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

    // Address stored at constant zero page address
    ZeroPageIndirect,

    // jump to address stored at (absolute address plus X register), e. g. `jmp ($1000,X)`.
    // 65C02 only
    AbsoluteIndexedIndirect,
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
            AddressingMode::ZeroPageIndirect => 1,
            AddressingMode::AbsoluteIndexedIndirect => 2,
        }
    }
}

/// A decoded instruction containing the instruction type, addressing mode, operand data,
/// and whether a page boundary was crossed during address calculation (used for cycle counting).
pub type DecodedInstr = (Instruction, AddressingMode, OpInput);

/// The NMOS 6502 variant. This one is present in the Commodore 64, early Apple IIs, etc.
#[derive(Copy, Clone, Debug, Default)]
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
            0x4b => Some((Instruction::ALR, AddressingMode::Immediate)),
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
            0x83 => Some((Instruction::SAX, AddressingMode::ZeroPage)),
            0x84 => Some((Instruction::STY, AddressingMode::ZeroPage)),
            0x85 => Some((Instruction::STA, AddressingMode::ZeroPage)),
            0x86 => Some((Instruction::STX, AddressingMode::ZeroPage)),
            0x87 => Some((Instruction::SAX, AddressingMode::ZeroPage)),
            0x88 => Some((Instruction::DEY, AddressingMode::Implied)),
            0x89 => None,
            0x8a => Some((Instruction::TXA, AddressingMode::Implied)),
            0x8b => Some((Instruction::XAA, AddressingMode::Immediate)),
            0x8c => Some((Instruction::STY, AddressingMode::Absolute)),
            0x8d => Some((Instruction::STA, AddressingMode::Absolute)),
            0x8e => Some((Instruction::STX, AddressingMode::Absolute)),
            0x8f => Some((Instruction::SAX, AddressingMode::Absolute)),
            0x90 => Some((Instruction::BCC, AddressingMode::Relative)),
            0x91 => Some((Instruction::STA, AddressingMode::IndirectIndexedY)),
            0x92 => None,
            0x93 => None,
            0x94 => Some((Instruction::STY, AddressingMode::ZeroPageX)),
            0x95 => Some((Instruction::STA, AddressingMode::ZeroPageX)),
            0x96 => Some((Instruction::STX, AddressingMode::ZeroPageY)),
            0x97 => Some((Instruction::SAX, AddressingMode::ZeroPageY)),
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

    /// NMOS 6502 ADC implementation in binary mode
    ///
    /// - Standard binary arithmetic
    /// - All flags calculated from binary result
    ///
    /// # References
    ///
    /// - [6502.org ADC documentation](http://www.6502.org/tutorials/decimal_mode.html)
    /// - [NESdev 6502 reference](https://www.nesdev.org/obelisk-6502-guide/reference.html#ADC)
    fn adc_binary(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // Binary addition with carry detection
        let carry = u8::from(carry_set);
        let result_16 = u16::from(accumulator) + u16::from(value) + u16::from(carry);
        let did_carry = result_16 > 0xFF;
        // Convert to 8-bit result using little-endian byte conversion
        let result = result_16.to_le_bytes()[0];

        // Calculate overflow from binary result
        let overflow = (!(accumulator ^ value) & (accumulator ^ result)) & 0x80 != 0;

        // Negative flag is set if the result's sign bit is set, which is the 8th bit.
        let negative = (result & 0x80) != 0;

        let zero = result == 0;

        ArithmeticOutput {
            result,
            did_carry,
            overflow,
            negative,
            zero,
        }
    }

    /// NMOS 6502 ADC implementation in decimal mode (BCD)
    ///
    /// - Binary Coded Decimal (BCD) arithmetic using 4-bit decimal digits
    /// - Each nibble represents a digit from 0-9
    /// - In decimal mode: N and Z flags are unreliable/undefined
    /// - In decimal mode: V flag calculated from binary result (documented behavior)
    ///
    /// # References
    ///
    /// - [6502.org ADC documentation](http://www.6502.org/tutorials/decimal_mode.html)
    /// - [NESdev 6502 reference](https://www.nesdev.org/obelisk-6502-guide/reference.html#ADC)
    fn adc_decimal(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // Perform binary addition first for overflow calculation
        let carry = u8::from(carry_set);
        let temp_result = accumulator.wrapping_add(value).wrapping_add(carry);

        // Decimal mode: treat each nibble as a decimal digit (0-9)
        let mut low_nibble = (accumulator & 0x0f)
            .wrapping_add(value & 0x0f)
            .wrapping_add(carry);

        let mut high_nibble = (accumulator >> 4).wrapping_add(value >> 4);
        let mut carry_to_high = false;

        // If low nibble is > 9, adjust it and carry to high nibble
        if low_nibble > 9 {
            low_nibble = low_nibble.wrapping_sub(10);
            carry_to_high = true;
        }

        high_nibble = high_nibble.wrapping_add(u8::from(carry_to_high));

        // Adjust high nibble if it's > 9 and set final carry flag
        let (adjusted_high, did_carry) = if high_nibble > 9 {
            (high_nibble.wrapping_sub(10), true)
        } else {
            (high_nibble, false)
        };

        let result = (adjusted_high << 4) | (low_nibble & 0x0f);

        // Calculate overflow from binary result (even in decimal mode)
        let overflow = (!(accumulator ^ value) & (accumulator ^ temp_result)) & 0x80 != 0;

        // Calculate other flags from final result
        let negative = (result & 0x80) != 0;
        let zero = result == 0;

        ArithmeticOutput {
            result,
            did_carry,
            overflow,
            negative,
            zero,
        }
    }

    /// NMOS 6502 SBC (Subtract with Carry) implementation in binary mode
    ///
    /// - Standard binary subtraction with borrow handling
    /// - All flags calculated from binary result
    ///
    /// # References
    ///
    /// - [6502.org SBC documentation](http://www.6502.org/tutorials/decimal_mode.html)
    /// - [NESdev 6502 reference](https://www.nesdev.org/obelisk-6502-guide/reference.html#SBC)
    fn sbc_binary(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // SBC performs: A = A - M - (1 - C)
        let carry = u8::from(carry_set);
        let temp_result = accumulator.wrapping_sub(value).wrapping_sub(1 - carry);

        // Check for borrow (unsigned underflow)
        // Borrow occurs when we need to "borrow" from a higher bit
        let did_borrow = u16::from(accumulator) < (u16::from(value) + u16::from(1 - carry));
        let did_carry = !did_borrow; // Carry is inverse of borrow in SBC

        // Calculate overflow: occurs when signs of A and M are different,
        // and the result has a different sign than A
        let overflow = (accumulator ^ value) & (accumulator ^ temp_result) & 0x80 != 0;

        // Calculate other flags
        let negative = (temp_result & 0x80) != 0;
        let zero = temp_result == 0;

        ArithmeticOutput {
            result: temp_result,
            did_carry,
            overflow,
            negative,
            zero,
        }
    }

    /// NMOS 6502 SBC implementation in decimal mode (BCD)
    ///
    /// - Binary Coded Decimal (BCD) subtraction using 4-bit decimal digits
    /// - Each nibble represents a digit from 0-9
    /// - In decimal mode: N and Z flags are unreliable/undefined (NMOS behavior)
    /// - V flag calculated from binary result (even in decimal mode)
    ///
    /// # References
    ///
    /// - [6502.org SBC documentation](http://www.6502.org/tutorials/decimal_mode.html)
    /// - [NESdev 6502 reference](https://www.nesdev.org/obelisk-6502-guide/reference.html#SBC)
    fn sbc_decimal(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // Perform binary subtraction first for overflow calculation
        let carry = u8::from(carry_set);
        let temp_result = accumulator.wrapping_sub(value).wrapping_sub(1 - carry);

        // Decimal mode: treat each nibble as a decimal digit (0-9)
        let mut low_nibble = (accumulator & 0x0f)
            .wrapping_sub(value & 0x0f)
            .wrapping_sub(1 - carry);
        let mut high_nibble = (accumulator >> 4).wrapping_sub(value >> 4);
        let mut borrow = false;

        // If low nibble underflowed (bit 4 set), correct it and set borrow
        if (low_nibble & 0x10) != 0 {
            low_nibble = (low_nibble.wrapping_add(10)) & 0x0f;
            borrow = true;
        }

        // Subtract borrow from high nibble
        high_nibble = high_nibble.wrapping_sub(u8::from(borrow));

        // If high nibble underflowed, correct it and set final borrow
        if (high_nibble & 0x10) != 0 {
            high_nibble = (high_nibble.wrapping_add(10)) & 0x0f;
            borrow = true;
        } else {
            borrow = false;
        }

        let result = (high_nibble << 4) | low_nibble;
        let did_carry = !borrow; // Carry is inverse of borrow

        // Calculate overflow from binary result (even in decimal mode)
        let overflow = (accumulator ^ value) & (accumulator ^ temp_result) & 0x80 != 0;

        // Calculate other flags from final result
        let negative = (result & 0x80) != 0;
        let zero = result == 0;

        ArithmeticOutput {
            result,
            did_carry,
            overflow,
            negative,
            zero,
        }
    }
}

/// The Ricoh variant which has no decimal mode. This is what to use if you want
/// to emulate the NES.
#[derive(Copy, Clone, Debug, Default)]
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
            other_instruction => other_instruction,
        }
    }

    /// `Ricoh2A03` (NES) ADC implementation in binary mode
    ///
    /// Same as NMOS 6502 ADC for binary.
    ///
    /// - Always performs binary arithmetic
    /// - All flags (N, Z, V, C) behave consistently like binary mode
    /// - Used in Nintendo Entertainment System (NES) and Famicom
    ///
    /// # References
    /// - [NESdev Ricoh2A03 reference](https://www.nesdev.org/wiki/CPU)
    #[inline]
    fn adc_binary(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // Ricoh2a03 behaves the same as NMOS 6502 for ADC
        Nmos6502::adc_binary(accumulator, value, carry_set)
    }

    /// `Ricoh2A03` (NES) ADC implementation - decimal mode not supported
    ///
    /// The `Ricoh2A03` removed the decimal mode circuitry entirely to save cost,
    /// so BCD operations are not supported. This method calls `adc_binary` instead.
    ///
    /// # References
    /// - [NESdev Ricoh2A03 reference](https://www.nesdev.org/wiki/CPU)
    #[inline]
    fn adc_decimal(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        Self::adc_binary(accumulator, value, carry_set)
    }

    /// `Ricoh2A03` (NES) SBC implementation in binary mode
    ///
    /// Same as NMOS 6502 SBC for binary.
    ///
    /// - Always performs binary subtraction
    /// - All flags (N, Z, V, C) behave consistently like binary mode
    /// - Used in Nintendo Entertainment System (NES) and Famicom
    ///
    /// # References
    /// - [NESdev Ricoh2A03 reference](https://www.nesdev.org/wiki/CPU)
    #[inline]
    fn sbc_binary(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // Ricoh2a03 behaves the same as NMOS 6502 for SBC
        Nmos6502::sbc_binary(accumulator, value, carry_set)
    }

    /// `Ricoh2A03` (NES) SBC implementation - decimal mode not supported
    ///
    /// The `Ricoh2A03` removed the decimal mode circuitry entirely to save cost,
    /// so BCD operations are not supported. This method calls `sbc_binary` instead.
    ///
    /// # References
    /// - [NESdev Ricoh2A03 reference](https://www.nesdev.org/wiki/CPU)
    #[inline]
    fn sbc_decimal(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // Ricoh2a03 (NES) has no decimal mode, so always use binary arithmetic
        Self::sbc_binary(accumulator, value, carry_set)
    }
}

/// Emulates some very early 6502s which have no ROR instruction. This one is used in very early
/// KIM-1s.
#[derive(Copy, Clone, Debug, Default)]
pub struct RevisionA;

impl crate::Variant for RevisionA {
    fn decode(opcode: u8) -> Option<(Instruction, AddressingMode)> {
        // It's the same as on NMOS, but has no ROR instruction.
        match Nmos6502::decode(opcode) {
            Some((Instruction::ROR, _)) => None,
            other_instruction => other_instruction,
        }
    }

    /// Revision A 6502 ADC implementation in binary mode
    ///
    /// - Identical ADC behavior to NMOS 6502
    /// - Found in very early 6502 processors (KIM-1, etc.)
    ///
    /// # References:
    ///
    /// - [Rev. A 6502 (Pre-June 1976) "ROR Bug"](https://www.masswerk.at/6502/6502_instruction_set.html#ror-bug)
    #[inline]
    fn adc_binary(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // RevisionA behaves the same as NMOS 6502 for ADC
        Nmos6502::adc_binary(accumulator, value, carry_set)
    }

    /// Revision A 6502 ADC implementation in decimal mode (BCD)
    ///
    /// - Identical ADC behavior to NMOS 6502
    /// - Supports decimal (BCD) mode with same flag behavior as NMOS
    /// - Found in very early 6502 processors (KIM-1, etc.)
    ///
    /// # Difference from NMOS 6502
    ///
    /// `RevisionA` lacks the ROR (Rotate Right) instruction entirely, but ADC
    /// behavior is identical to the standard NMOS 6502.
    ///
    /// # References:
    ///
    /// - [Rev. A 6502 (Pre-June 1976) "ROR Bug"](https://www.masswerk.at/6502/6502_instruction_set.html#ror-bug)
    #[inline]
    fn adc_decimal(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // RevisionA behaves the same as NMOS 6502 for ADC
        Nmos6502::adc_decimal(accumulator, value, carry_set)
    }

    /// Revision A 6502 SBC implementation in binary mode
    ///
    /// - Identical SBC behavior to NMOS 6502
    /// - Found in very early 6502 processors (KIM-1, etc.)
    #[inline]
    fn sbc_binary(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // RevisionA behaves the same as NMOS 6502 for SBC
        Nmos6502::sbc_binary(accumulator, value, carry_set)
    }

    /// Revision A 6502 SBC implementation in decimal mode (BCD)
    ///
    /// - Identical SBC behavior to NMOS 6502
    /// - Supports decimal (BCD) mode with same flag behavior as NMOS
    /// - Found in very early 6502 processors (KIM-1, etc.)
    #[inline]
    fn sbc_decimal(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // RevisionA behaves the same as NMOS 6502 for SBC
        Nmos6502::sbc_decimal(accumulator, value, carry_set)
    }
}

/// Emulates the Western Design Center (WDC) 65C02 microprocessor.
///
/// The 65C02 is a CMOS version of the NMOS 6502, which offers several
/// improvements while maintaining backward compatibility.
///
/// # Key Improvements Over NMOS 6502
///
/// ## Bug Fixes
/// - The NMOS 6502 had a bug when JMP (addr) crossed a page boundary.
///   The 65C02 correctly fetches both bytes of the target address.
/// - The N and Z flags now work correctly in decimal
///   (BCD) mode, whereas they were undefined in the NMOS 6502.
/// - The BRK instruction now properly clears the decimal
///   flag, preventing issues in interrupt handlers.
///
/// ## New Instructions
/// - `BRA`: Branch Always (unconditional relative branch)
/// - `PHX/PHY`: Push X/Y registers onto stack
/// - `PLX/PLY`: Pull X/Y registers from stack
/// - `STZ`: Store Zero to memory
/// - `TRB/TSB`: Test and Reset/Set memory Bits
/// - `INC A/DEC A`: Increment/Decrement Accumulator
/// - `WAI`: Wait for Interrupt (low-power mode)
/// - `STP`: Stop processor until reset (low-power mode)
///
/// ## New Addressing Modes
/// - **Zero Page Indirect**: `(zp)` for ORA, AND, EOR, ADC, STA, LDA, CMP, SBC
/// - **Absolute Indexed Indirect**: `JMP (abs,X)` - indexed indirect jump
/// - **Indexed addressing for BIT**: `BIT zp,X` and `BIT abs,X`
/// - **Immediate addressing for BIT**: `BIT #imm`
///
/// # References
/// - [WDC 65C02 Datasheet](http://www.westerndesigncenter.com/wdc/documentation/w65c02s.pdf)
/// - [65C02 Wikipedia Article](https://en.wikipedia.org/wiki/WDC_65C02)
#[derive(Copy, Clone, Debug, Default)]
pub struct Cmos6502;

impl crate::Variant for Cmos6502 {
    fn decode(opcode: u8) -> Option<(Instruction, AddressingMode)> {
        match opcode {
            0x00 => Some((Instruction::BRKcld, AddressingMode::Implied)),
            0x1a => Some((Instruction::INC, AddressingMode::Accumulator)),
            0x3a => Some((Instruction::DEC, AddressingMode::Accumulator)),
            0x6c => Some((Instruction::JMP, AddressingMode::Indirect)),
            0x7c => Some((Instruction::JMP, AddressingMode::AbsoluteIndexedIndirect)),
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
            0x12 => Some((Instruction::ORA, AddressingMode::ZeroPageIndirect)),
            0x32 => Some((Instruction::AND, AddressingMode::ZeroPageIndirect)),
            0x34 => Some((Instruction::BIT, AddressingMode::ZeroPageX)),
            0x3c => Some((Instruction::BIT, AddressingMode::AbsoluteX)),
            0x52 => Some((Instruction::EOR, AddressingMode::ZeroPageIndirect)),
            0x72 => Some((Instruction::ADC, AddressingMode::ZeroPageIndirect)),
            0x92 => Some((Instruction::STA, AddressingMode::ZeroPageIndirect)),
            0xb2 => Some((Instruction::LDA, AddressingMode::ZeroPageIndirect)),
            0xd2 => Some((Instruction::CMP, AddressingMode::ZeroPageIndirect)),
            0xf2 => Some((Instruction::SBC, AddressingMode::ZeroPageIndirect)),
            0x89 => Some((Instruction::BIT, AddressingMode::Immediate)),
            0xcb => Some((Instruction::WAI, AddressingMode::Implied)),
            0xdb => Some((Instruction::STP, AddressingMode::Implied)),
            _ => Nmos6502::decode(opcode),
        }
    }

    /// 65C02 (CMOS) ADC implementation in binary mode
    ///
    /// - Standard binary arithmetic
    /// - All flags calculated from binary result
    ///
    /// # References
    ///
    /// - [65C02 Programming Manual](http://www.westerndesigncenter.com/wdc/documentation/w65c02s.pdf)
    /// - [6502.org CMOS differences](http://www.6502.org/tutorials/65c02opcodes.html)
    fn adc_binary(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // Binary addition with carry detection
        let carry = u8::from(carry_set);
        let result_16 = u16::from(accumulator) + u16::from(value) + u16::from(carry);
        let did_carry = result_16 > 0xFF;
        #[allow(clippy::cast_possible_truncation)]
        let result = result_16 as u8;

        // Calculate overflow from binary result
        let overflow = (!(accumulator ^ value) & (accumulator ^ result)) & 0x80 != 0;

        // Calculate other flags
        let negative = (result & 0x80) != 0;
        let zero = result == 0;

        ArithmeticOutput {
            result,
            did_carry,
            overflow,
            negative,
            zero,
        }
    }

    /// 65C02 (CMOS) ADC implementation in decimal mode (BCD)
    ///
    /// - Supports decimal (BCD) mode with **corrected flag behavior**
    /// - In decimal mode: N and Z flags are **reliable** (calculated from BCD result)
    /// - In decimal mode: V flag behavior still undefined but calculated from binary result
    /// - In decimal mode: **Extra cycle** consumed (not implemented in this emulator yet)
    ///
    /// # Difference from NMOS 6502
    ///
    /// The 65C02 fixed the unreliable N and Z flags in decimal mode. These flags
    /// now correctly reflect the BCD result, making decimal arithmetic more predictable.
    ///
    /// # References
    ///
    /// - [65C02 Programming Manual](http://www.westerndesigncenter.com/wdc/documentation/w65c02s.pdf)
    /// - [6502.org CMOS differences](http://www.6502.org/tutorials/65c02opcodes.html)
    fn adc_decimal(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // Perform binary addition first for overflow calculation
        let carry = u8::from(carry_set);
        let temp_result = accumulator.wrapping_add(value).wrapping_add(carry);

        // Decimal mode: treat each nibble as a decimal digit (0-9)
        let mut low_nibble = (accumulator & 0x0f)
            .wrapping_add(value & 0x0f)
            .wrapping_add(carry);

        let mut high_nibble = (accumulator >> 4).wrapping_add(value >> 4);
        let mut carry_to_high = false;

        // If low nibble is > 9, adjust it and carry to high nibble
        if low_nibble > 9 {
            low_nibble = low_nibble.wrapping_sub(10);
            carry_to_high = true;
        }

        high_nibble = high_nibble.wrapping_add(u8::from(carry_to_high));

        // Adjust high nibble if it's > 9 and set final carry flag
        let (adjusted_high, did_carry) = if high_nibble > 9 {
            (high_nibble.wrapping_sub(10), true)
        } else {
            (high_nibble, false)
        };

        let result = (adjusted_high << 4) | (low_nibble & 0x0f);

        // Calculate overflow from binary result (even in decimal mode)
        let overflow = (!(accumulator ^ value) & (accumulator ^ temp_result)) & 0x80 != 0;

        // On 65C02, N and Z flags are valid in decimal mode (calculated from BCD result)
        // V flag behavior is still undocumented but calculated from binary result
        let negative = (result & 0x80) != 0;
        let zero = result == 0;

        ArithmeticOutput {
            result,
            did_carry,
            overflow,
            negative,
            zero,
        }
    }

    /// 65C02 (CMOS) SBC implementation in binary mode
    ///
    /// - Standard binary subtraction with borrow handling
    /// - All flags calculated from binary result
    ///
    /// # References
    ///
    /// - [65C02 Programming Manual](http://www.westerndesigncenter.com/wdc/documentation/w65c02s.pdf)
    /// - [6502.org CMOS differences](http://www.6502.org/tutorials/65c02opcodes.html)
    fn sbc_binary(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // Binary subtraction with borrow handling
        let carry = u8::from(carry_set);
        let temp_result = accumulator.wrapping_sub(value).wrapping_sub(1 - carry);

        // Check for borrow (unsigned underflow)
        let did_borrow = u16::from(accumulator) < (u16::from(value) + u16::from(1 - carry));
        let did_carry = !did_borrow; // Carry is inverse of borrow in SBC

        // Calculate overflow
        let overflow = (accumulator ^ value) & (accumulator ^ temp_result) & 0x80 != 0;

        // Calculate other flags
        let negative = (temp_result & 0x80) != 0;
        let zero = temp_result == 0;

        ArithmeticOutput {
            result: temp_result,
            did_carry,
            overflow,
            negative,
            zero,
        }
    }

    /// 65C02 (CMOS) SBC implementation in decimal mode (BCD)
    ///
    /// - Supports decimal (BCD) mode with **corrected flag behavior**
    /// - In decimal mode: N and Z flags are **reliable** (calculated from BCD result)
    /// - V flag calculated from binary result (even in decimal mode)
    /// - In decimal mode: **Extra cycle** consumed (not implemented in this emulator yet)
    ///
    /// # Difference from NMOS 6502
    ///
    /// The 65C02 fixed the unreliable N and Z flags in decimal mode. These flags
    /// now correctly reflect the BCD result, making decimal arithmetic more predictable.
    ///
    /// # References
    ///
    /// - [65C02 Programming Manual](http://www.westerndesigncenter.com/wdc/documentation/w65c02s.pdf)
    /// - [6502.org CMOS differences](http://www.6502.org/tutorials/65c02opcodes.html)
    fn sbc_decimal(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput {
        // Perform binary subtraction first for overflow calculation
        let carry = u8::from(carry_set);
        let temp_result = accumulator.wrapping_sub(value).wrapping_sub(1 - carry);

        // Decimal mode: treat each nibble as a decimal digit (0-9)
        let mut low_nibble = (accumulator & 0x0f)
            .wrapping_sub(value & 0x0f)
            .wrapping_sub(1 - carry);
        let mut high_nibble = (accumulator >> 4).wrapping_sub(value >> 4);
        let mut borrow = false;

        // If low nibble underflowed (bit 4 set), correct it and set borrow
        if (low_nibble & 0x10) != 0 {
            low_nibble = (low_nibble.wrapping_add(10)) & 0x0f;
            borrow = true;
        }

        // Subtract borrow from high nibble
        high_nibble = high_nibble.wrapping_sub(u8::from(borrow));

        // If high nibble underflowed, correct it and set final borrow
        if (high_nibble & 0x10) != 0 {
            high_nibble = (high_nibble.wrapping_add(10)) & 0x0f;
            borrow = true;
        } else {
            borrow = false;
        }

        let result = (high_nibble << 4) | low_nibble;
        let did_carry = !borrow; // Carry is inverse of borrow

        // Calculate overflow from binary result (even in decimal mode)
        let overflow = (accumulator ^ value) & (accumulator ^ temp_result) & 0x80 != 0;

        // On 65C02, N and Z flags are valid in decimal mode (calculated from BCD result)
        // V flag behavior is still undocumented but calculated from binary result
        let negative = (result & 0x80) != 0;
        let zero = result == 0;

        ArithmeticOutput {
            result,
            did_carry,
            overflow,
            negative,
            zero,
        }
    }

    fn penalty_cycles_for_decimal_mode() -> u8 {
        1 // 65C02 adds 1 cycle for ADC/SBC in decimal mode
    }

    fn penalty_cycles_for_indirect_jmp() -> u8 {
        1 // 65C02 takes 6 cycles for JMP (indirect) instead of 5
    }
}

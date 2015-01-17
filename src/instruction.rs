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

use address::Address;
use address::AddressDiff;
use machine::Machine;

// Abbreviations
//
// General
//
//        M | `Memory location`
//
// Registers
//
//        A | accumulator
//        X | general purpose register
//        Y | general purpose register
//        F | processor status flags, collectively
// NV-BDIZC | processor status flags, individually
//        S | stack pointer
//       PC | program counter
//

#[derive(Copy, Show, PartialEq, Eq)]
pub enum Instruction
      //                                  i/o vars should be listed as follows:
      //                                  NV BDIZC A X Y S PC M
      //
      //                                | outputs               | inputs
{ ADC // ADd with Carry................ | NV ...ZC A            = A + M + C
, AND // logical AND (bitwise)......... | N. ...Z. A            = A && M
, ASL // Arithmetic Shift Left......... | N. ...ZC A            = M << 1
, BCC // Branch if Carry Clear......... | .. .....         PC   = !C
, BCS // Branch if Carry Set........... | .. .....         PC   = C
, BEQ // Branch if Equal (to zero?).... | .. .....         PC   = Z
, BIT // BIT test...................... | NV ...Z.              = A & M
, BMI // Branch if Minus............... | .. .....         PC   = N
, BNE // Branch if Not Equal........... | .. .....         PC   = !Z
, BPL // Branch if Positive............ | .. .....         PC   = Z
, BRK // BReaK......................... | .. B....       S PC   =
, BVC // Branch if oVerflow Clear...... | .. .....         PC   = !V
, BVS // Branch if oVerflow Set........ | .. .....         PC   = V
, CLC // CLear Carry flag.............. | .. ....C              = 0
, CLD // Clear Decimal Mode............ | .. .D...              = 0
, CLI // Clear Interrupt Disable....... | .. ..I..              = 0
, CLV // Clear oVerflow flag........... | .V .....              = 0
, CMP // Compare....................... | N. ...ZC              = A - M
, CPX // Compare X register............ | N. ...ZC              = X - M
, CPY // Compare Y register............ | N. ...ZC              = Y - M
, DEC // DECrement memory.............. | N. ...Z.            M = M - 1
, DEX // DEcrement X register.......... | N. ...Z.   X          = X - 1
, DEY // DEcrement Y register.......... | N. ...Z.     Y        = Y - 1
, EOR // Exclusive OR (bitwise)........ | N. ...Z. A            = A ^ M
, INC // INCrement memory.............. | N. ...Z.            M = M + 1
, INX // INcrement X register.......... | N. ...Z.   X          = X + 1
, INY // INcrement Y register.......... | N. ...Z.     Y        = Y + 1
, JMP // JuMP.......................... | .. .....       S PC   =
, JSR // Jump to SubRoutine............ | .. .....       S PC   =
, LDA // LoaD Accumulator.............. | N. ...Z. A            = M
, LDX // LoaD X register............... | N. ...Z.   X          = M
, LDY // LoaD Y register............... | N. ...Z.     Y        = M
, LSR // Logical Shift Right........... | N. ...ZC A            = A/2
      //                               or N. ...ZC            M = M/2
, NOP // No OPeration.................. | .. .....              =
, ORA // inclusive OR (bitwise)........ | N. ...Z. A            = A | M
, PHA // PusH Accumulator.............. | .. .....       S    M = A
, PHP // PusH Processor status......... | .. .....       S    M = F
, PLA // PuLl Accumulator.............. | N. ...Z. A     S      = M (stack)
, PLP // PuLl Processor status......... | NV BDIZC       S      = M (stack)
, ROL // ROtate Left................... | N. ...ZC A            = C A rotated
      //                               or N. ...ZC            M = C M rotated
, ROR // ROtate Right.................. | N. ...ZC A            = C A rotated
      //                               or N. ...ZC            M = C M rotated
, RTI // ReTurn from Interrupt......... | NV BDIZC         PC   = M (stack)
, RTS // ReTurn from Subroutine........ | .. .....         PC   = M (stack)
, SBC // SuBtract with Carry........... | NV ...ZC A            = A-M-(1-C)
, SEC // SEt Carry flag................ | .. ....C              = 1
, SED // SEt Decimal flag.............. | .. .D...              = 1
, SEI // SEt Interrupt disable......... | .. ..I..              = 1
, STA // STore Accumulator............. | .. .....            M = A
, STX // STore X register.............. | .. .....            M = X
, STY // STore Y register.............. | .. .....            M = Y
, TAX // Transfer Accumulator to X..... | N. ...Z.   X          = A
, TAY // Transfer Accumulator to Y..... | N. ...Z.     Y        = A
, TSX // Transfer Stack pointer to X... | N. ...Z.   X          = S
, TXA // Transfer X to Accumulator..... | N. ...Z. A            = X
, TXS // Transfer X to Stack pointer... | .. .....       S      = X
, TYA // Transfer Y to Accumulator..... | N. ...Z. A            = Y
}

#[derive(Copy)]
pub enum OpInput {
    UseImplied,
    UseImmediate(u8),
    UseRelative(i8),
    UseAddress(Address),
}

#[derive(Copy)]
pub enum AddressingMode
//                 length
{ Accumulator      // 1    LSR A        work directly on accumulator
, Implied          // 1    BRK
, Immediate        // 2    LDA #10      8-bit constant in instruction 
, ZeroPage         // 2    LDA $00      zero-page address
, ZeroPageX        // 2    LDA $80,X    address is X register + 8-bit constant
, ZeroPageY        // 2    LDX $10,Y    address is Y register + 8-bit constant
, Relative         // 2    BNE LABEL    branch target as signed relative offset
, Absolute         // 3    JMP $1000    full 16-bit address
, AbsoluteX        // 3    STA $1000,X  full 16-bit address plus X register
, AbsoluteY        // 3    STA $1000,Y  full 16-bit address plus Y register
, Indirect         // 3    JMP ($1000)  jump to address stored at address
, IndexedIndirectX // 2    LDA ($10,X)  load from address stored at (constant
                   //                   zero page address plus X register)
, IndirectIndexedY // 2    LDA ($10),Y  load from (address stored at constant
                   //                   zero page address) plus Y register
}

fn arr_to_addr(arr: &[u8]) -> Address {
    debug_assert!(arr.len() == 2);

    let x = (arr[0] as u16) + ((arr[1] as u16) << 8us);
    Address(x)
}

impl AddressingMode {
    pub fn extra_bytes(self) -> AddressDiff {
        let x = match self {
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
            AddressingMode::IndexedIndirectX => 1,
            AddressingMode::IndirectIndexedY => 1,
        };
        AddressDiff(x)
    }

    pub fn process(self, machine: &Machine, arr: &[u8]) -> OpInput {

        debug_assert!({let AddressDiff(x) = self.extra_bytes();
                       arr.len() == x as usize});

        let x = machine.registers.index_x as u8;
        let y = machine.registers.index_y as u8;

        let memory = &machine.memory;

        match self {
            AddressingMode::Accumulator | AddressingMode::Implied => {
                // Always the same -- no input
                OpInput::UseImplied
            },
            AddressingMode::Immediate => {
                // Use [u8, ..1] specified in instruction as input
                OpInput::UseImmediate(arr[0])
            },
            AddressingMode::ZeroPage => {
                // Use [u8, ..1] from instruction
                // Interpret as zero page address
                // (Output: an 8-bit zero-page address)
                OpInput::UseAddress(Address(arr[0] as u16))
            },
            AddressingMode::ZeroPageX => {
                // Use [u8, ..1] from instruction
                // Add to X register (as u8 -- the final address is in 0-page)
                // (Output: an 8-bit zero-page address)
                OpInput::UseAddress(Address((arr[0] + x) as u16))
            },
            AddressingMode::ZeroPageY => {
                // Use [u8, ..1] from instruction
                // Add to Y register (as u8 -- the final address is in 0-page)
                // (Output: an 8-bit zero-page address)
                OpInput::UseAddress(Address((arr[0] + y) as u16))
            },
            AddressingMode::Relative => {
                // Use [u8, ..1] from instruction
                // (interpret as relative...)
                OpInput::UseRelative(arr[0] as i8)
            },
            AddressingMode::Absolute => {
                // Use [u8, ..2] from instruction as address
                // (Output: a 16-bit address)
                OpInput::UseAddress(arr_to_addr(arr))
            },
            AddressingMode::AbsoluteX => {
                // Use [u8, ..2] from instruction as address, add X
                // (Output: a 16-bit address)
                OpInput::UseAddress(arr_to_addr(arr) + AddressDiff(x as i32))
            },
            AddressingMode::AbsoluteY => {
                // Use [u8, ..2] from instruction as address, add Y
                // (Output: a 16-bit address)
                OpInput::UseAddress(arr_to_addr(arr) + AddressDiff(y as i32))
            },
            AddressingMode::Indirect => {
                // Use [u8, ..2] from instruction as an address. Interpret the
                // two bytes starting at that address as an address.
                // (Output: a 16-bit address)
                let slice = memory.get_slice(arr_to_addr(arr), AddressDiff(2));
                OpInput::UseAddress(arr_to_addr(slice))
            },
            AddressingMode::IndexedIndirectX => {
                // Use [u8, ..1] from instruction
                // Add to X register with 0-page wraparound, like ZeroPageX.
                // This is where the absolute (16-bit) target address is stored.
                // (Output: a 16-bit address)
                let start = arr[0] + x;
                let slice = memory.get_slice(Address(start as u16),
                                             AddressDiff(2));
                OpInput::UseAddress(arr_to_addr(slice))
            },
            AddressingMode::IndirectIndexedY => {
                // Use [u8, ..1] from instruction
                // This is where the absolute (16-bit) target address is stored.
                // Add Y register to this address to get the final address
                // (Output: a 16-bit address)
                let start = arr[0];
                let slice = memory.get_slice(Address(start as u16),
                                             AddressDiff(2));
                OpInput::UseAddress(arr_to_addr(slice) + AddressDiff(y as i32))
            },
        }
    }
}

pub type DecodedInstr = (Instruction, OpInput);

pub static OPCODES: [Option<(Instruction, AddressingMode)>; 256] = [
/*0x00*/ Some((Instruction::BRK, AddressingMode::Implied)),
/*0x01*/ Some((Instruction::ORA, AddressingMode::IndexedIndirectX)),
/*0x02*/ None,
/*0x03*/ None,
/*0x04*/ None,
/*0x05*/ Some((Instruction::ORA, AddressingMode::ZeroPage)),
/*0x06*/ Some((Instruction::ASL, AddressingMode::ZeroPage)),
/*0x07*/ None,
/*0x08*/ Some((Instruction::PHP, AddressingMode::Implied)),
/*0x09*/ Some((Instruction::ORA, AddressingMode::Immediate)),
/*0x0A*/ Some((Instruction::ASL, AddressingMode::Accumulator)),
/*0x0B*/ None,
/*0x0C*/ None,
/*0x0D*/ Some((Instruction::ORA, AddressingMode::Absolute)),
/*0x0E*/ Some((Instruction::ASL, AddressingMode::Absolute)),
/*0x0F*/ None,
/*0x10*/ Some((Instruction::BPL, AddressingMode::Relative)),
/*0x11*/ Some((Instruction::ORA, AddressingMode::IndirectIndexedY)),
/*0x12*/ None,
/*0x13*/ None,
/*0x14*/ None,
/*0x15*/ Some((Instruction::ORA, AddressingMode::ZeroPageX)),
/*0x16*/ Some((Instruction::ASL, AddressingMode::ZeroPageX)),
/*0x17*/ None,
/*0x18*/ Some((Instruction::CLC, AddressingMode::Implied)),
/*0x19*/ Some((Instruction::ORA, AddressingMode::AbsoluteY)),
/*0x1A*/ None,
/*0x1B*/ None,
/*0x1C*/ None,
/*0x1D*/ Some((Instruction::ORA, AddressingMode::AbsoluteX)),
/*0x1E*/ Some((Instruction::ASL, AddressingMode::AbsoluteX)),
/*0x1F*/ None,
/*0x20*/ Some((Instruction::JSR, AddressingMode::Absolute)),
/*0x21*/ Some((Instruction::AND, AddressingMode::IndexedIndirectX)),
/*0x22*/ None,
/*0x23*/ None,
/*0x24*/ Some((Instruction::BIT, AddressingMode::ZeroPage)),
/*0x25*/ Some((Instruction::AND, AddressingMode::ZeroPage)),
/*0x26*/ Some((Instruction::ROL, AddressingMode::ZeroPage)),
/*0x27*/ None,
/*0x28*/ Some((Instruction::PLP, AddressingMode::Implied)),
/*0x29*/ Some((Instruction::AND, AddressingMode::Immediate)),
/*0x2A*/ Some((Instruction::ROL, AddressingMode::Accumulator)),
/*0x2B*/ None,
/*0x2C*/ Some((Instruction::BIT, AddressingMode::Absolute)),
/*0x2D*/ Some((Instruction::AND, AddressingMode::Absolute)),
/*0x2E*/ Some((Instruction::ROL, AddressingMode::Absolute)),
/*0x2F*/ None,
/*0x30*/ Some((Instruction::BMI, AddressingMode::Relative)),
/*0x31*/ Some((Instruction::AND, AddressingMode::IndirectIndexedY)),
/*0x32*/ None,
/*0x33*/ None,
/*0x34*/ None,
/*0x35*/ Some((Instruction::AND, AddressingMode::ZeroPageX)),
/*0x36*/ Some((Instruction::ROL, AddressingMode::ZeroPageX)),
/*0x37*/ None,
/*0x38*/ Some((Instruction::SEC, AddressingMode::Implied)),
/*0x39*/ Some((Instruction::AND, AddressingMode::AbsoluteY)),
/*0x3A*/ None,
/*0x3B*/ None,
/*0x3C*/ None,
/*0x3D*/ Some((Instruction::AND, AddressingMode::AbsoluteX)),
/*0x3E*/ Some((Instruction::ROL, AddressingMode::AbsoluteX)),
/*0x3F*/ None,
/*0x40*/ Some((Instruction::RTI, AddressingMode::Implied)),
/*0x41*/ Some((Instruction::EOR, AddressingMode::IndexedIndirectX)),
/*0x42*/ None,
/*0x43*/ None,
/*0x44*/ None,
/*0x45*/ Some((Instruction::EOR, AddressingMode::ZeroPage)),
/*0x46*/ Some((Instruction::LSR, AddressingMode::ZeroPage)),
/*0x47*/ None,
/*0x48*/ Some((Instruction::PHA, AddressingMode::Implied)),
/*0x49*/ Some((Instruction::EOR, AddressingMode::Immediate)),
/*0x4A*/ Some((Instruction::LSR, AddressingMode::Accumulator)),
/*0x4B*/ None,
/*0x4C*/ Some((Instruction::JMP, AddressingMode::Absolute)),
/*0x4D*/ Some((Instruction::EOR, AddressingMode::Absolute)),
/*0x4E*/ Some((Instruction::LSR, AddressingMode::Absolute)),
/*0x4F*/ None,
/*0x50*/ Some((Instruction::BVC, AddressingMode::Relative)),
/*0x51*/ Some((Instruction::EOR, AddressingMode::IndirectIndexedY)),
/*0x52*/ None,
/*0x53*/ None,
/*0x54*/ None,
/*0x55*/ Some((Instruction::EOR, AddressingMode::ZeroPageX)),
/*0x56*/ Some((Instruction::LSR, AddressingMode::ZeroPageX)),
/*0x57*/ None,
/*0x58*/ None,
/*0x59*/ Some((Instruction::EOR, AddressingMode::AbsoluteY)),
/*0x5A*/ None,
/*0x5B*/ None,
/*0x5C*/ None,
/*0x5D*/ Some((Instruction::EOR, AddressingMode::AbsoluteX)),
/*0x5E*/ Some((Instruction::LSR, AddressingMode::AbsoluteX)),
/*0x5F*/ None,
/*0x60*/ Some((Instruction::RTS, AddressingMode::Implied)),
/*0x61*/ Some((Instruction::ADC, AddressingMode::IndexedIndirectX)),
/*0x62*/ None,
/*0x63*/ None,
/*0x64*/ None,
/*0x65*/ Some((Instruction::ADC, AddressingMode::ZeroPage)),
/*0x66*/ Some((Instruction::ROR, AddressingMode::ZeroPage)),
/*0x67*/ None,
/*0x68*/ Some((Instruction::PLA, AddressingMode::Implied)),
/*0x69*/ Some((Instruction::ADC, AddressingMode::Immediate)),
/*0x6A*/ Some((Instruction::ROR, AddressingMode::Accumulator)),
/*0x6B*/ None,
/*0x6C*/ Some((Instruction::JMP, AddressingMode::Indirect)),
/*0x6D*/ Some((Instruction::ADC, AddressingMode::Absolute)),
/*0x6E*/ Some((Instruction::ROR, AddressingMode::Absolute)),
/*0x6F*/ None,
/*0x70*/ Some((Instruction::BVS, AddressingMode::Relative)),
/*0x71*/ Some((Instruction::ADC, AddressingMode::IndirectIndexedY)),
/*0x72*/ None,
/*0x73*/ None,
/*0x74*/ None,
/*0x75*/ Some((Instruction::ADC, AddressingMode::ZeroPageX)),
/*0x76*/ Some((Instruction::ROR, AddressingMode::ZeroPageX)),
/*0x77*/ None,
/*0x78*/ Some((Instruction::SEI, AddressingMode::Implied)),
/*0x79*/ Some((Instruction::ADC, AddressingMode::AbsoluteY)),
/*0x7A*/ None,
/*0x7B*/ None,
/*0x7C*/ None,
/*0x7D*/ Some((Instruction::ADC, AddressingMode::AbsoluteX)),
/*0x7E*/ Some((Instruction::ROR, AddressingMode::AbsoluteX)),
/*0x7F*/ None,
/*0x80*/ None,
/*0x81*/ Some((Instruction::STA, AddressingMode::IndexedIndirectX)),
/*0x82*/ None,
/*0x83*/ None,
/*0x84*/ Some((Instruction::STY, AddressingMode::ZeroPage)),
/*0x85*/ Some((Instruction::STA, AddressingMode::ZeroPage)),
/*0x86*/ Some((Instruction::STX, AddressingMode::ZeroPage)),
/*0x87*/ None,
/*0x88*/ Some((Instruction::DEY, AddressingMode::Implied)),
/*0x89*/ None,
/*0x8A*/ Some((Instruction::TXA, AddressingMode::Implied)),
/*0x8B*/ None,
/*0x8C*/ Some((Instruction::STY, AddressingMode::Absolute)),
/*0x8D*/ Some((Instruction::STA, AddressingMode::Absolute)),
/*0x8E*/ Some((Instruction::STX, AddressingMode::Absolute)),
/*0x8F*/ None,
/*0x90*/ Some((Instruction::BCC, AddressingMode::Relative)),
/*0x91*/ Some((Instruction::STA, AddressingMode::IndirectIndexedY)),
/*0x92*/ None,
/*0x93*/ None,
/*0x94*/ Some((Instruction::STY, AddressingMode::ZeroPageX)),
/*0x95*/ Some((Instruction::STA, AddressingMode::ZeroPageX)),
/*0x96*/ Some((Instruction::STX, AddressingMode::ZeroPageY)),
/*0x97*/ None,
/*0x98*/ Some((Instruction::TYA, AddressingMode::Implied)),
/*0x99*/ Some((Instruction::STA, AddressingMode::AbsoluteY)),
/*0x9A*/ Some((Instruction::TXS, AddressingMode::Implied)),
/*0x9B*/ None,
/*0x9C*/ None,
/*0x9D*/ Some((Instruction::STA, AddressingMode::AbsoluteX)),
/*0x9E*/ None,
/*0x9F*/ None,
/*0xA0*/ Some((Instruction::LDY, AddressingMode::Immediate)),
/*0xA1*/ Some((Instruction::LDA, AddressingMode::IndexedIndirectX)),
/*0xA2*/ Some((Instruction::LDX, AddressingMode::Immediate)),
/*0xA3*/ None,
/*0xA4*/ Some((Instruction::LDY, AddressingMode::ZeroPage)),
/*0xA5*/ Some((Instruction::LDA, AddressingMode::ZeroPage)),
/*0xA6*/ Some((Instruction::LDX, AddressingMode::ZeroPage)),
/*0xA7*/ None,
/*0xA8*/ Some((Instruction::TAY, AddressingMode::Implied)),
/*0xA9*/ Some((Instruction::LDA, AddressingMode::Immediate)),
/*0xAA*/ Some((Instruction::TAX, AddressingMode::Implied)),
/*0xAB*/ None,
/*0xAC*/ Some((Instruction::LDY, AddressingMode::Absolute)),
/*0xAD*/ Some((Instruction::LDA, AddressingMode::Absolute)),
/*0xAE*/ Some((Instruction::LDX, AddressingMode::Absolute)),
/*0xAF*/ None,
/*0xB0*/ Some((Instruction::BCS, AddressingMode::Relative)),
/*0xB1*/ Some((Instruction::LDA, AddressingMode::IndirectIndexedY)),
/*0xB2*/ None,
/*0xB3*/ None,
/*0xB4*/ Some((Instruction::LDY, AddressingMode::ZeroPageX)),
/*0xB5*/ Some((Instruction::LDA, AddressingMode::ZeroPageX)),
/*0xB6*/ Some((Instruction::LDX, AddressingMode::ZeroPageY)),
/*0xB7*/ None,
/*0xB8*/ Some((Instruction::CLV, AddressingMode::Implied)),
/*0xB9*/ Some((Instruction::LDA, AddressingMode::AbsoluteY)),
/*0xBA*/ Some((Instruction::TSX, AddressingMode::Implied)),
/*0xBB*/ None,
/*0xBC*/ Some((Instruction::LDY, AddressingMode::AbsoluteX)),
/*0xBD*/ Some((Instruction::LDA, AddressingMode::AbsoluteX)),
/*0xBE*/ Some((Instruction::LDX, AddressingMode::AbsoluteY)),
/*0xBF*/ None,
/*0xC0*/ Some((Instruction::CPY, AddressingMode::Immediate)),
/*0xC1*/ Some((Instruction::CMP, AddressingMode::IndexedIndirectX)),
/*0xC2*/ None,
/*0xC3*/ None,
/*0xC4*/ Some((Instruction::CPY, AddressingMode::ZeroPage)),
/*0xC5*/ Some((Instruction::CMP, AddressingMode::ZeroPage)),
/*0xC6*/ Some((Instruction::DEC, AddressingMode::ZeroPage)),
/*0xC7*/ None,
/*0xC8*/ Some((Instruction::INY, AddressingMode::Implied)),
/*0xC9*/ Some((Instruction::CMP, AddressingMode::Immediate)),
/*0xCA*/ Some((Instruction::DEX, AddressingMode::Implied)),
/*0xCB*/ None,
/*0xCC*/ Some((Instruction::CPY, AddressingMode::Absolute)),
/*0xCD*/ Some((Instruction::CMP, AddressingMode::Absolute)),
/*0xCE*/ Some((Instruction::DEC, AddressingMode::Absolute)),
/*0xCF*/ None,
/*0xD0*/ Some((Instruction::BNE, AddressingMode::Relative)),
/*0xD1*/ Some((Instruction::CMP, AddressingMode::IndirectIndexedY)),
/*0xD2*/ None,
/*0xD3*/ None,
/*0xD4*/ None,
/*0xD5*/ Some((Instruction::CMP, AddressingMode::ZeroPageX)),
/*0xD6*/ Some((Instruction::DEC, AddressingMode::ZeroPageX)),
/*0xD7*/ None,
/*0xD8*/ Some((Instruction::CLD, AddressingMode::Implied)),
/*0xD9*/ Some((Instruction::CMP, AddressingMode::AbsoluteY)),
/*0xDA*/ None,
/*0xDB*/ None,
/*0xDC*/ None,
/*0xDD*/ Some((Instruction::CMP, AddressingMode::AbsoluteX)),
/*0xDE*/ Some((Instruction::DEC, AddressingMode::AbsoluteX)),
/*0xDF*/ None,
/*0xE0*/ Some((Instruction::CPX, AddressingMode::Immediate)),
/*0xE1*/ Some((Instruction::SBC, AddressingMode::IndexedIndirectX)),
/*0xE2*/ None,
/*0xE3*/ None,
/*0xE4*/ Some((Instruction::CPX, AddressingMode::ZeroPage)),
/*0xE5*/ Some((Instruction::SBC, AddressingMode::ZeroPage)),
/*0xE6*/ Some((Instruction::INC, AddressingMode::ZeroPage)),
/*0xE7*/ None,
/*0xE8*/ Some((Instruction::INX, AddressingMode::Implied)),
/*0xE9*/ Some((Instruction::SBC, AddressingMode::Immediate)),
/*0xEA*/ Some((Instruction::NOP, AddressingMode::Implied)),
/*0xEB*/ None,
/*0xEC*/ Some((Instruction::CPX, AddressingMode::Absolute)),
/*0xED*/ Some((Instruction::SBC, AddressingMode::Absolute)),
/*0xEE*/ Some((Instruction::INC, AddressingMode::Absolute)),
/*0xEF*/ None,
/*0xF0*/ Some((Instruction::BEQ, AddressingMode::Relative)),
/*0xF1*/ Some((Instruction::SBC, AddressingMode::IndirectIndexedY)),
/*0xF2*/ None,
/*0xF3*/ None,
/*0xF4*/ None,
/*0xF5*/ Some((Instruction::SBC, AddressingMode::ZeroPageX)),
/*0xF6*/ Some((Instruction::INC, AddressingMode::ZeroPageX)),
/*0xF7*/ None,
/*0xF8*/ Some((Instruction::SED, AddressingMode::Implied)),
/*0xF9*/ Some((Instruction::SBC, AddressingMode::AbsoluteY)),
/*0xFA*/ None,
/*0xFB*/ None,
/*0xFC*/ None,
/*0xFD*/ Some((Instruction::SBC, AddressingMode::AbsoluteX)),
/*0xFE*/ Some((Instruction::INC, AddressingMode::AbsoluteX)),
/*0xFF*/ None,
];


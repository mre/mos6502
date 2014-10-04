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

#[deriving(Show, PartialEq, Eq)]
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

pub enum AMOut {
    UseImplied,
    UseImmediate(u8),
    UseRelative(i8),
    UseAddress(Address),
}

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

    let x = (arr[0] as u16) + (arr[1] as u16 << 8u);
    Address(x)
}

impl AddressingMode {
    pub fn extra_bytes(self) -> AddressDiff {
        let x = match self {
            Accumulator => 0,
            Implied => 0,
            Immediate => 1,
            ZeroPage => 1,
            ZeroPageX => 1,
            ZeroPageY => 1,
            Relative => 1,
            Absolute => 2,
            AbsoluteX => 2,
            AbsoluteY => 2,
            Indirect => 2,
            IndexedIndirectX => 1,
            IndirectIndexedY => 1,
        };
        AddressDiff(x)
    }

    pub fn process(self, machine: &Machine, arr: &[u8]) -> AMOut {

        debug_assert!({let AddressDiff(x) = self.extra_bytes();
                       arr.len() == x as uint});

        let x = machine.registers.index_x as u8;
        let y = machine.registers.index_y as u8;

        let memory = &machine.memory;

        match self {
            Accumulator | Implied => {
                // Always the same -- no input
                UseImplied
            },
            Immediate => {
                // Use [u8, ..1] specified in instruction as input
                UseImmediate(arr[0])
            },
            ZeroPage => {
                // Use [u8, ..1] from instruction
                // Interpret as zero page address
                // (Output: an 8-bit zero-page address)
                UseAddress(Address(arr[0] as u16))
            },
            ZeroPageX => {
                // Use [u8, ..1] from instruction
                // Add to X register (as u8 -- the final address is in 0-page)
                // (Output: an 8-bit zero-page address)
                UseAddress(Address((arr[0] + x) as u16))
            },
            ZeroPageY => {
                // Use [u8, ..1] from instruction
                // Add to Y register (as u8 -- the final address is in 0-page)
                // (Output: an 8-bit zero-page address)
                UseAddress(Address((arr[0] + y) as u16))
            },
            Relative => {
                // Use [u8, ..1] from instruction
                // (interpret as relative...)
                UseRelative(arr[0] as i8)
            },
            Absolute => {
                // Use [u8, ..2] from instruction as address
                // (Output: a 16-bit address)
                UseAddress(arr_to_addr(arr))
            },
            AbsoluteX => {
                // Use [u8, ..2] from instruction as address, add X
                // (Output: a 16-bit address)
                UseAddress(arr_to_addr(arr) + AddressDiff(x as u16))
            },
            AbsoluteY => {
                // Use [u8, ..2] from instruction as address, add Y
                // (Output: a 16-bit address)
                UseAddress(arr_to_addr(arr) + AddressDiff(y as u16))
            },
            Indirect => {
                // Use [u8, ..2] from instruction as an address. Interpret the
                // two bytes starting at that address as an address.
                // (Output: a 16-bit address)
                let slice = memory.get_slice(arr_to_addr(arr), AddressDiff(2));
                UseAddress(arr_to_addr(slice))
            },
            IndexedIndirectX => {
                // Use [u8, ..1] from instruction
                // Add to X register with 0-page wraparound, like ZeroPageX.
                // This is where the absolute (16-bit) target address is stored.
                // (Output: a 16-bit address)
                let start = arr[0] + x;
                let slice = memory.get_slice(Address(start as u16),
                                             AddressDiff(2));
                UseAddress(arr_to_addr(slice))
            },
            IndirectIndexedY => {
                // Use [u8, ..1] from instruction
                // This is where the absolute (16-bit) target address is stored.
                // Add Y register to this address to get the final address
                // (Output: a 16-bit address)
                let start = arr[0];
                let slice = memory.get_slice(Address(start as u16),
                                             AddressDiff(2));
                UseAddress(arr_to_addr(slice) + AddressDiff(y as u16))
            },
        }
    }
}

pub static g_opcodes: [Option<(Instruction, AddressingMode)>, ..256] = [
/*0x00*/ Some((BRK, Implied)),
/*0x01*/ Some((ORA, IndexedIndirectX)),
/*0x02*/ None,
/*0x03*/ None,
/*0x04*/ None,
/*0x05*/ Some((ORA, ZeroPage)),
/*0x06*/ Some((ASL, ZeroPage)),
/*0x07*/ None,
/*0x08*/ Some((PHP, Implied)),
/*0x09*/ Some((ORA, Immediate)),
/*0x0A*/ Some((ASL, Accumulator)),
/*0x0B*/ None,
/*0x0C*/ None,
/*0x0D*/ Some((ORA, Absolute)),
/*0x0E*/ Some((ASL, Absolute)),
/*0x0F*/ None,
/*0x10*/ Some((BPL, Relative)),
/*0x11*/ Some((ORA, IndirectIndexedY)),
/*0x12*/ None,
/*0x13*/ None,
/*0x14*/ None,
/*0x15*/ Some((ORA, ZeroPageX)),
/*0x16*/ Some((ASL, ZeroPageX)),
/*0x17*/ None,
/*0x18*/ Some((CLC, Implied)),
/*0x19*/ Some((ORA, AbsoluteY)),
/*0x1A*/ None,
/*0x1B*/ None,
/*0x1C*/ None,
/*0x1D*/ Some((ORA, AbsoluteX)),
/*0x1E*/ Some((ASL, AbsoluteX)),
/*0x1F*/ None,
/*0x20*/ Some((JSR, Absolute)),
/*0x21*/ Some((AND, IndexedIndirectX)),
/*0x22*/ None,
/*0x23*/ None,
/*0x24*/ Some((BIT, ZeroPage)),
/*0x25*/ Some((AND, ZeroPage)),
/*0x26*/ Some((ROL, ZeroPage)),
/*0x27*/ None,
/*0x28*/ Some((PLP, Implied)),
/*0x29*/ Some((AND, Immediate)),
/*0x2A*/ Some((ROL, Accumulator)),
/*0x2B*/ None,
/*0x2C*/ Some((BIT, Absolute)),
/*0x2D*/ Some((AND, Absolute)),
/*0x2E*/ Some((ROL, Absolute)),
/*0x2F*/ None,
/*0x30*/ Some((BMI, Relative)),
/*0x31*/ Some((AND, IndirectIndexedY)),
/*0x32*/ None,
/*0x33*/ None,
/*0x34*/ None,
/*0x35*/ Some((AND, ZeroPageX)),
/*0x36*/ Some((ROL, ZeroPageX)),
/*0x37*/ None,
/*0x38*/ Some((SEC, Implied)),
/*0x39*/ Some((AND, AbsoluteY)),
/*0x3A*/ None,
/*0x3B*/ None,
/*0x3C*/ None,
/*0x3D*/ Some((AND, AbsoluteX)),
/*0x3E*/ Some((ROL, AbsoluteX)),
/*0x3F*/ None,
/*0x40*/ Some((RTI, Implied)),
/*0x41*/ Some((EOR, IndexedIndirectX)),
/*0x42*/ None,
/*0x43*/ None,
/*0x44*/ None,
/*0x45*/ Some((EOR, ZeroPage)),
/*0x46*/ Some((LSR, ZeroPage)),
/*0x47*/ None,
/*0x48*/ Some((PHA, Implied)),
/*0x49*/ Some((EOR, Immediate)),
/*0x4A*/ Some((LSR, Accumulator)),
/*0x4B*/ None,
/*0x4C*/ Some((JMP, Absolute)),
/*0x4D*/ Some((EOR, Absolute)),
/*0x4E*/ Some((LSR, Absolute)),
/*0x4F*/ None,
/*0x50*/ Some((BVC, Relative)),
/*0x51*/ Some((EOR, IndirectIndexedY)),
/*0x52*/ None,
/*0x53*/ None,
/*0x54*/ None,
/*0x55*/ Some((EOR, ZeroPageX)),
/*0x56*/ Some((LSR, ZeroPageX)),
/*0x57*/ None,
/*0x58*/ None,
/*0x59*/ Some((EOR, AbsoluteY)),
/*0x5A*/ None,
/*0x5B*/ None,
/*0x5C*/ None,
/*0x5D*/ Some((EOR, AbsoluteX)),
/*0x5E*/ Some((LSR, AbsoluteX)),
/*0x5F*/ None,
/*0x60*/ Some((RTS, Implied)),
/*0x61*/ Some((ADC, IndexedIndirectX)),
/*0x62*/ None,
/*0x63*/ None,
/*0x64*/ None,
/*0x65*/ Some((ADC, ZeroPage)),
/*0x66*/ Some((ROR, ZeroPage)),
/*0x67*/ None,
/*0x68*/ Some((PLA, Implied)),
/*0x69*/ Some((ADC, Immediate)),
/*0x6A*/ Some((ROR, Accumulator)),
/*0x6B*/ None,
/*0x6C*/ Some((JMP, Indirect)),
/*0x6D*/ Some((ADC, Absolute)),
/*0x6E*/ Some((ROR, Absolute)),
/*0x6F*/ None,
/*0x70*/ Some((BVS, Relative)),
/*0x71*/ Some((ADC, IndirectIndexedY)),
/*0x72*/ None,
/*0x73*/ None,
/*0x74*/ None,
/*0x75*/ Some((ADC, ZeroPageX)),
/*0x76*/ Some((ROR, ZeroPageX)),
/*0x77*/ None,
/*0x78*/ Some((SEI, Implied)),
/*0x79*/ Some((ADC, AbsoluteY)),
/*0x7A*/ None,
/*0x7B*/ None,
/*0x7C*/ None,
/*0x7D*/ Some((ADC, AbsoluteX)),
/*0x7E*/ Some((ROR, AbsoluteX)),
/*0x7F*/ None,
/*0x80*/ None,
/*0x81*/ Some((STA, IndexedIndirectX)),
/*0x82*/ None,
/*0x83*/ None,
/*0x84*/ Some((STY, ZeroPage)),
/*0x85*/ Some((STA, ZeroPage)),
/*0x86*/ Some((STX, ZeroPage)),
/*0x87*/ None,
/*0x88*/ Some((DEY, Implied)),
/*0x89*/ None,
/*0x8A*/ Some((TXA, Implied)),
/*0x8B*/ None,
/*0x8C*/ Some((STY, Absolute)),
/*0x8D*/ Some((STA, Absolute)),
/*0x8E*/ Some((STX, Absolute)),
/*0x8F*/ None,
/*0x90*/ Some((BCC, Relative)),
/*0x91*/ Some((STA, IndirectIndexedY)),
/*0x92*/ None,
/*0x93*/ None,
/*0x94*/ Some((STY, ZeroPageX)),
/*0x95*/ Some((STA, ZeroPageX)),
/*0x96*/ Some((STX, ZeroPageY)),
/*0x97*/ None,
/*0x98*/ Some((TYA, Implied)),
/*0x99*/ Some((STA, AbsoluteY)),
/*0x9A*/ Some((TXS, Implied)),
/*0x9B*/ None,
/*0x9C*/ None,
/*0x9D*/ Some((STA, AbsoluteX)),
/*0x9E*/ None,
/*0x9F*/ None,
/*0xA0*/ Some((LDY, Immediate)),
/*0xA1*/ Some((LDA, IndexedIndirectX)),
/*0xA2*/ Some((LDX, Immediate)),
/*0xA3*/ None,
/*0xA4*/ Some((LDY, ZeroPage)),
/*0xA5*/ Some((LDA, ZeroPage)),
/*0xA6*/ Some((LDX, ZeroPage)),
/*0xA7*/ None,
/*0xA8*/ Some((TAY, Implied)),
/*0xA9*/ Some((LDA, Immediate)),
/*0xAA*/ Some((TAX, Implied)),
/*0xAB*/ None,
/*0xAC*/ Some((LDY, Absolute)),
/*0xAD*/ Some((LDA, Absolute)),
/*0xAE*/ Some((LDX, Absolute)),
/*0xAF*/ None,
/*0xB0*/ Some((BCS, Relative)),
/*0xB1*/ Some((LDA, IndirectIndexedY)),
/*0xB2*/ None,
/*0xB3*/ None,
/*0xB4*/ Some((LDY, ZeroPageX)),
/*0xB5*/ Some((LDA, ZeroPageX)),
/*0xB6*/ Some((LDX, ZeroPageY)),
/*0xB7*/ None,
/*0xB8*/ Some((CLV, Implied)),
/*0xB9*/ Some((LDA, AbsoluteY)),
/*0xBA*/ Some((TSX, Implied)),
/*0xBB*/ None,
/*0xBC*/ Some((LDY, AbsoluteX)),
/*0xBD*/ Some((LDA, AbsoluteX)),
/*0xBE*/ Some((LDX, AbsoluteY)),
/*0xBF*/ None,
/*0xC0*/ Some((CPY, Immediate)),
/*0xC1*/ Some((CMP, IndexedIndirectX)),
/*0xC2*/ None,
/*0xC3*/ None,
/*0xC4*/ Some((CPY, ZeroPage)),
/*0xC5*/ Some((CMP, ZeroPage)),
/*0xC6*/ Some((DEC, ZeroPage)),
/*0xC7*/ None,
/*0xC8*/ Some((INY, Implied)),
/*0xC9*/ Some((CMP, Immediate)),
/*0xCA*/ Some((DEX, Implied)),
/*0xCB*/ None,
/*0xCC*/ Some((CPY, Absolute)),
/*0xCD*/ Some((CMP, Absolute)),
/*0xCE*/ Some((DEC, Absolute)),
/*0xCF*/ None,
/*0xD0*/ Some((BNE, Relative)),
/*0xD1*/ Some((CMP, IndirectIndexedY)),
/*0xD2*/ None,
/*0xD3*/ None,
/*0xD4*/ None,
/*0xD5*/ Some((CMP, ZeroPageX)),
/*0xD6*/ Some((DEC, ZeroPageX)),
/*0xD7*/ None,
/*0xD8*/ Some((CLD, Implied)),
/*0xD9*/ Some((CMP, AbsoluteY)),
/*0xDA*/ None,
/*0xDB*/ None,
/*0xDC*/ None,
/*0xDD*/ Some((CMP, AbsoluteX)),
/*0xDE*/ Some((DEC, AbsoluteX)),
/*0xDF*/ None,
/*0xE0*/ Some((CPX, Immediate)),
/*0xE1*/ Some((SBC, IndexedIndirectX)),
/*0xE2*/ None,
/*0xE3*/ None,
/*0xE4*/ Some((CPX, ZeroPage)),
/*0xE5*/ Some((SBC, ZeroPage)),
/*0xE6*/ Some((INC, ZeroPage)),
/*0xE7*/ None,
/*0xE8*/ Some((INX, Implied)),
/*0xE9*/ Some((SBC, Immediate)),
/*0xEA*/ Some((NOP, Implied)),
/*0xEB*/ None,
/*0xEC*/ Some((CPX, Absolute)),
/*0xED*/ Some((SBC, Absolute)),
/*0xEE*/ Some((INC, Absolute)),
/*0xEF*/ None,
/*0xF0*/ Some((BEQ, Relative)),
/*0xF1*/ Some((SBC, IndirectIndexedY)),
/*0xF2*/ None,
/*0xF3*/ None,
/*0xF4*/ None,
/*0xF5*/ Some((SBC, ZeroPageX)),
/*0xF6*/ Some((INC, ZeroPageX)),
/*0xF7*/ None,
/*0xF8*/ Some((SED, Implied)),
/*0xF9*/ Some((SBC, AbsoluteY)),
/*0xFA*/ None,
/*0xFB*/ None,
/*0xFC*/ None,
/*0xFD*/ Some((SBC, AbsoluteX)),
/*0xFE*/ Some((INC, AbsoluteX)),
/*0xFF*/ None,
];


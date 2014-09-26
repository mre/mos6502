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

extern crate std;

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
// NV-BDIZC | processor status flags -- see ProcessorStatus bitflags
//       SP | stack pointer
//       PC | program counter
//

pub bitflags! {
    flags ProcessorStatus: u8 {
        static N = 0b10000000, // Negative  -- sometimes called S for "sign"
        static V = 0b01000000, // oVerflow
        static B = 0b00010000, // Brk
        static D = 0b00001000, // Decimal mode active?
        static I = 0b00000100, // Irq disabled?
        static Z = 0b00000010, // Zero
        static C = 0b00000001, // Carry
    }
}

#[allow(non_snake_case)]
pub struct Machine {
    pub  A : u8,
    pub  X : u8,
    pub  Y : u8,
    pub  P : ProcessorStatus,
    pub SP : u8,
    pub PC : u16,
}

impl Machine {
    pub fn new() -> Machine {
        Machine {
            A: 0,
            X: 0,
            Y: 0,
            SP: 0,
            P: ProcessorStatus::empty(),
            PC: 0
        }
    }
}

#[deriving(PartialEq, Eq, PartialOrd, Ord)]
pub struct StackPointer(u8);

#[deriving(PartialEq, Eq, PartialOrd, Ord)]
pub struct Addr(u16);

#[deriving(PartialEq, Eq, PartialOrd, Ord)]
pub struct AddrDiff(u16);

// The idea here is that it doesn't make sense to add two addresses, but it
// does make sense to add an address and an "address-difference". (If this
// is too annoying to work with we should let it go.)

impl Add<AddrDiff, Addr> for Addr {
    fn add(&self, &AddrDiff(rhs): &AddrDiff) -> Addr {
        let &Addr(lhs) = self;

        // We probably don't want to overflow when doing arithmetic in our own
        // code.
        debug_assert!({
            match lhs.checked_add(&rhs) {
                None => false,
                _ => true
            }
        });

        return Addr(lhs + rhs);
    }
}

pub static STACK_POINTER_IN_MEMORY_LO: Addr = Addr(0x0100);
pub static STACK_POINTER_IN_MEMORY_HI: Addr = Addr(0x01FF);

pub fn stack_pointer_to_addr(StackPointer(x) : StackPointer) -> Addr
{
	STACK_POINTER_IN_MEMORY_LO + AddrDiff(x as u16)
}

// We can probably come up with a better way to represent address ranges
pub static IRQ_INTERRUPT_VECTOR_LO: Addr = Addr(0xFFFE);
pub static IRQ_INTERRUPT_VECTOR_HI: Addr = Addr(0xFFFE);

#[deriving(Show, PartialEq, Eq)]
pub enum Instruction
      //                                  i/o vars should be listed as follows:
      //                                  NV BDIZC A X Y SP PC M
      //
      //                                | outputs                | inputs
{ ADC // ADd with Carry................ | NV ...ZC A             = A + M + C
, AND // logical AND (bitwise)......... | N. ...Z. A             = A && M
, ASL // Arithmetic Shift Left......... | N. ...ZC A             = M << 1
, BCC // Branch if Carry Clear......... | .. .....          PC   = !C
, BCS // Branch if Carry Set........... | .. .....          PC   = C
, BEQ // Branch if Equal (to zero?).... | .. .....          PC   = Z
, BIT // BIT test...................... | NV ...Z.               = A & M
, BMI // Branch if Minus............... | .. .....          PC   = N
, BNE // Branch if Not Equal........... | .. .....          PC   = !Z
, BPL // Branch if Positive............ | .. .....          PC   = Z
, BRK // BReaK......................... | .. B....       SP PC   =
, BVC // Branch if oVerflow Clear...... | .. .....          PC   = !V
, BVS // Branch if oVerflow Set........ | .. .....          PC   = V
, CLC // CLear Carry flag.............. | .. ....C               = 0
, CLD // Clear Decimal Mode............ | .. .D...               = 0
, CLI // Clear Interrupt Disable....... | .. ..I..               = 0
, CLV // Clear oVerflow flag........... | .V .....               = 0
, CMP // Compare....................... | N. ...ZC               = A - M
, CPX // Compare X register............ | N. ...ZC               = X - M
, CPY // Compare Y register............ | N. ...ZC               = Y - M
, DEC // DECrement memory.............. | N. ...Z.             M = M - 1
, DEX // DEcrement X register.......... | N. ...Z.   X           = X - 1
, DEY // DEcrement Y register.......... | N. ...Z.     Y         = Y - 1
, EOR // Exclusive OR (bitwise)........ | N. ...Z. A             = A ^ M
, INC // INCrement memory.............. | N. ...Z.             M = M + 1
, INX // INcrement X register.......... | N. ...Z.   X           = X + 1
, INY // INcrement Y register.......... | N. ...Z.     Y         = Y + 1
, JMP // JuMP.......................... | .. .....          PC   =
, JSR // Jump to SubRoutine............ | .. .....
, LDA // LoaD Accumulator.............. | .. .....
, LDX // LoaD X register............... | .. .....
, LDY // LoaD Y register............... | .. .....
, LSR // Logical Shift Right........... | .. .....
, NOP // No OPeration.................. | .. .....
, ORA // inclusive OR.................. | .. .....
, PHA // PusH Accumulator.............. | .. .....
, PHP // PusH Processor status......... | .. .....
, PLA // PuLl Accumulator.............. | .. .....
, PLP // PuLl Processor status......... | .. .....
, ROL // ROtate Left................... | .. .....
, ROR // ROtate Right.................. | .. .....
, RTI // ReTurn from Interrupt......... | .. .....
, RTS // ReTurn from Subroutine........ | .. .....
, SBC // SuBtract with Carry........... | .. .....
, SEC // SEt Carry flag................ | .. .....
, SED // SEt Decimal flag.............. | .. .....
, SEI // SEt Interrupt disable......... | .. .....
, STA // STore Accumulator............. | .. .....
, STX // STore X register.............. | .. .....
, STY // STore Y register.............. | .. .....
, TAX // Transfer Accumulator to X..... | .. .....
, TAY // Transfer Accumulator to Y..... | .. .....
, TSX // Transfer Stack pointer to X... | .. .....
, TXA // Transfer X to Accumulator..... | .. .....
, TXS // Transfer X to Stack pointer... | .. .....
, TYA // Transfer Y to Accumulator..... | .. .....
}

pub enum AddressingMode {
    Immediate,
    Absolute,
    ZeroPage,
    Implied,
    IndirectAbsolute,
    AbsoluteIndexedX,
    AbsoluteIndexedY,
    ZeroPageIndexedX,
    ZeroageIndexedY,
    IndexedIndirect,
    IndirectIndexed,
    Relative,
    Accumulator,
}


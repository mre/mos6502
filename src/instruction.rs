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


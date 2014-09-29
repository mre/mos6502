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


#[deriving(Show, PartialEq, Eq)]
pub enum Instruction
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
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

use util::BitFlag;
use memory::Memory;
use registers::Registers;

pub struct Machine {
	pub registers: Registers,
	pub memory:    Memory
}

impl Machine {
	pub fn new() -> Machine {
		Machine{
			registers: Registers::new(),
			memory:    Memory::new()
		}
	}

	// TODO akeeton: Implement binary-coded decimal.
	pub fn add_with_carry(&mut self, value: i8) {
		let a: int = self.registers.accumulator  as int;
		let c: int = self.registers.status.carry as int;

		let a_new_full: int = a + c + value as int;
		let a_new:      i8  = a_new_full as i8;

		let did_carry    = a_new_full != a_new as int;
		let is_zero      = a_new == 0;
		let is_negative  = a_new < 0;
		let did_overflow =
			   (a < 0 && value < 0 && a_new >= 0)
			|| (a > 0 && value > 0 && a_new <= 0);

		self.registers.accumulator     = a_new;
		self.registers.status.carry    = BitFlag::new(did_carry);
		self.registers.status.zero     = BitFlag::new(is_zero);
		self.registers.status.sign     = BitFlag::new(is_negative);
		self.registers.status.overflow = BitFlag::new(did_overflow);
	}
}
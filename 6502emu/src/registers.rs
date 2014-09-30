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

use memory::STACK_ADDRESS_END;
use util::{ BitFlag, Off, On };

pub struct Status {
	pub carry:              BitFlag,
	pub zero:               BitFlag,
	pub disable_interrupts: BitFlag,
	pub decimal_mode:       BitFlag,
	pub brk:                BitFlag,
	pub unused:             BitFlag,
	pub overflow:           BitFlag,
	pub negative:           BitFlag
}

impl Status {
	pub fn to_byte(&self) -> u8 {
		  self.carry.to_bit()              << 0
		| self.zero.to_bit()               << 1
		| self.disable_interrupts.to_bit() << 2
		| self.decimal_mode.to_bit()       << 3
		| self.brk.to_bit()                << 4
		| self.unused.to_bit()             << 5
		| self.overflow.to_bit()           << 6
		| self.negative.to_bit()           << 7
	}

	pub fn new() -> Status {
		// TODO akeeton: Revisit these defaults.
		Status {
			carry:              Off,
			zero:               Off,
			disable_interrupts:  On,
			decimal_mode:       Off,
			brk:                Off,
			unused:              On,
			overflow:           Off,
			negative:           Off
		}
	}
}

#[deriving(PartialEq, Eq, PartialOrd, Ord)]
pub struct StackPointer(pub u8);

pub struct Registers {
	pub accumulator:     i8,
	pub index_x:         i8,
	pub index_y:         i8,
	pub stack_pointer:   StackPointer,
	pub program_counter: u16,
	pub status:          Status
}

impl Registers {
	pub fn new() -> Registers {
		// TODO akeeton: Revisit these defaults.
		Registers {
			accumulator:     0,
			index_x:         0,
			index_y:         0,
			stack_pointer:   StackPointer(STACK_ADDRESS_END.get_offset()),
			program_counter: 0,
			status:          Status::new()
		}
	}
}
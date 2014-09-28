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

mod registers {
	// Each status flag should be 0 or 1.
	pub struct Status {
		pub carry:        u8,
		pub zero:         u8,
		pub interrupt:    u8,
		pub decimal_mode: u8,
		pub brk:          u8,
		pub unused:       u8,
		pub overflow:     u8,
		pub sign:         u8
	}

	impl Status {
		pub fn to_byte(&self) -> u8 {
			  self.carry        << 0
			| self.zero         << 1
			| self.interrupt    << 2
			| self.decimal_mode << 3
			| self.brk          << 4
			| self.unused       << 5
			| self.overflow     << 6
			| self.sign         << 7
		}
	}

	pub struct Registers {
		pub accumulator:     i8,
		pub index_x:         i8,
		pub index_y:         i8,
		pub stack_pointer:   u8,
		pub program_counter: u16,
		pub status:          Status
	}

	impl Registers {
	}
}
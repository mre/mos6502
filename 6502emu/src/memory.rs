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
use registers::StackPointer;

pub static MEMORY_ADDRESS_BEGIN:    Address = Address(0x0000);
pub static MEMORY_ADDRESS_END:      Address = Address(0xFFFF);
pub static STACK_ADDRESS_BEGIN:     Address = Address(0x0100);
pub static STACK_ADDRESS_END:       Address = Address(0x01FF);
pub static IRQ_INTERRUPT_VECTOR_LO: Address = Address(0xFFFE);
pub static IRQ_INTERRUPT_VECTOR_HI: Address = Address(0xFFFF);


// static MEMORY_SIZE: uint    = MEMORY_ADDRESS_END - MEMORY_ADDRESS_BEGIN + 1;
pub struct Memory {
	// Rust doesn't seem to like this:
	// bytes: [u8, ..MEMORY_SIZE]
	bytes: [u8, ..2^16]
}

impl Memory {
	pub fn new() -> Memory {
		Memory { bytes: [0, ..2^16] }
	}

	pub fn get_byte(&self, address: &Address) -> u8 {
		self.bytes[address.to_uint()]
	}

	// Sets the byte at the given address to the given value and returns the
	// previous value at the address.
	pub fn set_byte(&mut self, address: &Address, value: u8) -> u8 {
		let old_value = self.get_byte(address);
		self.bytes[address.to_uint()] = value;

		return old_value;
	}

	pub fn is_stack_address(address: &Address) -> bool {
		STACK_ADDRESS_BEGIN <= *address && *address <= STACK_ADDRESS_END
	}

	pub fn stack_pointer_to_address(&StackPointer(sp): &StackPointer) -> Address
	{
		STACK_ADDRESS_BEGIN + AddressDiff(sp as u16)
	}
}

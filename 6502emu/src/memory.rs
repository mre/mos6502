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

mod address;

mod memory {
	use address::Address;

	pub struct Memory {
		bytes: [u8,.. 256]
	}

	impl Memory {
		fn address_to_byte_offset(address: &Address) -> uint {
			(address.to_int() - Address::min().to_int()) as uint
		}

		pub fn get_byte(&self, address: &Address) -> u8 {
			if !address.is_valid()
			{
				fail!("Invalid address.");
			}
			else
			{
				return self.bytes[Memory::address_to_byte_offset(address)];
			}
		}

		// Sets the byte at the given address to the given value and returns the
		// previous value at the address.
		pub fn set_byte(&mut self, address: &Address, value: u8) -> u8 {
			if !address.is_valid()
			{
				fail!("Invalid address.");
			}
			else
			{
				let old_value = self.get_byte(address);
				self.bytes[Memory::address_to_byte_offset(address)] = value;

				return old_value;
			}
		}
	}
}

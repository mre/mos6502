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

// JAM: We can probably come up with a better way to represent address ranges.
//      Address range type?
//
// // Address range -- inclusive on both sides
// pub struct AddressRangeIncl {
//     begin: Address,
//     end: Address,
// }

const ADDR_LO_BARE: u16 = 0x0000;
const ADDR_HI_BARE: u16 = 0xFFFF;

pub const MEMORY_ADDRESS_LO: u16 = ADDR_LO_BARE;
pub const MEMORY_ADDRESS_HI: u16 = ADDR_HI_BARE;
pub const STACK_ADDRESS_LO: u16 = 0x0100;
pub const STACK_ADDRESS_HI: u16 = 0x01FF;
pub const IRQ_INTERRUPT_VECTOR_LO: u16 = 0xFFFE;
pub const IRQ_INTERRUPT_VECTOR_HI: u16 = 0xFFFF;

const MEMORY_SIZE: usize = (ADDR_HI_BARE - ADDR_LO_BARE) as usize + 1usize;

// FIXME: Should this use indirection for `bytes`?
#[derive(Copy, Clone)]
pub struct Memory {
    bytes: [u8; MEMORY_SIZE],
}

impl Default for Memory {
    fn default() -> Self {
        Self::new()
    }
}

pub trait Bus {
    fn get_byte(&mut self, address: u16) -> u8;
    fn set_byte(&mut self, address: u16, value: u8);

    fn set_bytes(&mut self, start: u16, values: &[u8]) {
        for i in 0..values.len() as u16 {
            self.set_byte(start + i, values[i as usize]);
        }
    }
}

impl Memory {
    pub fn new() -> Memory {
        Memory {
            bytes: [0; MEMORY_SIZE],
        }
    }
}

impl Bus for Memory {
    fn get_byte(&mut self, address: u16) -> u8 {
        self.bytes[address as usize]
    }

    // Sets the byte at the given address to the given value and returns the
    // previous value at the address.
    fn set_byte(&mut self, address: u16, value: u8) {
        self.bytes[address as usize] = value;
    }

    fn set_bytes(&mut self, start: u16, values: &[u8]) {
        let start = start as usize;

        // This panics if the range is invalid
        let end = start + values.len();

        self.bytes[start..end].copy_from_slice(values);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic]
    fn test_memory_overflow_panic() {
        let mut memory = Memory::new();
        memory.set_bytes(0xFFFE, &[1, 2, 3]);
    }
}

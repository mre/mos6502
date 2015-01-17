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

use address::{Address, AddressDiff};

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

pub const MEMORY_ADDRESS_LO:       Address = Address(ADDR_LO_BARE);
pub const MEMORY_ADDRESS_HI:       Address = Address(ADDR_HI_BARE);
pub const STACK_ADDRESS_LO:        Address = Address(0x0100);
pub const STACK_ADDRESS_HI:        Address = Address(0x01FF);
pub const IRQ_INTERRUPT_VECTOR_LO: Address = Address(0xFFFE);
pub const IRQ_INTERRUPT_VECTOR_HI: Address = Address(0xFFFF);

const MEMORY_SIZE: usize = (ADDR_HI_BARE - ADDR_LO_BARE) as usize + 1us;

// FIXME: Should this use indirection for `bytes`?
#[derive(Copy)]
pub struct Memory {
    bytes: [u8; MEMORY_SIZE]
}

impl Memory {
    pub fn new() -> Memory {
        Memory { bytes: [0; MEMORY_SIZE] }
    }

    pub fn get_byte(&self, address: Address) -> u8 {
        self.bytes[address.to_usize()]
    }

    pub fn get_byte_mut_ref(&mut self, address: Address) -> &mut u8 {
        &mut self.bytes[address.to_usize()]
    }

    pub fn get_slice(&self, Address(start): Address,
                     AddressDiff(diff): AddressDiff) -> &[u8] {
        let start = start as usize;
        let diff = diff as usize;
        let end = start + diff;
        &self.bytes[start..end]
    }

    // Sets the byte at the given address to the given value and returns the
    // previous value at the address.
    pub fn set_byte(&mut self, address: Address, value: u8) -> u8 {
        let old_value = self.get_byte(address);
        self.bytes[address.to_usize()] = value;
        old_value
    }

    pub fn set_bytes(&mut self, Address(start): Address, values: &[u8]) {
        let start = start as usize;

        // This panics if the range is invalid
        let end = start + values.len();
        let slice = &mut self.bytes[start..end];

        // JAM: Is this the best way to do this copy?
        for (dest, src) in slice.iter_mut().zip(values.iter()) {
            *dest = *src;
        }
    }

    pub fn is_stack_address(address: &Address) -> bool {
        STACK_ADDRESS_LO <= *address && *address <= STACK_ADDRESS_HI
    }
}


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
use memory::{STACK_ADDRESS_LO, STACK_ADDRESS_HI};

// Useful for constructing Status instances
#[derive(Copy, Clone)]
pub struct StatusArgs {
    pub negative: bool,
    pub overflow: bool,
    pub unused: bool,
    pub brk: bool,
    pub decimal_mode: bool,
    pub disable_interrupts: bool,
    pub zero: bool,
    pub carry: bool,
}

impl StatusArgs {
    pub fn none() -> StatusArgs {
        StatusArgs {
            negative: false,
            overflow: false,
            unused: false,
            brk: false,
            decimal_mode: false,
            disable_interrupts: false,
            zero: false,
            carry: false,
        }
    }
}

bitflags! {
    pub struct Status: u8 {
        const PS_NEGATIVE           = 0b10000000;
        const PS_OVERFLOW           = 0b01000000;
        const PS_UNUSED             = 0b00100000; // JAM: Should this exist?
                                                  // (note that it affects the
                                                  // behavior of things like
                                                  // from_bits_truncate)
        const PS_BRK                = 0b00010000;
        const PS_DECIMAL_MODE       = 0b00001000;
        const PS_DISABLE_INTERRUPTS = 0b00000100;
        const PS_ZERO               = 0b00000010;
        const PS_CARRY              = 0b00000001;
    }
}

impl Status {
    pub fn default() -> Status {
        // TODO akeeton: Revisit these defaults.

        Status::new(StatusArgs {
            negative: false,
            overflow: false,
            unused: true,
            brk: false,
            decimal_mode: false,
            disable_interrupts: true,
            zero: false,
            carry: false,
        })
    }

    pub fn new(StatusArgs { negative,
                            overflow,
                            unused,
                            brk,
                            decimal_mode,
                            disable_interrupts,
                            zero,
                            carry }: StatusArgs) -> Status
    {
        let mut out = Status::empty();

        if negative {
            out = out | PS_NEGATIVE
        }
        if overflow {
            out = out | PS_OVERFLOW
        }
        if unused {
            out = out | PS_UNUSED
        }
        if brk {
            out = out | PS_BRK
        }
        if decimal_mode {
            out = out | PS_DECIMAL_MODE
        }
        if disable_interrupts {
            out = out | PS_DISABLE_INTERRUPTS
        }
        if zero {
            out = out | PS_ZERO
        }
        if carry {
            out = out | PS_CARRY
        }

        out
    }

    pub fn and(&mut self, rhs: Status) {
        *self = *self & rhs;
    }

    pub fn or(&mut self, rhs: Status) {
        *self = *self | rhs;
    }

    pub fn set_with_mask(&mut self, mask: Status, rhs: Status) {
        *self = (*self & !mask) | rhs;
    }
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct StackPointer(pub u8);

impl StackPointer {
    pub fn to_address(&self) -> Address {
        let StackPointer(sp) = *self;
        STACK_ADDRESS_LO + AddressDiff(sp as i32)
    }

    // JAM: FIXME: Should we prevent overflow here? What would a 6502 do?

    pub fn decrement(&mut self) {
        let StackPointer(val) = *self;
        *self = StackPointer(val - 1);
    }

    pub fn increment(&mut self) {
        let StackPointer(val) = *self;
        *self = StackPointer(val + 1);
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Registers {
    pub accumulator: i8,
    pub index_x: i8,
    pub index_y: i8,
    pub stack_pointer: StackPointer,
    pub program_counter: Address,
    pub status: Status,
}

impl Registers {
    pub fn new() -> Registers {
        // TODO akeeton: Revisit these defaults.
        Registers {
            accumulator: 0,
            index_x: 0,
            index_y: 0,
            stack_pointer: StackPointer(STACK_ADDRESS_HI.get_offset()),
            program_counter: Address(0),
            status: Status::default(),
        }
    }
}

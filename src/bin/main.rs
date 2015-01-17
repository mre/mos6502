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

extern crate emu6502;

#[cfg(not(test))]
use emu6502::machine;

#[cfg(not(test))]
use emu6502::address::Address;

#[cfg(not(test))]
fn main() {
    let mut machine = machine::Machine::new();

    // "Load" a program

    // JAM: FIXME: What's the syntax for specifying the array element type,
    //             but not the length? (For a fixed-size array)

    let zero_page_data: [u8; 17] = [
        // ZeroPage data start
        0x00,
        0x02, // ADC ZeroPage target
        0x00,
        0x04, // ADC ZeroPageX target
        0x00,
        0x00,
        0x00,
        0x00,
        0x10, // ADC IndexedIndirectX address
        0x80, // ADC IndexedIndirectX address
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x08, // ADC IndirectIndexedY address
        0x80, // ADC IndirectIndexedY address
    ];

    let program: [u8; 33] = [
        // Code start
        0xA9, // LDA Immediate
        0x01, //     Immediate operand

        0x69, // ADC Immediate
        0x07, //     Immediate operand

        0x65, // ADC ZeroPage
        0x01, //     ZeroPage operand

        0xA2, // LDX Immediate
        0x01, //     Immediate operand

        0x75, // ADC ZeroPageX
        0x02, //     ZeroPageX operand

        0x6D, // ADC Absolute
        0x01, //     Absolute operand
        0x80, //     Absolute operand

        0xA2, // LDX immediate
        0x08, //     Immediate operand

        0x7D, // ADC AbsoluteX
        0x00, //     AbsoluteX operand
        0x80, //     AbsoluteX operand

        0xA0, // LDY immediate
        0x04, //     Immediate operand

        0x79, // ADC AbsoluteY
        0x00, //     AbsoluteY operand
        0x80, //     AbsoluteY operand

        0xA2, // LDX immediate
        0x05, //     Immediate operand

        0x61, // ADC IndexedIndirectX
        0x03, //     IndexedIndirectX operand

        0xA0, // LDY immediate
        0x10, //     Immediate operand

        0x71, // ADC IndirectIndexedY
        0x0F, //     IndirectIndexedY operand

        0xEA, // NOP :)

        0xFF, // Something invalid -- the end!
    ];

    let data: [u8; 25] = [
        0x00,
        0x09, // ADC Absolute target
        0x00,
        0x00,
        0x40, // ADC AbsoluteY target
        0x00,
        0x00,
        0x00,
        0x11, // ADC AbsoluteX target
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x12, // ADC IndexedIndirectX target
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x06, // ADC IndirectIndexedY target
    ];

    machine.memory.set_bytes(Address(0x0000), &zero_page_data);
    machine.memory.set_bytes(Address(0x4000), &program);
    machine.memory.set_bytes(Address(0x8000), &data);

    machine.registers.program_counter = Address(0x4000);

    machine.run();

    println!("{:?}", machine);
}


#![no_std]
#![no_main]

// you can put a breakpoint on `rust_begin_unwind` to catch panics
extern crate panic_halt;

extern crate mos6502;

use cortex_m_rt::entry;
use mos6502::address::Address;
use mos6502::cpu;

use once_cell::unsync::Lazy;

#[entry]
fn main() -> ! {
    let mut cpu = Lazy::new(|| cpu::CPU::new());

    loop {
        let zero_page_data = [
            // ZeroPage data start
            0x00, 0x02, // ADC ZeroPage target
            0x00, 0x04, // ADC ZeroPageX target
            0x00, 0x00, 0x00, 0x00, 0x10, // ADC IndexedIndirectX address
            0x80, // ADC IndexedIndirectX address
            0x00, 0x00, 0x00, 0x00, 0x00, 0x08, // ADC IndirectIndexedY address
            0x80, // ADC IndirectIndexedY address
        ];

        let program = [
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
            0x00, 0x09, // ADC Absolute target
            0x00, 0x00, 0x40, // ADC AbsoluteY target
            0x00, 0x00, 0x00, 0x11, // ADC AbsoluteX target
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, // ADC IndexedIndirectX target
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, // ADC IndirectIndexedY target
        ];

        // "Load" a program
        cpu.memory.set_bytes(Address(0x0000), &zero_page_data);
        cpu.memory.set_bytes(Address(0x4000), &program);
        cpu.memory.set_bytes(Address(0x8000), &data);

        cpu.registers.program_counter = Address(0x4000);

        cpu.run();
    }
}

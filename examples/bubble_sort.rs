use mos6502::cpu;
use mos6502::instruction::Nmos6502;
use mos6502::memory::{Bus, Memory, IRQ_INTERRUPT_VECTOR_HI, IRQ_INTERRUPT_VECTOR_LO};
use std::fs::read;

fn main() {
    println!("Enter numbers (< 256) separated by spaces to sort:");
    let mut input = String::new();
    std::io::stdin().read_line(&mut input).unwrap();

    let numbers: Vec<u8> = input
        .split_whitespace()
        .filter_map(|s| s.parse::<u8>().ok())
        .collect();

    if numbers.is_empty() {
        println!("No valid numbers entered.");
        return;
    }

    if numbers.len() > 16 {
        println!("Too many numbers. Maximum is 16.");
        return;
    }

    println!("Before sorting: {:?}", numbers);

    // Handle trivial cases (0 or 1 element - already sorted)
    if numbers.len() <= 1 {
        println!("After sorting:  {:?}", numbers);
        return;
    }

    // Load the binary file from disk
    let program = match read("examples/asm/bubble_sort/bubble_sort.bin") {
        Ok(data) => data,
        Err(err) => {
            println!("Error reading bubble_sort.bin: {err}");
            println!("Make sure to build it first with: make build-examples");
            return;
        }
    };

    let mut cpu = cpu::CPU::new(Memory::new(), Nmos6502);

    // Set up memory:
    // $00: array length
    // $20-$2F: array data (avoiding $01-$0F which may have special uses)
    cpu.memory.set_byte(0x00, numbers.len() as u8);
    cpu.memory.set_bytes(0x20, &numbers);

    // Load program and set PC
    cpu.memory.set_bytes(0x0400, &program);
    cpu.registers.program_counter = 0x0400;

    // Set up BRK interrupt vector to point to $FFFF (halt)
    cpu.memory.set_byte(IRQ_INTERRUPT_VECTOR_LO, 0xFF);
    cpu.memory.set_byte(IRQ_INTERRUPT_VECTOR_HI, 0xFF);

    // Run the sort
    cpu.run();

    // Read sorted array back
    let mut sorted = Vec::new();
    for i in 0..numbers.len() {
        sorted.push(cpu.memory.get_byte(0x20 + i as u16));
    }

    println!("After sorting:  {:?}", sorted);
}

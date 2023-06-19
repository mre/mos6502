use mos6502::cpu;
use mos6502::memory::Bus;
use mos6502::memory::Memory;
use std::fs::read;

fn main() {
    println!("Enter two numbers (< 128) separated by a space to know their GCD.");
    let mut input = String::new();
    std::io::stdin().read_line(&mut input).unwrap();

    let zero_page_data = input
        .split_whitespace()
        .map(|s| s.parse::<u8>().unwrap())
        .collect::<Vec<u8>>();

    // Load the binary file from disk
    let program = match read("examples/asm/euclid/euclid.bin") {
        Ok(data) => data,
        Err(err) => {
            println!("Error reading euclid.bin: {}", err);
            return;
        }
    };

    let mut cpu = cpu::CPU::new(Memory::new());

    cpu.memory.set_bytes(0x00, &zero_page_data);
    cpu.memory.set_bytes(0x10, &program);
    cpu.registers.program_counter = 0x10;

    cpu.run();

    println!("GCD is {}", cpu.registers.accumulator);
}

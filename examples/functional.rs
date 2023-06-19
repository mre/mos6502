use mos6502::cpu;
use mos6502::memory::Bus;
use mos6502::memory::Memory;
use std::fs::read;

fn main() {
    // Load the binary file from disk
    let program = match read("examples/asm/functional_test/6502_functional_test.bin") {
        Ok(data) => data,
        Err(err) => {
            println!("Error reading functional test: {}", err);
            return;
        }
    };

    let mut cpu = cpu::CPU::new(Memory::new());

    cpu.memory.set_bytes(0x00, &program);
    cpu.registers.program_counter = 0x400;

    // run step-by-step
    let mut old_pc = cpu.registers.program_counter;
    while cpu.registers.program_counter != 0x3468 {
        // Use `fetch_next_and_decode` instead of
        // `single_step` to see the decoded instruction
        if let Some(decoded_instr) = cpu.fetch_next_and_decode() {
            println!("{decoded_instr:?}");
            cpu.execute_instruction(decoded_instr);
        }
        cpu.single_step();
        println!("{cpu:?}");

        if cpu.registers.program_counter == old_pc {
            println!("Infinite loop detected!");
            break;
        }

        old_pc = cpu.registers.program_counter;
    }
}

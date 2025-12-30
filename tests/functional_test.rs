// Klaus2m5 6502 functional test
// https://github.com/Klaus2m5/6502_65C02_functional_tests

use mos6502::cpu;
use mos6502::instruction::Nmos6502;
use mos6502::memory::Bus;
use mos6502::memory::Memory;
use std::fs::read;

// Test configuration constants
const TEST_BINARY_PATH: &str = "tests/assets/6502_functional_test.bin";
const PROGRAM_LOAD_ADDR: u16 = 0x0000;
const PROGRAM_START_ADDR: u16 = 0x0400;
const SUCCESS_ADDR: u16 = 0x331C; // Address of the "jmp *" success trap

// Safety limit to prevent infinite loops in case of emulator bugs
const MAX_INSTRUCTIONS: u64 = 100_000_000; // Klaus2m5 test needs ~30 million instructions

#[test]
fn klaus2m5_functional_test() {
    // Load the binary file from disk
    let program = read(TEST_BINARY_PATH).expect("Could not read functional test binary");

    let mut cpu = cpu::CPU::new(Memory::new(), Nmos6502);

    cpu.memory.set_bytes(PROGRAM_LOAD_ADDR, &program);
    cpu.registers.program_counter = PROGRAM_START_ADDR;

    // Run the test
    let mut old_pc = cpu.registers.program_counter;
    let mut instr_count = 0u64;

    while cpu.registers.program_counter != SUCCESS_ADDR {
        let current_pc = cpu.registers.program_counter;

        // Safety check to prevent infinite loops
        assert!(
            instr_count < MAX_INSTRUCTIONS,
            "Test exceeded maximum instruction count ({}) at PC ${:04X}\n\
             This likely indicates a bug in the emulator causing an infinite loop.\n\
             CPU state: {:?}",
            MAX_INSTRUCTIONS,
            current_pc,
            cpu.registers
        );

        // Fetch and decode instruction
        let decoded_instr = cpu.fetch_next_and_decode();
        let opcode = cpu.memory.get_byte(current_pc);

        assert!(
            decoded_instr.is_some(),
            "Illegal opcode ${:02X} at PC ${:04X} after {} instructions\n\
             Success address is ${:04X}\n\
             CPU state: {:?}",
            opcode,
            current_pc,
            instr_count,
            SUCCESS_ADDR,
            cpu.registers
        );

        instr_count += 1;

        cpu.execute_instruction(decoded_instr.unwrap());

        // Check for infinite loop (PC not advancing)
        assert_ne!(
            cpu.registers.program_counter, old_pc,
            "Infinite loop detected at PC ${:04X} after {} instructions\n\
             CPU state: {:?}",
            current_pc, instr_count, cpu.registers
        );

        old_pc = cpu.registers.program_counter;
    }

    // Success! The test reached the success trap (JMP to self at SUCCESS_ADDR)
    eprintln!(
        "Klaus2m5 6502 functional test PASSED after {} instructions",
        instr_count
    );
}

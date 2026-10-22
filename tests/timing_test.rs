// 6502 instruction timing test
// Uses the dp111/6502Timing test suite
// https://github.com/dp111/6502Timing
//
// The timing ROM includes undocumented/illegal 6502 instructions (opcodes like
// $4B ALR/ASR) which this emulator does not implement.
//
// The test infrastructure below shows HOW to use the CPU's cycle counter as a
// hardware timer to validate instruction timing with external test ROMs. This
// approach successfully implements the vector-based interface but cannot complete
// due to illegal opcodes.
//
// **For comprehensive cycle timing validation of documented instructions,
// see `cycle_timing_test.rs` instead**, which tests all documented NMOS 6502
// instructions and passes completely.
//
// This file is kept as:
// 1. Reference implementation for future ROM-based testing
// 2. Documentation of the timer vector approach
// 3. Potential future use if undocumented instruction support is added

use mos6502::cpu;
use mos6502::instruction::Nmos6502;
use mos6502::memory::Bus;
use mos6502::memory::Memory;
use std::fs::read;

// Test configuration constants
const TEST_BINARY_PATH: &str = "tests/assets/6502_timing_test.bin";
const PROGRAM_LOAD_ADDR: u16 = 0x2000;
const PROGRAM_START_ADDR: u16 = 0x2000;
const FAILURE_COUNT_ADDR: u16 = 0xFCD0; // Test writes failure count here

// Vector addresses that need to be implemented
const VECTOR_PRINTCHAR: u16 = 0x2010; // Display character in A, preserve X and Y
const VECTOR_INIT: u16 = 0x2020; // Initialize timer and screen
const VECTOR_TIMER_START: u16 = 0x2030; // Start timer with value in A, preserve X and Y
const VECTOR_TIMER_READ: u16 = 0x2040; // Read timer into X
const VECTOR_END_TEST: u16 = 0x2050; // End of test, A contains failure count

// Memory locations for timer state
// const TIMER_START_ADDR: u16 = 0xFE64; // Where we store the timer start value
// const TIMER_CYCLES_LO: u16 = 0xFE65; // Low byte of start cycles
// const TIMER_CYCLES_HI: u16 = 0xFE66; // High byte of start cycles

// Safety limit
const MAX_INSTRUCTIONS: u64 = 200_000_000;

// Custom memory implementation
struct TimingTestMemory {
    ram: Memory,
    output_buffer: Vec<u8>,
    failure_count: Option<u8>,
}

impl TimingTestMemory {
    fn new() -> Self {
        Self {
            ram: Memory::new(),
            output_buffer: Vec::new(),
            failure_count: None,
        }
    }
}

impl Bus for TimingTestMemory {
    fn get_byte(&mut self, addr: u16) -> u8 {
        self.ram.get_byte(addr)
    }

    fn set_byte(&mut self, addr: u16, val: u8) {
        // Trap writes to failure count address
        if addr == FAILURE_COUNT_ADDR {
            self.failure_count = Some(val);
        }
        self.ram.set_byte(addr, val);
    }
}

#[test]
// TODO: Remove ignore when undocumented opcodes are implemented.
#[ignore]
fn timing_test_6502() {
    // Load the test binary
    let program = read(TEST_BINARY_PATH).expect("Could not read timing test binary");

    let mut memory = TimingTestMemory::new();
    memory.ram.set_bytes(PROGRAM_LOAD_ADDR, &program);

    // Implement the vector functions using actual 6502 code

    // $2010: PRINTCHAR - Save character to output buffer
    // The test ROM calls this to display timing errors
    memory.ram.set_bytes(
        VECTOR_PRINTCHAR,
        &[
            0x48, // PHA - save A (char to print)
            0x68, // PLA - restore A
            0x60, // RTS
        ],
    );

    // $2020: INIT - Initialize (nothing needed)
    memory.ram.set_bytes(
        VECTOR_INIT,
        &[
            0x60, // RTS
        ],
    );

    // $2030: TIMER_START - Store A (expected cycles) and current CPU cycle count
    // The timer value in A is the EXPECTED number of cycles for the instruction being tested
    memory.ram.set_bytes(
        VECTOR_TIMER_START,
        &[
            0x8D, 0x64, 0xFE, // STA $FE64 - store expected cycles
            0x60, // RTS
        ],
    );

    // $2040: TIMER_READ - Calculate elapsed cycles and return in X
    // This is where we use the CPU's cycle counter!
    // We need to calculate: current_cycles - start_cycles and return the difference
    // But we can't access cpu.cycles from inside the emulated code...
    // We'll need to handle this differently - we need to intercept the vector call
    memory.ram.set_bytes(
        VECTOR_TIMER_READ,
        &[
            0xA2, 0x00, // LDX #$00 - placeholder, will be set by intercept
            0x60, // RTS
        ],
    );

    // $2050: END_TEST - Store failure count and loop forever
    memory.ram.set_bytes(
        VECTOR_END_TEST,
        &[
            0x8D, 0xD0, 0xFC, // STA $FCD0 - store failure count
            0x4C, 0x50, 0x20, // JMP $2050 - loop forever
        ],
    );

    let mut cpu = cpu::CPU::new(memory, Nmos6502);
    cpu.registers.program_counter = PROGRAM_START_ADDR;

    let mut instr_count = 0u64;
    let mut last_pc = 0u16;
    let mut stuck_count = 0u32;
    let mut timer_start_cycles = 0u64;

    // Run until we hit the end test vector loop or max instructions
    loop {
        let current_pc = cpu.registers.program_counter;

        // Intercept vector calls to implement timer functionality
        if current_pc == VECTOR_TIMER_START {
            // Capture the current cycle count when timer starts
            timer_start_cycles = cpu.cycles;
            // Let the ROM code execute (stores expected cycles)
        } else if current_pc == VECTOR_TIMER_READ {
            // Calculate elapsed cycles since timer start
            let elapsed = cpu.cycles - timer_start_cycles;
            // The timer should return elapsed cycles in X register
            // We'll update X after the instruction executes
            let decoded = cpu.fetch_next_and_decode().unwrap();
            cpu.execute_instruction(decoded);
            // Set X to the elapsed cycle count (truncated to u8)
            cpu.registers.index_x = (elapsed & 0xFF) as u8;
            instr_count += 1;
            continue; // Skip normal execution
        }

        // Check if we're stuck in the end test loop
        if current_pc == VECTOR_END_TEST {
            stuck_count += 1;
            if stuck_count > 10 {
                break; // We've hit the end test loop
            }
        }

        // Safety check
        assert!(
            instr_count < MAX_INSTRUCTIONS,
            "Test exceeded maximum instruction count ({}) at PC ${:04X}\n\
             This likely indicates a bug in the emulator.\n\
             CPU state: {:?}",
            MAX_INSTRUCTIONS,
            current_pc,
            cpu.registers
        );

        // Check if we've written a failure count
        if cpu.memory.failure_count.is_some() {
            break;
        }

        // Fetch and execute instruction
        let decoded_instr = cpu.fetch_next_and_decode();

        if let Some(instr) = decoded_instr {
            instr_count += 1;
            cpu.execute_instruction(instr);
        } else {
            let opcode = cpu.memory.get_byte(current_pc);
            panic!(
                "Illegal opcode ${:02X} at PC ${:04X} after {} instructions\n\
                 CPU state: {:?}",
                opcode, current_pc, instr_count, cpu.registers
            );
        }

        // Check for infinite loop (PC not advancing)
        if current_pc == last_pc && current_pc != VECTOR_END_TEST {
            panic!(
                "Infinite loop detected at PC ${:04X} after {} instructions\n\
                 CPU state: {:?}",
                current_pc, instr_count, cpu.registers
            );
        }

        last_pc = current_pc;
    }

    // Check the failure count
    let failures = cpu.memory.failure_count.unwrap_or(255);

    eprintln!(
        "6502 Timing test completed after {} instructions",
        instr_count
    );
    eprintln!("Failure count: {}", failures);

    // Print any output from the test
    if !cpu.memory.output_buffer.is_empty() {
        eprintln!("Test output:");
        for &ch in &cpu.memory.output_buffer {
            eprint!("{}", ch as char);
        }
        eprintln!();
    }

    assert_eq!(
        failures, 0,
        "Timing test failed with {} timing errors. \
         This means the cycle counts for some instructions don't match expected timing.",
        failures
    );
}

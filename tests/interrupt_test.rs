// Klaus2m5 6502 interrupt test
// https://github.com/Klaus2m5/6502_65C02_functional_tests
//
// This test validates IRQ and NMI interrupt handling using a feedback register
// at $BFFC that allows the test code to trigger interrupts programmatically.

use mos6502::cpu;
use mos6502::instruction::Nmos6502;
use mos6502::memory::Bus;
use std::fs::read;

// Test configuration constants
const TEST_BINARY_PATH: &str = "tests/assets/6502_interrupt_test.bin";
const PROGRAM_LOAD_ADDR: u16 = 0x0000;
const PROGRAM_START_ADDR: u16 = 0x0400;
const SUCCESS_ADDR: u16 = 0x04CA; // Address of the first "jmp *" success trap

// Interrupt feedback register configuration
const I_PORT: u16 = 0xBFFC; // Feedback port address
const IRQ_BIT: u8 = 0; // Bit number for IRQ feedback
const NMI_BIT: u8 = 1; // Bit number for NMI feedback
const I_FILTER: u8 = 0x7F; // Filter bit 7 (diag stop)

// Safety limit to prevent infinite loops
const MAX_INSTRUCTIONS: u64 = 10_000_000;

/// Memory bus with interrupt feedback register for Klaus2m5 interrupt test
///
/// This implements a special I/O port at $BFFC that allows the test program
/// to trigger IRQ and NMI interrupts by writing to specific bits.
struct InterruptTestMemory {
    ram: [u8; 65536],
    feedback_register: u8,
}

impl InterruptTestMemory {
    fn new() -> Self {
        InterruptTestMemory {
            ram: [0; 65536],
            feedback_register: 0x00, // Start with no interrupts asserted
        }
    }
}

impl Bus for InterruptTestMemory {
    fn get_byte(&mut self, address: u16) -> u8 {
        if address == I_PORT {
            self.feedback_register & I_FILTER
        } else {
            self.ram[address as usize]
        }
    }

    fn set_byte(&mut self, address: u16, value: u8) {
        if address == I_PORT {
            // Writing to feedback register updates interrupt state
            self.feedback_register = value & I_FILTER;
        } else {
            self.ram[address as usize] = value;
        }
    }

    fn set_bytes(&mut self, start: u16, values: &[u8]) {
        let start = start as usize;
        let end = start + values.len();
        self.ram[start..end].copy_from_slice(values);
    }

    fn nmi_pending(&mut self) -> bool {
        // In open collector mode, bit = 1 means interrupt is asserted
        (self.feedback_register & (1 << NMI_BIT)) != 0
    }

    fn irq_pending(&mut self) -> bool {
        // In open collector mode, bit = 1 means interrupt is asserted
        (self.feedback_register & (1 << IRQ_BIT)) != 0
    }
}

#[test]
fn klaus2m5_interrupt_test() {
    // Load the binary file from disk
    let program = read(TEST_BINARY_PATH).expect("Could not read interrupt test binary");

    let mut cpu = cpu::CPU::new(InterruptTestMemory::new(), Nmos6502);

    cpu.memory.set_bytes(PROGRAM_LOAD_ADDR, &program);
    cpu.registers.program_counter = PROGRAM_START_ADDR;

    // Zero out BSS section (uninitialized data)
    // Zero page BSS: $0A - $0F (6 bytes for interrupt save areas)
    for addr in 0x000A..=0x000F {
        cpu.memory.set_byte(addr, 0);
    }
    // Data segment BSS: $0200 - $0203 (4 bytes for test counters and I_src)
    for addr in 0x0200..=0x0203 {
        cpu.memory.set_byte(addr, 0);
    }

    // Run the test
    let mut old_pc = cpu.registers.program_counter;
    let mut instr_count = 0u64;
    let mut pc_history: Vec<(u16, u8, String)> = Vec::new(); // (PC, opcode, registers)

    loop {
        let current_pc = cpu.registers.program_counter;

        // Safety check to prevent infinite loops
        assert!(
            instr_count < MAX_INSTRUCTIONS,
            "Test exceeded maximum instruction count ({}) at PC ${:04X}\n\
             This likely indicates a bug in the emulator causing an infinite loop.\n\
             CPU state: {:?}\n\
             Feedback register: ${:02X}",
            MAX_INSTRUCTIONS,
            current_pc,
            cpu.registers,
            cpu.memory.feedback_register
        );

        // Track execution history (keep last 30 instructions)
        let opcode = cpu.memory.get_byte(current_pc);
        let reg_state = format!("A:{:02X} X:{:02X} Y:{:02X} SP:{:02X} P:{:08b} FB:{:02X}",
                                cpu.registers.accumulator,
                                cpu.registers.index_x,
                                cpu.registers.index_y,
                                cpu.registers.stack_pointer.0,
                                cpu.registers.status.bits(),
                                cpu.memory.feedback_register);

        // Detect interrupt vector jumps and RTI
        let is_interrupt = match current_pc {
            0x0739 => {
                eprintln!("==> NMI HANDLER ENTRY at ${:04X}: {}", current_pc, reg_state);
                true
            }
            0x077D => {
                eprintln!("==> IRQ/BRK HANDLER ENTRY at ${:04X}: {}", current_pc, reg_state);
                true
            }
            _ => false
        };

        // Detect RTI to track interrupt returns
        if opcode == 0x40 {
            eprintln!("<== RTI at ${:04X}, returning to handler or main code", current_pc);
        }

        pc_history.push((current_pc, opcode, reg_state));
        if pc_history.len() > 30 {
            pc_history.remove(0);
        }

        // Track writes to I_src ($0203) and interrupt state
        let old_i_src = cpu.memory.get_byte(0x0203);
        let fb_reg = cpu.memory.feedback_register;
        let has_nmi = cpu.memory.nmi_pending();
        let has_irq = cpu.memory.irq_pending();

        // Detect when we're looping at $06F3 (waiting for I_src != 0)
        if current_pc == 0x06F3 && old_i_src == 0 {
            eprintln!("WARNING: At $06F3, looping while I_src=0, FB=${:02X}, NMI={}, IRQ={}",
                     fb_reg, has_nmi, has_irq);
        }

        // Execute single step (handles interrupts and instruction execution)
        cpu.single_step();

        let new_i_src = cpu.memory.get_byte(0x0203);
        if old_i_src != new_i_src {
            eprintln!("${:04X}: I_src changed from ${:02X} to ${:02X}", current_pc, old_i_src, new_i_src);
        }

        instr_count += 1;

        // Check for infinite loop (PC not advancing)
        // Note: This is expected at the success address, but we break out of the loop before then
        if cpu.registers.program_counter == old_pc {
            // Check if we're stuck in an error trap (jmp * or branch to self)
            let opcode = cpu.memory.get_byte(current_pc);

            // Read some context around the PC for debugging
            let context_start = current_pc.saturating_sub(5);
            let mut context = Vec::new();
            for addr in context_start..current_pc.saturating_add(10) {
                context.push(format!("{:02X}", cpu.memory.get_byte(addr)));
            }

            if opcode == 0x4C {
                // JMP absolute
                let target_lo = cpu.memory.get_byte(current_pc.wrapping_add(1));
                let target_hi = cpu.memory.get_byte(current_pc.wrapping_add(2));
                let target = u16::from_le_bytes([target_lo, target_hi]);
                if target == current_pc {
                    panic!(
                        "Test failed: Stuck in error trap at PC ${:04X} after {} instructions\n\
                         This is likely a 'jmp *' error trap in the test.\n\
                         CPU state: {:?}\n\
                         Feedback register: ${:02X}\n\
                         Memory context: {}",
                        current_pc, instr_count, cpu.registers, cpu.memory.feedback_register,
                        context.join(" ")
                    );
                }
            } else if opcode == 0xF0 || opcode == 0xD0 {
                // BEQ or BNE to self
                let offset = cpu.memory.get_byte(current_pc.wrapping_add(1));
                if offset == 0xFE {
                    // Branch to self (-2 bytes)
                    // Check I_src at $0203 to see what interrupt was expected
                    let i_src = cpu.memory.get_byte(0x0203);

                    eprintln!("\n=== Last 30 instructions before trap ===");
                    for (pc, opc, regs) in &pc_history {
                        eprintln!("${:04X}: {:02X}  {}", pc, opc, regs);
                    }
                    eprintln!("==========================================\n");

                    panic!(
                        "Test failed: Stuck in error trap at PC ${:04X} after {} instructions\n\
                         This is likely a 'beq *' or 'bne *' error trap in the test.\n\
                         CPU state: {:?}\n\
                         Feedback register: ${:02X}\n\
                         I_src (expected interrupts): ${:02X}\n\
                         Memory context: {}",
                        current_pc, instr_count, cpu.registers, cpu.memory.feedback_register,
                        i_src, context.join(" ")
                    );
                }
            }
        }

        old_pc = cpu.registers.program_counter;
    }

    // Success! The test reached the success trap
    eprintln!(
        "Klaus2m5 6502 interrupt test PASSED after {} instructions",
        instr_count
    );
}

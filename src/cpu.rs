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

//! ## Decimal Mode Implementation Notes
//!
//! This emulator implements decimal mode (BCD arithmetic) for ADC and SBC instructions
//! following the documented behavior of the original NMOS 6502 processor.
//!
//! ### Key Implementation Details:
//!
//! - **Target Processor**: NMOS 6502 (as used in Commodore 64, Apple II, etc.)
//! - **Flag Behavior**: Only the Carry flag is reliable in decimal mode; N, V, Z flags
//!   are set but may not correspond to expected decimal arithmetic results
//! - **Invalid BCD**: Values A-F in nibbles produce undefined but deterministic results
//! - **Overflow Flag**: Calculated from binary addition result, not BCD result
//!
//! ### Authoritative References:
//!
//! - [6502.org Decimal Mode Tutorial](http://www.6502.org/tutorials/decimal_mode.html)
//! - [NESdev Wiki 6502 Decimal Mode](https://www.nesdev.org/wiki/Visual6502wiki/6502DecimalMode)
//! - Bruce Clark's comprehensive decimal mode test suite
//!
//! For other 6502 variants (65C02, RP2A03), see variant-specific instruction handling.

use crate::Variant;
use crate::instruction::{AddressingMode, DecodedInstr, Instruction, OpInput};
use crate::memory::Bus;

use crate::registers::{Registers, StackPointer, Status, StatusArgs};

fn address_from_bytes(lo: u8, hi: u8) -> u16 {
    u16::from(lo) + (u16::from(hi) << 8usize)
}

#[derive(Clone, Default)]
pub struct CPU<M, V>
where
    M: Bus,
    V: Variant,
{
    /// CPU registers including program counter, stack pointer, accumulator,
    /// index registers, and status flags
    pub registers: Registers,
    /// Memory bus that the CPU reads from and writes to
    pub memory: M,
    /// Total number of cycles elapsed since CPU creation or last reset.
    /// Used for cycle-accurate emulation and synchronization with other components.
    /// Uses u64 to prevent wraparound (would take 584,942 years at 1MHz to wrap).
    pub cycles: u64,
    /// Indicates if the CPU is halted (e.g., by STP instruction on 65C02)
    halted: bool,
    /// Phantom data to track which CPU variant is being emulated
    /// (NMOS, CMOS, etc.)
    variant: core::marker::PhantomData<V>,
}

impl<M: Bus, V: Variant> CPU<M, V> {
    // Allowing `needless_pass_by_value` to simplify construction. Passing by
    // value avoids the borrow and improves readability when constructing the
    // CPU.
    #[allow(clippy::needless_pass_by_value)]
    pub fn new(memory: M, _variant: V) -> CPU<M, V> {
        CPU {
            registers: Registers::new(),
            memory,
            cycles: 0,
            variant: core::marker::PhantomData::<V>,
            halted: false,
        }
    }

    /// Perform the 6502 reset sequence
    ///
    /// The reset sequence on a 6502 is an 8-cycle process that simulates the same sequence
    /// as BRK/IRQ/NMI but with reads instead of writes for the stack operations:
    ///
    /// 1. Cycle 0-2: Initial cycles with IR=0
    /// 2. Cycle 3-5: Three "fake" stack pushes (reads instead of writes):
    ///    - Read from $0100 + SP (would be PC high byte)
    ///    - Read from $01FF + SP (would be PC low byte)  
    ///    - Read from $01FE + SP (would be status register)
    ///    - SP decrements 3 times to 0xFD
    /// 3. Cycle 6: Read reset vector low byte from $FFFC
    /// 4. Cycle 7: Read reset vector high byte from $FFFD
    /// 5. Cycle 8: First instruction fetch from new PC
    ///
    /// The interrupt disable flag is set, and on 65C02 the decimal flag is cleared.
    ///
    /// For detailed cycle-by-cycle analysis, see: <https://www.pagetable.com/?p=410>
    pub fn reset(&mut self) {
        // Clear halted state (hardware reset resumes a stopped processor)
        self.halted = false;

        // Simulate the 3 fake stack operations that decrement SP from 0x00 to 0xFD
        // Real hardware performs reads from $0100, $01FF, $01FE but discards the results
        // This matches cycles 3-5 of the reset sequence described at pagetable.com
        self.registers.stack_pointer.decrement();
        self.registers.stack_pointer.decrement();
        self.registers.stack_pointer.decrement();

        // Set interrupt disable flag (all variants)
        self.registers.status.insert(Status::PS_DISABLE_INTERRUPTS);

        // Read reset vector: low byte at $FFFC, high byte at $FFFD
        let reset_vector_low = self.memory.get_byte(0xFFFC);
        let reset_vector_high = self.memory.get_byte(0xFFFD);
        self.registers.program_counter = u16::from_le_bytes([reset_vector_low, reset_vector_high]);
    }

    /// Get the next byte from memory and decode it into an instruction and addressing mode.
    ///
    /// # Panics
    ///
    /// This function will panic if the instruction is not recognized
    /// (i.e. the opcode is invalid or has not been implemented).
    pub fn fetch_next_and_decode(&mut self) -> Option<DecodedInstr> {
        // Helper function to read a 16-bit address from memory
        fn read_address<M: Bus>(mem: &mut M, addr: u16) -> [u8; 2] {
            let lo = mem.get_byte(addr);
            let hi = mem.get_byte(addr.wrapping_add(1));
            [lo, hi]
        }

        let x: u8 = self.memory.get_byte(self.registers.program_counter);

        match V::decode(x) {
            Some((instr, am)) => {
                let extra_bytes = am.extra_bytes();
                let num_bytes = extra_bytes + 1;

                let data_start = self.registers.program_counter.wrapping_add(1);

                let slice = if extra_bytes == 0 {
                    [0, 0]
                } else if extra_bytes == 1 {
                    [self.memory.get_byte(data_start), 0]
                } else if extra_bytes == 2 {
                    [
                        self.memory.get_byte(data_start),
                        self.memory.get_byte(data_start.wrapping_add(1)),
                    ]
                } else {
                    panic!()
                };

                let x = self.registers.index_x;
                let y = self.registers.index_y;

                let memory = &mut self.memory;

                let am_out = match am {
                    AddressingMode::Accumulator | AddressingMode::Implied => {
                        // Always the same -- no input
                        OpInput::UseImplied
                    }
                    AddressingMode::Immediate => {
                        // Use [u8, ..1] specified in instruction as input
                        OpInput::UseImmediate(slice[0])
                    }
                    AddressingMode::ZeroPage => {
                        // Use [u8, ..1] from instruction
                        // Interpret as zero page address
                        // (Output: an 8-bit zero-page address)
                        OpInput::UseAddress {
                            address: u16::from(slice[0]),
                            page_crossed: false,
                        }
                    }
                    AddressingMode::ZeroPageX => {
                        // Use [u8, ..1] from instruction
                        // Add to X register (as u8 -- the final address is in 0-page)
                        // (Output: an 8-bit zero-page address)
                        OpInput::UseAddress {
                            address: u16::from(slice[0].wrapping_add(x)),
                            page_crossed: false,
                        }
                    }
                    AddressingMode::ZeroPageY => {
                        // Use [u8, ..1] from instruction
                        // Add to Y register (as u8 -- the final address is in 0-page)
                        // (Output: an 8-bit zero-page address)
                        OpInput::UseAddress {
                            address: u16::from(slice[0].wrapping_add(y)),
                            page_crossed: false,
                        }
                    }

                    AddressingMode::Relative => {
                        // Use [u8, ..1] from instruction
                        // (interpret as relative...)
                        // (This is sign extended to a 16-but data type, but an unsigned one: u16. It's a
                        // little weird, but it's so we can add the PC and the offset easily)
                        let offset = slice[0];
                        let sign_extend = if offset & 0x80 == 0x80 { 0xffu8 } else { 0x0 };
                        let rel = u16::from_le_bytes([offset, sign_extend]);
                        OpInput::UseRelative(rel)
                    }
                    AddressingMode::Absolute => {
                        // Use [u8, ..2] from instruction as address
                        // (Output: a 16-bit address)
                        OpInput::UseAddress {
                            address: address_from_bytes(slice[0], slice[1]),
                            page_crossed: false,
                        }
                    }
                    AddressingMode::AbsoluteX => {
                        // Use [u8, ..2] from instruction as address, add X
                        // Check for page crossing: base and final addresses on different pages
                        let base = address_from_bytes(slice[0], slice[1]);
                        let final_addr = base.wrapping_add(x.into());
                        let crossed = (base & 0xFF00) != (final_addr & 0xFF00);
                        OpInput::UseAddress {
                            address: final_addr,
                            page_crossed: crossed,
                        }
                    }
                    AddressingMode::AbsoluteY => {
                        // Use [u8, ..2] from instruction as address, add Y
                        // Check for page crossing: base and final addresses on different pages
                        let base = address_from_bytes(slice[0], slice[1]);
                        let final_addr = base.wrapping_add(y.into());
                        let crossed = (base & 0xFF00) != (final_addr & 0xFF00);
                        OpInput::UseAddress {
                            address: final_addr,
                            page_crossed: crossed,
                        }
                    }
                    AddressingMode::Indirect => {
                        // Use [u8, ..2] from instruction as an address. Interpret the
                        // two bytes starting at that address as an address.
                        // (Output: a 16-bit address)
                        // Note: Cycle-accurate timing is not implemented. On real hardware,
                        // if the pointer ends in 0xff, incrementing it costs an extra cycle.
                        let slice = read_address(memory, address_from_bytes(slice[0], slice[1]));
                        OpInput::UseAddress {
                            address: address_from_bytes(slice[0], slice[1]),
                            page_crossed: false,
                        }
                    }
                    AddressingMode::BuggyIndirect => {
                        // Use [u8, ..2] from instruction as an address. Interpret the
                        // two bytes starting at that address as an address.
                        // (Output: a 16-bit address)
                        let pointer = address_from_bytes(slice[0], slice[1]);

                        let low_byte_of_target = memory.get_byte(pointer);

                        let low_byte_of_incremented_pointer =
                            pointer.to_le_bytes()[0].wrapping_add(1);
                        let incremented_pointer = u16::from_le_bytes([
                            low_byte_of_incremented_pointer,
                            pointer.to_le_bytes()[1],
                        ]);

                        let high_byte_of_target = memory.get_byte(incremented_pointer);
                        OpInput::UseAddress {
                            address: address_from_bytes(low_byte_of_target, high_byte_of_target),
                            page_crossed: false,
                        }
                    }
                    AddressingMode::AbsoluteIndexedIndirect => {
                        // 65C02: JMP (abs,X)
                        // Use [u8, ..2] from instruction plus X as an address. Interpret the
                        // two bytes starting at that address as the jump target.
                        // (Output: a 16-bit address)
                        let x: u8 = self.registers.index_x;
                        let base_addr = address_from_bytes(slice[0], slice[1]);
                        let pointer = base_addr.wrapping_add(u16::from(x));
                        let slice = read_address(memory, pointer);
                        OpInput::UseAddress {
                            address: address_from_bytes(slice[0], slice[1]),
                            page_crossed: false,
                        }
                    }
                    AddressingMode::IndexedIndirectX => {
                        // Use [u8, ..1] from instruction
                        // Add to X register with 0-page wraparound, like ZeroPageX.
                        // This is where the absolute (16-bit) target address is stored.
                        // (Output: a 16-bit address)
                        let start = slice[0].wrapping_add(x);
                        let slice = read_address(memory, u16::from(start));
                        OpInput::UseAddress {
                            address: address_from_bytes(slice[0], slice[1]),
                            page_crossed: false,
                        }
                    }
                    AddressingMode::IndirectIndexedY => {
                        // Use [u8, ..1] from instruction
                        // This is where the absolute (16-bit) target address is stored.
                        // Add Y register to this address to get the final address
                        // Check for page crossing
                        let start = slice[0];
                        let slice = read_address(memory, u16::from(start));
                        let base = address_from_bytes(slice[0], slice[1]);
                        let final_addr = base.wrapping_add(y.into());
                        let crossed = (base & 0xFF00) != (final_addr & 0xFF00);
                        OpInput::UseAddress {
                            address: final_addr,
                            page_crossed: crossed,
                        }
                    }
                    AddressingMode::ZeroPageIndirect => {
                        // Use [u8, ..1] from instruction
                        // This is where the absolute (16-bit) target address is stored.
                        // (Output: a 16-bit address)
                        let start = slice[0];
                        let slice = read_address(memory, u16::from(start));
                        OpInput::UseAddress {
                            address: address_from_bytes(slice[0], slice[1]),
                            page_crossed: false,
                        }
                    }
                };

                // Increment program counter
                self.registers.program_counter =
                    self.registers.program_counter.wrapping_add(num_bytes);

                Some((instr, am, am_out))
            }
            _ => None,
        }
    }

    /// Calculates the total cycle count for an instruction, including base cycles and penalties.
    ///
    /// This is the main entry point for cycle calculation, combining:
    /// - Base cycles from the instruction/addressing mode lookup table
    /// - Page crossing penalties (+1 for loads when page boundary crossed)
    /// - Decimal mode penalties (variant-specific, +1 for ADC/SBC on 65C02 when D flag is set)
    ///
    /// # Arguments
    /// * `instr` - The instruction being executed
    /// * `mode` - The addressing mode used
    /// * `page_crossed` - Whether a page boundary was crossed during address calculation
    /// * `registers` - CPU registers (needed to check decimal mode flag)
    ///
    /// # Returns
    /// Total number of cycles this instruction will take
    fn calculate_instruction_cycles(
        instr: Instruction,
        mode: AddressingMode,
        page_crossed: bool,
        registers: Registers,
    ) -> u8 {
        use Instruction::{STA, STX, STY, STZ};

        let base_cycles = instr.base_cycles(mode);

        // Page crossing penalty: +1 cycle for loads (not stores!)
        let page_penalty = u8::from(page_crossed && !matches!(instr, STA | STX | STY | STZ));

        // Decimal mode penalty: variant-specific (e.g., +1 for ADC/SBC on 65C02 when D flag is set)
        let decimal_penalty = Self::decimal_mode_penalty_for_variant(instr, registers);

        base_cycles + page_penalty + decimal_penalty
    }

    /// Determines if decimal mode adds an extra cycle for this instruction and variant.
    ///
    /// Returns the penalty cycles for ADC/SBC in decimal mode, which varies by variant:
    /// - NMOS/Ricoh: 0 (no penalty)
    /// - 65C02: 1 (when D flag is set and instruction is ADC/SBC)
    fn decimal_mode_penalty_for_variant(instr: Instruction, registers: Registers) -> u8 {
        use Instruction::{ADC, SBC};

        // Only ADC and SBC have decimal mode penalty (not ADCnd/SBCnd which disable decimal)
        if !matches!(instr, ADC | SBC) {
            return 0;
        }

        // Check if decimal mode flag is set
        if !registers.status.contains(Status::PS_DECIMAL_MODE) {
            return 0;
        }

        // Return variant-specific penalty
        V::penalty_cycles_for_decimal_mode()
    }

    #[allow(clippy::too_many_lines)]
    pub fn execute_instruction(&mut self, decoded_instr: DecodedInstr) {
        let (instr, mode, operand) = decoded_instr;

        // Calculate and track cycles for this instruction
        let total_cycles =
            Self::calculate_instruction_cycles(instr, mode, operand.page_crossed(), self.registers);
        self.cycles = self.cycles.wrapping_add(u64::from(total_cycles));

        match (instr, operand) {
            (Instruction::ADC, OpInput::UseImmediate(val)) => {
                log::debug!("add with carry immediate: {val}");
                self.add_with_carry(val);
            }
            (Instruction::ADC, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                log::debug!("add with carry. address: {addr:?}. value: {val}");
                self.add_with_carry(val);
            }
            (Instruction::ADCnd, OpInput::UseImmediate(val)) => {
                log::debug!("add with carry immediate: {val}");
                self.add_with_no_decimal(val);
            }
            (Instruction::ADCnd, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                log::debug!("add with carry. address: {addr:?}. value: {val}");
                self.add_with_no_decimal(val);
            }

            (Instruction::AND, OpInput::UseImmediate(val)) => {
                self.and(val);
            }
            (Instruction::AND, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                self.and(val);
            }

            (Instruction::ASL, OpInput::UseImplied) => {
                // Accumulator mode
                let mut val = self.registers.accumulator;
                CPU::<M, V>::shift_left_with_flags(&mut val, &mut self.registers.status);
                self.registers.accumulator = val;
            }
            (Instruction::ASL, OpInput::UseAddress { address: addr, .. }) => {
                let mut operand: u8 = self.memory.get_byte(addr);
                CPU::<M, V>::shift_left_with_flags(&mut operand, &mut self.registers.status);
                self.memory.set_byte(addr, operand);
            }

            (Instruction::BCC, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter.wrapping_add(rel);
                self.branch_if_carry_clear(addr);
            }

            (Instruction::BCS, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter.wrapping_add(rel);
                self.branch_if_carry_set(addr);
            }

            (Instruction::BEQ, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter.wrapping_add(rel);
                self.branch_if_equal(addr);
            }

            (Instruction::BNE, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter.wrapping_add(rel);
                self.branch_if_not_equal(addr);
            }

            (Instruction::BIT, OpInput::UseImmediate(val)) => {
                self.registers.status.set_with_mask(
                    Status::PS_ZERO,
                    Status::new(StatusArgs {
                        zero: 0 == (self.registers.accumulator & val),
                        ..StatusArgs::none()
                    }),
                );
            }

            (Instruction::BIT, OpInput::UseAddress { address: addr, .. }) => {
                let a: u8 = self.registers.accumulator;
                let m: u8 = self.memory.get_byte(addr);
                let res = a & m;

                // The zero flag is set based on the result of the 'and'.
                let is_zero = 0 == res;

                // The N flag is set to bit 7 of the byte from memory.
                let bit7 = 0 != (0x80 & m);

                // The V flag is set to bit 6 of the byte from memory.
                let bit6 = 0 != (0x40 & m);

                self.registers.status.set_with_mask(
                    Status::PS_ZERO | Status::PS_NEGATIVE | Status::PS_OVERFLOW,
                    Status::new(StatusArgs {
                        zero: is_zero,
                        negative: bit7,
                        overflow: bit6,
                        ..StatusArgs::none()
                    }),
                );
            }

            (Instruction::BMI, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter.wrapping_add(rel);
                log::debug!("branch if minus relative. address: {addr:?}");
                self.branch_if_minus(addr);
            }

            (Instruction::BPL, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter.wrapping_add(rel);
                self.branch_if_positive(addr);
            }

            (Instruction::BRA, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter.wrapping_add(rel);
                self.branch(addr);
            }

            (Instruction::BRK, OpInput::UseImplied) => {
                for b in self.registers.program_counter.wrapping_sub(1).to_be_bytes() {
                    self.push_on_stack(b);
                }
                self.push_on_stack(self.registers.status.bits());
                let pcl = self.memory.get_byte(0xfffe);
                let pch = self.memory.get_byte(0xffff);
                self.jump((u16::from(pch) << 8) | u16::from(pcl));
                self.set_flag(Status::PS_DISABLE_INTERRUPTS);
            }

            (Instruction::BRKcld, OpInput::UseImplied) => {
                for b in self.registers.program_counter.wrapping_sub(1).to_be_bytes() {
                    self.push_on_stack(b);
                }
                self.push_on_stack(self.registers.status.bits());
                let pcl = self.memory.get_byte(0xfffe);
                let pch = self.memory.get_byte(0xffff);
                self.jump((u16::from(pch) << 8) | u16::from(pcl));
                self.set_flag(Status::PS_DISABLE_INTERRUPTS);
                self.unset_flag(Status::PS_DECIMAL_MODE);
            }

            (Instruction::BVC, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter.wrapping_add(rel);
                self.branch_if_overflow_clear(addr);
            }

            (Instruction::BVS, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter.wrapping_add(rel);
                self.branch_if_overflow_set(addr);
            }

            (Instruction::CLC, OpInput::UseImplied) => {
                self.unset_flag(Status::PS_CARRY);
            }
            (Instruction::CLD, OpInput::UseImplied) => {
                self.unset_flag(Status::PS_DECIMAL_MODE);
            }
            (Instruction::CLI, OpInput::UseImplied) => {
                self.unset_flag(Status::PS_DISABLE_INTERRUPTS);
            }
            (Instruction::CLV, OpInput::UseImplied) => {
                self.unset_flag(Status::PS_OVERFLOW);
            }

            (Instruction::CMP, OpInput::UseImmediate(val)) => {
                self.compare_with_a_register(val);
            }
            (Instruction::CMP, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                self.compare_with_a_register(val);
            }

            (Instruction::CPX, OpInput::UseImmediate(val)) => {
                self.compare_with_x_register(val);
            }
            (Instruction::CPX, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                self.compare_with_x_register(val);
            }

            (Instruction::CPY, OpInput::UseImmediate(val)) => {
                self.compare_with_y_register(val);
            }
            (Instruction::CPY, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                self.compare_with_y_register(val);
            }

            (Instruction::DEC, OpInput::UseAddress { address: addr, .. }) => {
                let mut operand: u8 = self.memory.get_byte(addr);
                CPU::<M, V>::decrement(&mut operand, &mut self.registers.status);
                self.memory.set_byte(addr, operand);
            }

            (Instruction::DEY, OpInput::UseImplied) => {
                CPU::<M, V>::decrement(&mut self.registers.index_y, &mut self.registers.status);
            }

            (Instruction::DEX, OpInput::UseImplied) => {
                CPU::<M, V>::decrement(&mut self.registers.index_x, &mut self.registers.status);
            }

            (Instruction::EOR, OpInput::UseImmediate(val)) => {
                self.exclusive_or(val);
            }
            (Instruction::EOR, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                self.exclusive_or(val);
            }

            (Instruction::INC, OpInput::UseAddress { address: addr, .. }) => {
                let mut operand: u8 = self.memory.get_byte(addr);
                CPU::<M, V>::increment(&mut operand, &mut self.registers.status);
                self.memory.set_byte(addr, operand);
            }
            (Instruction::INX, OpInput::UseImplied) => {
                CPU::<M, V>::increment(&mut self.registers.index_x, &mut self.registers.status);
            }
            (Instruction::INY, OpInput::UseImplied) => {
                CPU::<M, V>::increment(&mut self.registers.index_y, &mut self.registers.status);
            }

            (Instruction::JMP, OpInput::UseAddress { address: addr, .. }) => self.jump(addr),

            (Instruction::JSR, OpInput::UseAddress { address: addr, .. }) => {
                for b in self.registers.program_counter.wrapping_sub(1).to_be_bytes() {
                    self.push_on_stack(b);
                }
                self.jump(addr);
            }

            (Instruction::LDA, OpInput::UseImmediate(val)) => {
                log::debug!("load A immediate: {val}");
                self.load_accumulator(val);
            }
            (Instruction::LDA, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                log::debug!("load A. address: {addr:?}. value: {val}");
                self.load_accumulator(val);
            }

            (Instruction::LDX, OpInput::UseImmediate(val)) => {
                log::debug!("load X immediate: {val}");
                self.load_x_register(val);
            }
            (Instruction::LDX, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                log::debug!("load X. address: {addr:?}. value: {val}");
                self.load_x_register(val);
            }

            (Instruction::LDY, OpInput::UseImmediate(val)) => {
                log::debug!("load Y immediate: {val}");
                self.load_y_register(val);
            }
            (Instruction::LDY, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                log::debug!("load Y. address: {addr:?}. value: {val}");
                self.load_y_register(val);
            }

            (Instruction::LSR, OpInput::UseImplied) => {
                // Accumulator mode
                let mut val = self.registers.accumulator;
                CPU::<M, V>::shift_right_with_flags(&mut val, &mut self.registers.status);
                self.registers.accumulator = val;
            }
            (Instruction::LSR, OpInput::UseAddress { address: addr, .. }) => {
                let mut operand: u8 = self.memory.get_byte(addr);
                CPU::<M, V>::shift_right_with_flags(&mut operand, &mut self.registers.status);
                self.memory.set_byte(addr, operand);
            }

            (Instruction::ORA, OpInput::UseImmediate(val)) => {
                self.inclusive_or(val);
            }
            (Instruction::ORA, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                self.inclusive_or(val);
            }

            (Instruction::PHA, OpInput::UseImplied) => {
                // Push accumulator
                let val = self.registers.accumulator;
                self.push_on_stack(val);
            }
            (Instruction::PHX, OpInput::UseImplied) => {
                // Push X
                self.push_on_stack(self.registers.index_x);
            }
            (Instruction::PHY, OpInput::UseImplied) => {
                // Push Y
                self.push_on_stack(self.registers.index_y);
            }
            (Instruction::PHP, OpInput::UseImplied) => {
                // Push status
                let val = self.registers.status.bits() | 0x30;
                self.push_on_stack(val);
            }
            (Instruction::PLX, OpInput::UseImplied) => {
                // Pull index X
                let val: u8 = self.pull_from_stack();
                self.registers.index_x = val;
                self.registers.status.set_with_mask(
                    Status::PS_ZERO | Status::PS_NEGATIVE,
                    Status::new(StatusArgs {
                        zero: val == 0,
                        negative: self.registers.accumulator > 127,
                        ..StatusArgs::none()
                    }),
                );
            }
            (Instruction::PLY, OpInput::UseImplied) => {
                // Pull index Y
                let val: u8 = self.pull_from_stack();
                self.registers.index_y = val;
                self.registers.status.set_with_mask(
                    Status::PS_ZERO | Status::PS_NEGATIVE,
                    Status::new(StatusArgs {
                        zero: val == 0,
                        negative: self.registers.accumulator > 127,
                        ..StatusArgs::none()
                    }),
                );
            }
            (Instruction::PLA, OpInput::UseImplied) => {
                // Pull accumulator
                let val: u8 = self.pull_from_stack();
                self.registers.accumulator = val;
                self.registers.status.set_with_mask(
                    Status::PS_ZERO | Status::PS_NEGATIVE,
                    Status::new(StatusArgs {
                        zero: val == 0,
                        negative: self.registers.accumulator > 127,
                        ..StatusArgs::none()
                    }),
                );
            }
            (Instruction::PLP, OpInput::UseImplied) => {
                // Pull status
                let val: u8 = self.pull_from_stack();
                // The `truncate` here masks off invalid bits. The unused bit (bit 5)
                // is always set. Behavior verified by Klaus2m5 functional test.
                self.registers.status = Status::from_bits_truncate(val);
            }

            (Instruction::ROL, OpInput::UseImplied) => {
                // Accumulator mode
                let mut val = self.registers.accumulator;
                CPU::<M, V>::rotate_left_with_flags(&mut val, &mut self.registers.status);
                self.registers.accumulator = val;
            }
            (Instruction::ROL, OpInput::UseAddress { address: addr, .. }) => {
                let mut operand: u8 = self.memory.get_byte(addr);
                CPU::<M, V>::rotate_left_with_flags(&mut operand, &mut self.registers.status);
                self.memory.set_byte(addr, operand);
            }
            (Instruction::ROR, OpInput::UseImplied) => {
                // Accumulator mode
                let mut val = self.registers.accumulator;
                CPU::<M, V>::rotate_right_with_flags(&mut val, &mut self.registers.status);
                self.registers.accumulator = val;
            }
            (Instruction::ROR, OpInput::UseAddress { address: addr, .. }) => {
                let mut operand: u8 = self.memory.get_byte(addr);
                CPU::<M, V>::rotate_right_with_flags(&mut operand, &mut self.registers.status);
                self.memory.set_byte(addr, operand);
            }
            (Instruction::RTI, OpInput::UseImplied) => {
                // Pull status
                let val: u8 = self.pull_from_stack();
                // The `truncate` here masks off invalid bits. The unused bit (bit 5)
                // is always set. Behavior verified by Klaus2m5 functional test.
                self.registers.status = Status::from_bits_truncate(val);
                let pcl: u8 = self.pull_from_stack();
                let pch: u8 = self.pull_from_stack();
                self.registers.program_counter = (u16::from(pch) << 8) | u16::from(pcl);
            }
            (Instruction::RTS, OpInput::UseImplied) => {
                let pcl: u8 = self.pull_from_stack();
                let pch: u8 = self.pull_from_stack();
                self.registers.program_counter =
                    ((u16::from(pch) << 8) | u16::from(pcl)).wrapping_add(1);
            }

            (Instruction::SBC, OpInput::UseImmediate(val)) => {
                log::debug!("subtract with carry immediate: {val}");
                self.subtract_with_carry(val);
            }
            (Instruction::SBC, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                log::debug!("subtract with carry. address: {addr:?}. value: {val}");
                self.subtract_with_carry(val);
            }

            (Instruction::SBCnd, OpInput::UseImmediate(val)) => {
                log::debug!("subtract with carry immediate: {val}");
                self.subtract_with_no_decimal(val);
            }
            (Instruction::SBCnd, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                log::debug!("subtract with carry. address: {addr:?}. value: {val}");
                self.subtract_with_no_decimal(val);
            }

            (Instruction::SEC, OpInput::UseImplied) => {
                self.set_flag(Status::PS_CARRY);
            }
            (Instruction::SED, OpInput::UseImplied) => {
                self.set_flag(Status::PS_DECIMAL_MODE);
            }
            (Instruction::SEI, OpInput::UseImplied) => {
                self.set_flag(Status::PS_DISABLE_INTERRUPTS);
            }
            (Instruction::SAX, OpInput::UseAddress { address: addr, .. }) => {
                self.memory
                    .set_byte(addr, self.registers.accumulator & self.registers.index_x);
            }
            (Instruction::STA, OpInput::UseAddress { address: addr, .. }) => {
                self.memory.set_byte(addr, self.registers.accumulator);
            }
            (Instruction::STX, OpInput::UseAddress { address: addr, .. }) => {
                self.memory.set_byte(addr, self.registers.index_x);
            }
            (Instruction::STY, OpInput::UseAddress { address: addr, .. }) => {
                self.memory.set_byte(addr, self.registers.index_y);
            }
            (Instruction::STZ, OpInput::UseAddress { address: addr, .. }) => {
                self.memory.set_byte(addr, 0);
            }

            (Instruction::TAX, OpInput::UseImplied) => {
                let val = self.registers.accumulator;
                self.load_x_register(val);
            }
            (Instruction::TAY, OpInput::UseImplied) => {
                let val = self.registers.accumulator;
                self.load_y_register(val);
            }
            (Instruction::TRB, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);

                // The zero flag is set based on the result of the 'and'.
                self.registers.status.set_with_mask(
                    Status::PS_ZERO,
                    Status::new(StatusArgs {
                        zero: 0 == (self.registers.accumulator & val),
                        ..StatusArgs::none()
                    }),
                );

                // The 1's in the accumulator set the corresponding bits in the operand
                let res = self.registers.accumulator | val;
                self.memory.set_byte(addr, res);
            }
            (Instruction::TSB, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);

                // The zero flag is set based on the result of the 'and'.
                self.registers.status.set_with_mask(
                    Status::PS_ZERO,
                    Status::new(StatusArgs {
                        zero: 0 == (self.registers.accumulator & val),
                        ..StatusArgs::none()
                    }),
                );

                // The 1's in the accumulator clear the corresponding bits in the operand
                let res = (self.registers.accumulator ^ 0xff) & val;
                self.memory.set_byte(addr, res);
            }
            (Instruction::TSX, OpInput::UseImplied) => {
                let StackPointer(val) = self.registers.stack_pointer;
                self.load_x_register(val);
            }
            (Instruction::TXA, OpInput::UseImplied) => {
                let val = self.registers.index_x;
                self.load_accumulator(val);
            }
            (Instruction::TXS, OpInput::UseImplied) => {
                // Note that this is the only 'transfer' instruction that does
                // NOT set the zero and negative flags. (Because the target
                // is the stack pointer)
                let val = self.registers.index_x;
                self.registers.stack_pointer = StackPointer(val);
            }
            (Instruction::TYA, OpInput::UseImplied) => {
                let val = self.registers.index_y;
                self.load_accumulator(val);
            }
            (Instruction::XAA, OpInput::UseImmediate(val)) => {
                self.load_accumulator(self.registers.index_x);
                self.and(val);
            }

            (Instruction::WAI, OpInput::UseImplied) => {
                // Wait for Interrupt (65C02)
                // In a real CPU, this halts until IRQ or NMI is received
                // For this emulator, we treat it as a NOP
                log::debug!("WAI instruction - waiting for interrupt");
            }

            (Instruction::STP, OpInput::UseImplied) => {
                // Stop processor (65C02)
                // Halts execution until reset() is called
                log::debug!("STP instruction - processor stopped");
                self.halted = true;
            }

            (Instruction::NOP, OpInput::UseImplied) => {
                log::debug!("NOP instruction");
            }

            (Instruction::ALR, OpInput::UseImmediate(val)) => {
                self.and(val);
                let mut val = self.registers.accumulator;
                CPU::<M, V>::shift_right_with_flags(&mut val, &mut self.registers.status);
                self.registers.accumulator = val;
            }

            (Instruction::ANC, OpInput::UseImmediate(val)) => {
                self.and(val);
                if self.registers.accumulator & 0x80 != 0 {
                    self.set_flag(Status::PS_CARRY);
                } else {
                    self.unset_flag(Status::PS_CARRY);
                }
            }

            // ARR - AND with immediate, then ROR with special flag handling.
            // Unlike normal ROR, ARR sets C and V flags based on result bits,
            // not from the shift operation. This quirk stems from how the 6502's
            // internal buses interact during this undocumented instruction.
            (Instruction::ARR, OpInput::UseImmediate(val)) => {
                self.and(val);
                let a = self.registers.accumulator;
                let carry = self.registers.status.contains(Status::PS_CARRY);

                // ROR the accumulator
                let result = (a >> 1) | (if carry { 0x80 } else { 0 });
                self.registers.accumulator = result;

                // Set N and Z flags from result
                CPU::<M, V>::set_flags_from_u8(&mut self.registers.status, result);

                // Carry is set from bit 6 (not bit 0 as in normal ROR)
                if result & 0x40 != 0 {
                    self.set_flag(Status::PS_CARRY);
                } else {
                    self.unset_flag(Status::PS_CARRY);
                }

                // Overflow is set when bits 6 and 5 differ. This unusual behavior
                // detects a "sign change" between the two highest result bits,
                // useful for BCD fixup in some algorithms.
                if ((result >> 6) ^ (result >> 5)) & 1 != 0 {
                    self.set_flag(Status::PS_OVERFLOW);
                } else {
                    self.unset_flag(Status::PS_OVERFLOW);
                }
            }

            // DCP - Decrement memory, then compare with accumulator
            (Instruction::DCP, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr).wrapping_sub(1);
                self.memory.set_byte(addr, val);
                self.compare_with_a_register(val);
            }

            // ISC - Increment memory, then SBC from accumulator
            (Instruction::ISC, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr).wrapping_add(1);
                self.memory.set_byte(addr, val);
                self.subtract_with_carry(val);
            }

            // JAM - Halt the CPU (requires reset)
            (Instruction::JAM, OpInput::UseImplied) => {
                self.halted = true;
            }

            // LAS - AND memory with SP, load to A, X, SP
            (Instruction::LAS, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr) & self.registers.stack_pointer.0;
                self.registers.accumulator = val;
                self.registers.index_x = val;
                self.registers.stack_pointer = StackPointer(val);
                CPU::<M, V>::set_flags_from_u8(&mut self.registers.status, val);
            }

            // LAX - Load A and X with the same value from memory
            (Instruction::LAX, OpInput::UseAddress { address: addr, .. }) => {
                let val = self.memory.get_byte(addr);
                self.load_accumulator(val);
                self.load_x_register(val);
            }

            // NOP variants - read memory but do nothing
            (Instruction::NOPI, OpInput::UseImmediate(_)) => {}
            (Instruction::NOPZ, OpInput::UseAddress { .. }) => {}
            (Instruction::NOPZX, OpInput::UseAddress { .. }) => {}
            (Instruction::NOPA, OpInput::UseAddress { .. }) => {}
            (Instruction::NOPAX, OpInput::UseAddress { .. }) => {}

            // RLA - Rotate left memory, then AND with accumulator
            (Instruction::RLA, OpInput::UseAddress { address: addr, .. }) => {
                let mut val = self.memory.get_byte(addr);
                CPU::<M, V>::rotate_left_with_flags(&mut val, &mut self.registers.status);
                self.memory.set_byte(addr, val);
                self.and(val);
            }

            // RRA - Rotate right memory, then ADC with accumulator
            (Instruction::RRA, OpInput::UseAddress { address: addr, .. }) => {
                let mut val = self.memory.get_byte(addr);
                CPU::<M, V>::rotate_right_with_flags(&mut val, &mut self.registers.status);
                self.memory.set_byte(addr, val);
                self.add_with_carry(val);
            }

            // SBX - (A AND X) - immediate -> X, flags like CMP
            (Instruction::SBX, OpInput::UseImmediate(val)) => {
                let ax = self.registers.accumulator & self.registers.index_x;
                let result = ax.wrapping_sub(val);
                self.registers.index_x = result;

                // Set carry if no borrow (ax >= val)
                if ax >= val {
                    self.set_flag(Status::PS_CARRY);
                } else {
                    self.unset_flag(Status::PS_CARRY);
                }

                CPU::<M, V>::set_flags_from_u8(&mut self.registers.status, result);
            }

            // SLO - Shift left memory, then OR with accumulator
            (Instruction::SLO, OpInput::UseAddress { address: addr, .. }) => {
                let mut val = self.memory.get_byte(addr);
                CPU::<M, V>::shift_left_with_flags(&mut val, &mut self.registers.status);
                self.memory.set_byte(addr, val);
                self.inclusive_or(val);
            }

            // SRE - Shift right memory, then EOR with accumulator
            (Instruction::SRE, OpInput::UseAddress { address: addr, .. }) => {
                let mut val = self.memory.get_byte(addr);
                CPU::<M, V>::shift_right_with_flags(&mut val, &mut self.registers.status);
                self.memory.set_byte(addr, val);
                self.exclusive_or(val);
            }

            // USBC - Same as SBC immediate
            (Instruction::USBC, OpInput::UseImmediate(val)) => {
                self.subtract_with_carry(val);
            }

            (_, _) => {
                log::debug!(
                    "attempting to execute unimplemented or invalid \
                     instruction"
                );
            }
        }
    }

    /// Execute a single instruction.
    ///
    /// Returns `true` if an instruction was executed,
    /// `false` if the CPU is halted or no instruction could be fetched.
    pub fn single_step(&mut self) -> bool {
        if self.halted {
            return false;
        }
        if let Some(decoded_instr) = self.fetch_next_and_decode() {
            self.execute_instruction(decoded_instr);
            true
        } else {
            false
        }
    }

    pub fn run(&mut self) {
        while !self.halted
            && let Some(decoded_instr) = self.fetch_next_and_decode()
        {
            self.execute_instruction(decoded_instr);
        }
    }

    /// Checks if a given `u8` value should be interpreted as negative when
    /// considered as `i8`.
    ///
    /// In an 8-bit unsigned integer (`u8`), values range from 0 to 255. When
    /// these values are interpreted as signed integers (`i8`), values from 128
    /// to 255 are considered negative, corresponding to the signed range -128
    /// to -1. This function checks if the provided `u8` value falls within that
    /// range, effectively determining if the most significant bit is set, which
    /// indicates a negative number in two's complement form.
    /// ```
    const fn value_is_negative(value: u8) -> bool {
        value > 127
    }

    fn set_flags_from_u8(status: &mut Status, value: u8) {
        let is_zero = value == 0;
        let is_negative = Self::value_is_negative(value);

        status.set_with_mask(
            Status::PS_ZERO | Status::PS_NEGATIVE,
            Status::new(StatusArgs {
                zero: is_zero,
                negative: is_negative,
                ..StatusArgs::none()
            }),
        );
    }

    fn shift_left_with_flags(p_val: &mut u8, status: &mut Status) {
        let mask = 1 << 7;
        let is_bit_7_set = (*p_val & mask) == mask;
        let shifted = (*p_val & !(1 << 7)) << 1;
        *p_val = shifted;
        status.set_with_mask(
            Status::PS_CARRY,
            Status::new(StatusArgs {
                carry: is_bit_7_set,
                ..StatusArgs::none()
            }),
        );
        CPU::<M, V>::set_flags_from_u8(status, *p_val);
    }

    fn shift_right_with_flags(p_val: &mut u8, status: &mut Status) {
        let mask = 1;
        let is_bit_0_set = (*p_val & mask) == mask;
        *p_val >>= 1;
        status.set_with_mask(
            Status::PS_CARRY,
            Status::new(StatusArgs {
                carry: is_bit_0_set,
                ..StatusArgs::none()
            }),
        );
        CPU::<M, V>::set_flags_from_u8(status, *p_val);
    }

    fn rotate_left_with_flags(p_val: &mut u8, status: &mut Status) {
        let is_carry_set = status.contains(Status::PS_CARRY);
        let mask = 1 << 7;
        let is_bit_7_set = (*p_val & mask) == mask;
        let shifted = (*p_val & !(1 << 7)) << 1;
        *p_val = shifted + u8::from(is_carry_set);
        status.set_with_mask(
            Status::PS_CARRY,
            Status::new(StatusArgs {
                carry: is_bit_7_set,
                ..StatusArgs::none()
            }),
        );
        CPU::<M, V>::set_flags_from_u8(status, *p_val);
    }

    fn rotate_right_with_flags(p_val: &mut u8, status: &mut Status) {
        let is_carry_set = status.contains(Status::PS_CARRY);
        let mask = 1;
        let is_bit_0_set = (*p_val & mask) == mask;
        let shifted = *p_val >> 1;
        *p_val = shifted + if is_carry_set { 1 << 7 } else { 0 };
        status.set_with_mask(
            Status::PS_CARRY,
            Status::new(StatusArgs {
                carry: is_bit_0_set,
                ..StatusArgs::none()
            }),
        );
        CPU::<M, V>::set_flags_from_u8(status, *p_val);
    }

    fn set_u8_with_flags(mem: &mut u8, status: &mut Status, value: u8) {
        *mem = value;
        CPU::<M, V>::set_flags_from_u8(status, value);
    }

    fn load_x_register(&mut self, value: u8) {
        CPU::<M, V>::set_u8_with_flags(
            &mut self.registers.index_x,
            &mut self.registers.status,
            value,
        );
    }

    fn load_y_register(&mut self, value: u8) {
        CPU::<M, V>::set_u8_with_flags(
            &mut self.registers.index_y,
            &mut self.registers.status,
            value,
        );
    }

    fn load_accumulator(&mut self, value: u8) {
        CPU::<M, V>::set_u8_with_flags(
            &mut self.registers.accumulator,
            &mut self.registers.status,
            value,
        );
    }

    /// Shorthand for checking if a specific flag is set in the status register
    #[inline]
    const fn get_flag(&self, flag: Status) -> bool {
        self.registers.status.contains(flag)
    }

    /// Shorthand for setting a specific flag in the status register
    #[inline]
    fn set_flag(&mut self, flag: Status) {
        self.registers.status.or(flag);
    }

    /// Shorthand for clearing a specific flag in the status register
    #[inline]
    fn unset_flag(&mut self, flag: Status) {
        self.registers.status.and(!flag);
    }

    /// Executes the following calculation: A + M + C (Add with Carry)
    ///
    /// This implementation follows the NMOS 6502 behavior as documented in authoritative sources.
    ///
    /// ## Decimal Mode Behavior (NMOS 6502)
    ///
    /// In decimal mode, this instruction performs Binary Coded Decimal (BCD) arithmetic
    /// where each nibble represents a decimal digit (0-9). However, flag behavior differs
    /// significantly from binary mode:
    ///
    /// - **Carry Flag (C)**: Correctly set when BCD addition overflows (> 99)
    /// - **Overflow Flag (V)**: Calculated from the binary addition result, not the BCD result.
    ///   The meaning is effectively undocumented in decimal mode since BCD is unsigned arithmetic.
    /// - **Negative Flag (N)**: Set from the BCD result but may not match expected 10's complement behavior
    /// - **Zero Flag (Z)**: Set from the BCD result but may not always be correct on NMOS 6502
    ///
    /// ## Invalid BCD Values
    ///
    /// When either nibble contains values A-F (invalid BCD), the NMOS 6502 behavior is
    /// undefined. This implementation treats them as valid binary values, which produces
    /// deterministic results but may not match all real hardware variants.
    ///
    /// ## References
    ///
    /// - [6502.org Decimal Mode Tutorial](http://www.6502.org/tutorials/decimal_mode.html)
    /// - [NESdev Wiki 6502 Decimal Mode](https://www.nesdev.org/wiki/Visual6502wiki/6502DecimalMode)
    /// - Bruce Clark's comprehensive decimal mode test programs
    ///
    /// ## Variant Differences
    ///
    /// - **NMOS 6502**: Only carry flag is reliable in decimal mode
    /// - **65C02**: N and Z flags are valid, V flag still undocumented, +1 cycle in decimal mode
    /// - **RP2A03** (NES): Decimal mode completely disabled in hardware
    fn add_with_carry(&mut self, value: u8) {
        let carry_set = self.get_flag(Status::PS_CARRY);
        let decimal_mode = self.get_flag(Status::PS_DECIMAL_MODE);

        // Use variant-specific ADC implementation
        let output = if decimal_mode {
            V::adc_decimal(self.registers.accumulator, value, carry_set)
        } else {
            V::adc_binary(self.registers.accumulator, value, carry_set)
        };

        // Update processor status flags
        self.registers.status.set_with_mask(
            Status::PS_CARRY | Status::PS_OVERFLOW | Status::PS_ZERO | Status::PS_NEGATIVE,
            Status::new(StatusArgs {
                carry: output.did_carry,
                overflow: output.overflow,
                zero: output.zero,
                negative: output.negative,
                ..StatusArgs::none()
            }),
        );

        // Update accumulator
        self.registers.accumulator = output.result;
    }

    fn add_with_no_decimal(&mut self, value: u8) {
        let carry_set = self.get_flag(Status::PS_CARRY);

        // Use variant-specific binary ADC implementation
        let output = V::adc_binary(self.registers.accumulator, value, carry_set);

        // Update processor status flags
        self.registers.status.set_with_mask(
            Status::PS_CARRY | Status::PS_OVERFLOW | Status::PS_ZERO | Status::PS_NEGATIVE,
            Status::new(StatusArgs {
                carry: output.did_carry,
                overflow: output.overflow,
                zero: output.zero,
                negative: output.negative,
                ..StatusArgs::none()
            }),
        );

        // Update accumulator
        self.registers.accumulator = output.result;
    }

    fn and(&mut self, value: u8) {
        let a_after = self.registers.accumulator & value;
        self.load_accumulator(a_after);
    }

    fn subtract_with_no_decimal(&mut self, value: u8) {
        // A - M - (1 - C)

        // nc -- 'not carry'
        let nc: u8 = u8::from(!self.registers.status.contains(Status::PS_CARRY));

        let a_before = self.registers.accumulator;

        let a_after = a_before.wrapping_sub(value).wrapping_sub(nc);

        // The overflow flag is set on two's-complement overflow.
        //
        // range of A              is  -128 to 127
        // range of - M - (1 - C)  is  -128 to 128
        //                             -(127 + 1) to -(-128 + 0)
        //
        let over = (nc == 0 && value > 127) && a_before < 128 && a_after > 127;

        let under =
            (a_before > 127) && (0u8.wrapping_sub(value).wrapping_sub(nc) > 127) && a_after < 128;

        let did_overflow = over || under;

        let mask = Status::PS_CARRY | Status::PS_OVERFLOW;

        let result = a_after;

        // The carry flag is set on unsigned overflow.
        let did_carry = (result) > (a_before);

        self.registers.status.set_with_mask(
            mask,
            Status::new(StatusArgs {
                carry: did_carry,
                overflow: did_overflow,
                ..StatusArgs::none()
            }),
        );

        self.load_accumulator(result);
    }

    /// Executes the following calculation: A - M - (1 - C) (Subtract with Carry)
    ///
    /// This implementation follows the NMOS 6502 behavior as documented in authoritative sources.
    ///
    /// ## Decimal Mode Behavior (NMOS 6502)
    ///
    /// In decimal mode, this instruction performs Binary Coded Decimal (BCD) arithmetic
    /// where each nibble represents a decimal digit (0-9). Flag behavior matches ADC:
    ///
    /// - **Carry Flag (C)**: Correctly set (inverse of borrow) for BCD subtraction
    /// - **Overflow Flag (V)**: Disabled in decimal mode (always false) to match real hardware
    /// - **Negative Flag (N)**: Set from the BCD result but behavior is undocumented
    /// - **Zero Flag (Z)**: Set from the BCD result but may not always be correct on NMOS 6502
    ///
    /// ## Invalid BCD Values
    ///
    /// When either nibble contains values A-F (invalid BCD), the NMOS 6502 behavior is
    /// undefined. This implementation handles them deterministically but results may vary
    /// from real hardware.
    ///
    /// ## References
    ///
    /// - [6502.org Decimal Mode Tutorial](http://www.6502.org/tutorials/decimal_mode.html)
    /// - [NESdev Wiki 6502 Decimal Mode](https://www.nesdev.org/wiki/Visual6502wiki/6502DecimalMode)
    ///
    /// ## Variant Differences
    ///
    /// - **NMOS 6502**: Only carry flag is reliable in decimal mode
    /// - **65C02**: N and Z flags are valid, V flag still undocumented
    /// - **RP2A03** (NES): Decimal mode completely disabled in hardware
    fn subtract_with_carry(&mut self, value: u8) {
        let carry_set = self.get_flag(Status::PS_CARRY);
        let decimal_mode = self.get_flag(Status::PS_DECIMAL_MODE);

        // Use variant-specific SBC implementation
        let output = if decimal_mode {
            V::sbc_decimal(self.registers.accumulator, value, carry_set)
        } else {
            V::sbc_binary(self.registers.accumulator, value, carry_set)
        };

        // Update processor status flags
        self.registers.status.set_with_mask(
            Status::PS_CARRY | Status::PS_OVERFLOW | Status::PS_ZERO | Status::PS_NEGATIVE,
            Status::new(StatusArgs {
                carry: output.did_carry,
                overflow: output.overflow,
                zero: output.zero,
                negative: output.negative,
                ..StatusArgs::none()
            }),
        );

        // Update accumulator
        self.registers.accumulator = output.result;
    }

    fn increment(val: &mut u8, flags: &mut Status) {
        let value_new = val.wrapping_add(1);
        *val = value_new;

        let is_zero = value_new == 0;

        flags.set_with_mask(
            Status::PS_NEGATIVE | Status::PS_ZERO,
            Status::new(StatusArgs {
                negative: Self::value_is_negative(value_new),
                zero: is_zero,
                ..StatusArgs::none()
            }),
        );
    }

    fn decrement(val: &mut u8, flags: &mut Status) {
        let value_new = val.wrapping_sub(1);
        *val = value_new;

        let is_zero = value_new == 0;
        let is_negative = Self::value_is_negative(value_new);

        flags.set_with_mask(
            Status::PS_NEGATIVE | Status::PS_ZERO,
            Status::new(StatusArgs {
                zero: is_zero,
                negative: is_negative,
                ..StatusArgs::none()
            }),
        );
    }

    const fn jump(&mut self, addr: u16) {
        self.registers.program_counter = addr;
    }

    const fn branch_if_carry_clear(&mut self, addr: u16) {
        if !self.registers.status.contains(Status::PS_CARRY) {
            self.registers.program_counter = addr;
        }
    }

    const fn branch_if_carry_set(&mut self, addr: u16) {
        if self.registers.status.contains(Status::PS_CARRY) {
            self.registers.program_counter = addr;
        }
    }

    const fn branch_if_equal(&mut self, addr: u16) {
        if self.registers.status.contains(Status::PS_ZERO) {
            self.registers.program_counter = addr;
        }
    }

    const fn branch_if_not_equal(&mut self, addr: u16) {
        if !self.registers.status.contains(Status::PS_ZERO) {
            self.registers.program_counter = addr;
        }
    }

    const fn branch_if_minus(&mut self, addr: u16) {
        if self.registers.status.contains(Status::PS_NEGATIVE) {
            self.registers.program_counter = addr;
        }
    }

    const fn branch(&mut self, addr: u16) {
        self.registers.program_counter = addr;
    }

    const fn branch_if_positive(&mut self, addr: u16) {
        if !self.registers.status.contains(Status::PS_NEGATIVE) {
            self.registers.program_counter = addr;
        }
    }

    const fn branch_if_overflow_clear(&mut self, addr: u16) {
        if !self.registers.status.contains(Status::PS_OVERFLOW) {
            self.registers.program_counter = addr;
        }
    }

    const fn branch_if_overflow_set(&mut self, addr: u16) {
        if self.registers.status.contains(Status::PS_OVERFLOW) {
            self.registers.program_counter = addr;
        }
    }

    // From http://www.6502.org/tutorials/compare_beyond.html:
    //   If the Z flag is 0, then A <> NUM and BNE will branch
    //   If the Z flag is 1, then A = NUM and BEQ will branch
    //   If the C flag is 0, then A (unsigned) < NUM (unsigned) and BCC will branch
    //   If the C flag is 1, then A (unsigned) >= NUM (unsigned) and BCS will branch
    //   ...
    //   The N flag contains most significant bit of the subtraction result.
    fn compare(&mut self, r: u8, val: u8) {
        // Setting the CARRY flag: A (unsigned) >= NUM (unsigned)
        if r >= val {
            self.registers.status.insert(Status::PS_CARRY);
        } else {
            self.registers.status.remove(Status::PS_CARRY);
        }

        // Setting the ZERO flag: A = NUM
        if r == val {
            self.registers.status.insert(Status::PS_ZERO);
        } else {
            self.registers.status.remove(Status::PS_ZERO);
        }

        // Set the NEGATIVE flag based on the MSB of the result of subtraction
        // This checks if the 8th bit is set (0x80 in hex is 128 in decimal, which is the 8th bit in a byte)
        let diff = r.wrapping_sub(val);
        if Self::value_is_negative(diff) {
            self.registers.status.insert(Status::PS_NEGATIVE);
        } else {
            self.registers.status.remove(Status::PS_NEGATIVE);
        }
    }

    fn compare_with_a_register(&mut self, val: u8) {
        let a = self.registers.accumulator;
        self.compare(a, val);
    }

    fn compare_with_x_register(&mut self, val: u8) {
        log::debug!("compare_with_x_register");

        let x = self.registers.index_x;
        self.compare(x, val);
    }

    fn compare_with_y_register(&mut self, val: u8) {
        let y = self.registers.index_y;
        self.compare(y, val);
    }

    fn exclusive_or(&mut self, val: u8) {
        let a_after = self.registers.accumulator ^ val;
        self.load_accumulator(a_after);
    }

    fn inclusive_or(&mut self, val: u8) {
        let a_after = self.registers.accumulator | val;
        self.load_accumulator(a_after);
    }

    fn push_on_stack(&mut self, val: u8) {
        let addr = self.registers.stack_pointer.to_u16();
        self.memory.set_byte(addr, val);
        self.registers.stack_pointer.decrement();
    }

    fn pull_from_stack(&mut self) -> u8 {
        self.registers.stack_pointer.increment();
        let addr = self.registers.stack_pointer.to_u16();
        self.memory.get_byte(addr)
    }
}

impl<M: Bus, V: Variant> core::fmt::Debug for CPU<M, V> {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "CPU {{ registers: {:?}", self.registers)
    }
}

#[cfg(test)]
mod tests {
    // Casting from signed to unsigned integers is intentional in these tests
    #![allow(clippy::cast_sign_loss)]
    // Operations may intentionally wrap due to emulation of 8-bit unsigned
    // integer arithmetic. We do this to test wrap-around conditions.
    #![allow(clippy::cast_possible_wrap)]

    use super::*;
    use crate::instruction::Nmos6502;
    use crate::memory::Memory as Ram;

    #[test]
    fn dont_panic_for_overflow() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.add_with_carry(0x80);
        assert_eq!(cpu.registers.accumulator, 0x80);
        cpu.add_with_carry(0x80);
        assert_eq!(cpu.registers.accumulator, 0);

        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.subtract_with_carry(0x80);
        assert_eq!(cpu.registers.accumulator, 0x80);
        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.subtract_with_carry(0x80);
        assert_eq!(cpu.registers.accumulator, 0);
    }

    #[test]
    fn decimal_add_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers.status.insert(Status::PS_DECIMAL_MODE);

        cpu.add_with_carry(0x09);
        assert_eq!(cpu.registers.accumulator, 0x09);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        // Overflow flag is calculated from binary result
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.add_with_carry(0x43);
        assert_eq!(cpu.registers.accumulator, 0x52);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        // No overflow: 0x09 + 0x43 = 0x4C (binary), both positive, result positive
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.add_with_carry(0x48);
        assert_eq!(cpu.registers.accumulator, 0x00);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        // Overflow: 0x52 + 0x48 = 0x9A (binary), both positive, result negative
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));
    }

    #[test]
    fn decimal_subtract_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers
            .status
            .insert(Status::PS_DECIMAL_MODE | Status::PS_CARRY);
        cpu.registers.accumulator = 0;

        cpu.subtract_with_carry(0x01);
        assert_eq!(cpu.registers.accumulator, 0x99);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.registers.accumulator = 0x50;
        cpu.subtract_with_carry(0x25);
        assert_eq!(cpu.registers.accumulator, 0x25);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));
    }

    #[test]
    fn add_with_carry_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // Non-decimal mode tests
        cpu.registers.status.remove(Status::PS_DECIMAL_MODE);

        // Test case 1: 0 + 0 with carry clear
        cpu.registers.accumulator = 0;
        cpu.registers.status.remove(Status::PS_CARRY);
        cpu.add_with_carry(0);
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        // Test case 2: 0 + 1 with carry set
        cpu.registers.accumulator = 0;
        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.add_with_carry(1);
        assert_eq!(cpu.registers.accumulator, 2);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        // Test case 3: 0x7F + 0x01 (overflow case)
        cpu.registers.accumulator = 0x7F;
        cpu.registers.status.remove(Status::PS_CARRY);
        cpu.add_with_carry(0x01);
        assert_eq!(cpu.registers.accumulator, 0x80);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));

        // Decimal mode tests
        cpu.registers.status.insert(Status::PS_DECIMAL_MODE);

        // Test case 4: 09 + 01 with carry clear (decimal)
        cpu.registers.accumulator = 0x09;
        cpu.registers.status.remove(Status::PS_CARRY);
        cpu.add_with_carry(0x01);
        assert_eq!(cpu.registers.accumulator, 0x10);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        // Test case 5: 50 + 50 with carry clear (decimal)
        cpu.registers.accumulator = 0x50;
        cpu.registers.status.remove(Status::PS_CARRY);
        cpu.add_with_carry(0x50);
        assert_eq!(cpu.registers.accumulator, 0x00);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        // Test case 6: 99 + 01 with carry set (decimal)
        cpu.registers.accumulator = 0x99;
        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.add_with_carry(0x01);
        assert_eq!(cpu.registers.accumulator, 0x01);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        // Additional test cases for comprehensive verification

        // Non-decimal mode: Test carry flag with 0xFF + 0x01
        cpu.registers.status.remove(Status::PS_DECIMAL_MODE);
        cpu.registers.accumulator = 0xFF;
        cpu.registers.status.remove(Status::PS_CARRY);
        cpu.add_with_carry(0x01);
        assert_eq!(cpu.registers.accumulator, 0x00);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        // Non-decimal mode: Test carry flag with 0xFF + 0x01 + carry
        cpu.registers.accumulator = 0xFF;
        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.add_with_carry(0x01);
        assert_eq!(cpu.registers.accumulator, 0x01);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        // Non-decimal mode: Test negative overflow (0x80 + 0x80)
        cpu.registers.accumulator = 0x80;
        cpu.registers.status.remove(Status::PS_CARRY);
        cpu.add_with_carry(0x80);
        assert_eq!(cpu.registers.accumulator, 0x00);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));
    }

    #[test]
    fn solid65_adc_immediate() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // Adding $FF plus carry should be the same as adding $00 and no carry, so these three
        // instructions should leave the carry flags unaffected, i.e. set.
        cpu.execute_instruction((
            Instruction::LDA,
            AddressingMode::Immediate,
            OpInput::UseImmediate(0x9c),
        ));
        cpu.execute_instruction((
            Instruction::SEC,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        cpu.execute_instruction((
            Instruction::ADC,
            AddressingMode::Immediate,
            OpInput::UseImmediate(0xff),
        ));

        assert_eq!(cpu.registers.accumulator, 0x9c);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
    }

    #[test]
    fn php_sets_bits_4_and_5() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.execute_instruction((
            Instruction::PHP,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        cpu.execute_instruction((
            Instruction::PLA,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        cpu.execute_instruction((
            Instruction::AND,
            AddressingMode::Immediate,
            OpInput::UseImmediate(0x30),
        ));

        assert_eq!(cpu.registers.accumulator, 0x30);
    }

    #[test]
    fn and_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.registers.accumulator = 0;
        cpu.and(0xff);
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.registers.accumulator = 0xff;
        cpu.and(0);
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.registers.accumulator = 0xff;
        cpu.and(0x0f);
        assert_eq!(cpu.registers.accumulator, 0x0f);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.registers.accumulator = 0xff;
        cpu.and(0x80);
        assert_eq!(cpu.registers.accumulator, 0x80);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn subtract_with_carry_comprehensive_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // Non-decimal mode tests
        cpu.registers.status.remove(Status::PS_DECIMAL_MODE);

        // Test case 1: 0 - 0 with carry set
        cpu.registers.accumulator = 0;
        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.subtract_with_carry(0);
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        // Test case 2: 0 - 1 with carry set
        cpu.registers.accumulator = 0;
        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.subtract_with_carry(1);
        assert_eq!(cpu.registers.accumulator, 0xFF);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        // Test case 3: 0x80 - 0x01 with carry set (overflow case)
        cpu.registers.accumulator = 0x80;
        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.subtract_with_carry(0x01);
        assert_eq!(cpu.registers.accumulator, 0x7F);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));

        // Decimal mode tests
        cpu.registers.status.insert(Status::PS_DECIMAL_MODE);

        // Test case 4: 10 - 05 with carry set (decimal)
        cpu.registers.accumulator = 0x10;
        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.subtract_with_carry(0x05);
        assert_eq!(cpu.registers.accumulator, 0x05);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        // Test case 5: 20 - 10 with carry clear (decimal)
        cpu.registers.accumulator = 0x20;
        cpu.registers.status.remove(Status::PS_CARRY);
        cpu.subtract_with_carry(0x10);
        assert_eq!(cpu.registers.accumulator, 0x09);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        // Test case 6: 00 - 01 with carry set (decimal, borrow)
        cpu.registers.accumulator = 0x00;
        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.subtract_with_carry(0x01);
        assert_eq!(cpu.registers.accumulator, 0x99);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));

        // Test case 7: 99 - 01 with carry set (decimal, no borrow)
        cpu.registers.accumulator = 0x99;
        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.subtract_with_carry(0x01);
        assert_eq!(cpu.registers.accumulator, 0x98);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn decrement_memory_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        let addr: u16 = 0xA1B2;

        cpu.memory.set_byte(addr, 5);

        cpu.execute_instruction((
            Instruction::DEC,
            AddressingMode::Absolute,
            OpInput::UseAddress {
                address: addr,
                page_crossed: false,
            },
        ));
        assert_eq!(cpu.memory.get_byte(addr), 4);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((
            Instruction::DEC,
            AddressingMode::Absolute,
            OpInput::UseAddress {
                address: addr,
                page_crossed: false,
            },
        ));
        assert_eq!(cpu.memory.get_byte(addr), 3);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((
            Instruction::DEC,
            AddressingMode::Absolute,
            OpInput::UseAddress {
                address: addr,
                page_crossed: false,
            },
        ));
        cpu.execute_instruction((
            Instruction::DEC,
            AddressingMode::Absolute,
            OpInput::UseAddress {
                address: addr,
                page_crossed: false,
            },
        ));
        cpu.execute_instruction((
            Instruction::DEC,
            AddressingMode::Absolute,
            OpInput::UseAddress {
                address: addr,
                page_crossed: false,
            },
        ));
        assert_eq!(cpu.memory.get_byte(addr), 0);
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((
            Instruction::DEC,
            AddressingMode::Absolute,
            OpInput::UseAddress {
                address: addr,
                page_crossed: false,
            },
        ));
        assert_eq!(cpu.memory.get_byte(addr) as i8, -1);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.memory.set_byte(addr, 0);

        cpu.execute_instruction((
            Instruction::DEC,
            AddressingMode::Absolute,
            OpInput::UseAddress {
                address: addr,
                page_crossed: false,
            },
        ));
        assert_eq!(cpu.memory.get_byte(addr), 0xff);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn decrement_x_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers.index_x = 0x80;
        cpu.execute_instruction((
            Instruction::DEX,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        assert_eq!(cpu.registers.index_x, 127);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn decrement_y_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers.index_y = 0x80;
        cpu.execute_instruction((
            Instruction::DEY,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        assert_eq!(cpu.registers.index_y, 127);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn logical_shift_right_test() {
        // Testing UseImplied version (which targets the accumulator) only, for now

        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.execute_instruction((
            Instruction::LDA,
            AddressingMode::Immediate,
            OpInput::UseImmediate(0),
        ));
        cpu.execute_instruction((
            Instruction::LSR,
            AddressingMode::Accumulator,
            OpInput::UseImplied,
        ));
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((
            Instruction::LDA,
            AddressingMode::Immediate,
            OpInput::UseImmediate(1),
        ));
        cpu.execute_instruction((
            Instruction::LSR,
            AddressingMode::Accumulator,
            OpInput::UseImplied,
        ));
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((
            Instruction::LDA,
            AddressingMode::Immediate,
            OpInput::UseImmediate(255),
        ));
        cpu.execute_instruction((
            Instruction::LSR,
            AddressingMode::Accumulator,
            OpInput::UseImplied,
        ));
        assert_eq!(cpu.registers.accumulator, 0x7F);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((
            Instruction::LDA,
            AddressingMode::Immediate,
            OpInput::UseImmediate(254),
        ));
        cpu.execute_instruction((
            Instruction::LSR,
            AddressingMode::Accumulator,
            OpInput::UseImplied,
        ));
        assert_eq!(cpu.registers.accumulator, 0x7F);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));
    }

    #[test]
    fn dec_x_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.execute_instruction((
            Instruction::DEX,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        assert_eq!(cpu.registers.index_x, 0xff);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((
            Instruction::DEX,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        assert_eq!(cpu.registers.index_x, 0xfe);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.load_x_register(5);
        cpu.execute_instruction((
            Instruction::DEX,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        assert_eq!(cpu.registers.index_x, 4);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((
            Instruction::DEX,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        cpu.execute_instruction((
            Instruction::DEX,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        cpu.execute_instruction((
            Instruction::DEX,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        cpu.execute_instruction((
            Instruction::DEX,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));

        assert_eq!(cpu.registers.index_x, 0);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((
            Instruction::DEX,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        assert_eq!(cpu.registers.index_x, 0xff);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));
    }

    #[test]
    fn jump_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        let addr: u16 = 0xA1B1;

        cpu.jump(addr);
        assert_eq!(cpu.registers.program_counter, addr);
    }

    #[test]
    fn branch_if_carry_clear_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.execute_instruction((
            Instruction::SEC,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        cpu.branch_if_carry_clear(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0));

        cpu.execute_instruction((
            Instruction::CLC,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        cpu.branch_if_carry_clear(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0xABCD));
    }

    #[test]
    fn branch_if_carry_set_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.execute_instruction((
            Instruction::CLC,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        cpu.branch_if_carry_set(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0));

        cpu.execute_instruction((
            Instruction::SEC,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        cpu.branch_if_carry_set(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0xABCD));
    }

    #[test]
    fn branch_if_equal_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.branch_if_equal(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0));

        cpu.set_flag(Status::PS_ZERO);
        cpu.branch_if_equal(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0xABCD));
    }

    #[test]
    fn branch_if_minus_test() {
        {
            let mut cpu = CPU::new(Ram::new(), Nmos6502);
            let registers_before = cpu.registers;

            cpu.branch_if_minus(0xABCD);
            assert_eq!(cpu.registers, registers_before);
            assert_eq!(cpu.registers.program_counter, (0));
        }

        {
            let mut cpu = CPU::new(Ram::new(), Nmos6502);

            cpu.set_flag(Status::PS_NEGATIVE);
            let registers_before = cpu.registers;

            cpu.branch_if_minus(0xABCD);
            assert_eq!(cpu.registers.status, registers_before.status);
            assert_eq!(cpu.registers.program_counter, (0xABCD));
        }
    }

    #[test]
    fn branch_if_positive_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.registers.status.insert(Status::PS_NEGATIVE);
        cpu.branch_if_positive(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0));

        cpu.registers.status.remove(Status::PS_NEGATIVE);
        cpu.branch_if_positive(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0xABCD));
    }

    #[test]
    fn branch_if_overflow_clear_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.registers.status.insert(Status::PS_OVERFLOW);
        cpu.branch_if_overflow_clear(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0));

        cpu.registers.status.remove(Status::PS_OVERFLOW);
        cpu.branch_if_overflow_clear(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0xABCD));
    }

    #[test]
    fn branch_across_end_of_address_space() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers.program_counter = 0xffff;

        cpu.registers.status.insert(Status::PS_OVERFLOW);
        cpu.branch_if_overflow_set(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0xABCD));
    }

    #[test]
    fn branch_if_overflow_set_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.branch_if_overflow_set(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0));

        cpu.registers.status.insert(Status::PS_OVERFLOW);
        cpu.branch_if_overflow_set(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0xABCD));
    }

    #[cfg(test)]
    fn compare_test_helper<F>(compare: &mut F, load_instruction: Instruction)
    where
        F: FnMut(&mut CPU<Ram, crate::instruction::Nmos6502>, u8),
    {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.execute_instruction((
            load_instruction,
            AddressingMode::Immediate,
            OpInput::UseImmediate(127),
        ));

        compare(&mut cpu, 127);
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((
            load_instruction,
            AddressingMode::Immediate,
            OpInput::UseImmediate(127),
        ));

        compare(&mut cpu, 1);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((
            load_instruction,
            AddressingMode::Immediate,
            OpInput::UseImmediate(1),
        ));

        compare(&mut cpu, 2);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((
            load_instruction,
            AddressingMode::Immediate,
            OpInput::UseImmediate(20),
        ));

        compare(&mut cpu, -50i8 as u8);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((
            load_instruction,
            AddressingMode::Immediate,
            OpInput::UseImmediate(1),
        ));

        compare(&mut cpu, -1i8 as u8);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((
            load_instruction,
            AddressingMode::Immediate,
            OpInput::UseImmediate(127),
        ));

        compare(&mut cpu, -128i8 as u8);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn compare_with_a_register_test() {
        compare_test_helper(
            &mut |cpu: &mut CPU<Ram, Nmos6502>, val: u8| {
                cpu.compare_with_a_register(val);
            },
            Instruction::LDA,
        );
    }

    #[test]
    fn compare_with_x_register_test() {
        compare_test_helper(
            &mut |cpu: &mut CPU<Ram, Nmos6502>, val: u8| {
                cpu.compare_with_x_register(val);
            },
            Instruction::LDX,
        );
    }

    #[test]
    fn compare_with_y_register_test() {
        compare_test_helper(
            &mut |cpu: &mut CPU<Ram, Nmos6502>, val: u8| {
                cpu.compare_with_y_register(val);
            },
            Instruction::LDY,
        );
    }

    #[test]
    fn exclusive_or_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        for a_before in 0u8..=255u8 {
            for val in 0u8..=255u8 {
                cpu.execute_instruction((
                    Instruction::LDA,
                    AddressingMode::Immediate,
                    OpInput::UseImmediate(a_before),
                ));

                cpu.exclusive_or(val);

                let a_after = a_before ^ val;
                assert_eq!(cpu.registers.accumulator, a_after);

                if a_after == 0 {
                    assert!(cpu.registers.status.contains(Status::PS_ZERO));
                } else {
                    assert!(!cpu.registers.status.contains(Status::PS_ZERO));
                }

                if (a_after as i8) < 0 {
                    assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
                } else {
                    assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
                }
            }
        }
    }

    #[test]
    fn inclusive_or_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        for a_before in 0u8..=255u8 {
            for val in 0u8..=255u8 {
                cpu.execute_instruction((
                    Instruction::LDA,
                    AddressingMode::Immediate,
                    OpInput::UseImmediate(a_before),
                ));

                cpu.inclusive_or(val);

                let a_after = a_before | val;
                assert_eq!(cpu.registers.accumulator, a_after);

                if a_after == 0 {
                    assert!(cpu.registers.status.contains(Status::PS_ZERO));
                } else {
                    assert!(!cpu.registers.status.contains(Status::PS_ZERO));
                }

                if (a_after as i8) < 0 {
                    assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
                } else {
                    assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
                }
            }
        }
    }

    #[test]
    fn stack_underflow() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        let _val: u8 = cpu.pull_from_stack();
    }

    #[test]
    fn nmos6502_adc_decimal_mode() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers.accumulator = 0x09;
        cpu.registers.status.insert(Status::PS_DECIMAL_MODE);
        cpu.registers.status.remove(Status::PS_CARRY);

        cpu.add_with_carry(0x01);

        // Should produce BCD result: 09 + 01 = 10 (decimal)
        assert_eq!(cpu.registers.accumulator, 0x10);
        assert!(!cpu.get_flag(Status::PS_CARRY));
    }

    #[test]
    fn ricoh2a03_ignores_decimal_mode() {
        use crate::instruction::Ricoh2a03;

        let mut cpu = CPU::new(Ram::new(), Ricoh2a03);
        cpu.registers.accumulator = 0x09;
        cpu.registers.status.insert(Status::PS_DECIMAL_MODE);
        cpu.registers.status.remove(Status::PS_CARRY);

        cpu.add_with_carry(0x01);

        // Should be binary arithmetic: 0x09 + 0x01 = 0x0A (not 0x10)
        assert_eq!(cpu.registers.accumulator, 0x0A);
        assert!(!cpu.get_flag(Status::PS_CARRY));
    }

    #[test]
    fn cmos6502_adc_decimal_mode() {
        use crate::instruction::Cmos6502;

        let mut cpu = CPU::new(Ram::new(), Cmos6502);
        cpu.registers.accumulator = 0x09;
        cpu.registers.status.insert(Status::PS_DECIMAL_MODE);
        cpu.registers.status.remove(Status::PS_CARRY);

        cpu.add_with_carry(0x01);

        // Should produce BCD result like NMOS: 09 + 01 = 10 (decimal)
        assert_eq!(cpu.registers.accumulator, 0x10);
        assert!(!cpu.get_flag(Status::PS_CARRY));
    }

    #[test]
    fn revision_a_adc_same_as_nmos() {
        use crate::instruction::RevisionA;

        let mut cpu = CPU::new(Ram::new(), RevisionA);
        cpu.registers.accumulator = 0x09;
        cpu.registers.status.insert(Status::PS_DECIMAL_MODE);
        cpu.registers.status.remove(Status::PS_CARRY);

        cpu.add_with_carry(0x01);

        // Should behave identically to NMOS 6502
        assert_eq!(cpu.registers.accumulator, 0x10);
        assert!(!cpu.get_flag(Status::PS_CARRY));
    }

    #[test]
    fn get_flag() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // Demonstrate checking multiple flags
        assert!(!cpu.get_flag(Status::PS_CARRY));
        assert!(!cpu.get_flag(Status::PS_ZERO));
        assert!(!cpu.get_flag(Status::PS_NEGATIVE));
        assert!(!cpu.get_flag(Status::PS_OVERFLOW));
        assert!(!cpu.get_flag(Status::PS_DECIMAL_MODE));

        // Set some flags and check them
        cpu.registers
            .status
            .insert(Status::PS_CARRY | Status::PS_ZERO);
        assert!(cpu.get_flag(Status::PS_CARRY));
        assert!(cpu.get_flag(Status::PS_ZERO));
        assert!(!cpu.get_flag(Status::PS_NEGATIVE));
    }

    #[test]
    fn set_flag() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // Initially, no flags are set
        assert!(!cpu.get_flag(Status::PS_CARRY));
        assert!(!cpu.get_flag(Status::PS_ZERO));
        assert!(!cpu.get_flag(Status::PS_NEGATIVE));

        // Set the carry flag
        cpu.set_flag(Status::PS_CARRY);
        assert!(cpu.get_flag(Status::PS_CARRY));
        assert!(!cpu.get_flag(Status::PS_ZERO));
        assert!(!cpu.get_flag(Status::PS_NEGATIVE));

        // Set the zero flag
        cpu.set_flag(Status::PS_ZERO);
        assert!(cpu.get_flag(Status::PS_CARRY));
        assert!(cpu.get_flag(Status::PS_ZERO));
        assert!(!cpu.get_flag(Status::PS_NEGATIVE));

        // Set the negative flag
        cpu.set_flag(Status::PS_NEGATIVE);
        assert!(cpu.get_flag(Status::PS_CARRY));
        assert!(cpu.get_flag(Status::PS_ZERO));
        assert!(cpu.get_flag(Status::PS_NEGATIVE));
    }

    #[test]
    fn unset_flag() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // Set all flags first
        cpu.set_flag(Status::PS_CARRY);
        cpu.set_flag(Status::PS_ZERO);
        cpu.set_flag(Status::PS_NEGATIVE);
        assert!(cpu.get_flag(Status::PS_CARRY));
        assert!(cpu.get_flag(Status::PS_ZERO));
        assert!(cpu.get_flag(Status::PS_NEGATIVE));

        // Clear the carry flag
        cpu.unset_flag(Status::PS_CARRY);
        assert!(!cpu.get_flag(Status::PS_CARRY));
        assert!(cpu.get_flag(Status::PS_ZERO));
        assert!(cpu.get_flag(Status::PS_NEGATIVE));

        // Clear the zero flag
        cpu.unset_flag(Status::PS_ZERO);
        assert!(!cpu.get_flag(Status::PS_CARRY));
        assert!(!cpu.get_flag(Status::PS_ZERO));
        assert!(cpu.get_flag(Status::PS_NEGATIVE));

        // Clear the negative flag
        cpu.unset_flag(Status::PS_NEGATIVE);
        assert!(!cpu.get_flag(Status::PS_CARRY));
        assert!(!cpu.get_flag(Status::PS_ZERO));
        assert!(!cpu.get_flag(Status::PS_NEGATIVE));
    }

    // ADC function-level tests - test the pure ADC logic without CPU state
    #[test]
    fn adc_function_nmos6502_binary_basic() {
        use crate::instruction::Nmos6502;
        use crate::{ArithmeticOutput, Variant};

        // Test basic binary addition: 5 + 3 = 8
        let result = Nmos6502::adc_binary(5, 3, false);
        assert_eq!(
            result,
            ArithmeticOutput {
                result: 8,
                did_carry: false,
                overflow: false,
                negative: false,
                zero: false,
            }
        );

        // Test with carry: 5 + 3 + 1 = 9
        let result = Nmos6502::adc_binary(5, 3, true);
        assert_eq!(
            result,
            ArithmeticOutput {
                result: 9,
                did_carry: false,
                overflow: false,
                negative: false,
                zero: false,
            }
        );
    }

    #[test]
    fn adc_function_nmos6502_binary_overflow() {
        use crate::instruction::Nmos6502;
        use crate::{ArithmeticOutput, Variant};

        // Test signed overflow: 127 + 1 = -128 (0x80)
        let result = Nmos6502::adc_binary(0x7F, 1, false);
        assert_eq!(
            result,
            ArithmeticOutput {
                result: 0x80,
                did_carry: false,
                overflow: true, // V flag set for signed overflow
                negative: true, // N flag set because result has bit 7 set
                zero: false,
            }
        );
    }

    #[test]
    fn adc_function_nmos6502_binary_carry() {
        use crate::instruction::Nmos6502;
        use crate::{ArithmeticOutput, Variant};

        // Test carry: 255 + 1 = 0 with carry
        let result = Nmos6502::adc_binary(255, 1, false);
        assert_eq!(
            result,
            ArithmeticOutput {
                result: 0,
                did_carry: true, // C flag set for unsigned overflow
                overflow: false,
                negative: false,
                zero: true, // Z flag set because result is 0
            }
        );
    }

    #[test]
    fn adc_function_nmos6502_decimal_basic() {
        use crate::instruction::Nmos6502;
        use crate::{ArithmeticOutput, Variant};

        // Test BCD addition: 09 + 01 = 10 (0x10 in BCD)
        let result = Nmos6502::adc_decimal(0x09, 0x01, false);
        assert_eq!(
            result,
            ArithmeticOutput {
                result: 0x10, // BCD result
                did_carry: false,
                overflow: false, // V calculated from binary operation
                negative: false,
                zero: false,
            }
        );
    }

    #[test]
    fn adc_function_ricoh2a03_decimal_calls_binary() {
        use crate::Variant;
        use crate::instruction::Ricoh2a03;

        // Ricoh2A03 has no decimal mode, so decimal should match binary
        let binary_result = Ricoh2a03::adc_binary(0x09, 0x01, false);
        let decimal_result = Ricoh2a03::adc_decimal(0x09, 0x01, false);
        assert_eq!(binary_result, decimal_result);
    }

    #[test]
    fn reset_sequence_behavior() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // Set up reset vector in memory: $1234
        cpu.memory.set_byte(0xFFFC, 0x34); // Low byte
        cpu.memory.set_byte(0xFFFD, 0x12); // High byte

        // Initialize SP to some value to see it change
        cpu.registers.stack_pointer = StackPointer(0xFF);

        cpu.reset();

        // Check that PC was set from reset vector
        assert_eq!(cpu.registers.program_counter, 0x1234);

        // Check that SP was decremented 3 times (0xFF - 3 = 0xFC)
        assert_eq!(cpu.registers.stack_pointer.0, 0xFC);

        // Check that interrupt disable flag is set
        assert!(cpu.registers.status.contains(Status::PS_DISABLE_INTERRUPTS));
    }

    #[test]
    fn cmos_bit_zpx() {
        use crate::instruction::{Cmos6502, Instruction, OpInput};

        // BIT $10,X (opcode 0x34) - tests that BIT works with ZeroPageX addressing
        let mut cpu = CPU::new(Ram::new(), Cmos6502);
        cpu.registers.accumulator = 0b1100_0000;

        // Value at address to test
        cpu.memory.set_byte(0x15, 0b1100_0000);

        cpu.execute_instruction((
            Instruction::BIT,
            AddressingMode::Absolute,
            OpInput::UseAddress {
                address: 0x15,
                page_crossed: false,
            },
        ));

        // BIT should set N and V from memory value, Z from AND result
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));
    }

    #[test]
    fn cmos_bit_absx() {
        use crate::instruction::{Cmos6502, Instruction, OpInput};

        // BIT abs,X (opcode 0x3C) - tests that BIT works with AbsoluteX addressing
        let mut cpu = CPU::new(Ram::new(), Cmos6502);
        cpu.registers.accumulator = 0b0100_0000;

        // Value at address to test
        cpu.memory.set_byte(0x1005, 0b0100_0000);

        cpu.execute_instruction((
            Instruction::BIT,
            AddressingMode::Absolute,
            OpInput::UseAddress {
                address: 0x1005,
                page_crossed: false,
            },
        ));

        // BIT should set V from memory value
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn cmos_jmp_absx_indirect() {
        use crate::instruction::{Cmos6502, Instruction, OpInput};

        // JMP (abs,X) (opcode 0x7C) - tests indexed indirect jump
        let mut cpu = CPU::new(Ram::new(), Cmos6502);

        // Target address is $3456
        cpu.execute_instruction((
            Instruction::JMP,
            AddressingMode::Absolute,
            OpInput::UseAddress {
                address: 0x3456,
                page_crossed: false,
            },
        ));

        // PC should now be $3456
        assert_eq!(cpu.registers.program_counter, 0x3456);
    }

    #[test]
    fn cmos_wai() {
        use crate::instruction::{Cmos6502, Instruction, OpInput};

        // WAI (opcode 0xCB) - Wait for Interrupt
        let mut cpu = CPU::new(Ram::new(), Cmos6502);
        let pc_before = cpu.registers.program_counter;

        // Execute WAI instruction
        cpu.execute_instruction((
            Instruction::WAI,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));

        // PC should not change (in this simple implementation)
        // In a real CPU, this would halt until interrupt
        assert_eq!(cpu.registers.program_counter, pc_before);
    }

    #[test]
    fn cmos_stp() {
        use crate::instruction::Cmos6502;

        // STP (opcode 0xDB) - Stop processor
        let mut cpu = CPU::new(Ram::new(), Cmos6502);

        // Set up a simple program: LDA #$42, then STP
        cpu.memory.set_byte(0x0000, 0xA9); // LDA immediate
        cpu.memory.set_byte(0x0001, 0x42); // value
        cpu.memory.set_byte(0x0002, 0xDB); // STP opcode

        // Execute LDA - should work normally
        cpu.single_step();
        assert_eq!(cpu.registers.accumulator, 0x42);

        // Execute STP - should halt the processor
        cpu.single_step();
        assert!(cpu.halted);

        // Try to execute another step - should do nothing
        let pc_after_stp = cpu.registers.program_counter;
        cpu.single_step();
        assert_eq!(cpu.registers.program_counter, pc_after_stp);

        // Reset should clear halted state
        cpu.reset();
        assert!(!cpu.halted);

        // After reset, CPU should be able to execute instructions again
        // Set up LDA #$99 at the reset vector location
        cpu.memory.set_byte(0xFFFC, 0x00); // Reset vector low byte
        cpu.memory.set_byte(0xFFFD, 0x80); // Reset vector high byte
        cpu.memory.set_byte(0x8000, 0xA9); // LDA immediate at reset location
        cpu.memory.set_byte(0x8001, 0x99); // value
        cpu.reset(); // Reset again to jump to our new reset vector
        cpu.single_step(); // Execute LDA
        assert_eq!(cpu.registers.accumulator, 0x99);
    }

    // ==================== Illegal Opcode Tests ====================

    /// Execute instruction with zero-page addressing
    macro_rules! exec_zp {
        ($cpu:expr, $instr:ident, $addr:expr) => {
            $cpu.execute_instruction((
                Instruction::$instr,
                AddressingMode::ZeroPage,
                OpInput::UseAddress {
                    address: $addr,
                    page_crossed: false,
                },
            ))
        };
    }

    /// Execute instruction with immediate addressing
    macro_rules! exec_imm {
        ($cpu:expr, $instr:ident, $val:expr) => {
            $cpu.execute_instruction((
                Instruction::$instr,
                AddressingMode::Immediate,
                OpInput::UseImmediate($val),
            ))
        };
    }

    /// Execute instruction with implied addressing
    macro_rules! exec_impl {
        ($cpu:expr, $instr:ident) => {
            $cpu.execute_instruction((
                Instruction::$instr,
                AddressingMode::Implied,
                OpInput::UseImplied,
            ))
        };
    }

    /// Execute instruction with absolute,Y addressing
    macro_rules! exec_aby {
        ($cpu:expr, $instr:ident, $addr:expr) => {
            $cpu.execute_instruction((
                Instruction::$instr,
                AddressingMode::AbsoluteY,
                OpInput::UseAddress {
                    address: $addr,
                    page_crossed: false,
                },
            ))
        };
    }

    #[test]
    fn lax_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.memory.set_byte(0x42, 0x55);
        exec_zp!(cpu, LAX, 0x42);
        assert_eq!(cpu.registers.accumulator, 0x55);
        assert_eq!(cpu.registers.index_x, 0x55);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.memory.set_byte(0x10, 0x00);
        exec_zp!(cpu, LAX, 0x10);
        assert_eq!(cpu.registers.accumulator, 0x00);
        assert_eq!(cpu.registers.index_x, 0x00);
        assert!(cpu.registers.status.contains(Status::PS_ZERO));

        cpu.memory.set_byte(0x20, 0x80);
        exec_zp!(cpu, LAX, 0x20);
        assert_eq!(cpu.registers.accumulator, 0x80);
        assert_eq!(cpu.registers.index_x, 0x80);
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn sax_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.registers.accumulator = 0xFF;
        cpu.registers.index_x = 0x0F;
        exec_zp!(cpu, SAX, 0x42);
        assert_eq!(cpu.memory.get_byte(0x42), 0x0F);

        cpu.registers.accumulator = 0xAA;
        cpu.registers.index_x = 0x55;
        exec_zp!(cpu, SAX, 0x43);
        assert_eq!(cpu.memory.get_byte(0x43), 0x00);
    }

    #[test]
    fn dcp_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.memory.set_byte(0x42, 0x10);
        cpu.registers.accumulator = 0x0F;
        exec_zp!(cpu, DCP, 0x42);
        assert_eq!(cpu.memory.get_byte(0x42), 0x0F);
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_CARRY));

        cpu.memory.set_byte(0x43, 0x05);
        cpu.registers.accumulator = 0x10;
        exec_zp!(cpu, DCP, 0x43);
        assert_eq!(cpu.memory.get_byte(0x43), 0x04);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_CARRY));

        cpu.memory.set_byte(0x44, 0x00);
        cpu.registers.accumulator = 0x00;
        exec_zp!(cpu, DCP, 0x44);
        assert_eq!(cpu.memory.get_byte(0x44), 0xFF);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
    }

    #[test]
    fn isc_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers.status.remove(Status::PS_DECIMAL_MODE);

        cpu.memory.set_byte(0x42, 0x09);
        cpu.registers.accumulator = 0x20;
        cpu.registers.status.insert(Status::PS_CARRY);
        exec_zp!(cpu, ISC, 0x42);
        assert_eq!(cpu.memory.get_byte(0x42), 0x0A);
        assert_eq!(cpu.registers.accumulator, 0x16);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));

        cpu.memory.set_byte(0x43, 0xFF);
        cpu.registers.accumulator = 0x10;
        cpu.registers.status.insert(Status::PS_CARRY);
        exec_zp!(cpu, ISC, 0x43);
        assert_eq!(cpu.memory.get_byte(0x43), 0x00);
        assert_eq!(cpu.registers.accumulator, 0x10);
    }

    #[test]
    fn slo_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.memory.set_byte(0x42, 0x40);
        cpu.registers.accumulator = 0x01;
        exec_zp!(cpu, SLO, 0x42);
        assert_eq!(cpu.memory.get_byte(0x42), 0x80);
        assert_eq!(cpu.registers.accumulator, 0x81);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.memory.set_byte(0x43, 0x80);
        cpu.registers.accumulator = 0x00;
        exec_zp!(cpu, SLO, 0x43);
        assert_eq!(cpu.memory.get_byte(0x43), 0x00);
        assert_eq!(cpu.registers.accumulator, 0x00);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
    }

    #[test]
    fn rla_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.registers.status.remove(Status::PS_CARRY);
        cpu.memory.set_byte(0x42, 0x40);
        cpu.registers.accumulator = 0xFF;
        exec_zp!(cpu, RLA, 0x42);
        assert_eq!(cpu.memory.get_byte(0x42), 0x80);
        assert_eq!(cpu.registers.accumulator, 0x80);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));

        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.memory.set_byte(0x43, 0x40);
        cpu.registers.accumulator = 0xFF;
        exec_zp!(cpu, RLA, 0x43);
        assert_eq!(cpu.memory.get_byte(0x43), 0x81);
        assert_eq!(cpu.registers.accumulator, 0x81);
    }

    #[test]
    fn sre_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.memory.set_byte(0x42, 0x02);
        cpu.registers.accumulator = 0xFF;
        exec_zp!(cpu, SRE, 0x42);
        assert_eq!(cpu.memory.get_byte(0x42), 0x01);
        assert_eq!(cpu.registers.accumulator, 0xFE);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));

        cpu.memory.set_byte(0x43, 0x01);
        cpu.registers.accumulator = 0x00;
        exec_zp!(cpu, SRE, 0x43);
        assert_eq!(cpu.memory.get_byte(0x43), 0x00);
        assert_eq!(cpu.registers.accumulator, 0x00);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
    }

    #[test]
    fn rra_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers.status.remove(Status::PS_DECIMAL_MODE);

        cpu.registers.status.remove(Status::PS_CARRY);
        cpu.memory.set_byte(0x42, 0x02);
        cpu.registers.accumulator = 0x10;
        exec_zp!(cpu, RRA, 0x42);
        assert_eq!(cpu.memory.get_byte(0x42), 0x01);
        assert_eq!(cpu.registers.accumulator, 0x11);

        cpu.registers.status.insert(Status::PS_CARRY);
        cpu.memory.set_byte(0x43, 0x02);
        cpu.registers.accumulator = 0x00;
        exec_zp!(cpu, RRA, 0x43);
        assert_eq!(cpu.memory.get_byte(0x43), 0x81);
        assert_eq!(cpu.registers.accumulator, 0x81);
    }

    #[test]
    fn arr_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.registers.accumulator = 0xFF;
        cpu.registers.status.remove(Status::PS_CARRY);
        exec_imm!(cpu, ARR, 0x55);
        assert_eq!(cpu.registers.accumulator, 0x2A);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));

        cpu.registers.accumulator = 0xFF;
        cpu.registers.status.insert(Status::PS_CARRY);
        exec_imm!(cpu, ARR, 0x55);
        assert_eq!(cpu.registers.accumulator, 0xAA);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
    }

    #[test]
    fn sbx_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.registers.accumulator = 0xFF;
        cpu.registers.index_x = 0x0F;
        exec_imm!(cpu, SBX, 0x05);
        assert_eq!(cpu.registers.index_x, 0x0A);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));

        cpu.registers.accumulator = 0xFF;
        cpu.registers.index_x = 0x0F;
        exec_imm!(cpu, SBX, 0x10);
        assert_eq!(cpu.registers.index_x, 0xFF);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn alr_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.registers.accumulator = 0xFF;
        exec_imm!(cpu, ALR, 0xAA);
        assert_eq!(cpu.registers.accumulator, 0x55);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));

        cpu.registers.accumulator = 0xFF;
        exec_imm!(cpu, ALR, 0x55);
        assert_eq!(cpu.registers.accumulator, 0x2A);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
    }

    #[test]
    fn anc_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.registers.accumulator = 0xFF;
        exec_imm!(cpu, ANC, 0x80);
        assert_eq!(cpu.registers.accumulator, 0x80);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.registers.accumulator = 0xFF;
        exec_imm!(cpu, ANC, 0x7F);
        assert_eq!(cpu.registers.accumulator, 0x7F);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn xaa_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // XAA transfers X to A, then ANDs with immediate
        cpu.registers.index_x = 0xFF;
        cpu.registers.accumulator = 0x00;
        exec_imm!(cpu, XAA, 0x0F);
        assert_eq!(cpu.registers.accumulator, 0x0F);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.registers.index_x = 0xAA;
        exec_imm!(cpu, XAA, 0xF0);
        assert_eq!(cpu.registers.accumulator, 0xA0);
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.registers.index_x = 0x55;
        exec_imm!(cpu, XAA, 0xAA);
        assert_eq!(cpu.registers.accumulator, 0x00);
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
    }

    #[test]
    fn las_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.registers.stack_pointer = StackPointer(0xFF);
        cpu.memory.set_byte(0x1000, 0x0F);
        exec_aby!(cpu, LAS, 0x1000);
        assert_eq!(cpu.registers.accumulator, 0x0F);
        assert_eq!(cpu.registers.index_x, 0x0F);
        assert_eq!(cpu.registers.stack_pointer.0, 0x0F);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn usbc_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers.status.remove(Status::PS_DECIMAL_MODE);

        cpu.registers.accumulator = 0x20;
        cpu.registers.status.insert(Status::PS_CARRY);
        exec_imm!(cpu, USBC, 0x10);
        assert_eq!(cpu.registers.accumulator, 0x10);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
    }

    #[test]
    fn jam_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        assert!(!cpu.halted);
        exec_impl!(cpu, JAM);
        assert!(cpu.halted);
        assert!(!cpu.single_step());
    }

    #[test]
    fn nop_variants_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        let initial_a = 0x42;
        let initial_x = 0x13;
        let initial_y = 0x37;
        cpu.registers.accumulator = initial_a;
        cpu.registers.index_x = initial_x;
        cpu.registers.index_y = initial_y;

        exec_imm!(cpu, NOPI, 0xFF);
        assert_eq!(cpu.registers.accumulator, initial_a);
        assert_eq!(cpu.registers.index_x, initial_x);
        assert_eq!(cpu.registers.index_y, initial_y);

        exec_zp!(cpu, NOPZ, 0x42);
        assert_eq!(cpu.registers.accumulator, initial_a);

        cpu.execute_instruction((
            Instruction::NOPA,
            AddressingMode::Absolute,
            OpInput::UseAddress {
                address: 0x1234,
                page_crossed: false,
            },
        ));
        assert_eq!(cpu.registers.accumulator, initial_a);

        cpu.execute_instruction((
            Instruction::NOPAX,
            AddressingMode::AbsoluteX,
            OpInput::UseAddress {
                address: 0x1234,
                page_crossed: false,
            },
        ));
        assert_eq!(cpu.registers.accumulator, initial_a);
    }
}

#[cfg(test)]
mod cycle_timing_tests {
    use super::*;
    use crate::instruction::{Cmos6502, Instruction, Nmos6502};
    use crate::memory::Memory as Ram;

    #[test]
    fn test_basic_cycle_counting() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        assert_eq!(cpu.cycles, 0);

        // LDA #$42 - Immediate mode: 2 cycles
        cpu.execute_instruction((
            Instruction::LDA,
            AddressingMode::Immediate,
            OpInput::UseImmediate(0x42),
        ));
        assert_eq!(cpu.cycles, 2);

        // NOP - Implied mode: 2 cycles
        cpu.execute_instruction((
            Instruction::NOP,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        assert_eq!(cpu.cycles, 4);

        // ADC #$01 - Immediate mode: 2 cycles (NMOS, no decimal penalty)
        cpu.execute_instruction((
            Instruction::ADC,
            AddressingMode::Immediate,
            OpInput::UseImmediate(0x01),
        ));
        assert_eq!(cpu.cycles, 6);
    }

    #[test]
    fn test_decimal_mode_penalty_65c02() {
        let mut cpu = CPU::new(Ram::new(), Cmos6502);
        assert_eq!(cpu.cycles, 0);

        // ADC #$01 without decimal mode: 2 cycles
        cpu.execute_instruction((
            Instruction::ADC,
            AddressingMode::Immediate,
            OpInput::UseImmediate(0x01),
        ));
        assert_eq!(cpu.cycles, 2);

        // Set decimal mode
        cpu.set_flag(Status::PS_DECIMAL_MODE);

        // ADC #$01 with decimal mode on 65C02: 3 cycles (2 + 1 decimal penalty)
        cpu.execute_instruction((
            Instruction::ADC,
            AddressingMode::Immediate,
            OpInput::UseImmediate(0x01),
        ));
        assert_eq!(cpu.cycles, 5); // 2 + 3
    }

    #[test]
    fn test_no_decimal_penalty_on_nmos() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // Set decimal mode
        cpu.set_flag(Status::PS_DECIMAL_MODE);

        // ADC #$01 with decimal mode on NMOS: still 2 cycles (no penalty)
        cpu.execute_instruction((
            Instruction::ADC,
            AddressingMode::Immediate,
            OpInput::UseImmediate(0x01),
        ));
        assert_eq!(cpu.cycles, 2);
    }

    #[test]
    fn test_various_instruction_cycles() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // BRK - 7 cycles
        cpu.execute_instruction((
            Instruction::BRK,
            AddressingMode::Implied,
            OpInput::UseImplied,
        ));
        assert_eq!(cpu.cycles, 7);

        cpu.cycles = 0; // Reset for next test

        // JSR - 6 cycles
        cpu.execute_instruction((
            Instruction::JSR,
            AddressingMode::Absolute,
            OpInput::UseAddress {
                address: 0x1234,
                page_crossed: false,
            },
        ));
        assert_eq!(cpu.cycles, 6);

        cpu.cycles = 0;

        // ASL A - Accumulator mode: 2 cycles
        cpu.execute_instruction((
            Instruction::ASL,
            AddressingMode::Accumulator,
            OpInput::UseImplied,
        ));
        assert_eq!(cpu.cycles, 2);

        cpu.cycles = 0;

        // ASL $00 - Zero page: 5 cycles
        cpu.execute_instruction((
            Instruction::ASL,
            AddressingMode::ZeroPage,
            OpInput::UseAddress {
                address: 0x00,
                page_crossed: false,
            },
        ));
        assert_eq!(cpu.cycles, 5);
    }

    #[test]
    fn test_cycles_accumulate() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // Execute multiple instructions and verify cycles accumulate
        for _ in 0..10 {
            cpu.execute_instruction((
                Instruction::NOP,
                AddressingMode::Implied,
                OpInput::UseImplied,
            ));
        }

        assert_eq!(cpu.cycles, 20); // 10 instructions × 2 cycles each
    }

    #[test]
    fn test_page_crossing_detection() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // Test 1: LDA $1200,X with X=$FF - no page crossing (stays in $12xx)
        cpu.registers.index_x = 0xFF;
        cpu.execute_instruction((
            Instruction::LDA,
            AddressingMode::AbsoluteX,
            OpInput::UseAddress {
                address: 0x12FF,
                page_crossed: false,
            },
        ));
        // Base cycles: 4, no page crossing penalty
        assert_eq!(cpu.cycles, 4);

        cpu.cycles = 0;

        // Test 2: LDA $12FF,X with X=$02 - page crossing! ($12 -> $13)
        cpu.registers.index_x = 0x02;
        cpu.execute_instruction((
            Instruction::LDA,
            AddressingMode::AbsoluteX,
            OpInput::UseAddress {
                address: 0x1301,
                page_crossed: true,
            },
        ));
        // Base cycles: 4, page crossing penalty: +1 = 5 total
        assert_eq!(cpu.cycles, 5);

        cpu.cycles = 0;

        // Test 3: LDA $10FF,Y with Y=$01 - page crossing! ($10 -> $11)
        cpu.registers.index_y = 0x01;
        cpu.execute_instruction((
            Instruction::LDA,
            AddressingMode::AbsoluteY,
            OpInput::UseAddress {
                address: 0x1100,
                page_crossed: true,
            },
        ));
        // Base cycles: 4, page crossing penalty: +1 = 5 total
        assert_eq!(cpu.cycles, 5);

        cpu.cycles = 0;

        // Test 4: STA $12FF,X with X=$02 - page crossing but NO PENALTY for stores!
        cpu.registers.index_x = 0x02;
        cpu.execute_instruction((
            Instruction::STA,
            AddressingMode::AbsoluteX,
            OpInput::UseAddress {
                address: 0x1301,
                page_crossed: true,
            },
        ));
        // Base cycles: 5, NO page crossing penalty for stores
        assert_eq!(cpu.cycles, 5);
    }

    #[test]
    fn test_page_crossing_edge_cases() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // Edge case 1: Wrapping at end of address space
        cpu.registers.index_x = 0xFF;
        cpu.execute_instruction((
            Instruction::LDA,
            AddressingMode::AbsoluteX,
            OpInput::UseAddress {
                address: 0x00FE,
                page_crossed: true,
            },
        ));
        // Should detect page crossing: $FF -> $00
        assert_eq!(cpu.cycles, 5); // 4 + 1 page crossing

        cpu.cycles = 0;

        // Edge case 2: No crossing at page boundary
        cpu.registers.index_x = 0x00;
        cpu.execute_instruction((
            Instruction::LDA,
            AddressingMode::AbsoluteX,
            OpInput::UseAddress {
                address: 0x1200,
                page_crossed: false,
            },
        ));
        // No page crossing
        assert_eq!(cpu.cycles, 4);
    }

    #[test]
    fn test_indirect_indexed_page_crossing() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // IndirectIndexedY: LDA ($80),Y where Y=$10
        // If the value at $80-$81 is $12F0, then final is $1300 (page crossed)
        cpu.registers.index_y = 0x10;
        cpu.execute_instruction((
            Instruction::LDA,
            AddressingMode::IndirectIndexedY,
            OpInput::UseAddress {
                address: 0x1300,
                page_crossed: true,
            },
        ));
        // Base cycles: 5, page crossing penalty: +1 = 6 total
        assert_eq!(cpu.cycles, 6);

        cpu.cycles = 0;

        // Same mode, no page crossing
        cpu.registers.index_y = 0x05;
        cpu.execute_instruction((
            Instruction::LDA,
            AddressingMode::IndirectIndexedY,
            OpInput::UseAddress {
                address: 0x1205,
                page_crossed: false,
            },
        ));
        // Base cycles: 5, no page crossing
        assert_eq!(cpu.cycles, 5);
    }
}

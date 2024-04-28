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

use crate::instruction::{AddressingMode, DecodedInstr, Instruction, OpInput};
use crate::memory::Bus;
use crate::Variant;

use crate::registers::{Registers, StackPointer, Status, StatusArgs};

fn address_from_bytes(lo: u8, hi: u8) -> u16 {
    u16::from(lo) + (u16::from(hi) << 8usize)
}

#[derive(Clone)]
pub struct CPU<M, V>
where
    M: Bus,
    V: Variant,
{
    pub registers: Registers,
    pub memory: M,
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
            variant: core::marker::PhantomData::<V>,
        }
    }

    pub fn reset(&mut self) {
        //TODO: should read some bytes from the stack and also get the PC from the reset vector
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
                        OpInput::UseAddress(u16::from(slice[0]))
                    }
                    AddressingMode::ZeroPageX => {
                        // Use [u8, ..1] from instruction
                        // Add to X register (as u8 -- the final address is in 0-page)
                        // (Output: an 8-bit zero-page address)
                        OpInput::UseAddress(u16::from(slice[0].wrapping_add(x)))
                    }
                    AddressingMode::ZeroPageY => {
                        // Use [u8, ..1] from instruction
                        // Add to Y register (as u8 -- the final address is in 0-page)
                        // (Output: an 8-bit zero-page address)
                        OpInput::UseAddress(u16::from(slice[0].wrapping_add(y)))
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
                        OpInput::UseAddress(address_from_bytes(slice[0], slice[1]))
                    }
                    AddressingMode::AbsoluteX => {
                        // Use [u8, ..2] from instruction as address, add X
                        // (Output: a 16-bit address)
                        OpInput::UseAddress(
                            address_from_bytes(slice[0], slice[1]).wrapping_add(x.into()),
                        )
                    }
                    AddressingMode::AbsoluteY => {
                        // Use [u8, ..2] from instruction as address, add Y
                        // (Output: a 16-bit address)
                        OpInput::UseAddress(
                            address_from_bytes(slice[0], slice[1]).wrapping_add(y.into()),
                        )
                    }
                    AddressingMode::Indirect => {
                        // Use [u8, ..2] from instruction as an address. Interpret the
                        // two bytes starting at that address as an address.
                        // (Output: a 16-bit address)
                        // TODO: If the pointer ends in 0xff, then incrementing it would propagate
                        // the carry to the high byte of the pointer. This incurs a cost of one
                        // machine cycle on the real 65C02, which is not implemented here.
                        let slice = read_address(memory, address_from_bytes(slice[0], slice[1]));
                        OpInput::UseAddress(address_from_bytes(slice[0], slice[1]))
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
                        OpInput::UseAddress(address_from_bytes(
                            low_byte_of_target,
                            high_byte_of_target,
                        ))
                    }
                    AddressingMode::IndexedIndirectX => {
                        // Use [u8, ..1] from instruction
                        // Add to X register with 0-page wraparound, like ZeroPageX.
                        // This is where the absolute (16-bit) target address is stored.
                        // (Output: a 16-bit address)
                        let start = slice[0].wrapping_add(x);
                        let slice = read_address(memory, u16::from(start));
                        OpInput::UseAddress(address_from_bytes(slice[0], slice[1]))
                    }
                    AddressingMode::IndirectIndexedY => {
                        // Use [u8, ..1] from instruction
                        // This is where the absolute (16-bit) target address is stored.
                        // Add Y register to this address to get the final address
                        // (Output: a 16-bit address)
                        let start = slice[0];
                        let slice = read_address(memory, u16::from(start));
                        OpInput::UseAddress(
                            address_from_bytes(slice[0], slice[1]).wrapping_add(y.into()),
                        )
                    }
                    AddressingMode::ZeroPageIndirect => {
                        // Use [u8, ..1] from instruction
                        // This is where the absolute (16-bit) target address is stored.
                        // (Output: a 16-bit address)
                        let start = slice[0];
                        let slice = read_address(memory, u16::from(start));
                        OpInput::UseAddress(address_from_bytes(slice[0], slice[1]))
                    }
                };

                // Increment program counter
                self.registers.program_counter =
                    self.registers.program_counter.wrapping_add(num_bytes);

                Some((instr, am_out))
            }
            _ => None,
        }
    }

    #[allow(clippy::too_many_lines)]
    pub fn execute_instruction(&mut self, decoded_instr: DecodedInstr) {
        match decoded_instr {
            (Instruction::ADC, OpInput::UseImmediate(val)) => {
                log::debug!("add with carry immediate: {}", val);
                self.add_with_carry(val);
            }
            (Instruction::ADC, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                log::debug!("add with carry. address: {:?}. value: {}", addr, val);
                self.add_with_carry(val);
            }
            (Instruction::ADCnd, OpInput::UseImmediate(val)) => {
                log::debug!("add with carry immediate: {}", val);
                self.add_with_no_decimal(val);
            }
            (Instruction::ADCnd, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                log::debug!("add with carry. address: {:?}. value: {}", addr, val);
                self.add_with_no_decimal(val);
            }

            (Instruction::AND, OpInput::UseImmediate(val)) => {
                self.and(val);
            }
            (Instruction::AND, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                self.and(val);
            }

            (Instruction::ASL, OpInput::UseImplied) => {
                // Accumulator mode
                let mut val = self.registers.accumulator;
                CPU::<M, V>::shift_left_with_flags(&mut val, &mut self.registers.status);
                self.registers.accumulator = val;
            }
            (Instruction::ASL, OpInput::UseAddress(addr)) => {
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

            (Instruction::BIT, OpInput::UseAddress(addr)) => {
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
                log::debug!("branch if minus relative. address: {:?}", addr);
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
                self.registers.status.or(Status::PS_DISABLE_INTERRUPTS);
            }

            (Instruction::BRKcld, OpInput::UseImplied) => {
                for b in self.registers.program_counter.wrapping_sub(1).to_be_bytes() {
                    self.push_on_stack(b);
                }
                self.push_on_stack(self.registers.status.bits());
                let pcl = self.memory.get_byte(0xfffe);
                let pch = self.memory.get_byte(0xffff);
                self.jump((u16::from(pch) << 8) | u16::from(pcl));
                self.registers.status.or(Status::PS_DISABLE_INTERRUPTS);
                self.registers.status.and(!Status::PS_DECIMAL_MODE);
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
                self.registers.status.and(!Status::PS_CARRY);
            }
            (Instruction::CLD, OpInput::UseImplied) => {
                self.registers.status.and(!Status::PS_DECIMAL_MODE);
            }
            (Instruction::CLI, OpInput::UseImplied) => {
                self.registers.status.and(!Status::PS_DISABLE_INTERRUPTS);
            }
            (Instruction::CLV, OpInput::UseImplied) => {
                self.registers.status.and(!Status::PS_OVERFLOW);
            }

            (Instruction::CMP, OpInput::UseImmediate(val)) => {
                self.compare_with_a_register(val);
            }
            (Instruction::CMP, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                self.compare_with_a_register(val);
            }

            (Instruction::CPX, OpInput::UseImmediate(val)) => {
                self.compare_with_x_register(val);
            }
            (Instruction::CPX, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                self.compare_with_x_register(val);
            }

            (Instruction::CPY, OpInput::UseImmediate(val)) => {
                self.compare_with_y_register(val);
            }
            (Instruction::CPY, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                self.compare_with_y_register(val);
            }

            (Instruction::DEC, OpInput::UseAddress(addr)) => {
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
            (Instruction::EOR, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                self.exclusive_or(val);
            }

            (Instruction::INC, OpInput::UseAddress(addr)) => {
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

            (Instruction::JMP, OpInput::UseAddress(addr)) => self.jump(addr),

            (Instruction::JSR, OpInput::UseAddress(addr)) => {
                for b in self.registers.program_counter.wrapping_sub(1).to_be_bytes() {
                    self.push_on_stack(b);
                }
                self.jump(addr);
            }

            (Instruction::LDA, OpInput::UseImmediate(val)) => {
                log::debug!("load A immediate: {}", val);
                self.load_accumulator(val);
            }
            (Instruction::LDA, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                log::debug!("load A. address: {:?}. value: {}", addr, val);
                self.load_accumulator(val);
            }

            (Instruction::LDX, OpInput::UseImmediate(val)) => {
                log::debug!("load X immediate: {}", val);
                self.load_x_register(val);
            }
            (Instruction::LDX, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                log::debug!("load X. address: {:?}. value: {}", addr, val);
                self.load_x_register(val);
            }

            (Instruction::LDY, OpInput::UseImmediate(val)) => {
                log::debug!("load Y immediate: {}", val);
                self.load_y_register(val);
            }
            (Instruction::LDY, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                log::debug!("load Y. address: {:?}. value: {}", addr, val);
                self.load_y_register(val);
            }

            (Instruction::LSR, OpInput::UseImplied) => {
                // Accumulator mode
                let mut val = self.registers.accumulator;
                CPU::<M, V>::shift_right_with_flags(&mut val, &mut self.registers.status);
                self.registers.accumulator = val;
            }
            (Instruction::LSR, OpInput::UseAddress(addr)) => {
                let mut operand: u8 = self.memory.get_byte(addr);
                CPU::<M, V>::shift_right_with_flags(&mut operand, &mut self.registers.status);
                self.memory.set_byte(addr, operand);
            }

            (Instruction::ORA, OpInput::UseImmediate(val)) => {
                self.inclusive_or(val);
            }
            (Instruction::ORA, OpInput::UseAddress(addr)) => {
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
                // Pull accumulator
                self.pull_from_stack();
                let val: u8 = self.fetch_from_stack();
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
                // Pull accumulator
                self.pull_from_stack();
                let val: u8 = self.fetch_from_stack();
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
                self.pull_from_stack();
                let val: u8 = self.fetch_from_stack();
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
                self.pull_from_stack();
                let val: u8 = self.fetch_from_stack();
                // The `truncate` here won't do anything because we have a
                // constant for the single unused flags bit. This probably
                // corresponds to the behavior of the 6502...? FIXME: verify
                self.registers.status = Status::from_bits_truncate(val);
            }

            (Instruction::ROL, OpInput::UseImplied) => {
                // Accumulator mode
                let mut val = self.registers.accumulator;
                CPU::<M, V>::rotate_left_with_flags(&mut val, &mut self.registers.status);
                self.registers.accumulator = val;
            }
            (Instruction::ROL, OpInput::UseAddress(addr)) => {
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
            (Instruction::ROR, OpInput::UseAddress(addr)) => {
                let mut operand: u8 = self.memory.get_byte(addr);
                CPU::<M, V>::rotate_right_with_flags(&mut operand, &mut self.registers.status);
                self.memory.set_byte(addr, operand);
            }
            (Instruction::RTI, OpInput::UseImplied) => {
                // Pull status
                self.pull_from_stack();
                let val: u8 = self.pull_from_stack();
                // The `truncate` here won't do anything because we have a
                // constant for the single unused flags bit. This probably
                // corresponds to the behavior of the 6502...? FIXME: verify
                self.registers.status = Status::from_bits_truncate(val);
                let pcl: u8 = self.pull_from_stack();
                let pch: u8 = self.fetch_from_stack();
                self.registers.program_counter = (u16::from(pch) << 8) | u16::from(pcl);
            }
            (Instruction::RTS, OpInput::UseImplied) => {
                self.pull_from_stack();
                let pcl: u8 = self.pull_from_stack();
                let pch: u8 = self.fetch_from_stack();
                self.registers.program_counter =
                    ((u16::from(pch) << 8) | u16::from(pcl)).wrapping_add(1);
            }

            (Instruction::SBC, OpInput::UseImmediate(val)) => {
                log::debug!("subtract with carry immediate: {}", val);
                self.subtract_with_carry(val);
            }
            (Instruction::SBC, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                log::debug!("subtract with carry. address: {:?}. value: {}", addr, val);
                self.subtract_with_carry(val);
            }

            (Instruction::SBCnd, OpInput::UseImmediate(val)) => {
                log::debug!("subtract with carry immediate: {}", val);
                self.subtract_with_no_decimal(val);
            }
            (Instruction::SBCnd, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                log::debug!("subtract with carry. address: {:?}. value: {}", addr, val);
                self.subtract_with_no_decimal(val);
            }

            (Instruction::SEC, OpInput::UseImplied) => {
                self.registers.status.or(Status::PS_CARRY);
            }
            (Instruction::SED, OpInput::UseImplied) => {
                self.registers.status.or(Status::PS_DECIMAL_MODE);
            }
            (Instruction::SEI, OpInput::UseImplied) => {
                self.registers.status.or(Status::PS_DISABLE_INTERRUPTS);
            }

            (Instruction::STA, OpInput::UseAddress(addr)) => {
                self.memory.set_byte(addr, self.registers.accumulator);
            }
            (Instruction::STX, OpInput::UseAddress(addr)) => {
                self.memory.set_byte(addr, self.registers.index_x);
            }
            (Instruction::STY, OpInput::UseAddress(addr)) => {
                self.memory.set_byte(addr, self.registers.index_y);
            }
            (Instruction::STZ, OpInput::UseAddress(addr)) => {
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
            (Instruction::TRB, OpInput::UseAddress(addr)) => {
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
            (Instruction::TSB, OpInput::UseAddress(addr)) => {
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

            (Instruction::NOP, OpInput::UseImplied) => {
                log::debug!("NOP instruction");
            }
            (_, _) => {
                log::debug!(
                    "attempting to execute unimplemented or invalid \
                     instruction"
                );
            }
        };
    }

    pub fn single_step(&mut self) {
        if let Some(decoded_instr) = self.fetch_next_and_decode() {
            self.execute_instruction(decoded_instr);
        }
    }

    pub fn run(&mut self) {
        while let Some(decoded_instr) = self.fetch_next_and_decode() {
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

    fn add_with_carry(&mut self, value: u8) {
        const fn decimal_adjust(result: u8) -> u8 {
            let bcd1: u8 = if (result & 0x0f) > 0x09 { 0x06 } else { 0x00 };

            let bcd2: u8 = if (result.wrapping_add(bcd1) & 0xf0) > 0x90 {
                0x60
            } else {
                0x00
            };

            result.wrapping_add(bcd1).wrapping_add(bcd2)
        }

        let a_before: u8 = self.registers.accumulator;
        let c_before: u8 = u8::from(self.registers.status.contains(Status::PS_CARRY));
        let a_after: u8 = a_before.wrapping_add(c_before).wrapping_add(value);

        debug_assert_eq!(a_after, a_before.wrapping_add(c_before).wrapping_add(value));

        let result: u8 = if self.registers.status.contains(Status::PS_DECIMAL_MODE) {
            decimal_adjust(a_after)
        } else {
            a_after
        };

        let did_carry = (result) < (a_before)
            || (a_after == 0 && c_before == 0x01)
            || (value == 0xff && c_before == 0x01);

        let did_overflow = (a_before > 127 && value > 127 && a_after < 128)
            || (a_before < 128 && value < 128 && a_after > 127);

        let mask = Status::PS_CARRY | Status::PS_OVERFLOW;

        self.registers.status.set_with_mask(
            mask,
            Status::new(StatusArgs {
                carry: did_carry,
                overflow: did_overflow,
                ..StatusArgs::none()
            }),
        );

        self.load_accumulator(result);

        log::debug!("accumulator: {}", self.registers.accumulator);
    }

    fn add_with_no_decimal(&mut self, value: u8) {
        let a_before: u8 = self.registers.accumulator;
        let c_before: u8 = u8::from(self.registers.status.contains(Status::PS_CARRY));
        let a_after: u8 = a_before.wrapping_add(c_before).wrapping_add(value);

        debug_assert_eq!(a_after, a_before.wrapping_add(c_before).wrapping_add(value));

        let result = a_after;

        let did_carry = (result) < (a_before)
            || (a_after == 0 && c_before == 0x01)
            || (value == 0xff && c_before == 0x01);

        let did_overflow = (a_before > 127 && value > 127 && a_after < 128)
            || (a_before < 128 && value < 128 && a_after > 127);

        let mask = Status::PS_CARRY | Status::PS_OVERFLOW;

        self.registers.status.set_with_mask(
            mask,
            Status::new(StatusArgs {
                carry: did_carry,
                overflow: did_overflow,
                ..StatusArgs::none()
            }),
        );

        self.load_accumulator(result);

        log::debug!("accumulator: {}", self.registers.accumulator);
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

    fn subtract_with_carry(&mut self, value: u8) {
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

        let bcd1: u8 = if (a_before & 0x0f).wrapping_sub(nc) < (value & 0x0f) {
            0x06
        } else {
            0x00
        };

        let bcd2: u8 = if (a_after.wrapping_sub(bcd1) & 0xf0) > 0x90 {
            0x60
        } else {
            0x00
        };

        let result: u8 = if self.registers.status.contains(Status::PS_DECIMAL_MODE) {
            a_after.wrapping_sub(bcd1).wrapping_sub(bcd2)
        } else {
            a_after
        };

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

    fn jump(&mut self, addr: u16) {
        self.registers.program_counter = addr;
    }

    fn branch_if_carry_clear(&mut self, addr: u16) {
        if !self.registers.status.contains(Status::PS_CARRY) {
            self.registers.program_counter = addr;
        }
    }

    fn branch_if_carry_set(&mut self, addr: u16) {
        if self.registers.status.contains(Status::PS_CARRY) {
            self.registers.program_counter = addr;
        }
    }

    fn branch_if_equal(&mut self, addr: u16) {
        if self.registers.status.contains(Status::PS_ZERO) {
            self.registers.program_counter = addr;
        }
    }

    fn branch_if_not_equal(&mut self, addr: u16) {
        if !self.registers.status.contains(Status::PS_ZERO) {
            self.registers.program_counter = addr;
        }
    }

    fn branch_if_minus(&mut self, addr: u16) {
        if self.registers.status.contains(Status::PS_NEGATIVE) {
            self.registers.program_counter = addr;
        }
    }

    fn branch(&mut self, addr: u16) {
        self.registers.program_counter = addr;
    }

    fn branch_if_positive(&mut self, addr: u16) {
        if !self.registers.status.contains(Status::PS_NEGATIVE) {
            self.registers.program_counter = addr;
        }
    }

    fn branch_if_overflow_clear(&mut self, addr: u16) {
        if !self.registers.status.contains(Status::PS_OVERFLOW) {
            self.registers.program_counter = addr;
        }
    }

    fn branch_if_overflow_set(&mut self, addr: u16) {
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
        let addr = self.registers.stack_pointer.to_u16();
        let out = self.memory.get_byte(addr);
        self.registers.stack_pointer.increment();
        out
    }

    fn fetch_from_stack(&mut self) -> u8 {
        // gets the next value on the stack but does not update the stack pointer
        let addr = self.registers.stack_pointer.to_u16();
        self.memory.get_byte(addr)
    }
}

impl<M: Bus, V: Variant> core::fmt::Debug for CPU<M, V> {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(
            f,
            "CPU Dump:\n\nAccumulator: {}",
            self.registers.accumulator
        )
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

        cpu.subtract_with_carry(0x80);
        assert_eq!(cpu.registers.accumulator, 0x80);
        cpu.subtract_with_carry(0x80);
        assert_eq!(cpu.registers.accumulator, 0);
    }

    #[cfg_attr(feature = "decimal_mode", test)]
    fn decimal_add_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers.status.or(Status::PS_DECIMAL_MODE);

        cpu.add_with_carry(0x09);
        assert_eq!(cpu.registers.accumulator, 0x09);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.add_with_carry(0x43);
        assert_eq!(cpu.registers.accumulator, 0x52);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.add_with_carry(0x48);
        assert_eq!(cpu.registers.accumulator, 0x00);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));
    }

    #[cfg_attr(feature = "decimal_mode", test)]
    fn decimal_subtract_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers
            .status
            .or(Status::PS_DECIMAL_MODE | Status::PS_CARRY);
        cpu.registers.accumulator = 0;

        cpu.subtract_with_carry(0x48);
        assert_eq!(cpu.registers.accumulator, 0x52);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.subtract_with_carry(0x43);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));
    }

    #[test]
    fn add_with_carry_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.add_with_carry(1);
        assert_eq!(cpu.registers.accumulator, 1);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.add_with_carry(0xff);
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.add_with_carry(1);
        assert_eq!(cpu.registers.accumulator, 2);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        assert_eq!(cpu.registers.accumulator, 0);
        cpu.add_with_carry(127);
        assert_eq!(cpu.registers.accumulator, 127);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        // Allow casting from i8 to u8; -127i8 wraps to 129u8, as intended for
        // two's complement arithmetic.
        cpu.add_with_carry(-127i8 as u8);
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.registers.status.remove(Status::PS_CARRY);
        cpu.add_with_carry(0x80);
        assert_eq!(cpu.registers.accumulator, 0x80);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.add_with_carry(127);
        assert_eq!(cpu.registers.accumulator, 0xff);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.add_with_carry(127);
        assert_eq!(cpu.registers.accumulator, 127);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.add_with_carry(1);
        assert_eq!(cpu.registers.accumulator, 0x80);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));

        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers.status.or(Status::PS_CARRY);
        cpu.add_with_carry(0xff);
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
    }

    #[test]
    fn solid65_adc_immediate() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        // Adding $FF plus carry should be the same as adding $00 and no carry, so these three
        // instructions should leave the carry flags unaffected, i.e. set.
        cpu.execute_instruction((Instruction::LDA, OpInput::UseImmediate(0x9c)));
        cpu.execute_instruction((Instruction::SEC, OpInput::UseImplied));
        cpu.execute_instruction((Instruction::ADC, OpInput::UseImmediate(0xff)));

        assert_eq!(cpu.registers.accumulator, 0x9c);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
    }

    #[test]
    fn php_sets_bits_4_and_5() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.execute_instruction((Instruction::PHP, OpInput::UseImplied));
        cpu.execute_instruction((Instruction::PLA, OpInput::UseImplied));
        cpu.execute_instruction((Instruction::AND, OpInput::UseImmediate(0x30)));

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
    fn subtract_with_carry_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.execute_instruction((Instruction::SEC, OpInput::UseImplied));
        cpu.registers.accumulator = 0;

        cpu.subtract_with_carry(1);
        assert_eq!(cpu.registers.accumulator, 0xff);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::SEC, OpInput::UseImplied));
        cpu.registers.accumulator = 0x80;
        cpu.subtract_with_carry(1);
        assert_eq!(cpu.registers.accumulator, 127);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::SEC, OpInput::UseImplied));
        cpu.registers.accumulator = 127;
        cpu.subtract_with_carry(0xff);
        assert_eq!(cpu.registers.accumulator, 0x80);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::CLC, OpInput::UseImplied));
        cpu.registers.accumulator = -64i8 as u8;
        cpu.subtract_with_carry(64);
        assert_eq!(cpu.registers.accumulator, 127);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::SEC, OpInput::UseImplied));
        cpu.registers.accumulator = 0;
        cpu.subtract_with_carry(0x80);
        assert_eq!(cpu.registers.accumulator, 0x80);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::CLC, OpInput::UseImplied));
        cpu.registers.accumulator = 0;
        cpu.subtract_with_carry(127);
        assert_eq!(cpu.registers.accumulator, 0x80);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));
    }

    #[test]
    fn decrement_memory_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        let addr: u16 = 0xA1B2;

        cpu.memory.set_byte(addr, 5);

        cpu.execute_instruction((Instruction::DEC, OpInput::UseAddress(addr)));
        assert_eq!(cpu.memory.get_byte(addr), 4);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((Instruction::DEC, OpInput::UseAddress(addr)));
        assert_eq!(cpu.memory.get_byte(addr), 3);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((Instruction::DEC, OpInput::UseAddress(addr)));
        cpu.execute_instruction((Instruction::DEC, OpInput::UseAddress(addr)));
        cpu.execute_instruction((Instruction::DEC, OpInput::UseAddress(addr)));
        assert_eq!(cpu.memory.get_byte(addr), 0);
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((Instruction::DEC, OpInput::UseAddress(addr)));
        assert_eq!(cpu.memory.get_byte(addr) as i8, -1);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.memory.set_byte(addr, 0);

        cpu.execute_instruction((Instruction::DEC, OpInput::UseAddress(addr)));
        assert_eq!(cpu.memory.get_byte(addr), 0xff);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn decrement_x_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers.index_x = 0x80;
        cpu.execute_instruction((Instruction::DEX, OpInput::UseImplied));
        assert_eq!(cpu.registers.index_x, 127);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn decrement_y_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.registers.index_y = 0x80;
        cpu.execute_instruction((Instruction::DEY, OpInput::UseImplied));
        assert_eq!(cpu.registers.index_y, 127);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn logical_shift_right_test() {
        // Testing UseImplied version (which targets the accumulator) only, for now

        let mut cpu = CPU::new(Ram::new(), Nmos6502);
        cpu.execute_instruction((Instruction::LDA, OpInput::UseImmediate(0)));
        cpu.execute_instruction((Instruction::LSR, OpInput::UseImplied));
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::LDA, OpInput::UseImmediate(1)));
        cpu.execute_instruction((Instruction::LSR, OpInput::UseImplied));
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::LDA, OpInput::UseImmediate(255)));
        cpu.execute_instruction((Instruction::LSR, OpInput::UseImplied));
        assert_eq!(cpu.registers.accumulator, 0x7F);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::LDA, OpInput::UseImmediate(254)));
        cpu.execute_instruction((Instruction::LSR, OpInput::UseImplied));
        assert_eq!(cpu.registers.accumulator, 0x7F);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));
    }

    #[test]
    fn dec_x_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.execute_instruction((Instruction::DEX, OpInput::UseImplied));
        assert_eq!(cpu.registers.index_x, 0xff);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::DEX, OpInput::UseImplied));
        assert_eq!(cpu.registers.index_x, 0xfe);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.load_x_register(5);
        cpu.execute_instruction((Instruction::DEX, OpInput::UseImplied));
        assert_eq!(cpu.registers.index_x, 4);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::DEX, OpInput::UseImplied));
        cpu.execute_instruction((Instruction::DEX, OpInput::UseImplied));
        cpu.execute_instruction((Instruction::DEX, OpInput::UseImplied));
        cpu.execute_instruction((Instruction::DEX, OpInput::UseImplied));

        assert_eq!(cpu.registers.index_x, 0);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::DEX, OpInput::UseImplied));
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

        cpu.execute_instruction((Instruction::SEC, OpInput::UseImplied));
        cpu.branch_if_carry_clear(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0));

        cpu.execute_instruction((Instruction::CLC, OpInput::UseImplied));
        cpu.branch_if_carry_clear(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0xABCD));
    }

    #[test]
    fn branch_if_carry_set_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.execute_instruction((Instruction::CLC, OpInput::UseImplied));
        cpu.branch_if_carry_set(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0));

        cpu.execute_instruction((Instruction::SEC, OpInput::UseImplied));
        cpu.branch_if_carry_set(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0xABCD));
    }

    #[test]
    fn branch_if_equal_test() {
        let mut cpu = CPU::new(Ram::new(), Nmos6502);

        cpu.branch_if_equal(0xABCD);
        assert_eq!(cpu.registers.program_counter, (0));

        cpu.registers.status.or(Status::PS_ZERO);
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

            cpu.registers.status.or(Status::PS_NEGATIVE);
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

        cpu.execute_instruction((load_instruction, OpInput::UseImmediate(127)));

        compare(&mut cpu, 127);
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((load_instruction, OpInput::UseImmediate(127)));

        compare(&mut cpu, 1);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((load_instruction, OpInput::UseImmediate(1)));

        compare(&mut cpu, 2);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((load_instruction, OpInput::UseImmediate(20)));

        compare(&mut cpu, -50i8 as u8);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((load_instruction, OpInput::UseImmediate(1)));

        compare(&mut cpu, -1i8 as u8);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.execute_instruction((load_instruction, OpInput::UseImmediate(127)));

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
                cpu.execute_instruction((Instruction::LDA, OpInput::UseImmediate(a_before)));

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
                cpu.execute_instruction((Instruction::LDA, OpInput::UseImmediate(a_before)));

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
}

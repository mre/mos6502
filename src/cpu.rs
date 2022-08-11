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

use crate::address::{Address, AddressDiff};
use crate::instruction::{self, DecodedInstr, Instruction, OpInput};
use crate::memory::Memory;
use crate::registers::{Registers, StackPointer, Status, StatusArgs};

#[derive(Clone)]
pub struct CPU {
    pub registers: Registers,
    pub memory: Memory,
}

impl Default for CPU {
    fn default() -> Self {
        Self::new()
    }
}

impl CPU {
    pub fn new() -> CPU {
        CPU {
            registers: Registers::new(),
            memory: Memory::new(),
        }
    }

    pub fn reset(&mut self) {
        *self = CPU::new();
    }

    pub fn fetch_next_and_decode(&mut self) -> Option<DecodedInstr> {
        let x: u8 = self.memory.get_byte(self.registers.program_counter);

        match instruction::OPCODES[x as usize] {
            Some((instr, am)) => {
                let extra_bytes = am.extra_bytes();
                let num_bytes = AddressDiff(1) + extra_bytes;

                let data_start = self.registers.program_counter + AddressDiff(1);

                let slice = self.memory.get_slice(data_start, extra_bytes);
                let am_out = am.process(self, slice);

                // Increment program counter
                self.registers.program_counter = self.registers.program_counter + num_bytes;

                Some((instr, am_out))
            }
            _ => None,
        }
    }

    pub fn execute_instruction(&mut self, decoded_instr: DecodedInstr) {
        match decoded_instr {
            (Instruction::ADC, OpInput::UseImmediate(val)) => {
                debug!("add with carry immediate: {}", val);
                self.add_with_carry(val as i8);
            }
            (Instruction::ADC, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr) as i8;
                debug!("add with carry. address: {:?}. value: {}", addr, val);
                self.add_with_carry(val);
            }

            (Instruction::AND, OpInput::UseImmediate(val)) => {
                self.and(val as i8);
            }
            (Instruction::AND, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr) as i8;
                self.and(val as i8);
            }

            (Instruction::ASL, OpInput::UseImplied) => {
                // Accumulator mode
                let mut val = self.registers.accumulator as u8;
                CPU::shift_left_with_flags(&mut val, &mut self.registers.status);
                self.registers.accumulator = val as i8;
            }
            (Instruction::ASL, OpInput::UseAddress(addr)) => {
                CPU::shift_left_with_flags(
                    self.memory.get_byte_mut_ref(addr),
                    &mut self.registers.status,
                );
            }

            (Instruction::BCC, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter + AddressDiff(i32::from(rel));
                self.branch_if_carry_clear(addr);
            }

            (Instruction::BCS, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter + AddressDiff(i32::from(rel));
                self.branch_if_carry_set(addr);
            }

            (Instruction::BEQ, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter + AddressDiff(i32::from(rel));
                self.branch_if_equal(addr);
            }

            (Instruction::BIT, OpInput::UseAddress(addr)) => {
                let a: u8 = self.registers.accumulator as u8;
                let m: u8 = self.memory.get_byte(addr);
                let res = a & m;

                // The zero flag is set based on the result of the 'and'.
                let is_zero = 0 == res;

                // The N flag is set to bit 7 of the byte from memory.
                let bit7 = 0 != (0x80 & res);

                // The V flag is set to bit 6 of the byte from memory.
                let bit6 = 0 != (0x40 & res);

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
                let addr = self.registers.program_counter + AddressDiff(i32::from(rel));
                debug!("branch if minus relative. address: {:?}", addr);
                self.branch_if_minus(addr);
            }

            (Instruction::BPL, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter + AddressDiff(i32::from(rel));
                self.branch_if_positive(addr);
            }

            (Instruction::BVC, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter + AddressDiff(i32::from(rel));
                self.branch_if_overflow_clear(addr);
            }

            (Instruction::BVS, OpInput::UseRelative(rel)) => {
                let addr = self.registers.program_counter + AddressDiff(i32::from(rel));
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

            (Instruction::DEC, OpInput::UseAddress(addr)) => self.decrement_memory(addr),

            (Instruction::DEX, OpInput::UseImplied) => {
                self.dec_x();
            }

            (Instruction::EOR, OpInput::UseImmediate(val)) => {
                self.exclusive_or(val);
            }
            (Instruction::EOR, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                self.exclusive_or(val);
            }

            (Instruction::INC, OpInput::UseAddress(addr)) => {
                let m = self.memory.get_byte(addr);
                let m = m + 1;
                self.memory.set_byte(addr, m);
                let i = m as i8;
                CPU::set_flags_from_i8(&mut self.registers.status, i);
            }
            (Instruction::INX, OpInput::UseImplied) => {
                let x = self.registers.index_x + 1;
                self.load_x_register(x);
            }
            (Instruction::INY, OpInput::UseImplied) => {
                let y = self.registers.index_y + 1;
                self.load_y_register(y);
            }

            (Instruction::JMP, OpInput::UseAddress(addr)) => self.jump(addr),

            (Instruction::LDA, OpInput::UseImmediate(val)) => {
                debug!("load A immediate: {}", val);
                self.load_accumulator(val as i8);
            }
            (Instruction::LDA, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                debug!("load A. address: {:?}. value: {}", addr, val);
                self.load_accumulator(val as i8);
            }

            (Instruction::LDX, OpInput::UseImmediate(val)) => {
                debug!("load X immediate: {}", val);
                self.load_x_register(val as i8);
            }
            (Instruction::LDX, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                debug!("load X. address: {:?}. value: {}", addr, val);
                self.load_x_register(val as i8);
            }

            (Instruction::LDY, OpInput::UseImmediate(val)) => {
                debug!("load Y immediate: {}", val);
                self.load_y_register(val as i8);
            }
            (Instruction::LDY, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr);
                debug!("load Y. address: {:?}. value: {}", addr, val);
                self.load_y_register(val as i8);
            }

            (Instruction::LSR, OpInput::UseImplied) => {
                // Accumulator mode
                let mut val = self.registers.accumulator as u8;
                CPU::shift_right_with_flags(&mut val, &mut self.registers.status);
                self.registers.accumulator = val as i8;
            }
            (Instruction::LSR, OpInput::UseAddress(addr)) => {
                CPU::shift_right_with_flags(
                    self.memory.get_byte_mut_ref(addr),
                    &mut self.registers.status,
                );
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
                let val = self.registers.accumulator as u8;
                self.push_on_stack(val);
            }
            (Instruction::PHP, OpInput::UseImplied) => {
                // Push status
                let val = self.registers.status.bits();
                self.push_on_stack(val);
            }
            (Instruction::PLA, OpInput::UseImplied) => {
                // Pull accumulator
                let val: u8 = self.pull_from_stack();
                self.registers.accumulator = val as i8;
            }
            (Instruction::PLP, OpInput::UseImplied) => {
                // Pull status
                let val: u8 = self.pull_from_stack();
                // The `truncate` here won't do anything because we have a
                // constant for the single unused flags bit. This probably
                // corresponds to the behavior of the 6502...? FIXME: verify
                self.registers.status = Status::from_bits_truncate(val);
            }

            (Instruction::ROL, OpInput::UseImplied) => {
                // Accumulator mode
                let mut val = self.registers.accumulator as u8;
                CPU::rotate_left_with_flags(&mut val, &mut self.registers.status);
                self.registers.accumulator = val as i8;
            }
            (Instruction::ROL, OpInput::UseAddress(addr)) => {
                CPU::rotate_left_with_flags(
                    self.memory.get_byte_mut_ref(addr),
                    &mut self.registers.status,
                );
            }
            (Instruction::ROR, OpInput::UseImplied) => {
                // Accumulator mode
                let mut val = self.registers.accumulator as u8;
                CPU::rotate_right_with_flags(&mut val, &mut self.registers.status);
                self.registers.accumulator = val as i8;
            }
            (Instruction::ROR, OpInput::UseAddress(addr)) => {
                CPU::rotate_right_with_flags(
                    self.memory.get_byte_mut_ref(addr),
                    &mut self.registers.status,
                );
            }

            (Instruction::SBC, OpInput::UseImmediate(val)) => {
                debug!("subtract with carry immediate: {}", val);
                self.subtract_with_carry(val as i8);
            }
            (Instruction::SBC, OpInput::UseAddress(addr)) => {
                let val = self.memory.get_byte(addr) as i8;
                debug!("subtract with carry. address: {:?}. value: {}", addr, val);
                self.subtract_with_carry(val);
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
                self.memory.set_byte(addr, self.registers.accumulator as u8);
            }
            (Instruction::STX, OpInput::UseAddress(addr)) => {
                self.memory.set_byte(addr, self.registers.index_x as u8);
            }
            (Instruction::STY, OpInput::UseAddress(addr)) => {
                self.memory.set_byte(addr, self.registers.index_y as u8);
            }

            (Instruction::TAX, OpInput::UseImplied) => {
                let val = self.registers.accumulator;
                self.load_x_register(val);
            }
            (Instruction::TAY, OpInput::UseImplied) => {
                let val = self.registers.accumulator;
                self.load_y_register(val);
            }
            (Instruction::TSX, OpInput::UseImplied) => {
                let StackPointer(val) = self.registers.stack_pointer;
                let val = val as i8;
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
                self.registers.stack_pointer = StackPointer(val as u8);
            }
            (Instruction::TYA, OpInput::UseImplied) => {
                let val = self.registers.index_y;
                self.load_accumulator(val);
            }

            (Instruction::NOP, OpInput::UseImplied) => {
                debug!("NOP instruction");
            }
            (_, _) => {
                debug!(
                    "attempting to execute unimplemented or invalid \
                     instruction"
                );
            }
        };
    }

    pub fn run(&mut self) {
        while let Some(decoded_instr) = self.fetch_next_and_decode() {
            self.execute_instruction(decoded_instr);
        }
    }

    fn set_flags_from_i8(status: &mut Status, value: i8) {
        let is_zero = value == 0;
        let is_negative = value < 0;

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
        CPU::set_flags_from_i8(status, *p_val as i8);
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
        CPU::set_flags_from_i8(status, *p_val as i8);
    }

    fn rotate_left_with_flags(p_val: &mut u8, status: &mut Status) {
        let is_carry_set = status.contains(Status::PS_CARRY);
        let mask = 1 << 7;
        let is_bit_7_set = (*p_val & mask) == mask;
        let shifted = (*p_val & !(1 << 7)) << 1;
        *p_val = shifted + if is_carry_set { 1 } else { 0 };
        status.set_with_mask(
            Status::PS_CARRY,
            Status::new(StatusArgs {
                carry: is_bit_7_set,
                ..StatusArgs::none()
            }),
        );
        CPU::set_flags_from_i8(status, *p_val as i8);
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
        CPU::set_flags_from_i8(status, *p_val as i8);
    }

    fn set_i8_with_flags(mem: &mut i8, status: &mut Status, value: i8) {
        *mem = value;
        CPU::set_flags_from_i8(status, value);
    }

    fn load_x_register(&mut self, value: i8) {
        CPU::set_i8_with_flags(
            &mut self.registers.index_x,
            &mut self.registers.status,
            value,
        );
    }

    fn load_y_register(&mut self, value: i8) {
        CPU::set_i8_with_flags(
            &mut self.registers.index_y,
            &mut self.registers.status,
            value,
        );
    }

    fn load_accumulator(&mut self, value: i8) {
        CPU::set_i8_with_flags(
            &mut self.registers.accumulator,
            &mut self.registers.status,
            value,
        );
    }

    fn add_with_carry(&mut self, value: i8) {
        let a_before: i8 = self.registers.accumulator;
        let c_before: i8 = if self.registers.status.contains(Status::PS_CARRY) {
            1
        } else {
            0
        };
        let a_after: i8 = a_before.wrapping_add(c_before).wrapping_add(value);

        debug_assert_eq!(
            a_after as u8,
            a_before.wrapping_add(c_before).wrapping_add(value) as u8
        );

        let bcd1: i8 = if (a_after & 0x0f) as u8 > 0x09 {
            0x06
        } else {
            0x00
        };

        let bcd2: i8 = if (a_after.wrapping_add(bcd1) as u8 & 0xf0) as u8 > 0x90 {
            0x60
        } else {
            0x00
        };

        #[cfg(feature = "decimal_mode")]
        let result: i8 = if self.registers.status.contains(Status::PS_DECIMAL_MODE) {
            a_after.wrapping_add(bcd1).wrapping_add(bcd2)
        } else {
            a_after
        };

        #[cfg(not(feature = "decimal_mode"))]
        let result: i8 = a_after;

        let did_carry = (result as u8) < (a_before as u8) || (c_before == 1 && value == -1);

        let did_overflow = (a_before < 0 && value < 0 && a_after >= 0)
            || (a_before > 0 && value > 0 && a_after <= 0);

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

        debug!("accumulator: {}", self.registers.accumulator);
    }

    fn and(&mut self, value: i8) {
        let a_after = self.registers.accumulator & value;
        self.load_accumulator(a_after);
    }

    fn subtract_with_carry(&mut self, value: i8) {
        // A - M - (1 - C)

        // nc -- 'not carry'
        let nc: i8 = if self.registers.status.contains(Status::PS_CARRY) {
            0
        } else {
            1
        };

        let a_before: i8 = self.registers.accumulator;

        let a_after = a_before.wrapping_sub(value).wrapping_sub(nc);

        // The overflow flag is set on two's-complement overflow.
        //
        // range of A              is  -128 to 127
        // range of - M - (1 - C)  is  -128 to 128
        //                             -(127 + 1) to -(-128 + 0)
        //
        let over =
            ((nc == 0 && value < 0) || (nc == 1 && value < -1)) && a_before >= 0 && a_after < 0;

        let under = (a_before < 0) && (-value - nc < 0) && a_after >= 0;

        let did_overflow = over || under;

        let mask = Status::PS_CARRY | Status::PS_OVERFLOW;

        let bcd1: i8 = if (a_before & 0x0f).wrapping_sub(nc) < (value & 0x0f) {
            0x06
        } else {
            0x00
        };

        let bcd2: i8 = if (a_after.wrapping_sub(bcd1) as u8 & 0xf0) as u8 > 0x90 {
            0x60
        } else {
            0x00
        };

        #[cfg(feature = "decimal_mode")]
        let result: i8 = if self.registers.status.contains(Status::PS_DECIMAL_MODE) {
            a_after.wrapping_sub(bcd1).wrapping_sub(bcd2)
        } else {
            a_after
        };

        #[cfg(not(feature = "decimal_mode"))]
        let result: i8 = a_after;

        // The carry flag is set on unsigned overflow.
        let did_carry = (result as u8) > (a_before as u8);

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

    fn decrement_memory(&mut self, addr: Address) {
        let value_new = self.memory.get_byte(addr).wrapping_sub(1);

        self.memory.set_byte(addr, value_new);

        let is_negative = (value_new as i8) < 0;
        let is_zero = value_new == 0;

        self.registers.status.set_with_mask(
            Status::PS_NEGATIVE | Status::PS_ZERO,
            Status::new(StatusArgs {
                negative: is_negative,
                zero: is_zero,
                ..StatusArgs::none()
            }),
        );
    }

    fn dec_x(&mut self) {
        let val = self.registers.index_x;
        self.load_x_register(val - 1);
    }

    fn jump(&mut self, addr: Address) {
        self.registers.program_counter = addr;
    }

    fn branch_if_carry_clear(&mut self, addr: Address) {
        if !self.registers.status.contains(Status::PS_CARRY) {
            self.registers.program_counter = addr;
        }
    }

    fn branch_if_carry_set(&mut self, addr: Address) {
        if self.registers.status.contains(Status::PS_CARRY) {
            self.registers.program_counter = addr;
        }
    }

    fn branch_if_equal(&mut self, addr: Address) {
        if self.registers.status.contains(Status::PS_ZERO) {
            self.registers.program_counter = addr;
        }
    }

    fn branch_if_minus(&mut self, addr: Address) {
        if self.registers.status.contains(Status::PS_NEGATIVE) {
            self.registers.program_counter = addr;
        }
    }

    fn branch_if_positive(&mut self, addr: Address) {
        if !self.registers.status.contains(Status::PS_NEGATIVE) {
            self.registers.program_counter = addr;
        }
    }

    fn branch_if_overflow_clear(&mut self, addr: Address) {
        if !self.registers.status.contains(Status::PS_OVERFLOW) {
            self.registers.program_counter = addr;
        }
    }

    fn branch_if_overflow_set(&mut self, addr: Address) {
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
    fn compare(&mut self, r: i8, val: u8) {
        if r as u8 >= val as u8 {
            self.registers.status.insert(Status::PS_CARRY);
        } else {
            self.registers.status.remove(Status::PS_CARRY);
        }

        if r as i8 == val as i8 {
            self.registers.status.insert(Status::PS_ZERO);
        } else {
            self.registers.status.remove(Status::PS_ZERO);
        }

        let diff: i8 = r.wrapping_sub(val as i8);
        if diff < 0 {
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
        debug!("compare_with_x_register");

        let x = self.registers.index_x;
        self.compare(x, val);
    }

    fn compare_with_y_register(&mut self, val: u8) {
        let y = self.registers.index_y;
        self.compare(y, val);
    }

    fn exclusive_or(&mut self, val: u8) {
        let a_after = self.registers.accumulator ^ (val as i8);
        self.load_accumulator(a_after);
    }

    fn inclusive_or(&mut self, val: u8) {
        let a_after = self.registers.accumulator | (val as i8);
        self.load_accumulator(a_after);
    }

    fn push_on_stack(&mut self, val: u8) {
        let addr = self.registers.stack_pointer.to_address();
        self.memory.set_byte(addr, val);
        self.registers.stack_pointer.decrement();
    }

    fn pull_from_stack(&mut self) -> u8 {
        let addr = self.registers.stack_pointer.to_address();
        let out = self.memory.get_byte(addr);
        self.registers.stack_pointer.increment();
        out
    }
}

impl core::fmt::Debug for CPU {
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

    use super::*;
    use num::range_inclusive;

    #[cfg_attr(feature = "decimal_mode", test)]
    fn decimal_add_test() {
        let mut cpu = CPU::new();
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
        let mut cpu = CPU::new();
        cpu.registers
            .status
            .or(Status::PS_DECIMAL_MODE | Status::PS_CARRY);
        cpu.registers.accumulator = 0;

        cpu.subtract_with_carry(0x48);
        assert_eq!(cpu.registers.accumulator as u8, 0x52);
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
        let mut cpu = CPU::new();

        cpu.add_with_carry(1);
        assert_eq!(cpu.registers.accumulator, 1);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.add_with_carry(-1);
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

        let mut cpu = CPU::new();

        cpu.add_with_carry(127);
        assert_eq!(cpu.registers.accumulator, 127);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.add_with_carry(-127);
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.registers.status.remove(Status::PS_CARRY);
        cpu.add_with_carry(-128);
        assert_eq!(cpu.registers.accumulator, -128);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.add_with_carry(127);
        assert_eq!(cpu.registers.accumulator, -1);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        let mut cpu = CPU::new();

        cpu.add_with_carry(127);
        assert_eq!(cpu.registers.accumulator, 127);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.add_with_carry(1);
        assert_eq!(cpu.registers.accumulator, -128);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));

        let mut cpu = CPU::new();
        cpu.registers.status.or(Status::PS_CARRY);
        cpu.add_with_carry(-1);
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
    }

    #[test]
    fn and_test() {
        let mut cpu = CPU::new();

        cpu.registers.accumulator = 0;
        cpu.and(-1);
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.registers.accumulator = -1;
        cpu.and(0);
        assert_eq!(cpu.registers.accumulator, 0);
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.registers.accumulator = -1;
        cpu.and(0x0f);
        assert_eq!(cpu.registers.accumulator, 0x0f);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.registers.accumulator = -1;
        cpu.and(-128);
        assert_eq!(cpu.registers.accumulator, -128);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn subtract_with_carry_test() {
        let mut cpu = CPU::new();

        cpu.execute_instruction((Instruction::SEC, OpInput::UseImplied));
        cpu.registers.accumulator = 0;

        cpu.subtract_with_carry(1);
        assert_eq!(cpu.registers.accumulator, -1);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::SEC, OpInput::UseImplied));
        cpu.registers.accumulator = -128;
        cpu.subtract_with_carry(1);
        assert_eq!(cpu.registers.accumulator, 127);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::SEC, OpInput::UseImplied));
        cpu.registers.accumulator = 127;
        cpu.subtract_with_carry(-1);
        assert_eq!(cpu.registers.accumulator, -128);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::CLC, OpInput::UseImplied));
        cpu.registers.accumulator = -64;
        cpu.subtract_with_carry(64);
        assert_eq!(cpu.registers.accumulator, 127);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::SEC, OpInput::UseImplied));
        cpu.registers.accumulator = 0;
        cpu.subtract_with_carry(-128);
        assert_eq!(cpu.registers.accumulator, -128);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.execute_instruction((Instruction::CLC, OpInput::UseImplied));
        cpu.registers.accumulator = 0;
        cpu.subtract_with_carry(127);
        assert_eq!(cpu.registers.accumulator, -128);
        assert!(cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));
    }

    #[test]
    fn decrement_memory_test() {
        let mut cpu = CPU::new();
        let addr = Address(0xA1B2);

        cpu.memory.set_byte(addr, 5);

        cpu.decrement_memory(addr);
        assert_eq!(cpu.memory.get_byte(addr), 4);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.decrement_memory(addr);
        assert_eq!(cpu.memory.get_byte(addr), 3);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.decrement_memory(addr);
        cpu.decrement_memory(addr);
        cpu.decrement_memory(addr);
        assert_eq!(cpu.memory.get_byte(addr), 0);
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));

        cpu.decrement_memory(addr);
        assert_eq!(cpu.memory.get_byte(addr) as i8, -1);
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
    }

    #[test]
    fn logical_shift_right_test() {
        // Testing UseImplied version (which targets the accumulator) only, for now

        let mut cpu = CPU::new();
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
        let mut cpu = CPU::new();

        cpu.dec_x();
        assert_eq!(cpu.registers.index_x, -1);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.dec_x();
        assert_eq!(cpu.registers.index_x, -2);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.load_x_register(5);
        cpu.dec_x();
        assert_eq!(cpu.registers.index_x, 4);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.dec_x();
        cpu.dec_x();
        cpu.dec_x();
        cpu.dec_x();

        assert_eq!(cpu.registers.index_x, 0);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(cpu.registers.status.contains(Status::PS_ZERO));
        assert!(!cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));

        cpu.dec_x();
        assert_eq!(cpu.registers.index_x, -1);
        assert!(!cpu.registers.status.contains(Status::PS_CARRY));
        assert!(!cpu.registers.status.contains(Status::PS_ZERO));
        assert!(cpu.registers.status.contains(Status::PS_NEGATIVE));
        assert!(!cpu.registers.status.contains(Status::PS_OVERFLOW));
    }

    #[test]
    fn jump_test() {
        let mut cpu = CPU::new();
        let addr = Address(0xA1B1);

        cpu.jump(addr);
        assert_eq!(cpu.registers.program_counter, addr);
    }

    #[test]
    fn branch_if_carry_clear_test() {
        let mut cpu = CPU::new();

        cpu.execute_instruction((Instruction::SEC, OpInput::UseImplied));
        cpu.branch_if_carry_clear(Address(0xABCD));
        assert_eq!(cpu.registers.program_counter, Address(0));

        cpu.execute_instruction((Instruction::CLC, OpInput::UseImplied));
        cpu.branch_if_carry_clear(Address(0xABCD));
        assert_eq!(cpu.registers.program_counter, Address(0xABCD));
    }

    #[test]
    fn branch_if_carry_set_test() {
        let mut cpu = CPU::new();

        cpu.execute_instruction((Instruction::CLC, OpInput::UseImplied));
        cpu.branch_if_carry_set(Address(0xABCD));
        assert_eq!(cpu.registers.program_counter, Address(0));

        cpu.execute_instruction((Instruction::SEC, OpInput::UseImplied));
        cpu.branch_if_carry_set(Address(0xABCD));
        assert_eq!(cpu.registers.program_counter, Address(0xABCD));
    }

    #[test]
    fn branch_if_equal_test() {
        let mut cpu = CPU::new();

        cpu.branch_if_equal(Address(0xABCD));
        assert_eq!(cpu.registers.program_counter, Address(0));

        cpu.registers.status.or(Status::PS_ZERO);
        cpu.branch_if_equal(Address(0xABCD));
        assert_eq!(cpu.registers.program_counter, Address(0xABCD));
    }

    #[test]
    fn branch_if_minus_test() {
        {
            let mut cpu = CPU::new();
            let registers_before = cpu.registers;

            cpu.branch_if_minus(Address(0xABCD));
            assert_eq!(cpu.registers, registers_before);
            assert_eq!(cpu.registers.program_counter, Address(0));
        }

        {
            let mut cpu = CPU::new();

            cpu.registers.status.or(Status::PS_NEGATIVE);
            let registers_before = cpu.registers;

            cpu.branch_if_minus(Address(0xABCD));
            assert_eq!(cpu.registers.status, registers_before.status);
            assert_eq!(cpu.registers.program_counter, Address(0xABCD));
        }
    }

    #[test]
    fn branch_if_positive_test() {
        let mut cpu = CPU::new();

        cpu.registers.status.insert(Status::PS_NEGATIVE);
        cpu.branch_if_positive(Address(0xABCD));
        assert_eq!(cpu.registers.program_counter, Address(0));

        cpu.registers.status.remove(Status::PS_NEGATIVE);
        cpu.branch_if_positive(Address(0xABCD));
        assert_eq!(cpu.registers.program_counter, Address(0xABCD));
    }

    #[test]
    fn branch_if_overflow_clear_test() {
        let mut cpu = CPU::new();

        cpu.registers.status.insert(Status::PS_OVERFLOW);
        cpu.branch_if_overflow_clear(Address(0xABCD));
        assert_eq!(cpu.registers.program_counter, Address(0));

        cpu.registers.status.remove(Status::PS_OVERFLOW);
        cpu.branch_if_overflow_clear(Address(0xABCD));
        assert_eq!(cpu.registers.program_counter, Address(0xABCD));
    }

    #[test]
    fn branch_if_overflow_set_test() {
        let mut cpu = CPU::new();

        cpu.branch_if_overflow_set(Address(0xABCD));
        assert_eq!(cpu.registers.program_counter, Address(0));

        cpu.registers.status.insert(Status::PS_OVERFLOW);
        cpu.branch_if_overflow_set(Address(0xABCD));
        assert_eq!(cpu.registers.program_counter, Address(0xABCD));
    }

    #[cfg(test)]
    fn compare_test_helper<F>(compare: &mut F, load_instruction: Instruction)
    where
        F: FnMut(&mut CPU, u8),
    {
        let mut cpu = CPU::new();

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
            &mut |cpu: &mut CPU, val: u8| {
                cpu.compare_with_a_register(val);
            },
            Instruction::LDA,
        );
    }

    #[test]
    fn compare_with_x_register_test() {
        compare_test_helper(
            &mut |cpu: &mut CPU, val: u8| {
                cpu.compare_with_x_register(val);
            },
            Instruction::LDX,
        );
    }

    #[test]
    fn compare_with_y_register_test() {
        compare_test_helper(
            &mut |cpu: &mut CPU, val: u8| {
                cpu.compare_with_y_register(val);
            },
            Instruction::LDY,
        );
    }

    #[test]
    fn exclusive_or_test() {
        let mut cpu = CPU::new();

        for a_before in range_inclusive(0u8, 255u8) {
            for val in range_inclusive(0u8, 255u8) {
                cpu.execute_instruction((Instruction::LDA, OpInput::UseImmediate(a_before)));

                cpu.exclusive_or(val);

                let a_after = a_before ^ val;
                assert_eq!(cpu.registers.accumulator, a_after as i8);

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
        let mut cpu = CPU::new();

        for a_before in range_inclusive(0u8, 255u8) {
            for val in range_inclusive(0u8, 255u8) {
                cpu.execute_instruction((Instruction::LDA, OpInput::UseImmediate(a_before)));

                cpu.inclusive_or(val);

                let a_after = a_before | val;
                assert_eq!(cpu.registers.accumulator, a_after as i8);

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
        let mut cpu = CPU::new();
        let _val: u8 = cpu.pull_from_stack();
    }
}

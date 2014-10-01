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

use memory::Memory;
use registers::{ Registers, Status, StatusArgs };
use registers::{ ps_negative, ps_overflow, ps_zero, ps_carry };

pub struct Machine {
    pub registers: Registers,
    pub memory:    Memory
}

impl Machine {
    pub fn new() -> Machine {
    	Machine{
    	    registers: Registers::new(),
    	    memory:    Memory::new()
    	}
    }
    
    pub fn reset(&mut self) {
    	*self = Machine::new();
    }
    
    // TODO akeeton: Implement binary-coded decimal.
    pub fn add_with_carry(&mut self, value: i8) {
        let a_before: i8 = self.registers.accumulator;
        let c_before: i8 = self.registers.status.get_carry();
        let a_after: i8 = a_before + c_before + value;

        debug_assert_eq!(a_after as u8, a_before as u8 + c_before as u8
                                        + value as u8);

        let did_carry = (a_after as u8) < (a_before as u8);

        let is_zero        = a_after == 0;
        let is_negative    = a_after < 0;
        let did_overflow   =
        	   (a_before < 0 && value < 0 && a_after >= 0)
        	|| (a_before > 0 && value > 0 && a_after <= 0);

        let mask = ps_carry | ps_zero | ps_negative | ps_overflow;

        self.registers.status.set_with_mask(mask,
            Status::new(StatusArgs { carry: did_carry,
                                     zero: is_zero,
                                     negative: is_negative,
                                     overflow: did_overflow,
                                     ..StatusArgs::none() } ));

        self.registers.accumulator = a_after;
    }
}

#[test]
fn add_with_carry_test() {

    let mut machine = Machine::new();

    machine.add_with_carry(1);
    assert_eq!(machine.registers.accumulator, 1);
    assert_eq!(machine.registers.status.contains(ps_carry),    false);
    assert_eq!(machine.registers.status.contains(ps_zero),     false);
    assert_eq!(machine.registers.status.contains(ps_negative), false);
    assert_eq!(machine.registers.status.contains(ps_overflow), false);

    machine.add_with_carry(-1);
    assert_eq!(machine.registers.accumulator, 0);
    assert_eq!(machine.registers.status.contains(ps_carry),    true);
    assert_eq!(machine.registers.status.contains(ps_zero),     true);
    assert_eq!(machine.registers.status.contains(ps_negative), false);
    assert_eq!(machine.registers.status.contains(ps_overflow), false);

    machine.add_with_carry(1);
    assert_eq!(machine.registers.accumulator, 2);
    assert_eq!(machine.registers.status.contains(ps_carry),    false);
    assert_eq!(machine.registers.status.contains(ps_zero),     false);
    assert_eq!(machine.registers.status.contains(ps_negative), false);
    assert_eq!(machine.registers.status.contains(ps_overflow), false);
    
    let mut machine = Machine::new();

    machine.add_with_carry(127);
    assert_eq!(machine.registers.accumulator, 127);
    assert_eq!(machine.registers.status.contains(ps_carry),    false);
    assert_eq!(machine.registers.status.contains(ps_zero),     false);
    assert_eq!(machine.registers.status.contains(ps_negative), false);
    assert_eq!(machine.registers.status.contains(ps_overflow), false);

    machine.add_with_carry(-127);
    assert_eq!(machine.registers.accumulator, 0);
    assert_eq!(machine.registers.status.contains(ps_carry),     true);
    assert_eq!(machine.registers.status.contains(ps_zero),      true);
    assert_eq!(machine.registers.status.contains(ps_negative), false);
    assert_eq!(machine.registers.status.contains(ps_overflow), false);

    machine.registers.status.remove(ps_carry);
    machine.add_with_carry(-128);
    assert_eq!(machine.registers.accumulator, -128);
    assert_eq!(machine.registers.status.contains(ps_carry),    false);
    assert_eq!(machine.registers.status.contains(ps_zero),     false);
    assert_eq!(machine.registers.status.contains(ps_negative),  true);
    assert_eq!(machine.registers.status.contains(ps_overflow), false);

    machine.add_with_carry(127);
    assert_eq!(machine.registers.accumulator, -1);
    assert_eq!(machine.registers.status.contains(ps_carry),    false);
    assert_eq!(machine.registers.status.contains(ps_zero),     false);
    assert_eq!(machine.registers.status.contains(ps_negative),  true);
    assert_eq!(machine.registers.status.contains(ps_overflow), false);

    let mut machine = Machine::new();

    machine.add_with_carry(127);
    assert_eq!(machine.registers.accumulator, 127);
    assert_eq!(machine.registers.status.contains(ps_carry),    false);
    assert_eq!(machine.registers.status.contains(ps_zero),     false);
    assert_eq!(machine.registers.status.contains(ps_negative), false);
    assert_eq!(machine.registers.status.contains(ps_overflow), false);

    machine.add_with_carry(1);
    assert_eq!(machine.registers.accumulator, -128);
    assert_eq!(machine.registers.status.contains(ps_carry),    false);
    assert_eq!(machine.registers.status.contains(ps_zero),     false);
    assert_eq!(machine.registers.status.contains(ps_negative),  true);
    assert_eq!(machine.registers.status.contains(ps_overflow),  true);
}

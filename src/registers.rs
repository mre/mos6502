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

use bitflags::bitflags;

// Useful for constructing Status instances
#[derive(Copy, Clone, Debug)]
#[allow(clippy::struct_excessive_bools)]
pub struct StatusArgs {
    pub negative: bool,
    pub overflow: bool,
    pub unused: bool,
    pub brk: bool,
    pub decimal_mode: bool,
    pub disable_interrupts: bool,
    pub zero: bool,
    pub carry: bool,
}

impl StatusArgs {
    #[must_use]
    pub const fn none() -> StatusArgs {
        StatusArgs {
            negative: false,
            overflow: false,
            unused: false,
            brk: false,
            decimal_mode: false,
            disable_interrupts: false,
            zero: false,
            carry: false,
        }
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct Status: u8 {
        const PS_NEGATIVE           = 0b1000_0000;
        const PS_OVERFLOW           = 0b0100_0000;
        const PS_UNUSED             = 0b0010_0000; // JAM: Should this exist?
                                                  // (note that it affects the
                                                  // behavior of things like
                                                  // from_bits_truncate)
        const PS_BRK                = 0b0001_0000;
        const PS_DECIMAL_MODE       = 0b0000_1000;
        const PS_DISABLE_INTERRUPTS = 0b0000_0100;
        const PS_ZERO               = 0b0000_0010;
        const PS_CARRY              = 0b0000_0001;
    }
}

impl Status {
    #[must_use]
    pub fn new(
        StatusArgs {
            negative,
            overflow,
            unused,
            brk,
            decimal_mode,
            disable_interrupts,
            zero,
            carry,
        }: StatusArgs,
    ) -> Status {
        let mut out = Status::empty();

        if negative {
            out |= Status::PS_NEGATIVE;
        }
        if overflow {
            out |= Status::PS_OVERFLOW;
        }
        if unused {
            out |= Status::PS_UNUSED;
        }
        if brk {
            out |= Status::PS_BRK;
        }
        if decimal_mode {
            out |= Status::PS_DECIMAL_MODE;
        }
        if disable_interrupts {
            out |= Status::PS_DISABLE_INTERRUPTS;
        }
        if zero {
            out |= Status::PS_ZERO;
        }
        if carry {
            out |= Status::PS_CARRY;
        }

        out
    }

    pub fn and(&mut self, rhs: Status) {
        *self &= rhs;
    }

    pub fn or(&mut self, rhs: Status) {
        *self |= rhs;
    }

    pub fn set_with_mask(&mut self, mask: Status, rhs: Status) {
        *self = (*self & !mask) | rhs;
    }
}

impl Default for Status {
    fn default() -> Self {
        // Safe emulator defaults chosen for reliability across all 6502 variants.
        // Real hardware varies: NMOS has undefined decimal flag on reset, 65C02 clears it.
        // We could implement variant-specific defaults, but given that the flags
        // are randomly set on real hardware, it's fair to use a safe default.
        Status::new(StatusArgs {
            negative: false,          // N: Negative result flag
            overflow: false,          // V: Overflow flag, not set on reset
            unused: true,             // -: Bit 5 typically set on all variants
            brk: false,               // B: Not stored in register, only during stack operations
            decimal_mode: false, // D: Matches 65C02 behavior, safe for NMOS (software uses CLD anyway)
            disable_interrupts: true, // I: Interrupts disabled on reset for all variants
            zero: false,         // Z: Flag for zero result
            carry: false,        // C: Flag for carry
        })
    }
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct StackPointer(pub u8);

impl StackPointer {
    #[must_use]
    pub const fn to_u16(self) -> u16 {
        let StackPointer(val) = self;
        u16::from_le_bytes([val, 0x01])
    }

    pub const fn decrement(&mut self) {
        self.0 = self.0.wrapping_sub(1);
    }

    pub const fn increment(&mut self) {
        self.0 = self.0.wrapping_add(1);
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Registers {
    pub accumulator: u8,
    pub index_x: u8,
    pub index_y: u8,
    pub stack_pointer: StackPointer,
    pub program_counter: u16,
    pub status: Status,
    /// `HuC6280` Memory Management Unit mapping registers (MPR0-MPR7).
    ///
    /// The `HuC6280` (`TurboGrafx-16` / PC Engine) extends the 16-bit logical
    /// address space to a 21-bit (2 MB) physical space. The top three bits of
    /// every logical address select one of these eight registers, whose 8-bit
    /// value supplies the upper bits of the physical address. They are
    /// programmed with the `TAM`/`TMA` instructions.
    ///
    /// These registers are unused by every other variant and remain zeroed.
    pub mpr: [u8; 8],
}

impl Registers {
    /// Translates a 16-bit logical address into the 21-bit physical address
    /// produced by the `HuC6280` MMU, using the current mapping registers.
    ///
    /// The top three bits of `logical` select the mapping register (MPR0-MPR7);
    /// the selected register supplies bits 13-20 of the physical address while
    /// the low 13 bits pass through unchanged.
    ///
    /// For every non-HuC6280 variant the mapping registers are zero, so this is
    /// the identity mapping within the low 64 KB.
    #[must_use]
    pub const fn physical_address(&self, logical: u16) -> u32 {
        map_physical_address(&self.mpr, logical)
    }
}

/// Translates a 16-bit logical address into the 21-bit physical address
/// produced by the `HuC6280` MMU for the given mapping registers (MPR0-MPR7).
///
/// The top three bits of `logical` select the mapping register; that register
/// supplies bits 13-20 of the physical address while the low 13 bits pass
/// through unchanged.
///
/// This is the single source of truth for the `HuC6280` address mapping.
/// [`Registers::physical_address`] delegates to it, and a translating
/// [`Bus`](crate::memory::Bus) that keeps its own copy of the mapping registers
/// (for example one that mirrors them via
/// [`Bus::set_mapping_register`](crate::memory::Bus::set_mapping_register))
/// should call this rather than re-deriving the formula.
#[must_use]
pub const fn map_physical_address(mpr: &[u8; 8], logical: u16) -> u32 {
    let bank = mpr[(logical >> 13) as usize];
    ((bank as u32) << 13) | (logical as u32 & 0x1FFF)
}

impl Default for Registers {
    fn default() -> Self {
        Self::new()
    }
}

impl Registers {
    #[must_use]
    pub fn new() -> Registers {
        // Zero initialization for emulator predictability.
        // Real hardware has undefined register states on power-on.
        Registers {
            accumulator: 0,
            index_x: 0,
            index_y: 0,
            stack_pointer: StackPointer(0), // Real hardware: random value on power-on
            program_counter: 0,             // Set by reset vector in practice
            status: Status::default(),
            mpr: [0; 8], // HuC6280 MMU registers; unused by other variants
        }
    }
}

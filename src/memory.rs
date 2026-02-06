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

// JAM: We can probably come up with a better way to represent address ranges.
//      Address range type?
//
// // Address range -- inclusive on both sides
// pub struct AddressRangeIncl {
//     begin: Address,
//     end: Address,
// }

const ADDR_LO_BARE: u16 = 0x0000;
const ADDR_HI_BARE: u16 = 0xFFFF;

pub const MEMORY_ADDRESS_LO: u16 = ADDR_LO_BARE;
pub const MEMORY_ADDRESS_HI: u16 = ADDR_HI_BARE;
pub const STACK_ADDRESS_LO: u16 = 0x0100;
pub const STACK_ADDRESS_HI: u16 = 0x01FF;
pub const NMI_INTERRUPT_VECTOR_LO: u16 = 0xFFFA;
pub const NMI_INTERRUPT_VECTOR_HI: u16 = 0xFFFB;
pub const RESET_VECTOR_LO: u16 = 0xFFFC;
pub const RESET_VECTOR_HI: u16 = 0xFFFD;
pub const IRQ_INTERRUPT_VECTOR_LO: u16 = 0xFFFE;
pub const IRQ_INTERRUPT_VECTOR_HI: u16 = 0xFFFF;

const MEMORY_SIZE: usize = (ADDR_HI_BARE - ADDR_LO_BARE) as usize + 1usize;

/// 64KB memory implementation  
#[derive(Copy, Clone, Debug)]
pub struct Memory {
    #[allow(clippy::large_stack_arrays)]
    bytes: [u8; MEMORY_SIZE],
}

impl Default for Memory {
    fn default() -> Self {
        Self::new()
    }
}

/// Trait for a bus that can read and write bytes.
///
/// This is used to abstract the memory and I/O operations of the CPU.
///
/// # Examples
///
/// ```
/// use mos6502::memory::{Bus, Memory};
///
/// let mut memory = Memory::new();
/// memory.set_byte(0x0000, 0x12);
/// assert_eq!(memory.get_byte(0x0000), 0x12);
/// ```
pub trait Bus {
    /// Returns the byte at the given address.
    fn get_byte(&mut self, address: u16) -> u8;

    /// Sets the byte at the given address to the given value.
    fn set_byte(&mut self, address: u16, value: u8);

    /// Sets a 16-bit word at the given address (little-endian).
    ///
    /// This is a convenience method that sets the low byte at `address`
    /// and the high byte at `address + 1`.
    fn set_word(&mut self, address: u16, value: u16) {
        let bytes = value.to_le_bytes();
        self.set_byte(address, bytes[0]);
        self.set_byte(address.wrapping_add(1), bytes[1]);
    }

    /// Sets the bytes starting at the given address to the given values.
    ///
    /// This is a default implementation that calls `set_byte` for each byte.
    ///
    /// # Note
    ///
    /// This assumes that the length of `values` is less than or equal to
    /// [`u16::MAX`] (65535). If the length of `values` is greater than `u16::MAX`,
    /// this will truncate the length. This assumption is made because the
    /// maximum addressable memory for the 6502 is 64KB.
    #[allow(clippy::cast_possible_truncation)]
    fn set_bytes(&mut self, start: u16, values: &[u8]) {
        for i in 0..values.len() as u16 {
            self.set_byte(start + i, values[i as usize]);
        }
    }

    /// Returns whether an NMI (Non-Maskable Interrupt) is pending.
    ///
    /// NMI is edge-triggered on the falling edge (high → low transition).
    /// The CPU will detect the transition and service the interrupt.
    ///
    /// Implementations may use `&mut self` to acknowledge or clear the pending state.
    ///
    /// Default implementation returns `false` (no NMI pending).
    ///
    /// # References
    ///
    /// - [W65C02S Datasheet, Section 3.6 (NMIB)](https://www.westerndesigncenter.com/wdc/documentation/w65c02s.pdf)
    fn nmi_pending(&mut self) -> bool {
        false
    }

    /// Returns whether an IRQ (Interrupt Request) is pending.
    ///
    /// IRQ is level-triggered and can be masked by the I flag in the status register.
    /// The interrupt will be serviced while pending and interrupts are enabled.
    ///
    /// Implementations may use `&mut self` to acknowledge or clear the pending state.
    ///
    /// Default implementation returns `false` (no IRQ pending).
    ///
    /// # References
    ///
    /// - [W65C02S Datasheet, Section 3.4 (IRQB)](https://www.westerndesigncenter.com/wdc/documentation/w65c02s.pdf)
    fn irq_pending(&mut self) -> bool {
        false
    }
}

impl Memory {
    #[must_use]
    #[allow(clippy::large_stack_arrays)]
    pub const fn new() -> Memory {
        Memory {
            #[allow(clippy::large_stack_arrays)]
            bytes: [0; MEMORY_SIZE],
        }
    }
}

impl Bus for Memory {
    fn get_byte(&mut self, address: u16) -> u8 {
        self.bytes[address as usize]
    }

    /// Sets the byte at the given address to the given value and returns the
    /// previous value at the address.
    fn set_byte(&mut self, address: u16, value: u8) {
        self.bytes[address as usize] = value;
    }

    /// Fast way to set multiple bytes in memory when the underlying memory is a
    /// consecutive block of bytes.
    fn set_bytes(&mut self, start: u16, values: &[u8]) {
        let start = start as usize;

        // This panics if the range is invalid
        let end = start + values.len();

        self.bytes[start..end].copy_from_slice(values);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic(expected = "range end index 65537 out of range for slice of length 65536")]
    fn test_memory_overflow_panic() {
        let mut memory = Memory::new();
        memory.set_bytes(0xFFFE, &[1, 2, 3]);
    }
}

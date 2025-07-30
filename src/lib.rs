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

#![warn(clippy::all, clippy::pedantic)]
#![warn(
    absolute_paths_not_starting_with_crate,
    rustdoc::invalid_html_tags,
    missing_copy_implementations,
    missing_debug_implementations,
    semicolon_in_expressions_from_macros,
    unreachable_pub,
    unused_crate_dependencies,
    unused_extern_crates,
    variant_size_differences,
    clippy::missing_const_for_fn
)]
#![deny(anonymous_parameters, macro_use_extern_crate)]
#![allow(clippy::module_name_repetitions, clippy::needless_doctest_main)]
// Registers and ops follow the 6502 naming convention and have similar names at
// times
#![allow(clippy::similar_names)]
#![allow(clippy::match_same_arms)]
#![allow(clippy::too_many_lines)]
#![no_std]

#[doc = include_str!("../README.md")]
pub mod cpu;
pub mod instruction;
pub mod memory;
pub mod registers;

/// Output of arithmetic instructions (ADC/SBC)
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)]
pub struct ArithmeticOutput {
    result: u8,
    did_carry: bool,
    overflow: bool,
    negative: bool,
    zero: bool,
}

/// Trait for different 6502 CPU variants with their historical differences.
///
/// The 6502 family evolved over decades with various manufacturers creating
/// specialized versions for different applications:
///
/// - **NMOS 6502** (1975): Original MOS Technology processor used in Apple II,
///   Commodore 64, Atari 2600. Has unreliable decimal mode flags but full BCD support.
///
/// - **65C02** (1982): WDC's CMOS version with bug fixes, additional instructions,
///   and reliable decimal mode flags. Used in Apple IIc/IIe.
///
/// - **Ricoh 2A03** (1983): Nintendo's cost-reduced variant for NES/Famicom.
///   Removed decimal mode entirely and added sound generation circuitry.
///
/// - **Revision A** (1976): Very early 6502 variant missing the ROR instruction
///   or having a buggy implementation. Found in early KIM-1 systems.
///
/// Choose the variant that matches your target system for accurate emulation.
pub trait Variant {
    fn decode(
        opcode: u8,
    ) -> Option<(
        crate::instruction::Instruction,
        crate::instruction::AddressingMode,
    )>;

    /// Execute Add with Carry (ADC) in binary mode
    ///
    /// # Arguments
    /// * `accumulator` - Current accumulator value
    /// * `value` - Value to add  
    /// * `carry_set` - Carry flag set at the time of execution (0 or 1)
    ///
    /// # Returns
    /// Tuple of (result, `carry_out`, overflow, negative, zero)
    fn adc_binary(accumulator: u8, value: u8, carry_set: u8) -> ArithmeticOutput;

    /// Execute Add with Carry (ADC) in decimal mode (BCD)
    ///
    /// # Arguments
    /// * `accumulator` - Current accumulator value
    /// * `value` - Value to add  
    /// * `carry_set` - Carry flag set at the time of execution (0 or 1)
    ///
    /// # Returns
    /// Tuple of (result, `carry_out`, overflow, negative, zero)
    fn adc_decimal(accumulator: u8, value: u8, carry_set: u8) -> ArithmeticOutput;

    /// Execute Subtract with Carry (SBC) in binary mode
    ///
    /// # Arguments
    /// * `accumulator` - Current accumulator value
    /// * `value` - Value to subtract  
    /// * `carry_set` - Carry flag set at the time of execution (0 or 1)
    ///
    /// # Returns
    /// Tuple of (result, `carry_out`, overflow, negative, zero)
    fn sbc_binary(accumulator: u8, value: u8, carry_set: u8) -> ArithmeticOutput;

    /// Execute Subtract with Carry (SBC) in decimal mode (BCD)
    ///
    /// # Arguments
    /// * `accumulator` - Current accumulator value
    /// * `value` - Value to subtract  
    /// * `carry_set` - Carry flag set at the time of execution (0 or 1)
    ///
    /// # Returns
    /// Tuple of (result, `carry_out`, overflow, negative, zero)
    fn sbc_decimal(accumulator: u8, value: u8, carry_set: u8) -> ArithmeticOutput;
}

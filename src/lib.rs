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

//! An emulator for the MOS 6502 CPU and its variants.
//!
//! The 6502 was one of the most influential microprocessors in history, powering
//! iconic systems like the Apple II, Commodore 64, Atari 2600, and Nintendo
//! Entertainment System. Its low cost democratized computing and helped birth
//! the personal computer industry.
//!
//! ## Variants
//!
//! This emulator supports multiple 6502 variants, each with subtle differences
//! in behavior. Choose the [variant] that matches your target system:
//!
//! - **NMOS 6502**: Original MOS Technology processor
//! - **65C02**: CMOS version with bug fixes and new instructions
//! - **Ricoh 2A03**: Nintendo's NES variant without decimal mode
//! - **Revision A**: Very early variant missing the ROR instruction
//!
//! [variant]: crate::Variant

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
/// - **Revision A (1975)**: Very early 6502 variant missing the ROR instruction
///   entirely. Found in early KIM-1 systems and some Apple-1 computers. The ROR
///   instruction was intentionally unimplemented due to design constraints, not
///   a "bug" as commonly believed. Production ended around June 1976.
///
/// - **NMOS 6502 (1976)**: Complete MOS Technology processor with working ROR
///   instruction, used in Apple II, Commodore 64, Atari 2600. Has unreliable
///   decimal mode flags but full BCD support. This became the standard reference
///   implementation.
///
/// - **65C02 (1983)**: WDC's CMOS version with bug fixes, additional instructions,
///   and reliable decimal mode flags. Development began in 1981 with samples
///   released in early 1983. Used in Apple IIc/IIe and many embedded systems.
///
/// - **Ricoh 2A03 (1983)**: Nintendo's cost-reduced variant for NES/Famicom
///   (released July 15, 1983). Removed decimal mode entirely to avoid patent
///   issues. Used as a core in their custom ASIC which also included sound
///   generation and other features.
///
/// Choose the variant that matches your target system for accurate emulation.
/// Note that software written for later variants may not run on earlier ones
/// due to missing instructions (particularly ROR on Revision A).
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
    /// * `carry_set` - Carry flag set at the time of execution
    ///
    /// # Returns
    /// Tuple of (result, `carry_out`, overflow, negative, zero)
    fn adc_binary(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput;

    /// Execute Add with Carry (ADC) in decimal mode (BCD)
    ///
    /// # Arguments
    /// * `accumulator` - Current accumulator value
    /// * `value` - Value to add
    /// * `carry_set` - Carry flag set at the time of execution
    ///
    /// # Returns
    /// Tuple of (result, `carry_out`, overflow, negative, zero)
    fn adc_decimal(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput;

    /// Execute Subtract with Carry (SBC) in binary mode
    ///
    /// # Arguments
    /// * `accumulator` - Current accumulator value
    /// * `value` - Value to subtract
    /// * `carry_set` - Carry flag set at the time of execution
    ///
    /// # Returns
    /// Tuple of (result, `carry_out`, overflow, negative, zero)
    fn sbc_binary(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput;

    /// Execute Subtract with Carry (SBC) in decimal mode (BCD)
    ///
    /// # Arguments
    /// * `accumulator` - Current accumulator value
    /// * `value` - Value to subtract
    /// * `carry_set` - Carry flag set at the time of execution
    ///
    /// # Returns
    /// Tuple of (result, `carry_out`, overflow, negative, zero)
    fn sbc_decimal(accumulator: u8, value: u8, carry_set: bool) -> ArithmeticOutput;
}

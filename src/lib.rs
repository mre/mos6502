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

//! In 1975, a small team of engineers led by [Chuck Peddle] at MOS Technology
//! created what would become one of the most influential microprocessors in
//! history. The 6502 was revolutionary not just for its elegant design, but for
//! its price: at $25, it cost a fraction of competing processors like the Intel
//! 8080 ($179) and Motorola 6800 ($175).
//!
//! This dramatic cost reduction democratized computing, making it possible for
//! hobbyists and small companies to build affordable computers. The result was
//! an explosion of innovation that gave birth to the personal computer industry.
//!
//! ## What People Built With It
//!
//! **Apple II (1977)** - Steve Wozniak chose the 6502 for its low cost and
//! elegant instruction set. The Apple II's success helped establish Apple as a
//! major computer company.
//!
//! **Atari 2600 (1977)** - The 6507, a cost-reduced 6502 with fewer address
//! pins, became the heart of the most successful game console of its era.
//!
//! **Commodore 64 (1982)** - Became the best-selling home computer of all time,
//! powered by a 6510 (6502 variant with built-in I/O port).
//!
//! **Nintendo Entertainment System (1983)** - Used a custom Ricoh 2A03, a 6502
//! with the decimal mode removed to avoid patent issues and costs, plus
//! integrated sound hardware.
//!
//! ## Variants and Their Engineering Trade-offs
//!
//! Each variant tells a story of engineering decisions driven by cost, patents,
//! and innovation. This emulator aims to capture these differences faithfully.
//! Pick one of the supported [variants] to emulate the specific behavior of
//! the 6502 family you are interested in.
//!
//! [Chuck Peddle]: https://en.wikipedia.org/wiki/Chuck_Peddle
//! [variants]: crate::Variant

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
///   issues and added integrated 5-channel sound generation circuitry (APU).
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

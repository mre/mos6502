# mos6502

![MOS6502](assets/6502.jpg)

![](https://github.com/mre/mos6502/workflows/test/badge.svg)
[![docs.rs](https://docs.rs/mos6502/badge.svg)](https://docs.rs/mos6502)

An emulator for the [MOS 6502 CPU](https://en.wikipedia.org/wiki/MOS_Technology_6502) written in Rust.\
Tested and validated by [solid65](https://github.com/omarandlorraine/solid65).\
It builds on stable Rust and supports `#[no_std]` targets.

## What is the MOS 6502?

> The MOS Technology 6502 (typically pronounced "sixty-five-oh-two" or "six-five-oh-two") is an 8-bit microprocessor that was designed by a small team led by Chuck Peddle for MOS Technology. [...]
>
> When it was introduced in 1975, the 6502 was the **least expensive microprocessor on the market** by a considerable margin. It initially sold for less than one-sixth the cost of competing designs from larger companies, such as the 6800 or Intel 8080. Its introduction caused rapid decreases in pricing across the entire processor market. **Along with the Zilog Z80, it sparked a series of projects that resulted in the home computer revolution of the early 1980s.**

Source: [Wikipedia](https://en.wikipedia.org/wiki/MOS_Technology_6502)


## How to use this library

```rust
use mos6502::memory::Bus;
use mos6502::memory::Memory;
use mos6502::instruction::Nmos6502;
use mos6502::cpu;

fn main() {
    // Calculate the greatest common divisor of 56 and 49
    // using Euclid's algorithm.
    let zero_page_data = [56, 49];

    let program = [
        // (F)irst | (S)econd
        // .algo
        0xa5, 0x00,       // Load from F to A
        // .algo_
        0x38,             // Set carry flag
        0xe5, 0x01,       // Substract S from number in A (from F)
        0xf0, 0x07,       // Jump to .end if diff is zero
        0x30, 0x08,       // Jump to .swap if diff is negative
        0x85, 0x00,       // Load A to F
        0x4c, 0x12, 0x00, // Jump to .algo_
        // .end
        0xa5, 0x00,       // Load from S to A
        0xff,
        // .swap
        0xa6, 0x00,       // load F to X
        0xa4, 0x01,       // load S to Y
        0x86, 0x01,       // Store X to F
        0x84, 0x00,       // Store Y to S
        0x4c, 0x10, 0x00, // Jump to .algo
    ];

    let mut cpu = cpu::CPU::new(Memory::new(), Nmos6502);

    cpu.memory.set_bytes(0x00, &zero_page_data);
    cpu.memory.set_bytes(0x10, &program);
    cpu.registers.program_counter = 0x10;

    cpu.run();

    // The expected GCD is 7
    assert_eq!(7, cpu.registers.accumulator);
}
```

The same can be achieved, by compiling the euclid example yourself.

First install a 6502 assembler and linker, e.g. [cc65](https://cc65.github.io/cc65/).

```sh
brew install cc65
```

Then compile and link the assembly file:

```sh
cd examples/asm/euclid
ca65 euclid.a65
ld65 -C ../linker.cfg -o euclid.bin euclid.o
```

This will create a binary file `euclid.bin` that you can load into the emulator:

```rust
use mos6502::memory::Bus;
use mos6502::memory::Memory;
use mos6502::instruction::Nmos6502;
use mos6502::cpu;
use std::fs::read;

fn main() {
    // Calculate the greatest common divisor of 56 and 49
    // using Euclid's algorithm.
    let zero_page_data = [56, 49];

    // Load the binary file from disk
    let program = match read("examples/asm/euclid/euclid.bin") {
        Ok(data) => data,
        Err(err) => {
            println!("Error reading euclid.bin: {}", err);
            return;
        }
    };

    let mut cpu = cpu::CPU::new(Memory::new(), Nmos6502);

    cpu.memory.set_bytes(0x00, &zero_page_data);
    cpu.memory.set_bytes(0x10, &program);
    cpu.registers.program_counter = 0x10;

    cpu.run();

    // The expected GCD is 7
    assert_eq!(7, cpu.registers.accumulator);
}
```

## History

The history of the MOS 6502 is highly interesting but largely unknown to many.
You might argue that the documentation of this project is the wrong place for
such a history lesson, but we believe that understanding the context in which
the 6502 was created and used adds significant value to anyone interested in
this microprocessor and provides context that can be beneficial for emulation
and software development.

In 1975, a small team of engineers led by [Chuck
Peddle](https://en.wikipedia.org/wiki/Chuck_Peddle) at MOS Technology created
what would become one of the most influential microprocessors in history. The
6502 was revolutionary not just for its elegant design, but for its price: at
$25, it cost a fraction of competing processors like the Intel 8080 ($179) and
Motorola 6800 ($175).

This dramatic cost reduction democratized computing, making it possible for
hobbyists and small companies to build affordable computers. The result was an
explosion of innovation that gave birth to the personal computer industry.

The significance of this small chip cannot be overstated. It powered some of the
most iconic early computers and gaming consoles, as we will see in the next
section. Its influence is still felt today.

### What People Built With It

**Apple II (1977)** - Steve Wozniak chose the 6502 for its low cost and elegant
instruction set. The Apple II's success helped establish Apple as a major
computer company.

> The 6502 was pin-for-pin compatible with the Motorola 6800 I had drafted my design around. That meant I could just pop it in without any redesigning at all. And the best part is they cost half ($20) of what the Motorola chip would have cost me.
>
> — Steve Wozniak, [*iWoz*](https://wwnorton.com/books/iWoz/)

**Atari 2600 (1977)** - The 6507, a cost-reduced 6502 with fewer address pins,
became the heart of the most successful game console of its era.

**Commodore 64 (1982)** - Became the best-selling home computer of all time,
powered by a 6510 (6502 variant with built-in I/O port).

**Nintendo Entertainment System (1983)** - Used a custom Ricoh 2A03, a 6502
variant as the core of their custom ASIC which also generated sound and video.

**Industrial Control** - Beyond consumer electronics, the 6502 became hugely
popular in embedded systems and industrial control applications. Its low cost,
reliability, and simple architecture made it ideal for automation,
instrumentation, and control systems that operated for decades.

### Variants and Their Engineering Trade-offs

Each variant tells a story of engineering decisions driven by cost, patents, and
innovation:

- **NMOS 6502** (1975): Original MOS Technology processor used in Apple II,
  Commodore 64, Atari 2600. Has unreliable decimal mode flags but full BCD
  support.

- **Revision A** (1976): Very early 6502 variant missing the ROR instruction or
  having a buggy implementation. Found in early KIM-1 systems.

- **65C02** (1982): WDC's CMOS version with bug fixes, additional instructions,
  and reliable decimal mode flags. Used in Apple IIc/IIe and many embedded
  systems due to lower power consumption.

- **Ricoh 2A03** (1983): Nintendo's cost-reduced variant for NES/Famicom.
  Removed decimal mode entirely to avoid patent issues and reduce costs. Used as
  the core of their custom ASIC which also included sound generation and other
  features.

### Further Reading

- [Visual 6502](http://visual6502.org/) - Simulation of the 6502 at the transistor level
- [6502.org](http://6502.org/) - Community resources and documentation
- [Easy 6502](https://skilldrick.github.io/easy6502/) - Interactive 6502 assembly tutorial

## Credits

This started off as a fork of [amw-zero/6502-rs](https://github.com/amw-zero/6502-rs),
which seems to be [unmaintained](https://github.com/amw-zero/6502-rs/pull/36) at this point.

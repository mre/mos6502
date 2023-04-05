extern crate mos6502;

use env_logger::Env;

use std::fs::File;
use std::io::Read;

use mos6502::cpu::CPU;
use mos6502::memory::Bus;
use mos6502::memory::Memory;

const PC_START: u16 = 0x400;
// const MAX_CYCLES: usize = 100000000;

#[test]
fn functional_test() {
    env_logger::init_from_env(Env::default().default_filter_or("mos6502=debug"));

    let mut f = File::open("test-roms/6502_functional_test.bin").unwrap();
    let mut rom = Vec::<u8>::new();
    f.read_to_end(&mut rom).unwrap();
    let mut cpu = CPU::new(Memory::<66560>::new());

    cpu.memory.set_bytes(PC_START, &rom);

    // cpu.run();

    let mut last_pc = PC_START;

    loop {
        cpu.single_step();
        // Prevent endless loop
        // TODO: We could add cycle count to the CPU struct to check that
        // if cpu.interconnect.elapsed_cycles() > MAX_CYCLES {
        //     assert!(false, "Took too many cycles to complete");
        // }

        if last_pc == cpu.registers.program_counter {
            if cpu.registers.program_counter == 0x3367 {
                // Success!
                break;
            } else {
                // assert!(false, "Trap detected");
            }
        }

        last_pc = cpu.registers.program_counter;
    }
}

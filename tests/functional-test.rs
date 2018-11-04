extern crate mos6502;

use std::fs::File;
use std::io::Read;

use mos6502::address::Address;
use mos6502::cpu::CPU;

const PC_START: u16 = 0x400;
// const MAX_CYCLES: usize = 100000000;

#[test]
fn functional_test() {
    let mut f = File::open("test-roms/6502_functional_test.bin").unwrap();
    let mut rom = Vec::<u8>::new();
    f.read_to_end(&mut rom).unwrap();
    let mut cpu = CPU::new();

    cpu.memory.set_bytes(Address(PC_START), &rom);

    cpu.run();

    /*
        let mut last_pc = PC_START;
    
        loop {
            cpu.step();
            // Prevent endless loop
            if cpu.interconnect.elapsed_cycles() > MAX_CYCLES {
                assert!(false, "Took too many cycles to complete");
            }
    
            if last_pc == cpu.registers.pc {
                if cpu.registers.pc == 0x3367 {
                    // Success!
                    break;
                } else {
                    assert!(false, "Trap detected");
                }
            }
    
            last_pc = cpu.registers.pc;
        }
    */
}

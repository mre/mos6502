use mos6502::cpu;
use mos6502::instruction::OpInput;
use mos6502::memory::Bus;
use mos6502::memory::Memory;
use std::collections::HashMap;
use std::fs::{read, File};
use std::io::{BufRead, BufReader};

fn main() {
    // Load the binary file from disk
    let program = match read("examples/asm/functional_test/6502_functional_test.bin") {
        Ok(data) => data,
        Err(err) => {
            println!("Error reading functional test: {}", err);
            return;
        }
    };

    let mut cpu = cpu::CPU::new(Memory::new());

    cpu.memory.set_bytes(0x00, &program);
    cpu.registers.program_counter = 0x400;

    let labels =
        load_labels("examples/asm/functional_test/labels.dbg").expect("Could not load labels");

    // run step-by-step
    let mut old_pc = cpu.registers.program_counter;
    while cpu.registers.program_counter != 0x3468 {
        // Use `fetch_next_and_decode` instead of
        // `single_step` to see the decoded instruction
        if let Some(decoded_instr) = cpu.fetch_next_and_decode() {
            let address = match decoded_instr.1 {
                OpInput::UseImmediate(v) => Some(v as u16),
                OpInput::UseRelative(v) => Some(v),
                OpInput::UseAddress(v) => Some(v),
                OpInput::UseImplied => None,
            };

            if let Some(address) = address {
                let label = labels.get(&address);
                match label {
                    Some(label) => println!("{} ({})", decoded_instr, label),
                    None => println!("{}", decoded_instr),
                }
            } else {
                println!("{}", decoded_instr);
            }

            // println!("Getting label for address: {:04X}", address);
            // let label = labels.get(&cpu.registers.program_counter);
            // match label {
            //     Some(name) => println!("{}: {}", name, decoded_instr),
            //     None => println!("{}", decoded_instr),
            // }
            cpu.execute_instruction(decoded_instr);
        }
        cpu.single_step();

        if cpu.registers.program_counter == old_pc {
            println!("Infinite loop detected!");
            println!("{cpu:?}");
            break;
        }

        old_pc = cpu.registers.program_counter;
    }
}

fn load_labels(path: &str) -> std::io::Result<HashMap<u16, String>> {
    let mut labels = HashMap::new();
    let file = File::open(path)?;
    let reader = BufReader::new(file);

    for line in reader.lines() {
        let line = line?;
        let parts: Vec<&str> = line.split(' ').collect();
        if parts.len() < 3 {
            continue;
        }
        let address = u16::from_str_radix(parts[1], 16).unwrap();
        let label = parts[2].to_owned();
        labels.insert(address, label);
    }

    Ok(labels)
}

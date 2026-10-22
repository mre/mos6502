//! NMOS 6502 final-state and instruction-cycle tests from SingleStepTests/65x02.
//! Only the 151 documented opcodes are covered. Bus-cycle order and dummy
//! accesses are not compared; the number of reference cycles is still checked.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

use mos6502::cpu::CPU;
use mos6502::instruction::Nmos6502;
use mos6502::memory::Bus;
use mos6502::registers::{StackPointer, Status};
use serde::{Deserialize, Serialize};

const REVISION: &str = include_str!("assets/nmos6502/REVISION");
const OPCODES: &str = include_str!("assets/nmos6502/opcodes.json");
const SAMPLE: &str = include_str!("assets/nmos6502/sample.json");
const CASES_PER_OPCODE: usize = 10_000;
const SAMPLE_CASES_PER_OPCODE: usize = 8;

#[derive(Clone, Deserialize, Serialize)]
struct State {
    pc: u16,
    s: u8,
    a: u8,
    x: u8,
    y: u8,
    p: u8,
    ram: Vec<(u16, u8)>,
}

#[derive(Clone, Deserialize, Serialize)]
struct Test {
    name: String,
    initial: State,
    #[serde(rename = "final")]
    final_state: State,
    cycles: Vec<(u16, u8, String)>,
}

struct TestBus {
    ram: BTreeMap<u16, u8>,
    writable: BTreeSet<u16>,
    unexpected_accesses: Vec<u16>,
}

impl Bus for TestBus {
    fn get_byte(&mut self, address: u16) -> u8 {
        self.ram.get(&address).copied().unwrap_or_else(|| {
            self.unexpected_accesses.push(address);
            0
        })
    }

    fn set_byte(&mut self, address: u16, value: u8) {
        if !self.writable.contains(&address) {
            self.unexpected_accesses.push(address);
        }
        self.ram.insert(address, value);
    }
}

fn opcode(test: &Test) -> u8 {
    test.initial
        .ram
        .iter()
        .find_map(|&(address, value)| (address == test.initial.pc).then_some(value))
        .expect("Fixture must initialize the opcode at PC")
}

fn run_one(test: &Test) -> Result<(), String> {
    let mut cpu = CPU::new(
        TestBus {
            ram: test.initial.ram.iter().copied().collect(),
            writable: test
                .final_state
                .ram
                .iter()
                .map(|&(address, _)| address)
                .collect(),
            unexpected_accesses: Vec::new(),
        },
        Nmos6502,
    );
    let initial = &test.initial;
    cpu.registers.accumulator = initial.a;
    cpu.registers.index_x = initial.x;
    cpu.registers.index_y = initial.y;
    cpu.registers.stack_pointer = StackPointer(initial.s);
    cpu.registers.program_counter = initial.pc;
    cpu.registers.status = Status::from_bits_retain(initial.p);

    // Execute one instruction without polling external interrupt lines.
    let decoded = cpu.fetch_next_and_decode().ok_or("Opcode not decoded")?;
    cpu.execute_instruction(decoded);

    let expected = &test.final_state;
    let reg = &cpu.registers;
    let mut differences = Vec::new();
    // B and U are not physical status latches. Compare the six real flags;
    // status bytes pushed onto the stack are compared unmasked as RAM below.
    let status_mask = !(Status::PS_BRK | Status::PS_UNUSED).bits();
    for (name, actual, expected) in [
        ("A", u64::from(reg.accumulator), u64::from(expected.a)),
        ("X", u64::from(reg.index_x), u64::from(expected.x)),
        ("Y", u64::from(reg.index_y), u64::from(expected.y)),
        ("S", u64::from(reg.stack_pointer.0), u64::from(expected.s)),
        ("PC", u64::from(reg.program_counter), u64::from(expected.pc)),
        (
            "P",
            u64::from(reg.status.bits() & status_mask),
            u64::from(expected.p & status_mask),
        ),
        (
            "cycles",
            cpu.cycles,
            u64::try_from(test.cycles.len()).unwrap(),
        ),
    ] {
        if actual != expected {
            differences.push(format!("{name}: got {actual:#x}, expected {expected:#x}"));
        }
    }
    for &(address, expected) in &expected.ram {
        let actual = cpu.memory.ram.get(&address).copied();
        if actual != Some(expected) {
            differences.push(format!(
                "RAM[{address:#06x}]: got {actual:?}, expected {expected:#04x}"
            ));
        }
    }
    if !cpu.memory.unexpected_accesses.is_empty() {
        differences.push(format!(
            "Accessed uninitialized RAM: {:04x?}",
            cpu.memory.unexpected_accesses
        ));
    }
    if differences.is_empty() {
        Ok(())
    } else {
        Err(differences.join(", "))
    }
}

fn documented_opcodes() -> Vec<u8> {
    let opcodes: Vec<u8> = serde_json::from_str(OPCODES).unwrap();
    assert_eq!(opcodes.len(), 151);
    assert!(opcodes.windows(2).all(|pair| pair[0] < pair[1]));
    opcodes
}

fn check_cases(tests: &[Test]) -> usize {
    let mut failures = 0;
    for test in tests {
        if let Err(reason) = run_one(test) {
            if failures < 5 {
                eprintln!(
                    "opcode {:02x}, case {:?}, PC {:04x}: {reason}",
                    opcode(test),
                    test.name,
                    test.initial.pc
                );
            }
            failures += 1;
        }
    }
    failures
}

#[test]
fn nmos_single_step_sample() {
    let tests: Vec<Test> = serde_json::from_str(SAMPLE).expect("Parse bundled NMOS sample");
    let mut counts = BTreeMap::new();
    for test in &tests {
        *counts.entry(opcode(test)).or_insert(0) += 1;
    }
    assert_eq!(
        counts.keys().copied().collect::<Vec<_>>(),
        documented_opcodes()
    );
    assert!(
        counts
            .values()
            .all(|&count| count == SAMPLE_CASES_PER_OPCODE)
    );
    assert_eq!(check_cases(&tests), 0, "Bundled NMOS cases failed");
}

fn parse_fixture(data: &[u8], expected_opcode: u8) -> Result<Vec<Test>, String> {
    let tests: Vec<Test> = serde_json::from_slice(data).map_err(|error| error.to_string())?;
    if tests.len() != CASES_PER_OPCODE {
        return Err(format!(
            "Expected {CASES_PER_OPCODE} cases, got {}",
            tests.len()
        ));
    }
    if tests.iter().any(|test| {
        !test
            .initial
            .ram
            .contains(&(test.initial.pc, expected_opcode))
    }) {
        return Err(format!("Wrong or missing opcode {expected_opcode:02x}"));
    }
    Ok(tests)
}

fn load_fixture(dir: &Path, opcode: u8) -> Vec<Test> {
    let path = dir.join(format!("{opcode:02x}.json"));
    let data = std::fs::read(&path)
        .unwrap_or_else(|error| panic!("{}: {error}; run cargo xtask fetch-nmos", path.display()));
    parse_fixture(&data, opcode)
        .unwrap_or_else(|error| panic!("{}: {error}; run cargo xtask fetch-nmos", path.display()))
}

#[test]
#[ignore = "run cargo xtask fetch-nmos first, then run explicitly with --ignored"]
fn nmos_single_step_full() {
    let dir = std::env::var_os("NMOS6502_TESTS").map_or_else(
        || PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("target/nmos6502/v1"),
        PathBuf::from,
    );
    let revision = std::fs::read_to_string(dir.join("REVISION"))
        .unwrap_or_else(|error| panic!("{}: {error}; run cargo xtask fetch-nmos", dir.display()));
    assert_eq!(
        revision.trim(),
        REVISION.trim(),
        "Wrong NMOS fixture revision"
    );
    let mut failures = 0;
    let mut total = 0;
    for opcode in documented_opcodes() {
        let tests = load_fixture(&dir, opcode);
        let failed = check_cases(&tests);
        eprintln!(
            "{opcode:02x}: {}/{} passed",
            tests.len() - failed,
            tests.len()
        );
        failures += failed;
        total += tests.len();
    }
    assert_eq!(total, documented_opcodes().len() * CASES_PER_OPCODE);
    assert_eq!(failures, 0, "{failures}/{total} NMOS cases failed");
}

#[test]
fn fixture_validation_rejects_invalid_data() {
    assert!(parse_fixture(b"not json", 0xA9).is_err());
    assert!(parse_fixture(SAMPLE.as_bytes(), 0xA9).is_err());
    let tests: Vec<Test> = serde_json::from_str(SAMPLE).unwrap();
    let test = tests.iter().find(|test| opcode(test) == 0xA9).unwrap();
    let data = serde_json::to_vec(&vec![test; CASES_PER_OPCODE]).unwrap();
    assert!(parse_fixture(&data, 0xA9).is_ok());
    assert!(parse_fixture(&data, 0xA2).is_err());
}

#[test]
fn harness_detects_wrong_expectations() {
    let tests: Vec<Test> = serde_json::from_str(SAMPLE).unwrap();
    let test = tests.iter().find(|test| opcode(test) == 0xA9).unwrap();
    assert!(run_one(test).is_ok());
    let mut bad = test.clone();
    bad.final_state.a ^= 1;
    assert!(run_one(&bad).unwrap_err().contains("A:"));
    let mut bad = test.clone();
    bad.final_state.p ^= Status::PS_CARRY.bits();
    assert!(run_one(&bad).unwrap_err().contains("P:"));
    let mut bad = test.clone();
    bad.final_state.ram[0].1 ^= 1;
    assert!(run_one(&bad).unwrap_err().contains("RAM["));
    let mut bad = test.clone();
    bad.cycles.pop();
    assert!(run_one(&bad).unwrap_err().contains("cycles:"));
}

// Instruction-cycle totals from dp111/6502Timing's vector build.
// https://github.com/dp111/6502Timing
// This checks documented and undocumented NMOS instructions, not bus-cycle
// accuracy. The upstream suite excludes BRK and JAM/HALT instructions.

use mos6502::cpu::CPU;
use mos6502::instruction::Nmos6502;
use mos6502::memory::{Bus, Memory};

const PROGRAM: &[u8] = include_bytes!("assets/6502_timing_test.bin");
const EXPECTED_MEASUREMENTS: usize = 346;
const PROGRAM_LOAD_ADDR: u16 = 0x2000;
const VECTOR_PRINTCHAR: u16 = 0x2010;
const VECTOR_INIT: u16 = 0x2020;
const VECTOR_TIMER_START: u16 = 0x2030;
const VECTOR_TIMER_READ: u16 = 0x2040;
const VECTOR_END_TEST: u16 = 0x2050;
const MAX_INSTRUCTIONS: u64 = 200_000;

#[derive(Default)]
struct Timer {
    start: Option<(u64, u8)>,
}

impl Timer {
    fn start(&mut self, cycles: u64, preset: u8) {
        assert!(
            self.start.replace((cycles, preset)).is_none(),
            "Timer restarted before reading"
        );
    }

    fn read(&mut self, cycles: u64) -> (u8, bool) {
        let (start, preset) = self.start.take().expect("Timer read before starting");
        // The ROM's TIME macro presets (expected_instruction_cycles + 8) * 2
        // and normally runs each instruction twice. Its fixed allowance is 16
        // cycles. Our patched start RTS and stop JSR account for 12 of those
        // between the two hooks, so include the remaining four here. Unlike an
        // elapsed-time counter, the ROM expects zero when timing is correct,
        // positive when execution is too quick, and negative when too slow.
        let elapsed = cycles - start + 4;
        (
            preset.wrapping_sub(elapsed.to_le_bytes()[0]),
            elapsed == u64::from(preset),
        )
    }
}

struct Report {
    failures: u8,
    measurements: usize,
    timing_errors: usize,
    output: String,
}

fn run_program(program: &[u8]) -> Report {
    let mut memory = Memory::new();
    memory.set_bytes(PROGRAM_LOAD_ADDR, program);
    // Host hooks supply output and timer values; real RTS instructions preserve
    // the ROM's stack behavior and contribute their normal six cycles.
    for vector in [VECTOR_PRINTCHAR, VECTOR_INIT, VECTOR_TIMER_START] {
        memory.set_byte(vector, 0x60);
    }
    // LDX #result sets N/Z for the ROM's CHECK routine; changing X directly
    // would leave stale flags and could silently accept an incorrect timing.
    memory.set_bytes(VECTOR_TIMER_READ, &[0xA2, 0x00, 0x60]);

    let mut cpu = CPU::new(memory, Nmos6502);
    cpu.registers.program_counter = PROGRAM_LOAD_ADDR;
    let mut timer = Timer::default();
    let mut output = String::new();
    let mut measurements = 0;
    let mut timing_errors = 0;

    for _ in 0..MAX_INSTRUCTIONS {
        let pc = cpu.registers.program_counter;
        match pc {
            VECTOR_PRINTCHAR => output.push(match cpu.registers.accumulator {
                b'\r' => '\n',
                byte => char::from(byte),
            }),
            VECTOR_TIMER_START => timer.start(cpu.cycles, cpu.registers.accumulator),
            VECTOR_TIMER_READ => {
                let (remaining, matches_expected) = timer.read(cpu.cycles);
                measurements += 1;
                timing_errors += usize::from(!matches_expected);
                cpu.memory.set_byte(VECTOR_TIMER_READ + 1, remaining);
            }
            VECTOR_END_TEST => {
                assert!(timer.start.is_none(), "Test ended with a running timer");
                return Report {
                    failures: cpu.registers.accumulator,
                    measurements,
                    timing_errors,
                    output,
                };
            }
            _ => {}
        }

        let opcode = cpu.memory.get_byte(pc);
        let decoded = cpu
            .fetch_next_and_decode()
            .unwrap_or_else(|| panic!("Unsupported opcode ${opcode:02X} at ${pc:04X}\n{output}"));
        cpu.execute_instruction(decoded);
    }

    panic!(
        "Timing ROM exceeded {MAX_INSTRUCTIONS} instructions at ${:04X}\n{output}",
        cpu.registers.program_counter
    );
}

#[test]
fn timing_test_6502() {
    let report = run_program(PROGRAM);
    eprintln!("{}", report.output);
    eprintln!("{} timing measurements", report.measurements);
    assert_eq!(
        report.measurements, EXPECTED_MEASUREMENTS,
        "ROM did not exercise the full suite"
    );
    assert_eq!(report.failures, 0, "{}", report.output);
    // Also catch the initial timer calibration error (the ROM resets its
    // failure counter afterwards), an 8-bit failure-count wrap, or a measured
    // duration that is wrong by a multiple of 256 cycles.
    assert_eq!(report.timing_errors, 0, "{}", report.output);
}

#[test]
fn timing_rom_detects_wrong_instruction_timing() {
    let mut program = PROGRAM.to_vec();
    // Replace the two NOPs in TIME 2 : NOP : NOP : STOP with one zero-page
    // unofficial NOP (three cycles rather than four, same two-byte footprint).
    // This changes the measured workload without disturbing the ROM's stack.
    let sequence = [0xA9, 20, 0x20, 0x30, 0x20, 0xEA, 0xEA, 0x20, 0x40, 0x20];
    let positions: Vec<_> = program
        .windows(sequence.len())
        .enumerate()
        .filter_map(|(index, bytes)| (bytes == sequence).then_some(index))
        .collect();
    assert_eq!(
        positions.len(),
        1,
        "Expected one documented NOP timing case"
    );
    let index = positions[0] + 5;
    program[index..index + 2].copy_from_slice(&[0x04, 0x00]);

    let report = run_program(&program);
    assert_eq!(report.measurements, EXPECTED_MEASUREMENTS);
    assert_eq!(report.failures, 1, "{}", report.output);
    assert_eq!(report.timing_errors, 1, "{}", report.output);
    assert!(report.output.contains("NOP"), "{}", report.output);
}

#[test]
fn countdown_timer_reports_signed_errors() {
    for (elapsed, expected, matches_expected) in [
        (12, 0, true),
        (11, 1, false),
        (13, 0xFF, false),
        (268, 0, false),
    ] {
        let mut timer = Timer::default();
        timer.start(100, 16);
        assert_eq!(timer.read(100 + elapsed), (expected, matches_expected));
    }
}

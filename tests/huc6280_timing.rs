use mos6502::cpu::CPU;
use mos6502::instruction::{Huc6280, Nmos6502};
use mos6502::memory::Bus;
use mos6502::registers::Status;

#[derive(Clone)]
struct TickBus {
    mem: [u8; 0x10000],
    ticks: Vec<u64>,
    irq: bool,
}

impl Default for TickBus {
    fn default() -> Self {
        Self {
            mem: [0; 0x10000],
            ticks: Vec::new(),
            irq: false,
        }
    }
}

impl TickBus {
    fn with_program(program: &[u8]) -> Self {
        let mut bus = Self::default();
        bus.mem[..program.len()].copy_from_slice(program);
        bus
    }
}

impl Bus for TickBus {
    fn tick(&mut self, cycles: u64) {
        self.ticks.push(cycles);
    }

    fn get_byte(&mut self, address: u16) -> u8 {
        self.mem[address as usize]
    }

    fn set_byte(&mut self, address: u16, value: u8) {
        self.mem[address as usize] = value;
    }

    fn irq_pending(&mut self) -> bool {
        self.irq
    }

    fn irq_vector(&mut self) -> u16 {
        0x1000
    }
}

#[test]
fn huc6280_taken_branch_costs_four_cycles() {
    let mut cpu = CPU::new(TickBus::with_program(&[0xD0, 0x02]), Huc6280); // BNE +2
    cpu.registers.status.remove(Status::PS_ZERO);

    cpu.single_step();

    assert_eq!(cpu.cycles, 4);
    assert_eq!(cpu.registers.program_counter, 0x0004);
}

#[test]
fn nmos_taken_branch_keeps_standard_three_cycle_timing() {
    let mut cpu = CPU::new(TickBus::with_program(&[0xD0, 0x02]), Nmos6502); // BNE +2
    cpu.registers.status.remove(Status::PS_ZERO);

    cpu.single_step();

    assert_eq!(cpu.cycles, 3);
    assert_eq!(cpu.registers.program_counter, 0x0004);
}

#[test]
fn huc6280_irq_dispatch_adds_eight_cycles() {
    let mut bus = TickBus::with_program(&[0xEA]); // NOP
    bus.irq = true;
    bus.mem[0x1000] = 0x34;
    bus.mem[0x1001] = 0x12;
    let mut cpu = CPU::new(bus, Huc6280);
    cpu.registers.status.remove(Status::PS_DISABLE_INTERRUPTS);

    cpu.single_step();

    assert_eq!(cpu.cycles, 10);
    assert_eq!(cpu.registers.program_counter, 0x1234);
}

#[test]
fn huc6280_block_transfer_ticks_each_byte() {
    // TII $1000,$2000,$0002
    let mut bus = TickBus::with_program(&[0x73, 0x00, 0x10, 0x00, 0x20, 0x02, 0x00]);
    bus.mem[0x1000] = 0xAA;
    bus.mem[0x1001] = 0x55;
    let mut cpu = CPU::new(bus, Huc6280);

    cpu.single_step();

    assert_eq!(cpu.cycles, 29);
    assert_eq!(cpu.memory.mem[0x2000], 0xAA);
    assert_eq!(cpu.memory.mem[0x2001], 0x55);
    assert_eq!(cpu.memory.ticks, [17, 6, 6]);
}

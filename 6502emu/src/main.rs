mod Registers {
	// Each status flag should be 0 or 1.
	pub struct Status {
		pub carry:        u8,
		pub zero:         u8,
		pub interrupt:    u8,
		pub decimal_mode: u8,
		pub brk:          u8,
		pub unused:       u8,
		pub overflow:     u8,
		pub sign:         u8
	}

	impl Status {
		pub fn to_byte(&self) -> u8 {
			  self.carry        << 0
			| self.zero         << 1
			| self.interrupt    << 2
			| self.decimal_mode << 3
			| self.brk          << 4
			| self.unused       << 5
			| self.overflow     << 6
			| self.sign         << 7
		}
	}

	pub struct Registers {
		pub accumulator:     i8,
		pub index_x:         i8,
		pub index_y:         i8,
		pub stack_pointer:   u8,
		pub program_counter: u16,
		pub status:          Status
	}

	impl Registers {
	}
}

mod Address {
	#[deriving(PartialEq)]
	#[deriving(Eq)]
	#[deriving(PartialOrd)]
	#[deriving(Ord)]
	pub struct Address(u16);

	impl Address {
		/* TODO akeeton: Hide struct Address(u16) "constructor."
		pub fn new(address_: u16) -> Address {
			Address(address_)
		}
		*/

		pub fn to_int(&self) -> u16 {
			match *self {
				Address(address_) => address_
			}
		}

		pub fn min() -> Address { Address(0x0100) }
		pub fn max() -> Address { Address(0x01ff) }

		pub fn is_valid(&self) -> bool {
			Address::min() <= *self && *self <= Address::max()
		}
	}
}

mod Memory {
	use Address::Address;

	pub struct Memory {
		bytes: [u8,.. 256]
	}

	impl Memory {
		fn address_to_byte_offset(address: &Address) -> uint {
			(address.to_int() - Address::min().to_int()) as uint
		}

		pub fn get_byte(&self, address: &Address) -> u8 {
			if !address.is_valid()
			{
				fail!("Invalid address.");
			}
			else
			{
				return self.bytes[Memory::address_to_byte_offset(address)];
			}
		}

		// Sets the byte at the given address to the given value and returns the
		// previous value at the address.
		pub fn set_byte(&mut self, address: &Address, value: u8) -> u8 {
			if !address.is_valid()
			{
				fail!("Invalid address.");
			}
			else
			{
				let old_value = self.get_byte(address);
				self.bytes[Memory::address_to_byte_offset(address)] = value;

				return old_value;
			}
		}
	}
}

mod Machine {
	use Registers::Registers;
	use Address::Address;
	use Memory::Memory;

	struct Machine {
		registers: Registers,
		memory:    Memory
	}

	impl Machine {
		// TODO akeeton: Implement binary-coded decimal.
		pub fn add_with_carry(&mut self, value: i8) {
			let a: int = self.registers.accumulator  as int;
			let c: int = self.registers.status.carry as int;

			let a_new_full: int = a + c + value as int;
			let a_new:      i8  = a_new_full as i8;

			self.registers.accumulator     = a_new;
			self.registers.status.carry    = if a_new_full == a_new as int { 0 } else { 1 };
			self.registers.status.zero     = if a_new == 0 { 1 } else { 0 };
			self.registers.status.sign     = if a_new  < 0 { 1 } else { 0 };
			self.registers.status.overflow =
				if (a < 0 && value < 0 && a_new >= 0)
				|| (a > 0 && value > 0 && a_new <= 0)
				{ 1 }
				else
				{ 0 }
		}
	}
}

fn main() {
	println!("Hello, world!")
}

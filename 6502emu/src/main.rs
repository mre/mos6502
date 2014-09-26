struct Processor {
	accumulator:     u8,
	index_x:         u8,
	index_y:         u8,
	stack_pointer:   u8,
	program_counter: u16,
	status:          u8,
}

impl Processor {
}

#[deriving(PartialEq)]
#[deriving(Eq)]
#[deriving(PartialOrd)]
#[deriving(Ord)]
struct Address(u16);

impl Address {
	fn to_int(&self) -> u16 {
		match *self {
			Address(address_) => address_
		}
	}

	fn min() -> Address { Address(0x0100) }
	fn max() -> Address { Address(0x01ff) }

	fn is_valid(&self) -> bool {
		Address::min() <= *self && *self <= Address::max()
	}
}

struct Memory {
	bytes: [u8,.. 256]
}

impl Memory {
	fn address_to_byte_offset(address: &Address) -> uint {
		(address.to_int() - Address::min().to_int()) as uint
	}

	fn get_byte(&self, address: &Address) -> u8 {
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
	fn set_byte(&mut self, address: &Address, value: u8) -> u8 {
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

fn main() {
	println!("Hello, world!")
}

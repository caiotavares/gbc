use crate::instruction::extended::ExtendedInstruction;
use crate::instruction::regular::Instruction;
use crate::memory::Memory;
use crate::*;

const CLOCK: f32 = 8.388608;

enum Flag {
    Z,
    N,
    H,
    C,
}

enum Register8bit {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
}

enum Register16bit {
    AF,
    BC,
    DE,
    HL,
    SP,
    PC,
}

#[derive(Debug)]
struct Registers {
    af: (u8, u8), // Accumulator & Flags register
    bc: (u8, u8),
    de: (u8, u8),
    hl: (u8, u8),
    sp: u16, // Stack Pointer
    pc: u16, // Program Counter
}

impl Registers {
    pub fn init() -> Registers {
        // Initial values for registers obtained from Pandocs
        Registers {
            af: (0x11, 0x80),
            bc: (0x00, 0x00),
            de: (0xFF, 0x56),
            hl: (0x00, 0x0D),
            pc: 0x0100, // ROM data starts at 0x0100, ignoring the bootloader checks
            sp: 0xFFFE,
        }
    }

    pub fn set_flags(
        &mut self,
        z: Option<bool>,
        n: Option<bool>,
        h: Option<bool>,
        c: Option<bool>,
    ) {
    }

    pub fn get_flag(&self, flag: Flag) -> u8 {
        let register = self.af.1;
        match flag {
            Flag::Z => register & 0x80,
            Flag::N => register & 0x40,
            Flag::H => register & 0x20,
            Flag::C => register & 0x10,
        }
    }

    pub fn set(&mut self, register: Register8bit, value: u8) {
        match register {
            Register8bit::A => self.af.0 = value,
            Register8bit::B => self.bc.0 = value,
            Register8bit::C => self.bc.1 = value,
            Register8bit::D => self.de.0 = value,
            Register8bit::E => self.de.1 = value,
            Register8bit::H => self.hl.0 = value,
            Register8bit::L => self.hl.1 = value,
        }
    }

    pub fn set_16bit(&mut self, register: Register16bit, msb: u8, lsb: u8) {
        match register {
            Register16bit::AF => self.af = (msb, lsb),
            Register16bit::BC => self.bc = (msb, lsb),
            Register16bit::DE => self.de = (msb, lsb),
            Register16bit::HL => self.hl = (msb, lsb),
            Register16bit::SP => self.sp = as_u16(msb, lsb),
            Register16bit::PC => self.pc = as_u16(msb, lsb),
        }
    }

    pub fn as_8bit(&mut self, register: &Register8bit) -> u8 {
        match register {
            Register8bit::A => self.af.0,
            Register8bit::B => self.bc.0,
            Register8bit::C => self.bc.1,
            Register8bit::D => self.de.0,
            Register8bit::E => self.de.1,
            Register8bit::H => self.hl.0,
            Register8bit::L => self.hl.1,
        }
    }

    pub fn as_16bit(&self, register: &Register16bit) -> u16 {
        match register {
            Register16bit::AF => self.af.join(),
            Register16bit::BC => self.bc.join(),
            Register16bit::DE => self.de.join(),
            Register16bit::HL => self.hl.join(),
            Register16bit::SP => self.sp,
            Register16bit::PC => self.pc,
        }
    }
}

struct Clock {
    cycles: u8,
    clock_speed: usize,
}

impl Clock {
    pub fn init() -> Clock {
        Clock {
            cycles: 0,
            clock_speed: CLOCK as usize,
        }
    }
}

pub struct CPU {
    memory: Memory,
    registers: Registers,
    clock: Clock,
}

impl CPU {
    pub fn new(memory: Memory) -> CPU {
        CPU {
            memory,
            registers: Registers::init(),
            clock: Clock::init(),
        }
    }

    pub fn init(&mut self) {
        // TODO: Should this be constrained according to CPU clock?
        loop {
            let data = CPU::fetch(&mut self.registers.pc, &self.memory);
            let ins = Instruction::decode(data);
            if ins == Instruction::STOP {
                break;
            } else {
                self.execute(ins);
            }
        }
    }

    fn inc_r8(&mut self, register: Register8bit) {
        let data = self.registers.as_8bit(&register);
        if data == 0xFF {
            self.registers.set(register, 0x00);
            self.registers
                .set_flags(Some(true), Some(false), Some(true), None);
        } else {
            self.registers.set(register, data + 1);
            self.registers.set_flags(None, Some(false), None, None);
        };
        self.clock.cycles += 1;
    }

    fn dec_r8(&mut self, register: Register8bit) {
        let data = self.registers.as_8bit(&register);
        if data == 0x00 {
            self.registers.set(register, 0xFF);
            self.registers
                .set_flags(Some(true), Some(false), Some(true), None);
        } else {
            self.registers.set(register, data - 1);
            self.registers.set_flags(None, Some(false), None, None);
        };
        self.clock.cycles += 1;
    }

    fn load_r8_n8(&mut self, register: Register8bit) {
        let data = CPU::fetch(&mut self.registers.pc, &self.memory);
        self.registers.set(register, data);
        self.clock.cycles += 2;
    }

    fn load_r8_r8(&mut self, to: Register8bit, from: Register8bit) {
        let data = self.registers.as_8bit(&from);
        self.registers.set(to, data);
        self.clock.cycles += 2;
    }

    fn load_r16_n16(&mut self, register: Register16bit) {
        let lsb = CPU::fetch(&mut self.registers.pc, &self.memory);
        let msb = CPU::fetch(&mut self.registers.pc, &self.memory);
        self.registers.set_16bit(register, msb, lsb);
        self.clock.cycles += 3;
    }

    fn load_r16_a(&mut self, register: Register16bit) {
        let address = self.registers.as_16bit(&register);
        let data = self.registers.as_8bit(&Register8bit::A);
        self.memory.write(address, data);
        self.clock.cycles += 2;
    }

    fn load_a_r16(&mut self, register: Register16bit) {
        let address = self.registers.as_16bit(&register);
        let data = self.memory.read(address);
        self.registers.set(Register8bit::A, data);
        self.clock.cycles += 2;
    }

    fn load_r8_hl(&mut self, register: Register8bit) {
        let address = self.registers.as_16bit(&Register16bit::HL);
        let data = self.memory.read(address);
        self.registers.set(register, data);
        self.clock.cycles += 2;
    }

    fn load_hl_r8(&mut self, register: Register8bit) {
        let address = self.registers.as_16bit(&Register16bit::HL);
        let data = self.registers.as_8bit(&register);
        self.memory.write(address, data);
        self.clock.cycles += 2;
    }

    fn add_a_r8(&mut self, register: Register8bit) {
        let a = self.registers.as_8bit(&Register8bit::A);
        let r8 = self.registers.as_8bit(&register);
        let value = a + r8;
        self.registers.set(Register8bit::A, value);
        self.clock.cycles += 1;
    }

    fn add_a_hl(&mut self) {
        let a = self.registers.as_8bit(&Register8bit::A);
        let address = self.registers.as_16bit(&Register16bit::HL);
        let data = self.memory.read(address);
        let value = a + data;
        self.registers.set(Register8bit::A, value);
        self.clock.cycles += 1;
    }

    fn adc_a_r8(&mut self, register: Register8bit) {
        // TODO: Set carry and half-carry
        let a = self.registers.as_8bit(&Register8bit::A);
        let r8 = self.registers.as_8bit(&register);
        let carry = self.registers.get_flag(Flag::C);
        let value = a + r8 + carry;
        self.registers.set(Register8bit::A, value);
        self.registers
            .set_flags(Some(value == 0), Some(false), Some(true), Some(false));
        self.clock.cycles += 1;
    }

    fn sub_a_r8(&mut self, register: Register8bit) {
        let a = self.registers.as_8bit(&Register8bit::A);
        let r8 = self.registers.as_8bit(&register);
        let value = a - r8;
        self.registers.set(Register8bit::A, value);
        self.clock.cycles += 1;
    }

    fn sub_a_hl(&mut self) {
        let a = self.registers.as_8bit(&Register8bit::A);
        let address = self.registers.as_16bit(&Register16bit::HL);
        let data = self.memory.read(address);
        let value = a - data;
        self.registers.set(Register8bit::A, value);
        self.clock.cycles += 1;
    }

    fn and_a_r8(&mut self, register: Register8bit) {
        let a = self.registers.as_8bit(&Register8bit::A);
        let data = self.registers.as_8bit(&register);
        let value = a & data;
        self.registers.set(Register8bit::A, value);
        self.registers
            .set_flags(Some(value == 0), Some(false), Some(true), Some(false));
        self.clock.cycles += 1;
    }

    fn and_a_hl(&mut self) {
        let a = self.registers.as_8bit(&Register8bit::A);
        let address = self.registers.as_16bit(&Register16bit::HL);
        let data = self.memory.read(address);
        let value = a & data;
        self.registers.set(Register8bit::A, value);
        self.registers
            .set_flags(Some(value == 0), Some(false), Some(true), Some(false));
        self.clock.cycles += 1;
    }

    fn or_a_r8(&mut self, register: Register8bit) {
        let a = self.registers.as_8bit(&Register8bit::A);
        let data = self.registers.as_8bit(&register);
        let value = a | data;
        self.registers.set(Register8bit::A, value);
        self.registers
            .set_flags(Some(value == 0), Some(false), Some(false), Some(false));
        self.clock.cycles += 1;
    }

    fn or_a_hl(&mut self) {
        let a = self.registers.as_8bit(&Register8bit::A);
        let address = self.registers.as_16bit(&Register16bit::HL);
        let data = self.memory.read(address);
        let value = a | data;
        self.registers.set(Register8bit::A, value);
        self.clock.cycles += 1;
    }

    fn xor_a_r8(&mut self, register: Register8bit) {
        let a = self.registers.as_8bit(&Register8bit::A);
        let data = self.registers.as_8bit(&register);
        let value = a ^ data;
        self.registers.set(Register8bit::A, value);
        self.clock.cycles += 1;
    }

    fn xor_a_hl(&mut self) {
        let a = self.registers.as_8bit(&Register8bit::A);
        let address = self.registers.as_16bit(&Register16bit::HL);
        let data = self.memory.read(address);
        let value = a ^ data;
        self.registers.set(Register8bit::A, value);
        self.clock.cycles += 1;
    }

    fn cp_a_r8(&mut self, register: Register8bit) {
        let a = self.registers.as_8bit(&Register8bit::A);
        let r8 = self.registers.as_8bit(&register);
        let zero = a == r8;
        let carry = a < r8;
        let half_carry = a.low_nibble() < r8.low_nibble();
        self.registers
            .set_flags(Some(zero), Some(true), Some(half_carry), Some(carry));
        self.clock.cycles += 1;
    }

    fn cp_a_hl(&mut self) {
        let a = self.registers.as_8bit(&Register8bit::A);
        let address = self.registers.as_16bit(&Register16bit::HL);
        let data = self.memory.read(address);
        let zero = a == data;
        let carry = a < data;
        let half_carry = a.low_nibble() < data.low_nibble();
        self.registers
            .set_flags(Some(zero), Some(true), Some(half_carry), Some(carry));
        self.clock.cycles += 2;
    }

    fn extended(&mut self) {
        let data = CPU::fetch(&mut self.registers.pc, &self.memory);
        let instruction = ExtendedInstruction::decode(data);
        self.execute_extended(instruction);
    }

    fn fetch(pc: &mut u16, memory: &Memory) -> u8 {
        let data = memory.read(*pc);
        *pc += 1;
        data
    }

    fn execute(&mut self, ins: Instruction) {
        match ins {
            Instruction::NOP => self.clock.cycles += 1,
            Instruction::Invalid => {}
            Instruction::CB => self.extended(),
            Instruction::LD_A_u8 => self.load_r8_n8(Register8bit::A),
            Instruction::LD_B_u8 => self.load_r8_n8(Register8bit::B),
            Instruction::LD_C_u8 => self.load_r8_n8(Register8bit::C),
            Instruction::LD_D_u8 => self.load_r8_n8(Register8bit::D),
            Instruction::LD_E_u8 => self.load_r8_n8(Register8bit::E),
            Instruction::LD_H_u8 => self.load_r8_n8(Register8bit::H),
            Instruction::LD_L_u8 => self.load_r8_n8(Register8bit::L),
            Instruction::LD_BC_u16 => self.load_r16_n16(Register16bit::BC),
            Instruction::LD_DE_u16 => self.load_r16_n16(Register16bit::DE),
            Instruction::LD_HL_u16 => self.load_r16_n16(Register16bit::HL),
            Instruction::INC_A => self.inc_r8(Register8bit::A),
            Instruction::INC_B => self.inc_r8(Register8bit::B),
            Instruction::INC_C => self.inc_r8(Register8bit::C),
            Instruction::INC_D => self.inc_r8(Register8bit::D),
            Instruction::INC_E => self.inc_r8(Register8bit::E),
            Instruction::INC_H => self.inc_r8(Register8bit::H),
            Instruction::INC_L => self.inc_r8(Register8bit::L),
            Instruction::DEC_A => self.dec_r8(Register8bit::A),
            Instruction::DEC_B => self.dec_r8(Register8bit::B),
            Instruction::DEC_C => self.dec_r8(Register8bit::C),
            Instruction::DEC_D => self.dec_r8(Register8bit::D),
            Instruction::DEC_E => self.dec_r8(Register8bit::E),
            Instruction::DEC_H => self.dec_r8(Register8bit::H),
            Instruction::DEC_L => self.dec_r8(Register8bit::L),
            Instruction::LD_BC_A => self.load_r16_a(Register16bit::BC),
            Instruction::LD_DE_A => self.load_r16_a(Register16bit::DE),
            Instruction::LD_A_BC => self.load_a_r16(Register16bit::BC),
            Instruction::LD_A_DE => self.load_a_r16(Register16bit::DE),
            Instruction::LD_HL_A_Plus => todo!(),
            Instruction::LD_HL_A_Minus => todo!(),
            Instruction::LD_A_HL_Plus => todo!(),
            Instruction::LD_A_HL_Minus => todo!(),
            Instruction::LD_HL_u8 => todo!(),
            Instruction::LD_B_A => self.load_r8_r8(Register8bit::B, Register8bit::A),
            Instruction::LD_B_B => self.load_r8_r8(Register8bit::B, Register8bit::B),
            Instruction::LD_B_C => self.load_r8_r8(Register8bit::B, Register8bit::C),
            Instruction::LD_B_D => self.load_r8_r8(Register8bit::B, Register8bit::D),
            Instruction::LD_B_E => self.load_r8_r8(Register8bit::B, Register8bit::E),
            Instruction::LD_B_H => self.load_r8_r8(Register8bit::B, Register8bit::H),
            Instruction::LD_B_L => self.load_r8_r8(Register8bit::B, Register8bit::L),
            Instruction::LD_B_HL => self.load_r8_hl(Register8bit::B),
            Instruction::LD_C_A => self.load_r8_r8(Register8bit::C, Register8bit::A),
            Instruction::LD_C_B => self.load_r8_r8(Register8bit::C, Register8bit::B),
            Instruction::LD_C_C => self.load_r8_r8(Register8bit::C, Register8bit::C),
            Instruction::LD_C_D => self.load_r8_r8(Register8bit::C, Register8bit::D),
            Instruction::LD_C_E => self.load_r8_r8(Register8bit::C, Register8bit::E),
            Instruction::LD_C_H => self.load_r8_r8(Register8bit::C, Register8bit::H),
            Instruction::LD_C_L => self.load_r8_r8(Register8bit::C, Register8bit::L),
            Instruction::LD_C_HL => self.load_r8_hl(Register8bit::C),
            Instruction::LD_D_A => self.load_r8_r8(Register8bit::D, Register8bit::A),
            Instruction::LD_D_B => self.load_r8_r8(Register8bit::D, Register8bit::B),
            Instruction::LD_D_C => self.load_r8_r8(Register8bit::D, Register8bit::C),
            Instruction::LD_D_D => self.load_r8_r8(Register8bit::D, Register8bit::D),
            Instruction::LD_D_E => self.load_r8_r8(Register8bit::D, Register8bit::E),
            Instruction::LD_D_H => self.load_r8_r8(Register8bit::D, Register8bit::H),
            Instruction::LD_D_L => self.load_r8_r8(Register8bit::D, Register8bit::L),
            Instruction::LD_D_HL => self.load_r8_hl(Register8bit::D),
            Instruction::LD_E_A => self.load_r8_r8(Register8bit::E, Register8bit::A),
            Instruction::LD_E_B => self.load_r8_r8(Register8bit::E, Register8bit::B),
            Instruction::LD_E_C => self.load_r8_r8(Register8bit::E, Register8bit::C),
            Instruction::LD_E_D => self.load_r8_r8(Register8bit::E, Register8bit::D),
            Instruction::LD_E_E => self.load_r8_r8(Register8bit::E, Register8bit::E),
            Instruction::LD_E_H => self.load_r8_r8(Register8bit::E, Register8bit::H),
            Instruction::LD_E_L => self.load_r8_r8(Register8bit::E, Register8bit::L),
            Instruction::LD_E_HL => self.load_r8_hl(Register8bit::E),
            Instruction::LD_H_A => self.load_r8_r8(Register8bit::H, Register8bit::A),
            Instruction::LD_H_B => self.load_r8_r8(Register8bit::H, Register8bit::B),
            Instruction::LD_H_C => self.load_r8_r8(Register8bit::H, Register8bit::C),
            Instruction::LD_H_D => self.load_r8_r8(Register8bit::H, Register8bit::D),
            Instruction::LD_H_E => self.load_r8_r8(Register8bit::H, Register8bit::E),
            Instruction::LD_H_H => self.load_r8_r8(Register8bit::H, Register8bit::H),
            Instruction::LD_H_L => self.load_r8_r8(Register8bit::H, Register8bit::L),
            Instruction::LD_H_HL => self.load_r8_hl(Register8bit::H),
            Instruction::LD_L_A => self.load_r8_r8(Register8bit::L, Register8bit::A),
            Instruction::LD_L_B => self.load_r8_r8(Register8bit::L, Register8bit::B),
            Instruction::LD_L_C => self.load_r8_r8(Register8bit::L, Register8bit::C),
            Instruction::LD_L_D => self.load_r8_r8(Register8bit::L, Register8bit::D),
            Instruction::LD_L_E => self.load_r8_r8(Register8bit::L, Register8bit::E),
            Instruction::LD_L_H => self.load_r8_r8(Register8bit::L, Register8bit::H),
            Instruction::LD_L_L => self.load_r8_r8(Register8bit::L, Register8bit::L),
            Instruction::LD_L_HL => self.load_r8_hl(Register8bit::L),
            Instruction::LD_HL_B => self.load_hl_r8(Register8bit::B),
            Instruction::LD_HL_C => self.load_hl_r8(Register8bit::C),
            Instruction::LD_HL_D => self.load_hl_r8(Register8bit::D),
            Instruction::LD_HL_E => self.load_hl_r8(Register8bit::E),
            Instruction::LD_HL_H => self.load_hl_r8(Register8bit::H),
            Instruction::LD_HL_L => self.load_hl_r8(Register8bit::L),
            Instruction::LD_SP_u16 => todo!(),
            Instruction::LD_u16_SP => todo!(),
            Instruction::DAA => todo!(),
            Instruction::SCF => todo!(),
            Instruction::CPL => todo!(),
            Instruction::CCF => todo!(),
            Instruction::INC_BC => todo!(),
            Instruction::INC_DE => todo!(),
            Instruction::INC_HL => todo!(),
            Instruction::INC_SP => todo!(),
            Instruction::DEC_BC => todo!(),
            Instruction::DEC_DE => todo!(),
            Instruction::DEC_HL => todo!(),
            Instruction::DEC_SP => todo!(),
            Instruction::ADD_HL_BC => todo!(),
            Instruction::ADD_HL_DE => todo!(),
            Instruction::ADD_HL_HL => todo!(),
            Instruction::ADD_HL_SP => todo!(),
            Instruction::RLCA => todo!(),
            Instruction::RRCA => todo!(),
            Instruction::RLA => todo!(),
            Instruction::RRA => todo!(),
            Instruction::JR_i8 => todo!(),
            Instruction::JR_NZ_i8 => todo!(),
            Instruction::JR_NC_i8 => todo!(),
            Instruction::JR_C_i8 => todo!(),
            Instruction::JR_Z_i8 => todo!(),
            Instruction::JP_u16 => todo!(),
            Instruction::STOP => todo!(),
            Instruction::HALT => todo!(),
            Instruction::LD_A_A => self.load_r8_r8(Register8bit::A, Register8bit::A),
            Instruction::LD_A_B => self.load_r8_r8(Register8bit::A, Register8bit::B),
            Instruction::LD_A_C => self.load_r8_r8(Register8bit::A, Register8bit::C),
            Instruction::LD_A_D => self.load_r8_r8(Register8bit::A, Register8bit::D),
            Instruction::LD_A_E => self.load_r8_r8(Register8bit::A, Register8bit::E),
            Instruction::LD_A_H => self.load_r8_r8(Register8bit::A, Register8bit::H),
            Instruction::LD_A_L => self.load_r8_r8(Register8bit::A, Register8bit::L),
            Instruction::LD_A_HL => self.load_r8_hl(Register8bit::A),
            Instruction::LD_HL_A => self.load_hl_r8(Register8bit::A),
            Instruction::ADD_A_A => self.add_a_r8(Register8bit::A),
            Instruction::ADD_A_B => self.add_a_r8(Register8bit::B),
            Instruction::ADD_A_C => self.add_a_r8(Register8bit::C),
            Instruction::ADD_A_D => self.add_a_r8(Register8bit::D),
            Instruction::ADD_A_E => self.add_a_r8(Register8bit::E),
            Instruction::ADD_A_H => self.add_a_r8(Register8bit::H),
            Instruction::ADD_A_L => self.add_a_r8(Register8bit::L),
            Instruction::ADD_A_HL => self.add_a_hl(),
            Instruction::ADC_A_A => self.adc_a_r8(Register8bit::A),
            Instruction::ADC_A_B => self.adc_a_r8(Register8bit::B),
            Instruction::ADC_A_C => self.adc_a_r8(Register8bit::C),
            Instruction::ADC_A_D => self.adc_a_r8(Register8bit::D),
            Instruction::ADC_A_E => self.adc_a_r8(Register8bit::E),
            Instruction::ADC_A_H => self.adc_a_r8(Register8bit::H),
            Instruction::ADC_A_L => self.adc_a_r8(Register8bit::L),
            Instruction::ADC_A_HL => todo!(),
            Instruction::SUB_A_A => self.sub_a_r8(Register8bit::A),
            Instruction::SUB_A_B => self.sub_a_r8(Register8bit::B),
            Instruction::SUB_A_C => self.sub_a_r8(Register8bit::C),
            Instruction::SUB_A_D => self.sub_a_r8(Register8bit::D),
            Instruction::SUB_A_E => self.sub_a_r8(Register8bit::E),
            Instruction::SUB_A_H => self.sub_a_r8(Register8bit::H),
            Instruction::SUB_A_L => self.sub_a_r8(Register8bit::L),
            Instruction::SUB_A_HL => self.sub_a_hl(),
            Instruction::SBC_A_A => todo!(),
            Instruction::SBC_A_B => todo!(),
            Instruction::SBC_A_C => todo!(),
            Instruction::SBC_A_D => todo!(),
            Instruction::SBC_A_E => todo!(),
            Instruction::SBC_A_H => todo!(),
            Instruction::SBC_A_L => todo!(),
            Instruction::SBC_A_HL => todo!(),
            Instruction::AND_A_A => self.and_a_r8(Register8bit::A),
            Instruction::AND_A_B => self.and_a_r8(Register8bit::B),
            Instruction::AND_A_C => self.and_a_r8(Register8bit::C),
            Instruction::AND_A_D => self.and_a_r8(Register8bit::D),
            Instruction::AND_A_E => self.and_a_r8(Register8bit::E),
            Instruction::AND_A_H => self.and_a_r8(Register8bit::H),
            Instruction::AND_A_L => self.and_a_r8(Register8bit::L),
            Instruction::AND_A_HL => self.and_a_hl(),
            Instruction::XOR_A_A => self.xor_a_r8(Register8bit::A),
            Instruction::XOR_A_B => self.xor_a_r8(Register8bit::B),
            Instruction::XOR_A_C => self.xor_a_r8(Register8bit::C),
            Instruction::XOR_A_D => self.xor_a_r8(Register8bit::D),
            Instruction::XOR_A_E => self.xor_a_r8(Register8bit::E),
            Instruction::XOR_A_H => self.xor_a_r8(Register8bit::H),
            Instruction::XOR_A_L => self.xor_a_r8(Register8bit::L),
            Instruction::XOR_A_HL => self.xor_a_hl(),
            Instruction::OR_A_A => self.or_a_r8(Register8bit::A),
            Instruction::OR_A_B => self.or_a_r8(Register8bit::B),
            Instruction::OR_A_C => self.or_a_r8(Register8bit::C),
            Instruction::OR_A_D => self.or_a_r8(Register8bit::D),
            Instruction::OR_A_E => self.or_a_r8(Register8bit::E),
            Instruction::OR_A_H => self.or_a_r8(Register8bit::H),
            Instruction::OR_A_L => self.or_a_r8(Register8bit::L),
            Instruction::OR_A_HL => self.or_a_hl(),
            Instruction::CP_A_A => self.cp_a_r8(Register8bit::A),
            Instruction::CP_A_B => self.cp_a_r8(Register8bit::B),
            Instruction::CP_A_C => self.cp_a_r8(Register8bit::C),
            Instruction::CP_A_D => self.cp_a_r8(Register8bit::D),
            Instruction::CP_A_E => self.cp_a_r8(Register8bit::E),
            Instruction::CP_A_H => self.cp_a_r8(Register8bit::H),
            Instruction::CP_A_L => self.cp_a_r8(Register8bit::L),
            Instruction::CP_A_HL => self.cp_a_hl(),
        }
    }

    fn execute_extended(&mut self, ins: ExtendedInstruction) {
        match ins {
            ExtendedInstruction::RLC_A => todo!(),
            ExtendedInstruction::RLC_B => todo!(),
            ExtendedInstruction::RLC_C => todo!(),
            ExtendedInstruction::RLC_D => todo!(),
            ExtendedInstruction::RLC_E => todo!(),
            ExtendedInstruction::RLC_H => todo!(),
            ExtendedInstruction::RLC_L => todo!(),
            ExtendedInstruction::RLC_HL => todo!(),
            ExtendedInstruction::RRC_A => todo!(),
            ExtendedInstruction::RRC_B => todo!(),
            ExtendedInstruction::RRC_C => todo!(),
            ExtendedInstruction::RRC_D => todo!(),
            ExtendedInstruction::RRC_E => todo!(),
            ExtendedInstruction::RRC_H => todo!(),
            ExtendedInstruction::RRC_L => todo!(),
            ExtendedInstruction::RRC_HL => todo!(),
            ExtendedInstruction::RL_A => todo!(),
            ExtendedInstruction::RL_B => todo!(),
            ExtendedInstruction::RL_C => todo!(),
            ExtendedInstruction::RL_D => todo!(),
            ExtendedInstruction::RL_E => todo!(),
            ExtendedInstruction::RL_H => todo!(),
            ExtendedInstruction::RL_L => todo!(),
            ExtendedInstruction::RL_HL => todo!(),
            ExtendedInstruction::RR_A => todo!(),
            ExtendedInstruction::RR_B => todo!(),
            ExtendedInstruction::RR_C => todo!(),
            ExtendedInstruction::RR_D => todo!(),
            ExtendedInstruction::RR_E => todo!(),
            ExtendedInstruction::RR_H => todo!(),
            ExtendedInstruction::RR_L => todo!(),
            ExtendedInstruction::RR_HL => todo!(),
            ExtendedInstruction::SLA_A => todo!(),
            ExtendedInstruction::SLA_B => todo!(),
            ExtendedInstruction::SLA_C => todo!(),
            ExtendedInstruction::SLA_D => todo!(),
            ExtendedInstruction::SLA_E => todo!(),
            ExtendedInstruction::SLA_H => todo!(),
            ExtendedInstruction::SLA_L => todo!(),
            ExtendedInstruction::SLA_HL => todo!(),
            ExtendedInstruction::SRA_A => todo!(),
            ExtendedInstruction::SRA_B => todo!(),
            ExtendedInstruction::SRA_C => todo!(),
            ExtendedInstruction::SRA_D => todo!(),
            ExtendedInstruction::SRA_E => todo!(),
            ExtendedInstruction::SRA_H => todo!(),
            ExtendedInstruction::SRA_L => todo!(),
            ExtendedInstruction::SRA_HL => todo!(),
            ExtendedInstruction::SWAP_A => todo!(),
            ExtendedInstruction::SWAP_B => todo!(),
            ExtendedInstruction::SWAP_C => todo!(),
            ExtendedInstruction::SWAP_D => todo!(),
            ExtendedInstruction::SWAP_E => todo!(),
            ExtendedInstruction::SWAP_H => todo!(),
            ExtendedInstruction::SWAP_L => todo!(),
            ExtendedInstruction::SWAP_HL => todo!(),
            ExtendedInstruction::SRL_A => todo!(),
            ExtendedInstruction::SRL_B => todo!(),
            ExtendedInstruction::SRL_C => todo!(),
            ExtendedInstruction::SRL_D => todo!(),
            ExtendedInstruction::SRL_E => todo!(),
            ExtendedInstruction::SRL_H => todo!(),
            ExtendedInstruction::SRL_L => todo!(),
            ExtendedInstruction::SRL_HL => todo!(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn new_test_cpu() -> CPU {
        CPU {
            memory: Memory::empty(),
            clock: Clock {
                cycles: 0,
                clock_speed: 0,
            },
            registers: Registers {
                af: (0x00, 0x00),
                bc: (0x00, 0x00),
                de: (0x00, 0x00),
                hl: (0x00, 0x00),
                sp: 0x0000,
                pc: 0x0000,
            },
        }
    }

    #[test]
    fn test_alu_r8() {
        let mut cpu = new_test_cpu();
        assert_eq!(cpu.registers.bc, (0x00, 0x00));
        cpu.inc_r8(Register8bit::B);
        assert_eq!(cpu.registers.bc, (0x01, 0x00));
        cpu.inc_r8(Register8bit::C);
        assert_eq!(cpu.registers.bc, (0x01, 0x01));
        cpu.dec_r8(Register8bit::B);
        assert_eq!(cpu.registers.bc, (0x00, 0x01));
        cpu.dec_r8(Register8bit::C);
        assert_eq!(cpu.registers.bc, (0x00, 0x00));
    }

    // TODO: Assert flags being set
    #[test]
    fn test_alu_r8_overflow() {
        let mut cpu = new_test_cpu();
        cpu.registers.set_16bit(Register16bit::BC, 0xFF, 0xFF);
        assert_eq!(cpu.registers.bc, (0xFF, 0xFF));
        cpu.inc_r8(Register8bit::C);
        assert_eq!(cpu.registers.bc, (0xFF, 0x00));
        cpu.inc_r8(Register8bit::B);
        assert_eq!(cpu.registers.bc, (0x00, 0x00));
        cpu.dec_r8(Register8bit::B);
        assert_eq!(cpu.registers.bc, (0xFF, 0x00));
        cpu.dec_r8(Register8bit::C);
        assert_eq!(cpu.registers.bc, (0xFF, 0xFF));
    }

    #[test]
    fn test_as8bit() {
        let mut cpu = new_test_cpu();
        cpu.registers = Registers {
            af: (0x01, 0x08),
            bc: (0x02, 0x03),
            de: (0x04, 0x05),
            hl: (0x06, 0x07),
            sp: 0x0000,
            pc: 0x0000,
        };
        assert_eq!(cpu.registers.as_8bit(&Register8bit::A), 0x01);
        assert_eq!(cpu.registers.as_8bit(&Register8bit::B), 0x02);
        assert_eq!(cpu.registers.as_8bit(&Register8bit::C), 0x03);
        assert_eq!(cpu.registers.as_8bit(&Register8bit::D), 0x04);
        assert_eq!(cpu.registers.as_8bit(&Register8bit::E), 0x05);
        assert_eq!(cpu.registers.as_8bit(&Register8bit::H), 0x06);
        assert_eq!(cpu.registers.as_8bit(&Register8bit::L), 0x07);
    }

    #[test]
    fn test_as16bit() {
        let mut cpu = new_test_cpu();
        cpu.registers = Registers {
            af: (0x01, 0x08),
            bc: (0x02, 0x03),
            de: (0x04, 0x05),
            hl: (0x06, 0x07),
            sp: 0x1234,
            pc: 0x5678,
        };
        assert_eq!(cpu.registers.as_16bit(&Register16bit::AF), 0x0108);
        assert_eq!(cpu.registers.as_16bit(&Register16bit::BC), 0x0203);
        assert_eq!(cpu.registers.as_16bit(&Register16bit::DE), 0x0405);
        assert_eq!(cpu.registers.as_16bit(&Register16bit::HL), 0x0607);
        assert_eq!(cpu.registers.as_16bit(&Register16bit::SP), 0x1234);
        assert_eq!(cpu.registers.as_16bit(&Register16bit::PC), 0x5678);
    }
}

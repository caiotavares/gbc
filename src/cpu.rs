use crate::instructions::Instruction;
use crate::memory::Memory;
use crate::*;

const CLOCK: f32 = 8.388608;

enum Register {
    AF,
    BC,
    DE,
    HL,
    SP,
    PC,
}

#[derive(Debug)]
struct Registers {
    af: u16, // Accumulator & Flags register
    bc: u16,
    de: u16,
    hl: u16,
    sp: u16, // Stack Pointer
    pc: u16, // Program Counter
}

impl Registers {
    pub fn init() -> Registers {
        // Initial values for registers obtained from Pandocs
        Registers {
            af: 0x1180,
            bc: 0x0000,
            de: 0xFF56,
            hl: 0x000D,
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
        let current_state = self.af.split().1;
        let new_state: u8 = 0x00;
    }

    pub fn from_enum(&mut self, reg: Register) -> &mut u16 {
        match reg {
            Register::AF => &mut self.af,
            Register::BC => &mut self.bc,
            Register::DE => &mut self.de,
            Register::HL => &mut self.hl,
            Register::SP => &mut self.sp,
            Register::PC => &mut self.pc,
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

struct ALU {}

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

    fn inc_r8_lsb(&mut self, reg: Register) {
        let register = self.registers.from_enum(reg);
        let (lsb, msb) = register.split();
        if lsb == 0xFF {
            *register = as_u16(0x00, msb);
            self.registers
                .set_flags(Some(true), Some(false), Some(true), None);
        } else {
            *register = as_u16(lsb + 1, msb);
            self.registers.set_flags(None, Some(false), None, None);
        };
        self.clock.cycles += 1;
    }

    fn inc_r8_msb(&mut self, reg: Register) {
        let register = self.registers.from_enum(reg);
        let (lsb, msb) = register.split();
        if msb == 0xFF {
            *register = as_u16(lsb, 0x00);
            self.registers
                .set_flags(Some(true), Some(false), Some(true), None);
        } else {
            *register = as_u16(lsb, msb + 1);
            self.registers.set_flags(None, Some(false), None, None);
        };
        self.clock.cycles += 1;
    }

    fn dec_r8_lsb(&mut self, reg: Register) {
        let register = self.registers.from_enum(reg);
        let (lsb, msb) = register.split();
        if lsb == 0x00 {
            *register = as_u16(0xFF, msb);
            self.registers
                .set_flags(Some(true), Some(false), Some(true), None);
        } else {
            *register = as_u16(lsb - 1, msb);
            self.registers.set_flags(None, Some(false), None, None);
        };
        self.clock.cycles += 1;
    }

    fn dec_r8_msb(&mut self, reg: Register) {
        let register = self.registers.from_enum(reg);
        let (lsb, msb) = register.split();
        if msb == 0x00 {
            *register = as_u16(lsb, 0xFF);
            self.registers
                .set_flags(Some(true), Some(false), Some(true), None);
        } else {
            *register = as_u16(lsb, msb - 1);
            self.registers.set_flags(None, Some(false), None, None);
        };
        self.clock.cycles += 1;
    }

    fn load_r8_lsb(&mut self, reg: Register) {
        let data = CPU::fetch(&mut self.registers.pc, &self.memory);
        let register = self.registers.from_enum(reg);
        *register = *register | (data as u16);
        self.clock.cycles += 2;
    }

    fn load_r8_msb(&mut self, reg: Register) {
        let data = CPU::fetch(&mut self.registers.pc, &self.memory);
        let register = self.registers.from_enum(reg);
        *register = *register | ((data as u16) << 8);
        self.clock.cycles += 2;
    }

    fn load_r16(&mut self, reg: Register) {
        let lsb = CPU::fetch(&mut self.registers.pc, &self.memory);
        let msb = CPU::fetch(&mut self.registers.pc, &self.memory);
        let register = self.registers.from_enum(reg);
        *register = as_u16(lsb, msb);
        self.clock.cycles += 3;
    }

    fn fetch(pc: &mut u16, memory: &Memory) -> u8 {
        let data = memory.read(*pc);
        *pc += 1;
        data
    }

    fn execute(&mut self, ins: Instruction) {
        match ins {
            // Control
            Instruction::NOP => self.clock.cycles += 1,
            Instruction::CB => {
                let next_byte = CPU::fetch(&mut self.registers.pc, &self.memory);
                self.execute(Instruction::decode_cb(next_byte));
            }
            // Load 8 bits
            Instruction::LD_A_u8 => self.load_r8_lsb(Register::AF),
            Instruction::LD_B_u8 => self.load_r8_lsb(Register::BC),
            Instruction::LD_C_u8 => self.load_r8_msb(Register::BC),
            Instruction::LD_D_u8 => self.load_r8_lsb(Register::DE),
            Instruction::LD_E_u8 => self.load_r8_msb(Register::DE),
            Instruction::LD_H_u8 => self.load_r8_lsb(Register::HL),
            Instruction::LD_L_u8 => self.load_r8_msb(Register::HL),

            // Load 16 bits
            Instruction::LD_BC_u16 => self.load_r16(Register::BC),
            Instruction::LD_DE_u16 => self.load_r16(Register::DE),
            Instruction::LD_HL_u16 => self.load_r16(Register::HL),

            // ALU 8 bits
            Instruction::INC_A => self.inc_r8_lsb(Register::AF),
            Instruction::INC_B => self.inc_r8_lsb(Register::BC),
            Instruction::INC_C => self.inc_r8_msb(Register::BC),
            Instruction::INC_D => self.inc_r8_lsb(Register::DE),
            Instruction::INC_E => self.inc_r8_msb(Register::DE),
            Instruction::INC_H => self.inc_r8_lsb(Register::HL),
            Instruction::INC_L => self.inc_r8_msb(Register::HL),
            Instruction::DEC_A => self.dec_r8_lsb(Register::AF),
            Instruction::DEC_B => self.dec_r8_lsb(Register::BC),
            Instruction::DEC_C => self.dec_r8_msb(Register::BC),
            Instruction::DEC_D => self.dec_r8_lsb(Register::DE),
            Instruction::DEC_E => self.dec_r8_msb(Register::DE),
            Instruction::DEC_H => self.dec_r8_lsb(Register::HL),
            Instruction::DEC_L => self.dec_r8_msb(Register::HL),

            Instruction::LD_BC_A => {
                self.memory
                    .write(self.registers.bc, self.registers.af.split().0);
                self.clock.cycles += 2;
            }

            Instruction::LD_DE_A => {
                self.memory
                    .write(self.registers.de, self.registers.af.split().0);
                self.clock.cycles += 2;
            }

            Instruction::Invalid => {}

            _ => {}
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
                af: 0x0000,
                bc: 0x0000,
                de: 0x0000,
                hl: 0x0000,
                sp: 0x0000,
                pc: 0x0000,
            },
        }
    }

    #[test]
    fn test_alu_r8() {
        let mut cpu = new_test_cpu();
        assert_eq!(cpu.registers.bc, 0x0000);
        cpu.inc_r8_lsb(Register::BC);
        assert_eq!(cpu.registers.bc, 0x0001);
        cpu.inc_r8_msb(Register::BC);
        assert_eq!(cpu.registers.bc, 0x0101);
        cpu.dec_r8_msb(Register::BC);
        assert_eq!(cpu.registers.bc, 0x0001);
        cpu.dec_r8_lsb(Register::BC);
        assert_eq!(cpu.registers.bc, 0x0000);
    }

    // TODO: Assert flags being set
    #[test]
    fn test_alu_r8_overflow() {
        let mut cpu = new_test_cpu();
        let bc = cpu.registers.from_enum(Register::BC);
        *bc = 0xFFFF;
        assert_eq!(cpu.registers.bc, 0xFFFF);
        cpu.inc_r8_lsb(Register::BC);
        assert_eq!(cpu.registers.bc, 0xFF00);
        cpu.inc_r8_msb(Register::BC);
        assert_eq!(cpu.registers.bc, 0x0000);
        cpu.dec_r8_lsb(Register::BC);
        assert_eq!(cpu.registers.bc, 0x00FF);
        cpu.dec_r8_msb(Register::BC);
        assert_eq!(cpu.registers.bc, 0xFFFF);
    }

    #[test]
    fn test_from_enum() {
        let mut cpu = new_test_cpu();
        cpu.registers = Registers {
            af: 0x0001,
            bc: 0x0002,
            de: 0x0003,
            hl: 0x0004,
            sp: 0x0005,
            pc: 0x0006,
        };
        assert_eq!(*cpu.registers.from_enum(Register::AF), 0x0001);
        assert_eq!(*cpu.registers.from_enum(Register::BC), 0x0002);
        assert_eq!(*cpu.registers.from_enum(Register::DE), 0x0003);
        assert_eq!(*cpu.registers.from_enum(Register::HL), 0x0004);
        assert_eq!(*cpu.registers.from_enum(Register::SP), 0x0005);
        assert_eq!(*cpu.registers.from_enum(Register::PC), 0x0006);
    }
}

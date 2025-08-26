pub mod io;

use crate::memory::io::IO;
use crate::{_32KB, _4KB, _8KB};

#[derive(Debug)]
pub struct Memory {
    /// 0x0000 ~ 0x7FFF Bank 0~NN
    rom: [u8; _32KB],
    /// 0x8000 ~ 0x9FFF
    vram: [u8; _8KB],
    /// 0xA000 ~ 0xBFFF External RAM
    ram: [u8; _32KB],
    /// 0xC000 ~ 0xCFFF WRAM
    wram1: [u8; _4KB],
    /// 0xD000 ~ 0xDFFF WRAM
    wram2: [u8; _4KB],
    // 0xFF00 ~ 0xFF7F I/O Registers
    io: IO,
}

impl Memory {
    pub fn empty() -> Memory {
        Memory {
            rom: [0; _32KB],
            vram: [0; _8KB],
            ram: [0; _32KB],
            wram1: [0; _4KB],
            wram2: [0; _4KB],
            io: IO::init(),
        }
    }

    pub fn new(rom: [u8; _32KB]) -> Memory {
        Memory {
            // TODO: Should we use Vec<u8> instead of [u8; _32KB]?
            rom,
            vram: [0; _8KB],
            ram: [0; _32KB],
            wram1: [0; _4KB],
            wram2: [0; _4KB],
            io: IO::init(),
        }
    }

    pub fn read(&self, address: u16) -> u8 {
        // TODO: Should read access to some regions be protected?
        match address {
            0x0000..=0x7FFF => self.rom[address as usize],
            0x8000..=0x9FFF => self.vram[address as usize - 0x8000],
            0xA000..=0xBFFF => self.ram[address as usize - 0xA000],
            0xC000..=0xCFFF => self.wram1[address as usize - 0xC000],
            0xD000..=0xDFFF => self.wram2[address as usize - 0xD000],
            0xFF00..=0xFF7F => self.io.read(address as usize),
            _ => panic!("Forbidden read from memory address: 0x{:04X}", address),
        }
    }

    pub fn write(&mut self, address: u16, data: u8) {
        // TODO: Write access to some regions should be protected
        match address {
            // ROM data
            0x0000..=0x7FFF => panic!("Forbidden write into ROM memory region!"),
            _ => self.ram[address as usize] = data,
        }
    }
}

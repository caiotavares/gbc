pub mod cpu;
pub mod instruction;
pub mod memory;
pub mod ppu;
pub mod sound;

pub const _128KB: usize = 128 * _1KB;
pub const _64KB: usize = 64 * _1KB;
pub const _32KB: usize = 32 * _1KB;
pub const _16KB: usize = 16 * _1KB;
pub const _8KB: usize = 8 * _1KB;
pub const _4KB: usize = 4 * _1KB;
pub const _2KB: usize = 2 * _1KB;
pub const _1KB: usize = 1024;

pub fn as_u16(msb: u8, lsb: u8) -> u16 {
    let value: u16 = 0x0000;
    (value | (lsb as u16)) | ((msb as u16) << 8)
}

pub trait Nibble {
    fn low_nibble(&self) -> u8;
    fn high_nibble(&self) -> u8;
}

pub trait Joinable {
    fn join(&self) -> u16;
}

pub trait Splitable {
    fn split(&self) -> (u8, u8);
}

impl Nibble for u8 {
    fn low_nibble(&self) -> u8 {
        self & 0xF
    }

    fn high_nibble(&self) -> u8 {
        self & 0xF0
    }
}

impl Splitable for u16 {
    fn split(&self) -> (u8, u8) {
        let msb: u8 = (self >> 8) as u8;
        let lsb: u8 = *self as u8;
        (lsb, msb)
    }
}

impl Joinable for (u8, u8) {
    fn join(&self) -> u16 {
        as_u16(self.0, self.1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_as_u16() {
        let lsb: u8 = 0x1F;
        let msb: u8 = 0x25;
        let word: u16 = as_u16(msb, lsb);
        assert_eq!(word, 0x251F);
    }

    #[test]
    fn test_split_u16() {
        let lsb: u8 = 0x1F;
        let msb: u8 = 0x25;
        let word: u16 = as_u16(msb, lsb);
        assert_eq!((0x1F, 0x25), word.split());
    }
}

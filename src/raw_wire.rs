//! Ojo UART v1. All multibyte fields little-endian, COBS + 0 delimiter.
use anyhow::{Result, ensure};
pub const HELLO: u8 = 1;
pub const ARM: u8 = 2;
pub const STATE: u8 = 3;
pub const CENTER: u8 = 4;
pub const BLINK: u8 = 5;
pub const LIMITS: u8 = 6;
pub const TARGET: u8 = 0x10;
pub const CONFIG: u8 = 0x81;
pub const TELEMETRY: u8 = 0x82;
pub const ACK: u8 = 0x83;
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet {
    pub kind: u8,
    pub session: u32,
    pub seq: u16,
    pub time: u32,
    pub data: Vec<u8>,
}
pub fn crc(bytes: &[u8]) -> u16 {
    let mut c = 0xffffu16;
    for &b in bytes {
        c ^= (b as u16) << 8;
        for _ in 0..8 {
            c = if c & 0x8000 != 0 {
                (c << 1) ^ 0x1021
            } else {
                c << 1
            };
        }
    }
    c
}
pub fn encode(p: &Packet) -> Result<Vec<u8>> {
    ensure!(p.data.len() <= 48, "Trama demasiado larga");
    let mut b = vec![1, p.kind, p.data.len() as u8];
    b.extend(p.session.to_le_bytes());
    b.extend(p.seq.to_le_bytes());
    b.extend(p.time.to_le_bytes());
    b.extend(&p.data);
    b.extend(crc(&b).to_le_bytes());
    let mut out = vec![0];
    let mut pos = 0;
    let mut code = 1u8;
    for byte in b {
        if byte == 0 {
            out[pos] = code;
            pos = out.len();
            out.push(0);
            code = 1;
        } else {
            out.push(byte);
            code += 1;
        }
    }
    out[pos] = code;
    out.push(0);
    Ok(out)
}
pub fn decode(encoded: &[u8]) -> Result<Packet> {
    ensure!(
        !encoded.is_empty() && encoded.len() <= 64,
        "Longitud COBS inválida"
    );
    let mut b = Vec::with_capacity(63);
    let mut i = 0;
    while i < encoded.len() {
        let c = encoded[i] as usize;
        i += 1;
        ensure!(c > 0 && i + c - 1 <= encoded.len(), "COBS inválido");
        b.extend_from_slice(&encoded[i..i + c - 1]);
        i += c - 1;
        if c != 255 && i < encoded.len() {
            b.push(0);
        }
    }
    ensure!(
        b.len() >= 15 && b[0] == 1 && b[2] <= 48 && b.len() == 15 + b[2] as usize,
        "Cabecera inválida"
    );
    ensure!(
        u16at(&b, b.len() - 2) == crc(&b[..b.len() - 2]),
        "CRC inválido"
    );
    Ok(Packet {
        kind: b[1],
        session: u32at(&b, 3),
        seq: u16at(&b, 7),
        time: u32at(&b, 9),
        data: b[13..b.len() - 2].to_vec(),
    })
}
pub fn u16at(b: &[u8], i: usize) -> u16 {
    u16::from_le_bytes(b[i..i + 2].try_into().unwrap())
}
pub fn u32at(b: &[u8], i: usize) -> u32 {
    u32::from_le_bytes(b[i..i + 4].try_into().unwrap())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn crc_check_and_all_payload_lengths() {
        assert_eq!(crc(b"123456789"), 0x29b1);
        for n in 0..=48 {
            let p = Packet {
                kind: 16,
                session: 0x12345678,
                seq: 65535,
                time: u32::MAX,
                data: (0..n).map(|i| i * 5).collect(),
            };
            let e = encode(&p).unwrap();
            assert_eq!(e.len(), n as usize + 17);
            assert_eq!(decode(&e[..e.len() - 1]).unwrap(), p);
        }
    }
    #[test]
    fn corruption_is_detected() {
        let p = Packet {
            kind: 16,
            session: 9,
            seq: 0,
            time: 20,
            data: vec![0; 7],
        };
        let e = encode(&p).unwrap();
        for i in 0..e.len() - 1 {
            for bit in 0..8 {
                let mut x = e.clone();
                x[i] ^= 1 << bit;
                assert!(decode(&x[..x.len() - 1]).is_err());
            }
        }
    }
}

#[cfg(test)]
mod golden_test {
    use super::*;
    #[test]
    fn matches_avr_golden_packet() {
        let mut data = Vec::new();
        for v in [-1234i16, 567, 3200] {
            data.extend(v.to_le_bytes());
        }
        data.push(1);
        let p = Packet {
            kind: TARGET,
            session: 0x12345678,
            seq: 42,
            time: 1000,
            data,
        };
        let expected = [
            0x09, 0x01, 0x10, 0x07, 0x78, 0x56, 0x34, 0x12, 0x2a, 0x03, 0xe8, 0x03, 0x01, 0x0a,
            0x2e, 0xfb, 0x37, 0x02, 0x80, 0x0c, 0x01, 0x80, 0xf7, 0x00,
        ];
        assert_eq!(encode(&p).unwrap(), expected);
        assert_eq!(decode(&expected[..23]).unwrap(), p);
    }
}

//! Pure A/B (two-slot) flash KV engine, independent of any flash backend.
//!
//! The board-local flash store wires its ROM erase/program routines around the
//! pure functions here. Keeping serialization, CRC, and slot-selection logic
//! here makes the torn-write and CRC-failure semantics host-testable without a
//! flash device.
//!
//! Layout: two equal-sized slots. Each slot begins with a fixed header
//! (`SLOT_HEADER_LEN`) followed by the fuel/ign/angles payload. The header
//! carries a magic, version, a monotonic `seq`, per-key lengths, and per-key
//! CRC16-CCITT. Boot selects the slot with a valid header, all CRCs matching,
//! and the highest `seq`.

use super::layout::{ANGLES_PAGE_LEN, FUEL_PAGE_LEN, IGN_PAGE_LEN};

pub const MAGIC: u32 = 0x4950_574B; // 'IPWK'
pub const VERSION: u16 = 3;

pub const LEN_FUEL: usize = FUEL_PAGE_LEN;
pub const LEN_IGN: usize = IGN_PAGE_LEN;
pub const LEN_ANGLES: usize = ANGLES_PAGE_LEN;

pub const SLOT_HEADER_LEN: usize = 24;
pub const PAYLOAD_LEN: usize = LEN_FUEL + LEN_IGN + LEN_ANGLES;
pub const SLOT_USED_LEN: usize = SLOT_HEADER_LEN + PAYLOAD_LEN;

const OFF_MAGIC: usize = 0;
const OFF_VERSION: usize = 4;
const OFF_RSV: usize = 6;
const OFF_SEQ: usize = 8;
const OFF_FUEL_LEN: usize = 12;
const OFF_IGN_LEN: usize = 14;
const OFF_ANGLES_LEN: usize = 16;
const OFF_FUEL_CRC: usize = 18;
const OFF_IGN_CRC: usize = 20;
const OFF_ANGLES_CRC: usize = 22;

const OFF_FUEL: usize = SLOT_HEADER_LEN;
const OFF_IGN: usize = OFF_FUEL + LEN_FUEL;
const OFF_ANGLES: usize = OFF_IGN + LEN_IGN;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct SlotContents {
    pub seq: u32,
    pub fuel: [u8; LEN_FUEL],
    pub ign: [u8; LEN_IGN],
    pub angles: [u8; LEN_ANGLES],
}

impl SlotContents {
    pub const fn zeroed() -> Self {
        Self {
            seq: 0,
            fuel: [0u8; LEN_FUEL],
            ign: [0u8; LEN_IGN],
            angles: [0u8; LEN_ANGLES],
        }
    }
}

/// Outcome of scanning both slots at boot.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum BootScan {
    /// No slot carries our magic: a blank (first boot) device. Only an
    /// unrecognized-magic slot is Blank; a recognized magic with the wrong
    /// version is treated as Corrupt (surfaced to the tuner), not Blank.
    Blank,
    /// At least one slot carried our magic but every such slot failed CRC,
    /// header, or version validation. A recognized-magic/wrong-version slot
    /// lands here so a stale persisted tune is surfaced to the tuner rather
    /// than silently discarded. Distinguishes corruption from a blank device.
    Corrupt,
    /// A valid slot was selected. `slot` is 0 (A) or 1 (B); `seq` is its
    /// sequence counter. `saw_corrupt` is true when the *other* slot carried
    /// our magic but failed validation (e.g. a torn write rolled back to the
    /// older committed slot): the selected tune is good, but the tuner's last
    /// write did not stick and a diagnostic should be surfaced.
    Valid {
        slot: u8,
        seq: u32,
        saw_corrupt: bool,
    },
}

pub fn crc16_ccitt(mut crc: u16, data: &[u8]) -> u16 {
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

fn rd_u16(buf: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([buf[off], buf[off + 1]])
}

fn rd_u32(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

/// Serialize a full slot (header + payload) into `out`, returning the number of
/// bytes written. `out` must be at least `SLOT_USED_LEN`. Bytes beyond the
/// written region are left untouched (the caller fills the erase pattern).
pub fn serialize_slot(c: &SlotContents, out: &mut [u8]) -> usize {
    if out.len() < SLOT_USED_LEN {
        return 0;
    }
    let fuel_crc = crc16_ccitt(0xFFFF, &c.fuel);
    let ign_crc = crc16_ccitt(0xFFFF, &c.ign);
    let angles_crc = crc16_ccitt(0xFFFF, &c.angles);

    out[OFF_MAGIC..OFF_MAGIC + 4].copy_from_slice(&MAGIC.to_le_bytes());
    out[OFF_VERSION..OFF_VERSION + 2].copy_from_slice(&VERSION.to_le_bytes());
    out[OFF_RSV..OFF_RSV + 2].copy_from_slice(&0u16.to_le_bytes());
    out[OFF_SEQ..OFF_SEQ + 4].copy_from_slice(&c.seq.to_le_bytes());
    out[OFF_FUEL_LEN..OFF_FUEL_LEN + 2].copy_from_slice(&(LEN_FUEL as u16).to_le_bytes());
    out[OFF_IGN_LEN..OFF_IGN_LEN + 2].copy_from_slice(&(LEN_IGN as u16).to_le_bytes());
    out[OFF_ANGLES_LEN..OFF_ANGLES_LEN + 2].copy_from_slice(&(LEN_ANGLES as u16).to_le_bytes());
    out[OFF_FUEL_CRC..OFF_FUEL_CRC + 2].copy_from_slice(&fuel_crc.to_le_bytes());
    out[OFF_IGN_CRC..OFF_IGN_CRC + 2].copy_from_slice(&ign_crc.to_le_bytes());
    out[OFF_ANGLES_CRC..OFF_ANGLES_CRC + 2].copy_from_slice(&angles_crc.to_le_bytes());

    out[OFF_FUEL..OFF_FUEL + LEN_FUEL].copy_from_slice(&c.fuel);
    out[OFF_IGN..OFF_IGN + LEN_IGN].copy_from_slice(&c.ign);
    out[OFF_ANGLES..OFF_ANGLES + LEN_ANGLES].copy_from_slice(&c.angles);
    SLOT_USED_LEN
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum SlotState {
    Blank,
    Corrupt,
    Valid(u32),
}

/// Classify a single slot. An unrecognized magic is `Blank` (genuinely
/// erased, first boot, no fault). A recognized magic with a wrong version is
/// `Corrupt` so the stale tune is surfaced to the tuner rather than silently
/// dropped as a blank device.
fn classify_slot(slot: &[u8]) -> SlotState {
    if slot.len() < SLOT_USED_LEN {
        return SlotState::Corrupt;
    }
    if rd_u32(slot, OFF_MAGIC) != MAGIC {
        return SlotState::Blank;
    }
    if rd_u16(slot, OFF_VERSION) != VERSION {
        return SlotState::Corrupt;
    }
    if rd_u16(slot, OFF_FUEL_LEN) as usize != LEN_FUEL
        || rd_u16(slot, OFF_IGN_LEN) as usize != LEN_IGN
        || rd_u16(slot, OFF_ANGLES_LEN) as usize != LEN_ANGLES
    {
        return SlotState::Corrupt;
    }
    let fuel_crc = crc16_ccitt(0xFFFF, &slot[OFF_FUEL..OFF_FUEL + LEN_FUEL]);
    let ign_crc = crc16_ccitt(0xFFFF, &slot[OFF_IGN..OFF_IGN + LEN_IGN]);
    let angles_crc = crc16_ccitt(0xFFFF, &slot[OFF_ANGLES..OFF_ANGLES + LEN_ANGLES]);
    if fuel_crc != rd_u16(slot, OFF_FUEL_CRC)
        || ign_crc != rd_u16(slot, OFF_IGN_CRC)
        || angles_crc != rd_u16(slot, OFF_ANGLES_CRC)
    {
        return SlotState::Corrupt;
    }
    SlotState::Valid(rd_u32(slot, OFF_SEQ))
}

/// Scan both slots and report the boot decision.
pub fn scan(slot_a: &[u8], slot_b: &[u8]) -> BootScan {
    let states = [classify_slot(slot_a), classify_slot(slot_b)];
    let mut best: Option<(u8, u32)> = None;
    let mut saw_corrupt = false;
    let mut saw_magic = false;
    for (idx, state) in states.iter().enumerate() {
        match state {
            SlotState::Blank => {}
            SlotState::Corrupt => {
                saw_corrupt = true;
                saw_magic = true;
            }
            SlotState::Valid(seq) => {
                saw_magic = true;
                let take = match best {
                    None => true,
                    // equal seq is not expected in normal operation; prefer the later-scanned slot deterministically
                    Some((_, best_seq)) => *seq >= best_seq,
                };
                if take {
                    best = Some((idx as u8, *seq));
                }
            }
        }
    }
    match best {
        Some((slot, seq)) => BootScan::Valid {
            slot,
            seq,
            saw_corrupt,
        },
        None if saw_corrupt || saw_magic => BootScan::Corrupt,
        None => BootScan::Blank,
    }
}

/// Read a key's bytes out of a validated slot. Returns the byte count on
/// success, or `None` if the key/length is unknown or the slot is too short.
pub fn read_key(slot: &[u8], key: &[u8], out: &mut [u8]) -> Option<usize> {
    if slot.len() < SLOT_USED_LEN {
        return None;
    }
    let (off, len) = match key {
        b"fuel" => (OFF_FUEL, LEN_FUEL),
        b"ign" => (OFF_IGN, LEN_IGN),
        b"angles" => (OFF_ANGLES, LEN_ANGLES),
        _ => return None,
    };
    if out.len() < len {
        return None;
    }
    out[..len].copy_from_slice(&slot[off..off + len]);
    Some(len)
}

/// Which physical slot a write targets given the currently-best slot.
/// Writes always go to the stale slot so a torn write cannot damage the
/// committed copy. With no valid slot, slot A (0) is chosen.
pub const fn write_target(current: BootScan) -> u8 {
    match current {
        BootScan::Valid { slot, .. } => 1 - slot,
        _ => 0,
    }
}

/// Sequence number to stamp on the next write.
pub const fn next_seq(current: BootScan) -> u32 {
    match current {
        BootScan::Valid { seq, .. } => seq.wrapping_add(1),
        _ => 1,
    }
}

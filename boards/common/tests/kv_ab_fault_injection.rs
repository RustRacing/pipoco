//! Host fault-injection tests for the two-slot (A/B) flash KV engine.
//!
//! A simulated flash device backs two slots and can truncate a write at any
//! byte offset, modelling a power loss mid-program. The property under test:
//! after a torn write to the stale slot, boot must observe either the old
//! committed state or the new committed state, never a torn/blended one.

use ecu_target_common::kv::ab::{
    self, BootScan, SlotContents, LEN_ANGLES, LEN_FUEL, LEN_IGN, SLOT_USED_LEN,
};

const SECTOR: usize = 4096;

/// Two-slot flash with byte-granular, truncatable writes.
struct SimFlash {
    slots: [[u8; SECTOR]; 2],
}

impl SimFlash {
    fn blank() -> Self {
        Self {
            slots: [[0xFFu8; SECTOR]; 2],
        }
    }

    fn scan(&self) -> BootScan {
        ab::scan(&self.slots[0], &self.slots[1])
    }

    fn read_contents(&self, slot: u8) -> SlotContents {
        let buf = &self.slots[slot as usize];
        let mut c = SlotContents::zeroed();
        ab::read_key(buf, b"fuel", &mut c.fuel).unwrap();
        ab::read_key(buf, b"ign", &mut c.ign).unwrap();
        ab::read_key(buf, b"angles", &mut c.angles).unwrap();
        c
    }

    /// Erase + program a slot from `contents`, programming only the first
    /// `truncate_at` bytes (the rest stays erased, i.e. 0xFF). `truncate_at >=
    /// SLOT_USED_LEN` is a complete (committed) write.
    fn write_truncated(&mut self, slot: u8, contents: &SlotContents, truncate_at: usize) {
        let mut serialized = [0xFFu8; SECTOR];
        ab::serialize_slot(contents, &mut serialized);
        let sector = &mut self.slots[slot as usize];
        sector.fill(0xFF);
        let n = truncate_at.min(SECTOR);
        sector[..n].copy_from_slice(&serialized[..n]);
    }

    /// Full, committed A/B write to the stale slot with `seq + 1`.
    fn commit(&mut self, contents: &SlotContents) {
        let current = self.scan();
        let target = ab::write_target(current);
        let mut c = *contents;
        c.seq = ab::next_seq(current);
        self.write_truncated(target, &c, SLOT_USED_LEN);
    }
}

fn distinct_contents(tag: u8) -> SlotContents {
    let mut c = SlotContents::zeroed();
    c.fuel = [tag; LEN_FUEL];
    c.ign = [tag.wrapping_add(0x10); LEN_IGN];
    c.angles = [tag.wrapping_add(0x20); LEN_ANGLES];
    c
}

#[test]
fn blank_device_boots_blank() {
    let flash = SimFlash::blank();
    assert_eq!(flash.scan(), BootScan::Blank);
}

#[test]
fn first_commit_boots_valid_slot_a() {
    let mut flash = SimFlash::blank();
    let v1 = distinct_contents(0x11);
    flash.commit(&v1);
    match flash.scan() {
        BootScan::Valid {
            slot,
            seq,
            saw_corrupt,
        } => {
            assert_eq!(slot, 0);
            assert_eq!(seq, 1);
            assert!(
                !saw_corrupt,
                "clean first commit must not see a corrupt sibling"
            );
            assert_eq!(flash.read_contents(0).fuel, v1.fuel);
        }
        other => panic!("expected valid slot A, got {other:?}"),
    }
}

#[test]
fn second_commit_targets_stale_slot_b_and_wins_by_seq() {
    let mut flash = SimFlash::blank();
    let v1 = distinct_contents(0x11);
    let v2 = distinct_contents(0x22);
    flash.commit(&v1);
    flash.commit(&v2);
    match flash.scan() {
        BootScan::Valid {
            slot,
            seq,
            saw_corrupt,
        } => {
            assert_eq!(slot, 1, "second write must land in stale slot B");
            assert_eq!(seq, 2);
            assert!(
                !saw_corrupt,
                "two clean commits must not see a corrupt sibling"
            );
            assert_eq!(flash.read_contents(1).fuel, v2.fuel);
        }
        other => panic!("expected valid slot B, got {other:?}"),
    }
}

/// Core property: for every truncation offset of the second write, boot yields
/// either the old (v1) or the new (v2) committed state, never a torn one.
#[test]
fn torn_second_write_never_yields_torn_state() {
    let v1 = distinct_contents(0x11);
    let v2 = distinct_contents(0x22);

    for truncate_at in 0..=SLOT_USED_LEN {
        let mut flash = SimFlash::blank();
        flash.commit(&v1);

        // Begin writing v2 into the stale slot, but cut power after
        // `truncate_at` bytes.
        let current = flash.scan();
        let target = ab::write_target(current);
        assert_eq!(target, 1, "stale slot must be B after first commit");
        let mut staged = v2;
        staged.seq = ab::next_seq(current);
        flash.write_truncated(target, &staged, truncate_at);

        match flash.scan() {
            BootScan::Valid {
                slot, saw_corrupt, ..
            } => {
                let got = flash.read_contents(slot).fuel;
                assert!(
                    got == v1.fuel || got == v2.fuel,
                    "offset {truncate_at}: torn fuel state {:?}",
                    &got[..4]
                );
                // Slot A (v1) is always intact, so a torn B must fall back to A.
                if slot == 1 {
                    assert_eq!(got, v2.fuel, "offset {truncate_at}: B selected but not v2");
                    // A complete, untouched B: no corrupt sibling.
                    assert!(
                        !saw_corrupt,
                        "offset {truncate_at}: intact B must not flag corruption"
                    );
                } else {
                    // Fell back to A. Once B has its magic but is incomplete it
                    // scans Corrupt, so the rollback must be signalled; while the
                    // magic is not yet programmed B reads Blank and no signal is
                    // expected.
                    let b_has_magic = truncate_at >= 4 && truncate_at < SLOT_USED_LEN;
                    assert_eq!(
                        saw_corrupt, b_has_magic,
                        "offset {truncate_at}: corrupt-sibling flag must match a torn B carrying our magic"
                    );
                }
            }
            BootScan::Corrupt => {
                // Acceptable only if the intact A slot somehow failed; assert A
                // is in fact still valid to catch a real regression.
                panic!("offset {truncate_at}: boot reported Corrupt despite intact slot A");
            }
            BootScan::Blank => panic!("offset {truncate_at}: boot reported Blank"),
        }
    }
}

/// When the only slot present is a torn write (no prior committed copy), boot
/// must distinguish corruption from blank so the policy can latch a fault.
#[test]
fn torn_first_write_is_blank_until_header_then_corrupt() {
    let v1 = distinct_contents(0x33);
    for truncate_at in 0..=SLOT_USED_LEN {
        let mut flash = SimFlash::blank();
        let mut staged = v1;
        staged.seq = 1;
        flash.write_truncated(0, &staged, truncate_at);

        match flash.scan() {
            // Before the magic word is fully programmed the slot reads blank;
            // once the magic is present but the slot is incomplete it must read
            // corrupt; once complete it is valid.
            BootScan::Valid { .. } => assert_eq!(truncate_at, SLOT_USED_LEN),
            BootScan::Corrupt => assert!(truncate_at >= 4 && truncate_at < SLOT_USED_LEN),
            BootScan::Blank => assert!(truncate_at < 4),
        }
    }
}

#[test]
fn corrupt_committed_slot_falls_back_to_other_valid_slot() {
    let mut flash = SimFlash::blank();
    let v1 = distinct_contents(0x44);
    let v2 = distinct_contents(0x55);
    flash.commit(&v1); // slot A, seq 1
    flash.commit(&v2); // slot B, seq 2 (current best)

    // Corrupt slot B's payload after commit (single-bit flip in fuel data).
    flash.slots[1][SLOT_USED_LEN / 2] ^= 0xFF;

    match flash.scan() {
        BootScan::Valid {
            slot,
            seq,
            saw_corrupt,
        } => {
            assert_eq!(slot, 0, "must fall back to intact slot A");
            assert_eq!(seq, 1);
            assert!(
                saw_corrupt,
                "a corrupt committed sibling must be flagged so the rollback is surfaced"
            );
            assert_eq!(flash.read_contents(0).fuel, v1.fuel);
        }
        other => panic!("expected fallback to slot A, got {other:?}"),
    }
}

/// A slot with valid magic and correct CRCs but a stale version must scan as
/// Corrupt (surfaced to the tuner), not Blank, so the persisted tune is not
/// silently discarded across a KV version bump. A truly erased pair stays Blank.
#[test]
fn stale_version_slot_is_corrupt_not_blank() {
    let v1 = distinct_contents(0x66);
    let mut serialized = [0xFFu8; SECTOR];
    let n = ab::serialize_slot(&v1, &mut serialized);
    assert_eq!(n, SLOT_USED_LEN);

    // Downgrade the version word to VERSION - 1, keeping magic and CRCs valid.
    let stale = ab::VERSION - 1;
    serialized[4..6].copy_from_slice(&stale.to_le_bytes());

    let mut flash = SimFlash::blank();
    flash.slots[0][..SLOT_USED_LEN].copy_from_slice(&serialized[..SLOT_USED_LEN]);
    assert_eq!(flash.scan(), BootScan::Corrupt);

    let blank = SimFlash::blank();
    assert_eq!(blank.scan(), BootScan::Blank);
}

/// `serialize_slot` must not panic on a too-short buffer: it returns 0 and
/// leaves the bytes untouched. The happy path returns SLOT_USED_LEN.
#[test]
fn serialize_slot_guards_short_buffer() {
    let c = distinct_contents(0x77);

    let mut short = [0xAAu8; SLOT_USED_LEN - 1];
    assert_eq!(ab::serialize_slot(&c, &mut short), 0);
    assert!(
        short.iter().all(|&b| b == 0xAA),
        "short buffer must be untouched"
    );

    let mut full = [0u8; SLOT_USED_LEN];
    assert_eq!(ab::serialize_slot(&c, &mut full), SLOT_USED_LEN);
}

#[test]
fn key_lengths_match_layout_constants() {
    assert_eq!(LEN_FUEL, 512);
    assert_eq!(LEN_IGN, 512);
    assert_eq!(LEN_ANGLES, 68);
    assert!(SLOT_USED_LEN <= SECTOR);
}

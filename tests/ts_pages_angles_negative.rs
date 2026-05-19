use ecu_core::ts::PageStore;
use ecu_core::EcuState;

#[test]
fn angles_page_wrong_size_is_error() {
    let mut state = EcuState::new();
    let mut pages = state.page_store();
    // Wrong size (short)
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_ANGLES, &[0u8; 10])
        .is_err());
}

#[test]
fn angles_page_range_validation() {
    let mut state = EcuState::new();
    let mut pages = state.page_store();
    // Build a valid base payload
    let mut ang = [0u8; 68];
    for i in 0..16u16 {
        let off = (i as usize) * 2;
        ang[off..off + 2].copy_from_slice(&(i.min(3600)).to_le_bytes());
    }
    for i in 0..16u16 {
        let off = 32 + (i as usize) * 2;
        ang[off..off + 2].copy_from_slice(&(i.min(7200)).to_le_bytes());
    }
    ang[64..66].copy_from_slice(&100u16.to_le_bytes());
    ang[66..68].copy_from_slice(&500u16.to_le_bytes());
    // First, valid write works
    pages
        .write_page(ecu_core::ts::pages::PAGE_ANGLES, &ang)
        .expect("valid angles write");

    // Now set an invalid inj angle > 3600
    let mut bad = ang;
    bad[0..2].copy_from_slice(&(3601u16).to_le_bytes());
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_ANGLES, &bad)
        .is_err());

    // Invalid tdc angle > 7200
    let mut bad2 = ang;
    bad2[32..34].copy_from_slice(&(7201u16).to_le_bytes());
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_ANGLES, &bad2)
        .is_err());

    // Invalid tooth0 angle > 3600
    let mut bad3 = ang;
    bad3[64..66].copy_from_slice(&(3605u16).to_le_bytes());
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_ANGLES, &bad3)
        .is_err());
}

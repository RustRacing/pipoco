use ecu_core::EcuState;
use ecu_ts::server::PageStore;

#[test]
fn idle_fan_cl_pages_validate_ranges_and_sizes() {
    let mut state = EcuState::new();
    let mut pages = state.page_store();

    // Idle: wrong size and invalid fields
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_IDLE, &[0u8; 3])
        .is_err());
    let mut idle = [0u8; 6];
    idle[0] = 1;
    idle[1..3].copy_from_slice(&1500u16.to_le_bytes()); // 150.0% -> invalid
    idle[3..5].copy_from_slice(&50u16.to_le_bytes());
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_IDLE, &idle)
        .is_err());
    let mut idle_freq0 = [0u8; 6];
    idle_freq0[0] = 1;
    idle_freq0[1..3].copy_from_slice(&500u16.to_le_bytes());
    // freq = 0 invalid
    idle_freq0[3..5].copy_from_slice(&0u16.to_le_bytes());
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_IDLE, &idle_freq0)
        .is_err());

    // Fan: on <= off rejected
    let mut fan = [0u8; 6];
    fan[0] = 1;
    fan[1..3].copy_from_slice(&80i16.to_le_bytes());
    fan[3..5].copy_from_slice(&85i16.to_le_bytes());
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_FAN, &fan)
        .is_err());

    // CL: size and target bounds
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_CL, &[0u8; 4])
        .is_err());
    let mut cl = [0u8; 8];
    cl[0] = 1;
    cl[2..4].copy_from_slice(&90u16.to_le_bytes()); // target below 1.00 lambda (100)
    cl[4..6].copy_from_slice(&10u16.to_le_bytes());
    cl[6..8].copy_from_slice(&0u16.to_le_bytes());
    assert!(pages.write_page(ecu_core::ts::pages::PAGE_CL, &cl).is_err());
    // Above upper bound
    cl[2..4].copy_from_slice(&300u16.to_le_bytes());
    assert!(pages.write_page(ecu_core::ts::pages::PAGE_CL, &cl).is_err());
}

use ecu_core::ts::pages::{
    PAGE_ANGLES, PAGE_ASE, PAGE_CL, PAGE_DIAG, PAGE_DIAG_LOG, PAGE_EXPERT_TRIGGER, PAGE_FAN,
    PAGE_IDLE, PAGE_LIMITS, PAGE_SNAPSHOT, PAGE_WUE,
};
use ecu_core::ts::PageStore;
use ecu_core::EcuState;
use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;

#[test]
fn ini_page_sizes_match_firmware() {
    let ini_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/assets/IPW-ECU.ini");
    let ini = fs::read_to_string(&ini_path).expect("load ini");
    let mut sections: HashMap<String, HashMap<String, String>> = HashMap::new();
    let mut current = String::new();
    for line in ini.lines() {
        let l = line.trim();
        if l.starts_with('[') && l.ends_with(']') {
            current = l.trim_matches(&['[', ']'][..]).to_string();
        } else if let Some((key, value)) = l.split_once('=') {
            if !current.is_empty() {
                sections.entry(current.clone()).or_default().insert(
                    key.trim().to_string(),
                    value.trim().trim_matches('"').to_string(),
                );
            }
        }
    }

    let mut state = EcuState::new();
    let store = state.page_store();
    let cases = [
        ("Diag", PAGE_DIAG),
        ("DiagLog", PAGE_DIAG_LOG),
        ("Limits", PAGE_LIMITS),
        ("Angles", PAGE_ANGLES),
        ("Snapshot", PAGE_SNAPSHOT),
        ("WUE", PAGE_WUE),
        ("ASE", PAGE_ASE),
        ("Idle", PAGE_IDLE),
        ("Fan", PAGE_FAN),
        ("CL", PAGE_CL),
        ("ExpertTrigger", PAGE_EXPERT_TRIGGER),
    ];
    for (section, page) in cases {
        let size = sections
            .get(section)
            .and_then(|fields| fields.get("size"))
            .and_then(|value| value.parse::<usize>().ok());
        assert_eq!(size, store.page_len(page));
    }

    let diag = sections.get("Diag").expect("diag section");
    assert_eq!(
        diag.get("page").and_then(|value| value.parse::<u8>().ok()),
        Some(PAGE_DIAG)
    );
    assert_eq!(
        diag.get("writable").map(|value| value.as_str()),
        Some("false")
    );
    let expected_diag_offsets = [
        ("currentToothCount_offset", 0usize),
        ("camSeen_offset", 1),
        ("syncState_offset", 2),
        ("phaseState_offset", 3),
        ("absoluteAuthority_offset", 4),
        ("triggerAngleSource_offset", 5),
        ("outputGatingReason_offset", 6),
        ("lastSyncLossReason_offset", 7),
        ("primaryRpm_offset", 8),
        ("detectedGapRatio_offset", 10),
        ("syncLossCounter_offset", 12),
        ("m50PinMapIdentity_offset", 14),
        ("m50ProfileIdentity_offset", 16),
        ("m50ProfileHash_offset", 20),
    ];
    for (key, expected) in expected_diag_offsets {
        assert_eq!(
            diag.get(key).and_then(|value| value.parse::<usize>().ok()),
            Some(expected),
            "diag key {key}"
        );
    }

    let expert = sections
        .get("ExpertTrigger")
        .expect("expert trigger section");
    assert_eq!(
        expert
            .get("page")
            .and_then(|value| value.parse::<u8>().ok()),
        Some(PAGE_EXPERT_TRIGGER)
    );
    assert_eq!(
        expert.get("writable").map(|value| value.as_str()),
        Some("true")
    );
    let expected_expert_offsets = [
        ("schemaVersion_offset", 0usize),
        ("expertUnlock_offset", 2),
        ("authority_offset", 3),
        ("profileIdentity_offset", 4),
        ("profileHash_offset", 8),
        ("triggerPattern_offset", 12),
        ("primaryBaseTeeth_offset", 13),
        ("missingTeeth_offset", 14),
        ("primaryTriggerSpeed_offset", 15),
        ("triggerAngleAtdc_offset", 16),
        ("triggerAngleMultiplier_offset", 18),
        ("primaryTriggerEdge_offset", 19),
        ("secondaryTriggerEdge_offset", 20),
        ("secondaryTriggerMode_offset", 21),
        ("pollLevelPolarity_offset", 22),
        ("triggerFilter_offset", 23),
        ("resyncEveryCycle_offset", 24),
        ("skipRevolutions_offset", 25),
        ("ignitionMode_offset", 26),
        ("injectionLayout_offset", 27),
        ("fixedTimingMode_offset", 28),
        ("fixedTimingDeg10_offset", 30),
    ];
    for (key, expected) in expected_expert_offsets {
        assert_eq!(
            expert
                .get(key)
                .and_then(|value| value.parse::<usize>().ok()),
            Some(expected),
            "expert key {key}"
        );
    }
}

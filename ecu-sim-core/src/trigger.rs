use crate::{config::TriggerConfig, types::*};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriggerChannel {
    Crank,
    Cam,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EdgePolarity {
    Rising,
    Falling,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TriggerEdge {
    pub timestamp_us: Micros,
    pub channel: TriggerChannel,
    pub edge: EdgePolarity,
    pub crank_angle_deg10: CrankDeg10,
}

impl TriggerEdge {
    pub const fn empty() -> Self {
        Self {
            timestamp_us: Micros(0),
            channel: TriggerChannel::Crank,
            edge: EdgePolarity::Rising,
            crank_angle_deg10: CrankDeg10(0),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaultInput {
    pub drop_next_crank_edges: u8,
    pub duplicate_next_crank_edges: u8,
    pub drop_next_cam_edges: u8,
    pub duplicate_next_cam_edges: u8,
    pub delay_next_edge_us: Micros,
    pub jitter_next_edge_us: i32,
    pub phase_offset_deg10: Degrees10,
    pub dropout_enabled: bool,
    pub dropout_start_deg10: CrankDeg10,
    pub dropout_end_deg10: CrankDeg10,
    pub invert_polarity: bool,
    pub suppress_cam: bool,
    pub suppress_crank: bool,
}

impl FaultInput {
    pub const fn none() -> Self {
        Self {
            drop_next_crank_edges: 0,
            duplicate_next_crank_edges: 0,
            drop_next_cam_edges: 0,
            duplicate_next_cam_edges: 0,
            delay_next_edge_us: Micros(0),
            jitter_next_edge_us: 0,
            phase_offset_deg10: Degrees10(0),
            dropout_enabled: false,
            dropout_start_deg10: CrankDeg10(0),
            dropout_end_deg10: CrankDeg10(0),
            invert_polarity: false,
            suppress_cam: false,
            suppress_crank: false,
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn generate_edges<const MAX: usize>(
    config: TriggerConfig,
    start: CrankDeg10,
    end: CrankDeg10,
    step_start_us: Micros,
    dt_us: Micros,
    faults: FaultInput,
    sequence: &mut u32,
    out: &mut FixedSlice<TriggerEdge, MAX>,
) -> Result<(), CapacityError> {
    if config.crank_teeth == 0 || config.missing_teeth >= config.crank_teeth {
        return Ok(());
    }
    if dt_us.0 == 0 || start == end {
        return Ok(());
    }
    let distance = if end.0 >= start.0 {
        end.0 as u32 - start.0 as u32
    } else {
        CYCLE_DEG10 - start.0 as u32 + end.0 as u32
    };
    if distance == 0 {
        return Ok(());
    }

    let spacing = CYCLE_DEG10 / config.crank_teeth as u32;
    if spacing == 0 {
        return Ok(());
    }
    let active_teeth = config.crank_teeth.saturating_sub(config.missing_teeth) as u32;

    let cam_spacing = if config.cam_pulses == 0 {
        0
    } else {
        CYCLE_DEG10 / config.cam_pulses as u32
    };
    let mut cam_sequence_this_step = 0u8;
    let mut last_travel = 0u32;

    loop {
        let mut best_travel = u32::MAX;

        for tooth in 0..config.crank_teeth as u32 {
            if tooth >= active_teeth {
                continue;
            }
            let tooth_angle = apply_phase_deg10(tooth * spacing, faults.phase_offset_deg10);
            let travel = travel_from_start(start, tooth_angle);
            if travel > last_travel && travel <= distance && travel < best_travel {
                best_travel = travel;
            }
        }

        if cam_spacing != 0 {
            for pulse in 0..config.cam_pulses as u32 {
                let cam_angle = apply_phase_deg10(pulse * cam_spacing, faults.phase_offset_deg10);
                let travel = travel_from_start(start, cam_angle);
                if travel > last_travel && travel <= distance && travel < best_travel {
                    best_travel = travel;
                }
            }
        }

        if best_travel == u32::MAX {
            break;
        }

        for tooth in 0..config.crank_teeth as u32 {
            if tooth >= active_teeth {
                continue;
            }
            let tooth_angle = apply_phase_deg10(tooth * spacing, faults.phase_offset_deg10);
            if travel_from_start(start, tooth_angle) != best_travel {
                continue;
            }
            if !faults.suppress_crank
                && *sequence >= faults.drop_next_crank_edges as u32
                && !in_dropout_window(tooth_angle, faults)
            {
                let mut edge = make_edge(
                    step_start_us,
                    dt_us,
                    best_travel,
                    distance,
                    TriggerChannel::Crank,
                    tooth,
                    tooth_angle,
                    faults,
                );
                out.push(edge)?;
                if *sequence < faults.duplicate_next_crank_edges as u32 {
                    edge.timestamp_us.0 = edge.timestamp_us.0.saturating_add(1);
                    out.push(edge)?;
                }
            }
            *sequence = sequence.saturating_add(1);
        }

        if cam_spacing != 0 {
            for pulse in 0..config.cam_pulses as u32 {
                let cam_angle = apply_phase_deg10(pulse * cam_spacing, faults.phase_offset_deg10);
                if travel_from_start(start, cam_angle) != best_travel {
                    continue;
                }
                if !faults.suppress_cam
                    && cam_sequence_this_step >= faults.drop_next_cam_edges
                    && !in_dropout_window(cam_angle, faults)
                {
                    let mut edge = make_edge(
                        step_start_us,
                        dt_us,
                        best_travel,
                        distance,
                        TriggerChannel::Cam,
                        pulse,
                        cam_angle,
                        faults,
                    );
                    out.push(edge)?;
                    if cam_sequence_this_step < faults.duplicate_next_cam_edges {
                        edge.timestamp_us.0 = edge.timestamp_us.0.saturating_add(1);
                        out.push(edge)?;
                    }
                }
                cam_sequence_this_step = cam_sequence_this_step.saturating_add(1);
            }
        }

        last_travel = best_travel;
    }

    Ok(())
}

fn travel_from_start(start: CrankDeg10, angle: u32) -> u32 {
    if angle > start.0 as u32 {
        angle - start.0 as u32
    } else {
        CYCLE_DEG10 - start.0 as u32 + angle
    }
}

fn apply_phase_deg10(angle: u32, offset: Degrees10) -> u32 {
    normalize_deg10_i32(angle as i32 + offset.0 as i32).0 as u32
}

fn in_dropout_window(angle: u32, faults: FaultInput) -> bool {
    if !faults.dropout_enabled {
        return false;
    }
    let angle = angle as u16;
    let start = faults.dropout_start_deg10.0;
    let end = faults.dropout_end_deg10.0;
    if start <= end {
        angle >= start && angle <= end
    } else {
        angle >= start || angle <= end
    }
}

#[allow(clippy::too_many_arguments)]
fn make_edge(
    step_start_us: Micros,
    dt_us: Micros,
    travel: u32,
    distance: u32,
    channel: TriggerChannel,
    index: u32,
    angle: u32,
    faults: FaultInput,
) -> TriggerEdge {
    let base_offset = (dt_us.0 as u64 * travel as u64 / distance as u64) as i64;
    let fault_offset = faults.delay_next_edge_us.0 as i64 + faults.jitter_next_edge_us as i64;
    let timestamp = (step_start_us.0 as i64 + base_offset + fault_offset).max(0) as u32;
    let rising = index.is_multiple_of(2);
    let polarity = match rising ^ faults.invert_polarity {
        true => EdgePolarity::Rising,
        false => EdgePolarity::Falling,
    };
    TriggerEdge {
        timestamp_us: Micros(timestamp),
        channel,
        edge: polarity,
        crank_angle_deg10: normalize_deg10(angle),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn trigger_config() -> TriggerConfig {
        TriggerConfig {
            crank_teeth: 36,
            missing_teeth: 1,
            cam_pulses: 1,
        }
    }

    #[test]
    fn wraparound_edges_are_timestamp_monotonic_without_faults() {
        let mut sequence = 0;
        let mut edges = FixedSlice::<TriggerEdge, 16>::empty(TriggerEdge::empty());

        generate_edges(
            trigger_config(),
            CrankDeg10(7000),
            CrankDeg10(1000),
            Micros(10_000),
            Micros(50_000),
            FaultInput::none(),
            &mut sequence,
            &mut edges,
        )
        .unwrap();

        for pair in edges.as_slice().windows(2) {
            assert!(pair[0].timestamp_us <= pair[1].timestamp_us);
        }
        assert!(edges.len() > 1);
    }

    #[test]
    fn phase_offset_moves_generated_edge_angles() {
        let mut sequence = 0;
        let mut edges = FixedSlice::<TriggerEdge, 8>::empty(TriggerEdge::empty());
        let mut faults = FaultInput::none();
        faults.phase_offset_deg10 = Degrees10(50);

        generate_edges(
            trigger_config(),
            CrankDeg10(0),
            CrankDeg10(500),
            Micros(0),
            Micros(10_000),
            faults,
            &mut sequence,
            &mut edges,
        )
        .unwrap();

        assert_eq!(edges.as_slice()[0].crank_angle_deg10, CrankDeg10(50));
    }

    #[test]
    fn dropout_window_suppresses_matching_edges() {
        let mut sequence = 0;
        let mut edges = FixedSlice::<TriggerEdge, 8>::empty(TriggerEdge::empty());
        let mut faults = FaultInput::none();
        faults.dropout_enabled = true;
        faults.dropout_start_deg10 = CrankDeg10(0);
        faults.dropout_end_deg10 = CrankDeg10(300);

        generate_edges(
            TriggerConfig {
                crank_teeth: 36,
                missing_teeth: 0,
                cam_pulses: 0,
            },
            CrankDeg10(0),
            CrankDeg10(700),
            Micros(0),
            Micros(10_000),
            faults,
            &mut sequence,
            &mut edges,
        )
        .unwrap();

        assert!(edges
            .as_slice()
            .iter()
            .all(|edge| edge.crank_angle_deg10.0 > 300));
        assert!(!edges.is_empty());
    }

    #[test]
    fn cam_edges_can_be_dropped_and_duplicated() {
        let mut sequence = 0;
        let mut edges = FixedSlice::<TriggerEdge, 4>::empty(TriggerEdge::empty());
        let mut faults = FaultInput::none();
        faults.suppress_crank = true;
        faults.duplicate_next_cam_edges = 1;

        generate_edges(
            trigger_config(),
            CrankDeg10(7000),
            CrankDeg10(100),
            Micros(0),
            Micros(10_000),
            faults,
            &mut sequence,
            &mut edges,
        )
        .unwrap();

        assert_eq!(edges.len(), 2);
        assert_eq!(edges.as_slice()[0].channel, TriggerChannel::Cam);
        assert_eq!(
            edges.as_slice()[0].crank_angle_deg10,
            edges.as_slice()[1].crank_angle_deg10
        );

        let mut dropped = FixedSlice::<TriggerEdge, 4>::empty(TriggerEdge::empty());
        faults.duplicate_next_cam_edges = 0;
        faults.drop_next_cam_edges = 1;
        generate_edges(
            trigger_config(),
            CrankDeg10(7000),
            CrankDeg10(100),
            Micros(0),
            Micros(10_000),
            faults,
            &mut sequence,
            &mut dropped,
        )
        .unwrap();
        assert_eq!(dropped.len(), 0);
    }
}

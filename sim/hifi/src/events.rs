const ENGINE_CYCLE_RAD: f64 = 4.0 * core::f64::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CycleEventKind {
    Ivo,
    Ivc,
    Evo,
    Evc,
    Spark,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CycleEvent {
    pub kind: CycleEventKind,
    pub angle_rad_offset: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct EventTable {
    events: [Option<CycleEvent>; 5],
}

impl EventTable {
    pub const fn empty() -> Self {
        Self { events: [None; 5] }
    }

    pub const fn with_events(events: [Option<CycleEvent>; 5]) -> Self {
        Self { events }
    }

    pub fn next_boundary_after(
        &self,
        current_theta_rad: f64,
        end_theta_rad: f64,
        cycle_start_rad: f64,
    ) -> Option<(f64, CycleEventKind)> {
        let mut next: Option<(f64, CycleEventKind)> = None;
        for event in self.events.iter().flatten() {
            let candidate = cycle_start_rad
                + (event.angle_rad_offset - cycle_start_rad).rem_euclid(ENGINE_CYCLE_RAD);
            if candidate <= current_theta_rad + 1.0e-12 || candidate >= end_theta_rad - 1.0e-12 {
                continue;
            }
            next = match next {
                Some((existing_angle, existing_kind)) if existing_angle <= candidate => {
                    Some((existing_angle, existing_kind))
                }
                _ => Some((candidate, event.kind)),
            };
        }
        next
    }
}

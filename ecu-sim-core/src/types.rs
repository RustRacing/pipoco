pub const MAX_CYLINDERS: usize = 8;
pub const MAX_TRIGGER_EDGES_PER_STEP: usize = 64;
pub const MAX_ECU_EVENTS_PER_STEP: usize = 32;
pub const MAX_DIAGNOSTIC_EVENTS_PER_STEP: usize = 16;
pub const MAX_TABLE_AXIS_POINTS: usize = 8;
pub const CRANK_CYCLE_DEG10: u16 = 7200;
pub const CYCLE_DEG10: u32 = CRANK_CYCLE_DEG10 as u32;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Micros(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Millis(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Rpm(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct CrankDeg10(pub u16);

impl CrankDeg10 {
    pub const fn new_normalized(value: u16) -> Self {
        Self(value % CRANK_CYCLE_DEG10)
    }

    pub const fn is_normalized(self) -> bool {
        self.0 < CRANK_CYCLE_DEG10
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Degrees10(pub i16);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Kpa10(pub u16);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Celsius10(pub i16);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Millivolts(pub u16);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct TorqueNmX100(pub i32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct PowerWatts(pub i32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct MassUg(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct MicrogramsPerMicros(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct CylinderIndex(pub u8);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Kelvin10(pub u16);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct PressurePa(pub i32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct VolumeMm3(pub u64);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct VolumeDerivativeMm3PerRadQ16(pub i64);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct EnergyMicroJ(pub i64);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct AngularVelocityRadPerSecQ16(pub i64);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct BmepBarX100(pub i32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct ImepBarX100(pub i32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct PmepBarX100(pub i32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct FmepBarX100(pub i32);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapacityError {
    pub capacity: usize,
}

pub const fn normalize_deg10(value: u32) -> CrankDeg10 {
    CrankDeg10((value % CYCLE_DEG10) as u16)
}

pub const fn normalize_deg10_i32(value: i32) -> CrankDeg10 {
    let mut normalized = value % CRANK_CYCLE_DEG10 as i32;
    if normalized < 0 {
        normalized += CRANK_CYCLE_DEG10 as i32;
    }
    CrankDeg10(normalized as u16)
}

pub const fn clamp_u16(value: u32, max: u16) -> u16 {
    if value > max as u32 {
        max
    } else {
        value as u16
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedSlice<T, const N: usize> {
    len: usize,
    items: [T; N],
}

impl<T: Copy, const N: usize> FixedSlice<T, N> {
    pub const fn empty(fill: T) -> Self {
        Self {
            len: 0,
            items: [fill; N],
        }
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    pub fn push(&mut self, item: T) -> Result<(), CapacityError> {
        if self.len == N {
            return Err(CapacityError { capacity: N });
        }

        self.items[self.len] = item;
        self.len += 1;
        Ok(())
    }

    pub fn as_slice(&self) -> &[T] {
        &self.items[..self.len]
    }

    pub fn as_mut_slice(&mut self) -> &mut [T] {
        &mut self.items[..self.len]
    }

    pub const fn len(&self) -> usize {
        self.len
    }

    pub const fn capacity(&self) -> usize {
        N
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_slice_push_and_as_slice_observe_len_only() {
        let mut items = FixedSlice::<u8, 3>::empty(0);

        assert!(items.is_empty());
        assert_eq!(items.as_slice(), &[]);

        items.push(7).unwrap();
        items.push(9).unwrap();

        assert_eq!(items.len(), 2);
        assert_eq!(items.capacity(), 3);
        assert_eq!(items.as_slice(), &[7, 9]);
    }

    #[test]
    fn fixed_slice_reports_capacity_without_mutating_len() {
        let mut items = FixedSlice::<u8, 1>::empty(0);

        assert_eq!(items.push(1), Ok(()));
        assert_eq!(items.push(2), Err(CapacityError { capacity: 1 }));
        assert_eq!(items.as_slice(), &[1]);
    }

    #[test]
    fn fixed_slice_clear_drops_logical_items_only() {
        let mut items = FixedSlice::<u8, 2>::empty(0);

        items.push(1).unwrap();
        items.clear();

        assert_eq!(items.len(), 0);
        assert_eq!(items.as_slice(), &[]);
    }
}

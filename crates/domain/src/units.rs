/// Engine speed in revolutions per minute.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Rpm(u16);

impl Rpm {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }

    pub const fn raw(self) -> u16 {
        self.0
    }
}

/// Microseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Micros(u32);

impl Micros {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u32 {
        self.0
    }

    pub const fn raw(self) -> u32 {
        self.0
    }
}

/// Scheduler ticks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Ticks(u32);

impl Ticks {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u32 {
        self.0
    }

    pub const fn raw(self) -> u32 {
        self.0
    }
}

/// Manifold pressure in kPa x 10.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Kpa10(u16);

impl Kpa10 {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn try_new(value: u16) -> Result<Self, UnitRangeError> {
        if value <= Self::MAX_PLAUSIBLE {
            Ok(Self(value))
        } else {
            Err(UnitRangeError::OutOfRange)
        }
    }

    pub const fn get(self) -> u16 {
        self.0
    }

    pub const fn raw(self) -> u16 {
        self.0
    }

    pub const MAX_PLAUSIBLE: u16 = 7_000;
}

/// Mass air flow in source-native units x 100.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct MassAirFlowX100(u16);

impl MassAirFlowX100 {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Normalized knock sensor level x 100.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct KnockLevelX100(u16);

impl KnockLevelX100 {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn try_new(value: u16) -> Result<Self, UnitRangeError> {
        if value <= Self::MAX_NORMALIZED {
            Ok(Self(value))
        } else {
            Err(UnitRangeError::OutOfRange)
        }
    }

    pub const fn get(self) -> u16 {
        self.0
    }

    pub const MAX_NORMALIZED: u16 = 10_000;
}

/// Percent value in the range 0-100.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Percent(u8);

impl Percent {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    pub const fn try_new(value: u8) -> Result<Self, UnitRangeError> {
        if value <= 100 {
            Ok(Self(value))
        } else {
            Err(UnitRangeError::OutOfRange)
        }
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}

/// Injector pulse width in microseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct PulseWidthUs(u16);

impl PulseWidthUs {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Dwell time in microseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct DwellUs(u16);

impl DwellUs {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Crank angle in tenths of a degree.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Degrees10(i16);

impl Degrees10 {
    pub const fn new(value: i16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> i16 {
        self.0
    }
}

/// Measured cam phase in tenths of a degree.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct CamPhaseDeg10(i16);

impl CamPhaseDeg10 {
    pub const fn new(value: i16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> i16 {
        self.0
    }
}

/// Lambda value scaled by 100.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Lambda100(u16);

impl Lambda100 {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn try_new(value: u16) -> Result<Self, UnitRangeError> {
        if value <= Self::MAX_PLAUSIBLE {
            Ok(Self(value))
        } else {
            Err(UnitRangeError::OutOfRange)
        }
    }

    pub const fn get(self) -> u16 {
        self.0
    }

    pub const MAX_PLAUSIBLE: u16 = 300;
}

/// Vehicle speed in km/h x 10.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct VehicleSpeedKph10(u16);

impl VehicleSpeedKph10 {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn try_new(value: u16) -> Result<Self, UnitRangeError> {
        if value <= Self::MAX_PLAUSIBLE {
            Ok(Self(value))
        } else {
            Err(UnitRangeError::OutOfRange)
        }
    }

    pub const fn get(self) -> u16 {
        self.0
    }

    pub const MAX_PLAUSIBLE: u16 = 5_000;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum UnitRangeError {
    OutOfRange,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quantity_newtypes_round_trip() {
        assert_eq!(Rpm::new(1234).get(), 1234);
        assert_eq!(Micros::new(42).get(), 42);
        assert_eq!(Ticks::new(7).get(), 7);
        assert_eq!(Kpa10::new(987).get(), 987);
        assert_eq!(Percent::new(88).get(), 88);
        assert_eq!(PulseWidthUs::new(1500).get(), 1500);
        assert_eq!(DwellUs::new(2500).get(), 2500);
        assert_eq!(Degrees10::new(-125).get(), -125);
        assert_eq!(Lambda100::new(101).get(), 101);
        assert_eq!(VehicleSpeedKph10::new(1234).get(), 1234);
    }

    #[test]
    fn checked_unit_constructors_reject_out_of_range_values() {
        assert_eq!(Percent::try_new(100), Ok(Percent::new(100)));
        assert_eq!(Percent::try_new(101), Err(UnitRangeError::OutOfRange));
        assert_eq!(Kpa10::try_new(7_000), Ok(Kpa10::new(7_000)));
        assert_eq!(Kpa10::try_new(7_001), Err(UnitRangeError::OutOfRange));
        assert_eq!(
            KnockLevelX100::try_new(10_001),
            Err(UnitRangeError::OutOfRange)
        );
        assert_eq!(Lambda100::try_new(301), Err(UnitRangeError::OutOfRange));
        assert_eq!(
            VehicleSpeedKph10::try_new(5_001),
            Err(UnitRangeError::OutOfRange)
        );
    }
}

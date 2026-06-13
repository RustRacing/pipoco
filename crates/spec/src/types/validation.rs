use super::calibration::Calibration;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ValidatedCalibration(pub Calibration);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValidationError {
    AxisTooShort,
    AxisNotStrictlyIncreasing,
    TableDimensionMismatch,
    CurveDimensionMismatch,
    CylinderCountZero,
    CylinderCountTooLarge,
    AngleOutOfRange,
    RequiredFuelZero,
    ReferencePressureZero,
    PwMaxZero,
    CorrectionBelowZero,
    CorrectionAboveLimit,
    VeOutOfRange,
    AfrOutOfRange,
    DwellOutOfRange,
    SparkAdvanceOutOfRange,
    InjectionTargetOutOfRange,
    TargetAfrOverrideOutOfRange,
    DfcoConfigInvalid,
    RevLimitConfigInvalid,
    KnockConfigInvalid,
    LaunchConfigInvalid,
    FlatShiftConfigInvalid,
    IdleConfigInvalid,
    TpsConfigInvalid,
    O2ConfigInvalid,
}

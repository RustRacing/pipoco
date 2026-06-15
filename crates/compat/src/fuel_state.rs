use crate::constants::corrections::UNITY_CORRECTION;

/// Fixed-point math helper (no floats!)
///
/// Multiplies value by (multiplier / 100) using integer arithmetic only.
/// Uses saturating multiplication to prevent overflow.
///
/// # Arguments
/// * `value` - Base value (e.g., pulse width in microseconds)
/// * `multiplier` - Correction factor scaled by 100 (e.g., 150 = 1.5x, 80 = 0.8x)
///
/// # Returns
/// Corrected value, saturated at u16::MAX if overflow would occur
///
/// # Example
/// ```
/// use ecu_compat::scale_u16;
///
/// assert_eq!(scale_u16(1000, 150), 1500);  // 1.5x
/// assert_eq!(scale_u16(1000, 80), 800);    // 0.8x
/// assert_eq!(scale_u16(1000, 100), 1000);  // 1.0x (no change)
/// ```
pub fn scale_u16(value: u16, multiplier: u8) -> u16 {
    // Use saturating multiply to prevent overflow
    let intermediate = (value as u32).saturating_mul(multiplier as u32);
    let result = intermediate / 100;

    // Clamp to u16::MAX
    if result > u16::MAX as u32 {
        u16::MAX
    } else {
        result as u16
    }
}

/// Apply a signed closed-loop delta in percent to a pulse width.
/// Positive increases fuel, negative decreases.
pub fn apply_cl_delta(pw: u16, cl_delta_percent: i16) -> u16 {
    if cl_delta_percent == 0 {
        return pw;
    }
    if cl_delta_percent > 0 {
        let m = (100i16 + cl_delta_percent).clamp(0, 200) as u8;
        scale_u16(pw, m)
    } else {
        // Decrease: scale by (100 - |delta|)
        let m = (100i16 - (-cl_delta_percent)).clamp(0, 200) as u8;
        scale_u16(pw, m)
    }
}

/// Correction multipliers (100 = 1.0x)
///
/// All corrections are represented as integers scaled by 100 to avoid
/// floating-point operations. A value of 100 means no correction (1.0x).
///
/// # Examples
/// - 150 = 1.5x (add 50% fuel)
/// - 80 = 0.8x (reduce fuel by 20%)
/// - 100 = 1.0x (no change)
#[derive(Debug, Clone, Copy)]
pub struct Corrections {
    /// Coolant temperature correction
    pub clt: u8,
    /// Intake air temperature correction
    pub iat: u8,
    /// Battery voltage correction (compensates for injector opening time)
    pub vbatt: u8,
}

impl Corrections {
    /// Default corrections (1.0x all - no corrections applied)
    pub const DEFAULT: Self = Self {
        clt: UNITY_CORRECTION,
        iat: UNITY_CORRECTION,
        vbatt: UNITY_CORRECTION,
    };

    /// Create new corrections with specified values
    pub const fn new(clt: u8, iat: u8, vbatt: u8) -> Self {
        Self { clt, iat, vbatt }
    }
}

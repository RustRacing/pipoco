pub const MAX_RPM_BINS: usize = 16;
pub const MAX_LOAD_BINS: usize = 16;
pub const MAX_CURVE_POINTS: usize = 16;
pub const MAX_CYLINDERS: usize = 8;
pub const MAX_CYLINDER_STORAGE: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Axis16 {
    pub len: u8,
    pub values: [u16; MAX_CURVE_POINTS],
}

impl Default for Axis16 {
    fn default() -> Self {
        Self {
            len: 0,
            values: [0; MAX_CURVE_POINTS],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Curve16 {
    pub axis: Axis16,
    pub values: [u16; MAX_CURVE_POINTS],
}

impl Default for Curve16 {
    fn default() -> Self {
        Self {
            axis: Axis16::default(),
            values: [0; MAX_CURVE_POINTS],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SignedCurve16 {
    pub axis: Axis16,
    pub values: [i16; MAX_CURVE_POINTS],
}

impl Default for SignedCurve16 {
    fn default() -> Self {
        Self {
            axis: Axis16::default(),
            values: [0; MAX_CURVE_POINTS],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Table2D16<T> {
    pub rpm_axis: Axis16,
    pub load_axis: Axis16,
    pub values: [[T; MAX_CURVE_POINTS]; MAX_CURVE_POINTS],
}

impl<T: Copy + Default> Default for Table2D16<T> {
    fn default() -> Self {
        Self {
            rpm_axis: Axis16::default(),
            load_axis: Axis16::default(),
            values: [[T::default(); MAX_CURVE_POINTS]; MAX_CURVE_POINTS],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CylinderArrayU16 {
    pub count: u8,
    pub values: [u16; MAX_CYLINDER_STORAGE],
}

impl Default for CylinderArrayU16 {
    fn default() -> Self {
        Self {
            count: 0,
            values: [0; MAX_CYLINDER_STORAGE],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CylinderArrayI16 {
    pub count: u8,
    pub values: [i16; MAX_CYLINDER_STORAGE],
}

impl Default for CylinderArrayI16 {
    fn default() -> Self {
        Self {
            count: 0,
            values: [0; MAX_CYLINDER_STORAGE],
        }
    }
}

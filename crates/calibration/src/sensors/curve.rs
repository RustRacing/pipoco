//! Generic piecewise-linear calibration curve
//!
//! Maps an input `x` to an output `y` via N monotonic breakpoints.
//! Intended for no_std environments and small N (<= 16).
//!
//! - `x` domain typically millivolts or ohms (u32)
//! - `y` range typically engineering units scaled (i32), e.g., degC, kPa*10, lambda*1000

#[derive(Copy, Clone)]
pub struct Piecewise<const N: usize> {
    pub xs: [u32; N],
    pub ys: [i32; N],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PiecewiseError {
    EmptyDomain,
    DegenerateDomain,
    UnsortedDomain,
    DuplicateBreakpoint,
}

impl<const N: usize> Piecewise<N> {
    pub const fn new(xs: [u32; N], ys: [i32; N]) -> Self {
        Self { xs, ys }
    }

    pub fn is_sorted(&self) -> bool {
        if N < 2 {
            return true;
        }
        let mut i = 1;
        while i < N {
            if self.xs[i] < self.xs[i - 1] {
                return false;
            }
            i += 1;
        }
        true
    }

    pub fn validate_domain(&self) -> Result<(), PiecewiseError> {
        if N == 0 {
            return Err(PiecewiseError::EmptyDomain);
        }
        if N == 1 {
            return Err(PiecewiseError::DegenerateDomain);
        }

        let mut i = 1;
        while i < N {
            if self.xs[i] < self.xs[i - 1] {
                return Err(PiecewiseError::UnsortedDomain);
            }
            if self.xs[i] == self.xs[i - 1] {
                return Err(PiecewiseError::DuplicateBreakpoint);
            }
            i += 1;
        }

        Ok(())
    }

    /// Map `x` to `y` using linear interpolation between breakpoints; clamps to ends.
    pub fn map(&self, x: u32) -> i32 {
        self.map_checked(x).unwrap_or(0)
    }

    /// Map `x` to `y`, rejecting malformed piecewise domains explicitly.
    pub fn map_checked(&self, x: u32) -> Result<i32, PiecewiseError> {
        self.validate_domain()?;

        if N == 0 {
            return Err(PiecewiseError::EmptyDomain);
        }
        if N == 1 {
            return Err(PiecewiseError::DegenerateDomain);
        }
        if x <= self.xs[0] {
            return Ok(self.ys[0]);
        }
        if x >= self.xs[N - 1] {
            return Ok(self.ys[N - 1]);
        }

        // find segment idx such that xs[idx] <= x < xs[idx+1]
        let mut idx = 0usize;
        while idx + 1 < N && !(self.xs[idx] <= x && x < self.xs[idx + 1]) {
            idx += 1;
        }
        if idx + 1 >= N {
            return Ok(self.ys[N - 1]);
        }

        let x0 = self.xs[idx] as i64;
        let x1 = self.xs[idx + 1] as i64;
        let y0 = self.ys[idx] as i64;
        let y1 = self.ys[idx + 1] as i64;
        let dx = x1 - x0;
        let num = (y1 - y0) * (x as i64 - x0);
        let y = y0 + num / dx;
        Ok(y as i32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_piecewise_basic() {
        let c = Piecewise::<3>::new([0, 1000, 2000], [0, 10, 20]);
        assert!(c.is_sorted());
        assert_eq!(c.map(0), 0);
        assert_eq!(c.map(500), 5);
        assert_eq!(c.map(1500), 15);
        assert_eq!(c.map(3000), 20);
    }

    #[test]
    fn piecewise_checked_rejects_empty_and_single_point_domains() {
        let empty = Piecewise::<0>::new([], []);
        assert_eq!(empty.validate_domain(), Err(PiecewiseError::EmptyDomain));
        assert_eq!(empty.map_checked(0), Err(PiecewiseError::EmptyDomain));

        let single = Piecewise::<1>::new([100], [7]);
        assert_eq!(
            single.validate_domain(),
            Err(PiecewiseError::DegenerateDomain)
        );
        assert_eq!(
            single.map_checked(100),
            Err(PiecewiseError::DegenerateDomain)
        );
    }

    #[test]
    fn piecewise_checked_rejects_unsorted_and_duplicate_breakpoints() {
        let unsorted = Piecewise::<3>::new([0, 2000, 1000], [0, 20, 10]);
        assert_eq!(
            unsorted.validate_domain(),
            Err(PiecewiseError::UnsortedDomain)
        );
        assert_eq!(
            unsorted.map_checked(1500),
            Err(PiecewiseError::UnsortedDomain)
        );

        let duplicate = Piecewise::<3>::new([0, 1000, 1000], [0, 10, 11]);
        assert_eq!(
            duplicate.validate_domain(),
            Err(PiecewiseError::DuplicateBreakpoint)
        );
        assert_eq!(
            duplicate.map_checked(1000),
            Err(PiecewiseError::DuplicateBreakpoint)
        );
    }
}

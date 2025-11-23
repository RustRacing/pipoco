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

    /// Map `x` to `y` using linear interpolation between breakpoints; clamps to ends.
    pub fn map(&self, x: u32) -> i32 {
        if N == 0 {
            return 0;
        }
        if N == 1 {
            return self.ys[0];
        }
        if x <= self.xs[0] {
            return self.ys[0];
        }
        if x >= self.xs[N - 1] {
            return self.ys[N - 1];
        }

        // find segment idx such that xs[idx] <= x < xs[idx+1]
        let mut idx = 0usize;
        while idx + 1 < N && !(self.xs[idx] <= x && x < self.xs[idx + 1]) {
            idx += 1;
        }
        if idx + 1 >= N {
            return self.ys[N - 1];
        }

        let x0 = self.xs[idx] as i64;
        let x1 = self.xs[idx + 1] as i64;
        let y0 = self.ys[idx] as i64;
        let y1 = self.ys[idx + 1] as i64;
        let dx = (x1 - x0).max(1);
        let num = (y1 - y0) * (x as i64 - x0);
        let y = y0 + num / dx;
        y as i32
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
}

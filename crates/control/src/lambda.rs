use ecu_domain::Lambda100;

/// Lambda control operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LambdaMode {
    #[default]
    OpenLoop,
    ClosedLoop,
}

/// Configuration for the first-pass lambda trim planner.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LambdaTrimConfig {
    pub open_loop_target: Lambda100,
    pub closed_loop_target: Lambda100,
    pub enable_clt_c: i16,
    pub disable_clt_c: i16,
    pub min_trim_x100: i16,
    pub max_trim_x100: i16,
    pub gain_x10: u8,
}

impl LambdaTrimConfig {
    pub const DEFAULT: Self = Self {
        open_loop_target: Lambda100::new(100),
        closed_loop_target: Lambda100::new(100),
        enable_clt_c: 40,
        disable_clt_c: 30,
        min_trim_x100: 85,
        max_trim_x100: 115,
        gain_x10: 4,
    };
}

impl Default for LambdaTrimConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Inputs required to compute lambda trim.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LambdaTrimInputs {
    pub clt_c: i16,
    pub lambda_valid: bool,
    pub measured_lambda100: Lambda100,
    pub requested_open_loop: bool,
}

/// Typed lambda trim result for downstream consumers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LambdaTrimResult {
    pub mode: LambdaMode,
    pub active: bool,
    pub target_lambda100: Lambda100,
    pub measured_lambda100: Lambda100,
    pub trim_x100: i16,
}

impl LambdaTrimResult {
    pub const fn new(
        mode: LambdaMode,
        active: bool,
        target_lambda100: Lambda100,
        measured_lambda100: Lambda100,
        trim_x100: i16,
    ) -> Self {
        Self {
            mode,
            active,
            target_lambda100,
            measured_lambda100,
            trim_x100,
        }
    }

    pub const fn identity(target_lambda100: Lambda100, measured_lambda100: Lambda100) -> Self {
        Self::new(
            LambdaMode::OpenLoop,
            false,
            target_lambda100,
            measured_lambda100,
            100,
        )
    }
}

/// First-pass lambda trim planner.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LambdaTrimPlanner {
    last_mode: LambdaMode,
    last_trim_x100: i16,
}

impl LambdaTrimPlanner {
    pub const fn new() -> Self {
        Self {
            last_mode: LambdaMode::OpenLoop,
            last_trim_x100: 100,
        }
    }

    pub fn update(&mut self, inputs: LambdaTrimInputs, cfg: &LambdaTrimConfig) -> LambdaTrimResult {
        let closed_loop_enabled = inputs.lambda_valid
            && !inputs.requested_open_loop
            && inputs.clt_c >= cfg.enable_clt_c
            && (self.last_mode == LambdaMode::ClosedLoop || inputs.clt_c >= cfg.disable_clt_c);

        if !closed_loop_enabled {
            self.last_mode = LambdaMode::OpenLoop;
            self.last_trim_x100 = 100;
            return LambdaTrimResult::new(
                LambdaMode::OpenLoop,
                false,
                cfg.open_loop_target,
                inputs.measured_lambda100,
                100,
            );
        }

        let target = cfg.closed_loop_target.get() as i16;
        let measured = inputs.measured_lambda100.get() as i16;
        let error = target - measured;
        let mut trim = 100 + (error * cfg.gain_x10 as i16) / 10;
        trim = trim.clamp(cfg.min_trim_x100, cfg.max_trim_x100);

        self.last_mode = LambdaMode::ClosedLoop;
        self.last_trim_x100 = trim;
        LambdaTrimResult::new(
            LambdaMode::ClosedLoop,
            true,
            cfg.closed_loop_target,
            inputs.measured_lambda100,
            trim,
        )
    }
}

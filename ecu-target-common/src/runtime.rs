#[macro_export]
macro_rules! tick_once {
    (
        $APP:ident,
        $capture_pop:path,
        $now_expr:expr,
        $outs_expr:expr,
        $ts_pump:block,
        $pet_expr:expr
    ) => {{
        while let Some(ts) = $capture_pop() {
            cortex_m::interrupt::free(|_| unsafe {
                if let Some(ref mut a) = $APP { a.on_timestamp(ts); }
            });
        }
        let now = $now_expr;
        let mut outputs = $outs_expr;
        cortex_m::interrupt::free(|_| unsafe {
            if let Some(ref mut a) = $APP { a.drive_outputs(now, &mut outputs) }
        });
        $ts_pump
        $pet_expr
    }};
}

# ecu-trigger

`ecu-trigger` is the trigger-model crate used by the workspace for decoder setup, authority tracking, and sync-state evaluation.

It is intentionally `#![no_std]`, has no allocator dependency, and uses `#![forbid(unsafe_code)]`. Keep additions heap-free, deterministic, and free of unsafe code.

## TriggerProfile Field Split

These fields are the Speeduino-compatible setup surface:

- `pattern`
- `primary_speed`
- `primary_edge`
- `secondary.{mode, edge, poll_level}`
- `tooth_angle_multiplier`
- `filter`
- `resync`
- `startup.{skip_revolutions, require_full_cycle}`
- `latency.{primary_edge_delay_us, secondary_edge_delay_us, output_schedule_delay_us}`

This field is the Pipoco authority extension:

- `trigger_angle_atdc_deg10`

`trigger_angle_atdc_deg10` carries the provenance of tooth-1 absolute timing through `TriggerAngleAuthority`. Generic wheel geometry can prove sync shape, but it does not prove absolute timing. Only `ExpertManual`, `CommunityProfile`, `CertifiedProfile`, or `BenchLearned` authority can do that.

## Safety Boundary

Use the trigger pattern and cam/primary geometry to prove that the decoder can lock and stay in sync. Do not treat geometry alone as a certification of absolute timing or safe sequential authority.

The default `Unknown` authority means the timing claim is not proven yet. That is the correct state for profiles that have not been backed by manual evidence, community evidence, certified evidence, or bench-learned evidence.

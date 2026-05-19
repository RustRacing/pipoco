# ecu-board-profiles

`ecu-board-profiles` collects board and engine profile slices that sit on top of `ecu-trigger`.

The profile data is split between Speeduino-compatible setup fields and Pipoco authority/safety extensions.

## Speeduino-Compatible Setup Slice

These fields are the compatibility-oriented setup surface:

- `engine.{cylinders, firing_order, cycle}`
- `trigger.{pattern, primary_speed, primary_edge, secondary, tooth_angle_multiplier, filter, resync, startup, latency}`
- `cam.{cams, sensor_default}`
- `injection.{strategy, channels}`
- `ignition.topology`
- `aux.outputs`
- `sensor_scaling.{clt, iat, map, vbatt, lambda, crank, cam}`
- `hardware_map.bindings`

## Pipoco Authority And Safety Extensions

These fields carry Pipoco-specific authority or safety meaning:

- `trigger.trigger_angle_atdc_deg10`
- `cam.phase_required_for_sequential`
- `safety.{sync_loss_cut_fuel, sync_loss_cut_ignition, safe_aux_outputs, notes}`
- `hardware_map.provenance`

The `trigger_angle_atdc_deg10` field is still governed by the same safety boundary as `ecu-trigger`: geometry proves sync shape, but only expert/manual, community, certified, or bench-learned authority can prove absolute timing.

## M50B25TU Profile Note

`M50B25TU_MEGA_COMPAT` keeps `trigger.trigger_angle_atdc_deg10` at `Unknown`. That means the profile is not certified for absolute tooth-1 timing yet, even though the 60-2 plus cam geometry is enough to show the sync shape.

`M50B25TU_FULL_COP` inherits that same timing authority state. It only changes the ignition topology.

The hardware bindings in this crate are a symbolic compatibility sketch, not a pin-verified production harness map.

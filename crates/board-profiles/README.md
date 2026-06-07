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
- `sensor_scaling.{clt, iat, map, baro, tps, maf, vbatt, lambda, vss, knock_front, knock_rear, crank, cam}`
- `sensor_inventory.entries`
- `hardware_map.bindings`

## Pipoco Authority And Safety Extensions

These fields carry Pipoco-specific authority or safety meaning:

- `trigger.trigger_angle_atdc_deg10`
- `cam.phase_required_for_sequential`
- `safety.{sync_loss_cut_fuel, sync_loss_cut_ignition, safe_aux_outputs, notes}`
- `hardware_map.provenance`

The `trigger_angle_atdc_deg10` field is still governed by the same safety boundary as `ecu-trigger`: geometry proves sync shape, but only expert/manual, community, certified, or bench-learned authority can prove absolute timing.

## Generic Software Readiness Boundary

The profile crate provides generic first-start compatibility primitives:

- `conservative_first_start_preset` derives required sensors, injector count, ignition count, auxiliary outputs, timing authority, and sync-loss cut requirements from any `EngineBoardProfile`.
- `ecu_board_api::BoardCapabilities` is the canonical recipe-level board capability source for trigger/cam inputs, load-source support, and output counts.
- `BoardFirstStartCapabilities` is a derived first-run/evidence overlay seeded from `BoardCapabilities`; first-start-only evidence such as CLT/IAT/VBATT/VSS/knock/baro, auxiliary output roles, and safety authority stays explicit.
- `check_profile_board_compatibility` compares a profile preset with board capabilities and returns a fixed-size `ProfileCompatibilityReport`.

This gate is software-only. It proves that a board application has enough declared software surface for a profile; it does not certify wiring, sensor installation, pressure-sensor scaling, timing-light evidence, or other bench artifacts.

## M50B25TU Profile Note

`M50B25TU_MEGA_COMPAT` keeps `trigger.trigger_angle_atdc_deg10` at `Unknown`. That means the profile is not certified for absolute tooth-1 timing yet, even though the 60-2 plus cam geometry is enough to show the sync shape.

`M50B25TU_FULL_COP` inherits that same timing authority state. It only changes the ignition topology.

The hardware bindings in this crate are a symbolic compatibility sketch, not a pin-verified production harness map.

## M50 Sensor Evidence Boundary

The M50 profile can name required sensor roles, but it does not certify installed hardware.

`sensor_inventory` separates factory-present signals from board-added or derived pressure signals. For the M50B25TU profile, crank, cam, TPS, CLT, IAT, HFM/MAF, and knock are modeled as factory engine sensors; VBatt, lambda, and VSS are harness/chassis inputs; MAP is board-added for speed-density first-run; and baro is a policy-derived input unless a dedicated second pressure sensor is wired.

Use `tools/init_m50_batch8_evidence.py` and `tools/check_m50_batch8_evidence.py` to create and verify the local evidence package before treating a bench setup as first-run ready. The checker requires:

- First-run load source, explicitly one of `map_speed_density`, `maf`, or `tps_alpha_n`.
- Sensor IO map entries with board input path, signal-conditioning path, and routing source for crank, cam, required core sensors, and installed optional sensors.
- ADC reference voltage, resolution, and source evidence.
- MAP model, model source, voltage scale, and scale source when MAP is installed or used for first-run load/baro. The M50 board profile names the board-supported `mpxh6400ac6u` and `mpx5700ap` options; the checker also accepts the broader `mpxh6400a` transfer-function family when the evidence source uses that name.
- TPS closed/open ADC counts with open greater than closed, plus calibration source.
- CLT and IAT curve source plus board bias resistor value/source. Verified CLT/IAT status requires monotonic temperature/resistance curve points.
- HFM/MAF explicit runtime-load support status. If MAF runtime-load is bench verified, curve source plus ADC-input voltage and flow points are required, must be monotonic, and the voltage points must fit the declared ADC reference.
- Baro source policy, explicitly fixed pressure, startup MAP sample, or dedicated sensor. Dedicated baro requires its own IO map entry, a supported pressure sensor model (`mpxh6400ac6u`, `mpxh6400a`, `mpx5700ap`, or `generic_linear_0v5_4v5`), voltage-to-pressure endpoints, calibration source, and first-run-ready mode requires verified dedicated-baro calibration.
- VBatt divider/source scale.
- Lambda installation status, controller type, voltage-to-lambda endpoints, and calibration status when installed or required by a first-run policy. If first-run policy requires lambda, it must also be marked installed.
- Knock sensor count and explicit authority status. Per-channel IO routing, front-end, cylinder coverage, window source, and threshold source are required for the two M50 knock sensors when monitor/retard authority is enabled. Knock retard authority needs a validation source in first-run-ready mode.
- VSS installation status, pulse source, pulses-per-km, and calibration status when installed or required by a first-run policy. If first-run policy requires VSS, it must also be marked installed.
- Cam phase reference tooth/window source and edge action for the generic `CamPhaseConfig`. The M50 profile uses a single reference cam pulse that sets a known phase rather than toggling phase every cycle.
- Dry-crank crank/cam capture metadata with spark and injectors disabled.

If `first_run_load_source` is `map_speed_density`, the MAP installation must be explicitly confirmed. If it is `maf`, the MAF runtime-load path must be bench verified. If it is `tps_alpha_n`, TPS calibration cannot remain `not_yet_certified`.

Use `--require-first-run-ready` when reviewing a package intended to start hardware. That stricter mode also requires TPS, CLT, and IAT calibration status to be `bench_verified` or `installed_verified`, and requires lambda calibration if lambda is marked as required for first run.

Passing the checker only means the package is structurally reviewable. It does not certify trigger angle, sensor curves, knock authority, or pin-level hardware wiring.

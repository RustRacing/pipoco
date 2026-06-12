# ecu-sim-hifi

Host-only `f64` high-fidelity plant crate for the ADR 0009 execution plan.

- Authority: `aidocs/architecture/adr-0009-f64-hifi-plant.md`
- Execution plan: `aidocs/010_hifi-plant-detailed-execution-plan.md`
- Model reference: `aidocs/simulator-research-v2.md`

## Boundaries

- Internal math is SI `f64` throughout: `Pa`, `K`, `kg`, `m`, `rad`, `s`.
- Crank angle is radians internally. Degrees are only for config or adapter
  boundaries.
- `ecu-sim-hifi` must not depend on `sim/driver`.
- Quantization belongs at adapter/export boundaries only.

## Scope

This crate is the host-only oracle and artifact generator described by ADR 0009.
It does not weaken `sim/core`'s `no_std`, integer, deterministic contract.
The runtime default config derives from source defaults inside `sim/hifi`; the
committed artifacts are consumed by `sim/core` and the core-side conformance
path, not by the hifi runtime path.

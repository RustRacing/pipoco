# TLA+ Verification Tooling

## Tool Versions (Pinned)

| Tool | Version | Path |
|------|---------|------|
| Java (OpenJDK) | 21.0.10 (2026-01-20) | `java` |
| TLA+ toolbox (tlc2.TLC) | 2.19 | `/home/user/.local/lib/tla/tla2tools.jar` |

## TLC Results

## 2026-04-24 US-FM0426 TLC v5 scheduler alignment

- Java:
  - `openjdk version "21.0.10" 2026-01-20`
  - `OpenJDK Runtime Environment (build 21.0.10+7-Debian-1deb13u1)`
  - `OpenJDK 64-Bit Server VM (build 21.0.10+7-Debian-1deb13u1, mixed mode, sharing)`
- Command:
  - `java -cp /home/user/.local/lib/tla/tla2tools.jar tlc2.TLC -config ecu-spec/tla/scheduler.cfg ecu-spec/tla/scheduler.tla`
- Result:
  - Exit code `0`
  - States generated: `4,822,273`
  - Distinct states: `64,512`
  - Queue remaining: `0`
  - Depth: `17`
  - Duration: `53s`
  - Final status: `Model checking completed. No error has been found.`
- Scope checked from config:
  - `SPECIFICATION Spec`
  - `INVARIANT Inv`
  - `PROPERTY ConditionalLiveness`

## 2026-04-23 US-FM0109 Results

Story: `US-FM0109`

- Command:
  - `java -cp /home/user/.local/lib/tla/tla2tools.jar tlc2.TLC -config ecu-spec/tla/scheduler.cfg ecu-spec/tla/scheduler.tla`
- Result:
  - Exit code `0`
  - Temporal checking branches: `4`
  - States generated: `33,091,297`
  - Distinct states: `615,552`
  - Queue remaining: `0`
  - Depth: `17`
  - Duration: `03min 16s`
  - Final status: `Model checking completed. No error has been found.`
- Scope checked from config:
  - `SPECIFICATION Spec`
  - `INVARIANT Inv`
  - `PROPERTY ConditionalLiveness`

## 2026-04-23 US-FM0260 Trigger TLC liveness+safety run

- Command:
  - `java -cp /home/user/.local/lib/tla/tla2tools.jar tlc2.TLC -config ecu-spec/tla/trigger.cfg ecu-spec/tla/trigger.tla`
- Result:
  - Exit code `0`
  - Temporal checking branches: `1`
  - States generated: `86`
  - Distinct states: `25`
  - Queue remaining: `0`
  - Depth: `11`
  - Duration: `00s`
  - Final status: `Model checking completed. No error has been found.`
- Scope checked from config:
  - `SPECIFICATION Spec`
  - `INVARIANT Inv`
  - `PROPERTY ConditionalLiveness`

## 2026-04-24 US-FM0261 Scheduler liveness+safety TLC run

- Command:
  - `java -cp /home/user/.local/lib/tla/tla2tools.jar tlc2.TLC -config ecu-spec/tla/scheduler.cfg ecu-spec/tla/scheduler.tla`
- Result:
  - Exit code `0`
  - Temporal checking branches: `5`
  - States generated: `4,822,273`
  - Distinct states: `64,512`
  - Queue remaining: `0`
  - Depth: `17`
  - Duration: `53s`
  - Final status: `Model checking completed. No error has been found.`
- Scope checked from config:
  - `SPECIFICATION Spec`
  - `INVARIANT Inv`
  - `PROPERTY ConditionalLiveness`
  - `ConditionalLiveness` includes enabled-cylinder injection and spark event emission over the bounded output-set abstraction.

## 2026-04-23 US-FM0263 Persistence burn/save TLC run

- Command:
  - `java -cp /home/user/.local/lib/tla/tla2tools.jar tlc2.TLC -config ecu-spec/tla/persist.cfg ecu-spec/tla/persist.tla`
- Result:
  - Exit code `0`
  - Temporal checking branches: `1`
  - States generated: `1,134`
  - Distinct states: `132`
  - Queue remaining: `0`
  - Depth: `9`
  - Duration: `00s`
  - Final status: `Model checking completed. No error has been found.`
- Scope checked from config:
  - `SPECIFICATION Spec`
  - `INVARIANT Inv`
  - `PROPERTY ConditionalLiveness`

## 2026-04-23 US-FM0265 Safety latching TLC run

- Command:
  - `java -cp /home/user/.local/lib/tla/tla2tools.jar tlc2.TLC -config ecu-spec/tla/safety.cfg ecu-spec/tla/safety.tla`
- Result:
  - Exit code `0`
  - Temporal checking branches: `1`
  - States generated: `161`
  - Distinct states: `16`
  - Queue remaining: `0`
  - Depth: `4`
  - Duration: `00s`
  - Final status: `Model checking completed. No error has been found.`
- Scope checked from config:
  - `SPECIFICATION Spec`
  - `INVARIANT Inv`
  - `PROPERTY ConditionalLiveness`

## 2026-04-23 US-FM0266 TLA module TLC results roll-up

- scheduler (`ecu-spec/tla/scheduler.tla`)
  - Command: `java -cp /home/user/.local/lib/tla/tla2tools.jar tlc2.TLC -config ecu-spec/tla/scheduler.cfg ecu-spec/tla/scheduler.tla`
  - States generated: `4,822,273`
  - Distinct states: `64,512`
  - Depth: `17`
  - Scope: `SPECIFICATION Spec`, `INVARIANT Inv`, `PROPERTY ConditionalLiveness`
  - Zero-error result: `Model checking completed. No error has been found.`
- trigger (`ecu-spec/tla/trigger.tla`)
  - Command: `java -cp /home/user/.local/lib/tla/tla2tools.jar tlc2.TLC -config ecu-spec/tla/trigger.cfg ecu-spec/tla/trigger.tla`
  - States generated: `86`
  - Distinct states: `25`
  - Depth: `11`
  - Zero-error result: `Model checking completed. No error has been found.`
- persist (`ecu-spec/tla/persist.tla`)
  - Command: `java -cp /home/user/.local/lib/tla/tla2tools.jar tlc2.TLC -config ecu-spec/tla/persist.cfg ecu-spec/tla/persist.tla`
  - States generated: `1,134`
  - Distinct states: `132`
  - Depth: `9`
  - Zero-error result: `Model checking completed. No error has been found.`
- safety (`ecu-spec/tla/safety.tla`)
  - Command: `java -cp /home/user/.local/lib/tla/tla2tools.jar tlc2.TLC -config ecu-spec/tla/safety.cfg ecu-spec/tla/safety.tla`
  - States generated: `161`
  - Distinct states: `16`
  - Depth: `4`
  - Zero-error result: `Model checking completed. No error has been found.`
- arbiter (`ecu-spec/tla/arbiter.tla`)
  - Command: `java -cp /home/user/.local/lib/tla/tla2tools.jar tlc2.TLC -config ecu-spec/tla/arbiter.cfg ecu-spec/tla/arbiter.tla`
  - States generated: `16,385`
  - Distinct states: `128`
  - Depth: `2`
  - Zero-error result: `Model checking completed. No error has been found.`
- ts_proto (`ecu-spec/tla/ts_proto.tla`)
  - Command: `java -cp /home/user/.local/lib/tla/tla2tools.jar tlc2.TLC -config ecu-spec/tla/ts_proto.cfg ecu-spec/tla/ts_proto.tla`
  - States generated: `62`
  - Distinct states: `25`
  - Depth: `5`
  - Zero-error result: `Model checking completed. No error has been found.`

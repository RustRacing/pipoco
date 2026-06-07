# ecu-sim-driver

`ecu-sim-driver` provides deterministic host-side replay fixtures for trigger decoding and simulator integration tests.

## Software Readiness Aggregation

`readiness` contains generic report types that aggregate:

- profile-to-target compatibility from `ecu-board-profiles`
- simulator evidence such as sync, output scheduling, fault cuts, and deterministic replay
- TunerStudio page validity
- metadata/evidence tooling status

The report does not certify hardware. It is the software-side answer to
"can this profile be exercised end-to-end by the current app/simulator path?"

## Trigger Replay

`TriggerReplayFixture` builds synthetic crank and cam edge streams, and `replay_trigger_edges` turns those streams into a `TriggerReplay`.

Each `TriggerReplayFrame` is a per-edge record that can later drive a tooth or composite log viewer without re-running the decoder. The current replay fields are:

- `index`
- `edge`
- `decoder_event`
- `sync_loss`
- `diagnostics`
- `unsupported_cam_edge`

The replay frame also preserves the timebase through `replay_frame_time_us`, so later viewer code can align the frame with tooth timing or composite traces directly. Viewer export schema versioning should live in the viewer/export layer, not in the generic simulator driver API.

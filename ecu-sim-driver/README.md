# ecu-sim-driver

`ecu-sim-driver` provides deterministic host-side replay fixtures for trigger decoding and simulator integration tests.

## Trigger Replay

`TriggerReplayFixture` builds synthetic crank and cam edge streams, and `replay_trigger_edges` turns those streams into a `TriggerReplay`.

Each `TriggerReplayFrame` is a stable per-edge record that can later drive a tooth or composite log viewer without re-running the decoder. The current viewer-facing schema is versioned and field-stable:

- `index`
- `edge`
- `decoder_event`
- `sync_loss`
- `diagnostics`
- `unsupported_cam_edge`

The replay frame also preserves the timebase through `replay_frame_time_us`, so later viewer code can align the frame with tooth timing or composite traces directly.

The `TriggerReplayFrame::VIEWER_SCHEMA_VERSION` and `TriggerReplayFrame::VIEWER_FIELD_NAMES` constants are the public contract for that export shape.

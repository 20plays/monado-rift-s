# Rift S controller tracking follow-up

This note deliberately separates controller investigation from the camera-clock
recovery change.

## What the timestamp fix can and cannot address

Rift S SLAM exposures and controller-constellation exposures come from the same
camera metadata path. Both previously used the same unsafe camera-to-monotonic
conversion. A bad camera epoch therefore gave the constellation tracker stale
capture times, a stale HMD camera pose, and stale controller priors. Correcting
that clock can plausibly improve optical pose matching and latency.

It cannot explain a Touch controller debug panel showing zero gyro,
accelerometer, and timestamp while button reports continue. Button and IMU
blocks are parsed independently from radio reports. That symptom points to an
additional controller report, initialization, or radio-state problem.

## Focused investigation plan

1. Add per-controller counters in `rift_s_controller.c` for radio reports, button
   blocks, IMU blocks, repeated IMU timestamps, calibration/config readiness,
   and time since the last valid IMU sample. Rate-limit a warning when buttons
   advance but no valid IMU block has arrived for a defined interval.
2. Trace controller creation and restart ordering through `rift_s_radio.c`,
   `rift_s.c`, and `rift_s_controller_create()`. Record device ID, handedness,
   calibration/config completion, `imu_time_valid`, and the first accepted IMU
   timestamp. Verify that reconnecting an existing radio device resets all
   timestamp-extension and fusion state.
3. Exercise 32-bit controller IMU wrap/repeat handling in `handle_imu_update()`.
   Its `uint32_t` subtraction intentionally handles wrap, while deltas of zero
   and greater than half an epoch are dropped. Add synthetic tests for repeated,
   wrapped, stale, and restarted controller counters before changing it.
4. Implement the currently unused constellation
   `notify_frame_received` callback for diagnostics only. Compare camera capture
   time, controller IMU device/local time, last optical pose time, and processing
   latency. This will show whether an optical freeze begins before or after the
   constellation tracker receives a frame.
5. Inspect `t_constellation_tracking.c` fast/full search timing and outcomes:
   blob counts, prior-pose match, last-seen-pose match, recovery search, matched
   LEDs, and `last_seen_pose_ts`. Correlate failures with exposure/brightness and
   camera processing time before changing filters.
6. Audit fusion semantics in `rift_s_controller_get_tracked_pose()` and
   `rift_s_controller_push_observed_pose()`. Position is the last optical sample;
   orientation is current 3DoF fusion with only ad-hoc optical yaw correction.
   Quantify timestamp age and error before proposing prediction, interpolation,
   or a proper pose filter.

The next experimental branch should begin only after the camera timestamp test
matrix is clean. Keep radio/IMU recovery and constellation pose filtering as
separate commits so a hardware regression can be bisected.

## Future hybrid Touch input plus Mercury pose

A realistic hybrid device would keep each physical Touch controller as the
input/output target and substitute only its grip/aim tracking relation:

```text
Touch buttons, triggers, sticks, haptics ──> hybrid controller wrapper
Mercury hand joints ──> hand-pose adapter ──> calibrated hand-to-grip pose
                                      └────> grip/aim relation returned by wrapper
```

Relevant existing pieces are:

- `src/xrt/targets/common/target_builder_rift_s.c`: creates physical Touch
  devices, Mercury, and the current gesture-based emulated controllers.
- `src/xrt/drivers/ht_ctrl_emu/ht_ctrl_emu.cpp`: derives grip and aim poses from
  palm/finger joints, but currently synthesizes controller inputs rather than
  preserving physical Touch input.
- `src/xrt/drivers/multi_wrapper/multi.c`: demonstrates a wrapper that mimics a
  target device, forwards `update_inputs` and `set_output`, and replaces tracking
  with another device plus a fixed pose offset.
- `src/xrt/drivers/multi_wrapper/multi.h` and `xrt_settings.h`: tracking override
  API and direct/attached modes.
- `src/xrt/auxiliary/util/u_config_json.c`: persistence format for tracking
  overrides.

Proposed separate implementation:

1. Refactor the hand-to-grip/aim calculation from `ht_ctrl_emu` into a reusable
   pose provider for a selected Mercury hand.
2. Create one pose adapter per hand that exposes tracked grip and aim relations
   at the hand sample timestamp, without synthesizing buttons.
3. Wrap each real Touch device so `update_inputs`, input storage, and `set_output`
   still come from Touch, while grip/aim pose queries come from the corresponding
   hand adapter. `multi_create_tracking_override()` is a useful prototype, but a
   dedicated wrapper is likely needed because grip and aim need separate
   calibrated transforms and explicit lifetime ownership.
4. Calibrate a rigid hand-to-controller grip transform per hand. Store position
   and orientation in Monado configuration, expose live adjustment in the debug
   UI, and validate it across different hand sizes and grip styles.
5. Define loss behavior explicitly: when Mercury is inactive or occluded, mark
   pose invalid or transition to the physical controller pose only with a
   measured continuity transform. Never silently freeze a stale hand pose.
6. Test timestamp alignment, button latency, haptics, hand occlusion, one-hand
   loss, and transitions in `hello_xr` before testing games.

This design is feasible with current Monado device/wrapper APIs, but it belongs
on a separate branch after physical validation of both Mercury pose stability
and the camera timestamp recovery.

# Rift S camera timestamp recovery: hardware test plan

This branch is based on the exact Envision Monado revision that exhibited the
camera timestamp failure. Synthetic tests and a local build pass, but the change
must not be considered hardware-verified until the following checks succeed on a
physical Rift S.

```text
Base branch: dev-constellation-controller-tracking
Base commit: b1c850a015d2d3776414b31168e8f135990c0d8f
Test branch: fix/rift-s-camera-timestamp-resync
```

## Build through Envision

After pushing the test branch to an authenticated fork or mirror, set these in
the editable Envision Development Profile:

```text
XR Service Repository: <URL of the authenticated fork or mirror>
XR Service Branch: fix/rift-s-camera-timestamp-resync
```

Keep the profile's existing Basalt and Mercury settings. If Basalt compilation
fails under parallel load, close Envision and start it for the build only with:

```sh
taskset -c 0 envision
```

Do not pin `monado-service`, `hello_xr`, Steam, xrizer, or a game to one CPU.

## Capture setup

The profile inspected while preparing this branch uses this prefix:

```sh
export RIFT_S_TEST_PREFIX="$HOME/.local/share/envision/prefixes/d431e409-d756-47e3-bbab-886632ba13b9"
export RIFT_S_TEST_LOG_DIR="$HOME/rift-s-timestamp-logs/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$RIFT_S_TEST_LOG_DIR"
```

Stop the running XR service with Envision before launching the instrumented
service below. Leave the Rift S connected. The optional timestamp logger emits
one SLAM and one controller-clock sample per second; normal operation is
otherwise unchanged.

```sh
env \
  LD_LIBRARY_PATH="$RIFT_S_TEST_PREFIX/lib:$RIFT_S_TEST_PREFIX/lib64" \
  RIFT_S_LOG=info \
  RIFT_S_TIMESTAMP_LOG=1 \
  AEG_USE_DYNAMIC_RANGE=true \
  SLAM_UI=1 \
  XRT_DEBUG_GUI=1 \
  "$RIFT_S_TEST_PREFIX/bin/monado-service" \
  2>&1 | tee "$RIFT_S_TEST_LOG_DIR/monado-service.log"
```

Important messages have `camera timestamp:` or `camera timestamp abnormal:`.
An automatic recovery is explicitly marked `resync=yes`. Preserve the full log,
not only matching lines, but this summary is convenient:

```sh
grep -E 'clock map stabilised|camera timestamp|time went backward|Basalt|SLAM|Controller' \
  "$RIFT_S_TEST_LOG_DIR/monado-service.log" \
  | tee "$RIFT_S_TEST_LOG_DIR/timestamp-summary.log"
```

In a second terminal, record device discovery once per test build:

```sh
env LD_LIBRARY_PATH="$RIFT_S_TEST_PREFIX/lib:$RIFT_S_TEST_PREFIX/lib64" \
  "$RIFT_S_TEST_PREFIX/bin/monado-cli" info \
  2>&1 | tee "$RIFT_S_TEST_LOG_DIR/monado-info.log"
```

## Test matrix

Run at least ten cold service starts and ten warm restarts. For each start, note
whether HMD position begins tracking, time to first stable 6DoF pose, whether a
timestamp resync appeared, and whether any repeated backward-time warning
continues for more than one second.

1. **Good HMD startup:** Start the instrumented service, leave both controllers
   asleep, and move the headset through translation and rotation for 60 seconds.
   Save the log as `good-hmd-startup.log`.
2. **Formerly bad startup:** Repeat service restarts until a resync is observed or
   the old failure would normally have appeared. Continue moving the HMD for two
   minutes. Recovery is successful only if camera frames resume, Basalt produces
   a tracked pose, and warnings do not repeat indefinitely. Save as
   `recovered-hmd-startup.log`.
3. **hello_xr:** With the service running, execute:

   ```sh
   env XR_RUNTIME_JSON="$RIFT_S_TEST_PREFIX/share/openxr/1/openxr_monado.json" \
     hello_xr -g Vulkan2 -ff Hmd -vc Stereo -s Local -v \
     2>&1 | tee "$RIFT_S_TEST_LOG_DIR/hello-xr.log"
   ```

   Walk and rotate for two minutes. Confirm smooth orientation and translation.
4. **Controllers off:** Let both Touch controllers sleep. Confirm HMD tracking
   remains stable for two minutes and record the SLAM timestamp samples.
5. **Controllers on:** Wake one controller, then the other. Confirm neither action
   disrupts HMD timestamp continuity.
6. **Controller optical tracking:** Move one controller at a time through each
   camera's field of view, then both together. Record choppiness, frozen pose,
   button responsiveness, and whether `controller camera timestamp` latency or
   discontinuities coincide with the problem.
7. **Half-Life: Alyx through xrizer:** Start the game through the existing
   Envision/xrizer setup while the instrumented service is still logging. Test
   ten minutes of HMD translation, turning, both controllers, buttons, and
   haptics. Save the service log as `alyx-xrizer.log`.

## Pass/fail criteria

- A `~647 s` camera reset may cause one confirmation frame to be dropped, then
  must produce `resync=yes` and resume monotonic frames.
- No startup may enter an endless stream of backward-camera-time warnings.
- Converted frames must not jump far into the future; `processing_latency`
  should remain plausible and must not become a large negative value.
- HMD SLAM must acquire and retain 6DoF across the restart matrix.
- Controller optical frame timestamps must remain monotonic. Controller pose
  quality is recorded separately because remaining constellation/IMU issues are
  outside this fix.

If a failure remains, attach the entire test directory plus the count of warm and
cold starts attempted. The raw/adjusted/IMU/monotonic values in the abnormal log
are sufficient to refine the threshold or timestamp-domain model without a
speculative controller rewrite.

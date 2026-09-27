# Oculus Rift S Tracking — Exhaustive Agent Handoff

**Date**: 2026-09-27
**Branch**: `fix/rift-s-camera-timestamp-resync`
**Remote**: `https://github.com/20plays/monado-rift-s.git`
**Head at handoff**: `4aef42868` (all pushed, prefix installed from it, 24/24 tests green)
**Status**: User has STOPPED the project. Tracking never reached Windows parity.
**Goal (unmet)**: Rift S controller tracking on-parity with Windows.

---

## 1. Hardware / software stack

- Oculus Rift S HMD + Left/Right Touch controllers, USB 3.0 + DisplayPort.
- Constellation optical tracking: 5 onboard mono cameras (~60 Hz alternating
  SLAM/controller mosaics → ~30 Hz per stream) + Touch IMU @ 500 Hz over the
  HMD radio link + per-controller factory JSON calibration (LED positions,
  IMU position/scales/rectification/biases, fetched over radio at connect).
- Monado (OpenXR runtime + `rift_s` driver), SteamVR via `driver_monado.so`
  built from `src/xrt/state_trackers/steamvr_drv/`, Envision profile manager,
  primary test games: Beat Saber (OpenXR via Wine/Proton + WineOpenXR thunk),
  Half-Life: Alyx (SteamVR).

## 2. Workspaces & strict sync rule

1. Dev repo: `/home/sprite/Documents/Codex/monado-rift-s` (branch above,
   build dir `build-dev/`).
   - Build: `ninja -C .../build-dev`
   - Test: `ctest --test-dir .../build-dev -E tests_quatexpmap` (must be 24/24;
     `tests_quatexpmap` is upstream-broken, always excluded).
2. Envision source: `/home/sprite/.local/share/envision/d431e409-d756-47e3-bbab-886632ba13b9/xrservice`
   (same branch). Sync: `git pull /home/sprite/Documents/Codex/monado-rift-s fix/rift-s-camera-timestamp-resync`.
3. Envision prefix: `/home/sprite/.local/share/envision/prefixes/d431e409-d756-47e3-bbab-886632ba13b9`
   - Install: `ninja -C .../xrservice/build install`
   - Binaries: `bin/monado-service`,
     `share/steamvr-monado/bin/linux64/driver_monado.so`.
4. Push: `git push origin fix/rift-s-camera-timestamp-resync`.

Envision profile UUID `d431e409-d756-47e3-bbab-886632ba13b9`, config in
`~/.config/envision/envision.json`. Profile `environment` currently contains
`RIFT_S_LOG=debug` and `RIFT_S_DIAG_FILE=/home/sprite/oculuslog.txt`
(user added these via the profile editor — do not remove).

## 3. Environment lessons (hard-won, do not re-derive)

- Manual `monado-service` launch needs the FULL profile env (LD_LIBRARY_PATH,
  XRT_* flags from `envision.json`) **plus `XRT_NO_STDIN=1`**, or it dies
  instantly (`epoll_ctl(stdin) failed`). A helper script exists:
  `/home/sprite/monado-manual.sh` (stops Envision service, re-registers
  runtime, launches with logging to `~/oculuslog.txt`).
- Envision creates `~/.config/openxr/1/active_runtime.json` (symlink to the
  prefix `share/openxr/1/openxr_monado.json`) on service Start and removes it
  on Stop. Without it, Proton games report "no openxr runtime configured".
  Steam `XR_RUNTIME_JSON` launch options do NOT reliably reach the Wine
  loader; prefer the active_runtime symlink.
- Game chain for Beat Saber: game → Wine registry
  `HKLM\SOFTWARE\Khronos\OpenXR\1\ActiveRuntime` =
  `C:\openxr\wineopenxr64.json` (WineOpenXR thunk) → Linux OpenXR loader →
  active_runtime.json → prefix `libopenxr_monado.so` → socket
  `/run/user/1000/monado_comp_ipc` → monado-service.
- Envision's Debug View log buffer truncates long sessions (only the tail
  survives copy). RIFT_S_DIAG_FILE logging was added so sessions record to
  disk with zero user effort. Do not rely on Debug View copies.
- `/usr/bin/monado-service` + `/usr/share/openxr/1/openxr_monado.json` are a
  SEPARATE system Monado (installed Sep 14) — not this project. Don't confuse
  the two.

## 4. Verified architecture facts (evidence-backed, keep)

- Relation chain (`m_space.cpp`): resolves innermost-child-first;
  `r = steps[n-1] × … × steps[0]`. Controller chain order child→parent is
  correct as-is (`flip` → `P_aim_grip` → `P_imu_device` → fusion rel).
  SteamVR driver pushes child `rel` before parent `offset` (lever-arm fix).
- SteamVR Touch path requests `XRT_INPUT_TOUCH_AIM_POSE` (body frame) — this
  is CORRECT: Valve's `oculus_rifts_controller_left.json` puts `body` at
  origin identity, and Valve's `openxr_grip` attachment (+20.6° pitch,
  10.2 cm) exactly matches Monado's `P_aim_grip`. Do not re-add aim offsets.
- `RIFT_S_FLOOR_OFFSET_Y` default `0.8f` (was 1.6f → giant-player bug).
- Blob thresholds `BLOB_PIXEL_THRESHOLD=0x60` / `BLOD_THRESHOLD_MIN=0x80` in
  `rift_s_tracker.c` are INVARIANT (prior agent rule; loosening was tried and
  reverted). Touch only with data.
- `math_quat_rotate(l, r)` = l × r (Eigen). `math_quat_normalize` on a zero
  quat yields NaN (Eigen, no guard) — never feed it near-zero quats.
- `fusion.rot` maps body→world (confirmed in `m_imu_3dof.c`: `world_accel =
  rot * accel`; at rest world accel ≈ +9.81 Y).
- Controller IMU rate is exactly 500 Hz (measured). Optical controller
  stream is 30 Hz nominal with frequent 100–800 ms total losses in real play.
- `xrt_quat` field order is `{x, y, z, w}`.

## 5. Calibration facts (user's hardware, from DIAG logs)

- Both controllers: `accelScale=0.000977` (1/1024), `gyroScale=0.122070`
  (1/8192), 500 Hz, 15 LEDs each, LED clouds mirrored correctly in X
  (L centroid x=+0.0016, R −0.0016; extents x ±0.043, y −0.004…+0.071,
  z −0.015…+0.036 — plausible halo geometry, no mirror bug indicated).
- L: `accelOff=(-0.0696,0.0133,0.3318)`, `gyroOff=(-0.0024,-0.0296,-0.0109)`.
- R: `accelOff=(0.0062,-0.0415,0.1236)`, `gyroOff=(0.0059,0.0099,0.0032)`.
- L gyro factory bias (−1.7°/s on Y) is compensated by calibration; residual
  drift is slow. `P_imu_device` ≈ pure translation (±0.0056, 0.0071, −0.0199).

## 6. Proven failure mechanisms (log evidence for each)

1. **NaN poisoning**: non-finite PnP solves entered the One-Euro filter
   (divides by dt; dt=0 on repeated ts) → permanent NaN position/velocity
   (`pos=(-nan)`, `spd=-nan` for whole windows). Sources found: unchecked
   `solvePnPRansac` failure + `1/angle` on zero rotation (fixed in
   `internal/ransac_pnp.cpp`), zero-match scoring div-by-zero (fixed in
   `internal/pose_metrics.c`), brightness average div-by-zero (fixed in
   `t_constellation_tracking.c`). Fenced at controller input too
   (`NONFINITE`/`stale` drops in `push_observed_pose`).
2. **180° optical yaw flips followed blindly**: still-hands 2%/frame nudge
   dragged fusion to flipped solves (`yawMaxDeg=180`, 40–60 applies/window).
   Now: 1.5 s trust delay + degenerate skip + 25° magnitude gate + stable
   consensus repair (6 still frames within 10° on a >25° error snaps once;
   oscillating flips can never build consensus).
3. **Hand swaps when close**: identical LED geometry on both hands; close
   hands cross-assign blobs (observed: L teleported to within 3 cm of R's
   live pose). Fenced by the continuity gate (raw observation vs
   anchor+velocity prediction; base 7 cm + 3·speed·dt; force-accept after 10
   with velocity dropped). Sub-7 cm theft passes invisibly by design.
4. **Starvation after loss**: recovery searched a 0.35 m box around stale
   priors → L starved at 3–11 Hz with constant gaps. Blanket 1.5 m widening
   caused cross-hand false GOOD matches parking BOTH hands (reverted).
   Current: sibling-aware pass-2 box (up to 1.5 m, never within 0.45 m of a
   fresh sibling pose; close/stale/unknown siblings keep old bounds).
5. **Choppiness**: 30 Hz steps + deadband + 35 ms cap + gap freezes. Current:
   decay coasting (τ 200 ms, horizon 250 ms, no deadband, decayed velocity
   reported to SteamVR). Numerically verified (exact linear follow at frame
   scale, saturates, eases to stop). No IMU integration (see §7.2).

## 7. Dead ends (do not retry without new evidence)

1. **IMU position predictor** (commit `6353d647c`, reverted `e9b6281e0`):
   velocity-poisoning cascade — optical finite-difference velocity fed an
   unbounded predictor while the gate scaled with that same velocity. Caused
   steady flyaway + wrong hands. Linear-only prediction + decay coasting is
   the approved pattern; never integrate unvalidated velocity again.
2. **Blanket recovery widening** (`1b78e13d6`, reverted `21a66c548`):
   rejection counts tripled on both hands; RANSAC fits sibling blobs and
   scores them GOOD. Use the sibling-aware box.
3. **Freshness-gated solver acceptance**: the score's prior-violating GOOD
   branch is LOAD-BEARING for fast motion (static priors + 0.10 m bound
   can't cover 4 m/s hands). Gating it breaks Beat Saber. Velocity-aware
   priors would be needed first (bigger redesign, needs hardware).
4. **Threshold/Euro/velocity retunes without data**: prior agent's tuning
   commit was reverted within 3 minutes. Keep `6.0/1.0/0.1` Euro, 4 m/s
   clamp, 0.35 alpha, 20 cm/50 ms teleport rule (gravity-glove proven).
5. **Steam launch-option runtime registration**: doesn't reach the Wine
   loader reliably. Use the active_runtime symlink.

## 8. Current code map (all on the branch head)

- `src/xrt/drivers/rift_s/rift_s_controller.c`
  - `rift_s_diag_file_log` + `CTRL_DIAG` macro (~line 51): dual log-view +
    `RIFT_S_DIAG_FILE` append, timestamped ms.
  - `handle_imu_update`: finite-sample guard for fusion input; stationary
    linear-accel diagnostic average (`diag_still_lin_accel_avg`, expect ~0).
  - `get_tracked_pose` (~line 618): anchor + decay coast (τ 0.2 s, cap
    250 ms), decayed velocity reported, max-age diagnostic.
  - `push_observed_pose` (~line 780): finite guard → stale guard →
    continuity gate (0.07 + 3·v·dt, force-accept drops velocity) → Euro
    filter → anchor → velocity (clamped, alpha 0.35) → teleport/gap resets
    → trust counter → gated nudge + consensus repair → 2 s DIAG summary.
  - `set_output` no-op (kills haptics error spam; haptics not driven).
  - One-time DIAG lines: `cfg` (poses), `imuCfg` (scales/offsets/rates),
    `leds` (cloud centroid/extents); events: `TELEPORT`, `GAP`, `REJECT`,
    `NONFINITE`; periodic `sum`.
- `src/xrt/drivers/rift_s/rift_s_controller.h`: `yaw_trust_count`,
  `optical_reject_count`, `yaw_consensus_{mean_deg,count}`, all `diag_*`.
- `src/xrt/drivers/rift_s/rift_s_camera.c`: per-frame counter log demoted
  DEBUG→TRACE (buffer hygiene, no behavior change).
- `src/xrt/tracking/constellation/t_constellation_tracking.c`: brightness
  div-zero guard; sibling-aware pass-2 box (locked sibling reads).
- `src/xrt/tracking/constellation/internal/ransac_pnp.cpp`: solver-failure
  bailout + degenerate-rotation guard (position still written on success).
- `src/xrt/tracking/constellation/internal/pose_metrics.c`: zero-match
  scoring guard.
- Shared-file changes affect WMR too; all are validation-only or
  behavior-preserving in the close/unknown cases by construction.

## 9. DIAG forensics cheat sheet

`DIAG L/R sum imuHz optHz maxAgeMs pos spd yawApply yawMove yawBad yawMaxDeg
tele gap nan stale rej stillLin` (every 2 s per hand):
- `imuHz` ≈ 500 healthy (first window inflated: counts predate window start).
- `optHz` ≈ 30 healthy; <15 = solver starving that hand.
- `maxAgeMs` = stalest pose SteamVR read (freshness; >300 = visible freeze).
- `yawApply/yawMove/yawBad/yawMaxDeg`: still-correction behavior; persistent
  `yawBad` with large `yawMaxDeg` = flipped solves being fenced.
- `tele/gap`: anchor jumps / tracking losses. `rej`: continuity rejections
  (high on both hands = cross-talk storm). `nan/stale` should be 0.
- `stillLin`: stationary |R·a−g|, expect <1.0; 10+ with still hands = broken
  attitude frame (transient at startup while yaw is dragged; converges).
- `spd=0.00` + gaps = velocity never builds → coasting can't help; the
  problem is upstream solve rate, not the controller.

## 10. Open problems, ranked

1. Sub-7 cm close-range theft (invisible by design) + reacquisition while
   sibling is CLOSE (box stays 0.35 m). True fix = joint blob assignment
   across devices (pipeline restructure; needs hardware validation).
2. Long-occlusion freezes (200–800 ms solver losses are routine in hard
   play). Coast covers 250 ms; beyond that needs either better detection or
   IMU bridging redone with hard bounds + offline proof (see §7.1).
3. Flip ambiguity is inherent to the near-symmetric LED ring at low visible
   counts; current policy (fence + repair) is correct, elimination probably
   requires more complete detection (exposure/AEG/blob work — thresholds
   invariant stands until data says otherwise).
4. Left-hand solve rate was systematically worse than right in two sessions
   (cause undetermined: dimmer emitters vs FOV vs label pathology). The
   per-solve brightness notifier exists in the tracker but is not surfaced;
   exposing per-hand brightness would discriminate hardware from algorithm.

## 11. Working agreement notes (user preferences, persist these)

- Factual, concise communication. No praise, no credit language, no
  superlatives, no emotional validation. State findings and actions only.
- User prefers terminal commands over UI navigation. Verify claims by
  execution; never ship behavioral guesses for them to test — validate
  offline (numeric simulation caught one real defect pre-ship).
- End-of-life state: user gave up, unhappy with outcomes. If work resumes,
  lead with a stable build and data, not process discussion.

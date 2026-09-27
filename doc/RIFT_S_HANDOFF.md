# Oculus Rift S Monado & SteamVR Driver — Agent Handoff Document

**Date**: 2026-09-26  
**Active Branch**: `fix/rift-s-camera-timestamp-resync`  
**GitHub Remote**: `https://github.com/20plays/monado-rift-s.git`  
**Latest Commit Pushed**: `83dc300b7` (*steamvr, rift_s: fix relation chain resolution order, remove aim_chain, and set 0.8m floor default*)

---

## 1. Executive Summary & Hardware Stack
This project maintains and fixes tracking, coordinate frames, kinematics, and responsiveness for the **Oculus Rift S** (HMD + Left/Right Touch Controllers) running natively on **Linux** using **Monado** and **SteamVR**.

- **HMD & Controllers**: Oculus Rift S (USB 3.0 + DisplayPort). Constellation optical tracking via 5 onboard monochrome cameras + Basalt visual-inertial SLAM + Touch controller IMU & IR LED constellations.
- **Software Stack**:
  - **Monado**: Open-source OpenXR runtime & XR device drivers.
  - **SteamVR Driver**: `driver_monado.so` (built from `src/xrt/state_trackers/steamvr_drv/`).
  - **Envision**: Linux GUI profile manager & launcher for Monado/OpenXR/SteamVR.
  - **Primary Testing Game**: *Half-Life: Alyx* (SteamVR).

---

## 2. Critical Workspaces & Synchronization Rules

There are **two repositories** and **one installation prefix** that must be kept strictly synchronized:

1. **Development Repository**:
   - Path: `/home/sprite/Documents/Codex/monado-rift-s`
   - Branch: `fix/rift-s-camera-timestamp-resync`
   - Upstream: `origin -> https://github.com/20plays/monado-rift-s.git`
   - Build directory: `build-dev/`
   - Compile: `ninja -C /home/sprite/Documents/Codex/monado-rift-s/build-dev`
   - Run tests: `ctest --test-dir /home/sprite/Documents/Codex/monado-rift-s/build-dev -E tests_quatexpmap` (All 24/24 tests must pass).

2. **Envision Source Repository**:
   - Path: `/home/sprite/.local/share/envision/d431e409-d756-47e3-bbab-886632ba13b9/xrservice`
   - Branch: `fix/rift-s-camera-timestamp-resync`
   - Remote: Points to `https://github.com/20plays/monado-rift-s.git` (or pull from dev repo).
   - Build directory: `build/`

3. **Envision Runtime Installation Prefix**:
   - Path: `/home/sprite/.local/share/envision/prefixes/d431e409-d756-47e3-bbab-886632ba13b9`
   - Binaries:
     - `bin/monado-service`
     - `share/steamvr-monado/bin/linux64/driver_monado.so`
   - Install command: `ninja -C /home/sprite/.local/share/envision/d431e409-d756-47e3-bbab-886632ba13b9/xrservice/build install`

> [!IMPORTANT]
> **Strict Synchronization Rule**:
> 1. Always edit code in `/home/sprite/Documents/Codex/monado-rift-s`.
> 2. Compile and test in `build-dev`.
> 3. Commit in dev repo.
> 4. In Envision repo, pull the commit: `git pull /home/sprite/Documents/Codex/monado-rift-s fix/rift-s-camera-timestamp-resync`.
> 5. Install to Envision prefix: `ninja -C .../xrservice/build install`.
> 6. Push to GitHub: `git push origin fix/rift-s-camera-timestamp-resync`.

---

## 3. Key Mathematical Discoveries & Architecture

### A. Monado Relation Chain Evaluation Order
In [`src/xrt/auxiliary/math/m_space.cpp`](file:///home/sprite/Documents/Codex/monado-rift-s/src/xrt/auxiliary/math/m_space.cpp), `m_relation_chain_resolve()` iterates through steps:
```cpp
struct xrt_space_relation r = xrc->steps[0];
for (uint32_t i = 1; i < xrc->step_count; i++) {
    apply_relation(&r, &xrc->steps[i], &r);
}
```
Inside `apply_relation(a, b, out)`:
- `body_pose = a` (the accumulating result)
- `base_pose = b` (the next step in the chain)
- `math_pose_transform(&base_pose, &body_pose, &pose)` is called.
- Per [`src/xrt/auxiliary/math/m_base.cpp`](file:///home/sprite/Documents/Codex/monado-rift-s/src/xrt/auxiliary/math/m_base.cpp), `math_pose_transform(T, P)` transforms child `P` into parent `T`:
  $$\mathbf{P}_{\text{out}} = \mathbf{R}_T \cdot \mathbf{P}_P + \mathbf{P}_T, \quad \mathbf{Q}_{\text{out}} = \mathbf{Q}_T \cdot \mathbf{Q}_P$$
- Therefore, the chain resolves in **reverse order**:
  $$\mathbf{r} = \mathbf{steps}[n-1] \times \dots \times \mathbf{steps}[1] \times \mathbf{steps}[0]$$
- **Rule**:
  - `steps[0]` is the **innermost child** (local body pose).
  - `steps[n-1]` is the **outermost parent** (tracking space or world frame).

### B. The 1.6m Lever Arm & Left Hand Flinging Bug
* **The Bug**:
  In `ovrd_driver.cpp`, `offset` (floor elevation $\sim 1.6\text{m}$) was pushed before `rel` (controller pose). This made `steps[0] = offset` and `steps[1] = rel`.
  The evaluation produced:
  $$\mathbf{P}_{\text{out}} = \mathbf{R}_{\text{rel}} \cdot \mathbf{P}_{\text{offset}} + \mathbf{P}_{\text{rel}}$$
  Rotating the controller revolved the hand around a $1.6\text{m}$ radius sphere. The cross product in `apply_relation` generated tangential velocity:
  $$\mathbf{v}_{\text{tangential}} = \boldsymbol{\omega}_{\text{rel}} \times (\mathbf{R}_{\text{rel}} \cdot \mathbf{P}_{\text{offset}}) \approx 10\text{ rad/s} \times 1.6\text{ m} \approx 16\text{ m/s}$$
  This caused the virtual hand to rocket off into space on wrist rotation.
* **The Fix** (Commit `83dc300b7`):
  Push child (`rel`) before parent (`offset`):
  ```cpp
  m_relation_chain_push_relation(&chain, &rel);                  // step 0: child
  m_relation_chain_push_pose_if_not_identity(&chain, offset);     // step 1: parent
  ```
  Result: $\mathbf{P}_{\text{out}} = \mathbf{P}_{\text{rel}} + \mathbf{P}_{\text{offset}}$, pure vertical elevation with **zero lever arm** and zero rotational velocity corruption.
  In [`rift_s_controller.c`](file:///home/sprite/Documents/Codex/monado-rift-s/src/xrt/drivers/rift_s/rift_s_controller.c), the local transforms were similarly ordered from child to parent:
  `push(flip)` -> `push(P_aim_grip)` -> `push(P_imu_device)` -> `reserve(rel)`.

### C. SteamVR Controller Orientation & Render Model
* **The Bug**:
  SteamVR's `oculus_rifts_controller_left.json` render model places `body.obj` directly at the driver pose origin $(0,0,0)$ with identity rotation.
  In `rift_s_controller.c`, `XRT_INPUT_TOUCH_AIM_POSE` already outputs the raw controller body origin $(0,0,0)$ with fusion orientation $\mathbf{R}$.
  An extra $+39.4^\circ$ pitch transform (`aim_chain`) in `ovrd_driver.cpp` double-rotated the controller, angling it $\approx 90^\circ$ incorrectly in SteamVR's main menu.
* **The Fix**:
  Removed `aim_chain` completely. The raw pose passed to SteamVR now directly matches the coordinate system expected by SteamVR's Touch controller render models.

### D. In-Game Player Height (Giant Player Bug)
* **The Bug**:
  Monado Basalt visual SLAM initializes $(0,0,0)$ when the headset is first tracked. In normal operation, the user starts Monado with the headset resting on a desk ($\sim 0.75\text{m}$ high). Adding a hardcoded $+1.6\text{m}$ floor offset set the virtual floor to $-2.35\text{m}$ below the desk. Standing up put player eye level at $\sim 2.5\text{–}2.7\text{m}$ ($>8.5\text{ feet}$ tall).
* **The Fix**:
  In [`src/xrt/targets/common/target_builder_rift_s.c`](file:///home/sprite/Documents/Codex/monado-rift-s/src/xrt/targets/common/target_builder_rift_s.c):
  Changed default `RIFT_S_FLOOR_OFFSET_Y` from `1.6f` to `0.8f`.
  - Desk ($0.75\text{m}$) + offset ($0.8\text{m}$) $\approx$ floor at physical room floor level ($0.0\text{m}$).
  - Standing player eye height $\approx 1.7\text{m}$ (normal human scale).
  - Can be adjusted via `RIFT_S_FLOOR_OFFSET_Y` environment variable or dynamically via `Floor Height (Y Offset)` in the Monado debug GUI (`u_var`).

### E. Gravity Glove Yanking in Half-Life: Alyx
* **Status**: **Working reliably** (confirmed by user).
* **Implementation**:
  In `rift_s_controller_push_observed_pose()` ([`src/xrt/drivers/rift_s/rift_s_controller.c`](file:///home/sprite/Documents/Codex/monado-rift-s/src/xrt/drivers/rift_s/rift_s_controller.c)), added optical jump rejection: if an optical observation jumps $>0.20\text{m}$ in $<50\text{ms}$ ($>4\text{m/s}$ teleport), `linear_velocity` and the position filter are reset. This eliminates runaway extrapolation spikes while preserving rapid human wrist flicks.

---

## 4. Invariant Constraints & Guidelines for Next Agent

1. **Blob Thresholds**:
   Do **NOT** alter the Rift S constellation optical blob thresholds in [`src/xrt/drivers/rift_s/rift_s_tracker.c`](file:///home/sprite/Documents/Codex/monado-rift-s/src/xrt/drivers/rift_s/rift_s_tracker.c):
   - `BLOB_PIXEL_THRESHOLD = 0x60`
   - `BLOB_THRESHOLD_MIN = 0x80`
   These thresholds were verified to provide optimal LED blob extraction without spurious noise.

2. **Unit Tests**:
   Before committing, always ensure `ctest --test-dir build-dev -E tests_quatexpmap` passes 100% (24/24 tests).
   *(Note: `tests_quatexpmap` is an upstream known broken test disabled by `-E`)*.

3. **Active Branch**:
   All work belongs on branch `fix/rift-s-camera-timestamp-resync`. Do not rebase or force push without reason.

---

## 5. Recent Git Commits Reference
- `837ebb86b`: `steamvr: fix Touch controller tracking origin, rendermodels, and left hand orientation`
- `0b9970926`: `rift_s: fix relation chain order, optical jump teleport rejection, and floor offset options`
- `83dc300b7`: `steamvr, rift_s: fix relation chain resolution order, remove aim_chain, and set 0.8m floor default`

---

## 6. Open Items / Next Steps for Incoming Agent
1. **User Testing Feedback on Commit `83dc300b7`**:
   - Verify that the left hand no longer rockets away on fast wrist rotation.
   - Verify that the SteamVR main menu controller angle and pointer laser align with the physical hands.
   - Verify that in-game eye height feels natural.
2. **If Hand Placement Relative to Real Life is Slightly Off**:
   - The calibrated translation and rotation of the IMU relative to the controller body are stored in `ctrl->P_imu_device`. Check JSON dumps in `scratch/calib_LEFT.json` and `scratch/calib_RIGHT.json` to confirm LED constellation coordinates match the physical chassis.
   - Check if any tracking drift occurs when controllers are briefly occluded by checking SLAM and constellation tracking confidence in Monado debug GUI (`XRT_DEBUG_GUI=1`).

// Copyright 2026, Monado contributors
// SPDX-License-Identifier: BSL-1.0

/*!
 * @file
 * @brief Rift S camera-to-IMU clock reconciliation.
 * @ingroup drv_rift_s
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Camera and IMU timestamps are microsecond counters. The HMD IMU transport
 * exposes 32 bits while the camera metadata has a 64-bit field, so independently
 * extended values can differ by whole 32-bit epochs. */
#define RIFT_S_TIMESTAMP_EPOCH_NS INT64_C(4294967296000)

/* A live USB camera frame and the latest HMD IMU clock observation should be
 * close. This intentionally leaves far more room than normal USB/frame latency. */
#define RIFT_S_CAMERA_IMU_MAX_SKEW_NS INT64_C(1000000000)

enum rift_s_clock_sync_result
{
	RIFT_S_CLOCK_SYNC_OK,
	RIFT_S_CLOCK_SYNC_WAITING_FOR_CONFIRMATION,
	RIFT_S_CLOCK_SYNC_RESYNCED,
	RIFT_S_CLOCK_SYNC_INVALID,
};

struct rift_s_clock_sync
{
	bool have_offset;
	int64_t camera_ts_offset_ns;

	bool have_good_frame_to_imu;
	int64_t good_frame_to_imu_ns;

	bool have_pending;
	int64_t pending_adjusted_camera_ns;
	int64_t pending_imu_ns;
};

struct rift_s_clock_sync_sample
{
	uint64_t raw_camera_ns;
	uint64_t imu_ns;
	int64_t adjusted_camera_ns;
	int64_t frame_to_imu_ns;
	int64_t camera_ts_offset_ns;
	bool epoch_adjusted;
	bool resynchronized;
};

/*!
 * Reconcile a camera metadata timestamp with the latest timestamp observed on
 * the HMD IMU clock. Two coherently advancing out-of-range samples are required
 * before an arbitrary camera reset is rebased, preventing one corrupt metadata
 * row from poisoning the mapping.
 */
enum rift_s_clock_sync_result
rift_s_clock_sync_push(struct rift_s_clock_sync *sync,
                       uint64_t raw_camera_ns,
                       uint64_t imu_ns,
                       struct rift_s_clock_sync_sample *out_sample);

#ifdef __cplusplus
}
#endif

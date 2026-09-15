// Copyright 2026, Monado contributors
// SPDX-License-Identifier: BSL-1.0

/*!
 * @file
 * @brief Rift S camera-to-IMU clock reconciliation.
 * @ingroup drv_rift_s
 */

#include "rift_s_clock_sync.h"

#include <limits.h>
#include <stdlib.h>

#define RIFT_S_RESYNC_CONFIRM_MAX_INTERVAL_NS INT64_C(1000000000)
#define RIFT_S_RESYNC_CONFIRM_MAX_DRIFT_NS INT64_C(100000000)
#define RIFT_S_GOOD_DELTA_MAX_NS INT64_C(250000000)

static bool
add_i64(int64_t a, int64_t b, int64_t *out)
{
	if ((b > 0 && a > INT64_MAX - b) || (b < 0 && a < INT64_MIN - b)) {
		return false;
	}
	*out = a + b;
	return true;
}

static bool
subtract_i64(int64_t a, int64_t b, int64_t *out)
{
	if ((b > 0 && a < INT64_MIN + b) || (b < 0 && a > INT64_MAX + b)) {
		return false;
	}
	*out = a - b;
	return true;
}

static bool
nearest_epoch_correction(int64_t delta_ns, int64_t *out_correction_ns)
{
	int64_t epochs = delta_ns / RIFT_S_TIMESTAMP_EPOCH_NS;
	int64_t remainder = delta_ns % RIFT_S_TIMESTAMP_EPOCH_NS;
	const int64_t half_epoch = RIFT_S_TIMESTAMP_EPOCH_NS / 2;

	if (remainder >= half_epoch) {
		epochs++;
	} else if (remainder <= -half_epoch) {
		epochs--;
	}

	if (epochs > INT64_MAX / RIFT_S_TIMESTAMP_EPOCH_NS || epochs < INT64_MIN / RIFT_S_TIMESTAMP_EPOCH_NS) {
		return false;
	}
	*out_correction_ns = epochs * RIFT_S_TIMESTAMP_EPOCH_NS;
	return true;
}

static bool
sample_is_coherent_with_pending(const struct rift_s_clock_sync *sync, int64_t adjusted_ns, int64_t imu_ns)
{
	int64_t camera_elapsed_ns = 0;
	int64_t imu_elapsed_ns = 0;
	if (!subtract_i64(adjusted_ns, sync->pending_adjusted_camera_ns, &camera_elapsed_ns) ||
	    !subtract_i64(imu_ns, sync->pending_imu_ns, &imu_elapsed_ns)) {
		return false;
	}

	if (camera_elapsed_ns <= 0 || camera_elapsed_ns > RIFT_S_RESYNC_CONFIRM_MAX_INTERVAL_NS || imu_elapsed_ns < 0 ||
	    imu_elapsed_ns > RIFT_S_RESYNC_CONFIRM_MAX_INTERVAL_NS) {
		return false;
	}

	return llabs(camera_elapsed_ns - imu_elapsed_ns) <= RIFT_S_RESYNC_CONFIRM_MAX_DRIFT_NS;
}

enum rift_s_clock_sync_result
rift_s_clock_sync_push(struct rift_s_clock_sync *sync,
                       uint64_t raw_camera_ns,
                       uint64_t imu_ns,
                       struct rift_s_clock_sync_sample *out_sample)
{
	*out_sample = (struct rift_s_clock_sync_sample){
	    .raw_camera_ns = raw_camera_ns,
	    .imu_ns = imu_ns,
	};

	if (raw_camera_ns > INT64_MAX || imu_ns > INT64_MAX || imu_ns == 0) {
		return RIFT_S_CLOCK_SYNC_INVALID;
	}

	int64_t raw_ns = (int64_t)raw_camera_ns;
	int64_t imu_device_ns = (int64_t)imu_ns;
	int64_t adjusted_ns = raw_ns;
	int64_t offset_ns = sync->have_offset ? sync->camera_ts_offset_ns : 0;

	if (!add_i64(adjusted_ns, offset_ns, &adjusted_ns)) {
		return RIFT_S_CLOCK_SYNC_INVALID;
	}

	/* Select the 32-bit microsecond epoch nearest the live IMU counter. This
	 * handles ordinary wrap without treating it as a camera reset. */
	int64_t imu_to_camera_ns = 0;
	int64_t epoch_correction_ns = 0;
	if (!subtract_i64(imu_device_ns, adjusted_ns, &imu_to_camera_ns) ||
	    !nearest_epoch_correction(imu_to_camera_ns, &epoch_correction_ns)) {
		return RIFT_S_CLOCK_SYNC_INVALID;
	}
	if (epoch_correction_ns != 0) {
		if (!add_i64(adjusted_ns, epoch_correction_ns, &adjusted_ns) ||
		    !add_i64(offset_ns, epoch_correction_ns, &offset_ns)) {
			return RIFT_S_CLOCK_SYNC_INVALID;
		}
		out_sample->epoch_adjusted = true;
	}
	if (adjusted_ns < 0) {
		return RIFT_S_CLOCK_SYNC_INVALID;
	}

	sync->have_offset = true;
	sync->camera_ts_offset_ns = offset_ns;

	int64_t frame_to_imu_ns = 0;
	if (!subtract_i64(adjusted_ns, imu_device_ns, &frame_to_imu_ns)) {
		return RIFT_S_CLOCK_SYNC_INVALID;
	}
	bool in_range =
	    frame_to_imu_ns >= -RIFT_S_CAMERA_IMU_MAX_SKEW_NS && frame_to_imu_ns <= RIFT_S_CAMERA_IMU_MAX_SKEW_NS;

	if (in_range) {
		sync->have_pending = false;
		if (llabs(frame_to_imu_ns) <= RIFT_S_GOOD_DELTA_MAX_NS) {
			sync->have_good_frame_to_imu = true;
			sync->good_frame_to_imu_ns = frame_to_imu_ns;
		}

		out_sample->adjusted_camera_ns = adjusted_ns;
		out_sample->frame_to_imu_ns = frame_to_imu_ns;
		out_sample->camera_ts_offset_ns = offset_ns;
		return RIFT_S_CLOCK_SYNC_OK;
	}

	if (!sync->have_pending || !sample_is_coherent_with_pending(sync, adjusted_ns, imu_device_ns)) {
		sync->have_pending = true;
		sync->pending_adjusted_camera_ns = adjusted_ns;
		sync->pending_imu_ns = imu_device_ns;

		out_sample->adjusted_camera_ns = adjusted_ns;
		out_sample->frame_to_imu_ns = frame_to_imu_ns;
		out_sample->camera_ts_offset_ns = offset_ns;
		return RIFT_S_CLOCK_SYNC_WAITING_FOR_CONFIRMATION;
	}

	/* The camera timestamp and the live IMU clock advanced coherently, so this
	 * is a real epoch/reset rather than one corrupt frame. Preserve the last
	 * known capture-to-IMU delta when available. */
	int64_t target_delta_ns = sync->have_good_frame_to_imu ? sync->good_frame_to_imu_ns : 0;
	int64_t target_ns = 0;
	if (!add_i64(imu_device_ns, target_delta_ns, &target_ns)) {
		return RIFT_S_CLOCK_SYNC_INVALID;
	}
	if (target_ns < 0) {
		return RIFT_S_CLOCK_SYNC_INVALID;
	}

	if (!subtract_i64(target_ns, raw_ns, &offset_ns)) {
		return RIFT_S_CLOCK_SYNC_INVALID;
	}
	sync->camera_ts_offset_ns = offset_ns;
	sync->have_pending = false;
	sync->have_good_frame_to_imu = true;
	sync->good_frame_to_imu_ns = target_delta_ns;

	out_sample->adjusted_camera_ns = target_ns;
	out_sample->frame_to_imu_ns = target_delta_ns;
	out_sample->camera_ts_offset_ns = offset_ns;
	out_sample->resynchronized = true;
	return RIFT_S_CLOCK_SYNC_RESYNCED;
}

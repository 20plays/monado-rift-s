// Copyright 2026, Monado contributors
// SPDX-License-Identifier: BSL-1.0

#include "catch_amalgamated.hpp"

#include "rift_s_clock_sync.h"

#include <cstdint>
#include <limits>

static constexpr uint64_t NS_PER_MS = 1000000;
static constexpr uint64_t NS_PER_SECOND = 1000000000;
static constexpr uint64_t FRAME_INTERVAL_NS = 33333333;

static rift_s_clock_sync_sample
push_expect(rift_s_clock_sync &sync, uint64_t camera_ns, uint64_t imu_ns, rift_s_clock_sync_result expected)
{
	rift_s_clock_sync_sample sample{};
	CAPTURE(camera_ns, imu_ns);
	REQUIRE(rift_s_clock_sync_push(&sync, camera_ns, imu_ns, &sample) == expected);
	return sample;
}

TEST_CASE("Rift S normal camera timestamps remain monotonic")
{
	rift_s_clock_sync sync{};
	int64_t previous = 0;

	for (uint64_t i = 0; i < 120; i++) {
		uint64_t imu_ns = 10 * NS_PER_SECOND + i * FRAME_INTERVAL_NS;
		uint64_t camera_ns = imu_ns - 10 * NS_PER_MS;
		auto sample = push_expect(sync, camera_ns, imu_ns, RIFT_S_CLOCK_SYNC_OK);
		CHECK(sample.frame_to_imu_ns == -10 * static_cast<int64_t>(NS_PER_MS));
		CHECK(sample.adjusted_camera_ns > previous);
		previous = sample.adjusted_camera_ns;
	}
}

TEST_CASE("Rift S minor out-of-order frame is not a clock reset")
{
	rift_s_clock_sync sync{};
	auto first = push_expect(sync, 20 * NS_PER_SECOND, 20 * NS_PER_SECOND + 5 * NS_PER_MS, RIFT_S_CLOCK_SYNC_OK);
	auto second = push_expect(sync, 20 * NS_PER_SECOND - 2 * NS_PER_MS, 20 * NS_PER_SECOND + 8 * NS_PER_MS,
	                          RIFT_S_CLOCK_SYNC_OK);

	CHECK_FALSE(second.resynchronized);
	CHECK(second.adjusted_camera_ns < first.adjusted_camera_ns);
}

TEST_CASE("Rift S recovers from the observed 647 second camera discontinuity")
{
	rift_s_clock_sync sync{};
	uint64_t imu_ns = 700 * NS_PER_SECOND;
	uint64_t camera_ns = imu_ns - 12 * NS_PER_MS;
	auto before = push_expect(sync, camera_ns, imu_ns, RIFT_S_CLOCK_SYNC_OK);

	imu_ns += FRAME_INTERVAL_NS;
	camera_ns -= 647 * NS_PER_SECOND;
	push_expect(sync, camera_ns, imu_ns, RIFT_S_CLOCK_SYNC_WAITING_FOR_CONFIRMATION);

	imu_ns += FRAME_INTERVAL_NS;
	camera_ns += FRAME_INTERVAL_NS;
	auto recovered = push_expect(sync, camera_ns, imu_ns, RIFT_S_CLOCK_SYNC_RESYNCED);
	CHECK(recovered.resynchronized);
	CHECK(recovered.adjusted_camera_ns > before.adjusted_camera_ns);
	CHECK(recovered.frame_to_imu_ns == -12 * static_cast<int64_t>(NS_PER_MS));

	imu_ns += FRAME_INTERVAL_NS;
	camera_ns += FRAME_INTERVAL_NS;
	auto next = push_expect(sync, camera_ns, imu_ns, RIFT_S_CLOCK_SYNC_OK);
	CHECK(next.adjusted_camera_ns > recovered.adjusted_camera_ns);
}

TEST_CASE("Rift S selects the nearest 32-bit microsecond epoch across wrap")
{
	rift_s_clock_sync sync{};
	uint64_t epoch = static_cast<uint64_t>(RIFT_S_TIMESTAMP_EPOCH_NS);
	auto before = push_expect(sync, epoch - 20 * NS_PER_MS, epoch - 10 * NS_PER_MS, RIFT_S_CLOCK_SYNC_OK);
	auto after = push_expect(sync, 15 * NS_PER_MS, epoch + 25 * NS_PER_MS, RIFT_S_CLOCK_SYNC_OK);

	CHECK(after.epoch_adjusted);
	CHECK(after.adjusted_camera_ns > before.adjusted_camera_ns);
	CHECK(after.frame_to_imu_ns == -10 * static_cast<int64_t>(NS_PER_MS));
}

TEST_CASE("Rift S startup with stale camera timing waits then rebases")
{
	rift_s_clock_sync sync{};
	uint64_t imu_ns = 647 * NS_PER_SECOND;
	uint64_t camera_ns = 1 * NS_PER_SECOND;
	push_expect(sync, camera_ns, imu_ns, RIFT_S_CLOCK_SYNC_WAITING_FOR_CONFIRMATION);

	imu_ns += FRAME_INTERVAL_NS;
	camera_ns += FRAME_INTERVAL_NS;
	auto recovered = push_expect(sync, camera_ns, imu_ns, RIFT_S_CLOCK_SYNC_RESYNCED);
	CHECK(recovered.adjusted_camera_ns == static_cast<int64_t>(imu_ns));
	CHECK(recovered.frame_to_imu_ns == 0);
}

TEST_CASE("Rift S camera stream restart recovers without accepting one corrupt sample")
{
	rift_s_clock_sync sync{};
	uint64_t imu_ns = 100 * NS_PER_SECOND;
	uint64_t camera_ns = imu_ns;
	auto before = push_expect(sync, camera_ns, imu_ns, RIFT_S_CLOCK_SYNC_OK);

	imu_ns += FRAME_INTERVAL_NS;
	push_expect(sync, std::numeric_limits<uint64_t>::max(), imu_ns, RIFT_S_CLOCK_SYNC_INVALID);

	camera_ns += FRAME_INTERVAL_NS;
	auto still_good = push_expect(sync, camera_ns, imu_ns, RIFT_S_CLOCK_SYNC_OK);
	CHECK(still_good.adjusted_camera_ns > before.adjusted_camera_ns);

	imu_ns += FRAME_INTERVAL_NS;
	camera_ns = 2 * NS_PER_SECOND;
	push_expect(sync, camera_ns, imu_ns, RIFT_S_CLOCK_SYNC_WAITING_FOR_CONFIRMATION);
	imu_ns += FRAME_INTERVAL_NS;
	camera_ns += FRAME_INTERVAL_NS;
	push_expect(sync, camera_ns, imu_ns, RIFT_S_CLOCK_SYNC_RESYNCED);
}

TEST_CASE("Rift S future discontinuity cannot create a future frame")
{
	rift_s_clock_sync sync{};
	uint64_t imu_ns = 100 * NS_PER_SECOND;
	uint64_t camera_ns = imu_ns + 647 * NS_PER_SECOND;
	push_expect(sync, camera_ns, imu_ns, RIFT_S_CLOCK_SYNC_WAITING_FOR_CONFIRMATION);

	imu_ns += FRAME_INTERVAL_NS;
	camera_ns += FRAME_INTERVAL_NS;
	auto recovered = push_expect(sync, camera_ns, imu_ns, RIFT_S_CLOCK_SYNC_RESYNCED);
	CHECK(recovered.adjusted_camera_ns <= static_cast<int64_t>(imu_ns));
	CHECK(recovered.adjusted_camera_ns >= static_cast<int64_t>(imu_ns - RIFT_S_CAMERA_IMU_MAX_SKEW_NS));
}

TEST_CASE("Rift S rejects timestamps outside the signed timepoint range")
{
	rift_s_clock_sync sync{};
	push_expect(sync, std::numeric_limits<uint64_t>::max(), NS_PER_SECOND, RIFT_S_CLOCK_SYNC_INVALID);
	push_expect(sync, NS_PER_SECOND, std::numeric_limits<uint64_t>::max(), RIFT_S_CLOCK_SYNC_INVALID);
	push_expect(sync, NS_PER_SECOND, 0, RIFT_S_CLOCK_SYNC_INVALID);
}

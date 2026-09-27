/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
/*!
 * @file
 * @brief  Oculus Rift S Touch Controller driver
 *
 * Handles communication and calibration information for the Touch Controllers
 *
 * Ported from OpenHMD
 *
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */


#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <stdarg.h>

#include "math/m_api.h"
#include "math/m_space.h"
#include "math/m_vec3.h"

#include "os/os_hid.h"

#include "util/u_device.h"
#include "util/u_time.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"
#include "util/u_debug.h"

#include "rift_s.h"
#include "rift_s_hmd.h"
#include "rift_s_radio.h"
#include "rift_s_protocol.h"
#include "rift_s_controller.h"

/* Set to 1 to print controller states continuously */
#define DUMP_CONTROLLER_STATE 0

/* Diagnostic file logging: set RIFT_S_DIAG_FILE=/path/to.log and every
 * DIAG line is appended there (in addition to the log view), so full
 * sessions survive truncated UI buffers. The file is opened per line -
 * negligible overhead at this rate, zero shared state or lifecycle bugs. */
static void
rift_s_diag_file_log(const char *fmt, ...)
{
	const char *path = getenv("RIFT_S_DIAG_FILE");
	if (path == NULL || path[0] == '\0') {
		return;
	}
	char buf[1024];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	FILE *f = fopen(path, "a");
	if (f == NULL) {
		return;
	}
	fprintf(f, "%llu %s\n", (unsigned long long)os_monotonic_get_ns() / 1000000ULL, buf);
	fclose(f);
}

#define CTRL_DIAG(...)                                                                                               \
	do {                                                                                                         \
		RIFT_S_DEBUG(__VA_ARGS__);                                                                           \
		rift_s_diag_file_log(__VA_ARGS__);                                                                   \
	} while (0)

DEBUG_GET_ONCE_BOOL_OPTION(flip_left_yaw, "RIFT_S_FLIP_LEFT_YAW", false)

static struct xrt_binding_input_pair simple_inputs_rift_s[4] = {
    {XRT_INPUT_SIMPLE_SELECT_CLICK, XRT_INPUT_TOUCH_TRIGGER_VALUE},
    {XRT_INPUT_SIMPLE_MENU_CLICK, XRT_INPUT_TOUCH_MENU_CLICK},
    {XRT_INPUT_SIMPLE_GRIP_POSE, XRT_INPUT_TOUCH_GRIP_POSE},
    {XRT_INPUT_SIMPLE_AIM_POSE, XRT_INPUT_TOUCH_AIM_POSE},
};

static struct xrt_binding_output_pair simple_outputs_rift_s[1] = {
    {XRT_OUTPUT_NAME_SIMPLE_VIBRATION, XRT_OUTPUT_NAME_TOUCH_HAPTIC},
};

static struct xrt_binding_profile binding_profiles_rift_s[1] = {
    {
        .name = XRT_DEVICE_SIMPLE_CONTROLLER,
        .inputs = simple_inputs_rift_s,
        .input_count = ARRAY_SIZE(simple_inputs_rift_s),
        .outputs = simple_outputs_rift_s,
        .output_count = ARRAY_SIZE(simple_outputs_rift_s),
    },
};

enum touch_controller_input_index
{
	/* Left controller */
	OCULUS_TOUCH_X_CLICK = 0,
	OCULUS_TOUCH_X_TOUCH,
	OCULUS_TOUCH_Y_CLICK,
	OCULUS_TOUCH_Y_TOUCH,
	OCULUS_TOUCH_MENU_CLICK,

	/* Right controller */
	OCULUS_TOUCH_A_CLICK = 0,
	OCULUS_TOUCH_A_TOUCH,
	OCULUS_TOUCH_B_CLICK,
	OCULUS_TOUCH_B_TOUCH,
	OCULUS_TOUCH_SYSTEM_CLICK,

	/* Common */
	OCULUS_TOUCH_SQUEEZE_VALUE,
	OCULUS_TOUCH_TRIGGER_TOUCH,
	OCULUS_TOUCH_TRIGGER_VALUE,
	OCULUS_TOUCH_THUMBSTICK_CLICK,
	OCULUS_TOUCH_THUMBSTICK_TOUCH,
	OCULUS_TOUCH_THUMBSTICK,
	OCULUS_TOUCH_THUMBREST_TOUCH,
	OCULUS_TOUCH_GRIP_POSE,
	OCULUS_TOUCH_AIM_POSE,

	INPUT_INDICES_LAST
};
#define SET_TOUCH_INPUT(d, NAME) ((d)->base.inputs[OCULUS_TOUCH_##NAME].name = XRT_INPUT_TOUCH_##NAME)
#define DEBUG_TOUCH_INPUT_BOOL(d, NAME, label)                                                                         \
	u_var_add_bool((d), &(d)->base.inputs[OCULUS_TOUCH_##NAME].value.boolean, label)
#define DEBUG_TOUCH_INPUT_F32(d, NAME, label)                                                                          \
	u_var_add_f32((d), &(d)->base.inputs[OCULUS_TOUCH_##NAME].value.vec1.x, label)
#define DEBUG_TOUCH_INPUT_VEC2(d, NAME, label1, label2)                                                                \
	u_var_add_f32((d), &(d)->base.inputs[OCULUS_TOUCH_##NAME].value.vec2.x, label1);                               \
	u_var_add_f32((d), &(d)->base.inputs[OCULUS_TOUCH_##NAME].value.vec2.y, label2)

#if DUMP_CONTROLLER_STATE
static void
print_controller_state(struct rift_s_controller *ctrl)
{
	if (rift_s_log_level > U_LOGGING_TRACE)
		return; // Only log at TRACE log_level

	/* Dump the controller state if we see something unexpected / unknown, otherwise be quiet */
	if (ctrl->extra_bytes_len == 0 && ctrl->mask08 == 0x50 && ctrl->mask0e == 0)
		return;

	char buf[16384] = "";
	int bufsize = sizeof(buf) - 2;
	int printed = 0;

	printed += snprintf(buf + printed, bufsize - printed,
	                    "Controller %16lx type 0x%08x IMU ts %8u v2 %x accel %6d %6d %6d gyro %6d %6d %6d | ",
	                    ctrl->device_id, ctrl->device_type, ctrl->imu_timestamp32, ctrl->imu_unknown_varying2,
	                    ctrl->raw_accel[0], ctrl->raw_accel[1], ctrl->raw_accel[2], ctrl->raw_gyro[0],
	                    ctrl->raw_gyro[1], ctrl->raw_gyro[2]);

	printed += snprintf(buf + printed, bufsize - printed, "unk %02x %02x buttons %02x fingers %02x | ",
	                    ctrl->mask08, ctrl->mask0e, ctrl->buttons, ctrl->fingers);
	printed += snprintf(buf + printed, bufsize - printed, "trigger %5d grip %5d |", ctrl->trigger, ctrl->grip);
	printed +=
	    snprintf(buf + printed, bufsize - printed, "joystick x %5d y %5d |", ctrl->joystick_x, ctrl->joystick_y);

	if (ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER) {
		printed +=
		    snprintf(buf + printed, bufsize - printed, "capsense x %u y %u joy %u trig %u | ",
		             ctrl->capsense_a_x, ctrl->capsense_b_y, ctrl->capsense_joystick, ctrl->capsense_trigger);
	} else if (ctrl->device_type == RIFT_S_DEVICE_RIGHT_CONTROLLER) {
		printed +=
		    snprintf(buf + printed, bufsize - printed, "capsense a %u b %u joy %u trig %u | ",
		             ctrl->capsense_a_x, ctrl->capsense_b_y, ctrl->capsense_joystick, ctrl->capsense_trigger);
	} else {
		printed +=
		    snprintf(buf + printed, bufsize - printed, "capsense ?? %u ?? %u ?? %u ?? %u | ",
		             ctrl->capsense_a_x, ctrl->capsense_b_y, ctrl->capsense_joystick, ctrl->capsense_trigger);
	}

	if (ctrl->extra_bytes_len) {
		printed += snprintf(buf + printed, bufsize - printed, " | extra ");
		printed += rift_s_snprintf_hexdump_buffer(buf + printed, bufsize - printed, NULL, ctrl->extra_bytes,
		                                          ctrl->extra_bytes_len);
	}

	RIFT_S_TRACE("%s", buf);
}
#endif

static void
handle_imu_update(struct rift_s_controller *ctrl,
                  timepoint_ns local_ts,
                  uint32_t imu_timestamp,
                  const int16_t raw_accel[3],
                  const int16_t raw_gyro[3])
{
	/* Logic to update 64-bit ns timestamp from
	 * 32-bit µS device timestamp that wraps every 71.5 minutes */
	uint32_t dt = 0;

	if (ctrl->imu_time_valid) {
		dt = imu_timestamp - ctrl->imu_timestamp32;

		/* Sometimes we see 1-2 repeated IMU updates from a controller,
		 * that must be ignored or else time jumps wildly */
		if (dt == 0 || dt > 2147483648) {
			RIFT_S_TRACE("Controller %" PRIx64 " - ignoring repeated IMU update", ctrl->device_id);
			return;
		}

		ctrl->last_imu_device_time_ns += (timepoint_ns)dt * OS_NS_PER_USEC;
	} else {
		ctrl->last_imu_device_time_ns = (timepoint_ns)imu_timestamp * OS_NS_PER_USEC;
		ctrl->imu_time_valid = true;
	}
	ctrl->imu_timestamp32 = imu_timestamp;
	ctrl->last_imu_local_time_ns = local_ts;

	if (!ctrl->have_calibration || !ctrl->have_config)
		return; /* We need to finish reading the calibration or config blocks first */

	const float gyro_scale = ctrl->config.gyro_scale;
	const float accel_scale = MATH_GRAVITY_M_S2 * ctrl->config.accel_scale;

	struct xrt_vec3 gyro, accel;

	gyro.x = DEG_TO_RAD(gyro_scale * raw_gyro[0]);
	gyro.y = DEG_TO_RAD(gyro_scale * raw_gyro[1]);
	gyro.z = DEG_TO_RAD(gyro_scale * raw_gyro[2]);

	accel.x = accel_scale * raw_accel[0];
	accel.y = accel_scale * raw_accel[1];
	accel.z = accel_scale * raw_accel[2];

	/* Apply correction offsets first, then rectify */
	accel = m_vec3_sub(accel, ctrl->calibration.accel.offset);
	gyro = m_vec3_sub(gyro, ctrl->calibration.gyro.offset);

	math_matrix_3x3_transform_vec3(&ctrl->calibration.accel.rectification, &accel, &ctrl->accel);
	math_matrix_3x3_transform_vec3(&ctrl->calibration.gyro.rectification, &gyro, &ctrl->gyro);

	/* Never feed non-finite samples to the fusion: one NaN orients the
	 * controller to nowhere permanently. */
	if (!isfinite(ctrl->accel.x) || !isfinite(ctrl->accel.y) || !isfinite(ctrl->accel.z) ||
	    !isfinite(ctrl->gyro.x) || !isfinite(ctrl->gyro.y) || !isfinite(ctrl->gyro.z)) {
		return;
	}

	m_imu_3dof_update(&ctrl->fusion, ctrl->last_imu_device_time_ns, &ctrl->accel, &ctrl->gyro);
	ctrl->pose.orientation = ctrl->fusion.rot;

	/* Diagnostics: usable IMU sample rate, plus mean linear-accel magnitude
	 * while stationary (reveals gravity leakage / accel bias: should sit
	 * near 0.00 m/s^2 when the hand is still). */
	ctrl->diag_imu_count++;
	{
		struct xrt_vec3 a_world;
		math_quat_rotate_vec3(&ctrl->fusion.rot, &ctrl->accel, &a_world);
		a_world.y -= (float)MATH_GRAVITY_M_S2;
		float gyro_speed = m_vec3_len(ctrl->gyro);
		float opt_speed = ctrl->have_linear_velocity ? m_vec3_len(ctrl->linear_velocity) : 0.0f;
		if (gyro_speed < 0.15f && opt_speed < 0.05f) {
			float m = m_vec3_len(a_world);
			if (isfinite(m)) {
				ctrl->diag_still_lin_accel_avg += 0.01f * (m - ctrl->diag_still_lin_accel_avg);
			}
		}
	}

#if 0
	RIFT_S_DEBUG("%" PRIx64 " dt %u device time %u ns %" PRIu64
	             " raw accel %d %d %d gyro %d %d %d -> accel %f %f %f  gyro %f %f %f\n",
	             ctrl->device_id, dt, imu_timestamp, ctrl->last_imu_device_time_ns, raw_accel[0], raw_accel[1],
	             raw_accel[2], raw_gyro[0], raw_gyro[1], raw_gyro[2], ctrl->accel.x, ctrl->accel.y, ctrl->accel.z,
	             ctrl->gyro.x, ctrl->gyro.y, ctrl->gyro.z);
#endif
}

bool
rift_s_controller_handle_report(struct rift_s_controller *ctrl,
                                timepoint_ns local_ts,
                                rift_s_controller_report_t *report)
{
#if DUMP_CONTROLLER_STATE
	bool saw_imu_update = false;
#endif
	bool saw_controls_update = false;

	os_mutex_lock(&ctrl->mutex);

	/* Collect state updates */
	ctrl->extra_bytes_len = 0;

	for (int i = 0; i < report->num_info; i++) {
		rift_s_controller_info_block_t *info = report->info + i;

		switch (info->block_id) {
		case RIFT_S_CTRL_MASK08:
			saw_controls_update = true;
			ctrl->mask08 = info->maskbyte.val;
			break;
		case RIFT_S_CTRL_BUTTONS:
			saw_controls_update = true;
			ctrl->buttons = info->maskbyte.val;
			break;
		case RIFT_S_CTRL_FINGERS:
			saw_controls_update = true;
			ctrl->fingers = info->maskbyte.val;
			break;
		case RIFT_S_CTRL_MASK0e:
			saw_controls_update = true;
			ctrl->mask0e = info->maskbyte.val;
			break;
		case RIFT_S_CTRL_TRIGGRIP: {
			saw_controls_update = true;
			ctrl->trigger = (uint16_t)(info->triggrip.vals[1] & 0x0f) << 8 | info->triggrip.vals[0];
			ctrl->grip =
			    (uint16_t)(info->triggrip.vals[1] & 0xf0) >> 4 | ((uint16_t)(info->triggrip.vals[2]) << 4);
			break;
		}
		case RIFT_S_CTRL_JOYSTICK:
			saw_controls_update = true;
			ctrl->joystick_x = info->joystick.val;
			ctrl->joystick_y = info->joystick.val >> 16;
			break;
		case RIFT_S_CTRL_CAPSENSE:
			saw_controls_update = true;
			ctrl->capsense_a_x = info->capsense.a_x;
			ctrl->capsense_b_y = info->capsense.b_y;
			ctrl->capsense_joystick = info->capsense.joystick;
			ctrl->capsense_trigger = info->capsense.trigger;
			break;
		case RIFT_S_CTRL_IMU: {
			int j;

#if DUMP_CONTROLLER_STATE
			/* print the state before updating the IMU timestamp a 2nd time */
			if (saw_imu_update)
				print_controller_state(ctrl);
			saw_imu_update = true;
#endif

			ctrl->imu_unknown_varying2 = info->imu.unknown_varying2;

			for (j = 0; j < 3; j++) {
				ctrl->raw_accel[j] = info->imu.accel[j];
				ctrl->raw_gyro[j] = info->imu.gyro[j];
			}
			handle_imu_update(ctrl, local_ts, info->imu.timestamp, ctrl->raw_accel, ctrl->raw_gyro);
			break;
		}
		default:
			RIFT_S_WARN("Invalid controller info block with ID %02x from device %08" PRIx64
			            ". Please report it.\n",
			            info->block_id, ctrl->device_id);
		}
	}

	if (saw_controls_update)
		ctrl->last_controls_local_time_ns = local_ts;

	if (report->extra_bytes_len > 0) {
		if (report->extra_bytes_len > sizeof(ctrl->extra_bytes)) {
			RIFT_S_WARN("Controller report from %16" PRIx64 " had too many extra bytes - %u (max %u)\n",
			            ctrl->device_id, report->extra_bytes_len,
			            (unsigned int)(sizeof(ctrl->extra_bytes)));
			report->extra_bytes_len = sizeof(ctrl->extra_bytes);
		}
		memcpy(ctrl->extra_bytes, report->extra_bytes, report->extra_bytes_len);
	}
	ctrl->extra_bytes_len = report->extra_bytes_len;

#if DUMP_CONTROLLER_STATE
	print_controller_state(ctrl);
#endif

	/* Finally, update and output the log */
	if (report->flags & 0x04) {
		/* New log line is starting, reset the counter */
		ctrl->log_bytes = 0;
	}

	if (ctrl->log_flags & 0x04 || (ctrl->log_flags & 0x02) != (report->flags & 0x02)) {
		/* New log bytes in this report, collect them */
		for (int i = 0; i < 3; i++) {
			uint8_t c = report->log[i];
			if (c != '\0') {
				if (ctrl->log_bytes == (MAX_LOG_SIZE - 1)) {
					/* Log line got too long... output it */
					ctrl->log[MAX_LOG_SIZE - 1] = '\0';
					RIFT_S_DEBUG("Controller: %s", ctrl->log);
					ctrl->log_bytes = 0;
				}
				ctrl->log[ctrl->log_bytes++] = c;
			} else if (ctrl->log_bytes > 0) {
				/* Found the end of the string */
				ctrl->log[ctrl->log_bytes] = '\0';
				rift_s_hexdump_buffer("Controller debug", ctrl->log, ctrl->log_bytes);
				ctrl->log_bytes = 0;
			}
		}
	}
	ctrl->log_flags = report->flags;

	os_mutex_unlock(&ctrl->mutex);
	return true;
}

#define READ_LE16(b) (b)[0] | ((b)[1]) << 8
#define READ_LE32(b) (b)[0] | ((b)[1]) << 8 | ((b)[2]) << 16 | ((b)[3]) << 24
#define READ_LEFLOAT32(b) (*(float *)(b));

static void
ctrl_config_cb(bool success, uint8_t *response_bytes, int response_bytes_len, struct rift_s_controller *ctrl)
{
	if (!success) {
		os_mutex_lock(&ctrl->mutex);
		ctrl->reading_config = false;
		os_mutex_unlock(&ctrl->mutex);
		RIFT_S_WARN("Failed to read config for controller 0x%016" PRIx64 "; will retry", ctrl->device_id);
		return;
	}

	if (response_bytes_len < 5) {
		os_mutex_lock(&ctrl->mutex);
		ctrl->reading_config = false;
		os_mutex_unlock(&ctrl->mutex);
		RIFT_S_WARN("Failed to read config for controller 0x%016" PRIx64 " (short result %d bytes); will retry",
		            ctrl->device_id, response_bytes_len);
		return;
	}

	/* Response 0u32 0x10   00 7d a0 0f f4 01 f4 01 00 00 80 3a ff ff f9 3d
	 *   0x7d00 = 32000 0x0fa0 = 4000 0x01f4 = 500 0x01f4 = 500
	 *   0x3a800000 = 0.9765625e-03  = 1/1024
	 *   0x3df9ffff = 0.1220703      = 1/8192
	 */

	response_bytes_len = response_bytes[4];
	if (response_bytes_len < 16) {
		char buf[16384] = "";
		int bufsize = sizeof(buf) - 2;
		int printed = 0;

		printed += rift_s_snprintf_hexdump_buffer(buf + printed, bufsize - printed, "Controller Config",
		                                          response_bytes, response_bytes_len);

		os_mutex_lock(&ctrl->mutex);
		ctrl->reading_config = false;
		os_mutex_unlock(&ctrl->mutex);
		RIFT_S_WARN("Failed to read config block for controller 0x%016" PRIx64 " (got %d bytes); will retry\n%s",
		            ctrl->device_id, response_bytes_len, buf);
		return;
	}
	response_bytes += 5;

	os_mutex_lock(&ctrl->mutex);
	ctrl->reading_config = false;

	ctrl->config.accel_limit = READ_LE16(response_bytes + 0);
	ctrl->config.gyro_limit = READ_LE16(response_bytes + 2);
	ctrl->config.accel_hz = READ_LE16(response_bytes + 4);
	ctrl->config.gyro_hz = READ_LE16(response_bytes + 6);
	ctrl->config.accel_scale = READ_LEFLOAT32(response_bytes + 8);
	ctrl->config.gyro_scale = READ_LEFLOAT32(response_bytes + 12);

	ctrl->have_config = true;
	os_mutex_unlock(&ctrl->mutex);

	RIFT_S_INFO("Read config for controller 0x%016" PRIx64
	            " type %08x. "
	            "limit/scale/hz Accel %u %f %u Gyro %u %f %u",
	            ctrl->device_id, ctrl->device_type, ctrl->config.accel_limit, ctrl->config.accel_scale,
	            ctrl->config.accel_hz, ctrl->config.gyro_limit, ctrl->config.gyro_scale, ctrl->config.gyro_hz);
}

static void
ctrl_json_cb(bool success, uint8_t *response_bytes, int response_bytes_len, struct rift_s_controller *ctrl)
{
	if (!success) {
		os_mutex_lock(&ctrl->mutex);
		ctrl->reading_calibration = false;
		os_mutex_unlock(&ctrl->mutex);
		RIFT_S_WARN("Failed to read calibration block for controller 0x%016" PRIx64 "; will retry",
		            ctrl->device_id);
		return;
	}

	os_mutex_lock(&ctrl->mutex);
	ctrl->reading_calibration = false;

	RIFT_S_TRACE("Got Controller calibration:\n%s", response_bytes);

	if (rift_s_controller_parse_imu_calibration((char *)response_bytes, &ctrl->calibration) == 0) {
		ctrl->P_device_imu.position = ctrl->calibration.imu_position;
		math_pose_invert(&ctrl->P_device_imu, &ctrl->P_imu_device);
		ctrl->have_calibration = true;
	} else {
		RIFT_S_WARN("Failed to parse calibration for controller 0x%016" PRIx64 "; will retry",
		            ctrl->device_id);
	}
	os_mutex_unlock(&ctrl->mutex);
}

static void
rift_s_update_input_bool(struct rift_s_controller *ctrl, int index, int64_t when_ns, int val)
{
	ctrl->base.inputs[index].timestamp = when_ns;
	ctrl->base.inputs[index].value.boolean = (val != 0);
}

static void
rift_s_update_input_analog(struct rift_s_controller *ctrl, int index, int64_t when_ns, float val)
{
	ctrl->base.inputs[index].timestamp = when_ns;
	ctrl->base.inputs[index].value.vec1.x = val;
}

static void
rift_s_update_input_vec2(struct rift_s_controller *ctrl, int index, int64_t when_ns, float x, float y)
{
	ctrl->base.inputs[index].timestamp = when_ns;
	ctrl->base.inputs[index].value.vec2.x = x;
	ctrl->base.inputs[index].value.vec2.y = y;
}

static xrt_result_t
rift_s_controller_update_inputs(struct xrt_device *xdev)
{
	struct rift_s_controller *ctrl = (struct rift_s_controller *)(xdev);

	os_mutex_lock(&ctrl->mutex);

	uint64_t last_ns = ctrl->last_controls_local_time_ns;

	if (ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER) {
		rift_s_update_input_bool(ctrl, OCULUS_TOUCH_X_CLICK, last_ns, ctrl->buttons & RIFT_S_BUTTON_A_X);
		rift_s_update_input_bool(ctrl, OCULUS_TOUCH_Y_CLICK, last_ns, ctrl->buttons & RIFT_S_BUTTON_B_Y);
		rift_s_update_input_bool(ctrl, OCULUS_TOUCH_MENU_CLICK, last_ns,
		                         ctrl->buttons & RIFT_S_BUTTON_MENU_OCULUS);
		rift_s_update_input_bool(
		    ctrl, OCULUS_TOUCH_X_TOUCH, last_ns,
		    !!((ctrl->fingers & RIFT_S_FINGER_A_X_STRONG) ||
		       ((ctrl->fingers & RIFT_S_FINGER_A_X_WEAK) &&
		        !(ctrl->fingers & (RIFT_S_FINGER_B_Y_STRONG | RIFT_S_FINGER_STICK_STRONG)))));
		rift_s_update_input_bool(
		    ctrl, OCULUS_TOUCH_Y_TOUCH, last_ns,
		    !!((ctrl->fingers & RIFT_S_FINGER_B_Y_STRONG) ||
		       ((ctrl->fingers & RIFT_S_FINGER_B_Y_WEAK) &&
		        !(ctrl->fingers & (RIFT_S_FINGER_A_X_STRONG | RIFT_S_FINGER_STICK_STRONG)))));
	} else {
		rift_s_update_input_bool(ctrl, OCULUS_TOUCH_A_CLICK, last_ns, ctrl->buttons & RIFT_S_BUTTON_A_X);
		rift_s_update_input_bool(ctrl, OCULUS_TOUCH_B_CLICK, last_ns, ctrl->buttons & RIFT_S_BUTTON_B_Y);
		rift_s_update_input_bool(ctrl, OCULUS_TOUCH_SYSTEM_CLICK, last_ns,
		                         ctrl->buttons & RIFT_S_BUTTON_MENU_OCULUS);
		rift_s_update_input_bool(
		    ctrl, OCULUS_TOUCH_A_TOUCH, last_ns,
		    !!((ctrl->fingers & RIFT_S_FINGER_A_X_STRONG) ||
		       ((ctrl->fingers & RIFT_S_FINGER_A_X_WEAK) &&
		        !(ctrl->fingers & (RIFT_S_FINGER_B_Y_STRONG | RIFT_S_FINGER_STICK_STRONG)))));
		rift_s_update_input_bool(
		    ctrl, OCULUS_TOUCH_B_TOUCH, last_ns,
		    !!((ctrl->fingers & RIFT_S_FINGER_B_Y_STRONG) ||
		       ((ctrl->fingers & RIFT_S_FINGER_B_Y_WEAK) &&
		        !(ctrl->fingers & (RIFT_S_FINGER_A_X_STRONG | RIFT_S_FINGER_STICK_STRONG)))));
	}

	rift_s_update_input_analog(ctrl, OCULUS_TOUCH_SQUEEZE_VALUE, last_ns, 1.0 - (float)(ctrl->grip) / 4096.0);
	rift_s_update_input_analog(ctrl, OCULUS_TOUCH_TRIGGER_VALUE, last_ns, 1.0 - (float)(ctrl->trigger) / 4096.0);

	rift_s_update_input_bool(ctrl, OCULUS_TOUCH_TRIGGER_TOUCH, last_ns,
	                         !!(ctrl->fingers & (RIFT_S_FINGER_TRIGGER_WEAK | RIFT_S_FINGER_TRIGGER_STRONG)));

	rift_s_update_input_bool(ctrl, OCULUS_TOUCH_THUMBSTICK_CLICK, last_ns, ctrl->buttons & RIFT_S_BUTTON_STICK);

	rift_s_update_input_bool(ctrl, OCULUS_TOUCH_THUMBSTICK_TOUCH, last_ns,
	                         !!((ctrl->fingers & RIFT_S_FINGER_STICK_STRONG) ||
	                            ((ctrl->fingers & RIFT_S_FINGER_STICK_WEAK) &&
	                             !(ctrl->fingers & (RIFT_S_FINGER_A_X_STRONG | RIFT_S_FINGER_B_Y_STRONG)))));

	rift_s_update_input_vec2(ctrl, OCULUS_TOUCH_THUMBSTICK, last_ns,
	                         (float)(ctrl->joystick_x) / 32768.0, /* FIXME: Scale this properly */
	                         (float)(ctrl->joystick_y) / 32768.0  /* FIXME: Scale this properly */
	);

	/* FIXME: Output touch detections:
	      OCULUS_TOUCH_THUMBREST_TOUCH, - does Rift S have a thumbrest?
	*/
	os_mutex_unlock(&ctrl->mutex);

	return XRT_SUCCESS;
}

static void
rift_s_controller_get_fusion_pose(struct rift_s_controller *ctrl,
                                  enum xrt_input_name name,
                                  int64_t at_timestamp_ns,
                                  struct xrt_space_relation *out_relation)
{
	out_relation->pose = ctrl->pose;
	out_relation->linear_velocity.x = 0.0f;
	out_relation->linear_velocity.y = 0.0f;
	out_relation->linear_velocity.z = 0.0f;

	/*!
	 * @todo This is hack, fusion reports angvel relative to the device but
	 * it needs to be in relation to the base space. Rotating it with the
	 * device orientation is enough to get it into the right space, angular
	 * velocity is a derivative so needs a special rotation.
	 */
	math_quat_rotate_derivative(&ctrl->pose.orientation, &ctrl->fusion.last.gyro, &out_relation->angular_velocity);

	out_relation->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	    XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT);
}

static xrt_result_t
rift_s_controller_get_tracked_pose(struct xrt_device *xdev,
                                   enum xrt_input_name name,
                                   int64_t at_timestamp_ns,
                                   struct xrt_space_relation *out_relation)
{
	struct rift_s_controller *ctrl = (struct rift_s_controller *)(xdev);

	if (name != XRT_INPUT_TOUCH_AIM_POSE && name != XRT_INPUT_TOUCH_GRIP_POSE &&
	    name != XRT_INPUT_GENERIC_TRACKER_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&ctrl->base, rift_s_log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct xrt_relation_chain xrc = {0};

	os_mutex_lock(&ctrl->mutex);
	if (ctrl->flip_left_yaw) {
		struct xrt_pose flip = {
		    .orientation = {.x = 0.0f, .y = 1.0f, .z = 0.0f, .w = 0.0f},
		    .position = {0.0f, 0.0f, 0.0f},
		};
		m_relation_chain_push_pose(&xrc, &flip);
	}
	if (name == XRT_INPUT_TOUCH_GRIP_POSE) {
		m_relation_chain_push_pose(&xrc, &ctrl->P_aim_grip);
	}
	m_relation_chain_push_pose_if_not_identity(&xrc, &ctrl->P_imu_device);

	/* Apply the fusion rotation */
	struct xrt_space_relation *rel = m_relation_chain_reserve(&xrc);

	rift_s_controller_get_fusion_pose(ctrl, name, at_timestamp_ns, rel);
	if (ctrl->last_tracked_pose_ts != 0) {
		rel->pose.position = ctrl->last_tracked_pose.position;
		rel->relation_flags |= (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_POSITION_VALID_BIT |
		                                                       XRT_SPACE_RELATION_POSITION_TRACKED_BIT);

		/* Diagnostics: how stale the optical anchor is at query time.
		 * Reported as the window maximum in the periodic summary. */
		float age_ms =
		    (float)(at_timestamp_ns - ctrl->last_tracked_pose_ts) / (float)U_TIME_1MS_IN_NS;
		if (age_ms > ctrl->diag_max_opt_age_ms) {
			ctrl->diag_max_opt_age_ms = age_ms;
		}

		if (ctrl->have_linear_velocity) {
			/* Coast the anchor with exponentially decaying velocity
			 * (tau 200 ms, horizon 250 ms). Between 30 Hz frames this
			 * is near-full linear extrapolation, so motion is smooth
			 * at any query rate with no deadband; during tracking
			 * loss the hand glides to a stop instead of freezing
			 * mid-air or flying away. Bounded by construction:
			 * displacement saturates at |v| * tau, velocity is
			 * clamped at 4 m/s upstream, and the decayed velocity is
			 * what SteamVR gets for its own prediction. */
			time_duration_ns dt_ns = at_timestamp_ns - ctrl->last_tracked_pose_ts;
			if (dt_ns < 0) {
				dt_ns = 0;
			} else if (dt_ns > 250 * U_TIME_1MS_IN_NS) {
				dt_ns = 250 * U_TIME_1MS_IN_NS;
			}
			float dt = (float)dt_ns / (float)U_TIME_1S_IN_NS;
			float decay = expf(-dt / 0.2f);
			float k = 0.2f * (1.0f - decay);
			rel->pose.position.x += ctrl->linear_velocity.x * k;
			rel->pose.position.y += ctrl->linear_velocity.y * k;
			rel->pose.position.z += ctrl->linear_velocity.z * k;
			rel->linear_velocity.x = ctrl->linear_velocity.x * decay;
			rel->linear_velocity.y = ctrl->linear_velocity.y * decay;
			rel->linear_velocity.z = ctrl->linear_velocity.z * decay;
			rel->relation_flags |=
			    (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT);
		}
	}
	os_mutex_unlock(&ctrl->mutex);

	m_relation_chain_resolve(&xrc, out_relation);

	return XRT_SUCCESS;
}

static xrt_result_t
rift_s_controller_set_output(struct xrt_device *xdev,
                             enum xrt_output_name name,
                             const struct xrt_output_value *value)
{
	/* Haptics are not driven yet. Acknowledge quietly instead of falling
	 * through to the not-implemented handler, which logs an error on every
	 * call and floods limited log buffers. */
	return XRT_SUCCESS;
}

static void
rift_s_controller_destroy(struct xrt_device *xdev)
{
	struct rift_s_controller *ctrl = (struct rift_s_controller *)(xdev);

	DRV_TRACE_MARKER();

	RIFT_S_DEBUG("Destroying %s controller", ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER ? "left" : "right");

	// Tell the tracker that we're going away
	if (ctrl->tracking_connection) {
		t_constellation_tracked_device_connection_disconnect(ctrl->tracking_connection);
		ctrl->tracking_connection = NULL;
	}

	/* Tell the system this controller is going away */
	rift_s_system_remove_controller(ctrl->sys, ctrl);

	/* Release the HMD reference */
	rift_s_system_reference(&ctrl->sys, NULL);

	u_var_remove_root(ctrl);

	m_imu_3dof_close(&ctrl->fusion);

	os_mutex_destroy(&ctrl->mutex);

	u_device_free(&ctrl->base);
}

static bool
rift_s_controller_get_led_model(struct xrt_device *xdev, struct t_constellation_led_model *led_model)
{
	struct rift_s_controller *ctrl = (struct rift_s_controller *)(xdev);

	os_mutex_lock(&ctrl->mutex);
	if (!ctrl->have_calibration) {
		os_mutex_unlock(&ctrl->mutex);
		return false;
	}
	os_mutex_unlock(&ctrl->mutex);

	t_constellation_led_model_init((int)ctrl->base.device_type, NULL, led_model, ctrl->calibration.num_leds,
	                               0);

	// Note: This LED model is in OpenXR coordinates with
	// XYZ = Right/Down/Forward. Flip to OpenCV for the constellation tracker
	// after moving to imu-relative position
	for (int i = 0; i < ctrl->calibration.num_leds; i++) {
		struct t_constellation_led *led = led_model->leds + i;
		struct xrt_vec3 pos, dir;

		math_pose_transform_point(&ctrl->P_imu_device, &ctrl->calibration.leds[i].pos, &pos);
		math_quat_rotate_vec3(&ctrl->P_imu_device.orientation, &ctrl->calibration.leds[i].dir, &dir);

		led->id = i;
		led->pos.x = pos.x;
		led->pos.y = -pos.y;
		led->pos.z = -pos.z;
		led->dir.x = dir.x;
		led->dir.y = -dir.y;
		led->dir.z = -dir.z;

		led->radius_mm = 3.5;
	}

	return true;
}

static void
rift_s_controller_push_observed_pose(struct xrt_device *xdev, timepoint_ns frame_mono_ns, const struct xrt_pose *pose)
{
	struct rift_s_controller *ctrl = (struct rift_s_controller *)(xdev);
	os_mutex_lock(&ctrl->mutex);

	timepoint_ns prev_ts = ctrl->last_tracked_pose_ts;

	/* Never let a non-finite PnP solve into the pipeline: one NaN poisons
	 * the One-Euro filter, the velocity, the anchor and (via priors) future
	 * solves permanently. Drop the frame instead. */
	if (!isfinite(pose->position.x) || !isfinite(pose->position.y) || !isfinite(pose->position.z) ||
	    !isfinite(pose->orientation.x) || !isfinite(pose->orientation.y) || !isfinite(pose->orientation.z) ||
	    !isfinite(pose->orientation.w)) {
		ctrl->diag_nan_count++;
		CTRL_DIAG("DIAG %s NONFINITE optical solve dropped",
		             ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER ? "L" : "R");
		os_mutex_unlock(&ctrl->mutex);
		return;
	}

	/* Stale or duplicate observation (same or older capture time than the
	 * anchor). Feeding it to the One-Euro filter would divide by dt=0 and
	 * NaN the whole position pipeline, so drop it. */
	if (prev_ts != 0 && frame_mono_ns <= prev_ts) {
		ctrl->diag_stale_count++;
		os_mutex_unlock(&ctrl->mutex);
		return;
	}

	/* Continuity gate on the RAW observation, so rejected outliers never
	 * touch the smoothing filter or the anchor. Predicts where the hand
	 * should be from the last accepted pose plus optical velocity and
	 * rejects absurd jumps: hand swaps when the controllers get close,
	 * flipped PnP solutions, and mismatched blobs all move decimeters in
	 * one 33 ms frame, which real hands cannot do. The gate scales with
	 * speed so fast swings pass; it only engages in steady track (a valid
	 * velocity estimate exists) and force-accepts after 10 consecutive
	 * rejects so a bad anchor can never freeze the hand forever. Pure
	 * linear prediction only - no integrated state, so this cannot
	 * diverge: worst case it drops a third of a second of frames. */
	if (ctrl->have_linear_velocity && prev_ts != 0 && frame_mono_ns > prev_ts) {
		time_duration_ns gap_ns = frame_mono_ns - prev_ts;
		if (gap_ns >= 15 * U_TIME_1MS_IN_NS && gap_ns <= 150 * U_TIME_1MS_IN_NS) {
			float gap_s = (float)gap_ns / (float)U_TIME_1S_IN_NS;
			struct xrt_vec3 predicted = {
			    .x = ctrl->last_tracked_pose.position.x + ctrl->linear_velocity.x * gap_s,
			    .y = ctrl->last_tracked_pose.position.y + ctrl->linear_velocity.y * gap_s,
			    .z = ctrl->last_tracked_pose.position.z + ctrl->linear_velocity.z * gap_s,
			};
			float residual = m_vec3_len(m_vec3_sub(pose->position, predicted));
			float speed = m_vec3_len(ctrl->linear_velocity);
			float gate = 0.07f + 3.0f * speed * gap_s;
			if (residual > gate && ctrl->optical_reject_count < 10) {
				ctrl->optical_reject_count++;
				ctrl->diag_reject_count++;
				ctrl->yaw_consensus_count = 0;
				CTRL_DIAG("DIAG %s REJECT residual=%.3fm gate=%.3fm pos=(%.3f,%.3f,%.3f)",
				             ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER ? "L" : "R",
				             residual, gate, pose->position.x, pose->position.y,
				             pose->position.z);
				os_mutex_unlock(&ctrl->mutex);
				return;
			}
		}
	}
	ctrl->optical_reject_count = 0;

	timepoint_ns now_mono_ns = os_monotonic_get_ns();
	ctrl->diag_opt_count++;

	/* One-time geometry line per session: proves which calibration the
	 * poses are built from (IMU-to-device offset, aim-to-grip offset,
	 * IMU scales/offsets/rates). */
	if (!ctrl->diag_config_logged) {
		ctrl->diag_config_logged = true;
		const char *side = ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER ? "L" : "R";
		CTRL_DIAG("DIAG %s cfg calib=%d P_imu_device pos=(%.4f,%.4f,%.4f) quat=(%.4f,%.4f,%.4f,%.4f) "
		             "P_aim_grip pos=(%.4f,%.4f,%.4f) quat=(%.4f,%.4f,%.4f,%.4f)",
		             side, (int)ctrl->have_calibration, ctrl->P_imu_device.position.x,
		             ctrl->P_imu_device.position.y, ctrl->P_imu_device.position.z,
		             ctrl->P_imu_device.orientation.x, ctrl->P_imu_device.orientation.y,
		             ctrl->P_imu_device.orientation.z, ctrl->P_imu_device.orientation.w,
		             ctrl->P_aim_grip.position.x, ctrl->P_aim_grip.position.y, ctrl->P_aim_grip.position.z,
		             ctrl->P_aim_grip.orientation.x, ctrl->P_aim_grip.orientation.y,
		             ctrl->P_aim_grip.orientation.z, ctrl->P_aim_grip.orientation.w);
		CTRL_DIAG("DIAG %s imuCfg config=%d accelScale=%.6f gyroScale=%.6f accelHz=%u gyroHz=%u "
		             "accelOff=(%.4f,%.4f,%.4f) gyroOff=(%.4f,%.4f,%.4f)",
		             side, (int)ctrl->have_config, ctrl->config.accel_scale, ctrl->config.gyro_scale,
		             ctrl->config.accel_hz, ctrl->config.gyro_hz, ctrl->calibration.accel.offset.x,
		             ctrl->calibration.accel.offset.y, ctrl->calibration.accel.offset.z,
		             ctrl->calibration.gyro.offset.x, ctrl->calibration.gyro.offset.y,
		             ctrl->calibration.gyro.offset.z);
		/* LED cloud geometry in the raw calibration (device) frame: proves
		 * the constellation model sits where the physical LEDs are. A
		 * mirrored/offset cloud fits mirrored poses - systematically wrong
		 * orientation with roughly-right position. */
		if (ctrl->have_calibration && ctrl->calibration.num_leds > 0 && ctrl->calibration.leds != NULL) {
			struct xrt_vec3 centroid = {0, 0, 0};
			struct xrt_vec3 min_p = {1e9f, 1e9f, 1e9f};
			struct xrt_vec3 max_p = {-1e9f, -1e9f, -1e9f};
			for (int li = 0; li < ctrl->calibration.num_leds; li++) {
				struct xrt_vec3 p = ctrl->calibration.leds[li].pos;
				centroid.x += p.x;
				centroid.y += p.y;
				centroid.z += p.z;
				if (p.x < min_p.x) {
					min_p.x = p.x;
				}
				if (p.y < min_p.y) {
					min_p.y = p.y;
				}
				if (p.z < min_p.z) {
					min_p.z = p.z;
				}
				if (p.x > max_p.x) {
					max_p.x = p.x;
				}
				if (p.y > max_p.y) {
					max_p.y = p.y;
				}
				if (p.z > max_p.z) {
					max_p.z = p.z;
				}
			}
			float n = (float)ctrl->calibration.num_leds;
			CTRL_DIAG("DIAG %s leds n=%d centroid=(%.4f,%.4f,%.4f) "
			             "min=(%.4f,%.4f,%.4f) max=(%.4f,%.4f,%.4f) imuPos=(%.4f,%.4f,%.4f)",
			             side, ctrl->calibration.num_leds, centroid.x / n, centroid.y / n,
			             centroid.z / n, min_p.x, min_p.y, min_p.z, max_p.x, max_p.y, max_p.z,
			             ctrl->calibration.imu_position.x, ctrl->calibration.imu_position.y,
			             ctrl->calibration.imu_position.z);
		}
	}

	// Run optical position observation through One-Euro filter to remove 30 Hz camera sensor discretization noise
	struct xrt_vec3 filtered_pos;
	if (!ctrl->pos_filter_initialized) {
		m_filter_euro_vec3_init(&ctrl->pos_filter, 6.0, 1.0, 0.1);
		ctrl->pos_filter_initialized = true;
	}
	m_filter_euro_vec3_run(&ctrl->pos_filter, (uint64_t)frame_mono_ns, &pose->position, &filtered_pos);

	struct xrt_pose prev_pose = ctrl->last_tracked_pose;

	ctrl->last_tracked_pose_ts = frame_mono_ns;
	ctrl->last_tracked_pose.position = filtered_pos;
	ctrl->last_tracked_pose.orientation = pose->orientation;

	if (prev_ts != 0 && frame_mono_ns > prev_ts) {
		time_duration_ns dt_ns = frame_mono_ns - prev_ts;
		// Normal inter-frame interval for ~30 FPS camera is ~33ms (allow 15ms to 150ms)
		if (dt_ns >= 15 * U_TIME_1MS_IN_NS && dt_ns <= 150 * U_TIME_1MS_IN_NS) {
			struct xrt_vec3 pos_diff = m_vec3_sub(filtered_pos, prev_pose.position);
			float step_dist = m_vec3_len(pos_diff);
			if (step_dist > 0.20f && dt_ns < 50 * U_TIME_1MS_IN_NS) {
				// Sudden unphysical teleport (> 20cm in < 50ms) - reset velocity to prevent runaway extrapolation
				ctrl->linear_velocity = (struct xrt_vec3){0, 0, 0};
				ctrl->have_linear_velocity = false;
				ctrl->pos_filter_initialized = false;
				ctrl->yaw_trust_count = 0;
				ctrl->yaw_consensus_count = 0;
				ctrl->diag_teleport_count++;
				CTRL_DIAG(
				    "DIAG %s TELEPORT step=%.3fm dtMs=%.1f pos=(%.3f,%.3f,%.3f)",
				    ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER ? "L" : "R", step_dist,
				    (float)dt_ns / (float)U_TIME_1MS_IN_NS, filtered_pos.x, filtered_pos.y,
				    filtered_pos.z);
			} else {
				float dt = (float)dt_ns / (float)U_TIME_1S_IN_NS;
				struct xrt_vec3 inst_vel = {
				    .x = (filtered_pos.x - prev_pose.position.x) / dt,
				    .y = (filtered_pos.y - prev_pose.position.y) / dt,
				    .z = (filtered_pos.z - prev_pose.position.z) / dt,
				};
				// Clamp velocity to human limits (4.0 m/s) to prevent numerical spikes
				float speed = m_vec3_len(inst_vel);
				if (speed > 4.0f) {
					float s = 4.0f / speed;
					inst_vel.x *= s;
					inst_vel.y *= s;
					inst_vel.z *= s;
				}
				if (ctrl->have_linear_velocity) {
					const float alpha = 0.35f;
					ctrl->linear_velocity.x =
					    alpha * inst_vel.x + (1.0f - alpha) * ctrl->linear_velocity.x;
					ctrl->linear_velocity.y =
					    alpha * inst_vel.y + (1.0f - alpha) * ctrl->linear_velocity.y;
					ctrl->linear_velocity.z =
					    alpha * inst_vel.z + (1.0f - alpha) * ctrl->linear_velocity.z;
				} else {
					ctrl->linear_velocity = inst_vel;
					ctrl->have_linear_velocity = true;
				}
			}
		} else if (dt_ns > 150 * U_TIME_1MS_IN_NS) {
			// After tracking loss / freeze, reset velocity so the recovery teleport doesn't cause a spike
			ctrl->linear_velocity = (struct xrt_vec3){0, 0, 0};
			ctrl->have_linear_velocity = false;
			ctrl->pos_filter_initialized = false;
			ctrl->yaw_trust_count = 0;
			ctrl->yaw_consensus_count = 0;
			ctrl->diag_gap_count++;
			CTRL_DIAG("DIAG %s GAP gapMs=%.1f pos=(%.3f,%.3f,%.3f)",
			             ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER ? "L" : "R",
			             (float)dt_ns / (float)U_TIME_1MS_IN_NS, filtered_pos.x, filtered_pos.y,
			             filtered_pos.z);
		}
	}

	if (ctrl->yaw_trust_count < 1000) {
		ctrl->yaw_trust_count++;
	}

	if (ctrl->update_yaw_from_optical) {
		// Only correct yaw if the controller is relatively stationary.
		// When the hand is rotating, the 1000 Hz gyroscope has zero latency
		// whereas optical orientation is 20-30 ms stale. Snapping during motion
		// causes severe rotational hitching.
		// Additionally, only trust optical orientation after ~1.5 s of
		// sustained solves: a bad (re)acquisition solve must not capture
		// the fusion yaw while the hands are held still.
		float gyro_speed = m_vec3_len(ctrl->fusion.last.gyro);
		if (gyro_speed < 0.15f && ctrl->yaw_trust_count >= 45) { // < ~8.6 deg/s
			// Calculate orientation error in WORLD coordinates:
			// q_err = q_optical * q_fusion^-1
			struct xrt_quat fusion_inv;
			math_quat_invert(&ctrl->fusion.rot, &fusion_inv);
			struct xrt_quat q_err;
			math_quat_rotate(&pose->orientation, &fusion_inv, &q_err);

			/* Gate the correction. Gyro yaw drift accumulates at
			 * degrees-per-minute, so a huge disagreement is a bad
			 * optical solve, not drift - following it visibly rotates
			 * still hands wrong. A near-zero yaw component would also
			 * normalize to NaN and poison the fusion permanently. */
			float yw_len_sq = q_err.y * q_err.y + q_err.w * q_err.w;
			bool yaw_ok = yw_len_sq > 1e-12f;
			float yaw_deg = 0.0f;
			float yaw_signed_deg = 0.0f;
			struct xrt_quat q_yaw = XRT_QUAT_IDENTITY;
			if (yaw_ok) {
				// In OpenXR world space, +Y is UP (gravity axis).
				// Extract pure rotation around World Y:
				q_yaw.x = 0.0f;
				q_yaw.y = q_err.y;
				q_yaw.z = 0.0f;
				q_yaw.w = q_err.w;
				math_quat_normalize(&q_yaw);
				float yaw_w = fminf(fmaxf(fabsf(q_yaw.w), 0.0f), 1.0f);
				yaw_deg = 2.0f * acosf(yaw_w) * 57.29578f;
				yaw_signed_deg = 2.0f * atan2f(q_yaw.y, q_yaw.w) * 57.29578f;
				yaw_ok = yaw_deg < 25.0f;
			}
			if (!yaw_ok) {
				ctrl->diag_yaw_bad_count++;
				if (yaw_deg > ctrl->diag_max_yaw_err_deg) {
					ctrl->diag_max_yaw_err_deg = yaw_deg;
				}
				/* Consensus repair: flips oscillate frame to frame,
				 * but a genuinely wrong fusion yaw disagrees with good
				 * optical solves by a STABLE large angle. After 6 still
				 * frames agreeing within 10 degrees, apply the full
				 * correction at once instead of never converging. */
				if (yw_len_sq > 1e-12f) {
					if (ctrl->yaw_consensus_count == 0 ||
					    fabsf(yaw_signed_deg - ctrl->yaw_consensus_mean_deg) < 10.0f) {
						ctrl->yaw_consensus_mean_deg =
						    (ctrl->yaw_consensus_mean_deg *
						         (float)ctrl->yaw_consensus_count +
						     yaw_signed_deg) /
						    (float)(ctrl->yaw_consensus_count + 1);
						ctrl->yaw_consensus_count++;
					} else {
						ctrl->yaw_consensus_mean_deg = yaw_signed_deg;
						ctrl->yaw_consensus_count = 1;
					}
					if (ctrl->yaw_consensus_count >= 6 &&
					    fabsf(ctrl->yaw_consensus_mean_deg) > 25.0f) {
						float half_rad = ctrl->yaw_consensus_mean_deg * 0.5f / 57.29578f;
						struct xrt_quat snap = {0.0f, sinf(half_rad), 0.0f, cosf(half_rad)};
						math_quat_rotate(&snap, &ctrl->fusion.rot, &ctrl->fusion.rot);
						ctrl->diag_yaw_apply_count++;
						ctrl->yaw_consensus_count = 0;
						ctrl->yaw_consensus_mean_deg = 0.0f;
					}
				} else {
					ctrl->yaw_consensus_count = 0;
				}
			} else {
				// Small error: gentle nudge handles it; consensus is for large errors only.
				ctrl->yaw_consensus_count = 0;
				// Apply a gentle complementary filter nudge (2% per 33 ms frame)
				struct xrt_quat correction;
				const struct xrt_quat id = XRT_QUAT_IDENTITY;
				math_quat_slerp(&id, &q_yaw, 0.02f, &correction);

				// Apply correction on the left (in world space):
				// q_fusion_new = correction * q_fusion
				math_quat_rotate(&correction, &ctrl->fusion.rot, &ctrl->fusion.rot);

				ctrl->diag_yaw_apply_count++;
				if (yaw_deg > ctrl->diag_max_yaw_err_deg) {
					ctrl->diag_max_yaw_err_deg = yaw_deg;
				}
			}
		} else {
			ctrl->diag_yaw_skip_count++;
		}
	}
	// Update pose position for the debug UI
	ctrl->pose.position = filtered_pos;

	/* Periodic per-controller summary (every 2 s): the single line that
	 * tells us whether tracking is healthy. IMU rate proves the radio
	 * link, optical rate proves the constellation solver, max query age
	 * proves freshness, |v| + teleport/gap counts prove stability, yaw
	 * counts prove the still-hands correction behaviour, and stillLin
	 * proves the accelerometer frame (expect ~0.00 when still). */
	if (ctrl->diag_win_start_ns == 0) {
		ctrl->diag_win_start_ns = now_mono_ns;
		ctrl->diag_last_summary_ns = now_mono_ns;
	}
	if (now_mono_ns - ctrl->diag_last_summary_ns >= 2 * U_TIME_1S_IN_NS) {
		float win_s =
		    (float)(now_mono_ns - ctrl->diag_win_start_ns) / (float)U_TIME_1S_IN_NS;
		float imu_hz = win_s > 0.0f ? (float)ctrl->diag_imu_count / win_s : 0.0f;
		float opt_hz = win_s > 0.0f ? (float)ctrl->diag_opt_count / win_s : 0.0f;
		float spd = ctrl->have_linear_velocity ? m_vec3_len(ctrl->linear_velocity) : 0.0f;
		CTRL_DIAG(
		    "DIAG %s sum imuHz=%.0f optHz=%.1f maxAgeMs=%.0f pos=(%.3f,%.3f,%.3f) spd=%.2f "
		    "yawApply=%u yawMove=%u yawBad=%u yawMaxDeg=%.1f tele=%u gap=%u nan=%u stale=%u rej=%u stillLin=%.3f",
		    ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER ? "L" : "R", imu_hz, opt_hz,
		    ctrl->diag_max_opt_age_ms, filtered_pos.x, filtered_pos.y, filtered_pos.z, spd,
		    ctrl->diag_yaw_apply_count, ctrl->diag_yaw_skip_count, ctrl->diag_yaw_bad_count,
		    ctrl->diag_max_yaw_err_deg, ctrl->diag_teleport_count, ctrl->diag_gap_count,
		    ctrl->diag_nan_count, ctrl->diag_stale_count, ctrl->diag_reject_count,
		    ctrl->diag_still_lin_accel_avg);
		ctrl->diag_win_start_ns = now_mono_ns;
		ctrl->diag_last_summary_ns = now_mono_ns;
		ctrl->diag_imu_count = 0;
		ctrl->diag_opt_count = 0;
		ctrl->diag_teleport_count = 0;
		ctrl->diag_gap_count = 0;
		ctrl->diag_yaw_apply_count = 0;
		ctrl->diag_yaw_skip_count = 0;
		ctrl->diag_yaw_bad_count = 0;
		ctrl->diag_nan_count = 0;
		ctrl->diag_stale_count = 0;
		ctrl->diag_reject_count = 0;
		ctrl->diag_max_yaw_err_deg = 0.0f;
		ctrl->diag_max_opt_age_ms = 0.0f;
	}
	os_mutex_unlock(&ctrl->mutex);
}

static struct t_constellation_tracked_device_callbacks tracking_callbacks = {
    .get_led_model = rift_s_controller_get_led_model,
    .notify_frame_received = NULL,
    .push_observed_pose = rift_s_controller_push_observed_pose,
};

struct rift_s_controller *
rift_s_controller_create(struct rift_s_system *sys, enum xrt_device_type device_type)
{
	DRV_TRACE_MARKER();

	enum u_device_alloc_flags flags = (enum u_device_alloc_flags)(U_DEVICE_ALLOC_TRACKING_NONE);

	struct rift_s_controller *ctrl = U_DEVICE_ALLOCATE(struct rift_s_controller, flags, INPUT_INDICES_LAST, 1);
	if (ctrl == NULL) {
		return NULL;
	}

	/* Store a ref to the parent hmd, released in destroy */
	rift_s_system_reference(&ctrl->sys, sys);

	os_mutex_init(&ctrl->mutex);

	ctrl->pos_filter_initialized = false;

	u_device_populate_function_pointers(&ctrl->base, rift_s_controller_get_tracked_pose, rift_s_controller_destroy);
	ctrl->base.update_inputs = rift_s_controller_update_inputs;
	ctrl->base.set_output = rift_s_controller_set_output;
	ctrl->base.get_view_poses = u_device_get_view_poses;
	ctrl->base.name = XRT_DEVICE_TOUCH_CONTROLLER;
	ctrl->base.device_type = device_type;

	ctrl->base.supported.orientation_tracking = true;
	ctrl->base.supported.position_tracking = true;

	struct xrt_vec3 axis = {1.0f, 0.0f, 0.0f};
	if (device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		ctrl->device_type = RIFT_S_DEVICE_LEFT_CONTROLLER;
		ctrl->flip_left_yaw = debug_get_bool_option_flip_left_yaw();
		struct xrt_vec3 translation = {0.007f, -0.0018f, 0.102f};
		math_quat_from_angle_vector(DEG_TO_RAD(20.6f), &axis, &ctrl->P_aim_grip.orientation);
		ctrl->P_aim_grip.position = translation;
	} else {
		ctrl->device_type = RIFT_S_DEVICE_RIGHT_CONTROLLER;
		ctrl->flip_left_yaw = false;
		struct xrt_vec3 translation = {-0.007f, -0.0018f, 0.102f};
		math_quat_from_angle_vector(DEG_TO_RAD(20.6f), &axis, &ctrl->P_aim_grip.orientation);
		ctrl->P_aim_grip.position = translation;
	}

	ctrl->pose.orientation.w = 1.0f; // All other values set to zero by U_DEVICE_ALLOCATE (which calls U_CALLOC)
	m_imu_3dof_init(&ctrl->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);

	// Real offset will be updated from the calibration once available
	ctrl->P_imu_device = ctrl->P_device_imu = (struct xrt_pose)XRT_POSE_IDENTITY;

	// Setup inputs and outputs
	if (device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		snprintf(ctrl->base.str, XRT_DEVICE_NAME_LEN, "Oculus Rift S Left Touch Controller");
		snprintf(ctrl->base.serial, XRT_DEVICE_NAME_LEN, "Left Controller");
		SET_TOUCH_INPUT(ctrl, X_CLICK);
		SET_TOUCH_INPUT(ctrl, X_TOUCH);
		SET_TOUCH_INPUT(ctrl, Y_CLICK);
		SET_TOUCH_INPUT(ctrl, Y_TOUCH);
		SET_TOUCH_INPUT(ctrl, MENU_CLICK);
	} else {
		snprintf(ctrl->base.str, XRT_DEVICE_NAME_LEN, "Oculus Rift S Right Touch Controller");
		snprintf(ctrl->base.serial, XRT_DEVICE_NAME_LEN, "Right Controller");
		SET_TOUCH_INPUT(ctrl, A_CLICK);
		SET_TOUCH_INPUT(ctrl, A_TOUCH);
		SET_TOUCH_INPUT(ctrl, B_CLICK);
		SET_TOUCH_INPUT(ctrl, B_TOUCH);
		SET_TOUCH_INPUT(ctrl, SYSTEM_CLICK);
	}

	SET_TOUCH_INPUT(ctrl, SQUEEZE_VALUE);
	SET_TOUCH_INPUT(ctrl, TRIGGER_TOUCH);
	SET_TOUCH_INPUT(ctrl, TRIGGER_VALUE);
	SET_TOUCH_INPUT(ctrl, THUMBSTICK_CLICK);
	SET_TOUCH_INPUT(ctrl, THUMBSTICK_TOUCH);
	SET_TOUCH_INPUT(ctrl, THUMBSTICK);
	SET_TOUCH_INPUT(ctrl, THUMBREST_TOUCH);
	SET_TOUCH_INPUT(ctrl, GRIP_POSE);
	SET_TOUCH_INPUT(ctrl, AIM_POSE);

	ctrl->base.outputs[0].name = XRT_OUTPUT_NAME_TOUCH_HAPTIC;

	ctrl->base.binding_profiles = binding_profiles_rift_s;
	ctrl->base.binding_profile_count = ARRAY_SIZE(binding_profiles_rift_s);

	u_var_add_root(ctrl, ctrl->base.str, true);
	u_var_add_gui_header(ctrl, NULL, "Tracking");
	u_var_add_pose(ctrl, &ctrl->pose, "Tracked Pose");

	u_var_add_pose(ctrl, &ctrl->P_aim_grip, "Grip pose offset");
	if (device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		u_var_add_bool(ctrl, &ctrl->flip_left_yaw, "Flip Left Yaw (180 deg)");
	}

	u_var_add_gui_header(ctrl, NULL, "3DoF Tracking");
	m_imu_3dof_add_vars(&ctrl->fusion, ctrl, "");

	u_var_add_gui_header(ctrl, NULL, "Controls");
	if (device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		DEBUG_TOUCH_INPUT_BOOL(ctrl, X_CLICK, "X button");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, X_TOUCH, "X button touch");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, Y_CLICK, "Y button");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, Y_TOUCH, "Y button touch");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, MENU_CLICK, "Menu button");
	} else {
		DEBUG_TOUCH_INPUT_BOOL(ctrl, A_CLICK, "A button");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, A_TOUCH, "A button touch");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, B_CLICK, "B button");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, B_TOUCH, "B button touch");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, SYSTEM_CLICK, "Oculus button");
	}

	DEBUG_TOUCH_INPUT_F32(ctrl, SQUEEZE_VALUE, "Grip value");

	DEBUG_TOUCH_INPUT_BOOL(ctrl, TRIGGER_TOUCH, "Trigger touch");
	DEBUG_TOUCH_INPUT_F32(ctrl, TRIGGER_VALUE, "Trigger");
	DEBUG_TOUCH_INPUT_BOOL(ctrl, THUMBSTICK_CLICK, "Thumbstick click");
	DEBUG_TOUCH_INPUT_BOOL(ctrl, THUMBSTICK_TOUCH, "Thumbstick touch");
	DEBUG_TOUCH_INPUT_VEC2(ctrl, THUMBSTICK, "Thumbstick X", "Thumbstick Y");
	DEBUG_TOUCH_INPUT_BOOL(ctrl, THUMBREST_TOUCH, "Thumbrest touch");

	struct rift_s_tracker *tracker = rift_s_system_get_tracker(sys);
	ctrl->update_yaw_from_optical = true;
	ctrl->tracking_connection = rift_s_tracker_add_controller(tracker, &ctrl->base, &tracking_callbacks);
	return ctrl;
}

#define RIFT_S_CONFIG_RETRY_INTERVAL_NS (1 * U_TIME_1S_IN_NS)

void
rift_s_controller_update_configuration(struct rift_s_controller *ctrl, uint64_t device_id)
{
	rift_s_radio_state *radio = rift_s_system_radio(ctrl->sys);
	timepoint_ns now = os_monotonic_get_ns();

	if (ctrl->device_id != device_id) {
		ctrl->device_id = device_id;
		snprintf(ctrl->base.serial, XRT_DEVICE_NAME_LEN, "%016" PRIx64, device_id);
		// If the device ID changed somehow, re-read the JSON blocks
		ctrl->have_config = ctrl->have_calibration = false;
		ctrl->reading_config = ctrl->reading_calibration = false;
		ctrl->last_config_attempt_ns = 0;
		ctrl->last_calibration_attempt_ns = 0;
	}

	if (!ctrl->have_config && !ctrl->reading_config) {
		if (ctrl->last_config_attempt_ns == 0 ||
		    (now - ctrl->last_config_attempt_ns) >= RIFT_S_CONFIG_RETRY_INTERVAL_NS) {
			const uint8_t config_req[] = {0x32, 0x20, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
			ctrl->last_config_attempt_ns = now;
			ctrl->reading_config = true;
			rift_s_radio_queue_command(radio, ctrl->device_id, config_req, sizeof(config_req),
			                           (rift_s_radio_completion_fn)ctrl_config_cb, ctrl);
		}
	}

	if (!ctrl->have_calibration && !ctrl->reading_calibration) {
		if (ctrl->last_calibration_attempt_ns == 0 ||
		    (now - ctrl->last_calibration_attempt_ns) >= RIFT_S_CONFIG_RETRY_INTERVAL_NS) {
			ctrl->last_calibration_attempt_ns = now;
			ctrl->reading_calibration = true;
			rift_s_radio_get_json_block(radio, ctrl->device_id, (rift_s_radio_completion_fn)ctrl_json_cb, ctrl);
		}
	}
}

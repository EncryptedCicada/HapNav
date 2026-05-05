/*
 * HapNav obstacle-detection pipeline (pin-side).
 *
 * Stages:
 *   1.  per-pixel validity mask (status flag + range bounds)
 *   2.  pre-computed sensor-frame ray table, downtilt baked in
 *   3.  rotate every ray into world frame using the Madgwick quaternion
 *   4.  classify zones as floor / ceiling / candidate (height-banded)
 *   5.  per-azimuth-bin nearest-obstacle search with a small support test
 *   6.  closing-rate estimate per bin (frame-to-frame Δrange)
 *   7.  urgency = max(proximity_score, ttc_score), gated by stationary /
 *       yaw-slew / sensor-blocked / mostly-invalid flags
 *   8.  drop-off check (separate module — sees the same world-frame rays)
 */
#ifndef HAPNAV_OBSTACLE_H_
#define HAPNAV_OBSTACLE_H_

#include <hapnav/ble_proto.h>
#include <stdint.h>

void hapnav_obstacle_init(void);

/*
 * @quat_wxyz   body→world unit quaternion from Madgwick
 * @gyro_radps  body-frame angular rates [x, y, z] in rad/s
 * @accel_g     body-frame acceleration [x, y, z] in g (incl. gravity)
 * @dt_s        seconds since the previous call (≈ sample period)
 * @out         filled in place; never NULL
 */
void hapnav_obstacle_step(const int16_t distances_mm[HAPNAV_TOF_ZONES],
			  const uint8_t target_status[HAPNAV_TOF_ZONES],
			  const float   quat_wxyz[4],
			  const float   gyro_radps[3],
			  const float   accel_g[3],
			  float         dt_s,
			  struct hapnav_obstacles *out);

#endif /* HAPNAV_OBSTACLE_H_ */

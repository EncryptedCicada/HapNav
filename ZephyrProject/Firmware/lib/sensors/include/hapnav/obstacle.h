/*
 * HapNav obstacle-detection pipeline (pin-side).
 *
 * The VL53L5CX is physically rolled 45° around its optical axis, so the
 * original 8×8 grid sits as a diamond in the world frame. The pipeline
 * applies the same R_z(45°) to its body-frame ray table; classification
 * (floor / ceiling / obstacle) and the 4-bin lateral urgency mapping then
 * fall out of the post-rotation directions. See
 * Docs/Implementation/05_obstacle_detection.md for the geometry.
 *
 * Stages:
 *   1.  per-pixel validity mask (status flag + range bounds)
 *   2.  pre-computed sensor-frame ray table with 45° roll and downtilt baked in
 *   3.  rotate every ray into world frame using the Madgwick quaternion
 *   4.  classify zones as floor / ceiling / candidate (height-banded)
 *   5.  per-azimuth-bin nearest-obstacle search with a small support test
 *   6.  closing-rate estimate per bin (frame-to-frame Δrange)
 *   7.  urgency = max(proximity_score, ttc_score), gated by stationary /
 *       yaw-slew / sensor-blocked / mostly-invalid flags
 *   8.  drop-off check (separate module — sees the same world-frame rays)
 *   9.  head-clearance check from the VL53L1X single-zone ToF
 */
#ifndef HAPNAV_OBSTACLE_H_
#define HAPNAV_OBSTACLE_H_

#include <hapnav/ble_proto.h>
#include <stdint.h>

void hapnav_obstacle_init(void);

/*
 * @quat_wxyz             body→world unit quaternion from Madgwick
 * @gyro_radps            body-frame angular rates [x, y, z] in rad/s
 * @accel_g               body-frame acceleration [x, y, z] in g (incl. gravity)
 * @head_distance_mm      latest VL53L1X reading in mm, or -1 if unavailable.
 *                        When in (0, @head_proximity_max_mm] the pipeline
 *                        raises HAPNAV_OBS_FLAG_HEAD_OBSTACLE.
 * @head_proximity_max_mm slant-range threshold for the head-clearance flag.
 * @dt_s                  seconds since the previous call (≈ sample period)
 * @out                   filled in place; never NULL
 */
void hapnav_obstacle_step(const int16_t distances_mm[HAPNAV_TOF_ZONES],
			  const uint8_t target_status[HAPNAV_TOF_ZONES],
			  const float   quat_wxyz[4],
			  const float   gyro_radps[3],
			  const float   accel_g[3],
			  int16_t       head_distance_mm,
			  int16_t       head_proximity_max_mm,
			  float         dt_s,
			  struct hapnav_obstacles *out);

#endif /* HAPNAV_OBSTACLE_H_ */

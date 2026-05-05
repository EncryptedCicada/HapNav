/*
 * Drop-off detection — given the world-frame rays for the 8x8 grid, decide
 * whether the floor where it should be is missing. Stateless; the obstacle
 * pipeline owns the ray table and calls this each frame.
 */
#ifndef HAPNAV_DROPOFF_H_
#define HAPNAV_DROPOFF_H_

#include <hapnav/ble_proto.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * @ray_W            unit ray for each zone, expressed in gravity-aligned
 *                   world frame (Z = up). Computed from R_WB · ray_B by
 *                   the obstacle pipeline.
 * @sensor_height_m  vertical distance from sensor to the (assumed flat)
 *                   floor — positive value, e.g. 1.35 m.
 *
 * Returns true when ≥ MIN_DROPOFF_PIXELS of the rays that *should* hit floor
 * inside our operating range read either invalid status or much longer slant
 * range than the predicted floor intercept.
 */
bool hapnav_dropoff_check(const int16_t distances_mm[HAPNAV_TOF_ZONES],
			  const uint8_t target_status[HAPNAV_TOF_ZONES],
			  const float   ray_W[HAPNAV_TOF_ZONES][3],
			  float         sensor_height_m);

#endif /* HAPNAV_DROPOFF_H_ */

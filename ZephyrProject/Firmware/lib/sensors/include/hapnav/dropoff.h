/*
 * Drop-off detection from a single forward-and-down-pointing VL53L1X.
 *
 * The sensor is mounted at a fixed downtilt (HAPNAV_DOWN_TOF_DOWNTILT_DEG —
 * 45°). On flat floor at the worn pose (sensor ~1.35 m above the ground) the
 * slant range is sensor_height / cos(downtilt) ≈ 1.91 m. A cliff edge ahead
 * of the user makes that range jump; a gentle slope shifts it gradually.
 *
 * Detection uses change-detection against a slow-tracked baseline rather than
 * an absolute height target, so gradual inclines/declines (and slow drift in
 * the user's posture) don't trigger:
 *
 *   baseline_mm  ← slow EMA of slant_mm (alpha ~ 0.1 per frame)
 *   residual_mm  = slant_mm − baseline_mm
 *   flag         = residual_mm > DROPOFF_THRESH_MM for DROPOFF_MIN_STREAK frames
 *
 * No state is leaked across boots; the caller owns one struct per sensor
 * instance and initialises it with hapnav_dropoff_init().
 */
#ifndef HAPNAV_DROPOFF_H_
#define HAPNAV_DROPOFF_H_

#include <stdint.h>
#include <stdbool.h>

/* Sensor tilt below horizontal. Constant for now; future versions can let
 * the caller pass it in to support reconfiguration without a rebuild. */
#define HAPNAV_DOWN_TOF_DOWNTILT_DEG  45.0f

struct hapnav_dropoff_state {
	float baseline_mm;     /* slow EMA of the slant-range readings */
	bool  baseline_valid;
	int   streak;          /* consecutive frames over threshold */
};

void hapnav_dropoff_init(struct hapnav_dropoff_state *s);

/*
 * Update the detector with one fresh slant reading from the down ToF.
 *
 *   slant_mm        — VL53L1X RangeMilliMeter, or -1 if unavailable/invalid.
 *                     The caller is responsible for pre-filtering by the
 *                     sensor's range status (we don't see that here).
 *   sensor_height_m — present-day worn-pose mount height (unused for now;
 *                     reserved for an absolute-mode fallback).
 *   pitch_rad       — user pitch from the BNO055 (unused for now; reserved
 *                     for compensating "leaning forward" against the EMA).
 *
 * Returns true if the dropoff condition has held for DROPOFF_MIN_STREAK
 * consecutive frames.
 */
bool hapnav_dropoff_check_down(struct hapnav_dropoff_state *s,
			       int16_t slant_mm,
			       float   sensor_height_m,
			       float   pitch_rad);

#endif /* HAPNAV_DROPOFF_H_ */

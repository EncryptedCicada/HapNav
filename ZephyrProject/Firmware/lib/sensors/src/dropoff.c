#include <hapnav/dropoff.h>

#include <math.h>

/* Require ray to point at least ~11.5° below horizontal in world frame
 * (sin ≈ 0.20) to count as a "ground ray". With the user upright and 8°
 * sensor downtilt, three bottom rows clear this; pitching down adds more,
 * pitching up drops some — both are correct behaviours. */
#define MIN_FLOOR_RAY_DOWN_SIN  0.20f

/* Predicted floor slant-range cutoff. Past this we'd be reasoning about
 * floor outside our 3 m operating range, where missing returns are
 * uninformative. */
#define PREDICTED_RANGE_CUTOFF_M  3.5f

/* Measured-vs-predicted ratio above which we judge the floor is gone. */
#define DROPOFF_RANGE_RATIO  1.30f

/* Need at least this many bottom-row pixels both eligible as floor rays
 * AND missing returns, before we cry drop-off. Suppresses single-pixel
 * glitches and reflective-floor anomalies. */
#define MIN_DROPOFF_PIXELS  3

bool hapnav_dropoff_check(const int16_t distances_mm[HAPNAV_TOF_ZONES],
			  const uint8_t target_status[HAPNAV_TOF_ZONES],
			  const float   ray_W[HAPNAV_TOF_ZONES][3],
			  float         sensor_height_m)
{
	int floor_ray_pixels = 0;
	int dropoff_pixels   = 0;

	for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
		float rz = ray_W[i][2];   /* world Z; negative = pointing down */
		if (rz > -MIN_FLOOR_RAY_DOWN_SIN) {
			continue;          /* not enough downward tilt */
		}

		/* Slant range from sensor to a flat floor at z = -sensor_height_m. */
		float predicted = sensor_height_m / (-rz);
		if (predicted > PREDICTED_RANGE_CUTOFF_M) {
			continue;          /* floor would be outside useful range */
		}

		floor_ray_pixels++;

		uint8_t s = target_status[i];
		bool status_ok = (s == 5 || s == 6 || s == 9);

		bool floor_missing;
		if (!status_ok) {
			/* No reliable return where floor should be. */
			floor_missing = true;
		} else {
			float r_m = distances_mm[i] * 0.001f;
			floor_missing = (r_m > predicted * DROPOFF_RANGE_RATIO);
		}

		if (floor_missing) {
			dropoff_pixels++;
		}
	}

	return (floor_ray_pixels >= MIN_DROPOFF_PIXELS &&
		dropoff_pixels    >= MIN_DROPOFF_PIXELS);
}

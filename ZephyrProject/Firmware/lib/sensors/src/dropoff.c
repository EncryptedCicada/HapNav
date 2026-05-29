#include <hapnav/dropoff.h>

#include <math.h>

/* Require ray to point at least ~11.5° below horizontal in world frame
 * (sin ≈ 0.20) to count as a "ground ray". With the user upright and 8°
 * sensor downtilt, three bottom rows clear this; pitching down adds more,
 * pitching up drops some — both are correct behaviours. */
#define MIN_FLOOR_RAY_DOWN_SIN  0.20f

/* Predicted floor slant-range cutoff. Past this we'd be reasoning about
 * floor outside our useful operating range, where missing returns are
 * uninformative. Bench-mode scales this down to a 50 cm envelope so the
 * geometry matches the BENCH_MODE constants in obstacle.c. */
#if defined(CONFIG_HAPNAV_BENCH_MODE)
#define PREDICTED_RANGE_CUTOFF_M  0.5f
#else
#define PREDICTED_RANGE_CUTOFF_M  3.5f
#endif

/* Measured-vs-predicted ratio above which we judge the floor is gone. */
#define DROPOFF_RANGE_RATIO  1.30f

/* How many pixels both eligible as floor rays AND missing the expected
 * return we need before flagging DROPOFF, and whether invalid-status
 * pixels count as "missing". Bench setups are noisy (lots of status=255
 * even on a healthy desk) so we raise the bar and require an actual
 * valid-but-too-far reading rather than counting invalid pixels. */
#if defined(CONFIG_HAPNAV_BENCH_MODE)
#define MIN_DROPOFF_PIXELS      8
#define DROPOFF_COUNT_INVALID   false
#else
#define MIN_DROPOFF_PIXELS      3
#define DROPOFF_COUNT_INVALID   true
#endif

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

		bool floor_missing = false;
		if (!status_ok) {
			/* Bench mode: invalid pixels are noise, not cliff. */
			floor_missing = DROPOFF_COUNT_INVALID;
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

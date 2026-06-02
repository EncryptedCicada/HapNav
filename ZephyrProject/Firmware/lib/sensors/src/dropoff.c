#include <hapnav/dropoff.h>

/* EMA alpha for the baseline, expressed as a small rational so we don't
 * pull a libm divide in for a single line. 0.1/frame at 20 Hz gives a
 * time-constant of ~0.5 s — fast enough to track posture and slope drift,
 * slow enough that a cliff edge stays clear of it long enough to flag. */
#define BASELINE_ALPHA_NUM    1
#define BASELINE_ALPHA_DEN   10

/* Slant range jumping >= 200 mm above the baseline = floor gone. */
#define DROPOFF_THRESH_MM    200

/* 3 consecutive frames at 20 Hz = 150 ms — long enough to ride out a
 * noisy reading, short enough to fire before the foot lands. */
#define DROPOFF_MIN_STREAK   3

/* Past this slant the VL53L1X default mode isn't trustworthy; treat as
 * no-update rather than as "saw a cliff". */
#define MAX_TRUSTED_MM      3500

void hapnav_dropoff_init(struct hapnav_dropoff_state *s)
{
	s->baseline_mm    = 0.0f;
	s->baseline_valid = false;
	s->streak         = 0;
}

bool hapnav_dropoff_check_down(struct hapnav_dropoff_state *s,
			       int16_t slant_mm,
			       float   sensor_height_m,
			       float   pitch_rad)
{
	(void)sensor_height_m;   /* reserved for future absolute-mode fallback */
	(void)pitch_rad;         /* reserved for posture compensation */

	if (slant_mm < 0 || slant_mm > MAX_TRUSTED_MM) {
		s->streak = 0;
		return false;
	}

	float meas = (float)slant_mm;

	if (!s->baseline_valid) {
		s->baseline_mm    = meas;
		s->baseline_valid = true;
		s->streak         = 0;
		return false;
	}

	float residual = meas - s->baseline_mm;

	if (residual > (float)DROPOFF_THRESH_MM) {
		/* Floor disappeared — keep the streak running and freeze the
		 * baseline so it doesn't drift up to meet the cliff. */
		s->streak++;
	} else {
		s->streak = 0;
		s->baseline_mm +=
			(meas - s->baseline_mm) *
			BASELINE_ALPHA_NUM / (float)BASELINE_ALPHA_DEN;
	}

	return (s->streak >= DROPOFF_MIN_STREAK);
}

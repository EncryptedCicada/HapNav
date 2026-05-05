/*
 * Madgwick AHRS — orientation filter fusing accel + gyro + mag into a unit
 * quaternion. Single-precision; no allocations; no globals.
 *
 * Convention: quaternion is [w, x, y, z]; gyro in rad/s; accel and mag in
 * any consistent units (only their ratio matters); dt in seconds.
 *
 * `beta` is the filter gain (≈ sqrt(3/4) * gyro_drift_rate). 0.1f is a
 * reasonable starting point for a moderately drifty consumer-grade IMU.
 */
#ifndef HAPNAV_MADGWICK_H_
#define HAPNAV_MADGWICK_H_

struct madgwick {
	float q[4];   /* [w, x, y, z], starts at identity */
	float beta;
};

void madgwick_init(struct madgwick *m, float beta);

void madgwick_update(struct madgwick *m,
		     float ax, float ay, float az,
		     float gx, float gy, float gz,
		     float mx, float my, float mz,
		     float dt);

#endif /* HAPNAV_MADGWICK_H_ */

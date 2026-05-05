/*
 * Madgwick AHRS (S. O. H. Madgwick, 2010), MARG variant.
 * Adapted to single-precision and [w,x,y,z] quaternion ordering.
 */
#include <hapnav/madgwick.h>
#include <math.h>
#include <stdbool.h>

void madgwick_init(struct madgwick *m, float beta)
{
	m->q[0] = 1.0f;
	m->q[1] = 0.0f;
	m->q[2] = 0.0f;
	m->q[3] = 0.0f;
	m->beta = beta;
}

static inline float inv_sqrt(float x)
{
	return 1.0f / sqrtf(x);
}

void madgwick_update(struct madgwick *m,
		     float ax, float ay, float az,
		     float gx, float gy, float gz,
		     float mx, float my, float mz,
		     float dt)
{
	float q0 = m->q[0], q1 = m->q[1], q2 = m->q[2], q3 = m->q[3];

	/* Gyro-only rate of change of quaternion. */
	float qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
	float qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
	float qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
	float qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

	/* Skip accel/mag correction if either is degenerate (free-fall, mag dropout). */
	bool acc_ok = !(ax == 0.0f && ay == 0.0f && az == 0.0f);
	bool mag_ok = !(mx == 0.0f && my == 0.0f && mz == 0.0f);

	if (acc_ok && mag_ok) {
		float recipNorm;

		recipNorm = inv_sqrt(ax*ax + ay*ay + az*az);
		ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

		recipNorm = inv_sqrt(mx*mx + my*my + mz*mz);
		mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

		/* Reference direction of Earth's magnetic field. */
		float _2q0mx = 2.0f * q0 * mx;
		float _2q0my = 2.0f * q0 * my;
		float _2q0mz = 2.0f * q0 * mz;
		float _2q1mx = 2.0f * q1 * mx;
		float hx = mx*q0*q0 - _2q0my*q3 + _2q0mz*q2 + mx*q1*q1
			   + 2.0f*q1*my*q2 + 2.0f*q1*mz*q3 - mx*q2*q2 - mx*q3*q3;
		float hy = _2q0mx*q3 + my*q0*q0 - _2q0mz*q1 + _2q1mx*q2
			   - my*q1*q1 + my*q2*q2 + 2.0f*q2*mz*q3 - my*q3*q3;
		float _2bx = sqrtf(hx*hx + hy*hy);
		float _2bz = -_2q0mx*q2 + _2q0my*q1 + mz*q0*q0 + _2q1mx*q3
			     - mz*q1*q1 + 2.0f*q2*my*q3 - mz*q2*q2 + mz*q3*q3;
		float _4bx = 2.0f * _2bx;
		float _4bz = 2.0f * _2bz;

		/* Gradient descent step (objective: align measured field & gravity
		 * with predicted directions from the current orientation). */
		float s0 = -2.0f*q2*(2.0f*q1*q3 - 2.0f*q0*q2 - ax)
			   + 2.0f*q1*(2.0f*q0*q1 + 2.0f*q2*q3 - ay)
			   - _2bz*q2*(_2bx*(0.5f - q2*q2 - q3*q3) + _2bz*(q1*q3 - q0*q2) - mx)
			   + (-_2bx*q3 + _2bz*q1)*(_2bx*(q1*q2 - q0*q3) + _2bz*(q0*q1 + q2*q3) - my)
			   + _2bx*q2*(_2bx*(q0*q2 + q1*q3) + _2bz*(0.5f - q1*q1 - q2*q2) - mz);
		float s1 = 2.0f*q3*(2.0f*q1*q3 - 2.0f*q0*q2 - ax)
			   + 2.0f*q0*(2.0f*q0*q1 + 2.0f*q2*q3 - ay)
			   - 4.0f*q1*(1.0f - 2.0f*q1*q1 - 2.0f*q2*q2 - az)
			   + _2bz*q3*(_2bx*(0.5f - q2*q2 - q3*q3) + _2bz*(q1*q3 - q0*q2) - mx)
			   + (_2bx*q2 + _2bz*q0)*(_2bx*(q1*q2 - q0*q3) + _2bz*(q0*q1 + q2*q3) - my)
			   + (_2bx*q3 - _4bz*q1)*(_2bx*(q0*q2 + q1*q3) + _2bz*(0.5f - q1*q1 - q2*q2) - mz);
		float s2 = -2.0f*q0*(2.0f*q1*q3 - 2.0f*q0*q2 - ax)
			   + 2.0f*q3*(2.0f*q0*q1 + 2.0f*q2*q3 - ay)
			   - 4.0f*q2*(1.0f - 2.0f*q1*q1 - 2.0f*q2*q2 - az)
			   + (-_4bx*q2 - _2bz*q0)*(_2bx*(0.5f - q2*q2 - q3*q3) + _2bz*(q1*q3 - q0*q2) - mx)
			   + (_2bx*q1 + _2bz*q3)*(_2bx*(q1*q2 - q0*q3) + _2bz*(q0*q1 + q2*q3) - my)
			   + (_2bx*q0 - _4bz*q2)*(_2bx*(q0*q2 + q1*q3) + _2bz*(0.5f - q1*q1 - q2*q2) - mz);
		float s3 = 2.0f*q1*(2.0f*q1*q3 - 2.0f*q0*q2 - ax)
			   + 2.0f*q2*(2.0f*q0*q1 + 2.0f*q2*q3 - ay)
			   + (-_4bx*q3 + _2bz*q1)*(_2bx*(0.5f - q2*q2 - q3*q3) + _2bz*(q1*q3 - q0*q2) - mx)
			   + (-_2bx*q0 + _2bz*q2)*(_2bx*(q1*q2 - q0*q3) + _2bz*(q0*q1 + q2*q3) - my)
			   + _2bx*q1*(_2bx*(q0*q2 + q1*q3) + _2bz*(0.5f - q1*q1 - q2*q2) - mz);

		recipNorm = inv_sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
		s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

		qDot1 -= m->beta * s0;
		qDot2 -= m->beta * s1;
		qDot3 -= m->beta * s2;
		qDot4 -= m->beta * s3;
	}

	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	float recipNorm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	m->q[0] = q0 * recipNorm;
	m->q[1] = q1 * recipNorm;
	m->q[2] = q2 * recipNorm;
	m->q[3] = q3 * recipNorm;
}

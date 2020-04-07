#include "MadgwickAHRS.hpp"

namespace madgwickahrs{

MadgwickAHRS::MadgwickAHRS(float samplePeriod, float beta){
	SamplePeriod = samplePeriod;
	Beta = beta;
	Quaternion[0] = 1; Quaternion[1] = 0; Quaternion[2] = 0; Quaternion[3] = 0;
}

MadgwickAHRS::~MadgwickAHRS(){

}

/*
        /// Algorithm AHRS update method. Requires gyroscope, accelerometer and magnetometer data.
        /// <param name="gx">
        /// Gyroscope x axis measurement in radians/s.

        /// <param name="gy">
        /// Gyroscope y axis measurement in radians/s.

        /// <param name="gz">
        /// Gyroscope z axis measurement in radians/s.

        /// <param name="ax">
        /// Accelerometer x axis measurement in any calibrated units.

        /// <param name="ay">
        /// Accelerometer y axis measurement in any calibrated units.

        /// <param name="az">
        /// Accelerometer z axis measurement in any calibrated units.

        /// <param name="mx">
        /// Magnetometer x axis measurement in any calibrated units.

        /// <param name="my">
        /// Magnetometer y axis measurement in any calibrated units.

        /// <param name="mz">
        /// Magnetometer z axis measurement in any calibrated units.

        /// Optimised for minimal arithmetic.
        /// Total ±: 160
        /// Total *: 172
        /// Total /: 5
        /// Total sqrt: 5
 */
void MadgwickAHRS::Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float q1 = Quaternion[0], q2 = Quaternion[1], q3 = Quaternion[2], q4 = Quaternion[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _8bx;
	float _8bz;
	float _2q1 = 2.0 * q1;
	float _2q2 = 2.0 * q2;
	float _2q3 = 2.0 * q3;
	float _2q4 = 2.0 * q4;
	float _2q1q3 = 2.0 * q1 * q3;
	float _2q3q4 = 2.0 * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalize accelerometer measurement
	norm = (float)std::sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = (float)std::sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0 * q1 * mx;
	_2q1my = 2.0 * q1 * my;
	_2q1mz = 2.0 * q1 * mz;
	_2q2mx = 2.0 * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = (float)std::sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0 * _2bx;
	_4bz = 2.0 * _2bz;
	_8bx = 2.0 * _4bx;
	_8bz = 2.0 * _4bz;

	// Gradient decent algorithm corrective step
	/*s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	 */
	s1= -_2q3*(2.0*(q2q4 - q1q3) - ax) + _2q2*(2.0*(q1q2 + q3q4) - ay) + (-_4bz)*q3*(_4bx*(0.5 - q3q3 - q4q4) + _4bz*(q2q4 - q1q3) - mx) + (-_4bx*q4+_4bz*q2)*(_4bx*(q2q3 - q1q4) + _4bz*(q1q2 + q3q4) - my) + _4bx*q3*(_4bx*(q1q3 + q2q4) + _4bz*(0.5 - q2q2 - q3q3) - mz);
	s2= _2q4*(2*(q2q4 - q1q3) - ax) + _2q1*(2.0*(q1q2 + q3q4) - ay) - 4.0*q2*(2.0*(0.5 - q2q2 - q3q3) - az) + _4bz*q4*(_4bx*(0.5 - q3q3 - q4q4) + _4bz*(q2q4 - q1q3) - mx) + (_4bx*q3+_4bz*q1)*(_4bx*(q2q3 - q1q4) + _4bz*(q1q2 + q3q4) - my) + (_4bx*q4-_8bz*q2)*(_4bx*(q1q3 + q2q4) + _4bz*(0.5 - q2q2 - q3q3) - mz);
	s3= -_2q1*(2.0*(q2q4 - q1q3) - ax) + _2q4*(2.0*(q1q2 + q3q4) - ay) + (-4.0*q3)*(2.0*(0.5 - q2q2 - q3q3) - az) + (-_8bx*q3-_4bz*q1)*(_4bx*(0.5 - q3q3 - q4q4) + _4bz*(q2q4 - q1q3) - mx)+(_4bx*q2+_4bz*q4)*(_4bx*(q2q3 - q1q4) + _4bz*(q1q2 + q3q4) - my)+(_4bx*q1-_8bz*q3)*(_4bx*(q1q3 + q2q4) + _4bz*(0.5 - q2q2 - q3q3) - mz);
	s4= _2q2*(2.0*(q2q4 - q1q3) - ax) + _2q3*(2.0*(q1q2 + q3q4) - ay)+(-_8bx*q4+_4bz*q2)*(_4bx*(0.5 - q3q3 - q4q4) + _4bz*(q2q4 - q1q3) - mx)+(-_4bx*q1+_4bz*q3)*(_4bx*(q2q3 - q1q4) + _4bz*(q1q2 + q3q4) - my)+(_4bx*q2)*(_4bx*(q1q3 + q2q4) + _4bz*(0.5 - q2q2 - q3q3) - mz);


	norm = 1.0 / (float)std::sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
	qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
	qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
	qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * SamplePeriod;
	q2 += qDot2 * SamplePeriod;
	q3 += qDot3 * SamplePeriod;
	q4 += qDot4 * SamplePeriod;
	norm = 1.0 / (float)std::sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	Quaternion[0] = q1 * norm;
	Quaternion[1] = q2 * norm;
	Quaternion[2] = q3 * norm;
	Quaternion[3] = q4 * norm;
}

}

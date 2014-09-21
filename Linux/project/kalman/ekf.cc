#include <iostream>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <unistd.h>
#include <signal.h>
#include <fstream>
#include "Eigen/Dense"
#include "KalmanFilter.h"
#include "LinuxDARwIn.h"

using namespace Robot;

double diff_sec(timespec start, timespec end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return (double)(temp.tv_sec + temp.tv_nsec / 1000000000.0);
}

// gyro & accel
double gyro_radps(int gyro) {
	return (gyro-512)*0.017453229251;
}

double accel_ms2(int accel) {
	return ((accel-512) / 128.0) * 9.81; // in m/s^2
}

double twoKp = 2.0;
double twoKi = 0.0;

//TODO dt should be hz
void MahonyAHRSupdateIMU(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
	double recipNorm;
	double halfvx, halfvy, halfvz;
	double halfex, halfey, halfez;
	double qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		//recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		recipNorm = 1.0/sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / dt);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / dt);
			integralFBz += twoKi * halfez * (1.0f / dt);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / dt));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / dt));
	gz *= (0.5f * (1.0f / dt));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 

	// Normalise quaternion
	//recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	recipNorm = 1.0/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


int main(int argc, char **argv) {
	std::ifstream in("in2.txt");
	std::ofstream out("out2.txt");
	Kalman<double, 6> k;
	
	int dim = 6;
	typedef SimpleState<dim> state_t; // state should be angles and ang_vels?
	typedef ExtendedProcess<dim, state_t> process_t;
	AbsoluteMeasurement<state_t> meas;

	KalmanFilter<state_t, process_t> filt;

	filt.processModel.sigma = state_t::VecState::Constant(0.01);
	filt.jacobian<<


	//////////////////// Darwin Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);
	if(cm730.Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return 0;
	}

	// Disable all motors for safety
	for (int id=JointData::ID_R_SHOULDER_PITCH; id<=JointData::ID_HEAD_TILT; id++) {
		cm730.WriteWord(id, MX28::P_TORQUE_ENABLE, 0, 0);
	}	
	///////////////////////////////////////////////////////////////////

	// Init Time
	static struct timespec start_time;
	static struct timespec interval;
	static struct timespec prev_int;


	// Init Gyro & Accel
	unsigned char table[CM730::MAXNUM_ADDRESS];

	double gyro_z, gyro_y, gyro_x, accel_z, accel_y, accel_x;
	double time;

	Eigen::Matrix<double, 6, 1> measurement;
	Eigen::Matrix<double, 6, 1> stateEstimate;

	//////////////////// Begin Collection /////////////////////////////

	clock_gettime(CLOCK_MONOTONIC, &start_time);
	clock_gettime(CLOCK_MONOTONIC, &prev_int);

	while (1) {
		// reads a chunk of data into table to be formatted after
		if(cm730.ReadTable(CM730::ID_CM, CM730::P_GYRO_Z_L, CM730::P_ACCEL_Z_H, table, 0) == CM730::SUCCESS)
		{
			// Darwin Time
			clock_gettime(CLOCK_MONOTONIC, &interval);
			time = diff_sec(start_time, interval);

			// Gyro and Accel
			gyro_z = gyro_radps(cm730.MakeWord(table[CM730::P_GYRO_Z_L], table[CM730::P_GYRO_Z_H]));
			gyro_y = gyro_radps(cm730.MakeWord(table[CM730::P_GYRO_Y_L], table[CM730::P_GYRO_Y_H]));
			gyro_x = gyro_radps(cm730.MakeWord(table[CM730::P_GYRO_X_L], table[CM730::P_GYRO_X_H]));

			accel_z = accel_ms2(cm730.MakeWord(table[CM730::P_ACCEL_Z_L], table[CM730::P_ACCEL_Z_H]));
			accel_y = accel_ms2(cm730.MakeWord(table[CM730::P_ACCEL_Y_L], table[CM730::P_ACCEL_Y_H]));
			accel_x = accel_ms2(cm730.MakeWord(table[CM730::P_ACCEL_X_L], table[CM730::P_ACCEL_X_H]));

			measurement << gyro_z, gyro_y, gyro_x, accel_z, accel_y, accel_x;

			// Apply Kalman Filter
			stateEstimate = k.Filter(measurement);

			// print
			if (diff_sec(prev_int, interval) > 1.0) {
				printf("TIME: %1.3f\tACCEL: %1.3f %1.3f %1.3f\tGRYO: %1.3f %1.3f %1.3f \n",
						time, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
				printf("TIME: %1.3f\tACCEL: %1.3f %1.3f %1.3f\tGRYO: %1.3f %1.3f %1.3f \n",
						time, stateEstimate(5, 0), stateEstimate(4, 0), stateEstimate(3, 0),
						stateEstimate(2, 0), stateEstimate(1, 0), stateEstimate(0, 0));

				clock_gettime(CLOCK_MONOTONIC, &prev_int);
			}

			usleep(500);
		}
	}

	return 0;
}


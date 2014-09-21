/* Copyright (c) 2014 Dylan Holmes. All rights reserved.
 *
 * Demonstration of the Kalman filter.
 */
#include <iostream>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <unistd.h>
#include <signal.h>
#include <fstream>
#include "Eigen/Dense"
#include "Kalman.h"
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

int main(int argc, char **argv) {
	std::ifstream in("in2.txt");
	std::ofstream out("out2.txt");
	Kalman<double, 6> k;

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


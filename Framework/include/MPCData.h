
#ifndef _MPC_DATA_H_
#define _MPC_DATA_H_

#include <time.h>

inline double sec_diff(timespec start, timespec end)
{
	// buggyy??? whyyyy
	//long temp = ((long)end.tv_sec*1000000000 + end.tv_nsec) -
	//	((long)start.tv_sec*1000000000 + start.tv_nsec);
	//return (double)(temp/1000000000.0);

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

inline double timespec2sec(timespec s)
{
	return (double)(s.tv_sec + s.tv_nsec / 1000000000.0);
}

// joint positions for converting to and from
inline double joint2radian(int joint_value) {
	//return (joint_value * 0.088) * 3.14159265 / 180.0;
	return (joint_value-2048.0) * 0.00153398078;
}

inline int radian2joint(double radian) {
	return (int)(radian * 651.898650256) + 2048;
}

// joint speeds for converting to and from
inline double j_rpm2rads_ps(int rpm) {
	//return rpm * 0.11 * 2 * 3.14159265 / 60;

	// bitwise
	int neg = !!(rpm & ~0x3ff); // bool for negative
	rpm = rpm & 0x3ff;			 // get speed magnitude
	rpm = (!neg*rpm)-(neg*rpm); //use bool as switch

	/* logical
		if (rpm > 1023) {
	// negative
	rpm = 1024 - rpm;
	}
	*/
	return rpm * 0.01151917306;
}

inline int rad_ps2rpm(double rad_ps) {
	//return rpm * 0.11 * 2 * 3.14159265 / 60;
	return (int)(rad_ps * 86.8117871649);
}

// gyro & accel
inline double gyro2rads_ps(int gyro) {
	return (gyro-512)*0.017453229251;
}

inline double accel2ms2(int accel) {
	//return (accel-512) / 128.0; in G's
	return ((accel-512) / 128.0) * 9.81; // in m/s^2
}

// fsr
inline double fsr2newton(int fsr) {
	return fsr / 1000.0;
}



#endif

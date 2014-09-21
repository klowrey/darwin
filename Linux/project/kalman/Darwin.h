

// gyro & accel
double gyro_radps(int gyro) {
	return (gyro-512)*0.017453229251;
}

double accel_ms2(int accel) {
	return ((accel-512) / 128.0) * 9.81; // in m/s^2
}

void darwin_init(void) {

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
}

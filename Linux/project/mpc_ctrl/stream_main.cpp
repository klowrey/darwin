#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"

#include <time.h>
#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iterator>

#include <fstream>
#include <iostream>

#define MPC_PORT 13131

using namespace Robot;
using namespace std;

const int SEND_BUF_SIZE = 47;
const int RECV_BUF_SIZE = 21; // time + position

/*
   enum
   {
   1 ID_R_SHOULDER_PITCH     = 1,
   2 ID_R_SHOULDER_ROLL      = 3,
   3 ID_R_ELBOW              = 5,
   4 ID_R_HIP_YAW            = 7,
   5 ID_R_HIP_ROLL           = 9,
   6 ID_R_HIP_PITCH          = 11,
   7 ID_R_KNEE               = 13,
   8 ID_R_ANKLE_PITCH        = 15,
   9 ID_R_ANKLE_ROLL         = 17,

   10 ID_L_SHOULDER_PITCH     = 2,
   11 ID_L_SHOULDER_ROLL      = 4,
   12 ID_L_ELBOW              = 6,
   13 ID_L_HIP_YAW            = 8,
   14 ID_L_HIP_ROLL           = 10,
   15 ID_L_HIP_PITCH          = 12,
   16 ID_L_KNEE               = 14,
   17 ID_L_ANKLE_PITCH        = 16,
   18 ID_L_ANKLE_ROLL         = 18,

   19 ID_HEAD_PAN             = 19,
   20 ID_HEAD_TILT            = 20,

   ID_R_SHOULDER_PITCH     = 1,
   ID_L_SHOULDER_PITCH     = 2,
   ID_R_SHOULDER_ROLL      = 3,
   ID_L_SHOULDER_ROLL      = 4,
   ID_R_ELBOW              = 5,
   ID_L_ELBOW              = 6,
   ID_R_HIP_YAW            = 7,
   ID_L_HIP_YAW            = 8,
   ID_R_HIP_ROLL           = 9,

   ID_L_HIP_ROLL           = 10,
   ID_R_HIP_PITCH          = 11,
   ID_L_HIP_PITCH          = 12,
   ID_R_KNEE               = 13,
   ID_L_KNEE               = 14,
   ID_R_ANKLE_PITCH        = 15,
   ID_L_ANKLE_PITCH        = 16,
   ID_R_ANKLE_ROLL         = 17,
   ID_L_ANKLE_ROLL         = 18,

   ID_HEAD_PAN             = 19,
   ID_HEAD_TILT            = 20,
 
   };
   */

double ms_diff(timespec start, timespec end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return (double)(temp.tv_sec*1000.0+temp.tv_nsec/1000000.0);
}

double joint2radian(int joint_value) {
	//return (joint_value * 0.088) * 3.14159265 / 180.0;
	return (joint_value-2048.0) * 0.00153398078;
}

int radian2joint(double radian) {
	return (int)(radian * 651.898650256) + 2048;;
}

double rpm2rads_ps(int rpm) {
	//return rpm * 0.11 * 2 * 3.14159265 / 60;
	return rpm * 0.01151917306;
}

int rad_ps2rpm(double rad_ps) {
	//return rpm * 0.11 * 2 * 3.14159265 / 60;
	return (int)(rad_ps * 86.8117871649);
}

int main(int argc, char* argv[])
{
	printf( "\n===== Read/Write Tutorial for DARwIn =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);

	if(cm730.Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return 0;
	}

	cm730.MakeBulkReadPacketMPC();

	/////////////////////////////////////////////////////////////////////

	int p_gain = 2; 
	int d_gain = 2;
	if (argc >= 2) {
		p_gain = atoi(argv[1]);	
	}
	if (argc >= 3) {
		d_gain = atoi(argv[2]);	
	}

	printf("P: %d, D: %d\n", p_gain, d_gain);

	for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<JointData::NUMBER_OF_JOINTS; joint++) {
		cm730.WriteWord(joint, MX28::P_TORQUE_ENABLE, 0, 0);
		cm730.WriteByte(joint, MX28::P_P_GAIN, p_gain, 0);
		cm730.WriteByte(joint, MX28::P_D_GAIN, d_gain, 0);
	}

	int value;
	int count = 0;
	int runs = -1;
	static struct timespec start_time;
	//static struct timespec begin_time;
	//static struct timespec end_time;
	static struct timespec read_time;
	//static struct timespec write_time;
	vector<double> r_time;
	vector<double> w_time;
	clockid_t TEST_CLOCK = CLOCK_MONOTONIC;

	LinuxServer data_sock;
	LinuxServer server ( MPC_PORT );

	double* buf = (double*) malloc(SEND_BUF_SIZE * sizeof(double));
	double* raw_buf = (double*) malloc(SEND_BUF_SIZE * sizeof(double));
	double* p_buf;
	
	double* ctrl = (double*) malloc(RECV_BUF_SIZE * sizeof(double));
	double* raw_ctrl = (double*) malloc(RECV_BUF_SIZE * sizeof(double));

	clock_gettime(TEST_CLOCK, &start_time);

	try
	{
		while(true)
		{
			cout << "[Waiting..]" << endl;            
			server.accept(data_sock); // tcp_nodelay, but a blocking port
			//data_sock.m_socket.set_non_blocking(false);
			cout << "[Accepted..]" << endl;

			try
			{
				while (true) {
					if (cm730.BulkRead() == CM730::SUCCESS) {

						//int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
						//int param[(JointData::NUMBER_OF_JOINTS-2) * MX28::PARAM_BYTES];

						clock_gettime(TEST_CLOCK, &read_time);
						buf[0] = ms_diff(start_time, read_time);
						p_buf = buf+1;
						int index = 1;

						int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
						int n = 0;
						int joint_num = 0;
						int id;
						double qpos;
						double qvel;


						// prepare data for sending to mpc
						for (int joint=1; joint<=JointData::ID_R_ANKLE_ROLL; joint++) {
							// right joints
							value = cm730.m_BulkReadData[joint].ReadWord(MX28::P_PRESENT_POSITION_L);
							qpos = joint2radian(value);
							*p_buf = qpos;
							index++;

							value = cm730.m_BulkReadData[joint].ReadWord(MX28::P_PRESENT_SPEED_L);
							if (value > 1023) {
								value = 1024 - value;
							}
							qvel = rpm2rads_ps(value);
							*(p_buf+20) = qvel;
							index++;

							//p_buf++;

							// left joints
							joint++;
							value = cm730.m_BulkReadData[joint].ReadWord(MX28::P_PRESENT_POSITION_L);
							qpos = joint2radian(value);
							*(p_buf+9) = qpos;
							index++;

							value = cm730.m_BulkReadData[joint].ReadWord(MX28::P_PRESENT_SPEED_L);
							if (value > 1023) {
								value = 1024 - value;
							}
							qvel = rpm2rads_ps(value);
							*(p_buf+29) = qvel;
							index++;

							p_buf++;
						}

						p_buf += 9;

						for (int joint=JointData::ID_HEAD_PAN; joint<=JointData::ID_HEAD_TILT; joint++) {
							// head joints
							value = cm730.m_BulkReadData[joint].ReadWord(MX28::P_PRESENT_POSITION_L);
							qpos = joint2radian(value);
							*p_buf = qpos;
							index++;

							value = cm730.m_BulkReadData[joint].ReadWord(MX28::P_PRESENT_SPEED_L);
							if (value > 1023) {
								value = 1024 - value;
							}
							qvel = rpm2rads_ps(value);
							*(p_buf+20) = qvel;
							index++;

							p_buf++;
						}


#ifdef VERBOSE
						printf("\nbuf  ");
						for (int joint=1; joint<=20; joint++) {
							raw_buf[joint] = buf[joint];
							printf("%1.1f ", raw_buf[joint]);
						}
						printf("\n");
#endif

						p_buf += 20;

						// gyroscope dps to rps
						*p_buf = (cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Z_L)-512)*0.017453229251;
						p_buf++;
						*p_buf = (cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L)-512)*0.017453229251;
						p_buf++;
						*p_buf = (cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L)-512)*0.017453229251;
						p_buf++;
						index+=3;

						// accelerometer +- 4g's
						*p_buf = (cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Z_L)-512) / 128.0;
						p_buf++;
						*p_buf = (cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L)-512) / 128.0;
						p_buf++;
						*p_buf = (cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L)-512) / 128.0;
						p_buf++;
						index+=3;

						// FSR Feet, Right and Left
						*p_buf = (cm730.m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR1_L)) / 1000.0;
						p_buf++;
						*p_buf = (cm730.m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR2_L)) / 1000.0;
						p_buf++;
						*p_buf = (cm730.m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR3_L)) / 1000.0;
						p_buf++;
						*p_buf = (cm730.m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR4_L)) / 1000.0;
						p_buf++;
						index+=4;

						*p_buf = (cm730.m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR1_L)) / 1000.0;
						p_buf++;
						*p_buf = (cm730.m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR2_L)) / 1000.0;
						p_buf++;
						*p_buf = (cm730.m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR3_L)) / 1000.0;
						p_buf++;
						*p_buf = (cm730.m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR4_L)) / 1000.0;
						p_buf++;
						index+=4;


						///////// Verify Output
#ifdef VERBOSE
						printf("Added %d things to buffer.\n", index);
						printf("time: ");
						printf("%f ", buf[0]);

						printf("\nqpos R: ");
						for (int i=1; i<10; i++) {
							printf("%1.2f ", buf[i]);
						}
						printf("L: ");
						for (int i=10; i<19; i++) {
							printf("%1.2f ", buf[i]);
						}
						printf("H: ");
						for (int i=19; i<21; i++) {
							printf("%1.2f ", buf[i]);
						}

						printf("\nqvel R: ");
						for (int i=21; i<30; i++) {
							printf("%1.2f ", buf[i]);
						}
						printf("L: ");
						for (int i=30; i<39; i++) {
							printf("%1.2f ", buf[i]);
						}
						printf("H: ");
						for (int i=39; i<41; i++) {
							printf("%1.2f ", buf[i]);
						}
						printf("\nsnsrs : ");
						for (int i=41; i<47; i++) {
							printf("%1.2f ", buf[i]);
						}
						printf("\n");
#endif

						//printf("S: %1.2f P: %1.2f %1.2f %1.2f %1.2f\n",
						//		buf[0], buf[1], buf[2], buf[3], buf[4]);

						if ( !data_sock.send((unsigned char*)buf, SEND_BUF_SIZE*sizeof(double)))
						{
							throw LinuxSocketException ( "Could not write to socket." );
						}

						memset((void*) ctrl, 0, RECV_BUF_SIZE*sizeof(double));

						if ( !data_sock.recv((unsigned char*)ctrl, RECV_BUF_SIZE*sizeof(double))) {
							throw LinuxSocketException ( "Could not write to socket." );
						}

#ifdef VERBOSE
						printf("ctrl ");
						for (int joint=1; joint<=20; joint++) {
							printf("%1.1f ", ctrl[joint]);
						}
						printf("\n");
#endif

						// prepare commands to joints
						for (int joint=1; joint<=JointData::ID_R_HIP_ROLL; joint++) {

							joint_num++;
							param[n++] = joint_num;
							value = radian2joint(ctrl[joint]); // in joint space?
							param[n++] = CM730::GetLowByte(value);
							param[n++] = CM730::GetHighByte(value);
							//printf("%d %d\t%d %d\n",
							//		joint_num, value, joint, cm730.m_BulkReadData[joint_num].ReadWord(MX28::P_PRESENT_POSITION_L));
							raw_ctrl[joint_num] = ctrl[joint];
							//printf("%d ", joint_num);

							joint_num++;
							param[n++] = joint_num;
							value = radian2joint(ctrl[joint+9]); // in joint space?
							param[n++] = CM730::GetLowByte(value);
							param[n++] = CM730::GetHighByte(value);
							//printf("%d %d\t%d %d\n",
							//		joint_num, value, joint+9, cm730.m_BulkReadData[joint_num].ReadWord(MX28::P_PRESENT_POSITION_L));
							raw_ctrl[joint_num] = ctrl[joint+9];
							//printf("%d ", joint_num);
						}

						id = JointData::ID_HEAD_PAN;
						param[n++] = id;
						value = radian2joint(ctrl[id]);
						param[n++] = CM730::GetLowByte(value);
						param[n++] = CM730::GetHighByte(value);
						raw_ctrl[id] = ctrl[id];
						joint_num++;
						//printf("%d ", id);

						id = JointData::ID_HEAD_TILT;
						param[n++] = id;
						value = radian2joint(ctrl[id]);
						param[n++] = CM730::GetLowByte(value);
						param[n++] = CM730::GetHighByte(value);
						raw_ctrl[id] = ctrl[id];
						joint_num++;
						//printf("%d %f ", id, ctrl[id]);

						printf("\nRecieved %d joint positions.\n", joint_num);

#ifdef VERBOSE
						for (int joint=1; joint<=20; joint++) {
							printf("%d %f %f\n", joint, raw_buf[joint], raw_ctrl[joint]);
						}
#endif

						if(joint_num > 0) {
							//cm730.SyncWrite(MX28::P_D_GAIN, MX28::PARAM_BYTES, joint_num, param);
							cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 3, joint_num, param);
						}
					}
					else {
						printf("Couldn't read data!\n");
						usleep(1000);
					}
					count++;
				}
			}
			catch ( LinuxSocketException& )
			{
				cout << "[Disconnected]" << endl;

				for (int joint=1; joint<JointData::NUMBER_OF_JOINTS; joint++) {
					// go slack
					cm730.WriteWord(joint, MX28::P_TORQUE_ENABLE, 0, 0);
				}
			}
		}
	}
	catch ( LinuxSocketException& e)
	{
		cout << "Exception was caught:" << e.description() << "\nExiting.\n";
	}

	//clock_gettime(TEST_CLOCK, &end_time);
	//printf("Total time: %fms\n", ms_diff(start_time, end_time));

	free(buf);
	free(raw_buf);
	free(ctrl);
	free(raw_ctrl);

	return 0;
}

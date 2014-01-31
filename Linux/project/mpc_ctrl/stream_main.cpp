#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"
#include "MPCData.h"

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

const int RECV_BUF_SIZE = 21; // time + position
const int SEND_BUF_SIZE = 55; // 1+20+20+3+3+4+4

/*
	1  ID_R_SHOULDER_PITCH     = 1,
	2  ID_R_SHOULDER_ROLL      = 3,
	3  ID_R_ELBOW              = 5,
	4  ID_R_HIP_YAW            = 7,
	5  ID_R_HIP_ROLL           = 9,
	6  ID_R_HIP_PITCH          = 11,
	7  ID_R_KNEE               = 13,
	8  ID_R_ANKLE_PITCH        = 15,
	9  ID_R_ANKLE_ROLL         = 17,

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

	=========

	1  ID_R_SHOULDER_PITCH     = 1,
	10 ID_L_SHOULDER_PITCH     = 2,
	2  ID_R_SHOULDER_ROLL      = 3,
	11 ID_L_SHOULDER_ROLL      = 4,
	3  ID_R_ELBOW              = 5,
	12 ID_L_ELBOW              = 6,

	4  ID_R_HIP_YAW            = 7,
	13 ID_L_HIP_YAW            = 8,
	5  ID_R_HIP_ROLL           = 9,
	14 ID_L_HIP_ROLL           = 10,
	6  ID_R_HIP_PITCH          = 11,
	15 ID_L_HIP_PITCH          = 12,
	7  ID_R_KNEE               = 13,
	16 ID_L_KNEE               = 14,
	8  ID_R_ANKLE_PITCH        = 15,
	17 ID_L_ANKLE_PITCH        = 16,
	9  ID_R_ANKLE_ROLL         = 17,
	18 ID_L_ANKLE_ROLL         = 18,

	19 ID_HEAD_PAN             = 19,
	20 ID_HEAD_TILT            = 20,

*/

void print_send_buf(double* buf)
{
	printf("time: %f\n", buf[0]);

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

}

int main(int argc, char* argv[])
{
	printf( "\n===== Stream 2 MPC Tutorial for DARwIn =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);

	if(cm730.Connect() == false)
	{
		printf("Failure to connect CM-730!\n");
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
	LinuxServer server ( "128.208.4.38", MPC_PORT );

	double* buf = (double*) malloc(SEND_BUF_SIZE * sizeof(double));
	double* p_buf;

	double* ctrl = (double*) malloc(RECV_BUF_SIZE * sizeof(double));
#ifdef VERBOSE
	double* raw_buf = (double*) malloc(SEND_BUF_SIZE * sizeof(double));
	double* raw_ctrl = (double*) malloc(RECV_BUF_SIZE * sizeof(double));
#endif

	clock_gettime(TEST_CLOCK, &start_time);

	try
	{
		while(true)
		{
			cout << "[Waiting..]" << endl;            
			server.accept(data_sock); // tcp_nodelay, but a blocking port
			//data_sock.set_non_blocking(true);
			cout << "[Accepted..]" << endl;

			try
			{
				while (true) {
					if (cm730.BulkRead() == CM730::SUCCESS) {

						//int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
						//int param[(JointData::NUMBER_OF_JOINTS-2) * MX28::PARAM_BYTES];

						clock_gettime(TEST_CLOCK, &read_time);
						buf[0] = sec_diff(start_time, read_time);
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
							qvel = j_rpm2rads_ps(value);
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
							qvel = j_rpm2rads_ps(value);
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
							qvel = j_rpm2rads_ps(value);
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

						// gyroscope dps to rad_ps
						*p_buf = gyro2rads_ps(cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Z_L));
						p_buf++;
						*p_buf = gyro2rads_ps(cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L));
						p_buf++;
						*p_buf = gyro2rads_ps(cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L));
						p_buf++;
						index+=3;

						// accelerometer in G's (range is +/- 4g's) 
						*p_buf = accel2ms2(cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Z_L));
						p_buf++;
						*p_buf = accel2ms2(cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L));
						p_buf++;
						*p_buf = accel2ms2(cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L));
						p_buf++;
						index+=3;

						// FSR Feet, Right and Left
						*p_buf = fsr2newton(cm730.m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR1_L));
						p_buf++;
						*p_buf = fsr2newton(cm730.m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR2_L));
						p_buf++;
						*p_buf = fsr2newton(cm730.m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR3_L));
						p_buf++;
						*p_buf = fsr2newton(cm730.m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR4_L));
						p_buf++;
						index+=4;

						*p_buf = fsr2newton(cm730.m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR1_L));
						p_buf++;
						*p_buf = fsr2newton(cm730.m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR2_L));
						p_buf++;
						*p_buf = fsr2newton(cm730.m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR3_L));
						p_buf++;
						*p_buf = fsr2newton(cm730.m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR4_L));
						p_buf++;
						index+=4;


						///////// Verify Output
#ifdef VERBOSE
						print_send_buf(buf);
#endif

						if ( !data_sock.send((unsigned char*)buf, SEND_BUF_SIZE*sizeof(double)))
						{
							throw LinuxSocketException ( "Could not write to socket." );
						}

						memset((void*) ctrl, 0, RECV_BUF_SIZE*sizeof(double));

						if ( !data_sock.recv((unsigned char*)ctrl, RECV_BUF_SIZE*sizeof(double)))
						{
							// bug in the MPC-side code not killing the thread
							// properly
							throw LinuxSocketException ( "Could not read from socket." );
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
							//raw_ctrl[joint_num] = ctrl[joint];
							//printf("%d ", joint_num);

							joint_num++;
							param[n++] = joint_num;
							value = radian2joint(ctrl[joint+9]); // in joint space?
							param[n++] = CM730::GetLowByte(value);
							param[n++] = CM730::GetHighByte(value);
							//printf("%d %d\t%d %d\n",
							//		joint_num, value, joint+9, cm730.m_BulkReadData[joint_num].ReadWord(MX28::P_PRESENT_POSITION_L));
							//raw_ctrl[joint_num] = ctrl[joint+9];
							//printf("%d ", joint_num);
						}

						id = JointData::ID_HEAD_PAN;
						param[n++] = id;
						value = radian2joint(ctrl[id]);
						param[n++] = CM730::GetLowByte(value);
						param[n++] = CM730::GetHighByte(value);
						//raw_ctrl[id] = ctrl[id];
						joint_num++;
						//printf("%d ", id);

						id = JointData::ID_HEAD_TILT;
						param[n++] = id;
						value = radian2joint(ctrl[id]);
						param[n++] = CM730::GetLowByte(value);
						param[n++] = CM730::GetHighByte(value);
						//raw_ctrl[id] = ctrl[id];
						joint_num++;
						//printf("%d %f ", id, ctrl[id]);

						//printf("\nRecieved %d joint positions.\n", joint_num);

#ifdef VERBOSE
						for (int joint=1; joint<=20; joint++) {
							printf("%d %f %f\n", joint, raw_buf[joint], raw_ctrl[joint]);
						}
#endif

						if(joint_num > 0) {
							//cm730.SyncWrite(MX28::P_D_GAIN, MX28::PARAM_BYTES, joint_num, param);

							//cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 3, joint_num, param);
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
	//printf("Total time: %fms\n", sec_diff(start_time, end_time));

	free(buf);
	free(ctrl);
#ifdef VERBOSE
	free(raw_buf);
	free(raw_ctrl);
#endif

	return 0;
}

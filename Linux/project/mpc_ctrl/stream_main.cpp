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

#define MPC_PORT 13105

using namespace Robot;
using namespace std;

const int SEND_BUF_SIZE = 47;
const int RECV_BUF_SIZE = 60; // pos, p gain, d gain

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
	return joint_value * 0.00153398078;
}

double rpm2radianspersecond(int rpm) {
	//return rpm * 0.11 * 2 * 3.14159265 / 60;
	return rpm * 0.01151917306;
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

	int p_gain = 10; 
	int d_gain = 10;
	if (argc >= 2) {
		p_gain = atoi(argv[1]);	
	}
	if (argc >= 3) {
		d_gain = atoi(argv[2]);	
	}

	printf("P: %d, D: %d\n", p_gain, d_gain);

	for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<JointData::NUMBER_OF_JOINTS; joint++) {
		cm730.WriteWord(joint, MX28::P_TORQUE_ENABLE, 0, 0);
		//cm730.WriteByte(joint, MX28::P_P_GAIN, p_gain, 0);
		//cm730.WriteByte(joint, MX28::P_D_GAIN, d_gain, 0);
	}

	int value;
	int count = 0;
	int runs = -1;
	int alternate = 1;
	static struct timespec start_time;
	static struct timespec begin_time;
	static struct timespec end_time;
	static struct timespec read_time;
	static struct timespec write_time;
	vector<double> r_time;
	vector<double> w_time;
	clockid_t TEST_CLOCK = CLOCK_MONOTONIC;

	LinuxServer data_sock;
	LinuxServer server ( MPC_PORT );

	double* buf = (double*) malloc(SEND_BUF_SIZE * sizeof(double));
	double* p_buf;
	
	double* ctrl = (double*) malloc(RECV_BUF_SIZE * sizeof(double));

	clock_gettime(TEST_CLOCK, &start_time);

	while(1)
	{
		cout << "[Waiting..]" << endl;            
		server.accept(data_sock); // tcp_nodelay, but a blocking port
		//data_sock.m_socket.set_non_blocking(true);
		cout << "[Accepted..]" << endl;

		try
		{
			if (cm730.BulkRead() == CM730::SUCCESS) {

				//int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
				//int param[(JointData::NUMBER_OF_JOINTS-2) * MX28::PARAM_BYTES];

				clock_gettime(TEST_CLOCK, &read_time);
				buf[0] = ms_diff(start_time, read_time);
				p_buf = buf+1;
				int index = 1;
				int idx=1;

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
					qvel = rpm2radianspersecond(value);
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
					qvel = rpm2radianspersecond(value);
					*(p_buf+29) = qvel;
					index++;

					p_buf++;
					idx++;
				}

				p_buf += 9;
				idx += 9;
				printf("idx: %d\n", idx);

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
					qvel = rpm2radianspersecond(value);
					*(p_buf+20) = qvel;
					index++;

					p_buf++;
					idx++;
				}

				p_buf += 20;
				idx += 20;
				printf("idx: %d\n", idx);

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

				data_sock.send((unsigned char*)buf, SEND_BUF_SIZE*sizeof(double));
				data_sock.recv((unsigned char*)ctrl, SEND_BUF_SIZE*sizeof(double));

				// prepare commands to joints
				for (int joint=1; joint<JointData::NUMBER_OF_JOINTS; joint++) {
					value = cm730.m_BulkReadData[joint].ReadWord(MX28::P_PRESENT_POSITION_L);
					id = joint+1;
					value = MX28::GetMirrorValue(value);
					param[n++] = id;
					param[n++] = CM730::GetLowByte(value);
					param[n++] = CM730::GetHighByte(value);
					joint_num++;
				}

				/*
					if(joint_num > 0) {
				//cm730->SyncWrite(MX28::P_D_GAIN, MX28::PARAM_BYTES, joint_num, param);
				if (alternate == 0) {
				cm730.SyncWrite(MX28::P_TORQUE_ENABLE, 2, joint_num, param);
				alternate = 1;
				}
				else {
				cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 3, joint_num, param);
				alternate = 0;
				}
				}
				*/
			}
			else {
				printf("Couldn't read data!\n");
				usleep(1000);
			}
			count++;
		}
		catch ( LinuxSocketException& )
		{
			cout << "[Disconnected]" << endl;

			/*
				if(Action::GetInstance()->IsRunning() == 1)
				{
				Action::GetInstance()->Stop();
				while(Action::GetInstance()->IsRunning() == 1)
				usleep(1);
				MotionManager::GetInstance()->SetEnable(false);
			//motion_timer->Stop();
			}
			*/
		}


		/*
			clock_gettime(TEST_CLOCK, &write_time);

			r_time.push_back(ms_diff(start_time, read_time));
			w_time.push_back(ms_diff(read_time, write_time));

			if (count == 100) {
			double r_sum = accumulate(r_time.begin(), r_time.end(), 0.0);
			double w_sum = accumulate(w_time.begin(), w_time.end(), 0.0);
			double r_mean = r_sum / r_time.size();
			double w_mean = w_sum / w_time.size();

			vector<double> r_diff(r_time.size());
			transform(r_time.begin(), r_time.end(), r_diff.begin(), bind2nd(minus<double>(), r_mean));
			double sq_sum = inner_product(r_diff.begin(), r_diff.end(), r_diff.begin(), 0.0);
			double r_stdev = sqrt(sq_sum / r_time.size());

			vector<double> w_diff(r_time.size());
			transform(w_time.begin(), w_time.end(), w_diff.begin(), bind2nd(minus<double>(), w_mean));
			sq_sum = inner_product(w_diff.begin(), w_diff.end(), w_diff.begin(), 0.0);
			double w_stdev = sqrt(sq_sum / w_time.size());

			printf("Total: %f\t\tRead: %f ms, stdev: %f Write: %f ms, stdev: %f\n",
			r_mean+w_mean, r_mean, r_stdev, w_mean, w_stdev);
			if ((r_stdev+w_stdev) > 1.0) {
			copy(r_time.begin(), r_time.end(), ostream_iterator<double>(cout, " "));
			printf("\n");
			}
			r_time.clear();
			w_time.clear();

			runs--;
			if (runs == 0) {
			break;
			}
			count = 0;
			}
			*/
	}
	clock_gettime(TEST_CLOCK, &end_time);
	printf("Total time: %fms\n", ms_diff(start_time, end_time));

	free(buf);
	free(ctrl);


	return 0;
}

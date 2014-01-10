#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"

#include <time.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iterator>

#include <fstream>
#include <iostream>

using namespace Robot;

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


void force_to_kp(int* pp, int* pk, int F, int max, int cur_p, int min) {

	double diff;
	if (F == 0) {
		*pk = 1;
		*pp = cur_p;
		return;
	}
	else if (F > 0) {
		diff = (max - cur_p);
	}
	else {
		diff = (min - cur_p);
	}

	if (diff == 0) {
		*pk = 0;
		*pp = cur_p;
		return;
	}

	double k_tilda = double(F) / double(diff); 

	int k = int(ceil(k_tilda));
	if (k<0) {
		k = k*-1;
	}
	if (k > 254) {
		k = 254;
	}

	int p = (F/k + cur_p);

	//printf("F: %d p: %d k: %d k_t: %f\n", F, p, k, k_tilda);

	*pp = p;
	*pk = k;
}

#define MIRRORED 
#define PRINTING

int main(int argc, char* argv[])
{
	printf( "\n===== Read/Write Tutorial for DARwIn =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);
	cm730.MakeBulkReadPacketMPC();
	if(cm730.Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return 0;
	}
	/////////////////////////////////////////////////////////////////////

	int p_gain = 10; 
	int l_gain = 1; 
	int d_gain = 0;
	if (argc >= 2) {
		p_gain = atoi(argv[1]);	
	}
	if (argc >= 3) {
		l_gain = atoi(argv[2]);	
	}

	printf("P: %d, L: %d\n", p_gain, l_gain);

	for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<=JointData::ID_L_ANKLE_ROLL; joint++) {
		cm730.WriteByte(joint, MX28::P_TORQUE_ENABLE, 1, 0);
		cm730.WriteByte(joint, MX28::P_P_GAIN, 0, 0);
		cm730.WriteByte(joint, MX28::P_D_GAIN, 0, 0);
		cm730.WriteWord(joint, MX28::P_PUNCH_L, 0, 0);
	}
		cm730.WriteByte(2, MX28::P_TORQUE_ENABLE, 0, 0);
		cm730.WriteByte(4, MX28::P_TORQUE_ENABLE, 0, 0);


	// Unit test for force_to_kp
	for (int i = 0; i<4096; i+=128) {
		int p = 0;
		int k = 0;
		force_to_kp(&p, &k, 32768, 4096, i, 0);
		printf("p: %d, k: %d\n", p, k);
	}

	int value;
	int load = 0;
	int vel = 0;
	int count = 0;
	int runs = -1;
	int alt = 0;
	static struct timespec start_time;
	static struct timespec begin_time;
	static struct timespec end_time;
	static struct timespec read_time;
	static struct timespec write_time;
	std::vector<double> r_time;
	std::vector<double> w_time;
	clockid_t TEST_CLOCK = CLOCK_MONOTONIC;

	double r_sum = 0.0;
	double w_sum = 0.0;
	double r_mean= 7.0; 
	double w_mean= 0.0; 

	while(1)
	{
#ifdef PRINTING
		clock_gettime(TEST_CLOCK, &start_time);
#endif

#ifdef MIRRORED 
		if (cm730.BulkRead() == CM730::SUCCESS) {
			//int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
			//int param[(JointData::NUMBER_OF_JOINTS-2) * MX28::PARAM_BYTES];

			clock_gettime(TEST_CLOCK, &read_time);
			int ctrl_joints = 9;
			int param[ctrl_joints * MX28::PARAM_BYTES];
			int n = 0;
			int joint_num = 0;
			int id;
			int goal_p_gain = p_gain;

			int diff;
			int new_value;

			//for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<=JointData::ID_L_ANKLE_ROLL; joint+=2)
			for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<=JointData::ID_L_ANKLE_ROLL; joint++)
			{
				id = joint;
				value = cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L);

				//value = MX28::GetMirrorValue(value);

				//id = joint+1;
				load = cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_LOAD_L);
				vel = cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_SPEED_L);
				if (vel > 1023) {
					vel = 1024 - vel;
				}
				if (load > 1023) {
					load = 1024 - load;
				}

				//new_value = value + l_gain * ( 0.0077824 * vel * r_mean);
				new_value = value + p_gain * ( 0.0077824 * vel * r_mean);

				if (id == 5) {
					int force = l_gain * vel;

					//printf("p: %d l: %d v: %d \tnp: %d %d\n", value, load, vel, new_value, goal_p_gain);

					int new_k = 0;
					int new_p = 0;

					force_to_kp(&new_p, &new_k, force, 2844, new_value, 968);
					printf("force: %d, p: %d, %d, %d, k: %d\n", force, new_p, new_value, value, new_k);

					param[n++] = id;
					param[n++] = new_k; 
					param[n++] = 0;
					param[n++] = CM730::GetLowByte(new_p);
					param[n++] = CM730::GetHighByte(new_p);
					joint_num++;
				}
			}

			if(joint_num > 0) {
				//cm730->SyncWrite(MX28::P_D_GAIN, MX28::PARAM_BYTES, joint_num, param);
				//cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 3, joint_num, param);
				int res = cm730.SyncWrite(MX28::P_P_GAIN, 5, joint_num, param);
				if (res != CM730::SUCCESS) {
					switch(res)
					{
						case CM730::SUCCESS:
							printf("SUCCESS\n");
							break;

						case CM730::TX_CORRUPT:
							printf("TX_CORRUPT\n");
							break;

						case CM730::TX_FAIL:
							printf("TX_FAIL\n");
							break;

						case CM730::RX_FAIL:
							printf("RX_FAIL\n");
							break;

						case CM730::RX_TIMEOUT:
							printf("RX_TIMEOUT\n");
							break;

						case CM730::RX_CORRUPT:
							printf("RX_CORRUPT\n");
							break;

						default:
							printf("UNKNOWN\n");
							break;
					}
				}
			}
		}
		else {
			printf("Couldn't read data!\n");
		}
#else
		if(cm730.ReadWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
			cm730.WriteWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L, MX28::GetMirrorValue(value), 0);
		}

		if(cm730.ReadWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
			cm730.WriteWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L, MX28::GetMirrorValue(value), 0);
		}

		if(cm730.ReadWord(JointData::ID_R_ELBOW, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
			cm730.WriteWord(JointData::ID_L_ELBOW, MX28::P_GOAL_POSITION_L, MX28::GetMirrorValue(value), 0);
		}
#endif

		count++;
#ifdef PRINTING
		clock_gettime(TEST_CLOCK, &write_time);

		r_time.push_back(ms_diff(start_time, read_time));
		w_time.push_back(ms_diff(read_time, write_time));

		if (count == 100) {
			r_sum = std::accumulate(r_time.begin(), r_time.end(), 0.0);
			w_sum = std::accumulate(w_time.begin(), w_time.end(), 0.0);
			r_mean = r_sum / r_time.size();
			w_mean = w_sum / w_time.size();

			std::vector<double> r_diff(r_time.size());
			std::transform(r_time.begin(), r_time.end(), r_diff.begin(), std::bind2nd(std::minus<double>(), r_mean));
			double sq_sum = std::inner_product(r_diff.begin(), r_diff.end(), r_diff.begin(), 0.0);
			double r_stdev = std::sqrt(sq_sum / r_time.size());

			std::vector<double> w_diff(r_time.size());
			std::transform(w_time.begin(), w_time.end(), w_diff.begin(), std::bind2nd(std::minus<double>(), w_mean));
			sq_sum = std::inner_product(w_diff.begin(), w_diff.end(), w_diff.begin(), 0.0);
			double w_stdev = std::sqrt(sq_sum / w_time.size());

			printf("Total: %f\t\tRead: %f ms, stdev: %f Write: %f ms, stdev: %f\n",
					r_mean+w_mean, r_mean, r_stdev, w_mean, w_stdev);
			if ((r_stdev+w_stdev) > 1.0) {
				copy(r_time.begin(), r_time.end(), std::ostream_iterator<double>(std::cout, " "));
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
#else
		// Summary
		clock_gettime(TEST_CLOCK, &end_time);
		if (count == 100) {
			runs--;
			if (runs == 0) {
				printf("Total RW: %d, Time: %fms, %fmsprw\n",
						runs*100, ms_diff(begin_time, end_time),
						ms_diff(begin_time, end_time) / (runs*100));
				break;
			}
			count = 0;
		}
#endif
		//usleep(500);
	}
	clock_gettime(TEST_CLOCK, &end_time);
	printf("Total time: %fms\n", ms_diff(start_time, end_time));

	for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<=JointData::ID_L_ANKLE_ROLL; joint++) {
		cm730.WriteByte(joint, MX28::P_TORQUE_ENABLE, 0, 0);
	}

	return 0;
}

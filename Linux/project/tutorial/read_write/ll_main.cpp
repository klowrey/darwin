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

#define SYNCED

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

	for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<=JointData::ID_L_ANKLE_ROLL; joint++) {
	cm730.WriteWord(joint, MX28::P_TORQUE_ENABLE, 0, 0);
	}
	//cm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_TORQUE_ENABLE, 0, 0);
	//cm730.WriteWord(JointData::ID_R_SHOULDER_ROLL,  MX28::P_TORQUE_ENABLE, 0, 0);
	//cm730.WriteWord(JointData::ID_R_ELBOW,          MX28::P_TORQUE_ENABLE, 0, 0);

	//cm730.WriteByte(JointData::ID_L_SHOULDER_PITCH, MX28::P_P_GAIN, 5, 0);
	//cm730.WriteByte(JointData::ID_L_SHOULDER_ROLL,  MX28::P_P_GAIN, 5, 0);
	//cm730.WriteByte(JointData::ID_L_ELBOW,          MX28::P_P_GAIN, 5, 0);

	int p_gain = 10; 
	int d_gain = 10;
	if (argc >= 2) {
		p_gain = atoi(argv[1]);	
	}
	if (argc >= 3) {
		d_gain = atoi(argv[2]);	
	}
	
	printf("P: %d, D: %d\n", p_gain, d_gain);

	for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<=JointData::ID_L_ANKLE_ROLL; joint++) {
		cm730.WriteByte(joint, MX28::P_P_GAIN, p_gain, 0);
		cm730.WriteByte(joint, MX28::P_D_GAIN, d_gain, 0);
	}

	int value;
	int count = 0;
	static struct timespec start_time;
	static struct timespec read_time;
	static struct timespec write_time;
	std::vector<double> r_time;
	std::vector<double> w_time;
	clockid_t TEST_CLOCK = CLOCK_MONOTONIC;

	while(1)
	{
		clock_gettime(TEST_CLOCK, &start_time);

#ifdef SYNCED
		if (cm730.BulkRead() == CM730::SUCCESS) {
			//int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
			//int param[(JointData::NUMBER_OF_JOINTS-2) * MX28::PARAM_BYTES];
			clock_gettime(TEST_CLOCK, &read_time);
			int ctrl_joints = 9;
			int param[ctrl_joints * MX28::PARAM_BYTES];
			int n = 0;
			int joint_num = 0;
			int id;

			for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<=JointData::ID_L_ANKLE_ROLL; joint+=2) {
				value = cm730.m_BulkReadData[joint].ReadWord(MX28::P_PRESENT_POSITION_L);
				id = joint+1;
				value = MX28::GetMirrorValue(value);
				param[n++] = id;
				param[n++] = CM730::GetLowByte(value);
				param[n++] = CM730::GetHighByte(value);
				joint_num++;
			}

			/*
				for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<=JointData::ID_L_ANKLE_ROLL; joint++) {
				value = cm730.m_BulkReadData[joint].ReadWord(MX28::P_PRESENT_POSITION_L);
				if (joint % 2 == 0) {
				id = joint-1;
				}
				else {
				id = joint+1;
				}
				value = MX28::GetMirrorValue(value);
				param[n++] = id;
				param[n++] = CM730::GetLowByte(value);
				param[n++] = CM730::GetHighByte(value);
				joint_num++;
				}
				*/

			if(joint_num > 0) {
				//m_CM730->SyncWrite(MX28::P_D_GAIN, MX28::PARAM_BYTES, joint_num, param);
				cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 3, joint_num, param);
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

		clock_gettime(TEST_CLOCK, &write_time);

		r_time.push_back(ms_diff(start_time, read_time));
		w_time.push_back(ms_diff(read_time, write_time));

		count++;
		if (count == 100) {
			count = 0;
			double r_sum = std::accumulate(r_time.begin(), r_time.end(), 0.0);
			double w_sum = std::accumulate(w_time.begin(), w_time.end(), 0.0);
			double r_mean = r_sum / r_time.size();
			double w_mean = w_sum / w_time.size();

			std::vector<double> r_diff(r_time.size());
			std::transform(r_time.begin(), r_time.end(), r_diff.begin(), std::bind2nd(std::minus<double>(), r_mean));
			double sq_sum = std::inner_product(r_diff.begin(), r_diff.end(), r_diff.begin(), 0.0);
			double r_stdev = std::sqrt(sq_sum / r_time.size());

			std::vector<double> w_diff(r_time.size());
			std::transform(w_time.begin(), w_time.end(), w_diff.begin(), std::bind2nd(std::minus<double>(), w_mean));
			sq_sum = std::inner_product(w_diff.begin(), w_diff.end(), w_diff.begin(), 0.0);
			double w_stdev = std::sqrt(sq_sum / w_time.size());

			printf("Read: %f ms, stdev: %f Write: %f ms, stdev: %f\n", r_mean, r_stdev, w_mean, w_stdev);
			if ((r_stdev+w_stdev) > 1.0) {
				copy(r_time.begin(), r_time.end(), std::ostream_iterator<double>(std::cout, " "));
				printf("\n");
			}
			r_time.clear();
			w_time.clear();
		}

		//usleep(500);
	}

	return 0;
}

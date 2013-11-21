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

int main()
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

	int value;
	cm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_TORQUE_ENABLE, 0, 0);
	cm730.WriteWord(JointData::ID_R_SHOULDER_ROLL,  MX28::P_TORQUE_ENABLE, 0, 0);
	cm730.WriteWord(JointData::ID_R_ELBOW,          MX28::P_TORQUE_ENABLE, 0, 0);

	cm730.WriteByte(JointData::ID_L_SHOULDER_PITCH, MX28::P_P_GAIN, 5, 0);
	cm730.WriteByte(JointData::ID_L_SHOULDER_ROLL,  MX28::P_P_GAIN, 5, 0);
	cm730.WriteByte(JointData::ID_L_ELBOW,          MX28::P_P_GAIN, 5, 0);

	int count = 0;
	static struct timespec start_time;
	static struct timespec end_time;
	std::vector<double> timings;
	clockid_t TEST_CLOCK = CLOCK_THREAD_CPUTIME_ID; //CLOCK_MONOTONIC;
	while(1)
	{
		clock_gettime(TEST_CLOCK, &start_time);

#ifdef SYNCED
		if (cm730.BulkRead() == CM730::SUCCESS) {
			//int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
			//int param[(JointData::NUMBER_OF_JOINTS-2) * MX28::PARAM_BYTES];
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
			id = JointData::ID_L_SHOULDER_PITCH;
			value = cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L);
			value = MX28::GetMirrorValue(value);
			param[n++] = id;
			param[n++] = CM730::GetLowByte(value);
			param[n++] = CM730::GetHighByte(value);
			joint_num++;

			id = JointData::ID_L_SHOULDER_ROLL;
			value = cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L);
			value = MX28::GetMirrorValue(value);
			param[n++] = id;
			param[n++] = CM730::GetLowByte(value);
			param[n++] = CM730::GetHighByte(value);
			joint_num++;

			id = JointData::ID_L_ELBOW;
			value = cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L);
			value = MX28::GetMirrorValue(value);
			param[n++] = id;
			param[n++] = CM730::GetLowByte(value);
			param[n++] = CM730::GetHighByte(value);
			joint_num++;
			*/

			if(joint_num > 0) {
				//m_CM730->SyncWrite(MX28::P_D_GAIN, MX28::PARAM_BYTES, joint_num, param);
				cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 3, joint_num, param);
			}
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

		clock_gettime(TEST_CLOCK, &end_time);

		double msec = ms_diff(start_time, end_time);
		timings.push_back(msec);

		count++;
		if (count == 100) {
			count = 0;
			double sum = std::accumulate(timings.begin(), timings.end(), 0.0);
			double mean = sum / timings.size();

			std::vector<double> diff(timings.size());
			std::transform(timings.begin(), timings.end(), diff.begin(), std::bind2nd(std::minus<double>(), mean));
			double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
			double stdev = std::sqrt(sq_sum / timings.size());

			printf("Mean: %f ms, Stdev: %f\n", mean, stdev);
			if (stdev > 1.0) {
				copy(timings.begin(), timings.end(), std::ostream_iterator<double>(std::cout, " "));
				printf("\n");
			}
			timings.clear();
		}

		//usleep(500);
	}

	return 0;
}

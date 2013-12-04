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

//#define SYNCED
//#define PRINTING
#define LOGFILE

std::ofstream m_LogFileStream;

void StartLogging(int p, int d)
{
	char szFile[32] = {0,};

	int count = 0;
	while(1)
	{
		//sprintf(szFile, "log%d.csv", count);
		sprintf(szFile, "pid_%d_0_%d.csv", p, d);
		if(0 != access(szFile, F_OK))
			break;
		count++;
		if(count > 256) return;
	}

	m_LogFileStream.open(szFile, std::ios::out);
		m_LogFileStream << "TIME_MS,ID_25,GOAL,POS,VEL,LOAD,VOLT,TEMP"<< std::endl;
		/*
	for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++)
	{
		m_LogFileStream << "ID_" << id << "_GP,ID_" << id << "_PP,";
	}
	*/
	//m_LogFileStream << "GyroFB,GyroRL,AccelFB,AccelRL,L_FSR_X,L_FSR_Y,R_FSR_X,R_FSR_Y" << std::endl;
}

void StopLogging()
{
	m_LogFileStream.close();
}


int main(int argc, char* argv[])
{
	printf( "\n===== Read/Write Tutorial for DARwIn =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);
	//cm730.MakeBulkReadPacketMPC();
	cm730.MakeBulkReadPacketServo25();
	if(cm730.Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return 0;
	}
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

	for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<=JointData::ID_L_ANKLE_ROLL; joint++) {
		cm730.WriteWord(joint, MX28::P_TORQUE_ENABLE, 0, 0);
		cm730.WriteByte(joint, MX28::P_P_GAIN, p_gain, 0);
		cm730.WriteByte(joint, MX28::P_D_GAIN, d_gain, 0);
	}

	int value;
	int count = 0;
	int runs = 0;
	int target_pos = 0, delta = 8;
	static struct timespec start_time;
	static struct timespec begin_time;
	static struct timespec end_time;
	static struct timespec read_time;
	static struct timespec write_time;
	std::vector<double> r_time;
	std::vector<double> w_time;
	clockid_t TEST_CLOCK = CLOCK_REALTIME;//CLOCK_MONOTONIC;



	// file logging
#ifdef LOGFILE
	StartLogging();
	cm730.WriteWord(25, MX28::P_TORQUE_ENABLE, 0, 0);
	cm730.WriteByte(25, MX28::P_P_GAIN, p_gain, 0);
	cm730.WriteByte(25, MX28::P_D_GAIN, d_gain, 0);
	cm730.WriteWord(25, MX28::P_GOAL_POSITION_L, target_pos, 0);
	usleep(2000000);
	printf("At Zero Position\n");

	clock_gettime(TEST_CLOCK, &start_time);
#endif


	while(1)
	{
#ifdef PRINTING
		clock_gettime(TEST_CLOCK, &start_time);
#endif

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

			if(joint_num > 0) {
				//cm730->SyncWrite(MX28::P_D_GAIN, MX28::PARAM_BYTES, joint_num, param);
				cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 3, joint_num, param);
			}
		}
		else {
			printf("Couldn't read data!\n");
		}
#else
		/*
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
		*/
#endif

		count++;
#ifdef PRINTING
		clock_gettime(TEST_CLOCK, &write_time);

		r_time.push_back(ms_diff(start_time, read_time));
		w_time.push_back(ms_diff(read_time, write_time));

		if (count == 100) {
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

			printf("Total: %f\t\tRead: %f ms, stdev: %f Write: %f ms, stdev: %f\n", r_mean+w_mean, r_mean, r_stdev, w_mean, w_stdev);
			if ((r_stdev+w_stdev) > 1.0) {
				copy(r_time.begin(), r_time.end(), std::ostream_iterator<double>(std::cout, " "));
				printf("\n");
			}
			r_time.clear();
			w_time.clear();

			runs++;
			if (runs >= 10) {
				break;
			}
			count = 0;
		}
#else
		/*
			clock_gettime(TEST_CLOCK, &end_time);
			if (count == 100) {
			runs++;
			if (runs >= 20) {
			printf("Total RW: %d, Time: %fms, %fmsprw\n",
			runs*100, ms_diff(begin_time, end_time),
			ms_diff(begin_time, end_time) / (runs*100));
			break;
			}
			count = 0;
			}
			*/
#endif

#ifdef LOGFILE
		clock_gettime(TEST_CLOCK, &begin_time);
		if (cm730.BulkRead() == CM730::SUCCESS) {

			cm730.WriteWord(25, MX28::P_GOAL_POSITION_L, target_pos, 0);
			target_pos+=delta;
			if (target_pos > 4095 ) {
				delta = 2*delta;
				if (delta > 256) {
					cm730.WriteWord(25, MX28::P_GOAL_POSITION_L, target_pos, 0);
					StopLogging();
					break;
				}
				delta = -1 * delta;
			}
			if (target_pos < 0) {
				delta = 2*delta;
				delta = -1 * delta;
			}

			printf("taget_pos: %d delta: %d\n",
					target_pos, delta);
		}
		else {
			printf("Couldn't read data!\n");
		}


		// gp, pos, vel, torque, volt, temp
		/*
			for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++)
			{
			m_LogFileStream << ","; //MotionStatus::m_CurrentJoints.GetValue(id)
			m_LogFileStream << cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L) << ",";
			m_LogFileStream << cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_SPEED_L) << ",";
			m_LogFileStream << cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_LOAD_L) << ",";
			m_LogFileStream << cm730.m_BulkReadData[id].ReadByte(MX28::P_PRESENT_VOLTAGE) << ",";
			m_LogFileStream << cm730.m_BulkReadData[id].ReadByte(MX28::P_PRESENT_TEMPERATURE) << ",";
			}
			*/

		//printf("taget_pos: %d current: %d delta: %d\n",
		//		target_pos, cm730.m_BulkReadData[25].ReadWord(MX28::P_PRESENT_POSITION_L), delta);
		m_LogFileStream << ms_diff(start_time, begin_time) << ",";

		m_LogFileStream << target_pos << ","; //MotionStatus::m_CurrentJoints.GetValue(id)
		m_LogFileStream << cm730.m_BulkReadData[25].ReadWord(MX28::P_PRESENT_POSITION_L) << ",";
		m_LogFileStream << cm730.m_BulkReadData[25].ReadWord(MX28::P_PRESENT_SPEED_L) << ",";
		m_LogFileStream << cm730.m_BulkReadData[25].ReadWord(MX28::P_PRESENT_LOAD_L) << ",";
		m_LogFileStream << cm730.m_BulkReadData[25].ReadByte(MX28::P_PRESENT_VOLTAGE) << ",";
		m_LogFileStream << cm730.m_BulkReadData[25].ReadByte(MX28::P_PRESENT_TEMPERATURE) << ",";
		/*

			m_LogFileStream << cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L) << ",";
			m_LogFileStream << cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L) << ",";
			m_LogFileStream << cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L) << ",";
			m_LogFileStream << cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L) << ",";
			*/
		/*
			m_LogFileStream << cm730->m_BulkReadData[FSR::ID_L_FSR].ReadByte(FSR::P_FSR_X) << ",";
			m_LogFileStream << cm730->m_BulkReadData[FSR::ID_L_FSR].ReadByte(FSR::P_FSR_Y) << ",";
			m_LogFileStream << cm730->m_BulkReadData[FSR::ID_R_FSR].ReadByte(FSR::P_FSR_X) << ",";
			m_LogFileStream << cm730->m_BulkReadData[FSR::ID_R_FSR].ReadByte(FSR::P_FSR_Y) << ",";
			*/
		m_LogFileStream << std::endl;

		clock_gettime(TEST_CLOCK, &end_time);
		printf("Log Time: %fms\n", ms_diff(begin_time, end_time));
#endif

		//usleep(500);
	}
	printf("Total time: %fms\n", ms_diff(start_time, end_time));

	return 0;
}

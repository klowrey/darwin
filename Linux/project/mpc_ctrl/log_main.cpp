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

#define SINGLE_MX28

using namespace Robot;

std::ofstream m_LogFileStream;

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

#ifdef SINGLE_MX28	
	m_LogFileStream << "TIME_MS,GOAL,POS,VEL,LOAD,VOLT,TEMP"<< std::endl;
#else
	for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++)
	{
		m_LogFileStream << "ID_" << id << "_GP,ID_" << id << "_PP,";
	}
	m_LogFileStream << "GyroFB,GyroRL,AccelFB,AccelRL";
	//m_LogFileStream << "L_FSR_X,L_FSR_Y,R_FSR_X,R_FSR_Y";
	m_LogFileStream << << std::endl;
#endif
}

void StopLogging()
{
	m_LogFileStream.close();
}


int main(int argc, char* argv[])
{
	printf( "\n===== Data Logger for System ID for DARwIn =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);
#ifdef SINGLE_MX28
	cm730.MakeBulkReadPacketServo25();
#else
	cm730.MakeBulkReadPacketMPC();
#endif
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

	static struct timespec start_time;
	static struct timespec begin_time;
	static struct timespec end_time;
	std::vector<double> r_time;
	std::vector<double> w_time;
	clockid_t TEST_CLOCK = CLOCK_MONOTONIC;

	// file logging
	StartLogging(p_gain, d_gain);
#ifdef SINGLE_MX28
	int target_pos = 0, delta = 2;
	cm730.WriteWord(25, MX28::P_TORQUE_ENABLE, 0, 0);
	cm730.WriteByte(25, MX28::P_P_GAIN, p_gain, 0);
	cm730.WriteByte(25, MX28::P_D_GAIN, d_gain, 0);
	cm730.WriteWord(25, MX28::P_GOAL_POSITION_L, target_pos, 0);
	usleep(2000000);
	printf("At Zero Position\n");
#else
	int target_pos[21];
	for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++)
	{
		target_pos[id] = 0;
		cm730.WriteWord(joint, MX28::P_TORQUE_ENABLE, 0, 0);
		cm730.WriteByte(joint, MX28::P_P_GAIN, p_gain, 0);
		cm730.WriteByte(joint, MX28::P_D_GAIN, d_gain, 0);
	}
#endif


	clock_gettime(TEST_CLOCK, &start_time);


	while(1)
	{
		clock_gettime(TEST_CLOCK, &begin_time);

		// Read and Write
		if (cm730.BulkRead() == CM730::SUCCESS) {
#ifdef SINGLE_MX28
			cm730.WriteWord(25, MX28::P_GOAL_POSITION_L, target_pos, 0);
			target_pos+=delta;
			if (target_pos > 4095 ) {
				delta = 2*delta;
				if (delta > 256) {
					cm730.WriteWord(25, MX28::P_GOAL_POSITION_L, 0, 0);
					StopLogging();
					break;
				}
				delta = -1 * delta;
			}
			if (target_pos < 0) {
				delta = 2*delta;
				delta = -1 * delta;
			}

			printf("taget_pos: %d delta: %d\n", target_pos, delta);
#else
			// read from file? generate trajectories for all 20 joints?
#endif
		}
		else {
			printf("Couldn't read data!\n");
		}


		// gp, pos, vel, torque, volt, temp

		//printf("taget_pos: %d current: %d delta: %d\n",
		//		target_pos, cm730.m_BulkReadData[25].ReadWord(MX28::P_PRESENT_POSITION_L), delta);
		m_LogFileStream << ms_diff(start_time, begin_time) << ",";
#ifdef SINGLE_MX28

		m_LogFileStream << target_pos << ",";
		m_LogFileStream << cm730.m_BulkReadData[25].ReadWord(MX28::P_PRESENT_POSITION_L) << ",";
		m_LogFileStream << cm730.m_BulkReadData[25].ReadWord(MX28::P_PRESENT_SPEED_L) << ",";
		m_LogFileStream << cm730.m_BulkReadData[25].ReadWord(MX28::P_PRESENT_LOAD_L) << ",";
		m_LogFileStream << cm730.m_BulkReadData[25].ReadByte(MX28::P_PRESENT_VOLTAGE) << ",";
		m_LogFileStream << cm730.m_BulkReadData[25].ReadByte(MX28::P_PRESENT_TEMPERATURE) << ",";
#else
		for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++)
		{
			m_LogFileStream << target_pos[id] ",";
			m_LogFileStream << cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L) << ",";
			m_LogFileStream << cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_SPEED_L) << ",";
			m_LogFileStream << cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_LOAD_L) << ",";
			m_LogFileStream << cm730.m_BulkReadData[id].ReadByte(MX28::P_PRESENT_VOLTAGE) << ",";
			m_LogFileStream << cm730.m_BulkReadData[id].ReadByte(MX28::P_PRESENT_TEMPERATURE) << ",";
		}

		m_LogFileStream << cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L) << ",";
		m_LogFileStream << cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L) << ",";
		m_LogFileStream << cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L) << ",";
		m_LogFileStream << cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L) << ",";

		m_LogFileStream << cm730->m_BulkReadData[FSR::ID_L_FSR].ReadByte(FSR::P_FSR_X) << ",";
		m_LogFileStream << cm730->m_BulkReadData[FSR::ID_L_FSR].ReadByte(FSR::P_FSR_Y) << ",";
		m_LogFileStream << cm730->m_BulkReadData[FSR::ID_R_FSR].ReadByte(FSR::P_FSR_X) << ",";
		m_LogFileStream << cm730->m_BulkReadData[FSR::ID_R_FSR].ReadByte(FSR::P_FSR_Y) << ",";
#endif
		m_LogFileStream << std::endl;

		clock_gettime(TEST_CLOCK, &end_time);
		printf("Log Time: %fms\n", ms_diff(begin_time, end_time));

		//usleep(500);
	}
	printf("Total time: %fms\n", ms_diff(start_time, end_time));

	return 0;
}

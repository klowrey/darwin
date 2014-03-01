/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <pthread.h>
#include <termios.h>
#include <term.h>

#include "LinuxDARwIn.h"
#include "MPCData.h"

#define U2D_DEV_NAME        "/dev/ttyUSB0"

using namespace Robot;


void change_current_dir()
{
	char exepath[1024] = {0};
	if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
		chdir(dirname(exepath));
}

int _getch()
{
	struct termios oldt, newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

void* log_thread(void* ptr)
{
	while(1) {
		int ch = _getch();
		if(ch == 0x20) {
			if(MotionManager::GetInstance()->IsLogging() == true) {
				MotionManager::GetInstance()->StopLogging();
			}
			else {
				MotionManager::GetInstance()->StartLogging();
			}
		}
	}
	return NULL;
}


int main(int argc, char* argv[])
{
	double dt = 0.02;
	if (argc > 2){
		dt = atof(argv[2]);
	}
	static struct timespec start_time;
	static struct timespec end_time;

	printf( "\n===== Trajectory following Tutorial for DARwIn =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730(U2D_DEV_NAME);
	CM730 cm730(&linux_cm730);
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
		return 0;
	}

	//MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
	//MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
	//LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
	//motion_timer->Start();
	/////////////////////////////////////////////////////////////////////

	int n = 0;
	int param[JointData::NUMBER_OF_JOINTS * 5];
	int wGoalPosition, wStartPosition, wDistance;

	// Move to initial position
	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
		wGoalPosition = 2048; //Walking::GetInstance()->m_Joint.GetValue(id);
		if( wStartPosition > wGoalPosition )
			wDistance = wStartPosition - wGoalPosition;
		else
			wDistance = wGoalPosition - wStartPosition;

		wDistance >>= 2;
		if( wDistance < 8 )
			wDistance = 8;

		param[n++] = id;
		param[n++] = CM730::GetLowByte(wGoalPosition);
		param[n++] = CM730::GetHighByte(wGoalPosition);
		param[n++] = CM730::GetLowByte(wDistance);
		param[n++] = CM730::GetHighByte(wDistance);
	}
	cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);	

	printf("Press the ENTER key to begin!\n");
	getchar();

	Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	MotionManager::GetInstance()->SetEnable(true);

	printf("Press the SPACE key to log!\n");
	pthread_t thread_t;
	pthread_create(&thread_t, NULL, log_thread, NULL);


	double timestamp = 0.0;

	// file stuff start
	FILE * trajectory_file  = fopen(filename, "rb");

	if (!trajectory_file) {
		printf("Unable to open trajectory file %s!\n", filename);
		return 0;
	}

	fseek(trajectory_file, 0, SEEK_END);
	unsigned long fileLen = ftell(trajectory_file);
	fseek(trajectory_file, 0, SEEK_SET);

	if (fileLen % sizeof(double)) {
		printf("Mis-aligned data from trajectory file %s\n", filename);
		close(trajectory_file);
		return 0;
	}

	printf("%f Data samples from file sized %d\n", fileLen/sizeof(double), %d);
	double* binary_line = (double*) malloc(fileLen);

	fread((void*)binary_line, fileLen, 1, trajectory_file);
	fclose(trajectory_file);
	// file stuff done

	double time_passed = 0;
	clock_gettime(CLOCK_MONOTONIC, &start_time);
	clock_gettime(CLOCK_MONOTONIC, &end_time);

	int STRIDE = 0;
	// there is still data in the buffer
	if (timestamp == binary_line[STRIDE]) {
		timestamp += dt;

		while ( time_passed < timestamp) {
			printf("%f is < %f\n", time_passed, timestamp);
			double radian = 0;
			int joint_num = 0;

			// INTERPOLATION STEP

			// prepare commands to joints
			for (int joint=1; joint<=JointData::ID_R_HIP_ROLL; joint++) {
				joint_num++;
				param[n++] = joint_num;
				value = radian2joint(binary_line[STRIDE+joint]); // in joint space?
				param[n++] = CM730::GetLowByte(value);
				param[n++] = CM730::GetHighByte(value);

				joint_num++;
				param[n++] = joint_num;
				value = radian2joint(binary_line[STRIDE+joint+9]); // in joint space?
				param[n++] = CM730::GetLowByte(value);
				param[n++] = CM730::GetHighByte(value);
			}

			id = JointData::ID_HEAD_PAN;
			param[n++] = id;
			value = radian2joint(binary_line[STRIDE+id]);
			param[n++] = CM730::GetLowByte(value);
			param[n++] = CM730::GetHighByte(value);
			joint_num++;

			id = JointData::ID_HEAD_TILT;
			param[n++] = id;
			value = radian2joint(binary_line[STRIDE+id]);
			param[n++] = CM730::GetLowByte(value);
			param[n++] = CM730::GetHighByte(value);
			joint_num++;

			printf("\nParsed %d joint positions.\n", joint_num);

			clock_gettime(CLOCK_MONOTONIC, &end_time);
			time_passed = sec_diff(start_time, end_time);
		}

		STRIDE += DATA_SIZE;
	}
	else {

		printf("bad timestamp!");
		break;
	}


	return 0;
}


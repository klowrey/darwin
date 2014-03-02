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

void initial_pose(double* joints, CM730 * cm730) {
	int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
	int wGoalPosition, wStartPosition, wDistance;
	int joint_num = 0;
	int n = 0;
	int id = 0;

	for (int joint=1; joint<=JointData::ID_R_HIP_ROLL; joint++) {
		wStartPosition = MotionStatus::m_CurrentJoints.GetValue(joint);

		joint_num++;
		param[n++] = joint_num;
		wGoalPosition = radian2joint(joints[joint]); // in joint space?
		if( wStartPosition > wGoalPosition )
			wDistance = wStartPosition - wGoalPosition;
		else
			wDistance = wGoalPosition - wStartPosition;
		wDistance >>= 2;
		if( wDistance < 8 )
			wDistance = 8;
		param[n++] = CM730::GetLowByte(wGoalPosition);
		param[n++] = CM730::GetHighByte(wGoalPosition);
		param[n++] = CM730::GetLowByte(wDistance);
		param[n++] = CM730::GetHighByte(wDistance);

		joint_num++;
		param[n++] = joint_num;
		wGoalPosition = radian2joint(joints[joint+9]); // in joint space?
		if( wStartPosition > wGoalPosition )
			wDistance = wStartPosition - wGoalPosition;
		else
			wDistance = wGoalPosition - wStartPosition;
		wDistance >>= 2;
		if( wDistance < 8 )
			wDistance = 8;
		param[n++] = CM730::GetLowByte(wGoalPosition);
		param[n++] = CM730::GetHighByte(wGoalPosition);
		param[n++] = CM730::GetLowByte(wDistance);
		param[n++] = CM730::GetHighByte(wDistance);
	}

	id = JointData::ID_HEAD_PAN;
	param[n++] = id;
	wGoalPosition = radian2joint(joints[id]);
	if( wStartPosition > wGoalPosition )
		wDistance = wStartPosition - wGoalPosition;
	else
		wDistance = wGoalPosition - wStartPosition;
	wDistance >>= 2;
	if( wDistance < 8 )
		wDistance = 8;
	param[n++] = CM730::GetLowByte(wGoalPosition);
	param[n++] = CM730::GetHighByte(wGoalPosition);
	param[n++] = CM730::GetLowByte(wDistance);
	param[n++] = CM730::GetHighByte(wDistance);
	joint_num++;

	id = JointData::ID_HEAD_TILT;
	param[n++] = id;
	wGoalPosition = radian2joint(joints[id]);
	if( wStartPosition > wGoalPosition )
		wDistance = wStartPosition - wGoalPosition;
	else
		wDistance = wGoalPosition - wStartPosition;
	wDistance >>= 2;
	if( wDistance < 8 )
		wDistance = 8;
	param[n++] = CM730::GetLowByte(wGoalPosition);
	param[n++] = CM730::GetHighByte(wGoalPosition);
	param[n++] = CM730::GetLowByte(wDistance);
	param[n++] = CM730::GetHighByte(wDistance);
	joint_num++;

	cm730->SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);	
}


int main(int argc, char* argv[])
{
	char* filename;
	if (argc > 1){
		filename = argv[1];
	}
	printf("Reading from file %s\n", filename);

	double dt = 0.02;
	if (argc > 2){
		dt = atof(argv[2]);
	}
	printf("Timestep is set at %d\n", dt);

	int STRIDE = 0;
	int LINE_SIZE = 21;

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
	//int param[JointData::NUMBER_OF_JOINTS * 5];
	int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];

	printf("Press the ENTER key to begin!\n");
	getchar();

	//Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
	//Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	//MotionManager::GetInstance()->SetEnable(true);

	//printf("Press the SPACE key to log!\n");
	//pthread_t thread_t;
	//pthread_create(&thread_t, NULL, log_thread, NULL);
	
	



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

	printf("%d %d\n", fileLen, fileLen % sizeof(double));
	if (fileLen % sizeof(double)) {
		printf("Mis-aligned data from trajectory file %s\n", filename);
		fclose(trajectory_file);
		return 0;
	}

	int samples = fileLen/sizeof(double)/LINE_SIZE;

	printf("Grabbed %d data samples from file sized %lu bytes.\n", samples, fileLen);

	double* binary_line = (double*) malloc(fileLen);
	double* joint_data;
	double* prev_joint;
	double* interp = (double*) malloc(LINE_SIZE*sizeof(double));

	fread((void*)binary_line, fileLen, 1, trajectory_file);
	fclose(trajectory_file);
	// file stuff done

	double time_passed = 0;
	clock_gettime(CLOCK_MONOTONIC, &start_time);
	clock_gettime(CLOCK_MONOTONIC, &end_time);

	prev_joint = &(binary_line[0]);
	timestamp += dt;
	STRIDE += LINE_SIZE;

	// Move to initial position
	initial_pose(prev_joint, &cm730);

	for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<JointData::NUMBER_OF_JOINTS; joint++) {
		cm730.WriteByte(joint, MX28::P_P_GAIN, 32, 0);
		cm730.WriteByte(joint, MX28::P_D_GAIN, 0, 0);
		cm730.WriteWord(joint, MX28::P_MOVING_SPEED_L, 0, 0);
	}

	printf("Press ENTER if happy with initial pose!\n");
	getchar();

	// there is still data in the buffer
	for (int sample = 1; sample<samples; sample++) {
		printf("%3.2f% through the file.\n", (double)sample/samples);

		// might not need to bother checking this...
		if ((timestamp - binary_line[sample*LINE_SIZE]) < 10e-6) {

			timestamp += dt;

			joint_data = &(binary_line[STRIDE]);

			// can print out 20 joint datas
			printf("t: %1.3f; ", prev_joint[0]);
			for (int joint=1; joint<20; joint++) {
				printf("%1.3f ", prev_joint[joint]);
			}
			printf("\n");

			printf("t: %1.3f; ", joint_data[0]);
			for (int joint=1; joint<20; joint++) {
				printf("%1.3f ", joint_data[joint]);
			}
			printf("\n");


			while ( time_passed < timestamp) {
				//printf("%f is < %f\n", time_passed, timestamp);
				double radian = 0;
				int joint_num = 0;
				int value;
				int id;
				int n = 0;

				clock_gettime(CLOCK_MONOTONIC, &end_time);
				time_passed = sec_diff(start_time, end_time);

				// INTERPOLATION STEP
				double percent = 1.0 - ((joint_data[0]-prev_joint[0]) / (time_passed-prev_joint[0]));
				//printf("%3.2f through interpolation.\n", percent);
				for (int joint=1; joint<20; joint++) {
					double diff = joint_data[joint] - prev_joint[joint];
					interp[joint] = prev_joint[joint] + percent*diff;
					//printf("%1.3f -- %1.3f -- %1.3f\n", prev_joint[joint], interp[joint], joint_data[joint]);
				}

				// prepare commands to joints
				for (int joint=1; joint<=JointData::ID_R_HIP_ROLL; joint++) {
					joint_num++;
					param[n++] = joint_num;
					value = radian2joint(interp[joint]); // in joint space?
					param[n++] = CM730::GetLowByte(value);
					param[n++] = CM730::GetHighByte(value);

					joint_num++;
					param[n++] = joint_num;
					value = radian2joint(interp[joint+9]); // in joint space?
					param[n++] = CM730::GetLowByte(value);
					param[n++] = CM730::GetHighByte(value);
				}

				id = JointData::ID_HEAD_PAN;
				param[n++] = id;
				value = radian2joint(interp[id]);
				param[n++] = CM730::GetLowByte(value);
				param[n++] = CM730::GetHighByte(value);
				joint_num++;

				id = JointData::ID_HEAD_TILT;
				param[n++] = id;
				value = radian2joint(interp[id]);
				param[n++] = CM730::GetLowByte(value);
				param[n++] = CM730::GetHighByte(value);
				joint_num++;

				if(joint_num > 0) {
					cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 3, joint_num, param);
				}
			}

			STRIDE += LINE_SIZE;
			prev_joint = joint_data;
		}
		else {

			printf("\nbad timestamp!\n");
			break;
		}
	}


	free(binary_line);
	free(interp);

	return 0;
}

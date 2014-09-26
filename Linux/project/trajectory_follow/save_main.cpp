#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <libgen.h>
#include <pthread.h>
#include <termios.h>
#include <term.h>

#include "MotionManager.h"
#include "MotionModule.h"
#include "JointData.h"
#include "MX28.h"
#include "LinuxDARwIn.h"
#include "MPCData.h"

#include <Eigen/Dense>

#include <boost/program_options.hpp>

#define U2D_DEV_NAME        "/dev/ttyUSB0"

using namespace Robot;
using namespace Eigen;
namespace po = boost::program_options;

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

double accel_g(int accel) {
	return ((accel-512) / 128.0); // in m/s^2
}
double gyro_radps(int gyro) {
	return (gyro-512)*0.017453229251;
}

volatile bool ready = false;

void* walk_thread(void* ptr)
{
	while(1) {
		int ch = _getch();
		if(ch == 0x20) {
			ready = true;
			printf("READY!\n");
			break;
		}
	}
	return NULL;
}

double twoKp = 10.0;
double twoKi = 0.0;
double beta = 0.5;
double q0=0.65;
double q1=-0.3;
double q2=0.3;
double q3=0.65;

void MadgwickAHRSupdateIMU(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
	double recipNorm;
	double s0, s1, s2, s3;
	double qDot1, qDot2, qDot3, qDot4;
	double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

	// Normalise accelerometer measurement
	recipNorm = 1.0/sqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;   

	// Auxiliary variables to avoid repeated arithmetic
	_2q0 = 2.0 * q0;
	_2q1 = 2.0 * q1;
	_2q2 = 2.0 * q2;
	_2q3 = 2.0 * q3;
	_4q0 = 4.0 * q0;
	_4q1 = 4.0 * q1;
	_4q2 = 4.0 * q2;
	_8q1 = 8.0 * q1;
	_8q2 = 8.0 * q2;
	q0q0 = q0 * q0;
	q1q1 = q1 * q1;
	q2q2 = q2 * q2;
	q3q3 = q3 * q3;

	// Gradient decent algorithm corrective step
	s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
	s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
	s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
	s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
	recipNorm = 1.0/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
	s0 *= recipNorm;
	s1 *= recipNorm;
	s2 *= recipNorm;
	s3 *= recipNorm;

	// Apply feedback step
	qDot1 -= beta * s0;
	qDot2 -= beta * s1;
	qDot3 -= beta * s2;
	qDot4 -= beta * s3;

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = 1.0/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

struct quat_s {
	double q0;
	double q1;
	double q2;
	double q3;
};

void MahonyAHRSupdateIMU(quat_s *q, double gx, double gy, double gz, double ax, double ay, double az, double dt) {
	double recipNorm;
	double integralFBx=0.0, integralFBy=0.0, integralFBz=0.0;
	double halfvx, halfvy, halfvz;
	double halfex, halfey, halfez;
	double qa, qb, qc;
	//double q0, q1, q2, q3;

	// Normalise accelerometer measurement
	recipNorm = 1.0/sqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;        

	// Estimated direction of gravity and vector perpendicular to magnetic flux
	halfvx = q1 * q3 - q0 * q2;
	halfvy = q0 * q1 + q2 * q3;
	halfvz = q0 * q0 - 0.5 + q3 * q3;

	// Error is sum of cross product between estimated and measured direction of gravity
	halfex = (ay * halfvz - az * halfvy);
	halfey = (az * halfvx - ax * halfvz);
	halfez = (ax * halfvy - ay * halfvx);

	// Compute and apply integral feedback if enabled
	if(twoKi > 0.0) {
		integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
		integralFBy += twoKi * halfey * dt;
		integralFBz += twoKi * halfez * dt;
		gx += integralFBx;	// apply inegral feedback
		gy += integralFBy;
		gz += integralFBz;
	}
	else {
		integralFBx = 0.0;	// prevent integral windup
		integralFBy = 0.0;
		integralFBz = 0.0;
	}

	// Apply proportional feedback
	gx += twoKp * halfex;
	gy += twoKp * halfey;
	gz += twoKp * halfez;

	// Integrate rate of change of quaternion
	gx *= (0.5 * dt);		// pre-multiply common factors
	gy *= (0.5 * dt);
	gz *= (0.5 * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 

	// Normalise quaternion
	recipNorm = 1.0/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// quat rotate to trajectory frame
	r0 = 0.6285;
	r1 = 0.6261;
	r2 = -0.3268;
	r3 = 0.3257;

	q->q0 = q0*r0 - q1*r1 - q2*r2 - q3*r3;
	q->q1 = q0*r1 + q1*r0 - q2*r3 + q3*r2;
	q->q2 = q0*r2 + q1*r3 + q2*r0 - q3*r1;
	q->q3 = q0*r3 - q1*r2 + q2*r1 + q3*r0;
}

void initial_pose(double* joints, CM730 * cm730) {
	int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
	int wGoalPosition, wStartPosition, wDistance;
	int joint_num = 0;
	int n = 0;
	int id = 0;
	int min_speed = 50;

	if (cm730->BulkRead() == CM730::SUCCESS) {

		//for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<JointData::NUMBER_OF_JOINTS; joint++) {
		//	//cm730->WriteWord(joint, MX28::P_MOVING_SPEED_L, 2, 0);
		//	cm730->WriteByte(joint, MX28::P_MOVING_SPEED_L, CM730::GetLowByte(8), 0);
		//	cm730->WriteByte(joint, MX28::P_MOVING_SPEED_H, CM730::GetHighByte(8), 0);
		//}

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
			if( wDistance < min_speed )
				wDistance = min_speed;

			//printf("%d\n", wDistance);
			param[n++] = CM730::GetLowByte(wGoalPosition);
			param[n++] = CM730::GetHighByte(wGoalPosition);
			param[n++] = CM730::GetLowByte(wDistance);
			param[n++] = CM730::GetHighByte(wDistance);

			MotionStatus::m_CurrentJoints.SetValue(joint_num, wGoalPosition);
			MotionStatus::m_CurrentJoints.SetEnable(id, true);

			joint_num++;
			param[n++] = joint_num;
			wGoalPosition = radian2joint(joints[joint+9]); // in joint space?
			if( wStartPosition > wGoalPosition )
				wDistance = wStartPosition - wGoalPosition;
			else
				wDistance = wGoalPosition - wStartPosition;
			wDistance >>= 2;
			if( wDistance < min_speed )
				wDistance = min_speed;
			param[n++] = CM730::GetLowByte(wGoalPosition);
			param[n++] = CM730::GetHighByte(wGoalPosition);
			param[n++] = CM730::GetLowByte(wDistance);
			param[n++] = CM730::GetHighByte(wDistance);

			MotionStatus::m_CurrentJoints.SetValue(joint_num, wGoalPosition);
			MotionStatus::m_CurrentJoints.SetEnable(id, true);
		}

		id = JointData::ID_HEAD_PAN;
		param[n++] = id;
		wGoalPosition = radian2joint(joints[id]);
		if( wStartPosition > wGoalPosition )
			wDistance = wStartPosition - wGoalPosition;
		else
			wDistance = wGoalPosition - wStartPosition;
		wDistance >>= 2;
		if( wDistance < min_speed )
			wDistance = min_speed;
		param[n++] = CM730::GetLowByte(wGoalPosition);
		param[n++] = CM730::GetHighByte(wGoalPosition);
		param[n++] = CM730::GetLowByte(wDistance);
		param[n++] = CM730::GetHighByte(wDistance);
		MotionStatus::m_CurrentJoints.SetValue(id, wGoalPosition);
		MotionStatus::m_CurrentJoints.SetEnable(id, true);
		joint_num++;

		id = JointData::ID_HEAD_TILT;
		param[n++] = id;
		wGoalPosition = radian2joint(joints[id]);
		if( wStartPosition > wGoalPosition )
			wDistance = wStartPosition - wGoalPosition;
		else
			wDistance = wGoalPosition - wStartPosition;
		wDistance >>= 2;
		if( wDistance < min_speed )
			wDistance = min_speed;
		param[n++] = CM730::GetLowByte(wGoalPosition);
		param[n++] = CM730::GetHighByte(wGoalPosition);
		param[n++] = CM730::GetLowByte(wDistance);
		param[n++] = CM730::GetHighByte(wDistance);
		MotionStatus::m_CurrentJoints.SetValue(id, wGoalPosition);
		MotionStatus::m_CurrentJoints.SetEnable(id, true);
		joint_num++;

		cm730->SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);	
	}
	else {
		printf("Couldn't read from ROBOT!!\n");
	}
}

int samples;

double* open_read_binary(const char* filename, int LINE_SIZE) {
	FILE * trajectory_file  = fopen(filename, "rb");

	if (!trajectory_file) {
		printf("Unable to open trajectory file %s!\n", filename);
		return 0;
	}

	fseek(trajectory_file, 0, SEEK_END);
	unsigned long fileLen = ftell(trajectory_file);
	fseek(trajectory_file, 0, SEEK_SET);

	if (fileLen % sizeof(double)) {
		printf("Mis-aligned data from trajectory file %s; %d extra doubles\n",
				filename, fileLen % sizeof(double));
		fclose(trajectory_file);
		return 0;
	}

	samples = fileLen/sizeof(double)/LINE_SIZE;

	printf("Grabbed %d data samples from file sized %lu bytes.\n", samples, fileLen);

	double* data_ptr = (double*) malloc(fileLen);

	fread((void*)data_ptr, fileLen, 1, trajectory_file);
	fclose(trajectory_file);

	return data_ptr;
}

void print_status(CM730 * cm730) {
	printf("Joint Info:\n");
	int out;
	for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<JointData::NUMBER_OF_JOINTS; joint++) {
		printf("Joint: [%d]\t", joint);
		cm730->ReadByte(joint, MX28::P_P_GAIN, &out, 0);
		printf("P: %d ", out);
		cm730->ReadByte(joint, MX28::P_I_GAIN, &out, 0);
		printf("I: %d ", out);
		cm730->ReadByte(joint, MX28::P_D_GAIN, &out, 0);
		printf("D: %d\t", out);
		cm730->ReadByte(joint, MX28::P_PRESENT_VOLTAGE, &out, 0);
		printf("Volts: %d ", out);
		cm730->ReadWord(joint, MX28::P_MOVING_SPEED_L, &out, 0);
		printf("Speed: %3.1f %% max speed\n", 100.0*out/1024.0);
	}
}


void set_positions(double percent, double* interp) {
	// prepare commands to joints
	int joint_num = 0;
	for (int joint=1; joint<=JointData::ID_R_HIP_ROLL; joint++) {
		joint_num++;
		MotionStatus::m_CurrentJoints.SetValue(joint_num, radian2joint(interp[joint]));
		MotionStatus::m_CurrentJoints.SetEnable(joint_num, true);

		joint_num++;
		MotionStatus::m_CurrentJoints.SetValue(joint_num, radian2joint(interp[joint+9]));
		MotionStatus::m_CurrentJoints.SetEnable(joint_num, true);
	}

	int id;
	id = JointData::ID_HEAD_PAN;
	MotionStatus::m_CurrentJoints.SetValue(id, radian2joint(interp[id]));
	MotionStatus::m_CurrentJoints.SetEnable(id, true);
	joint_num++;

	id = JointData::ID_HEAD_TILT;
	MotionStatus::m_CurrentJoints.SetValue(id, radian2joint(interp[id]));
	MotionStatus::m_CurrentJoints.SetEnable(id, true);
	joint_num++;
}

int main(int argc, char* argv[])
{
	bool engage;
	int p_gain;
	int i_gain;
	int d_gain;
	int sref_size;
	bool use_gains;
	bool use_quat;
	bool use_vel;
	double dt;
	std::string *filename = new std::string();

	try {
		po::options_description desc("Allowed options");
		desc.add_options()
			("help", "Usage guide")
			("engage,e", po::value<bool>(&engage)->default_value(false),
			 "Engage motors for live run, or dry run of file.")
			("trajectory,q", po::value<std::string>(filename)->required(), "Binary file of trajectory joint data")
			("sref,s", po::value<int>(&sref_size)->default_value(0), "Size of SREF vector; helps with data organization")
			("gains,g", po::value<bool>(&use_gains)->default_value(false), "Use feedback gains if available")
			("quat", po::value<bool>(&use_quat)->default_value(false), "Use quat feedback gains if available")
			("vels", po::value<bool>(&use_vel)->default_value(false), "Use vel feedback gains if available")
			("dt,t", po::value<double>(&dt)->default_value(0.02),
			 "Timestep in binary file -- checks for file corruption")
			("p_gain,p", po::value<int>(&p_gain)->default_value(20), "P gain of PiD controller, 2-160")
			("i_gain,i", po::value<int>(&i_gain)->default_value(0), "I gain of PiD controller, 0-32")
			("d_gain,d", po::value<int>(&d_gain)->default_value(0), "D gain of PiD controller, 0-32")
			;

		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, desc), vm);

		if (vm.count("help")) {
			std::cout << desc << std::endl;
			return 0;
		}

		po::notify(vm);
	}
	catch(std::exception& e) {
		std::cerr << "Error: " << e.what() << "\n";
		return 0;
	}
	catch(...) {
		std::cerr << "Unknown error!\n";
		return 0;
	}

	printf("Engaging joints: %s\n", engage ? "true":"false");
	printf("Reading trajectory from%s\n", filename->c_str());
	printf("Size of SREF vector in file: %d\n", sref_size);
	printf("\tGain use: %s\n", use_gains ? "true":"false");
	printf("\tQuat use: %s\n", use_quat ? "true":"false");
	printf("\tVels use: %s\n", use_avel ? "true":"false");
	printf("Timestep is set at %f\n", dt);
	printf("P Gain is set at %d\n", p_gain);
	printf("I Gain is set at %d\n", i_gain);
	printf("D Gain is set at %d\n", d_gain);

	int STRIDE = 0;
	// TODO line_size might include max velocities --> 41
	const int A_size = 20*sref_size; 
	const int LINE_SIZE = 1 + 20 + sref_size + A_size; // pos / uref, A matrix, sref
	printf("\nFile line size should be %d\n", LINE_SIZE);

	static struct timespec start_time;
	static struct timespec end_time;
	static struct timespec final_time;

	printf( "\n===== Trajectory following Tutorial for DARwIn =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730(U2D_DEV_NAME);
	CM730 cm730(&linux_cm730);
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
		return 0;
	}
	cm730.MakeBulkReadPacketMPC();

	//MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
	//MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
	//LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
	//motion_timer->Start();
	/////////////////////////////////////////////////////////////////////

	int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];

	printf("Press the ENTER key to begin!\n");
	getchar();


	// file stuff start
	double* binary_line = open_read_binary(filename->c_str(), LINE_SIZE);

	if (binary_line == NULL) {
		printf("Bad trajectory\n");
		return 0;
	}

	double* joint_data;
	double* prev_joint;
	double* interp = (double*) malloc(LINE_SIZE*sizeof(double));
	double* s_vec= (double*) malloc(sref_size*sizeof(double));

	//fread((void*)binary_line, fileLen, 1, trajectory_file);
	//fclose(trajectory_file);
	// file stuff done

	double time_passed = 0;

	prev_joint = &(binary_line[0]);

	double timestamp = 0.0;
	timestamp += dt;
	STRIDE += LINE_SIZE;

	// Move to initial position
	initial_pose(prev_joint, &cm730);

	printf("t: %1.3f; ", prev_joint[0]);
	for (int joint=1; joint<20; joint++) {
		printf("%1.3f ", prev_joint[joint]);
	}
	printf("\n");
	printf("Timestep from file: %f\n", binary_line[LINE_SIZE] - binary_line[0]);

	printf("Press ENTER if happy with initial pose!\n");
	getchar();

	if (engage) {
		MotionManager::GetInstance()->SetEnable(true);
	}

	int max_speed = 1024;
	for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<JointData::NUMBER_OF_JOINTS; joint++) {
		cm730.WriteByte(joint, MX28::P_P_GAIN, p_gain, 0);
		cm730.WriteByte(joint, MX28::P_I_GAIN, i_gain, 0);
		cm730.WriteByte(joint, MX28::P_D_GAIN, d_gain, 0);
		cm730.WriteWord(joint, MX28::P_MOVING_SPEED_L, max_speed, 0);

		MotionStatus::m_CurrentJoints.SetPGain(joint, p_gain);
		MotionStatus::m_CurrentJoints.SetIGain(joint, i_gain);
		MotionStatus::m_CurrentJoints.SetDGain(joint, d_gain);
	}

	print_status(&cm730);

	printf("Streaming Started. Press SPACE to play trajectory\n");

	pthread_t thread_t;
	pthread_create(&thread_t, NULL, walk_thread, NULL);

	while (!ready)
	{
		MotionManager::GetInstance()->Process();
	}
	pthread_join(thread_t, NULL);

	if (MotionManager::GetInstance()->IsLogging() == false) {
		MotionManager::GetInstance()->StartLogging();
	}

	quat_s quat;
	quat.q0 = 0.6285;
	quat.q1 = 0.6261;
	quat.q2 = -0.3268;
	quat.q3 = 0.3257;

	clock_gettime(CLOCK_MONOTONIC, &start_time);
	clock_gettime(CLOCK_MONOTONIC, &end_time);

	std::ofstream out("TEST.txt");

	// there is still data in the buffer
	for (int sample = 1; sample<samples; sample++) {
		printf("%3.2f percent through the file.\n", ((double)sample/samples)*100.0);

		// might not need to bother checking this...
		if (abs(timestamp - binary_line[sample*LINE_SIZE]) < 10e-6) {

			joint_data = &(binary_line[STRIDE]);

			//printf("%f ??? %f\n", time_passed, timestamp);
			while ( time_passed < timestamp) {
				//printf("%f is < %f\n", time_passed, timestamp);
				int joint_num = 0;
				int value;
				int id;

				//double percent = 1.0 - ((joint_data[0]-prev_joint[0]) / (time_passed-prev_joint[0]));
				double percent = ((time_passed-prev_joint[0]) / (joint_data[0]-prev_joint[0]));
				//printf("percent: %f, %f %f\n", percent, prev_joint[0], joint_data[0]);

				clock_gettime(CLOCK_MONOTONIC, &end_time);
				//double t_now = sec_diff(start_time, end_time);
				double dt_interp = sec_diff(start_time, end_time) - time_passed;
				time_passed = time_passed + dt_interp; 

				// INTERPOLATE POSITION aka (uref+du)
				for (int joint=1; joint<=20; joint++) {
					double diff = joint_data[joint] - prev_joint[joint];
					interp[joint] = prev_joint[joint] + percent*diff;
					//printf("%1.3f -- %1.3f -- %1.3f\n", prev_joint[joint], interp[joint], joint_data[joint]);
				}

				for (int idx=21; idx<LINE_SIZE; idx++) {
					double diff = joint_data[idx] - prev_joint[idx];
					interp[idx] = prev_joint[idx] + percent*diff;
				}
				if (sref_size > 1 && use_gains == true) {
					// TODO probably not a good idea, but for now...
					// Interpolate SREF & A matrix

					// (uref + du) + A * (s - sref)
					int i = 0;
					for(int id = 1; id <= 17; id+=2) // Right Joints
						s_vec[i++] = joint2radian(cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L));
					for(int id = 2; id <= 18; id+=2) // Left Joints
						s_vec[i++] = joint2radian(cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L));
					for(int id = 19; id <= 20; id++) // Head Joints
						s_vec[i++] = joint2radian(cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L));

					for(int id = 1; id <= 17; id+=2) // Right Joints
						s_vec[i++] = j_rpm2rads_ps(cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_SPEED_L));
					for(int id = 2; id <= 18; id+=2) // Left Joints
						s_vec[i++] = j_rpm2rads_ps(cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_SPEED_L));
					for(int id = 19; id <= 20; id++) // Head Joints
						s_vec[i++] = j_rpm2rads_ps(cm730.m_BulkReadData[id].ReadWord(MX28::P_PRESENT_SPEED_L));

					Map<VectorXd> sref(interp+1+20, sref_size); // better way of indexing this?

					double gyro_x = -1*gyro2rads_ps(cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L));
					double gyro_y = -1*gyro2rads_ps(cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L));
					double gyro_z = gyro2rads_ps(cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Z_L));

					double accel_x = accel_g(cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L));
					double accel_y = -1*accel_g(cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L));
					double accel_z = accel_g(cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Z_L));

					//printf("dt_interp: %f\n", dt_interp);

					// sref 40
					// no pose, orientation
					// sref 47
					// 3 position, 4 quat
					// sref 53
					// 3 position, 4 quat, 3 vel, 3 ang_vel
					
					if (sref_size > 40) {
						// zero out all sref error first
						for (int i=40; i<sref_size; i++) {
							s_vec[i] = sref(i);
						}
						// apply appropriate data from sensors
						if (use_quat == true && sref >= 47) {
							MahonyAHRSupdateIMU(&quat, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, dt_interp);
							printf("%1.3f %1.3f %1.3f %1.3f\n",
									quat.q0,
									quat.q1,								
									quat.q2,
									quat.q3);
							s_vec[43] = quat.q0;
							s_vec[44] = quat.q1;
							s_vec[45] = quat.q2;
							s_vec[46] = quat.q3;
						}
						if (use_avel == true && sref >= 53) {
							// TODO filter gyro before using this
							s_vec[50] = gyro_x;
							s_vec[51] = gyro_y;
							s_vec[52] = gyro_z;
						}
					}

					Map<VectorXd> s(s_vec, sref_size);

					//printf("Error sum: %f\n", (s-sref).sum());

					Map<MatrixXd> A(interp+1+20+sref_size, 20, sref_size); // better way of indexing this?
					//printf("A: %d rows, %d cols\n", A.rows(), A.cols());
					//printf("s: %d rows, %d cols\n", s.rows(), s.cols());
					//printf("sref: %d rows, %d cols\n", sref.rows(), sref.cols());
					MatrixXd vec =  A * (s-sref);

					Map<MatrixXd> (s_vec, vec.rows(), vec.cols()) = vec;

					for (int idx=1; idx<=20; idx++) {
						interp[idx] = interp[idx] + s_vec[idx-1];
					}
				}

				/// UNITS MAKING A BIG DIFFERENCE?
				//MadgwickAHRSupdateIMU(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, dt_interp);
				out << time<<","<<quat.q0<<","<<quat.q1<<","<<quat.q2<<","<<quat.q3<<std::endl;

				if (use_gains == false && sref_size >1) {
					// get the sref instead of the uref
					double* sref = interp + 20; // remember that interp should have time at index 0
					//for (int i=0;i<20;i++)
					//	printf("%1.3f ", sref[i]);
					//printf("\n");
					set_positions(percent, sref);
				}
				else {
					set_positions(percent, interp);
				}

				MotionManager::GetInstance()->Process(); // does the logging
			}

			//print_status(&cm730);

			STRIDE += LINE_SIZE;
			prev_joint = joint_data;

			timestamp += dt;
		}
		else {
			printf("\nbad timestamp! %f and %f\n", timestamp, binary_line[sample*LINE_SIZE]);
			break;
		}
	}


	clock_gettime(CLOCK_MONOTONIC, &final_time);
	printf("Trajectory took %f seconds\n", sec_diff(start_time, final_time));

	printf("Streaming Started. Press SPACE to play trajectory\n");

	//pthread_t thread_t;
	pthread_create(&thread_t, NULL, walk_thread, NULL);

	while (!ready)
	{
		MotionManager::GetInstance()->Process();
	}
	pthread_join(thread_t, NULL);


	if (MotionManager::GetInstance()->IsLogging() == true) {
		printf("Logging to file... ");
		MotionManager::GetInstance()->StopLogging();
		printf("Done.\n");
	}

	free(binary_line);
	free(interp);
	free(s_vec);

	print_status(&cm730);
	delete filename;

	return 0;
}


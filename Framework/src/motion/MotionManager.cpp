/*
 *   MotionManager.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "FSR.h"
#include "MX28.h"
#include "MotionManager.h"

#define SEND_BUF_SIZE 75 
#define MPC_PORT 13131

using namespace Robot;
using namespace std;

MotionManager* MotionManager::m_UniqueInstance = new MotionManager();

MotionManager::MotionManager() :
	m_CM730(0),
	m_ProcessEnable(false),
	m_Enabled(false),
	m_IsRunning(false),
	m_IsThreadRunning(false),
	m_IsLogging(false),
	DEBUG_PRINT(false)
{
	for(int i = 0; i < JointData::NUMBER_OF_JOINTS; i++)
		m_Offset[i] = 0;

	clock_gettime(CLOCK_MONOTONIC,&start_time);
}

MotionManager::~MotionManager()
{
}

bool MotionManager::Initialize(CM730 *cm730)
{
	int value, error;

	m_CM730 = cm730;
	m_Enabled = false;
	m_ProcessEnable = true;

	if(m_CM730->Connect() == false)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Fail to connect CM-730\n");
		return false;
	}

	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "ID:%d initializing...", id);

		if(m_CM730->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, &error) == CM730::SUCCESS)
		{
			MotionStatus::m_CurrentJoints.SetValue(id, value);
			MotionStatus::m_CurrentJoints.SetEnable(id, true);

			if(DEBUG_PRINT == true)
				fprintf(stderr, "[%d] Success\n", value);
		}
		else
		{
			MotionStatus::m_CurrentJoints.SetEnable(id, false);

			if(DEBUG_PRINT == true)
				fprintf(stderr, " Fail\n");
		}
	}

	m_CalibrationStatus = 0;
	m_FBGyroCenter = 512;
	m_RLGyroCenter = 512;

	return true;
}

bool MotionManager::Reinitialize()
{
	m_ProcessEnable = false;

	m_CM730->DXLPowerOn();

	int value, error;
	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "ID:%d initializing...", id);

		if(m_CM730->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, &error) == CM730::SUCCESS)
		{
			MotionStatus::m_CurrentJoints.SetValue(id, value);
			MotionStatus::m_CurrentJoints.SetEnable(id, true);

			if(DEBUG_PRINT == true)
				fprintf(stderr, "[%d] Success\n", value);
		}
		else
		{
			MotionStatus::m_CurrentJoints.SetEnable(id, false);

			if(DEBUG_PRINT == true)
				fprintf(stderr, " Fail\n");
		}
	}

	m_ProcessEnable = true;
	return true;
}

/*
void * MotionManager::ServerListener(void *param)
{
	LinuxServer server ( "128.208.4.38", MPC_PORT );
	server.set_non_blocking(true);

	try
	{
		while(data_sock.valid() == false)
		{
			cout << "[Waiting..]" << endl;            
			server.accept(data_sock); // tcp_nodelay, but a blocking port
			//data_sock.set_non_blocking(true);
			cout << "[Accepted..]" << endl;
			m_IsStreaming = true;
		}
	}
	catch ( LinuxSocketException& e)
	{
		cout << "Exception was caught:" << e.description() << "\nExiting.\n";
	}

	pthread_exit(NULL);
}
*/


void MotionManager::StartStreaming()
{
	//pthread_attr_t attr;
	//pthread_attr_init(&attr);

	m_streamBuffer.clear();
	m_streamBuffer.reserve(SEND_BUF_SIZE);

	LinuxServer server ( "128.208.4.38", MPC_PORT );
	//server.set_non_blocking(true);

	try
	{
		while(data_sock.valid() == false)
		{
			cout << "[Waiting..]" << endl;            
			server.accept(data_sock); // tcp_nodelay, but a blocking port
			data_sock.set_non_blocking(true);
			cout << "[Accepted..]" << endl;
			m_IsStreaming = true;
		}
	}
	catch ( LinuxSocketException& e)
	{
		cout << "Exception was caught:" << e.description() << "\nExiting.\n";
	}

	server.close();

	//start a thread to async listen to connections

	//int error = 0;
	//if((error = pthread_create(&this->network_Thread, &attr, this->ServerListener, NULL))!= 0)
	//{
	//	printf("Couldn't Start Listener Socket...\n");
	//	m_IsStreaming = false;
	//}
}

void MotionManager::StopStreaming()
{
	//m_IsStreaming = false;
	//if (data_sock.valid()) {
	//	data_sock.close();
	//}
	//int error = 0;
	//if((error = pthread_join(this->network_Thread, NULL))!= 0)
	//{
	//	printf("Problem Closing Listening socket thread...\n");
	//}
	//
	cout << "Closing connection\n" << endl;

	m_IsStreaming = false;
	if (data_sock.valid()) {
		data_sock.close();
	}
}

bool MotionManager::IsStreaming()
{
	return m_IsStreaming;
}

void MotionManager::StartLogging()
{
	m_logBuffer.clear();
	m_logBuffer.reserve(10000);

	printf("Starting to Log\n");
	m_IsLogging = true;
}

bool MotionManager::IsLogging()
{
	return m_IsLogging;
}

void MotionManager::StopLogging()
{
	char szFile[32] = {0,};

	int count = 0;
	while(1)
	{
		sprintf(szFile, "Log%d.csv", count);
		if(0 != access(szFile, F_OK))
			break;
		count++;
		if(count > 256) return;
	}

	printf("Flushing to file... ");

	// Setup logfilestream
	m_LogFileStream.open(szFile, std::ios::out);
	m_LogFileStream << "% MS_TIME,";
	/*
		for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++) {
		m_LogFileStream << "ID_" << id << "_GP,;
		}
		for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++) {
		m_LogFileStream << "ID_" << id << "_GP,ID_" << id << "_PP,ID_" << id << "_P,ID_" << id << "_D,";
		}
		for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++) {
		m_LogFileStream << "ID_" << id << "_GP,ID_" << id << "_PP,ID_" << id << "_P,ID_" << id << "_D,";
		}
		for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++) {
		m_LogFileStream << "ID_" << id << "_GP,ID_" << id << "_PP,ID_" << id << "_P,ID_" << id << "_D,";
		}
		*/
	m_LogFileStream <<"goal positions, present positions, speed,";

	m_LogFileStream << "GyroZ,GyroY,GyroX,AccelZ,AccelY,AccelX,";
	m_LogFileStream << "L_FSR_1,L_FSR_2,L_FSR_3,L_FSR_4,";
	m_LogFileStream << "R_FSR_1,R_FSR_2,R_FSR_3,R_FSR_4," << std::endl;

	// write data from m_logBuffer to m_logFileStream; doing data conversion as
	// well. time, goal pose, pose, vel, gyro, accel, l fsr, r fsr
	for (std::vector<int>::iterator it=m_logBuffer.begin(); it!=m_logBuffer.end(); ++it) {
		m_LogFileStream << (*it)/1000.0 << ",";

		it++;

		// Goal Position
		for(int id = 1; id <= 20; id++) { // Joints are in Mujoco frame 
			m_LogFileStream << joint2radian(*it) << ",";
			it++;
		}
		// Positions
		for(int id = 1; id <= 20; id++) { // Joints are in Mujoco frame 
			m_LogFileStream << joint2radian(*it) << ",";
			it++;
		}
		// Speed
		for(int id = 1; id <= 20; id++) { // Joints are in Mujoco frame 
			m_LogFileStream << j_rpm2rads_ps(*it) << ",";
			it++;
		}

		m_LogFileStream<<-1*gyro2rads_ps(*it) << ","; it++; 
		m_LogFileStream<<-1*gyro2rads_ps(*it) << ","; it++;
		m_LogFileStream<<gyro2rads_ps(*it) << ","; it++;

		m_LogFileStream<<-1*accel2ms2(*it) << ","; it++;
		m_LogFileStream<<-1*accel2ms2(*it) << ","; it++;
		m_LogFileStream<<-1*accel2ms2(*it) << ","; it++;

		m_LogFileStream<<fsr2newton(*it) << ","; it++;
		m_LogFileStream<<fsr2newton(*it) << ","; it++;
		m_LogFileStream<<fsr2newton(*it) << ","; it++;
		m_LogFileStream<<fsr2newton(*it) << ","; it++;

		m_LogFileStream<<fsr2newton(*it) << ","; it++;
		m_LogFileStream<<fsr2newton(*it) << ","; it++;
		m_LogFileStream<<fsr2newton(*it) << ","; it++;
		m_LogFileStream<<fsr2newton(*it) << std::endl;
	}

	printf("done!\n");

	m_IsLogging = false;
	m_LogFileStream.close();
}

void MotionManager::LoadINISettings(minIni* ini)
{
	LoadINISettings(ini, OFFSET_SECTION);
}
void MotionManager::LoadINISettings(minIni* ini, const std::string &section)
{
	int ivalue = INVALID_VALUE;

	for(int i = 1; i < JointData::NUMBER_OF_JOINTS; i++)
	{
		char key[10];
		sprintf(key, "ID_%.2d", i);
		if((ivalue = ini->geti(section, key, INVALID_VALUE)) != INVALID_VALUE)  m_Offset[i] = ivalue;
	}
}
void MotionManager::SaveINISettings(minIni* ini)
{
	SaveINISettings(ini, OFFSET_SECTION);
}
void MotionManager::SaveINISettings(minIni* ini, const std::string &section)
{
	for(int i = 1; i < JointData::NUMBER_OF_JOINTS; i++)
	{
		char key[10];
		sprintf(key, "ID_%.2d", i);
		ini->put(section, key, m_Offset[i]);
	}
}

#define GYRO_WINDOW_SIZE    100
#define ACCEL_WINDOW_SIZE   30
#define MARGIN_OF_SD        2.0
void MotionManager::Process()
{
	if(m_ProcessEnable == false || m_IsRunning == true)
		return;

	m_IsRunning = true;

	// calibrate gyro sensor
	if(m_CalibrationStatus == 0 || m_CalibrationStatus == -1)
	{
		static int fb_gyro_array[GYRO_WINDOW_SIZE] = {512,};
		static int rl_gyro_array[GYRO_WINDOW_SIZE] = {512,};
		static int buf_idx = 0;

		if(buf_idx < GYRO_WINDOW_SIZE)
		{
			if(m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
			{
				fb_gyro_array[buf_idx] = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L);
				rl_gyro_array[buf_idx] = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L);
				buf_idx++;
			}
		}
		else
		{
			double fb_sum = 0.0, rl_sum = 0.0;
			double fb_sd = 0.0, rl_sd = 0.0;
			double fb_diff, rl_diff;
			double fb_mean = 0.0, rl_mean = 0.0;

			buf_idx = 0;

			for(int i = 0; i < GYRO_WINDOW_SIZE; i++)
			{
				fb_sum += fb_gyro_array[i];
				rl_sum += rl_gyro_array[i];
			}
			fb_mean = fb_sum / GYRO_WINDOW_SIZE;
			rl_mean = rl_sum / GYRO_WINDOW_SIZE;

			fb_sum = 0.0; rl_sum = 0.0;
			for(int i = 0; i < GYRO_WINDOW_SIZE; i++)
			{
				fb_diff = fb_gyro_array[i] - fb_mean;
				rl_diff = rl_gyro_array[i] - rl_mean;
				fb_sum += fb_diff * fb_diff;
				rl_sum += rl_diff * rl_diff;
			}
			fb_sd = sqrt(fb_sum / GYRO_WINDOW_SIZE);
			rl_sd = sqrt(rl_sum / GYRO_WINDOW_SIZE);

			if(fb_sd < MARGIN_OF_SD && rl_sd < MARGIN_OF_SD)
			{
				m_FBGyroCenter = (int)fb_mean;
				m_RLGyroCenter = (int)rl_mean;
				m_CalibrationStatus = 1;
				if(DEBUG_PRINT == true)
					fprintf(stderr, "FBGyroCenter:%d , RLGyroCenter:%d \n", m_FBGyroCenter, m_RLGyroCenter);
			}
			else
			{
				m_FBGyroCenter = 512;
				m_RLGyroCenter = 512;
				m_CalibrationStatus = -1;
			}
		}
	}

	if(m_CalibrationStatus == 1 && m_Enabled == true)
	{
		static int fb_array[ACCEL_WINDOW_SIZE] = {512,};
		static int buf_idx = 0;
		if(m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
		{
			MotionStatus::FB_GYRO = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L) - m_FBGyroCenter;
			MotionStatus::RL_GYRO = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L) - m_RLGyroCenter;
			MotionStatus::RL_ACCEL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L);
			MotionStatus::FB_ACCEL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L);
			fb_array[buf_idx] = MotionStatus::FB_ACCEL;
			if(++buf_idx >= ACCEL_WINDOW_SIZE) buf_idx = 0;
		}

		int sum = 0, avr = 512;
		for(int idx = 0; idx < ACCEL_WINDOW_SIZE; idx++)
			sum += fb_array[idx];
		avr = sum / ACCEL_WINDOW_SIZE;

		if(avr < MotionStatus::FALLEN_F_LIMIT)
			MotionStatus::FALLEN = FORWARD;
		else if(avr > MotionStatus::FALLEN_B_LIMIT)
			MotionStatus::FALLEN = BACKWARD;
		else
			MotionStatus::FALLEN = STANDUP;

		// For each motion module,
		// get Module's desired joint information and propagate to MotionStatus
		if(m_Modules.size() != 0)
		{
			for(std::list<MotionModule*>::iterator i = m_Modules.begin(); i != m_Modules.end(); i++)
			{
				(*i)->Process();
				for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
				{
					if((*i)->m_Joint.GetEnable(id) == true)
					{
						MotionStatus::m_CurrentJoints.SetSlope(id, (*i)->m_Joint.GetCWSlope(id), (*i)->m_Joint.GetCCWSlope(id));
						MotionStatus::m_CurrentJoints.SetValue(id, (*i)->m_Joint.GetValue(id));

						MotionStatus::m_CurrentJoints.SetPGain(id, (*i)->m_Joint.GetPGain(id));
						MotionStatus::m_CurrentJoints.SetIGain(id, (*i)->m_Joint.GetIGain(id));
						MotionStatus::m_CurrentJoints.SetDGain(id, (*i)->m_Joint.GetDGain(id));
					}
				}
			}
		}

		// Build packet of Joint Data from MotionStatus (filled by modules above)
		int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
		int n = 0;
		int joint_num = 0;
		for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
		{
			if(MotionStatus::m_CurrentJoints.GetEnable(id) == true)
			{
				param[n++] = id;
				param[n++] = MotionStatus::m_CurrentJoints.GetDGain(id);
				param[n++] = MotionStatus::m_CurrentJoints.GetIGain(id);
				param[n++] = MotionStatus::m_CurrentJoints.GetPGain(id);
				param[n++] = 0;
				param[n++] = CM730::GetLowByte(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]);
				param[n++] = CM730::GetHighByte(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]);
				joint_num++;
			}

			if(DEBUG_PRINT == true)
				fprintf(stderr, "ID[%d] : %d \n", id, MotionStatus::m_CurrentJoints.GetValue(id));
		}

		if(joint_num > 0) {
			m_CM730->SyncWrite(MX28::P_D_GAIN, MX28::PARAM_BYTES, joint_num, param);
		}
	}

	m_CM730->BulkRead();

	if(m_IsStreaming)
	{
		clock_gettime(CLOCK_MONOTONIC,&ms_time);

		m_streamBuffer.push_back(sec_diff(start_time, ms_time)); 

		for(int id = 1; id <= 17; id+=2) { // Right Joints
			m_streamBuffer.push_back(joint2radian(MotionStatus::m_CurrentJoints.GetValue(id)));
		}
		for(int id = 2; id <= 18; id+=2) { // Left Joints
			m_streamBuffer.push_back(joint2radian(MotionStatus::m_CurrentJoints.GetValue(id)));
		}
		for(int id = 19; id <= 20; id++) { // Head Joints
			m_streamBuffer.push_back(joint2radian(MotionStatus::m_CurrentJoints.GetValue(id)));
		}

		for(int id = 1; id <= 17; id+=2) { // Right Joints
			m_streamBuffer.push_back(joint2radian(m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L)));
		}
		for(int id = 2; id <= 18; id+=2) { // Left Joints
			m_streamBuffer.push_back(joint2radian(m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L)));
		}
		for(int id = 19; id <= 20; id++) { // Head Joints
			m_streamBuffer.push_back(joint2radian(m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L)));
		} 
		for(int id = 1; id <= 17; id+=2) { // Right Joints
			m_streamBuffer.push_back(j_rpm2rads_ps(m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_SPEED_L)));
		}
		for(int id = 2; id <= 18; id+=2) { // Left Joints
			m_streamBuffer.push_back(j_rpm2rads_ps(m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_SPEED_L)));
		}
		for(int id = 19; id <= 20; id++) { // Head Joints
			m_streamBuffer.push_back(j_rpm2rads_ps(m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_SPEED_L)));
		}

		m_streamBuffer.push_back(-1*gyro2rads_ps(m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L)));
		m_streamBuffer.push_back(-1*gyro2rads_ps(m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L)));
		m_streamBuffer.push_back(gyro2rads_ps(m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Z_L)));

		m_streamBuffer.push_back(accel2ms2(m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L)));
		m_streamBuffer.push_back(-1*accel2ms2(m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L)));
		m_streamBuffer.push_back(accel2ms2(m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Z_L)));

		m_streamBuffer.push_back(fsr2newton(m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR4_L)));
		m_streamBuffer.push_back(fsr2newton(m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR1_L)));
		m_streamBuffer.push_back(fsr2newton(m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR2_L)));
		m_streamBuffer.push_back(fsr2newton(m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR3_L)));

		m_streamBuffer.push_back(fsr2newton(m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR2_L)));
		m_streamBuffer.push_back(fsr2newton(m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR3_L)));
		m_streamBuffer.push_back(fsr2newton(m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR4_L)));
		m_streamBuffer.push_back(fsr2newton(m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR1_L)));

		if (data_sock.valid())
		{
			try {
				//double *send_buf = &(m_streamBuffer[0]);
				double *send_buf = m_streamBuffer.data();
				if ( !data_sock.send((unsigned char*)send_buf, SEND_BUF_SIZE*sizeof(double)))
				{
					throw LinuxSocketException ( "Could not write to socket." );
				}
			}
			catch ( LinuxSocketException& )
			{
				cout << "[Disconnected]" << endl;
				StopStreaming();
			}
		}
		m_streamBuffer.clear();
		m_streamBuffer.reserve(SEND_BUF_SIZE);
	}

	if(m_IsLogging)
	{
		clock_gettime(CLOCK_MONOTONIC,&ms_time);

		m_logBuffer.push_back((int)(sec_diff(start_time, ms_time)*1000)); 

		for(int id = 1; id <= 17; id+=2) { // Right Joints
			m_logBuffer.push_back(MotionStatus::m_CurrentJoints.GetValue(id));
		}
		for(int id = 2; id <= 18; id+=2) { // Left Joints
			m_logBuffer.push_back(MotionStatus::m_CurrentJoints.GetValue(id));
		}
		for(int id = 19; id <= 20; id++) { // Head Joints
			m_logBuffer.push_back(MotionStatus::m_CurrentJoints.GetValue(id));
		}

		for(int id = 1; id <= 17; id+=2) { // Right Joints
			m_logBuffer.push_back(m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L));
		}
		for(int id = 2; id <= 18; id+=2) { // Left Joints
			m_logBuffer.push_back(m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L));
		}
		for(int id = 19; id <= 20; id++) { // Head Joints
			m_logBuffer.push_back(m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L));
		} 

		for(int id = 1; id <= 17; id+=2) { // Right Joints
			m_logBuffer.push_back(m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_SPEED_L));
		}
		for(int id = 2; id <= 18; id+=2) { // Left Joints
			m_logBuffer.push_back(m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_SPEED_L));
		}
		for(int id = 19; id <= 20; id++) { // Head Joints
			m_logBuffer.push_back(m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_SPEED_L));
		}

		m_logBuffer.push_back(m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L));
		m_logBuffer.push_back(m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L));
		m_logBuffer.push_back(m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Z_L));

		m_logBuffer.push_back(m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L));
		m_logBuffer.push_back(m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L));
		m_logBuffer.push_back(m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Z_L));

		m_logBuffer.push_back(m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR1_L));
		m_logBuffer.push_back(m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR2_L));
		m_logBuffer.push_back(m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR3_L));
		m_logBuffer.push_back(m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR4_L));

		m_logBuffer.push_back(m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR1_L));
		m_logBuffer.push_back(m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR2_L));
		m_logBuffer.push_back(m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR3_L));
		m_logBuffer.push_back(m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR4_L));

		// Original
		//m_LogFileStream << m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L) << ",";
		//m_LogFileStream << m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L) << ",";
		//m_LogFileStream << m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L) << ",";
		//m_LogFileStream << m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L) << ",";
		//m_LogFileStream << m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadByte(FSR::P_FSR_X) << ",";
		//m_LogFileStream << m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadByte(FSR::P_FSR_Y) << ",";
		//m_LogFileStream << m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadByte(FSR::P_FSR_X) << ",";
		//m_LogFileStream << m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadByte(FSR::P_FSR_Y) << ",";
		//m_LogFileStream << std::endl;
	}

	if(m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
		MotionStatus::BUTTON = m_CM730->m_BulkReadData[CM730::ID_CM].ReadByte(CM730::P_BUTTON);

	m_IsRunning = false;
}

void MotionManager::SetEnable(bool enable)
{
	m_Enabled = enable;
	if(m_Enabled == true)
		m_CM730->WriteWord(CM730::ID_BROADCAST, MX28::P_MOVING_SPEED_L, 0, 0);
}

void MotionManager::AddModule(MotionModule *module)
{
	module->Initialize();
	m_Modules.push_back(module);
}

void MotionManager::RemoveModule(MotionModule *module)
{
	m_Modules.remove(module);
}

void MotionManager::SetJointDisable(int index)
{
	if(m_Modules.size() != 0)
	{
		for(std::list<MotionModule*>::iterator i = m_Modules.begin(); i != m_Modules.end(); i++)
			(*i)->m_Joint.SetEnable(index, false);
	}
}


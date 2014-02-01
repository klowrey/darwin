#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <libgen.h>

#include <termios.h>
#include <term.h>
#include <pthread.h>

#include "LinuxDARwIn.h"
#include "FSR.h"

using namespace Robot;

#define INI_FILE_PATH       "../../../../Data/config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"


bool running = false;

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

void* walk_thread(void* ptr)
{
	while(1) {
		int ch = _getch();
		if(ch == 0x20) {
			if(MotionManager::GetInstance()->IsStreaming() == true) {
				running = false;
				Walking::GetInstance()->Stop();
				MotionManager::GetInstance()->StopStreaming();
			}
			else {
				MotionManager::GetInstance()->StartStreaming();
				sleep(1);
				Walking::GetInstance()->Start();
				running = true;
			}
		}
	}

	return NULL;
}

int main()
{
	printf( "\n===== FSR Tutorial for DARwIn =====\n\n");

	change_current_dir();
	minIni* ini = new minIni(INI_FILE_PATH);

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730(U2D_DEV_NAME);
	CM730 cm730(&linux_cm730);
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
		return 0;
	}
	MotionManager::GetInstance()->LoadINISettings(ini);
	Walking::GetInstance()->LoadINISettings(ini);

	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
	LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
	motion_timer->Start();
	/////////////////////////////////////////////////////////////////////

	int n = 0;
	int param[JointData::NUMBER_OF_JOINTS * 5];
	int wGoalPosition, wStartPosition, wDistance;

	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
		wGoalPosition = Walking::GetInstance()->m_Joint.GetValue(id);
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
	printf("Press the SPACE key to start/stop walking.. \n\n");

	Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	MotionManager::GetInstance()->SetEnable(true);

	static const int MAX_FSR_VALUE = 254;

	Walking::GetInstance()->LoadINISettings(ini);

	pthread_t thread_t;
	pthread_create(&thread_t, NULL, walk_thread, NULL);

	double m_FBStep = 0;
	double m_RLTurn = 0;
	double d_step = 0.6;

	while(1)
	{
		if (running == true) {
			m_FBStep += d_step; 
			if (m_FBStep > 10.0) {
				d_step = -0.3;
			}
			if (m_FBStep < 0) {
				m_FBStep = 0.0;
				running = false;
				Walking::GetInstance()->Stop();
			}
			printf("FB Step : %f\n", m_FBStep);
			Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
		}

		usleep(250000);
	}

	return 0;
}


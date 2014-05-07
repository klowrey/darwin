#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <libgen.h>
#include <signal.h>

#include <termios.h>
#include <term.h>
#include <pthread.h>

#include "LinuxDARwIn.h"
#include "FSR.h"

using namespace Robot;

#define MOTION_FILE_PATH    "/home/darwin/darwin/Data/motion_4096.bin"
#define INI_FILE_PATH       "../../../../Data/config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"

#define a_STANDUP 1
#define a_SITDOWN 15
#define a_WALKRDY 9


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
				MotionManager::GetInstance()->StopStreaming();
			}
			else {
				MotionManager::GetInstance()->StartStreaming();
			}
		}
	}
	return NULL;
}

void sighandler(int sig)
{
	exit(0);
}

void do_action(int act) {

	Action::GetInstance()->Start(act);
	while(Action::GetInstance()->IsRunning() == 1) usleep(4000);

}

int main()
{
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGQUIT, &sighandler);
	signal(SIGINT, &sighandler);


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
	Walking::GetInstance()->LoadINISettings(ini);

	//MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());

	MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
	LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
	motion_timer->Start();
	/////////////////////////////////////////////////////////////////////

	MotionManager::GetInstance()->LoadINISettings(ini);

	printf("Press the ENTER key to begin!\n");
	getchar();
	printf("Press the SPACE key to start streaming data.. \n\n");

	//Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
	//Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	//Walking::GetInstance()->m_Joint.SetAngle(JointData::ID_HEAD_TILT, 0);
	//Walking::GetInstance()->m_Joint.SetAngle(JointData::ID_HEAD_PAN, 0);

	//for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	//{
	//	Walking::GetInstance()->m_Joint.SetPGain(id, 50);
	//}
	//MotionManager::GetInstance()->SetEnable(true);

	static const int MAX_FSR_VALUE = 254;

	//Walking::GetInstance()->LoadINISettings(ini);
	//pthread_t thread_t;
	//pthread_create(&thread_t, NULL, walk_thread, NULL);


	Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH);
	// move to initial position

	printf("Ready...\n");
	Action::GetInstance()->m_Joint.SetEnableBody(true, true);
	Action::GetInstance()->m_Joint.SetAngle(JointData::ID_HEAD_TILT, 0);
	Action::GetInstance()->m_Joint.SetAngle(JointData::ID_HEAD_PAN, 0);
	Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	MotionManager::GetInstance()->SetEnable(true);

	do_action(a_STANDUP);

	bool crouched = false;
	bool walking = false;
	bool running = true;

	while(running)
	{
		// detect and act on falls
		if(MotionStatus::FALLEN != STANDUP)
		{
			if (walking) {
				Walking::GetInstance()->Stop();
				while(Walking::GetInstance()->IsRunning() == 1) usleep(8000);
				walking = false;
			}

			Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

			if(MotionStatus::FALLEN == FORWARD)
				do_action(10);
			else if(MotionStatus::FALLEN == BACKWARD)
				do_action(11);

			crouched = false;

			//Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
			//Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
		}

		// if not streaming, wait for connection
		if (MotionManager::GetInstance()->IsStreaming() == false) {
			MotionManager::GetInstance()->StartStreaming();
		}

		// respond to user input
		int ch = _getch();

		switch (ch) {

			case 0x20:
				printf("Space pressed. Skipping...\n");
				break;

			case 'c':
				if (crouched) {
					printf("Crouching\n");
					do_action(a_STANDUP);
					crouched = false;
				}
				else {
					printf("Standing\n");
					do_action(a_SITDOWN);
					crouched = true;
				}
				break;

			case 'w':
				// walking stuff
				break;

			case 'a':
				// walking stuff
				break;

			case 's':
				// walking stuff
				break;

			case 'd':
				// walking stuff
				break;

			case 'Q':
				running = false;
				break;

			default:
				printf("Unsupported key command.\n");
				break;
		}
	}
	printf("Quitting.\n");

	return 0;
}



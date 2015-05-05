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

#include "Phasespace.h"

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


int main(int argc, char* argv[])
{
	int p_gain;
	int i_gain;
	int d_gain;
	bool use_ps;

	try {
		po::options_description desc("Allowed options");
		desc.add_options()
			("help", "Usage guide")
			("ps", po::value<bool>(&use_ps)->default_value(true), "Use phasespace. Remember to turn it on")
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

	printf("P Gain is set at %d\n", p_gain);
	printf("I Gain is set at %d\n", i_gain);
	printf("D Gain is set at %d\n", d_gain);

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

	if (use_ps) {
		MotionManager::GetInstance()->SetEnable(true);
		MotionManager::GetInstance()->SetCalibrationStatus(1);
		printf("Using Phasespace\n\tLoading Phasespace Module...\n");
		MotionManager::GetInstance()->AddModule((MotionModule*)Phasespace::GetInstance());
		printf(" Done.\n");
	}
	/////////////////////////////////////////////////////////////////////

	//int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];

	printf("Press the ENTER key to begin!\n");
	getchar();

	int max_speed = 1023;
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

	ready = false;
	int count = 0;
	while (!ready)
	{
		MotionManager::GetInstance()->Process();

		std::cout<<count++<<":: "<<
			MotionStatus::PS_DATA[0]<<", "<<
			MotionStatus::PS_DATA[1]<<", "<<
			MotionStatus::PS_DATA[2]<<", "<<
			MotionStatus::PS_DATA[3]<<", "<<
			MotionStatus::PS_DATA[4]<<", "<<
			MotionStatus::PS_DATA[5]<<", "<<
			MotionStatus::PS_DATA[6]<<std::endl;

		usleep(50000);
	}

	pthread_join(thread_t, NULL);

	// there is still data in the buffer
	if (use_ps) { 
		MotionManager::GetInstance()->RemoveModule((MotionModule*)Phasespace::GetInstance());
		delete Phasespace::GetInstance();
	}

	print_status(&cm730);

	return 0;
}



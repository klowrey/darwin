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

#include <boost/program_options.hpp>


#define U2D_DEV_NAME        "/dev/ttyUSB0"

using namespace Robot;
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

double* open_read_binary(char* filename, int LINE_SIZE) {
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

   int samples = fileLen/sizeof(double)/LINE_SIZE;

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
      cm730->ReadByte(joint, MX28::P_P_GAIN, &out, 0);
      printf("P: %d ", out);
      cm730->ReadByte(joint, MX28::P_D_GAIN, &out, 0);
      printf("D: %d ", out);
      cm730->ReadWord(joint, MX28::P_MOVING_SPEED_L, &out, 0);
      printf("Speed: %3.1f %% max speed\n", 100.0*out/1024.0);
   }
}

int main(int argc, char* argv[])
{
   bool engage;
   int p_gain;
   int d_gain;
   double dt;
   std::string *filename = new std::string();
   std::string *= new std::string();

   try {
      po::options_description desc("Allowed options");
      desc.add_options()
         ("help", "Usage guide")
         ("engage,e", po::value<bool>(&engage)->default_value(false),
          "Engage motors for live run")
         ("trajectory,q", po::value<std::string>(filename)->required(), "Binary file of trajectory joint data")
         ("velocity,v", po::value<std::string>(vel_file), "Binary file of trajectory joint data")
         ("dt,t", po::value<double>(&dt)->default_value(0.02),
          "Timestep in binary file -- checks for file corruption")
         ("p_gain,p", po::value<int>(&p_gain)->default_value(20), "P gain of PD controller, 2-160")
         ("d_gain,d", po::value<int>(&d_gain)->default_value(0), "D gain of PD controller, 0-30")
         ;

      po::variables_map vm;
      po::store(po::parse_command_line(argc, argv, desc), vm);

      if (vm.count("help")) {
         printf("Usage: ./follower [0-1 engage] [filename] [timestep] [p_gain]\n");
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
   printf("Reading velocities from%s\n", vel_file->c_str());
   printf("Timestep is set at %f\n", dt);
   printf("P Gain is set at %d\n", p_gain);
   printf("D Gain is set at %d\n", d_gain);

   int STRIDE = 0;
   // TODO line_size might include max velocities --> 41
   int LINE_SIZE = 21;

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

   int n = 0;
   int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];

   printf("Press the ENTER key to begin!\n");
   getchar();


   // file stuff start
   double* binary_line = open_read_binary(filename->c_str());

   if (binary_line == NULL) {
      printf("Bad trajectory\n");
      return 0;
   }

   double* vel_line = NULL;
   if (vel_file.empty() == false) {
      double* vel_line = open_read_binary(vel_file->c_str());

      if (vel_line == NULL) {
         printf("Bad velocity file\n");
         return 0;
      }
   }

   double* joint_data;
   double* prev_joint;
   double* interp = (double*) malloc(LINE_SIZE*sizeof(double));

   fread((void*)binary_line, fileLen, 1, trajectory_file);
   fclose(trajectory_file);
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

   int max_speed = 512;
   for (int joint=JointData::ID_R_SHOULDER_PITCH; joint<JointData::NUMBER_OF_JOINTS; joint++) {
      cm730.WriteByte(joint, MX28::P_P_GAIN, p_gain, 0);
      cm730.WriteByte(joint, MX28::P_D_GAIN, d_gain, 0);
      cm730.WriteWord(joint, MX28::P_MOVING_SPEED_L, max_speed, 0);

      MotionStatus::m_CurrentJoints.SetPGain(joint, p_gain);
      MotionStatus::m_CurrentJoints.SetIGain(joint, 0);
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

   clock_gettime(CLOCK_MONOTONIC, &start_time);
   clock_gettime(CLOCK_MONOTONIC, &end_time);

   // there is still data in the buffer
   for (int sample = 1; sample<samples; sample++) {
      printf("%3.2f percent through the file.\n", (double)sample/samples);

      // might not need to bother checking this...
      if ((timestamp - binary_line[sample*LINE_SIZE]) < 10e-6) {


         joint_data = &(binary_line[STRIDE]);

         // can print out 20 joint datas
         /*
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
         */

         printf("%f ??? %f\n", time_passed, timestamp);
         while ( time_passed < timestamp) {
            //printf("%f is < %f\n", time_passed, timestamp);
            double radian = 0;
            int joint_num = 0;
            int value;
            int id;
            int n = 0;

            //double percent = 1.0 - ((joint_data[0]-prev_joint[0]) / (time_passed-prev_joint[0]));
            double percent = ((time_passed-prev_joint[0]) / (joint_data[0]-prev_joint[0]));
            printf("percent: %f, %f %f\n", percent, joint_data[0], prev_joint[0]);

            clock_gettime(CLOCK_MONOTONIC, &end_time);
            time_passed = sec_diff(start_time, end_time);
            //printf("s: %f e: %f\n", timespec2sec(start_time), timespec2sec(end_time));

            // INTERPOLATION STEP
            for (int joint=1; joint<20; joint++) {
               double diff = joint_data[joint] - prev_joint[joint];
               interp[joint] = prev_joint[joint] + percent*diff;
               //printf("%1.3f -- %1.3f -- %1.3f\n", prev_joint[joint], interp[joint], joint_data[joint]);
            }

            // prepare commands to joints
            for (int joint=1; joint<=JointData::ID_R_HIP_ROLL; joint++) {
               joint_num++;
               MotionStatus::m_CurrentJoints.SetValue(joint_num, radian2joint(interp[joint]));
               MotionStatus::m_CurrentJoints.SetEnable(joint_num, true);

               joint_num++;
               MotionStatus::m_CurrentJoints.SetValue(joint_num, radian2joint(interp[joint+9]));
               MotionStatus::m_CurrentJoints.SetEnable(joint_num, true);
            }

            id = JointData::ID_HEAD_PAN;
            MotionStatus::m_CurrentJoints.SetValue(id, radian2joint(interp[id]));
            MotionStatus::m_CurrentJoints.SetEnable(id, true);
            joint_num++;

            id = JointData::ID_HEAD_TILT;
            MotionStatus::m_CurrentJoints.SetValue(id, radian2joint(interp[id]));
            MotionStatus::m_CurrentJoints.SetEnable(id, true);
            joint_num++;

            //if(joint_num > 0 && engage) {
            //cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 3, joint_num, param);
            //}

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

   if (MotionManager::GetInstance()->IsLogging() == true) {
      printf("Logging to file... ");
      MotionManager::GetInstance()->StopLogging();
      printf("Done.\n");
   }

   free(binary_line);
   free(interp);

   print_status(&cm730);
   delete filename;

   return 0;
}


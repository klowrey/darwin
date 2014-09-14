#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "LinuxDARwIn.h"

#include "owl.h"

#define MARKER_COUNT 8
#define PS_SERVER_NAME "128.208.4.127"
#define INIT_FLAGS 0
#define PHASESPACE_CONFIDENCE 1


using namespace Robot;

double diff_sec(timespec start, timespec end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return (double)(temp.tv_sec + temp.tv_nsec / 1000000000.0);
}

// gyro & accel
double gyro_radps(int gyro) {
	return (gyro-512)*0.017453229251;
}

double accel_ms2(int accel) {
	return ((accel-512) / 128.0) * 9.81; // in m/s^2
}

void owl_print_error(const char *s, int n)
{
	if(n < 0) printf("%s: %d\n", s, n);
	else if(n == OWL_NO_ERROR) printf("%s: No Error\n", s);
	else if(n == OWL_INVALID_VALUE) printf("%s: Invalid Value\n", s);
	else if(n == OWL_INVALID_ENUM) printf("%s: Invalid Enum\n", s);
	else if(n == OWL_INVALID_OPERATION) printf("%s: Invalid Operation\n", s);
	else printf("%s: 0x%x\n", s, n);
}

void copy_p(const float *a, float *b) { for(int i = 0; i < 7; i++) b[i] = a[i]; }
void print_p(const float *p) { for(int i = 0; i < 7; i++) printf("%f ", p[i]); }

void sighandler(int sig)
{
	owlDone();
	printf("\n\nExiting.\n");
	exit(0);
}


int main()
{
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGQUIT, &sighandler);
	signal(SIGINT, &sighandler);

	printf( "\n===== HI EMO, I am DARWIN =====\n\n");

	//////////////////// Darwin Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);
	if(cm730.Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return 0;
	}

	// Disable all motors for safety
	for (int id=JointData::ID_R_SHOULDER_PITCH; id<=JointData::ID_HEAD_TILT; id++) {
		cm730.WriteWord(id, MX28::P_TORQUE_ENABLE, 0, 0);
	}	
	///////////////////////////////////////////////////////////////////

	// Init Time
	static struct timespec start_time;
	static struct timespec interval;
	static struct timespec prev_int;


	// Init Gyro & Accel
	unsigned char table[CM730::MAXNUM_ADDRESS];

	double gyro_z, gyro_y, gyro_x, accel_z, accel_y, accel_x;
	double time;

   // Init Phasespace Marker Tracking
   OWLRigid rigid;
   OWLMarker markers[32];
   OWLCamera cameras[32];
   int tracker;
   if (owlInit(PS_SERVER_NAME, INIT_FLAGS) < 0) {
      printf("Couldn't connect to Phase Space\n");
      return 0;
   }
   // create tracker 0
   tracker = 0;
   owlTrackeri(tracker, OWL_CREATE, OWL_POINT_TRACKER);
   for (int i = 0; i < MARKER_COUNT; i++) {
      owlMarkeri(MARKER(tracker, i), OWL_SET_LED, i);
   }
   owlTracker(tracker, OWL_ENABLE);
   if (!owlGetStatus()) {
      owl_print_error("error in point tracker setup", owlGetError());
      return 0;
   }
   owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY);
   owlSetInteger(OWL_STREAMING, OWL_ENABLE);


   //////////////////// Begin Collection /////////////////////////////

   clock_gettime(CLOCK_MONOTONIC, &start_time);
   clock_gettime(CLOCK_MONOTONIC, &prev_int);

   while (1) {
      // reads a chunk of data into table to be formatted after
      if(cm730.ReadTable(CM730::ID_CM, CM730::P_GYRO_Z_L, CM730::P_ACCEL_Z_H, table, 0) == CM730::SUCCESS)
      {
         // Darwin Time
         clock_gettime(CLOCK_MONOTONIC, &interval);
         time = diff_sec(start_time, interval);


         // Gyro and Accel
         gyro_z = gyro_radps(cm730.MakeWord(table[CM730::P_GYRO_Z_L], table[CM730::P_GYRO_Z_H]));
         gyro_y = gyro_radps(cm730.MakeWord(table[CM730::P_GYRO_Y_L], table[CM730::P_GYRO_Y_H]));
         gyro_x = gyro_radps(cm730.MakeWord(table[CM730::P_GYRO_X_L], table[CM730::P_GYRO_X_H]));

         accel_z = accel_ms2(cm730.MakeWord(table[CM730::P_ACCEL_Z_L], table[CM730::P_ACCEL_Z_H]));
         accel_y = accel_ms2(cm730.MakeWord(table[CM730::P_ACCEL_Y_L], table[CM730::P_ACCEL_Y_H]));
         accel_x = accel_ms2(cm730.MakeWord(table[CM730::P_ACCEL_X_L], table[CM730::P_ACCEL_X_H]));

         if (diff_sec(prev_int, interval) > 1.0) {
            printf("\r");
            printf("TIME: %1.3f\tACCEL: %1.3f %1.3f %1.3f\tGRYO: %1.3f %1.3f %1.3f",
                  time, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);
            clock_gettime(CLOCK_MONOTONIC, &prev_int);
         }

         // Phasespace
         // Rigid body -- we have to set the rigid body from this end?
         /*
         int err;

         // get the rigid body markers
         //  note: markers have to be read,
         //  even if they are not used
         int m = owlGetMarkers(markers, 32);
         int o = owlGetCameras(cameras, 32);
         int n = owlGetRigids(&rigid, 1);

         // check for error
         if ((err = owlGetError()) != OWL_NO_ERROR) {
            owl_print_error("error", err);
            break;
         }

         // no data yet
         if (n == 0 || o == 0) continue;

         if (n > 0) {
            printf("%d rigid body, %d markers, %d cameras:\n", n, m, o);
            if (rigid.cond > 0) {
               float inv_pose[7];
               copy_p(rigid.pose, inv_pose);
               invert_p(inv_pose);

               printf("\nRigid: ");
               print_p(rigid.pose);

               printf("\nCameras: ");
               for (int i = 0; i < o; i++) {
                  // multiply each camera's pose by inverse of rigid pose
                  float cam_pose[7];
                  copy_p(cameras[i].pose, cam_pose);
                  mult_pp(inv_pose, cam_pose, cameras[i].pose);

                  print_p(cameras[i].pose);
                  printf("\n");
               }
               printf("\n");
            }
            printf("\n");
         }
         */

         // Markers
         int n = owlGetMarkers(markers, 32);
         int err;
         if ((err = owlGetError()) != OWL_NO_ERROR) {
            owl_print_error("error", err);
            break;
         }

         if (n > 0) {
            for(int i = 0; i < MARKER_COUNT; i++) {
               if(markers[i].cond > PHASESPACE_CONFIDENCE) {
                  double x = markers[i].x/1000.0;
                  double y = markers[i].y/1000.0;
                  double z = markers[i].z/1000.0;
               }
            }
            printf("\t %d good markers", n);
         }
      }
   }

   return 0;
}

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


float RIGID_BODY[MARKER_COUNT][3] = {
	{0.00, 0.00, 0.00},
	{69.24, 3.39, -74.19}, 
	{43.14, 19.77, 21.19},
	{78.63, -101.10, 19.91}, 
	{16.91, -7.61, -69.78},
	{94.86, -37.21, 32.17},
	{21.44, 2.66, -31.64},
	{115.67, -49.50, -45.21} 
};


// data buffer
#define NDATA 15000
float pose[NDATA][7];
float cond[NDATA];
double tm[NDATA];
int frame[NDATA];
double accel[NDATA][3];
double gyro[NDATA][3];


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
	owlTrackeri(tracker, OWL_CREATE, OWL_RIGID_TRACKER);
	for (int i = 0; i < MARKER_COUNT; i++) {
		owlMarkeri(MARKER(tracker, i), OWL_SET_LED, i);
		owlMarkerfv(MARKER(tracker, i), OWL_SET_POSITION, RIGID_BODY[i]);
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

	int cnt = 0;
	int nrigid, nmarker, err;

	FILE* fp = fopen("data.txt", "wt");

	while (cnt<NDATA) {
		// reads a chunk of data into table to be formatted after
		if(cm730.ReadTable(CM730::ID_CM, CM730::P_GYRO_Z_L, CM730::P_ACCEL_Z_H, table, 0) == CM730::SUCCESS)
		{
			// get the rigid body and markers
			nmarker = owlGetMarkers(markers, 32);
			nrigid = owlGetRigids(&rigid, 1);

			// check for error
			if ((err = owlGetError()) != OWL_NO_ERROR) {
				owl_print_error("error", err);
				return 0;
			}

			// make sure we got a new frame
			if( nrigid<1 )
				continue;

			// save phasespace data
			for( int j=0; j<7; j++ )
				pose[cnt][j] = rigid.pose[j];
			cond[cnt] = rigid.cond;
			frame[cnt] = rigid.frame;

			// Darwin Time
			clock_gettime(CLOCK_MONOTONIC, &interval);
			time = diff_sec(start_time, interval);

			// Gyro and Accel
			gyro[cnt][2] = gyro_radps(cm730.MakeWord(table[CM730::P_GYRO_Z_L], table[CM730::P_GYRO_Z_H]));
			gyro[cnt][1] = gyro_radps(cm730.MakeWord(table[CM730::P_GYRO_Y_L], table[CM730::P_GYRO_Y_H]));
			gyro[cnt][0] = gyro_radps(cm730.MakeWord(table[CM730::P_GYRO_X_L], table[CM730::P_GYRO_X_H]));

			accel[cnt][2] = accel_ms2(cm730.MakeWord(table[CM730::P_ACCEL_Z_L], table[CM730::P_ACCEL_Z_H]));
			accel[cnt][1] = accel_ms2(cm730.MakeWord(table[CM730::P_ACCEL_Y_L], table[CM730::P_ACCEL_Y_H]));
			accel[cnt][0] = accel_ms2(cm730.MakeWord(table[CM730::P_ACCEL_X_L], table[CM730::P_ACCEL_X_H]));

			// advance counter, save time
			tm[cnt] = time;
			cnt++;

			// sleep: Darwin returns repeated data at higher rates
			// usleep(4000);
		}
	}

	// save data to file
	FILE* fp = fopen("data.txt", "wt");
	for( cnt=0; cnt<NDATA; cnt++ )
		fprintf(fp, "%d %f %d %f   %f %f %f   %f %f %f %f   %f %f %f  %f %f %f\n",
				cnt, 1000.0*tm[cnt], frame[cnt], cond[cnt], 
				pose[cnt][0], pose[cnt][1], pose[cnt][2],
				pose[cnt][3], pose[cnt][4], pose[cnt][5], pose[cnt][6],
				accel[cnt][0], accel[cnt][1], accel[cnt][2],
				gyro[cnt][0], gyro[cnt][1], gyro[cnt][2]); 
	fclose(fp);
	return 0;
}

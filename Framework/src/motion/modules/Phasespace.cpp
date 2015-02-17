/*
 *   Phasespace.cpp
 */

#include <stdio.h>
#include "MX28.h"
//#include "Kinematics.h"
#include "MotionStatus.h"
#include "Phasespace.h"

using namespace Robot;

#include "phasespace/include/owl.h"

#define MARKER_COUNT 8
#define PS_SERVER_NAME "128.208.4.127"
#define INIT_FLAGS 0

float RIGID_BODY[MARKER_COUNT][3] = {
	{00.00, 0.00, 0.00}, 
	{48.22, 11.49, 51.58}, 
	{97.48, -44.94, 21.19},
	{-7.28, -32.28, 68.49}, 
	{28.26, 6.30, -64.73}, 
	{-27.31, -37.49, -51.00}, 
	{84.31, -41.45, -51.53}, 
	{86.56, -86.03, -12.11} 
};


Phasespace* Phasespace::m_UniqueInstance = new Phasespace();

Phasespace::Phasespace()
{
	m_Joint.SetEnableBody(false); // disable motor control
	int error;
	/*
	   struct sched_param param;
	   pthread_attr_t attr;
	   pthread_attr_init(&attr);

	   error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
	   if(error != 0)
	   printf("error = %d\n",error);
	   error = pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
	   if(error != 0)
	   printf("error = %d\n",error);

	   memset(&param, 0, sizeof(param));
	   param.sched_priority = 31;// RT
	   error = pthread_attr_setschedparam(&attr, &param);
	   if(error != 0)
	   printf("error = %d\n",error);
	   */

	// create and start the thread
	//if((error = pthread_create(&this->m_Thread, &attr, this->PhasespaceProc, this))!= 0)

	// we Could run the phasespace thread at a higher priority if we wanted...

	if((error = pthread_create(&this->m_Thread, NULL, this->PhasespaceProc, this))!= 0)
		exit(-1);

	std::memset(pose, 0, sizeof(float)*POSE_SIZE);
	this->m_Initialized=false;
	this->m_TrackerRunning=true;
	MotionStatus::PHASESPACE_ON = true;
}

Phasespace::~Phasespace()
{
	if(this->m_TrackerRunning)
	{
		this->m_FinishTracking = true;
		// wait for the thread to end
		if((error = pthread_join(this->m_Thread, NULL))!= 0)
			exit(-1);
		this->m_Initialized=false;
		this->m_FinishTracking = false;
		this->m_TrackerRunning = false;
		MotionStatus::PHASESPACE_ON = false;
	}
}

void Phasespace::owl_print_error(const char *s, int n)
{
	if(n < 0) printf("%s: %d\n", s, n);
	else if(n == OWL_NO_ERROR) printf("%s: No Error\n", s);
	else if(n == OWL_INVALID_VALUE) printf("%s: Invalid Value\n", s);
	else if(n == OWL_INVALID_ENUM) printf("%s: Invalid Enum\n", s);
	else if(n == OWL_INVALID_OPERATION) printf("%s: Invalid Operation\n", s);
	else printf("%s: 0x%x\n", s, n);
}

void* Phasespace::PhasespaceProc(void* param)
{
	// Init Phasespace Marker Tracking
	OWLRigid rigid;
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

	std::ofstream out("TEST.txt");
	Phasespace *tracker = (Phasespace *)param;

	while (!tracker>m_FinishTracking) {
		int nrigid = owlGetRigids(&rigid, 1);

		// check for error
		int err;
		if ((err = owlGetError()) != OWL_NO_ERROR) {
			owl_print_error("error", err);
			return 0;
		}

		// make sure we got a new frame
		if( nrigid<1 )
			continue;

		//out<<rigid.pose[0]<<","<<rigid.pose[1]<<","<<rigid.pose[2]<<","
		//	<<rigid.pose[3]<<","<<rigid.pose[4]<<","<<rigid.pose[5]<<","
		//	<<rigid.pose[6]<<std::endl;
		//	there's potentially stuff we can do with the cond and frame
		//cond[cnt] = rigid.cond;
		//frame[cnt] = rigid.frame;

		//mutex
		tracker->mutex.lock();
		std::memcpy(pose, rigid.pose, sizeof(float)*POSE_SIZE);
		tracker->mutex.unlock();

		usleep(500);
	}

	owlDone();

	return 0;
}

void Phasespace::Initialize()
{
	if (this->m_TrackerRunning)
		this->m_Initialized = true;
}

void Phasespace::Process()
{
	// Copy most recent? Copy the average?
	if(this->m_TrackerRunning && this->m_Initialized)
	{
		this->mutex.lock();
		//std::memcpy(MotionStatus::PS_DATA, pose, sizeof(float) * POSE_SIZE);
		for (int i=0; i<POSE_SIZE; i++) {
			MotionStatus::PS_DATA[i] = (double)pose[i];
		}
		this->mutex.unlock();
	}
}

bool Phasespace::IsRunning(void)
{
	return this->m_TrackerRunning;
}

/*
 *   Phasespace.h
 */

#ifndef _PHASESPACE_H_
#define _PHASESPACE_H_

#include <string.h>

#include <mutex>
#include "MotionModule.h"

#define POSE_SIZE 7
#define COND_SIZE 1

namespace Robot
{
	class Phasespace : public MotionModule
	{
		private:
			static Phasespace* m_UniqueInstance;
			std::mutex mutex;
      		pthread_t m_Thread;
			bool m_TrackerRunning;
			bool m_FinishTracking;
			bool m_Initialized;

			Phasespace();
			void owl_print_error(const char *s, int n);
			float pose[POSE_SIZE+COND_SIZE];

		protected:
			static void *PhasespaceProc(void *param);

		public:
			static Phasespace* GetInstance() { return m_UniqueInstance; }

			~Phasespace();

			void Initialize();
			void Process();
			bool IsRunning(void);
	};
}

#endif


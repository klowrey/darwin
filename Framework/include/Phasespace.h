/*
 *   Phasespace.h
 */

#ifndef _PHASESPACE_H_
#define _PHASESPACE_H_

#include <string.h>

#include "MotionModule.h"

#define POSE_SIZE 7

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
			void owl_print_error();
			float pose[POSE_SIZE];

		protected:
			static void *PhasespaceProc(void *param);

		public:
			static Phasespace* GetInstance() { return m_UniqueInstance; }

			~Phasespace();

			void Initialize();
			void Process();
			bool IsRunnting();
	};
}

#endif

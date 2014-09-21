/*
*/
#include <iostream>
#include "Eigen/Dense"

template <class T, int dim>
class Kalman {
	public:
		Kalman<T, dim>(void);
		~Kalman<T, dim>(void);
		Eigen::Matrix<T, dim, 1> Filter(Eigen::Matrix<T, dim, 1> measurement);
	private:
		void PrintState(void);
		void UpdatePrediction(void);
		void UpdateEstimate(void);
		Eigen::Matrix<T, dim, 1> measurement;           // nq x 1
		Eigen::Matrix<T, dim, 1> lastStateEstimate;     // nq x 1
		Eigen::Matrix<T, dim, 1> priorStateEstimate;    // nq x 1
		Eigen::Matrix<T, dim, 1> stateEstimate;         // nq x 1
		Eigen::Matrix<T, dim, dim> lastErrorCovariance;   // nq x 1
		Eigen::Matrix<T, dim, dim> priorErrorCovariance;  // nq x 1
		Eigen::Matrix<T, dim, dim> errorCovariance;       // nq x 1
		Eigen::Matrix<T, dim, dim> measurementNoise;    // nq x nq
		Eigen::Matrix<T, dim, dim> kalmanGain;          // nq x nq
		Eigen::Matrix<T, dim, dim> I;                   // nq x nq
		int iteration;

};

#include "Kalman.cc"


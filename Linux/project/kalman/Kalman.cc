/*
*/

#include <iostream>

template <class T, int dim>
Kalman<T, dim>::Kalman(void) {
	iteration = 0;
	stateEstimate.setZero();
	errorCovariance.setIdentity();
	I.setIdentity();
	measurementNoise.setIdentity();
	measurementNoise *= 0.1;  // must be 0 < noise < 1
}

template <class T, int dim>
Kalman<T, dim>::~Kalman(void) {
}

template <class T, int dim>
Eigen::Matrix<T, dim, 1> Kalman<T, dim>::Filter(Eigen::Matrix<T, dim, 1> measurement) {
	iteration++;
	this->measurement = measurement;
	lastStateEstimate = stateEstimate;
	lastErrorCovariance = errorCovariance;

	UpdatePrediction();
	UpdateEstimate();

	return stateEstimate;
}

template<class T, int dim>
void Kalman<T, dim>::UpdatePrediction(void) {
	priorStateEstimate = lastStateEstimate;
	priorErrorCovariance = lastErrorCovariance;
}

template<class T, int dim>
void Kalman<T, dim>::UpdateEstimate(void) {
	kalmanGain = priorErrorCovariance *
		(priorErrorCovariance + measurementNoise).inverse();
	stateEstimate = priorStateEstimate +
		kalmanGain * (measurement - priorStateEstimate);
	errorCovariance = (I - kalmanGain) * priorErrorCovariance;
}

template<class T, int dim>
void Kalman<T, dim>::PrintState(void) {
	std::cout << "measurement:" << std::endl;
	std::cout <<  measurement   << std::endl << std::endl;
	std::cout << "lastStateEstimate:" << std::endl;
	std::cout <<  lastStateEstimate   << std::endl << std::endl;
	std::cout << "priorStateEstimate:" << std::endl;
	std::cout <<  priorStateEstimate   << std::endl << std::endl;
	std::cout << "stateEstimate:" << std::endl;
	std::cout <<  stateEstimate   << std::endl << std::endl;
	std::cout << "lastErrorCovariance:" << std::endl;
	std::cout <<  lastErrorCovariance   << std::endl << std::endl;
	std::cout << "priorErrorCovariance:" << std::endl;
	std::cout <<  priorErrorCovariance   << std::endl << std::endl;
	std::cout << "errorCovariance:" << std::endl;
	std::cout <<  errorCovariance   << std::endl << std::endl;
	std::cout << "measurementNoise:" << std::endl;
	std::cout <<  measurementNoise   << std::endl << std::endl;
	std::cout << "kalmanGain:" << std::endl;
	std::cout <<  kalmanGain   << std::endl << std::endl;
	std::cout << "I:" << std::endl;
	std::cout <<  I   << std::endl << std::endl;
	std::cout << "iteration:" << std::endl;
	std::cout <<  iteration   << std::endl << std::endl;
}


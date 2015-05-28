#ifndef STATE_H
#define STATE_H

#include <Eigen/Dense>

class State
{
public:
	Eigen::Vector3d    position;
	Eigen::Quaterniond orientation;
	Eigen::Vector3d    linearMomentum;
	Eigen::Vector3d    angularMomentum;
};

class Derivative
{
public:
	Eigen::Vector3d    velocity;
	Eigen::Quaterniond orientationDeriv;
	Eigen::Vector3d    accumulatedForce;
	Eigen::Vector3d    accumulatedTorque;
};

#endif
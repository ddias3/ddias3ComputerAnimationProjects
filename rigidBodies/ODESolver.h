#ifndef ODE_SOLVER_H
#define ODE_SOLVER_H

#define USE_RK4 (2)

#include <Eigen/Dense>

#include "State.h"

class ODESolver
{
private:
	int integratorType;
	void (ODESolver::*integratorFunctionPointer)(int index);

	double deltaTime;
	double time;

	State *states;
	Derivative *derivatives;
	int numberStates;

	State RK4Evaluate(const State &initial, double time, double deltaTime, const Derivative &derivative, const Eigen::Vector3d &acceleration);

public:
	ODESolver(int type);
	~ODESolver();

	void SetTime(double time, double deltaTime)
	{
		this->time = time;
		this->deltaTime = deltaTime;
	}

	void ExplicitEulerIntegrator(int index);
	void RK4Integrator(int index);

	void SetDimension(int numberStates);
	void SetState(int index, Eigen::Vector3d &position, Eigen::Quaterniond &orientation,
	              Eigen::Vector3d &linearMomentum, Eigen::Vector3d &angularMomentum);
	void SetDerivative(int index, Eigen::Vector3d &velocity, Eigen::Quaterniond &orientationDeriv,
	                   Eigen::Vector3d &accumulatedForce, Eigen::Vector3d &accumulatedTorque);
	void SolveODE();
	State& GetState(int index);

	void PrintAllStates();
};

#endif
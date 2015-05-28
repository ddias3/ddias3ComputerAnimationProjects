#ifndef ODE_SOLVER_H
#define ODE_SOLVER_H

#define USE_RK4 (2)

#include <Eigen/Dense>

#include "ParticleSystem.h"
#include "State.h"

class ODESolver
{
private:
	int integratorType;
	ParticleSystem* particleSystem;
	void (ODESolver::*integratorFunctionPointer)(int index);

	double deltaTime;
	double time;

	State *states;
	State *derivedStates;
	int numberStates;

	State RK4Evaluate(const State &initial, double time, double deltaTime, const State &derivative, const Eigen::Vector3d &acceleration);

public:
	ODESolver(int type);
	~ODESolver();

	void SetTime(double time, double deltaTime)
	{
		this->time = time;
		this->deltaTime = deltaTime;
	}

	void PassParticleSystemPointer(ParticleSystem *particleSystem)
	{
		this->particleSystem = particleSystem;
	}

	void ExplicitEulerIntegrator(int index);
	void RK4Integrator(int index);

	void SetDimension(int numberParticles);
	void SetState(int index, Eigen::Vector3d &first, Eigen::Vector3d &second);
	void SetDerivedState(int index, Eigen::Vector3d &first, Eigen::Vector3d &second);
	void SolveODE();
	State& GetState(int index);

	void PrintAllStates();
};

#endif
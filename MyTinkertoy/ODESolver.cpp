#include "ODESolver.h"

#include <Eigen/Dense>
#include <iostream>

ODESolver::ODESolver(int type)
{
	deltaTime = 1.0 / 60.0;

	numberStates = 0;
	states = NULL; //((void *)0)
	derivedStates = NULL; //((void *)0)

	switch (type)
	{
	case 0:
	case 1:
		integratorFunctionPointer = &ODESolver::ExplicitEulerIntegrator;
		integratorType = 0;
		break;
	case 2:
	default:
		integratorFunctionPointer = &ODESolver::RK4Integrator;
		integratorType = 2;
		break;
	}
}

ODESolver::~ODESolver()
{
	if (states != NULL)
		delete[] states;
	if (derivedStates != NULL)
		delete[] derivedStates;
}

void ODESolver::SetDimension(int numberParticles)
{
	if (numberStates != numberParticles)
	{
		numberStates = numberParticles;

		if (states != NULL)
			delete[] states;
		if (derivedStates != NULL)
			delete[] derivedStates;

		states = new State[numberStates];
		derivedStates = new State[numberStates];
	}
}

void ODESolver::SetState(int index, Eigen::Vector3d &first, Eigen::Vector3d &second)
{
	if (index < 0 || index >= numberStates)
	{
		std::cerr << "Out of bounds error: index (" << index <<
			") not in integer bound [0, " << numberStates <<
			") in function ODESolver::SetState";
		return;
	}
	states[index].first = first;
	states[index].second = second;
}

void ODESolver::SetDerivedState(int index, Eigen::Vector3d &first, Eigen::Vector3d &second)
{
	if (index < 0 || index >= numberStates)
	{
		std::cerr << "Out of bounds error: index (" << index <<
			") not in integer bound [0, " << numberStates <<
			") in function ODESolver::SetDerivedState";
		return;
	}
	derivedStates[index].first = first;
	derivedStates[index].second = second;
}

void ODESolver::PrintAllStates()
{
	std::cout << "number of states = " << numberStates << std::endl;
	for (int n = 0; n < numberStates; ++n)
	{
		std::cout << "state: 1st = (" << states[n].first[0] << ", " << states[n].first[1] << ", " << states[n].first[2] <<
			") | 2nd = (" << states[n].second[0] << ", " << states[n].second[1] << ", " << states[n].second[2] << ")" << std::endl
		<< "derivedState: 1st = (" << derivedStates[n].first[0] << ", " << derivedStates[n].first[1] << ", " << derivedStates[n].first[2] <<
			") | 2nd = (" << derivedStates[n].second[0] << ", " << derivedStates[n].second[1] << ", " << derivedStates[n].second[2] << ")" << std::endl;
	}
	std::cout << std::endl;
}

void ODESolver::SolveODE()
{
	for (int n = 0; n < numberStates; ++n)
	{
		// states[n].first += derivedStates[n].first * deltaTime;
		// states[n].second += derivedStates[n].second * deltaTime;
		(this->*integratorFunctionPointer)(n);
	}
}

State& ODESolver::GetState(int index)
{
	if (index < 0 || index >= numberStates)
	{
		std::cerr << "Out of bounds error: index (" << index <<
			") not in integer bound [0, " << numberStates <<
			") in function ODESolver::SetState";

		std::cerr.flush();
	}

	return states[index];
}

void ODESolver::ExplicitEulerIntegrator(int index)
{
	// particle->mPosition += particle->mVelocity * deltaTime;
	// particle->mVelocity += acceleration * deltaTime;

	states[index].first += derivedStates[index].first * deltaTime;
	states[index].second += derivedStates[index].second * deltaTime;
}

void ODESolver::RK4Integrator(int index)
{
	// using namespace Eigen;

	// Vector3d k1_pos = particle->mVelocity;
	// Vector3d k1_vel = acceleration;

	// Vector3d k2_pos = particle->mVelocity + k1_vel * 0.5f * deltaTime;
	// Vector3d k2_vel = acceleration;

	// Vector3d k3_pos = particle->mVelocity + k2_vel * 0.5f * deltaTime;
	// Vector3d k3_vel = acceleration;

	// Vector3d k4_pos = particle->mVelocity + k3_vel * deltaTime;
	// Vector3d k4_vel = acceleration;
	
	// particle->mPosition += 0.1666666 * (k1_pos + 2 * k2_pos + 2 * k3_pos + k4_pos) * deltaTime;
	// particle->mVelocity += 0.1666666 * (k1_vel + 2 * k2_vel + 2 * k3_vel + k4_vel) * deltaTime;

	State k1, k2, k3, k4;

	k1 = RK4Evaluate(states[index], time, 0.0, State(), derivedStates[index].second);
	k2 = RK4Evaluate(states[index], time, deltaTime * 0.5, k1, derivedStates[index].second);
	k3 = RK4Evaluate(states[index], time, deltaTime * 0.5, k2, derivedStates[index].second);
	k4 = RK4Evaluate(states[index], time, deltaTime, k3, derivedStates[index].second);

	Eigen::Vector3d derivativeOfFirst( 1.0 / 6.0 * (k1.first  + 2.0 * (k2.first  + k3.first ) + k4.first ));
	Eigen::Vector3d derivativeOfSecond(1.0 / 6.0 * (k1.second + 2.0 * (k2.second + k3.second) + k4.second));

	states[index].first  += deltaTime * derivativeOfFirst;
	states[index].second += deltaTime * derivativeOfSecond;
}

State ODESolver::RK4Evaluate(const State &initial, double time, double deltaTime, const State &derivative, const Eigen::Vector3d &acceleration)
{
	State state;
	state.first = initial.first + derivative.first * deltaTime;
	state.second = initial.second + derivative.second * deltaTime;

	State output;
	output.first = state.second;
	output.second = acceleration;
	return output;
}

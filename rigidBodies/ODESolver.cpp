#include "ODESolver.h"

#include <Eigen/Dense>
#include <iostream>

ODESolver::ODESolver(int type)
{
		// User Euler for now.
	type = 0;

	deltaTime = 1.0 / 60.0;

	numberStates = 0;
	states = NULL; //((void *)0)
	derivatives = NULL; //((void *)0)

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
	if (derivatives != NULL)
		delete[] derivatives;
}

void ODESolver::SetDimension(int numberStates)
{
	if (this->numberStates != numberStates)
	{
		this->numberStates = numberStates;

		if (states != NULL)
			delete[] states;
		if (derivatives != NULL)
			delete[] derivatives;

		states = new State[numberStates];
		derivatives = new Derivative[numberStates];
	}
}

void ODESolver::SetState(int index, Eigen::Vector3d &position, Eigen::Quaterniond &orientation,
                         Eigen::Vector3d &linearMomentum, Eigen::Vector3d &angularMomentum)
{
	if (index < 0 || index >= numberStates)
	{
		std::cerr << "Out of bounds error: index (" << index <<
			") not in integer bound [0, " << numberStates <<
			") in function ODESolver::SetState";
		return;
	}
	states[index].position = position;
	states[index].orientation = orientation;
	states[index].linearMomentum = linearMomentum;
	states[index].angularMomentum = angularMomentum;
}

void ODESolver::SetDerivative(int index, Eigen::Vector3d &velocity, Eigen::Quaterniond &orientationDeriv,
                              Eigen::Vector3d &accumulatedForce, Eigen::Vector3d &accumulatedTorque)
{
	if (index < 0 || index >= numberStates)
	{
		std::cerr << "Out of bounds error: index (" << index <<
			") not in integer bound [0, " << numberStates <<
			") in function ODESolver::SetDerivedState";
		return;
	}
	derivatives[index].velocity = velocity;
	derivatives[index].orientationDeriv = orientationDeriv;
	derivatives[index].accumulatedForce = accumulatedForce;
	derivatives[index].accumulatedTorque = accumulatedTorque;
}

void ODESolver::PrintAllStates()
{
	// std::cout << "number of states = " << numberStates << std::endl;
	// for (int n = 0; n < numberStates; ++n)
	// {
	// 	std::cout << "state: 1st = (" << states[n].first[0] << ", " << states[n].first[1] << ", " << states[n].first[2] <<
	// 		") | 2nd = (" << states[n].second[0] << ", " << states[n].second[1] << ", " << states[n].second[2] << ")" << std::endl
	// 	<< "derivedState: 1st = (" << derivedStates[n].first[0] << ", " << derivedStates[n].first[1] << ", " << derivedStates[n].first[2] <<
	// 		") | 2nd = (" << derivedStates[n].second[0] << ", " << derivedStates[n].second[1] << ", " << derivedStates[n].second[2] << ")" << std::endl;
	// }
	// std::cout << std::endl;
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

	states[index].position += derivatives[index].velocity * deltaTime;
	states[index].orientation.w() += derivatives[index].orientationDeriv.w() * deltaTime;
	states[index].orientation.x() += derivatives[index].orientationDeriv.x() * deltaTime;
	states[index].orientation.y() += derivatives[index].orientationDeriv.y() * deltaTime;
	states[index].orientation.z() += derivatives[index].orientationDeriv.z() * deltaTime;
	states[index].linearMomentum += derivatives[index].accumulatedForce * deltaTime;
	states[index].angularMomentum += derivatives[index].accumulatedTorque * deltaTime;
}

void ODESolver::RK4Integrator(int index)
{
	// State k1, k2, k3, k4;

	// k1 = RK4Evaluate(states[index], time, 0.0, State(), derivedStates[index].second);
	// k2 = RK4Evaluate(states[index], time, deltaTime * 0.5, k1, derivedStates[index].second);
	// k3 = RK4Evaluate(states[index], time, deltaTime * 0.5, k2, derivedStates[index].second);
	// k4 = RK4Evaluate(states[index], time, deltaTime, k3, derivedStates[index].second);

	// Eigen::Vector3d derivativeOfFirst( 1.0 / 6.0 * (k1.first  + 2.0 * (k2.first  + k3.first ) + k4.first ));
	// Eigen::Vector3d derivativeOfSecond(1.0 / 6.0 * (k1.second + 2.0 * (k2.second + k3.second) + k4.second));

	// states[index].first  += deltaTime * derivativeOfFirst;
	// states[index].second += deltaTime * derivativeOfSecond;
}

State ODESolver::RK4Evaluate(const State &initial, double time, double deltaTime, const Derivative &derivative, const Eigen::Vector3d &acceleration)
{
	// State state;
	// state.first = initial.first + derivative.first * deltaTime;
	// state.second = initial.second + derivative.second * deltaTime;

	// State output;
	// output.first = state.second;
	// output.second = acceleration;
	// return output;
	return State();
}

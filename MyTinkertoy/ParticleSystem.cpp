#include "ParticleSystem.h"
#include "ODESolver.h"
#include "Force.h"
#include "Constraint.h"

#include <iostream>

void ParticleSystem::AddParticleAtPosition(double x, double y, double z)
{
	Particle *particle = new Particle();

	particle->mPosition[0] = x;
	particle->mPosition[1] = y;
	particle->mPosition[2] = z;

	mParticles.push_back(particle);
}

ParticleSystem::ParticleSystem()
{
	
}

void ParticleSystem::SetupSystem()
{
	ApplyGlobalForces();

	ConstraintCircle *circleConstraint = new ConstraintCircle(mParticles[0]);
	ConstraintRigidLine *rigidLineConstraint = new ConstraintRigidLine(mParticles[0], mParticles[1]);
	mConstraintSystem.AddConstraint(circleConstraint);
	mConstraintSystem.AddConstraint(rigidLineConstraint);

	mConstraintSystem.SetNumberParticles((int)mParticles.size());
	mConstraintSystem.SetupConstraints();
}

void ParticleSystem::ApplyGlobalForces()
{
	Force *gravity = new ForceGravity();
	Force *damping = new ForceDamping(0.8);

	for (unsigned int n = 0; n < mParticles.size(); ++n)
	{
		gravity->AddAffectedParticle(mParticles[n]);
		damping->AddAffectedParticle(mParticles[n]);
	}

	mForces.push_back(gravity);
	mForces.push_back(damping);
}

ParticleSystem::~ParticleSystem()
{
	for (unsigned int n = 0; n < mParticles.size(); ++n)
		delete mParticles[n];
	mParticles.clear();

	for (unsigned int n = 0; n < mForces.size(); ++n)
		delete mForces[n];
	mForces.clear();
}

void ParticleSystem::AccumulateForces()
{
	for (unsigned int n = 0; n < mParticles.size(); ++n)
		mParticles[n]->mAccumulatedForce = Eigen::Vector3d(0, 0, 0);

	for (unsigned int n = 0; n < mForces.size(); ++n)
	 	mForces[n]->ApplyForce();

	mConstraintSystem.ApplyConstraints();
	mConstraintSystem.SetupMatrices(mParticles);
	mConstraintSystem.SolveForConstraintForces();

	Eigen::VectorXd constraintForceVector = mConstraintSystem.GetConstraintForceVector();

	for (int n = 0; (unsigned int)n < mParticles.size(); ++n)
	{
		mParticles[n]->mAccumulatedForce +=
			Eigen::Vector3d(constraintForceVector[3 * n], constraintForceVector[3 * n + 1], constraintForceVector[3 * n + 2]);
	}
}

void ParticleSystem::CalculateNextState()
{
	solver->SetDimension((int)mParticles.size());

	for (unsigned int n = 0; n < mParticles.size(); ++n)
	{
		// std::cout << "Particle: m = " << mParticles[n]->mMass << std::endl
		// 	<< "x = (" << mParticles[n]->mPosition[0] << ", " << mParticles[n]->mPosition[1] << ", " << mParticles[n]->mPosition[2] << ")" << std::endl 
		// 	<< "v = (" << mParticles[n]->mVelocity[0] << ", " << mParticles[n]->mVelocity[1] << ", " << mParticles[n]->mVelocity[2] << ")" << std::endl
		// 	<< "f = (" << mParticles[n]->mAccumulatedForce[0] << ", " << mParticles[n]->mAccumulatedForce[1] << ", " << mParticles[n]->mAccumulatedForce[2] << ")" << std::endl;

		Eigen::Vector3d forceOverMass(mParticles[n]->mAccumulatedForce * mParticles[n]->mInverseMass);

		solver->SetState(n, mParticles[n]->mPosition, mParticles[n]->mVelocity);
		solver->SetDerivedState(n, mParticles[n]->mVelocity, forceOverMass);
	}

	// solver->PrintAllStates();

	solver->SolveODE();

	for (unsigned int n = 0; n < mParticles.size(); ++n)
	{
		struct State temp = solver->GetState(n);
		mParticles[n]->mPosition = temp.first;
		mParticles[n]->mVelocity = temp.second;
	}

	// std::cout << "Particle " << 1 << ", K = " << (0.5 * mParticles[0]->mMass * (mParticles[0]->mVelocity.norm() * mParticles[0]->mVelocity.norm()))
	// 	<< ",\tU = " << (9.81 * mParticles[0]->mMass * (mParticles[0]->mPosition[1] + 0.2)) << ",\tTotal Energy = "
	// 	<< ((0.5 * mParticles[0]->mMass * (mParticles[0]->mVelocity.norm() * mParticles[0]->mVelocity.norm())) + (9.81 * mParticles[0]->mMass * (mParticles[0]->mPosition[1] + 0.2))) << std::endl;
	// std::cout << "Particle " << 2 << ", K = " << (0.5 * mParticles[1]->mMass * (mParticles[1]->mVelocity.norm() * mParticles[1]->mVelocity.norm()))
	// 	<< ",\tU = " << (9.81 * mParticles[1]->mMass * (mParticles[1]->mPosition[1] + 0.3)) << ",\tTotal Energy = "
	// 	<< ((0.5 * mParticles[1]->mMass * (mParticles[1]->mVelocity.norm() * mParticles[1]->mVelocity.norm())) + (9.81 * mParticles[1]->mMass * (mParticles[1]->mPosition[1] + 0.3))) << std::endl;

	// std::cout << "Energy of whole system = " << ((0.5 * mParticles[0]->mMass * (mParticles[0]->mVelocity.norm() * mParticles[0]->mVelocity.norm())) + (9.81 * mParticles[0]->mMass * (mParticles[0]->mPosition[1] + 0.2))) + ((0.5 * mParticles[1]->mMass * (mParticles[1]->mVelocity.norm() * mParticles[1]->mVelocity.norm())) + (9.81 * mParticles[1]->mMass * (mParticles[1]->mPosition[1] + 0.3))) << std::endl;

	// std::cout << std::endl;
}
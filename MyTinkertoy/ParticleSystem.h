#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include "Particle.h"
#include "State.h"

class ODESolver;

#include <vector>
#include "Force.h"
#include "ConstraintSystem.h"

#include <Eigen/Dense>

class ParticleSystem
{
private:
	std::vector<Particle*> mParticles;
    std::vector<Force*> mForces;

	ODESolver* solver;
    ConstraintSystem mConstraintSystem;

public:
	ParticleSystem();
	~ParticleSystem();

	int GetNumParticles()
    {
        return mParticles.size();
    }

    Particle* GetParticle(int _index)
    {
        return mParticles[_index];
    }

    void PassODESolverPointer(ODESolver *solver)
    {
    	this->solver = solver;
    }

    void AddParticleAtPosition(double x, double y, double z);
    void AccumulateForces();
    void ApplyGlobalForces();
    void SetupSystem();

    void CalculateNextState();
};

#endif
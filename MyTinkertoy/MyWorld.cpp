#include "MyWorld.h"
#include <iostream>

int MyWorld::getNumParticles()
{
    return mParticleSystem->GetNumParticles();
}

Particle* MyWorld::getParticle(int _index)
{
    return mParticleSystem->GetParticle(_index);
}

MyWorld::MyWorld(int numParticles)
{
    mParticleSystem = new ParticleSystem();
    mSolver = new ODESolver(0);

    mParticleSystem->AddParticleAtPosition(0.2, 0, 0);
    mParticleSystem->AddParticleAtPosition(0.2, -0.1, 0);

    mParticleSystem->SetupSystem();

    mParticleSystem->PassODESolverPointer(mSolver);
    mSolver->PassParticleSystemPointer(mParticleSystem);

    time = 0.0;
    deltaTime = 0.005;
}

MyWorld::~MyWorld()
{
    delete mParticleSystem;
    delete mSolver;
}

void MyWorld::simulate()
{
    mSolver->SetTime(time, deltaTime);

    mParticleSystem->AccumulateForces();
    mParticleSystem->CalculateNextState();

    time += deltaTime;
}

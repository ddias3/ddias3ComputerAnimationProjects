#ifndef _MYWORLD_
#define _MYWORLD_

#include <Eigen/Dense>
#include <iostream>

#include "Particle.h"
#include "ParticleSystem.h"
#include "ODESolver.h"

class MyWorld
{
protected:
    ParticleSystem *mParticleSystem;
    ODESolver      *mSolver;

    double time;
    double deltaTime;

public:
    MyWorld(int numParticles);

    int getNumParticles();
    Particle* getParticle(int _index);

    virtual ~MyWorld();

    // TODO: your simulation code goes here
    void simulate();
};

#endif

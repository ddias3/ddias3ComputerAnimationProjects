#include "MyWorld.h"
#include "Particle.h"

using namespace Eigen;

void AnalyticalGravity(Particle *particle, const Vector3d &acceleration, float absoluteTime);
void ExplicitEulerIntegrator(Particle *particle, const Vector3d &acceleration, float deltaTime);
void RK4Integrator(Particle *particle, const Vector3d &acceleration, float deltaTime);

MyWorld::MyWorld(int _numParticles)
{
    absoluteTime = 0.0;

    // Create particles
    for (int i = 0; i < _numParticles; i++)
    {
        Particle *p = new Particle();
        mParticles.push_back(p);
    }

    // Init particle positions (default is 0, 0, 0)
    mParticles[0]->mPosition[0] = -0.3;
    mParticles[0]->mPosition[1] = 20.0;
    mParticles[1]->mPosition[1] = 20.0;
    mParticles[2]->mPosition[0] = 0.3;
    mParticles[2]->mPosition[1] = 20.0;

    // Init particle colors (default is red)
    mParticles[1]->mColor = Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
    mParticles[2]->mColor = Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
}

MyWorld::~MyWorld()
{
    for (int i = 0; (unsigned int)i < mParticles.size(); i++)
        delete mParticles[i];
    mParticles.clear();
}

void MyWorld::simulate()
{
    absoluteTime += 0.01;

    AnalyticalGravity(mParticles[0], Vector3d(0.0f, -9.81f, 0.0f), absoluteTime);
    ExplicitEulerIntegrator(mParticles[1], Vector3d(0.0f, -9.81f, 0.0f), 0.01f);
    RK4Integrator(mParticles[2], Vector3d(0.0f, -9.81f, 0.0f), 0.01f);

    // using namespace std;
    // cout << "Particle 0: \n" << mParticles[0]->mVelocity << "\nParticle 1:\n" << mParticles[1]->mVelocity << endl << endl;
}

void AnalyticalGravity(Particle *particle, const Vector3d &acceleration, float absoluteTime)
{
    particle->mPosition = 0.5f * acceleration * absoluteTime * absoluteTime + Vector3d(-0.3f, 20.0f, 0);
    particle->mVelocity = acceleration * absoluteTime;
}

void ExplicitEulerIntegrator(Particle *particle, const Vector3d &acceleration, float deltaTime)
{
    particle->mPosition += particle->mVelocity * deltaTime;
    particle->mVelocity += acceleration * deltaTime;
}

void RK4Integrator(Particle *particle, const Vector3d &acceleration, float deltaTime)
{
    Vector3d k1_pos = particle->mVelocity;
    Vector3d k1_vel = acceleration;

    Vector3d k2_pos = particle->mVelocity + k1_vel * 0.5f * deltaTime;
    Vector3d k2_vel = acceleration;

    Vector3d k3_pos = particle->mVelocity + k2_vel * 0.5f * deltaTime;
    Vector3d k3_vel = acceleration;

    Vector3d k4_pos = particle->mVelocity + k3_vel * deltaTime;
    Vector3d k4_vel = acceleration;
    
    particle->mPosition += 0.1666666 * (k1_pos + 2 * k2_pos + 2 * k3_pos + k4_pos) * deltaTime;
    particle->mVelocity += 0.1666666 * (k1_vel + 2 * k2_vel + 2 * k3_vel + k4_vel) * deltaTime;
}
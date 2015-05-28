#include "Force.h"
#include "Particle.h"

void Force::AddAffectedParticle(Particle *particle)
{
	mAffectedParticles.push_back(particle);
}

void ForceGravity::ApplyForce()
{
	for (unsigned int n = 0; n < mAffectedParticles.size(); ++n)
	{
		mAffectedParticles[n]->mAccumulatedForce += Eigen::Vector3d(0, -9.81, 0);
	}
}

ForceDirection::ForceDirection(double x, double y, double z)
{
	force = Eigen::Vector3d(x, y, z);
}


void ForceDirection::ApplyForce()
{
	for (unsigned int n = 0; n < mAffectedParticles.size(); ++n)
	{
		mAffectedParticles[n]->mAccumulatedForce += force;
	}
}

ForceDamping::ForceDamping(double dampingConstant)
{
	this->dampingConstant = dampingConstant;
}

void ForceDamping::ApplyForce()
{
	for (unsigned int n = 0; n < mAffectedParticles.size(); ++n)
	{
		mAffectedParticles[n]->mAccumulatedForce -= dampingConstant * mAffectedParticles[n]->mVelocity;
	}
}
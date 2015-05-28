#include "Force.h"
#include "RigidBody.h"

void Force::AddAffectedRigidBody(RigidBody *rigidBody)
{
	mAffectedRigidBodies.push_back(rigidBody);
}

void ForceGravity::ApplyForce()
{
	for (unsigned int n = 0; n < mAffectedRigidBodies.size(); ++n)
	{
		mAffectedRigidBodies[n]->mAccumulatedForce += Eigen::Vector3d(0, -9.81, 0);
	}
}

ForceDirection::ForceDirection(double x, double y, double z)
{
	force = Eigen::Vector3d(x, y, z);
}


void ForceDirection::ApplyForce()
{
	for (unsigned int n = 0; n < mAffectedRigidBodies.size(); ++n)
	{
		mAffectedRigidBodies[n]->mAccumulatedForce += force;
	}
}

ForceDamping::ForceDamping(double dampingConstant)
{
	this->dampingConstant = dampingConstant;
}

void ForceDamping::ApplyForce()
{
	for (unsigned int n = 0; n < mAffectedRigidBodies.size(); ++n)
	{
		mAffectedRigidBodies[n]->mAccumulatedForce -= dampingConstant * mAffectedRigidBodies[n]->mLinMomentum * mAffectedRigidBodies[n]->mInverseMass;
	}
}
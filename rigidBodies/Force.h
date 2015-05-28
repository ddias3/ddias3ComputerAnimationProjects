#ifndef FORCE_H
#define FORCE_H

#include <vector>
#include <Eigen/Dense>
#include "RigidBody.h"

class Force
{
protected:
	std::vector<RigidBody*> mAffectedRigidBodies;

public:
	Force()
	{
		// do nothing
	}
	virtual ~Force()
	{
		// do nothing
	}

	virtual void ApplyForce() = 0;

	void AddAffectedRigidBody(RigidBody *rigidBody);
};

class ForceGravity : public Force
{
public:
	ForceGravity()
	{
		// do nothing
	}

	virtual ~ForceGravity()
	{
		// do nothing
	}

	void ApplyForce();
};

class ForceDirection : public Force
{
private:
	Eigen::Vector3d force;

public:
	ForceDirection(double x, double y, double z);

	virtual ~ForceDirection()
	{
		// do nothing
	}

	void ApplyForce();
};

class ForceDamping : public Force
{
private:
	double dampingConstant;

public:
	ForceDamping(double dampingConstant);

	virtual ~ForceDamping()
	{
		// do nothing
	}

	void ApplyForce();
};

#endif
#ifndef FORCE_H
#define FORCE_H

#include <vector>
#include <Eigen/Dense>
#include "Particle.h"

class Force
{
protected:
	// ParticleSystem *mSystem;
	std::vector<Particle*> mAffectedParticles;

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

	void AddAffectedParticle(Particle *particle);
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
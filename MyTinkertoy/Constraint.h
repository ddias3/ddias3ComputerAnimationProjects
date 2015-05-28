#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <vector>
#include "Particle.h"
#include <Eigen/Dense>

class Constraint
{
protected:
	int index;
	int length;
	double constraint;
	double constraintDot;
	Eigen::VectorXd constraintGradient;
	Eigen::VectorXd constraintGradientDot;

public:
	std::vector<Particle*> mAffectedParticles;
	virtual void EvaluateConstraint() = 0;

	Constraint()
	{
		index = -1;
		length = -1;
	}

	virtual ~Constraint() { }

	void AddParticle(Particle *particle);

	void SetIndex(int index)
	{
		this->index = index;
	}

	int GetIndex()
	{
		return index;
	}

	int GetLength()
	{
		return length;
	}

	double GetConstraint()
	{
		return constraint;
	}

	double GetConstraintDot()
	{
		return constraintDot;
	}

	Eigen::VectorXd& GetConstraintGradient()
	{
		return constraintGradient;
	}

	Eigen::VectorXd& GetConstraintGradientDot()
	{
		return constraintGradientDot;
	}
};

class ConstraintCircle : public Constraint
{
private:
	double radius;

public:
	ConstraintCircle(Particle *particle);

	~ConstraintCircle()
	{

	}

	void EvaluateConstraint();
};

class ConstraintRigidLine : public Constraint
{
private:
	double distance;

public:
	ConstraintRigidLine(Particle *particle1, Particle *particle2);

	~ConstraintRigidLine()
	{

	}

	void EvaluateConstraint();
};

#endif
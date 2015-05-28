#ifndef CONSTRAINT_SYSTEM_H
#define CONSTRAINT_SYSTEM_H

#include <vector>
#include "Constraint.h"
#include <Eigen/Dense>
#include "Particle.h"

class ConstraintSystem
{
private:
	std::vector<Constraint*> mConstraints;
	int numberParticles;

	Eigen::VectorXd locationVector;
	Eigen::VectorXd velocityVector;
	Eigen::VectorXd forceVector;
	Eigen::VectorXd constraintForceVector;
	Eigen::MatrixXd inverseMassMatrix;
	Eigen::VectorXd lagragianMultipliersVector;
	Eigen::MatrixXd jacobianMatrix;
	Eigen::MatrixXd jacobianTimeDerivativeMatrix;

	Eigen::VectorXd feedbackCVector;
	double feedbackCScalar;
	Eigen::VectorXd feedbackCDotVector;
	double feedbackCDotScalar;

public:
	ConstraintSystem();
	~ConstraintSystem();

	void AddConstraint(Constraint *constraint)
	{
		mConstraints.push_back(constraint);
	}

	void SetNumberParticles(int x)
	{
		numberParticles = x;
	}

	void SetupConstraints();
	void ApplyConstraints();
	void SetupMatrices(std::vector<Particle*> &particles);
	void SolveForConstraintForces();

	Eigen::Vector3d SPECIALHardCodedCircleConstraint(Particle *particle);

	Eigen::VectorXd& GetConstraintForceVector();
};

#endif
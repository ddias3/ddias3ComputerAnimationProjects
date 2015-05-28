#include "ConstraintSystem.h"
#include <iostream>

void ConstraintSystem::SetupConstraints()
{
	for (int n = 0, index = 0; (unsigned int)n < mConstraints.size(); ++n)
	{
		mConstraints[n]->SetIndex(index);
		index += mConstraints[n]->GetLength();
	}
}

void ConstraintSystem::ApplyConstraints()
{
	for (unsigned int n = 0; n < mConstraints.size(); ++n)
		mConstraints[n]->EvaluateConstraint();
}

void ConstraintSystem::SetupMatrices(std::vector<Particle*> &particles)
{
	if ((int)particles.size() != numberParticles)
		std::cerr << "ERROR: particles.size() != numberParticles" << std::endl;

	if (0 == mConstraints.size())
		return;

	locationVector = Eigen::VectorXd(3 * numberParticles);
	velocityVector = Eigen::VectorXd(3 * numberParticles);
	forceVector = Eigen::VectorXd(3 * numberParticles);
	constraintForceVector = Eigen::VectorXd(3 * numberParticles);
	inverseMassMatrix = Eigen::MatrixXd(3 * numberParticles, 3 * numberParticles);
	lagragianMultipliersVector = Eigen::VectorXd((int)mConstraints.size());
	jacobianMatrix = Eigen::MatrixXd((int)mConstraints.size(), 3 * numberParticles);
	jacobianTimeDerivativeMatrix = Eigen::MatrixXd((int)mConstraints.size(), 3 * numberParticles);

	feedbackCVector = Eigen::VectorXd((int)mConstraints.size());
	feedbackCDotVector = Eigen::VectorXd((int)mConstraints.size());
	
	inverseMassMatrix.setZero();
	lagragianMultipliersVector.setZero();
	jacobianMatrix.setZero();
	jacobianTimeDerivativeMatrix.setZero();

	constraintForceVector.setZero();

	for (int n = 0; (unsigned int)n < particles.size(); ++n)
	{
		locationVector[3 * n + 0] = particles[n]->mPosition[0];
		locationVector[3 * n + 1] = particles[n]->mPosition[1];
		locationVector[3 * n + 2] = particles[n]->mPosition[2];

		velocityVector[3 * n + 0] = particles[n]->mVelocity[0];
		velocityVector[3 * n + 1] = particles[n]->mVelocity[1];
		velocityVector[3 * n + 2] = particles[n]->mVelocity[2];

		forceVector[3 * n + 0]    = particles[n]->mAccumulatedForce[0];
		forceVector[3 * n + 1]    = particles[n]->mAccumulatedForce[1];
		forceVector[3 * n + 2]    = particles[n]->mAccumulatedForce[2];

		inverseMassMatrix(3 * n + 0, 3 * n + 0) = particles[n]->mInverseMass;
		inverseMassMatrix(3 * n + 1, 3 * n + 1) = particles[n]->mInverseMass;
		inverseMassMatrix(3 * n + 2, 3 * n + 2) = particles[n]->mInverseMass;
	}

	for (int n = 0; (unsigned int)n < mConstraints.size(); ++n)
	{
		Constraint *constraint = mConstraints[n];

		feedbackCVector[n] = constraint->GetConstraint();
		feedbackCDotVector[n] = constraint->GetConstraintDot();

		// int index = constraint->GetIndex();

		// BEGIN TEMPORARY SOLUTION
		for (int m = 0; (unsigned int)m < particles.size(); ++m)
		{
			for (int o = 0; (unsigned int)o < constraint->mAffectedParticles.size(); ++o)
			{
				if (particles[m] == constraint->mAffectedParticles[o])
				{
					Eigen::VectorXd constraintGradient = constraint->GetConstraintGradient();
					jacobianMatrix(n, 3 * m + 0) = constraintGradient[3 * o + 0];
					jacobianMatrix(n, 3 * m + 1) = constraintGradient[3 * o + 1];
					jacobianMatrix(n, 3 * m + 2) = constraintGradient[3 * o + 2];

					Eigen::VectorXd constraintGradientDot = constraint->GetConstraintGradientDot();
					jacobianTimeDerivativeMatrix(n, 3 * m + 0) = constraintGradientDot[3 * o + 0];
					jacobianTimeDerivativeMatrix(n, 3 * m + 1) = constraintGradientDot[3 * o + 1];
					jacobianTimeDerivativeMatrix(n, 3 * m + 2) = constraintGradientDot[3 * o + 2];
				}
			}
		}
		// END TEMPORARY SOLUTION
	}

// 	std::cout << "No. Particles = " << numberParticles << std::endl;
// 	std::cout << "No. Constraints = " << mConstraints.size() << std::endl;
// 	std::cout << "m = " << mConstraints.size() << ", 3 * n = " << (3 * numberParticles) << std::endl;
// 	std::cout << "locationVector, q" << std::endl << locationVector << std::endl << std::endl;
// 	std::cout << "velocityVector, q_dot" << std::endl << velocityVector << std::endl << std::endl;
// 	std::cout << "forceVector, Q" << std::endl << forceVector << std::endl << std::endl;
// 	// std::cout << "constraintForceVector, Q_hat" << std::endl << constraintForceVector << std::endl << std::endl;
// 	std::cout << "inverseMassMatrix, W" << std::endl << inverseMassMatrix << std::endl << std::endl;
// 	// std::cout << "lagragianMultipliersVector, lambda" << std::endl << lagragianMultipliersVector << std::endl << std::endl;
// 	std::cout << "jacobianMatrix, J" << std::endl << jacobianMatrix << std::endl << std::endl;
// 	std::cout << "jacobianTimeDerivativeMatrix, J_dot" << std::endl << jacobianTimeDerivativeMatrix << std::endl << std::endl;
}

void ConstraintSystem::SolveForConstraintForces()
{
	if (0 == mConstraints.size())
		return;

	Eigen::MatrixXd matrixA(jacobianMatrix * inverseMassMatrix * jacobianMatrix.transpose());
	Eigen::VectorXd vectorB(-jacobianTimeDerivativeMatrix * velocityVector - jacobianMatrix * inverseMassMatrix * forceVector
		- feedbackCScalar * feedbackCVector - feedbackCDotScalar * feedbackCDotVector);
	lagragianMultipliersVector = matrixA.inverse() * vectorB;

	constraintForceVector = jacobianMatrix.transpose() * (lagragianMultipliersVector);// - feedbackCScalar * feedbackCVector - feedbackCDotScalar * feedbackCDotVector);

	// std::cout << "After solve" << std::endl;
	// std::cout << "constraintForceVector, Q_hat" << std::endl << constraintForceVector << std::endl << std::endl;
	// std::cout << "lagragianMultipliersVector, lambda" << std::endl << lagragianMultipliersVector << std::endl << std::endl;
	// std::cout << "feedbackCVector, C" << std::endl << feedbackCVector << std::endl << std::endl;
	// std::cout << "feedbackCDotVector, C_dot" << std::endl << feedbackCDotVector << std::endl << std::endl;
}

Eigen::VectorXd& ConstraintSystem::GetConstraintForceVector()
{
	if (0 == mConstraints.size())
	{
		constraintForceVector = Eigen::VectorXd(3 * numberParticles);
		constraintForceVector.setZero();
	}

	return constraintForceVector;
}

Eigen::Vector3d ConstraintSystem::SPECIALHardCodedCircleConstraint(Particle *particle)
{
	double constraintValue = 0.5 * particle->mPosition.dot(particle->mPosition) - 0.5 * 0.2 * 0.2;
	double constraintDotValue = particle->mPosition.dot(particle->mVelocity);

	double lambda = (-(particle->mPosition).dot(particle->mAccumulatedForce) - particle->mMass * (particle->mVelocity).dot(particle->mVelocity) - 10 * constraintValue - 0.0001 * constraintDotValue)
	                / ((particle->mPosition).dot(particle->mPosition));

    Eigen::Vector3d constraintForce(lambda * particle->mPosition);

    return constraintForce;
}

ConstraintSystem::ConstraintSystem()
{
	numberParticles = 0;
	feedbackCScalar = 10.0;
	feedbackCDotScalar = 4.0;
}

ConstraintSystem::~ConstraintSystem()
{
	for (unsigned int n = 0; n < mConstraints.size(); ++n)
		delete mConstraints[n];
	mConstraints.clear();
}
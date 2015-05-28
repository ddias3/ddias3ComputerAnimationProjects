#include "Constraint.h"
#include <iostream>

void ConstraintCircle::EvaluateConstraint()
{
	if (index < 0)
		std::cerr << "ERROR: ConstraintCircle is not properly initialized" << std::endl;

	Particle *particle = mAffectedParticles[0];

	constraint = 0.5 * particle->mPosition.dot(particle->mPosition) - 0.5 * radius * radius;
	constraintDot = particle->mPosition.dot(particle->mVelocity);

	constraintGradient = Eigen::VectorXd(particle->mPosition);
	constraintGradientDot = Eigen::VectorXd(particle->mVelocity);
}

void Constraint::AddParticle(Particle *particle)
{
	mAffectedParticles.push_back(particle);
}

ConstraintCircle::ConstraintCircle(Particle *particle)
{
	mAffectedParticles.clear();
	mAffectedParticles.push_back(particle);

	radius = 0.2;
	length = 1;
}

ConstraintRigidLine::ConstraintRigidLine(Particle *particle1, Particle *particle2)
{
	mAffectedParticles.clear();
	mAffectedParticles.push_back(particle1);
	mAffectedParticles.push_back(particle2);

	distance = 0.1;
	length = 1;
}

void ConstraintRigidLine::EvaluateConstraint()
{
	if (index < 0)
		std::cerr << "ERROR: ConstraintRigidLine is not properly initialized" << std::endl;

	Particle *particle1 = mAffectedParticles[0];
	Particle *particle2 = mAffectedParticles[1];

	constraint = 0.5 * (particle1->mPosition - particle2->mPosition).dot((particle1->mPosition - particle2->mPosition)) - 0.5 * distance * distance;
	constraintDot = (particle1->mPosition - particle2->mPosition).dot(particle1->mVelocity) - (particle1->mPosition - particle2->mPosition).dot(particle2->mVelocity) ;

	constraintGradient = Eigen::VectorXd(6);
	constraintGradientDot = Eigen::VectorXd(6);

	Eigen::Vector3d tempVector1(particle1->mPosition - particle2->mPosition);
	constraintGradient[0 * 3 + 0] = tempVector1[0];
	constraintGradient[0 * 3 + 1] = tempVector1[1];
	constraintGradient[0 * 3 + 2] = tempVector1[2];

	Eigen::Vector3d tempVector2(particle2->mPosition - particle1->mPosition);
	constraintGradient[1 * 3 + 0] = tempVector2[0];
	constraintGradient[1 * 3 + 1] = tempVector2[1];
	constraintGradient[1 * 3 + 2] = tempVector2[2];

	Eigen::Vector3d tempVectorDot1(particle1->mVelocity - particle2->mVelocity);
	constraintGradientDot[0 * 3 + 0] = tempVectorDot1[0];
	constraintGradientDot[0 * 3 + 1] = tempVectorDot1[1];
	constraintGradientDot[0 * 3 + 2] = tempVectorDot1[2];

	Eigen::Vector3d tempVectorDot2(particle2->mVelocity - particle1->mVelocity);
	constraintGradientDot[1 * 3 + 0] = tempVectorDot2[0];
	constraintGradientDot[1 * 3 + 1] = tempVectorDot2[1];
	constraintGradientDot[1 * 3 + 2] = tempVectorDot2[2];
}
#include "MyWorld.h"
#include "RigidBody.h"
#include "CollisionInterface.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/constraint/WeldJointConstraint.h"

#include <iostream>

using namespace Eigen;

#define EPSILON (0.985)

MyWorld::MyWorld() : mODESolver(0)
{
	mFrame = 0;
	mTimeStep = 0.001;
	mGravity = Vector3d(0.0, -9.8, 0.0);
	mForce.setZero();
	// Create a collision detector
	mCollisionDetector = new CollisionInterface();

	// Create and intialize two default rigid bodies (You can add more rigid bodies if you want) 
	RigidBody *rb1 = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.05, 0.05, 0.05));
	mCollisionDetector->addRigidBody(rb1, "box"); // Put rb1 in collision detector
	rb1->mPosition[0] = -0.3;
	rb1->mPosition[1] = -0.5;
	rb1->mAngMomentum = Vector3d(0.0, 0.1, 0.0);
	mRigidBodies.push_back(rb1);
		
	RigidBody *rb2 = new RigidBody(dart::dynamics::Shape::ELLIPSOID, Vector3d(0.06, 0.06, 0.06));
	mCollisionDetector->addRigidBody(rb2, "ellipse"); // Put rb2 in collision detector
	rb2->mPosition[0] = 0.3;
	rb2->mPosition[1] = -0.5;
	rb2->mAngMomentum = Vector3d(0.1, 0.0, 0.0);
	rb2->mColor = Vector4d(0.2, 0.8, 0.2, 1.0); // Blue
	mRigidBodies.push_back(rb2);

	RigidBody *rb3 = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.1, 0.1, 0.1));
	mCollisionDetector->addRigidBody(rb3, "box");
	rb3->mPosition[1] = -0.5;
	rb3->mAngMomentum = Vector3d(0.001, 0.05, 0.0);
	rb3->mColor = Vector4d(0.01, 0.01, 0.01, 1.0);
	mRigidBodies.push_back(rb3);

	ApplyGlobalForces();
}

void MyWorld::initializePinata() {
	// Add pinata to the collison detector
	mCollisionDetector->addSkeleton(mPinataWorld->getSkeleton(0));
	int nJoints = mPinataWorld->getSkeleton(0)->getNumBodyNodes();
	for (int i = 0; i < nJoints; i++) {
		int nDofs = mPinataWorld->getSkeleton(0)->getJoint(i)->getNumDofs();
		for (int j = 0; j < nDofs; j++)
			mPinataWorld->getSkeleton(0)->getJoint(i)->setDampingCoefficient(j, 1.0);
	}

	// Weld two seems to make a box
	dart::dynamics::BodyNode* top = mPinataWorld->getSkeleton(0)->getBodyNode("top");
	dart::dynamics::BodyNode* front = mPinataWorld->getSkeleton(0)->getBodyNode("front");
	dart::dynamics::BodyNode* back = mPinataWorld->getSkeleton(0)->getBodyNode("back");
	dart::constraint::WeldJointConstraint *joint1 = new dart::constraint::WeldJointConstraint(top, front);    
	dart::constraint::WeldJointConstraint *joint2 = new dart::constraint::WeldJointConstraint(top, back);    
	mPinataWorld->getConstraintSolver()->addConstraint(joint1);
	mPinataWorld->getConstraintSolver()->addConstraint(joint2);
}

MyWorld::~MyWorld() {
	for (unsigned int i = 0; i < mRigidBodies.size(); i++)
		delete mRigidBodies[i];
	mRigidBodies.clear();
	if (mCollisionDetector)
		delete mCollisionDetector;
	if (mPinataWorld)
		delete mPinataWorld;
	for (unsigned int i = 0; i < mForces.size(); ++i)
		delete mForces[i];
	mForces.clear();
}

void MyWorld::ApplyGlobalForces()
{
	Force *gravity = new ForceGravity();

	for (unsigned int n = 0; n < mRigidBodies.size(); ++n)
	{
		gravity->AddAffectedRigidBody(mRigidBodies[n]);
	}

	mForces.push_back(gravity);
}

void MyWorld::AccumulateForces()
{
	for (unsigned int n = 0; n < mRigidBodies.size(); ++n)
		mRigidBodies[n]->mAccumulatedForce.setZero();

	for (unsigned int n = 0; n < mForces.size(); ++n)
	 	mForces[n]->ApplyForce();
}

void MyWorld::CalculateNextState()
{
	mODESolver.SetDimension((int)mRigidBodies.size());

	for (unsigned int n = 0; n < mRigidBodies.size(); ++n)
	{
		RigidBody *rigidBody = mRigidBodies[n];

		Eigen::Matrix3d inertiaTensor = rigidBody->GetInertiaTensor();

		Eigen::Vector3d    angularVelocity(inertiaTensor.inverse() * rigidBody->mAngMomentum);
		Eigen::Quaterniond angularVelocityQuaternion(0, angularVelocity[0], angularVelocity[1], angularVelocity[2]);
		Eigen::Quaterniond orientationDeriv(angularVelocityQuaternion * rigidBody->mQuatOrientation);
		orientationDeriv.w() *= 0.5;
		orientationDeriv.x() *= 0.5;
		orientationDeriv.y() *= 0.5;
		orientationDeriv.z() *= 0.5;

		mODESolver.SetState((int)n, rigidBody->mPosition,
			rigidBody->mQuatOrientation,
			rigidBody->mLinMomentum,
			rigidBody->mAngMomentum);

		Eigen::Vector3d velocity(rigidBody->mLinMomentum * rigidBody->mInverseMass);
		mODESolver.SetDerivative((int)n, velocity,
			orientationDeriv,
			rigidBody->mAccumulatedForce,
			rigidBody->mAccumulatedTorque);

		rigidBody->angularVelocity = angularVelocity;
		rigidBody->velocity = velocity;
	}

	mODESolver.SolveODE();

	for (unsigned int n = 0; n < mRigidBodies.size(); ++n)
	{
		State temp = mODESolver.GetState(n);
		mRigidBodies[n]->mPosition = temp.position;
		mRigidBodies[n]->mQuatOrientation = temp.orientation;
		mRigidBodies[n]->mLinMomentum = temp.linearMomentum;
		mRigidBodies[n]->mAngMomentum = temp.angularMomentum;

		mRigidBodies[n]->mQuatOrientation.normalize();
		mRigidBodies[n]->mOrientation = mRigidBodies[n]->mQuatOrientation.toRotationMatrix();
	}
}

void MyWorld::simulate()
{
	mFrame++;

	mODESolver.SetTime(0.0, mTimeStep);

	AccumulateForces();
	CalculateNextState();

	// Apply external force to the pinata
	mPinataWorld->getSkeleton(0)->getBodyNode("bottom")->addExtForce(mForce);
	mForce.setZero();
	// Simulate Pinata using DART
	mPinataWorld->step();
	// Run collision detector
	mCollisionDetector->checkCollision();

	collisionHandling();

	// Break the pinata if it has enough momentum
	if (mPinataWorld->getSkeleton(0)->getWorldCOMVelocity().norm() > 0.4)
		mPinataWorld->getConstraintSolver()->removeAllConstraints();
}

void MyWorld::collisionHandling()
{
	int numberContacts = mCollisionDetector->getNumContacts();

	// if (numberContacts > 0)
	// 	std::cout << "Actual RigidBody 1 @ " << mRigidBodies[0] << std::endl
	// 	<< "Actual RigidBody 2 @ " << mRigidBodies[1] << std::endl;

	for (int n = 0; n < numberContacts; ++n)
	{
		RigidContact contact = mCollisionDetector->getContact(n);

		// std::cout << "Contact " << n << std::endl
		// << "- Contact RigidBody 1 @ " << contact.rb1 << std::endl
		// << "- Contact RigidBody 2 @ " << contact.rb2 << std::endl;

		Eigen::Vector3d rb1ContactPointVelocity;
		Eigen::Vector3d rb2ContactPointVelocity;

		if (NULL != contact.rb1)
			rb1ContactPointVelocity = (contact.rb1->velocity + contact.rb1->angularVelocity.cross(contact.point - contact.rb1->mPosition));
		else
			rb1ContactPointVelocity = contact.pinataVelocity;

		if (NULL != contact.rb2)
			rb2ContactPointVelocity = (contact.rb2->velocity + contact.rb2->angularVelocity.cross(contact.point - contact.rb2->mPosition));
		else
			rb2ContactPointVelocity = contact.pinataVelocity;

		double contactPointSpeed = contact.normal.dot(rb1ContactPointVelocity - rb2ContactPointVelocity);

		Eigen::Vector3d displacement1;
		Eigen::Vector3d displacement2;
		
		double denominator = 0.0;
		if (NULL != contact.rb1)
		{
			displacement1 = contact.point - contact.rb1->mPosition;
			denominator += contact.rb1->mInverseMass + contact.normal.dot((contact.rb1->GetInertiaTensor().inverse() * (displacement1.cross(contact.normal))).cross(displacement1));
		}
		if (NULL != contact.rb2)
		{
			displacement2 = contact.point - contact.rb2->mPosition;
			denominator += contact.rb2->mInverseMass + contact.normal.dot((contact.rb2->GetInertiaTensor().inverse() * (displacement2.cross(contact.normal))).cross(displacement2));
		}

		double impulseScalar = (-(1 + EPSILON) * contactPointSpeed) / denominator;

		if (NULL != contact.rb1)
		{
			Eigen::Vector3d impulseRB1(impulseScalar * contact.normal);
			Eigen::Vector3d impulseTorqueRB1(displacement1.cross(impulseRB1));

			contact.rb1->mLinMomentum += impulseRB1;
			contact.rb1->mAngMomentum += impulseTorqueRB1;
		}
		if (NULL != contact.rb2)
		{
			Eigen::Vector3d impulseRB2((-impulseScalar) * contact.normal);
			Eigen::Vector3d impulseTorqueRB2(displacement2.cross(impulseRB2));

			contact.rb2->mLinMomentum += impulseRB2;
			contact.rb2->mAngMomentum += impulseTorqueRB2;
		}
	}

	// if (numberContacts > 0)
	// 	std::cout << std::endl;
}

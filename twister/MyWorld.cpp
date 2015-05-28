#include "MyWorld.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Marker.h"
#include <iostream>

using namespace Eigen;
using namespace dart::dynamics;

MyWorld::MyWorld() {
	// Load a skeleton from file
	mSkel = dart::utils::SkelParser::readSkeleton(DART_DATA_PATH"skel/human.skel");
	// Assume that there is only one constraint
	mJ = MatrixXd::Zero(3, mSkel->getNumDofs());
	mConstrainedMarker = -1;
}

MyWorld::~MyWorld() {
	delete mSkel;
}

void MyWorld::solve()
{
	if (mConstrainedMarker == -1)
		return;
	int numIter = 300;
	double alpha = 0.01;
	int nDof = mSkel->getNumDofs();
	VectorXd gradients(nDof);
	VectorXd newPose(nDof);
	for (int i = 0; i < numIter; i++)
	{
		gradients = updateGradients();
		newPose = mSkel->getPositions() - alpha * gradients;
		mSkel->setPositions(newPose); 
		mSkel->computeForwardKinematics(true, false, false); // DART updates all the transformations based on newPose
	}
}

// Current code only works for the left leg with only one constraint
VectorXd MyWorld::updateGradients()
{
	// compute c(q)
	mC = mSkel->getMarker(mConstrainedMarker)->getWorldPosition() - mTarget;

	// compute J(q)
	Vector4d offset;
	offset << mSkel->getMarker(mConstrainedMarker)->getLocalPosition(), 1; // Create a vector in homogeneous coordinates

	BodyNode *node = mSkel->getMarker(mConstrainedMarker)->getBodyNode();

	while (node->getParentBodyNode() != NULL)
	{
		Joint *joint = node->getParentJoint();

		Matrix4d worldToParent = node->getParentBodyNode()->getTransform().matrix();
		Matrix4d parentToJoint = joint->getTransformFromParentBodyNode().matrix();

		Matrix4d jointToChild = joint->getTransformFromChildBodyNode().inverse().matrix();
		
		for (unsigned int currentDoF = 0; currentDoF < joint->getNumDofs(); ++currentDoF)
		{
			Matrix4d rotation;
			rotation.setIdentity();

			for (unsigned int n = 0; n < joint->getNumDofs(); ++n)
				if (n == currentDoF)
					rotation *= joint->getTransformDerivative(n);
				else
					rotation *= joint->getTransform(n).matrix();

			Vector4d jCol = worldToParent * parentToJoint * rotation * jointToChild * offset;

			mJ.col(joint->getIndexInSkeleton(currentDoF)) = jCol.head(3);
		}

		Matrix4d offsetAdjust;
		offsetAdjust.setIdentity();

		for (unsigned int n = 0; n < joint->getNumDofs(); ++n)
			offsetAdjust *= joint->getTransform(n).matrix();

		offset = parentToJoint * offsetAdjust * jointToChild * offset;

		node = node->getParentBodyNode();
	}

	// compute gradients
	VectorXd gradients = 2 * mJ.transpose() * mC;
	return gradients;
}

// Current code only handlse one constraint on the left foot.
void MyWorld::createConstraint(int _index)
{
	mTarget = mSkel->getMarker(_index)->getWorldPosition();
	mConstrainedMarker = _index;

	mJ.setZero();

	// std::cout << "createConstraint(" << _index << ") called" << std::endl;
}

void MyWorld::modifyConstraint(Vector3d _deltaP)
{
	mTarget += _deltaP;
}

void MyWorld::removeConstraint(int _index)
{
	mConstrainedMarker = -1;
	
	mJ.setZero();

	// std::cout << "removeConstraint(" << _index << ") called" << std::endl;
}




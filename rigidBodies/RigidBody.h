#ifndef _RIGIDBODY_
#define _RIGIDBODY_

#include <Eigen/Dense>
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/EllipsoidShape.h"

namespace dart{
	namespace renderer {
		class RenderInterface;
	}
}

class RigidBody {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		RigidBody(dart::dynamics::Shape::ShapeType _type, Eigen::Vector3d _dim) {
		// Create a default rigid body
		mMass = 1.0;
		mInverseMass = 1.0 / mMass;
		mPosition.setZero(); // x = (0, 0, 0)
		mOrientation.setIdentity(); // R = identity
		mQuatOrientation.setIdentity();
		mColor << 0.9, 0.2, 0.2, 1.0; // Red
		if (_type == dart::dynamics::Shape::BOX) {
			mShape = new dart::dynamics::BoxShape(_dim);
			mInertiaTensorBody << (1.0 / 12.0) * mMass * (_dim[1]*_dim[1] + _dim[2]*_dim[2]), 0.0, 0.0,
								   0.0, (1.0 / 12.0) * mMass * (_dim[0]*_dim[0] + _dim[2]*_dim[2]), 0.0,
								   0.0, 0.0, (1.0 / 12.0) * mMass * (_dim[0]*_dim[0] + _dim[1]*_dim[1]);
		} else if (_type == dart::dynamics::Shape::ELLIPSOID) {
			mShape = new dart::dynamics::EllipsoidShape(_dim);
			mInertiaTensorBody << (1.0 / 5.0) * mMass * (_dim[1]*_dim[1] + _dim[2]*_dim[2]), 0.0, 0.0,
								   0.0, (1.0 / 5.0) * mMass * (_dim[0]*_dim[0] + _dim[2]*_dim[2]), 0.0,
								   0.0, 0.0, (1.0 / 5.0) * mMass * (_dim[0]*_dim[0] + _dim[1]*_dim[1]);
		}
		mLinMomentum.setZero();
		mAngMomentum.setZero();
		mAccumulatedForce.setZero();
		mAccumulatedTorque.setZero();
	}
	virtual ~RigidBody() {}

	void draw(dart::renderer::RenderInterface* _ri);

	int getConfigSize() {
		return mPosition.size() + mOrientation.size();
	}

	Eigen::Matrix3d GetInertiaTensor();
	
	double mMass;
	double mInverseMass;

	Eigen::Matrix3d    mInertiaTensorBody;

	Eigen::Vector3d    mPosition;
	Eigen::Matrix3d    mOrientation;
	Eigen::Quaterniond mQuatOrientation;

	Eigen::Vector3d    mLinMomentum;
	Eigen::Vector3d    mAngMomentum;

	Eigen::Vector3d    mAccumulatedForce;
	Eigen::Vector3d    mAccumulatedTorque;

	dart::dynamics::Shape* mShape;

	Eigen::Vector4d mColor;

		// Values saved for convenience
	Eigen::Vector3d angularVelocity;
	Eigen::Vector3d velocity;
};

#endif

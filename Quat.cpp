/******************************************************************************/
/*!
\file		Quat.cpp
\author		Justin Leow
\brief
	Floating point quaternion that can represents a rotation about an axis in 3D
	space. Rotations are ideally constructed using an axis/angle format.

	Conversion between this format and the Rotator's Pitch/Yaw/Roll
	(Euler) format of representing rotations is also supported, and rotations
	can be converted between the two just fine.

	Order matters when composing quaternions: C = A * B will yield a quaternion C that logically
	first applies B then A to any subsequent transformation (right first, then left).

	Note that the game's X/Y/Z axes corresponds to an object's Left/Up/Forward
	Vectors correspondingly (imagine an object looking into the camera).

All content (C) 2019 DigiPen (SINGAPORE) Corporation, all rights reserved.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "Quat.h"
#include "Rotator.h"
#include "Vector3D.h"
#include "Matrix3x3.h"
#include "Transform.h"
#include <iostream>

namespace SNova
{

// Allowed error for a normalized quaternion
#define THRESH_QUAT_NORMALIZED 0.01f

const Quat Quat::Identity = Quat{};

Quat::Quat(Vec3 Axis, float AngleRad)
{
	const float half_a = 0.5f * AngleRad;
	float s, c;
	Math::SinCos(half_a, s, c);

	// Old
	//x = s * Axis.x;
	//y = s * Axis.y;
	//z = s * Axis.z;

	x = s * -Axis.z;
	y = s * -Axis.x;
	z = s *  Axis.y;
	w = c;

	DiagnosticCheckNaN();
}

Quat Quat::MakeFromEuler(const Vec3& eulers)
{
	return Rotator::MakeFromEuler(eulers).Quaternion();
}

Vec3 Quat::Euler() const
{
	return GetRotator().Euler();
}

Rotator Quat::GetRotator() const
{
	/*
	 * Reference: 
	 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	 * http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/ 
	 *
	 * This method of conversion has safeties to address "gymbal lock"
	 * when converting from Quat to Eulers.
	 *
	 * While this conversion works in a static frame, it is recommended
	 * to use Quat::MakeFromEuler or the Rotator Constructor followed 
	 * by successive Quat::operator* to perform rotations on GameObjects.
	 *
	 * Staying in Quaternion space prevents gymbal lock and is more
	 * efficient than going back and forth to Euler space
	 *
	 * i.e.
	 * Rotator x, y, z;
	 * Quat a{x}, b{y}, c{z};
	 * a * b * c is better than x.Combine(y).Combine(z)
	 */
	
	DiagnosticCheckNaN();

	const float yawY = 2.f * (w * z + x * y);
	const float yawX = (1.f - 2.f * (y * y + z * z));
	const float singularityTest = z * x - w * y;
	const float SINGULARITY_THRESHOLD = 0.4999995f;
	Rotator result;

	if (singularityTest < -SINGULARITY_THRESHOLD)
	{
		result.pitch = -90.f;
		result.yaw = RAD2DEG(atan2f(yawY, yawX));
		result.roll = Rotator::NormalizeAxis(-result.yaw - RAD2DEG(2.f * atan2f(x, w)));
	}
	else if (singularityTest > SINGULARITY_THRESHOLD)
	{
		result.pitch = 90.f;
		result.yaw = RAD2DEG(atan2f(yawY, yawX));
		result.roll = Rotator::NormalizeAxis(result.yaw - RAD2DEG(2.f * atan2f(x, w)));
	}
	else
	{
		result.pitch = RAD2DEG(asinf(2.f * singularityTest));
		result.yaw = RAD2DEG(atan2f(yawY, yawX));
		result.roll = RAD2DEG(atan2(-2.f * (w * x + y * z), 1.f - 2.f * (x * x + y * y)));
	}

	result.DiagnosticCheckNaN();
	return result;
}

Matrix3x3 Quat::Matrix() const
{
	return GetRotator().Matrix();
}

bool Quat::IsNormalized() const
{
	return fabsf(1.f - SizeSquared()) < THRESH_QUAT_NORMALIZED;
}

Quat Quat::Slerp_NotNormalized(const Quat& q1, const Quat& q2, float t)
{
	/**
	 * Solution adapted from:
	 * https://en.wikipedia.org/wiki/Slerp
	 */

	// Compute the cosine of the angle between the two vectors.
	const float rawCosSum = q1 | q2;

	// Align quats so they take the shorter route
	const float cosSum = (rawCosSum >= 0.f) ? rawCosSum : -rawCosSum;

	float scale0, scale1;
	const float DOT_THRESHOLD = 0.9999f;

	if (cosSum < DOT_THRESHOLD)
	{
		const float omega = acosf(cosSum);
		const float invSin = 1.f / sinf(omega);
		scale0 = sinf((1.f - t) * omega) * invSin;
		scale1 = sinf(t * omega) * invSin;
	}
	else
	{
		// Inputs too close; use linear interpolation.
		scale0 = 1.0f - t;
		scale1 = t;
	}

	// From above, flip if necessary
	scale1 = (rawCosSum >= 0.f) ? scale1 : -scale1;

	Quat result;

	result.w = scale0 * q1.w + scale1 * q2.w;
	result.x = scale0 * q1.x + scale1 * q2.x;
	result.y = scale0 * q1.y + scale1 * q2.y;
	result.z = scale0 * q1.z + scale1 * q2.z;

	return result;
}

void Quat::UpdateBoundTransform()
{
	if (mp_BoundTransform)
	{
		Rotator tmp = GetRotator();
		mp_BoundTransform->rotator.pitch = tmp.pitch;
		mp_BoundTransform->rotator.yaw = tmp.yaw;
		mp_BoundTransform->rotator.roll = tmp.roll;
		//mp_BoundTransform->mtx = Matrix();
		mp_BoundTransform->UpdateMtx();
	}
}

} // namespace SNova
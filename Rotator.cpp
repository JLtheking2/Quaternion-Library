#pragma once
#include "Rotator.h"
#include "Quat.h"
#include "Matrix3x3.h"
#include "Transform.h"

namespace SNova
{

const Rotator Rotator::ZeroRotator = Rotator{};

Rotator::Rotator(const Quat& q)
{
	*this = q.GetRotator();
	DiagnosticCheckNaN();
}

Rotator Rotator::GetInverse() const
{
	return Quaternion().Inverse().GetRotator();
}

Vec3 Rotator::Vector() const
{
	float CP, SP, CY, SY;
	Math::SinCos(DEG2RAD(pitch),	SP, CP);
	Math::SinCos(DEG2RAD(yaw),		SY, CY);

	return Vec3{ CP*CY, CP*SY, SP };
}

Quat Rotator::Quaternion() const
{
	/*
	 * Adapted from:
	 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	 */

	constexpr float DEG_TO_RAD_DIVIDED_BY_2 = (PI / (180.f)) / 2.f;
	float SP, SY, SR, CP, CY, CR;

	Math::SinCos(pitch * DEG_TO_RAD_DIVIDED_BY_2, SP, CP);
	Math::SinCos(yaw   * DEG_TO_RAD_DIVIDED_BY_2, SY, CY);
	Math::SinCos(roll  * DEG_TO_RAD_DIVIDED_BY_2, SR, CR);

	Quat result;

	result.x =  CR * SP * SY - SR * CP * CY;
	result.y = -CR * SP * CY - SR * CP * SY;
	result.z =  CR * CP * SY - SR * SP * CY;
	result.w =  CR * CP * CY + SR * SP * SY;

	DiagnosticCheckNaN();

	return result;
}

Vec3 Rotator::Euler() const
{
	return Vec3{ pitch, yaw, roll };
}

Matrix3x3 Rotator::Matrix() const
{
	Matrix3x3 m;
	float SP, SY, SR, CP, CY, CR;

	float newPitch = yaw;
	float newYaw   = -roll;
	float newRoll  = pitch;

	Math::SinCos(DEG2RAD(newPitch), SP, CP);
	Math::SinCos(DEG2RAD(newYaw), SY, CY);
	Math::SinCos(DEG2RAD(newRoll), SR, CR);

	m.m2[0][0] = CP * CY;
	m.m2[0][1] = CP * SY;
	m.m2[0][2] = SP;
	
	m.m2[1][0] = SR * SP * CY - CR * SY;
	m.m2[1][1] = SR * SP * SY + CR * CY;
	m.m2[1][2] = -SR * CP;
	
	m.m2[2][0] = -(CR * SP * CY + SR * SY);
	m.m2[2][1] = CY * SR - CR * SP * SY;
	m.m2[2][2] = CR * CP;

	return m;
}

Vec3 Rotator::RotateVector(const Vec3& v) const
{
	return Matrix() * v;
}

Vec3 Rotator::UnrotateVector(const Vec3& v) const
{
	// Inverse of rotation matrices == transpose (faster)
	return Matrix().GetTranspose() * v;
}

Rotator Rotator::Combine(const Rotator& A, const Rotator& B)
{
	return (A.Quaternion() * B.Quaternion()).GetRotator();
}

void Rotator::UpdateBoundTransform()
{
	if (mp_BoundTransform)
	{
		Quat tmp = Quaternion();
		mp_BoundTransform->rotation.w = tmp.w;
		mp_BoundTransform->rotation.x = tmp.x;
		mp_BoundTransform->rotation.y = tmp.y;
		mp_BoundTransform->rotation.z = tmp.z;
		//mp_BoundTransform->mtx = Matrix();
		mp_BoundTransform->UpdateMtx();
	}
}

#pragma warning( push )
#pragma warning( disable : 26444 )
void Rotator::PerformTest()
{
	Quat q{ *this };

	static int testNum = 11;
	switch (testNum++)
	{
		// ----------- Test Rotator ------------
	case 1:
		std::cout << "Test ZeroRotator" << std::endl;
		*this = Rotator{};
		break;

	case 2:
		std::cout << "Test EulerAngleConstruct" << std::endl;
		*this = Rotator{10.f, 20.f, 30.f};
		break;

	case 3:
		std::cout << "Test Quaternion Construction" << std::endl;
		std::cout << "Current Rotation: " << ToString() << std::endl;
		std::cout << "Convert to quat and back: " << Quaternion().GetRotator().ToString() << std::endl;

	/*case 4:
		std::cout << "Test Quat/Rotator \"Addition\"" << std::endl;
		Quat q = Quaternion();
		Quat q2 = q * q;
		Rotator r1 = *this + *this; // wrong way to combine rotations
		Rotator r2 = Rotator::Combine(*this, *this);

		std::cout << "\nq.quat: " << q.ToString() << "\nq.rotator: " << q.GetRotator().ToString() << std::endl;
		std::cout << "\nq2.quat: " << q2.ToString() << "\nq2.rotator: " << q2.GetRotator().ToString() << std::endl;
		std::cout << "\nr1.quat: " << r1.Quaternion().ToString() << "\nr1.rotator: " << r1.ToString() << std::endl;
		std::cout << "\nr2.quat: " << r2.Quaternion().ToString() << "\nr2.rotator: " << r2.ToString() << std::endl;
	*/
	case 4:
		std::cout << "Test MakeFromEuler (should be (5,10,15))" << std::endl;
		std::cout << Rotator::MakeFromEuler(Vec3{ 5.f, 10.f, 15.f }).ToString();
		break;

	case 5:
		std::cout << "Test DirectionVector" << std::endl;
		std::cout << ToString() + ": " + Vector().ToString() << std::endl;
		break;

	case 6:
		std::cout << "\nTest Matrix" << std::endl;
		std::cout << "Matrix(): " << Matrix().ToString() << std::endl;
		std::cout << "Matrix() * {1,0,0}: " << (Matrix() * Vec3 { 1.f, 0.f, 0.f }).ToString() << std::endl;
		break;

	//case 7:
	//	Vec3 v1{ 1.0f,0.f,0.f };
	//	Vec3 v2{ 0.f, 1.f, 0.f };

	//	std::cout << "\nTest RotateVector" << std::endl;

	//	std::cout << "\nRotating " << v1.ToString() << " by: " << ToString() << std::endl;
	//	//std::cout << "After Rotation: " << RotateVector(v1).ToString() << std::endl;
	//	std::cout << "Rotator: After Rotation: " << RotateVector(v1).ToString() << std::endl;
	//	std::cout << "Quat:    After Rotation: " << Quaternion().RotateVector(v1).ToString() << std::endl;
	//	std::cout << "Rotator: After Unrotation: " << UnrotateVector(RotateVector(v1)).ToString() << std::endl;
	//	std::cout << "Quat:    After Unrotation: " << Quaternion().UnrotateVector(Quaternion().RotateVector(v1)).ToString() << std::endl;

	//	std::cout << "\nRotating " << v2.ToString() << " by: " << ToString() << std::endl;
	//	//std::cout << "After Rotation: " << RotateVector(v2).ToString() << std::endl;
	//	std::cout << "Rotator: After Rotation: " << RotateVector(v2).ToString() << std::endl;
	//	std::cout << "Quat:    After Rotation: " << Quaternion().RotateVector(v2).ToString() << std::endl;
	//	std::cout << "Rotator: After Unrotation: " << UnrotateVector(RotateVector(v2)).ToString() << std::endl;
	//	std::cout << "Quat:    After Unrotation: " << Quaternion().UnrotateVector(Quaternion().RotateVector(v2)).ToString() << std::endl;

	case 7:
		std::cout << "Test Normalize" << std::endl;
		std::cout << "Normalized: " << GetNormalized().ToString() << std::endl;
		std::cout << "Clamped: " << GetClamped().ToString() << std::endl;
		Clamp();
		std::cout << "Post-Clamp: " << ToString() << std::endl;
		Normalize();
		std::cout << "Post-Normalize: " << ToString() << std::endl;

	/*case 8:
		std::cout << "Test Combine" << std::endl;
		Quat q{ *this };
		Rotator r{ *this };
		std::cout << (q* q).GetRotator().ToString() << std::endl;
		std::cout << r.Combine(r).ToString() << std::endl;
		std::cout << (q* q* q).GetRotator().ToString() << std::endl;
		std::cout << r.Combine(r).Combine(r).ToString() << std::endl;
	*/
		
	// ----------- Test Quat ------------
	case 8:
		std::cout << "Test QuatIdentity" << std::endl;
		std::cout << Quat{}.ToString() << std::endl;
		std::cout << Quat::Identity.ToString() << std::endl;
		std::cout << (q* Quat::Identity).ToString() << std::endl;
		break;

	case 9:
		std::cout << "Test Axis Angle" << std::endl;
		std::cout << "q: " << q.ToString() << std::endl;
		std::cout << "Axis: " << q.GetRotationAxis().ToString() << std::endl;
		std::cout << "Angle: " << q.GetAngle() << std::endl;
		std::cout << "Back to quat: " << Quat{ q.GetRotationAxis(), q.GetAngle() }.ToString() << std::endl;
		break;

	case 10:
		std::cout << "Test Euler" << std::endl;
		std::cout << std::boolalpha << Quat::MakeFromEuler(Vec3{ pitch, yaw, roll }).Equals(Quaternion()) << std::endl;
		std::cout << Vec3{ pitch, yaw, roll }.Equals(Quaternion().Euler()) << std::endl;
		break;

	//case 11:
		//Rotator r1{ 10.f,10.f,0.f };
		//Rotator r2{ 20.f,10.f,0.f };
		//Rotator r3{ 30.f,10.f,0.f };

		//Quat q1 = Quat::MakeFromEuler(10.f, 10.f, 0.f);
		//Quat q2 = Quat::MakeFromEuler(20.f, 10.f, 0.f);
		//Quat q3 = Quat::MakeFromEuler(30.f, 10.f, 0.f);

		//// Following 4 expressions give equal results
		//std::cout << "Test Multiplication" << std::endl;
		//std::cout << (q1* q2* q3).ToString() << std::endl;
		//std::cout << (q1* (q2* q3)).ToString() << std::endl;
		//std::cout << ((q1* q2)* q3).ToString() << std::endl;
		//std::cout << r1.Combine(r2).Combine(r3).Quaternion().ToString() << std::endl;

		//// Equivalent to q3 * q2 * q1 because Rotator::Combine uses quaternion
		//// multiplication internally
		//std::cout << r3.Combine(r2).Combine(r1).Quaternion().ToString() << std::endl;

	case 11:
		std::cout << "Test Inverse" << std::endl;
		std::cout << std::boolalpha << (Quaternion() * Quaternion().Inverse()).Equals(Quat::Identity) << std::endl;
		break;

	case 12:
		std::cout << "\nTest Forward Up Right Vectors" << std::endl;
		std::cout << "Forward(X): " << q.GetForwardVector().ToString() << std::endl;
		std::cout << "Up(Y):      " << q.GetUpVector().ToString() << std::endl;
		std::cout << "Right(Z):   " << q.GetRightVector().ToString() << std::endl;
		break;

	case 13:
		std::cout << "\nTest Angular Distance" << std::endl;
		std::cout << "Vector(): " << q.Vector().ToString() << std::endl;
		std::cout << "Rotator(): " << q.GetRotator().ToString() << std::endl;

		// Note that rotations q and -q are equal
		std::cout << q.AngularDistance(Quat::Identity) << std::endl;
		std::cout << (-q).AngularDistance(Quat::Identity) << std::endl;

		// Note that distance between 2 vectors != angular distance between 2 quaternions
		std::cout << Vector3DAngle(q.Vector(), Vec3{ 1.f, 0.f, 0.f }) << std::endl;
		break;

	/*case 14:
		std::cout << "\nTest Find Between Vectors" << std::endl;
		
		Vec3 v1{ 1.f, 3.6f,0.f };
		Vec3 v2{ 0.f, 1.f, 3.6f };
		Quat a = Quat::FindBetween(v1, v2);

		std::cout << a.ToString() << std::endl;
		std::cout << a.RotateVector(v1).ToString() << std::endl;
		std::cout << std::boolalpha << a.RotateVector(v1).Equals(v2) << std::endl;*/

	/*case 15:
		std::cout << "Test Slerp" << std::endl;

		Quat q1 = Quat::MakeFromEuler(10.f, 0.f, 0.f);
		Quat q2 = Quat::MakeFromEuler(20.f, 100.f, 0.f);

		std::cout << Quat::Slerp(q1, q2, .5f).ToString() << std::endl;*/
	
	default:
		std::cout << "\n\n-----Restarting tests-----\n\n" << std::endl;
		testNum = 1;
		//testNum -= 2;
		//PerformTest();
		break;
	}
}
#pragma warning( pop )

} // namespace SNova
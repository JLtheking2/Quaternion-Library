/******************************************************************************/
/*!
\file		Rotator.h
\author		Justin Leow
\brief
	Implements a container for rotation information using the
	Pitch/Yaw/Roll representation. This also lines up with X/Y/Z (Euler)
	rotations.

	Note that the game's X/Y/Z axes corresponds to an object's Forward/Up/Right
	Vectors respectively (imagine an object looking into the camera).

All content (C) 2019 DigiPen (SINGAPORE) Corporation, all rights reserved.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "GenMath.h"
#include "Vector3D.h"

namespace SNova
{

struct Rotator
{
	friend struct Quat;
	friend class Transform;
	
	/////////////////////////////////////////////////////
	// Data Members
public:
	// (DO NOT MODIFY THESE DIRECTLY!!! Use .Add or operator=)
	// Rotation about the left axis (X Axis): Looking Up/Down
	float pitch;
	
	// (DO NOT MODIFY THESE DIRECTLY!!! Use .Add or operator=)
	// Rotation about the up axis (Y Axis): Looking Left/Right
	float yaw;
	
	// (DO NOT MODIFY THESE DIRECTLY!!! Use .Add or operator=)
	// Rotation about the forward axis (Z Axis): Tilting Head
	float roll;

private:
	// To update an object's transform if this is modified
	Transform* mp_BoundTransform = nullptr;

	/////////////////////////////////////////////////////
	// Constants
public:
	static const Rotator ZeroRotator;

	/////////////////////////////////////////////////////
	// Constructors
public:
	// Default Constructor (Generates ZeroRotator)
	inline Rotator() : pitch{ 0.f }, yaw{ 0.f }, roll{ 0.f } {}

	// Construct from Euler Angles (X,Y,Z)
	inline Rotator(float inPitch, float inYaw, float inRoll)
		: pitch{ inPitch }, yaw{ inYaw }, roll{ inRoll }
	{}

	// Construct from Quaternion
	explicit Rotator(const Quat& q);

	/*
	 * Convert a vector of Euler angles (in degrees) into a Rotator.
	 * In this engine, this is equivalent to the Pitch-Yaw-Roll constructor.
	 */
	static inline Rotator MakeFromEuler(const Vec3& eulers);
	
	static inline Rotator MakeFromEuler(float x_pitch, float y_yaw, float z_roll);

	/////////////////////////////////////////////////////
	// Conversion Functions

	// Convert this rotation into a unit (direction) vector facing in its direction
	Vec3 Vector() const;

	// Get this rotation as a quaternion
	Quat Quaternion() const;

	// Get a Vec3 representation as a rotation in Euler angles
	Vec3 Euler() const;

	// Get a matrix representation as a rotation transformation
	// ONLY USE FOR RENDERING!!!
	Matrix3x3 Matrix() const;

	/////////////////////////////////////////////////////
	// Operators

	// Assignment Operator.
	inline Rotator& operator=(const Rotator& r);

	// Component-wise Addition
	// NOTE: Addition does not "combine" rotations as you expect. Use Rotator::Combine
	inline Rotator operator+(const Rotator& r) const;
	inline Rotator operator-(const Rotator& r) const;
	inline Rotator& operator+=(const Rotator& r);
	inline Rotator& operator-=(const Rotator& r);

	// Component-wise Scaling
	inline Rotator operator*(float scale) const;
	inline Rotator& operator*=(float scale);

	// Comparison Operators. Checks for exact equality.
	inline bool operator== (const Rotator& r) const;
	inline bool operator!= (const Rotator& r) const;

	/////////////////////////////////////////////////////
	// Member Functions

	/*
	 * Checks if this rotation is nearly zero within specified tolerance,
	 *
	 * Note that Rotator(0,0,360) == Rotator(0,0,0) because it causes
	 * the same final orientation as the ZeroRotator.
	 */
	 inline bool IsNearlyZero(float tolerance = KINDA_SMALL_NUMBER) const;

	/*
	 * Checks if this rotation has exactly zero rotation.
	 *
	 * Note that Rotator(0,0,360) == Rotator(0,0,0) because it causes
	 * the same final orientation as the ZeroRotator.
	 */
	inline bool IsZero() const;

	/*
	 * Checks whether two rotators are equal within specified tolerance.
	 *
	 * Note that Rotator(0,0,360) == Rotator(0,0,0) because it causes
	 * the same final orientation as the ZeroRotator.
	 */
	inline bool Equals(const Rotator& r, float tolerance = KINDA_SMALL_NUMBER) const;

	// Add to each component of this rotator. This MODIFIES this rotator.
	// Does NOT combine rotations as you expect. Use Rotator::Combine
	inline Rotator& Add(float deltaPitch, float deltaYaw, float deltaRoll);

	// Returns the inverse of this rotator
	Rotator GetInverse() const;

	// Returns a vector rotated by this rotator
	Vec3 RotateVector(const Vec3& v) const;

	// Returns the vector rotated by the inverse of this vector
	Vec3 UnrotateVector(const Vec3& v) const;

	// Clamps rotation values so they fall within the range [0,360].
	inline Rotator& Clamp();

	// Clamps this rotator's values within (-180, 180].
	inline Rotator& Normalize();

	// Creates a copy of this rotator with values within (-180, 180]
	inline Rotator GetNormalized() const;

	// Creates a copy of this rotator with values within [0,360)
	inline Rotator GetClamped() const;

	// Combines 2 rotations by first applying A, then B
	static Rotator Combine(const Rotator& A, const Rotator& B);

	// Sets this rotator to the result of applying this rotator first, then R
	inline Rotator Combine(const Rotator& R);

	/////////////////////////////////////////////////////
	// For Debugging

	// Converts value of rotator into human-readable string
	inline std::string ToString() const;

	// Check that no NaN values in rotator
	inline void DiagnosticCheckNaN() const;
	inline bool ContainsNaN() const;

	void PerformTest();

private:
	/////////////////////////////////////////////////////
	// Helper Functions

	// Clamps an angle to the range of [0,360)
	inline static float ClampAxis(float angle);

	// Clamps an angle to the range of (-180, 180]
	inline static float NormalizeAxis(float angle);

	// Preserve correctness of game object's transform
	void UpdateBoundTransform();

}; // struct Rotator

// ------------------------- INLINE IMPLEMENTATIONS ---------------------

inline Rotator Rotator::MakeFromEuler(const Vec3& eulers)
{
	return Rotator{ eulers.x, eulers.y, eulers.z };
}

inline Rotator Rotator::MakeFromEuler(float x_pitch, float y_yaw, float z_roll)
{
	return Rotator(x_pitch, y_yaw, z_roll);
}

inline Rotator& Rotator::operator=(const Rotator& r)
{
	pitch = r.pitch;
	yaw = r.yaw;
	roll = r.roll;
	UpdateBoundTransform();
	// Do not shallow copy mp_BoundQuat because our transform's quat is unique
	return *this;
}

inline Rotator Rotator::operator+(const Rotator& r) const
{
	return Rotator(pitch + r.pitch, yaw + r.yaw, roll + r.roll);
}

inline Rotator Rotator::operator-(const Rotator& r) const
{
	return Rotator(pitch - r.pitch, yaw - r.yaw, roll - r.roll);
}

inline Rotator& Rotator::operator+=(const Rotator& r)
{
	pitch += r.pitch;
	yaw += r.yaw;
	roll += r.roll;
	DiagnosticCheckNaN();
	UpdateBoundTransform();
	return *this;
}

inline Rotator& Rotator::operator-=(const Rotator& r)
{
	pitch -= r.pitch;
	yaw -= r.yaw;
	roll -= r.roll;
	DiagnosticCheckNaN();
	UpdateBoundTransform();
	return *this;
}

inline Rotator Rotator::operator*(float scale) const
{
	return Rotator(pitch * scale, yaw * scale, roll * scale);
}

inline Rotator& Rotator::operator*=(float scale)
{
	pitch *= scale;
	yaw *= scale;
	roll *= scale;
	DiagnosticCheckNaN();
	UpdateBoundTransform();
	return *this;
}

inline bool Rotator::operator==(const Rotator& r) const
{
	return pitch == r.pitch && yaw == r.yaw && roll == r.roll;
}

inline bool Rotator::operator!=(const Rotator& r) const
{
	return pitch != r.pitch || yaw != r.yaw || roll != r.roll;
}

inline bool Rotator::IsNearlyZero(float tolerance) const
{
	return (fabsf(NormalizeAxis(pitch))	<= tolerance)
		&& (fabsf(NormalizeAxis(yaw))	<= tolerance)
		&& (fabsf(NormalizeAxis(roll))	<= tolerance);
}

inline bool Rotator::IsZero() const
{
	return (fabsf(ClampAxis(pitch))	== 0.f)
		&& (fabsf(ClampAxis(yaw))	== 0.f)
		&& (fabsf(ClampAxis(roll))	== 0.f);
}

inline bool Rotator::Equals(const Rotator& r, float tolerance) const
{
	return (fabsf(NormalizeAxis(pitch	- r.pitch))	<= tolerance)
		&& (fabsf(NormalizeAxis(yaw		- r.yaw))	<= tolerance)
		&& (fabsf(NormalizeAxis(roll	- r.roll))	<= tolerance);
}

inline Rotator& Rotator::Add(float deltaPitch, float deltaYaw, float deltaRoll)
{
	pitch += deltaPitch;
	yaw += deltaYaw;
	roll += deltaRoll;
	UpdateBoundTransform();
	return *this;
}

inline Rotator& Rotator::Clamp()
{
	pitch = ClampAxis(pitch);
	yaw = ClampAxis(yaw);
	roll = ClampAxis(roll);
	return *this;
}

inline Rotator& Rotator::Normalize()
{
	pitch = NormalizeAxis(pitch);
	yaw = NormalizeAxis(yaw);
	roll = NormalizeAxis(roll);
	return *this;
}

inline Rotator Rotator::GetNormalized() const
{
	Rotator r = *this;
	return r.Normalize();
}

inline Rotator Rotator::GetClamped() const
{
	Rotator r = *this;
	return r.Clamp();
}

inline Rotator Rotator::Combine(const Rotator& R)
{
	return Rotator::Combine(*this, R);
}

inline std::string Rotator::ToString() const
{
	std::ostringstream oss;
	oss << "p=" << pitch << " y=" << yaw << " r=" << roll;
	return oss.str();
}

#if ENABLE_NAN_CHECK
inline void Rotator::DiagnosticCheckNaN() const
{
	if (ContainsNaN())
	{
		std::cout << "Rotator contains NaN: " << ToString() << std::endl;
		*const_cast<Rotator*>(this) = Rotator::ZeroRotator;
	}
}

#else
inline void Rotator::DiagnosticCheckNaN() const {}
#endif

inline bool Rotator::ContainsNaN() const
{
	return (!Math::IsFinite(pitch) ||
			!Math::IsFinite(yaw) ||
			!Math::IsFinite(roll)
			);
}

inline float Rotator::ClampAxis(float angle)
{
	// Get angle between (-360, 360)
	angle = fmodf(angle, 360.f);

	// Shift to [0, 360)
	if (angle < 0.f)
		angle += 360.f;

	return angle;
}

inline float Rotator::NormalizeAxis(float angle)
{
	// Get angle between (-360, 360)
	angle = fmodf(angle, 360.f);

	// Shift to (-180, 180]
	if (angle > 180.f)
		angle -= 360.f;
	else if (angle < -180.f)
		angle += 360.f;

	return angle;
}

} // namespace SNova
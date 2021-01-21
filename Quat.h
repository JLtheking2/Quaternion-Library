/******************************************************************************/
/*!
\file		Quat.h
\author		Justin Leow
\brief
	Floating point quaternion that can represents a rotation about an axis in 3D
	space. Rotations are ideally constructed using an axis/angle format.

	Conversion between this format and the Rotator's Pitch/Yaw/Roll 
	(Euler) format of representing rotations is also supported, and rotations
	can be converted between the two just fine.

	Order matters when composing quaternions: C = A * B will yield a quaternion C that logically
	first applies B then A to any subsequent transformation (right first, then left).

	Note that the game's X/Y/Z axes corresponds to an object's Forward/Up/Right
	Vectors respectively (imagine an object looking into the camera).

All content (C) 2019 DigiPen (SINGAPORE) Corporation, all rights reserved.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "Vector3D.h"
#include "Rotator.h"

namespace SNova
{

struct Quat
{
public:

	friend class Transform;
	
	/////////////////////////////////////////////////////
	// Data Members
public:
	// (DO NOT MODIFY THESE DIRECTLY EVER!!!)
	float w;
	float x;
	float y;
	float z;

private:
	// To update an object's transform if this is modified
	Transform* mp_BoundTransform = nullptr;

	/////////////////////////////////////////////////////
	// Constants
public:
	// Identity Quaternion accessible from class
	static const Quat Identity;

	/////////////////////////////////////////////////////
	// Constructors
public:
	// Default constructor (Generates Identity Quaternion).
	inline Quat();

	// Member-Wise Constructor
	inline Quat(float InW, float InX, float InY, float InZ);

	// Copy Constructor
	inline Quat(const Quat& q);

	// Construct from Rotator
	explicit inline Quat(const Rotator& r);

	/**
	 * Creates and initializes a new quaternion from the a rotation around the given axis.
	 *
	 * @param Axis assumed to be a normalized vector
	 * @param Angle angle to rotate above the given axis (in radians)
	 */
	Quat(Vec3 Axis, float AngleRad);

	// Convert a vector of floating-point Euler angles (in degrees) into a Quaternion.
	static Quat MakeFromEuler(const Vec3& eulers);

	// Construct a quaternion from Euler angles (in degrees)
	inline static Quat MakeFromEuler(float x_pitch, float y_yaw, float z_roll);

	/////////////////////////////////////////////////////
	// Conversion Functions
public:
	// Convert this quaternion to floating point Euler angles (in degrees)
	Vec3 Euler() const;

	/**
	 * Convert a rotation into a unit vector facing in its direction.
	 * Equivalent to GetAxisX() and GetForwardVector()
	 */
	inline Vec3 Vector() const;

	/*
	 * Get a Rotator representation of this quaternion.
	 *
	 * While this conversion works in a static frame, it is recommended
	 * to use Quat::MakeFromEuler followed by Quat::operator* to perform
	 * rotations on GameObjects. Stay in quaternion space to prevent
	 * gymbal lock.
	 */
	Rotator GetRotator() const;

	// ONLY USE FOR RENDERING!!!
	Matrix3x3 Matrix() const;

	/////////////////////////////////////////////////////
	// Member Functions
public:

	// Assignment Operator
	inline Quat& operator=(const Quat& q);

	// Component-wise Operations. 
	// WARNING: Combining quaternions should be done by multiplication
	inline Quat& operator+=(const Quat& q);
	inline Quat& operator-=(const Quat& q);
	inline Quat  operator+(const Quat& q) const;
	inline Quat  operator-(const Quat& q) const;

	/**
	 * Gets the result of multiplying this by another quaternion (this * Q). 
	 *
	 * Order matters when composing quaternions: C = A * B will yield a 
	 * quaternion that logically first applies B then A to any subsequent 
	 * transformation (right first, then left).
	 *
	 * Note that (A * B) * C is equivalent to A * (B * C)
	 * [Associative] but not [Commutative]
	 */
	inline Quat  operator*(const Quat& q) const;
	inline Quat& operator*=(const Quat& q);

	// Quaternion scaling operations
	// Do not use unless you know what you're doing
	inline Quat  operator*(float scale) const;
	inline Quat& operator*=(float scale);
	inline Quat  operator/(float scale) const;
	inline Quat& operator/=(float scale);
	inline Quat  operator-() const;

	// Comparison operations
	inline bool Equals(const Quat& q, float tolerance = KINDA_SMALL_NUMBER) const;
	inline bool IsIdentity(float tolerance = SMALL_NUMBER) const;

	/** 
	 * Checks if two quaternions are identical. See Equals() for a 
	 * comparison that allows for an error tolerance
	 */
	inline bool operator==(const Quat& q) const;
	inline bool operator!=(const Quat& q) const;

	// Quaternion Inner Product
	inline float operator|(const Quat& q) const;

	// Normalize this quaternion if its large enough. Returns Identity if too small.
	inline void Normalize(float tolerance = SMALL_NUMBER);
	
	// Get normalized copy of this quat
	inline Quat GetNormalized(float tolerance = SMALL_NUMBER);

	// Returns True if this quaternion is Normalized
	bool IsNormalized() const;

	// Length of the quaternion
	inline float Size() const;
	inline float SizeSquared() const;

	// Get Axis and Angle of rotation of this quaternion
	inline void ToAxisAndAngle(Vec3& axis, float& angle);
	inline float GetAngle() const;
	inline Vec3 GetRotationAxis() const;

	// Returns a vector rotated by this quaternion.
	inline Vec3 RotateVector(Vec3 v) const;

	// Returns a vector rotated by the inverse of this quaternion
	inline Vec3 UnrotateVector(Vec3 v) const;

	// Get the inverse of this quaternion (the inverse rotation). This quat must be normalized.
	inline Quat Inverse() const;

	/*
	 * Enforce that the delta between this quat and another represents 
	 * the shortest possible rotation angle.
	 */
	inline void EnforceShortestArcWith(const Quat& q);
	
	/*
     *       Y (up)
	 *       |
	 *       |
	 *       |
	 *       ------ X (forward)
	 *      /
	 *     /
	 *    /
	 *   Z (right)
	 */

	// Get the Forward direction (X-axis) after it has been rotated by this quaternion
	inline Vec3 GetAxisX() const;

	// Get the Forward direction (X-axis) after it has been rotated by this quaternion
	inline Vec3 GetForwardVector() const;

	// Get the Up direction (Y-axis) after it has been rotated by this quaternion
	inline Vec3 GetAxisY() const;
	
	// Get the Up direction (Y-axis) after it has been rotated by this quaternion
	inline Vec3 GetUpVector() const;

	// Get the Right direction (Z-axis) after it has been rotated by this quaternion
	inline Vec3 GetAxisZ() const;

	// Get the Right direction (Z-axis) after it has been rotated by this quaternion
	inline Vec3 GetRightVector() const;

	// Get the angular distance between this and another quat (in radians)
	inline float AngularDistance(const Quat& q) const;

	/**
	 * Generates the smallest geodesic rotation between 2 vectors of arbitrary length.
	 * Equivalent to FindBetweenVectors().
	 */
	static inline Quat FindBetween(const Vec3& v1, const Vec3& v2);

	// Generates the smallest geodesic rotation between 2 vectors of arbitrary length
	static inline Quat FindBetweenVectors(const Vec3& v1, const Vec3& v2);

	/**
	 * Generates the smallest geodesic rotation between 2 vectors of unit length (This is assumed).
	 * Use this if you know the vectors are normalized to speed up computation.
	 */
	static inline Quat FindBetweenNormals(const Vec3& v1, const Vec3& v2);
	
	/*
	 * Spherical Interpolation. Will correct alignment. 
	 * Input must be normalized. Result is normalized.
	 */
	static inline Quat Slerp(const Quat& q1, const Quat& q2, float t);

	/////////////////////////////////////////////////////
	// For Debugging

	// Converts value of quaternion into human-readable string
	std::string ToString() const;
	
	// Check that no NaN values in quaternion
	inline void DiagnosticCheckNaN() const;
	inline bool ContainsNaN() const;

private:
	/////////////////////////////////////////////////////
	// Helper Functions

	static inline Quat FindBetween_Helper(const Vec3& A, const Vec3& B, float normAB);
	static Quat Slerp_NotNormalized(const Quat& q1, const Quat& q2, float t);

	// Preserve correctness of game object's transform
	void UpdateBoundTransform();
};


// ------------------------- INLINE IMPLEMENTATIONS ---------------------

/** Default constructor (Generates Identity Quaternion). */
inline Quat::Quat()
	: w(1.0f), x(0), y(0), z(0)
{}

// Constructor
inline Quat::Quat(float InW, float InX, float InY, float InZ)
	: w(InW), x(InX), y(InY), z(InZ)
{}

inline Quat::Quat(const Quat& q)
	: x(q.x), y(q.y), z(q.z), w(q.w)
{}

inline Quat::Quat(const Rotator & r)
{
	*this = r.Quaternion();
}

inline Quat Quat::MakeFromEuler(float x_pitch, float y_yaw, float z_roll)
{
	return Rotator{ x_pitch, y_yaw, z_roll }.Quaternion();
}

inline Quat& Quat::operator=(const Quat& q)
{
	x = q.x;
	y = q.y;
	z = q.z;
	w = q.w;
	UpdateBoundTransform();
	// Do not shallow copy because mp_BoundRotator is unique
	return *this;
}

inline Quat& Quat::operator+=(const Quat& q)
{
	x += q.x;
	y += q.y;
	z += q.z;
	w += q.w;
	DiagnosticCheckNaN();
	UpdateBoundTransform();
	return *this;
}

inline Quat& Quat::operator-=(const Quat& q)
{
	x -= q.x;
	y -= q.y;
	z -= q.z;
	w -= q.w;
	DiagnosticCheckNaN();
	UpdateBoundTransform();
	return *this;
}

inline Quat Quat::operator+(const Quat& q) const
{
	Quat r { w + q.w, x + q.x, y + q.y, z + q.z };
	r.DiagnosticCheckNaN();
	return r;
}

inline Quat Quat::operator-(const Quat& q) const
{
	Quat r { w - q.w, x - q.x, y - q.y, z - q.z };
	r.DiagnosticCheckNaN();
	return r;
}

inline Quat Quat::operator*(const Quat& q) const
{
	// Hamilton Product
	Quat r{
		w * q.w - x * q.x - y * q.y - z * q.z,
		w * q.x + x * q.w + y * q.z - z * q.y,
		w * q.y - x * q.z + y * q.w + z * q.x,
		w * q.z + x * q.y - y * q.x + z * q.w
	};
	r.DiagnosticCheckNaN();
	return r;
}

inline Quat& Quat::operator*=(const Quat& q)
{
	*this = *this * q;
	DiagnosticCheckNaN();
	UpdateBoundTransform();
	return *this;
}

inline Quat Quat::operator*(float scale) const
{
	return Quat(w * scale, x * scale, y * scale, z * scale);
}

inline Quat& Quat::operator*=(float scale)
{
	w *= scale;
	x *= scale;
	y *= scale;
	z *= scale;

	DiagnosticCheckNaN();
	UpdateBoundTransform();
	return *this;
}

inline Quat Quat::operator/(float scale) const
{
	return Quat(w / scale, x / scale, y / scale, z / scale);
}

inline Quat& Quat::operator/=(float scale)
{
	w /= scale;
	x /= scale;
	y /= scale;
	z /= scale;
	DiagnosticCheckNaN();
	UpdateBoundTransform();
	return *this;
}

inline Quat Quat::operator-() const
{
	return Quat{ -w,-x,-y,-z };
}

inline bool Quat::Equals(const Quat& q, float tolerance) const
{
	return (
		Math::FloatEqual(w, q.w, tolerance) &&
		Math::FloatEqual(x, q.x, tolerance) &&
		Math::FloatEqual(y, q.y, tolerance) &&
		Math::FloatEqual(z, q.z, tolerance)
		);
}

inline bool Quat::IsIdentity(float tolerance) const
{
	return Equals(Quat::Identity, tolerance);
}

inline bool Quat::operator==(const Quat& q) const
{
	return (
		x == q.x &&
		y == q.y &&
		z == q.z &&
		w == q.w
	);
}

inline bool Quat::operator!=(const Quat& q) const
{
	return !operator==(q);
}

// Dot Product
inline float Quat::operator|(const Quat& q) const
{
	return w * q.w + x * q.x + y * q.y + z * q.z;
}


inline void Quat::Normalize(float tolerance)
{
	const float squareSum = w * w + x * x + y * y + z * z;
	if (squareSum >= tolerance)
	{
		const float scale = Math::InvSqrt(squareSum);
		w *= scale;
		x *= scale;
		y *= scale;
		z *= scale;
	}
	else
	{
		*this = Quat::Identity;
	}
}

inline Quat Quat::GetNormalized(float tolerance)
{
	Quat r{ *this };
	r.Normalize(tolerance);
	return r;
}

inline float Quat::Size() const
{
	return sqrtf(w * w + x * x + y * y + z * z);
}

inline float Quat::SizeSquared() const
{
	return w * w + x * x + y * y + z * z;
}

inline void Quat::ToAxisAndAngle(Vec3& axis, float& angle)
{
	axis = GetRotationAxis();
	angle = GetAngle();
}

inline float Quat::GetAngle() const
{
	return 2.f * acosf(w);
}

inline Vec3 Quat::GetRotationAxis() const
{
	// Calculating sin value of angle. 
	// Also, ensure we never sqrt a negative number
	const float s = sqrtf(Math::Max(1.f - (w * w), 0.f));

	if (s > KINDA_SMALL_NUMBER)
		return Vec3{ -y / s, z / s, -x / s }; // Old: x / s, y / s, z / s
	else
		return Vec3{ 1.f, 0.f, 0.f };
}

inline Vec3 Quat::RotateVector(Vec3 V) const
{
	// http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
	// V' = V + 2w(Q x V) + (2Q x (Q x V))
	// refactor:
	// V' = V + w(2(Q x V)) + (Q x (2(Q x V)))
	// T = 2(Q x V);
	// V' = V + w*(T) + (Q x T)

	const Vec3 Q(x, y, z);
	const Vec3 T = 2.f * (Q ^ V);
	return V + (w * T) + (Q ^ T);
}

inline Vec3 Quat::UnrotateVector(Vec3 V) const
{
	// Same formula RotateVector
	const Vec3 Q(-x, -y, -z); // Inverse quat
	const Vec3 T = 2.f * (Q ^ V);
	return V + (w * T) + (Q ^ T);
}

inline Quat Quat::Inverse() const
{
	if (IsNormalized())
		return Quat{ w, -x, -y, -z };
	else
		return Quat::Identity; // Non-normalized quats not supported.
}

inline void Quat::EnforceShortestArcWith(const Quat& q)
{
	const float dotResult = *this | q;
	const float bias = (dotResult >= 0.f) ? 1.f : -1.f;
	w *= bias;
	x *= bias;
	y *= bias;
	z *= bias;
}

inline Vec3 Quat::GetAxisX() const
{
	return RotateVector(Vec3{ 1.f, 0.f, 0.f });
}

inline Vec3 Quat::GetForwardVector() const
{
	return GetAxisX();
}

inline Vec3 Quat::GetAxisY() const
{
	return RotateVector(Vec3{ 0.f, 1.f, 0.f });
}

inline Vec3 Quat::GetUpVector() const
{
	return GetAxisY();
}

inline Vec3 Quat::GetAxisZ() const
{
	return RotateVector(Vec3{ 0.f, 0.f, 1.f });
}

inline Vec3 Quat::GetRightVector() const
{
	return GetAxisZ();
}

inline Vec3 Quat::Vector() const
{
	return GetAxisX();
}

inline float Quat::AngularDistance(const Quat& q) const
{
	float innerProduct = *this | q;
	return acosf((2 * innerProduct * innerProduct) - 1.f);
}

inline Quat Quat::FindBetween(const Vec3& v1, const Vec3& v2)
{
	return FindBetweenVectors(v1, v2);
}

inline Quat Quat::FindBetweenVectors(const Vec3& v1, const Vec3& v2)
{
	return FindBetween_Helper(v1, v2, sqrtf(v1.MagnitudeSq() * v2.MagnitudeSq()));
}

inline Quat Quat::FindBetweenNormals(const Vec3& v1, const Vec3& v2)
{
	return FindBetween_Helper(v1, v2, 1.f);
}

inline Quat Quat::FindBetween_Helper(const Vec3& A, const Vec3& B, float normAB)
{
	/**
	 * Solution adapted from:
	 * http://lolengine.net/blog/2014/02/24/quaternion-from-two-vectors-final
	 * http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/index.htm
	 */

	float result_w = normAB + A * B;
	Quat result;

	if (result_w >= KINDA_SMALL_NUMBER * normAB)
	{
		// Calculate quat normally using cross product
		result = Quat{
			result_w,
			A.y * B.z - A.z * B.y,
			A.z * B.x - A.x * B.z,
			A.x * B.y - A.y * B.x
		};
	}
	else
	{
		// A and B are exactly opposite; So we generate a rotation that is
		// 180 degrees around an arbitrary axis.
		result = (fabsf(A.x) > fabsf(B.x))
			? Quat{ 0.f, -A.z, 0.f, A.x }
			: Quat{ 0.f, 0.f, -A.z, A.y };
	}

	result.Normalize();
	return result;
}

inline Quat Quat::Slerp(const Quat& q1, const Quat& q2, float t)
{
	return Slerp_NotNormalized(q1, q2, t).GetNormalized();
}

inline std::string Quat::ToString() const
{
	std::ostringstream oss;
	oss << "w=" << w << " x=" << x << " y=" << y << " z=" << z;
	return oss.str();
}

#if ENABLE_NAN_CHECK
inline void Quat::DiagnosticCheckNaN() const
{
	if (ContainsNaN())
	{
		std::cout << "Quat contains NaN: " << ToString() << std::endl;
		*const_cast<Quat*>(this) = Quat::Identity;
	}
}
#else
inline void Quat::DiagnosticCheckNaN() const {}
#endif

inline bool Quat::ContainsNaN() const
{
	return (!Math::IsFinite(x) ||
		!Math::IsFinite(y) ||
		!Math::IsFinite(z) ||
		!Math::IsFinite(w)
		);
}

} // namespace SNova
#pragma once
#include "GenMath.h"
#include "glm.hpp"

namespace SNova
{

#ifdef _MSC_VER
// Supress warning: nonstandard extension used : nameless struct/union
#pragma warning( disable : 4201 )
#endif

class Vector3D
{
public:
	union
	{
		struct { float x, y, z; };
		float v[3];
	};

	//Constructors
	Vector3D(float x = 0.0f, float y = 0.0f, float z = 0.0f);
	Vector3D(const glm::vec3& vec) : x{ vec.x }, y{ vec.y }, z{ vec.z } {};
	explicit Vector3D(const Vec4& v4);

	//Assignment operators
	Vector3D& operator +=(const Vector3D& rhs);
	Vector3D& operator -=(const Vector3D& rhs);
	Vector3D& operator *=(float rhs);
	Vector3D& operator /=(float rhs);

	operator glm::vec3() const { return glm::vec3{ x,y,z }; }

	//Unary operator
	Vector3D operator -() const;

	//Other operations
	void Normalize();
	Vector3D Normalized() const;
	float Magnitude() const;
	float MagnitudeSq() const;

	// Converts value of Vector into human-readable string
	inline std::string ToString() const;

	// Checks whether 2 vectors are equal with specified tolerance
	inline bool Equals(const Vector3D& v, float tolerance = KINDA_SMALL_NUMBER) const;
};

#ifdef _MSC_VER
// Supress warning: nonstandard extension used : nameless struct/union
#pragma warning( disable : 4201 )
#endif

// Type Aliases
typedef Vector3D Vec3;
typedef Vector3D Point3D;
typedef Vector3D Pt3;

//Binary operators
Vector3D operator + (const Vector3D& lhs, const Vector3D& rhs);
Vector3D operator - (const Vector3D& lhs, const Vector3D& rhs);
Vector3D operator * (const Vector3D& lhs, float rhs);
Vector3D operator * (float lhs, const Vector3D& rhs);
Vector3D operator / (const Vector3D& lhs, float rhs);
bool operator == (const Vector3D& lhs, const Vector3D& rhs);
bool operator != (const Vector3D& lhs, const Vector3D& rhs);

// Dot Product
float operator*(const Vec3& lhs, const Vec3& rhs);

// Cross Product
Vec3 operator^(const Vec3& lhs, const Vec3& rhs);

//Zeroes out vector passed in
void ZeroVector(Vector3D& pResult);
//Negate the vector passed in
void NegateVector(Vector3D& pResult);
//Normalize vector 
void NormalizeVector3D(Vector3D& pResult, const Vector3D& pVec0);
//Get vector length
float Vector3DLength(const Vector3D& pVec0);
//Get vector length squared
float Vector3DLengthSquared(const Vector3D& pVec0);
//Get distance between two points
float Vector3DDistance(const Vector3D& pVec0, const Vector3D& pVec1);
//Get squared distance between two points
float Vector3DDistanceSquared(const Vector3D& pVec0, const Vector3D& pVec1);
//Dot product
float Vector3DDotProduct(const Vector3D& pVec0, const Vector3D& pVec1);
//Cross product magnitude
float Vector3DCrossProductMag(const Vector3D& pVec0, const Vector3D& pVec1);
//Get angle between vectors (in radians)
float Vector3DAngle(const Vector3D& pVec0, const Vector3D& pVec1);
//Check if vectors are parallel
bool Vector3DParallel(const Vector3D& pVec0, const Vector3D& pVec1);

// ------------------------- INLINE IMPLEMENTATIONS ---------------------

inline std::string Vector3D::ToString() const
{
	std::ostringstream oss;
	oss << "x=" << x << " y=" << y << " z=" << z;
	return oss.str();
}

inline bool Vector3D::Equals(const Vector3D& r, float tolerance) const
{
	return (fabsf(x - r.x) <= tolerance)
		&& (fabsf(y - r.y) <= tolerance)
		&& (fabsf(z - r.z) <= tolerance);
}

} // namespace SNova
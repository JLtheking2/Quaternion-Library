#include "SNova.h"
#include "Vector4D.h"

namespace SNova
{
/*********************************
*****Vector3D class functions*****
**********************************/
Vector3D::Vector3D(float _x, float _y, float _z)
	:x(_x), y(_y), z(_z)
{
}

Vector3D::Vector3D(const Vec4& v4) 
	: x{ v4.x }, y{ v4.y }, z{ v4.z }
{}

Vector3D& Vector3D::operator+=(const Vector3D& rhs)
{
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;

	return *this;
}

Vector3D& Vector3D::operator-=(const Vector3D& rhs)
{
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;

	return *this;
}

Vector3D& Vector3D::operator*=(float rhs)
{
	x *= rhs;
	y *= rhs;
	z *= rhs;

	return *this;
}

Vector3D& Vector3D::operator/=(float rhs)
{
	x /= rhs;
	y /= rhs;
	z /= rhs;

	return *this;
}

Vector3D Vector3D::operator-() const
{
	return Vector3D{ -x, -y, -z };
}

void Vector3D::Normalize()
{
	NormalizeVector3D(*this, *this);
}

Vector3D Vector3D::Normalized() const
{
	Vector3D normVec = *this;
	NormalizeVector3D(normVec, *this);
	return normVec;
}

float Vector3D::Magnitude() const
{
	return Vector3DLength(*this);
}

float Vector3D::MagnitudeSq() const
{
	return Vector3DLengthSquared(*this);
}
/*********************************
*End of Vector3D class functions**
**********************************/

/*********************************
**Vector3D non-member functions***
**********************************/

Vector3D operator+(const Vector3D& lhs, const Vector3D& rhs)
{
	Vector3D temp{ lhs.x, lhs.y, lhs.z };
	temp += rhs;

	return temp;
}

Vector3D operator-(const Vector3D& lhs, const Vector3D& rhs)
{
	Vector3D temp{ lhs.x, lhs.y, lhs.z };
	temp -= rhs;

	return temp;
}

Vector3D operator*(const Vector3D& lhs, float rhs)
{
	return Vector3D{ lhs.x * rhs, lhs.y * rhs, lhs.z * rhs };
}

Vector3D operator*(float lhs, const Vector3D& rhs)
{
	return operator*(rhs, lhs);
}

Vector3D operator/(const Vector3D& lhs, float rhs)
{
	return Vector3D{ lhs.x / rhs, lhs.y / rhs, lhs.z / rhs };
}

float operator*(const Vec3& lhs, const Vec3& rhs)
{
	return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Vec3 operator^(const Vec3& lhs, const Vec3& rhs)
{
	return Vec3{ lhs.y * rhs.z - rhs.y * lhs.z,
				 lhs.z * rhs.x - rhs.z * lhs.x,
				 lhs.x * rhs.y - rhs.x * lhs.y };
}

bool operator==(const Vector3D& lhs, const Vector3D& rhs)
{
	return (fabsf(lhs.x - rhs.x) < EPSILON && fabsf(lhs.y - rhs.y) < EPSILON && fabsf(lhs.z - rhs.z) < EPSILON);
}

bool operator!=(const Vector3D& lhs, const Vector3D& rhs)
{
	return !operator==(lhs, rhs);
}

void ZeroVector(Vector3D& pResult)
{
	pResult.x = 0.0f;
	pResult.y = 0.0f;
	pResult.z = 0.0f;
}

void NegateVector(Vector3D& pResult)
{
	pResult.x = -pResult.x;
	pResult.y = -pResult.y;
	pResult.z = -pResult.z;
}

void NormalizeVector3D(Vector3D& pResult, const Vector3D& pVec0)
{
	// magnitude of pVec0
	float magnitudeSq = Vector3DLengthSquared(pVec0);

	// prevent division by zero
	if (Approximate(magnitudeSq, 0.000f)) return;

	magnitudeSq = FastInverseSqrt(magnitudeSq);

	// divide each coordinate by magnitude
	pResult.x = pVec0.x * magnitudeSq;
	pResult.y = pVec0.y * magnitudeSq;
	pResult.z = pVec0.z * magnitudeSq;
}

float Vector3DLength(const Vector3D& pVec0)
{
	return Sqrt(pVec0.x * pVec0.x + pVec0.y * pVec0.y + pVec0.z * pVec0.z);
}

float Vector3DLengthSquared(const Vector3D& pVec0)
{
	return (pVec0.x * pVec0.x + pVec0.y * pVec0.y + pVec0.z * pVec0.z);
}

float Vector3DDistance(const Vector3D& pVec0, const Vector3D& pVec1)
{
	return sqrt(Vector3DDistanceSquared(pVec0, pVec1));
}

float Vector3DDistanceSquared(const Vector3D& pVec0, const Vector3D& pVec1)
{
	float xDiff = pVec0.x - pVec1.x;
	float yDiff = pVec0.y - pVec1.y;
	float zDiff = pVec0.z - pVec1.z;

	return (xDiff * xDiff) + (yDiff * yDiff) + (zDiff * zDiff);
}

float Vector3DDotProduct(const Vector3D& pVec0, const Vector3D& pVec1)
{
	return (pVec0.x * pVec1.x) + (pVec0.y * pVec1.y) + (pVec0.z * pVec1.z);
}

float Vector3DCrossProductMag(const Vector3D& pVec0, const Vector3D& pVec1)
{
	return (Vector3DLength(pVec0) * Vector3DLength(pVec1));
}

float Vector3DAngle(const Vector3D& pVec0, const Vector3D& pVec1)
{
	float dot = Vector3DDotProduct(pVec0, pVec1);
	float det = Vector3DCrossProductMag(pVec0, pVec1);
	return acosf(dot / det);

	// This gives you counterclockwise angle from v0 to v1
	//float angle = -atan2f(det, dot);	// gives -PI to PI
	//if (angle < 0.0f)
	//	angle = (2.0f * PI) + angle;
}

bool Vector3DParallel(const Vector3D& pVec0, const Vector3D& pVec1)
{
	return Approximate(fabsf(pVec0.x), fabsf(pVec1.x)) && 
		   Approximate(fabsf(pVec0.y), fabsf(pVec1.y)) &&
		   Approximate(fabsf(pVec0.z), fabsf(pVec1.z));
}

}
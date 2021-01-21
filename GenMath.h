#pragma once

#pragma warning(push)
#pragma warning(disable : 4201)
#include <glm.hpp>
#pragma warning(pop)

#include <sstream>
#include <iomanip>

namespace SNova
{
	/*-----------------------------------------------------------------------------
		Definitions.
	-----------------------------------------------------------------------------*/

	// Forward Declarations
	class Vector3D;
	class Vector4D;
	struct Matrix3x3;
	union  Matrix4x4;
	struct Quat;
	struct Rotator;

	// Type Aliases
	typedef Vector3D Vec3;
	typedef Vector3D Point3D;
	typedef Vector3D Pt3;
	typedef Vector4D Vec4;
	typedef Vector4D Point4D;
	typedef Vector4D Pt4;

	/*-----------------------------------------------------------------------------
	Flags
	-----------------------------------------------------------------------------*/

	#define ENABLE_NAN_CHECK 0

	/*-----------------------------------------------------------------------------
	Floating point constants.
	-----------------------------------------------------------------------------*/

	#define SMALL_NUMBER (1.e-8f)
	#define KINDA_SMALL_NUMBER (1.e-4f)

	// actual PI value, up to 10 d.p.
	static constexpr float PI = 3.1415926535f;
	static constexpr float PI2 = PI * 2.f;
	static constexpr float PIOVER2 = PI / 2.f;

	// a small floating point value
	static constexpr float EPSILON = KINDA_SMALL_NUMBER;

	static constexpr float PIOVER180 = PI / 180;
	static constexpr float OVERPI = 180 / PI;

	static constexpr float easeBounce_Cond1 = 1.0f / 2.75f;
	static constexpr float easeBounce_Cond2 = 2.0f / 2.75f;
	static constexpr float easeBounce_Cond3 = 2.5f / 2.75f;
	static constexpr float easeBounce_t1 = 1.5f / 2.75f;
	static constexpr float easeBounce_t2 = 2.25f / 2.75f;
	static constexpr float easeBounce_t3 = 2.625f / 2.75f;

	/*-----------------------------------------------------------------------------
		MACRO FUNCTIONS
	-----------------------------------------------------------------------------*/

	#define DEG2RAD(deg) (deg * PIOVER180)
	#define RAD2DEG(RAD) (RAD * OVERPI)

	/*-----------------------------------------------------------------------------
		Global functions.
	-----------------------------------------------------------------------------*/

	// convert from degrees to radians
	float DegToRad(float degrees);

	// convert from radians to degrees
	float RadToDeg(float radians);

	// computes the inverse squareroot fast
	// returns 1 / sqrt(number)
	float FastInverseSqrt(float number);
	float Sqrt(float number);

	// compare two floating point values and see if they are about the same
	bool Approximate(float a, float b);

	// ensure that value is between min and max
	float Clamp(float min, float max, float value);

	glm::mat4 ConvertMtx44(const Matrix4x4& mtx);

	// when t = 0, a is returned. when t = 1, b is returned
	template <typename T> T Lerp(const T& a, const T& b, const float& t);

	template <typename T> T Lerp(const T& a, const T& b, const float& t)
	{
		return (1.0f - t) * a + t * b;
	}

	// ------------------------- Justin's Math Library ---------------------
	namespace Math
	{

		inline bool IsFinite(float x) { return _finitef(x) != 0; }
		inline bool IsNaN(float x) { return _isnanf(x) != 0; }

		inline void SinCos(float rad, float& sinOut, float& cosOut);
		inline bool FloatEqual(float x, float y, float tolerance = KINDA_SMALL_NUMBER);

		// Fast Inverse Square Root
		inline float InvSqrt(float num);

		template <typename T>
		constexpr inline bool InRange(T min, T max, T value);

		template <typename T>
		constexpr inline T Max(const T A, const T B) { return (A >= B) ? A : B; }

		template <typename T>
		constexpr inline T Min(const T A, const T B){ return (A <= B) ? A : B; }

		// ------------------------- INLINE / TEMPLATE IMPLEMENTATIONS ---------------------

		inline void SinCos(float rad, float& sinOut, float& cosOut)
		{
			sinOut = sinf(rad);
			cosOut = cosf(rad);
		}

		inline bool FloatEqual(float x, float y, float tolerance)
		{
			return fabs(x - y) < tolerance;
		}

		inline float InvSqrt(float num)
		{
			union {
				float f;
				uint32_t i;
			} conv;

			float x2;
			const float threehalfs = 1.5F;

			x2 = num * 0.5F;
			conv.f = num;
			conv.i = 0x5f3759df - (conv.i >> 1);
			conv.f = conv.f * (threehalfs - (x2 * conv.f * conv.f));
			return conv.f;
		}

		template <typename T>
		constexpr inline bool InRange(T min, T max, T value)
		{
			return value >= min && value <= max;
		}

	}
}
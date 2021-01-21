#include "SNova.h"

namespace SNova
{
	const float DEG_2_RAD = PI / 180.0f;
	const float RAD_2_DEG = 180.0f / PI;

	float DegToRad(float degrees)
	{
		return degrees * DEG_2_RAD;
	}

	float RadToDeg(float radians)
	{
		return radians * RAD_2_DEG;
	}

	float FastInverseSqrt(float number)
	{
		//Source: https://en.wikipedia.org/wiki/Fast_inverse_square_root
		
		/*long i;
		float x2, y;
		const float threehalfs = 1.5f;

		x2 = number * 0.5f;
		y = number;
		i = *(long*)& y;
		i = 0x5f3759df - (i >> 1);
		y = *(float*)& i;
		y = y * (threehalfs - (x2 * y * y));

		return y;*/
		

		////https://arxiv.org/pdf/1802.06302v1.pdf
		//float halfnumber = 0.5f * number;
		//int i = *(int*)& number;
		//i = 0x5F376908 - (i >> 1);
		//number = *(float*)& i;
		//number = number * (1.5008789f - halfnumber * number * number);
		////number = number * (1.5000006f - halfnumber * number * number);
		//return number;
		return 1.0f / sqrtf(number);
	}

	float Sqrt(float number)
	{
		//return 1.0f / FastInverseSqrt(number);
		return sqrtf(number);
	}

	bool Approximate(float a, float b)
	{
		if ((a - EPSILON) > b) return false;
		if ((a + EPSILON) < b) return false;
		return true;
	}

	float Clamp(float min, float max, float value)
	{
		if (value < min) return min;
		if (value > max) return max;
		return value;
	}

	glm::mat4 ConvertMtx44(const Matrix4x4& mtx)
	{
		glm::mat4 transform;
		transform[0] = glm::vec4{ mtx.m00, mtx.m10, mtx.m20, mtx.m30 };
		transform[1] = glm::vec4{ mtx.m01, mtx.m11, mtx.m21, mtx.m31 };
		transform[2] = glm::vec4{ mtx.m02, mtx.m12, mtx.m22, mtx.m32 };
		transform[3] = glm::vec4{ mtx.m03, mtx.m13, mtx.m23, mtx.m33 };
		return transform;
	}
}
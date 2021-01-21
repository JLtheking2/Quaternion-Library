#include "Matrix3x3.h"
#include "Vector3D.h"

namespace SNova
{


Matrix3x3::Matrix3x3(const float * pArr)
	: m2{}
{
	int x = 0;

	for (unsigned i = 0; i < 3; ++i)
	{
		for (unsigned j = 0; j < 3; ++j, ++x)
		{
			m2[i][j] = *(pArr + x);
		}
	}
}

Matrix3x3::Matrix3x3(float _00, float _01, float _02, float _10, float _11, float _12, float _20, float _21, float _22)
: m2{ { _00, _01, _02 },{ _10, _11, _12 },{ _20, _21, _22 } }
{}

Matrix3x3 & Matrix3x3::operator=(const Matrix3x3 & rhs)
{
	//copy over values

	for (unsigned i = 0; i < 3; ++i)
	{
		for (unsigned j = 0; j < 3; ++j)
		{
			m2[i][j] = rhs.m2[i][j];
		}
	}

	return *this;
}

Matrix3x3 operator*(const Matrix3x3 & lhs, const Matrix3x3 & rhs)
{
	Matrix3x3 m{ 0,0,0,0,0,0,0,0,0 };
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			for (int k = 0; k < 3; ++k)
				m.m2[i][j] += lhs.m2[i][k] * rhs.m2[k][j];
	return m;
}

Matrix3x3 & Matrix3x3::operator*=(const Matrix3x3 & rhs)
{
	Matrix3x3 tmp = *this;

	tmp = tmp * rhs;

	*this = tmp;

	return *this;

}

Matrix3x3& Matrix3x3::operator+=(const Matrix3x3 rhs)
{
	for (int i = 0; i < 3; ++i)
	{ 
		for (int j = 0; j < 3; ++j)
			m2[i][j] += rhs.m2[i][j];
	}
	return *this;
}

Matrix3x3& Matrix3x3::operator-=(const Matrix3x3 rhs)
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
			m2[i][j] -= rhs.m2[i][j];
	}
	return *this;
}

void Matrix3x3::PrintMatrix3x3()
{
	int x = 0;

	for (unsigned i = 0; i < 3; ++i)
	{
		for (unsigned j = 0; j < 3; ++j, ++x)
		{
			std::cout << m2[i][j] << " , ";
		}
		std::cout << std::endl;
	}
}

inline float Matrix3x3::Determinant() const
{
	return	  m2[0][0] * (m2[1][1] * m2[2][2] - m2[1][2] * m2[2][1])
		- m2[0][1] * (m2[1][0] * m2[2][2] - m2[1][2] * m2[2][0])
		+ m2[0][2] * (m2[1][0] * m2[2][1] - m2[1][1] * m2[2][0]);
}

Matrix3x3 Matrix3x3::GetInverse()
{
	return Mtx33Inverse(*this);
}

Matrix3x3 Matrix3x3::GetTranspose()
{
	Matrix3x3 res;
	Mtx33Transpose(res, *this);
	return res;
}

std::string Matrix3x3::ToString() const
{
	std::ostringstream oss;
	oss << '[' << m2[0][0] << ' ' << m2[0][1] << ' ' << m2[0][2] << "] ";
	oss << '[' << m2[1][0] << ' ' << m2[1][1] << ' ' << m2[1][2] << "] ";
	oss << '[' << m2[2][0] << ' ' << m2[2][1] << ' ' << m2[2][2] << "] ";
	return oss.str();
}



Matrix3x3 operator+(const Matrix3x3 & lhs, const Matrix3x3 & rhs)
{
	Matrix3x3 result(lhs);
	result += rhs;
	return result;
}

Matrix3x3 operator-(const Matrix3x3 & lhs, const Matrix3x3 & rhs)
{
	Matrix3x3 result(lhs);
	result -= rhs;
	return result;
}

Vec3 operator*(const Matrix3x3& pMtx, const Vec3& rhs)
{
	Vec3 result;
	result.x = pMtx.m2[0][0] * rhs.x + pMtx.m2[1][0] * rhs.y + pMtx.m2[2][0] * rhs.z;
	result.y = pMtx.m2[0][1] * rhs.x + pMtx.m2[1][1] * rhs.y + pMtx.m2[2][1] * rhs.z;
	result.z = pMtx.m2[0][2] * rhs.x + pMtx.m2[1][2] * rhs.y + pMtx.m2[2][2] * rhs.z;
	/*result.x = pMtx.m2[0][0] * rhs.x + pMtx.m2[0][1] * rhs.y + pMtx.m2[0][2] * rhs.z;
	result.y = pMtx.m2[1][0] * rhs.x + pMtx.m2[1][1] * rhs.y + pMtx.m2[1][2] * rhs.z;
	result.z = pMtx.m2[2][0] * rhs.x + pMtx.m2[2][1] * rhs.y + pMtx.m2[2][2] * rhs.z;
	*/
	return result;
}

void Mtx33Identity(Matrix3x3 & pResult)
{
	// set identity matrix values
	/*
	1 0 0
	0 1 0
	0 0 1
	*/
	

	for (unsigned i = 0; i < 3; ++i)
	{
		for (unsigned j = 0; j < 3; ++j)
		{
			pResult.m2[i][j] = 0.0f;
		}
	}
	// Sets the required points to 1 to form identity matrix
	pResult.m2[0][0] = pResult.m2[1][1] = pResult.m2[2][2] = 1.0f;

}

void Mtx33Translate(Matrix3x3 & pResult, float x, float y)
{
	/*
	1 0 x
	0 1 y
	0 0 1
	*/
	Mtx33Identity(pResult);
	pResult.m2[0][2] = x;
	pResult.m2[1][2] = y;
}

void Mtx33Scale(Matrix3x3 & pResult, float x, float y)
{
	/*
	x 0 0
	0 y 0
	0 0 1
	*/
	Mtx33Identity(pResult);
	pResult.m2[0][0] = x;
	pResult.m2[1][1] = y;
}

void Mtx33RotRad(Matrix3x3 & pResult, float angle)
{
	/*
	cos -sin 0
	sin  cos 0
	0    0  1
	*/
	Mtx33Identity(pResult);
	pResult.m2[0][0] = cosf(angle);
	pResult.m2[0][1] = -(sinf(angle));
	pResult.m2[1][0] = sinf(angle);
	pResult.m2[1][1] = cosf(angle);

}

void Mtx33RotDeg(Matrix3x3 & pResult, float angle)
{
	/*
	cos -sin 0
	sin  cos 0
	0    0  1
	*/

	//convert to radians
	angle = static_cast<float>(angle * PI / 180);

	Mtx33Identity(pResult);
	pResult.m2[0][0] = cosf(angle);
	pResult.m2[0][1] = -(sinf(angle));
	pResult.m2[1][0] = sinf(angle);
	pResult.m2[1][1] = cosf(angle);


}

void Mtx33Transpose(Matrix3x3 & pResult, const Matrix3x3 & pMtx)
{
	//flip matrix to the side
	pResult.m2[0][0] = pMtx.m2[0][0];
	pResult.m2[0][1] = pMtx.m2[1][0];
	pResult.m2[0][2] = pMtx.m2[2][0];
		   
	pResult.m2[1][0] = pMtx.m2[0][1];
	pResult.m2[1][1] = pMtx.m2[1][1];
	pResult.m2[1][2] = pMtx.m2[2][1];
		   
	pResult.m2[2][0] = pMtx.m2[0][2];
	pResult.m2[2][1] = pMtx.m2[1][2];
	pResult.m2[2][2] = pMtx.m2[2][2];
}

Matrix3x3 Mtx33Inverse(const Matrix3x3 & pMtx)
{
	//get determinant
	float determinant = pMtx.Determinant();
	if (determinant == 0)//if determinant is zero the matrix cannot be inversed
	{
		return pMtx;
	}

	//get adjoint
	Matrix3x3 Adj(
		pMtx.m2[1][1] *pMtx.m2[2][2] - pMtx.m2[1][2] *pMtx.m2[2][1], -(pMtx.m2[1][0] *pMtx.m2[2][2] - pMtx.m2[1][2] *pMtx.m2[2][0]), pMtx.m2[1][0] *pMtx.m2[2][1] - pMtx.m2[1][1] *pMtx.m2[2][0],
		-(pMtx.m2[0][1] *pMtx.m2[2][2] - pMtx.m2[0][2] *pMtx.m2[2][1]), pMtx.m2[0][0] *pMtx.m2[2][2] - pMtx.m2[0][2] *pMtx.m2[2][0], -(pMtx.m2[0][0] *pMtx.m2[2][1] - pMtx.m2[0][1] *pMtx.m2[2][0]),
		pMtx.m2[0][1] *pMtx.m2[1][2] - pMtx.m2[0][2] *pMtx.m2[1][1], -(pMtx.m2[0][0] *pMtx.m2[1][2] - pMtx.m2[0][2] *pMtx.m2[1][0]), pMtx.m2[0][0] *pMtx.m2[1][1] - pMtx.m2[0][1] *pMtx.m2[1][0]);
	auto temp = Adj;
	Mtx33Transpose(Adj, temp);

	//get inverse
	float inv_det = 1.0f / determinant;
	Matrix3x3 inverse(Adj.m2[0][0] * inv_det, Adj.m2[0][1] * inv_det, Adj.m2[0][2] * inv_det,
		Adj.m2[1][0] * inv_det, Adj.m2[1][1] * inv_det, Adj.m2[1][2] * inv_det,
		Adj.m2[2][0] * inv_det, Adj.m2[2][1] * inv_det, Adj.m2[2][2] * inv_det);

	return inverse;
}


} // namespace SNova
#pragma once
#include <iostream>

#include "GenMath.h"

namespace SNova
{

struct Matrix3x3
{
	Matrix3x3(const float* Array);
	
	Matrix3x3(float _00 = 1.0f, float _01 = 0.0f, float _02 = 0.0f,
			  float _10 = 0.0f, float _11 = 1.0f, float _12 = 0.0f,
			  float _20 = 0.0f, float _21 = 0.0f, float _22 = 1.0f);

	Matrix3x3& operator=(const Matrix3x3 &rhs);

	Matrix3x3& operator*=(const Matrix3x3 &rhs);
	Matrix3x3& operator+=(const Matrix3x3 rhs);
	Matrix3x3& operator-=(const Matrix3x3 rhs);

	void PrintMatrix3x3();

	float Determinant() const;
	Matrix3x3 GetInverse();
	Matrix3x3 GetTranspose();

	std::string ToString() const;

	float m2[3][3];
};

Matrix3x3 operator+(const Matrix3x3& lhs, const Matrix3x3& rhs);
Matrix3x3 operator-(const Matrix3x3& lhs, const Matrix3x3& rhs);
Matrix3x3 operator * (const Matrix3x3 &lhs, const Matrix3x3 &rhs);

Vec3  operator * (const Matrix3x3 &pMtx, const Vec3 &rhs);

void Mtx33Identity(Matrix3x3 &pResult);
void Mtx33Translate(Matrix3x3 &pResult, float x, float y);
void Mtx33Scale(Matrix3x3 &pResult, float x, float y);
void Mtx33RotRad(Matrix3x3 &pResult, float angle);
void Mtx33RotDeg(Matrix3x3 &pResult, float angle);
void Mtx33Transpose(Matrix3x3 &pResult, const Matrix3x3 &pMtx);

Matrix3x3 Mtx33Inverse(const Matrix3x3 &pMtx);

} // namespace SNova
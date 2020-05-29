/*
 *   Matrix.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "Matrix3x3.h"
#include <iostream>
using namespace Robot;
using namespace std;


Matrix3x3::Matrix3x3()
{
    //达尔文默认赋值为单位阵，这会使得矩阵相乘会出错  改为赋值为零矩阵
    m[m00] = 0;
    m[m01] = 0;
    m[m02] = 0;
    m[m10] = 0;
    m[m11] = 0;
    m[m12] = 0;
    m[m20] = 0;
    m[m21] = 0;
    m[m22] = 0;
}

Matrix3x3::Matrix3x3(const Matrix3x3 &mat)
{
    *this = mat;
}

Matrix3x3::~Matrix3x3()
{
}

void Matrix3x3::Identity()
{
    m[m00] = 1;
    m[m01] = 0;
    m[m02] = 0;
    m[m10] = 0;
    m[m11] = 1;
    m[m12] = 0;
    m[m20] = 0;
    m[m21] = 0;
    m[m22] = 1;
}


void Matrix3x3::Scale(Vector3D scale)
{
    Matrix3x3 mat;
    mat.m[m00] = scale.X;
    mat.m[m11] = scale.Y;
    mat.m[m22] = scale.Z;

    *this *= mat;
}

void Matrix3x3::Rotate(double angle, Vector3D axis)
{
    double rad = angle * 3.141592 / 180.0;
    double C = cos(rad);
    double S = sin(rad);
    Matrix3x3 mat;

    mat.m[m00] = C + axis.X * axis.X * (1 - C);
    mat.m[m01] = axis.X * axis.Y * (1 - C) - axis.Z * S;
    mat.m[m02] = axis.X * axis.Z * (1 - C) + axis.Y * S;
    mat.m[m10] = axis.X * axis.Y * (1 - C) + axis.Z * S;
    mat.m[m11] = C + axis.Y * axis.Y * (1 - C);
    mat.m[m12] = axis.Y * axis.Z * (1 - C) - axis.X * S;
    mat.m[m20] = axis.X * axis.Z * (1 - C) - axis.Y * S;
    mat.m[m21] = axis.Y * axis.Z * (1 - C) + axis.X * S;
    mat.m[m22] = C + axis.Z * axis.Z * (1 - C);

    *this *= mat;
}

Point3D Matrix3x3::Transform(Point3D point)
{
    Point3D result;
    result.X = m[m00]*point.X + m[m01]*point.Y + m[m02]*point.Z;
    result.Y = m[m10]*point.X + m[m11]*point.Y + m[m12]*point.Z;
    result.Z = m[m20]*point.X + m[m21]*point.Y + m[m22]*point.Z;

    return result;
}

void Matrix3x3::SetRx(double Roll)
{
    m[m00] = 1;
    m[m11] = cos(Roll);
    m[m12] = -sin(Roll);
    m[m21] = sin(Roll);
    m[m22] = cos(Roll);
}

void Matrix3x3::SetRy(double Pitch)
{
    m[m11] = 1;
    m[m00] = cos(Pitch);
    m[m02] = sin(Pitch);
    m[m20] = -sin(Pitch);
    m[m22] = cos(Pitch);
}

void Matrix3x3::SetRz(double Yaw)
{
    m[m22] = 1;
    m[m00] = cos(Yaw);
    m[m01] = -sin(Yaw);
    m[m10] = sin(Yaw);
    m[m11] = cos(Yaw);
}

Matrix3x3 Matrix3x3::Transpose()
{
    Matrix3x3 mat;
    mat.m[m00] = m[m00];
    mat.m[m01] = m[m10];
    mat.m[m02] = m[m20];
    mat.m[m10] = m[m01];
    mat.m[m11] = m[m11];
    mat.m[m12] = m[m21];
    mat.m[m20] = m[m02];
    mat.m[m21] = m[m12];
    mat.m[m22] = m[m22];

    return mat;
}

Vector3D Matrix3x3::Transform(Vector3D vector)
{
    Vector3D result;
    result.X = m[m00]*vector.X + m[m01]*vector.Y + m[m02]*vector.Z;
    result.Y = m[m10]*vector.X + m[m11]*vector.Y + m[m12]*vector.Z;
    result.Z = m[m20]*vector.X + m[m21]*vector.Y + m[m22]*vector.Z;

    return result;
}

void Matrix3x3::SetTransform(Point3D point, Vector3D angle)
{
    double Cx = cos(angle.X * 3.141592 / 180.0);
    double Cy = cos(angle.Y * 3.141592 / 180.0);
    double Cz = cos(angle.Z * 3.141592 / 180.0);
    double Sx = sin(angle.X * 3.141592 / 180.0);
    double Sy = sin(angle.Y * 3.141592 / 180.0);
    double Sz = sin(angle.Z * 3.141592 / 180.0);

    Identity();
    m[0] = Cz * Cy;
    m[1] = Cz * Sy * Sx - Sz * Cx;
    m[2] = Cz * Sy * Cx + Sz * Sx;
    m[3] = Sz * Cy;
    m[4] = Sz * Sy * Sx + Cz * Cx;
    m[5] = Sz * Sy * Cx - Cz * Sx;
    m[6] = -Sy;
    m[7] = Cy * Sx;
    m[8] = Cy * Cx;
}

Matrix3x3 & Matrix3x3::operator = (const Matrix3x3 &mat)
{
    for (int i = 0; i < MAXNUM_ELEMENT; i++)
        m[i] = mat.m[i];
    return *this;
}

Matrix3x3 & Matrix3x3::operator *= (const Matrix3x3 &mat)
{
    Matrix3x3 tmp = *this * mat;
    *this = tmp;
    return *this;
}

Matrix3x3 Matrix3x3::operator * (const Matrix3x3 &mat)
{
    Matrix3x3 result;
    for (int j = 0; j < 3; j++)
    {
        for (int i = 0; i <3; i++)
        {
            for (int c = 0; c <3; c++)
            {
                result.m[j*3+i] += m[j*3+c] * mat.m[c*3+i];
            }
        }
    }
    return result;
}







// void Matrix3x3::Translate(Vector3D offset)
// {
//  Matrix3x3 mat;
//  mat.m[m03] = offset.X;
//  mat.m[m13] = offset.Y;
//  mat.m[m23] = offset.Z;

//  *this *= mat;
// }


// bool Matrix3x3::Inverse()
// {
//  Matrix3x3 src, dst, tmp;
//  double det;

//     /* transpose matrix */
//     for (int i = 0; i < 4; i++)
//     {
//         src.m[i] = m[i*4];
//         src.m[i + 4] = m[i*4 + 1];
//         src.m[i + 8] = m[i*4 + 2];
//         src.m[i + 12] = m[i*4 + 3];
//     }

//     /* calculate pairs for first 8 elements (cofactors) */
//     tmp.m[0] = src.m[10] * src.m[15];
//     tmp.m[1] = src.m[11] * src.m[14];
//     tmp.m[2] = src.m[9] * src.m[15];
//     tmp.m[3] = src.m[11] * src.m[13];
//     tmp.m[4] = src.m[9] * src.m[14];
//     tmp.m[5] = src.m[10] * src.m[13];
//     tmp.m[6] = src.m[8] * src.m[15];
//     tmp.m[7] = src.m[11] * src.m[12];
//     tmp.m[8] = src.m[8] * src.m[14];
//     tmp.m[9] = src.m[10] * src.m[12];
//     tmp.m[10] = src.m[8] * src.m[13];
//     tmp.m[11] = src.m[9] * src.m[12];
//     /* calculate first 8 elements (cofactors) */
//     dst.m[0] = (tmp.m[0]*src.m[5] + tmp.m[3]*src.m[6] + tmp.m[4]*src.m[7]) - (tmp.m[1]*src.m[5] + tmp.m[2]*src.m[6] + tmp.m[5]*src.m[7]);
//     dst.m[1] = (tmp.m[1]*src.m[4] + tmp.m[6]*src.m[6] + tmp.m[9]*src.m[7]) - (tmp.m[0]*src.m[4] + tmp.m[7]*src.m[6] + tmp.m[8]*src.m[7]);
//     dst.m[2] = (tmp.m[2]*src.m[4] + tmp.m[7]*src.m[5] + tmp.m[10]*src.m[7]) - (tmp.m[3]*src.m[4] + tmp.m[6]*src.m[5] + tmp.m[11]*src.m[7]);
//     dst.m[3] = (tmp.m[5]*src.m[4] + tmp.m[8]*src.m[5] + tmp.m[11]*src.m[6]) - (tmp.m[4]*src.m[4] + tmp.m[9]*src.m[5] + tmp.m[10]*src.m[6]);
//     dst.m[4] = (tmp.m[1]*src.m[1] + tmp.m[2]*src.m[2] + tmp.m[5]*src.m[3]) - (tmp.m[0]*src.m[1] + tmp.m[3]*src.m[2] + tmp.m[4]*src.m[3]);
//     dst.m[5] = (tmp.m[0]*src.m[0] + tmp.m[7]*src.m[2] + tmp.m[8]*src.m[3]) - (tmp.m[1]*src.m[0] + tmp.m[6]*src.m[2] + tmp.m[9]*src.m[3]);
//     dst.m[6] = (tmp.m[3]*src.m[0] + tmp.m[6]*src.m[1] + tmp.m[11]*src.m[3]) - (tmp.m[2]*src.m[0] + tmp.m[7]*src.m[1] + tmp.m[10]*src.m[3]);
//     dst.m[7] = (tmp.m[4]*src.m[0] + tmp.m[9]*src.m[1] + tmp.m[10]*src.m[2]) - (tmp.m[5]*src.m[0] + tmp.m[8]*src.m[1] + tmp.m[11]*src.m[2]);
//     /* calculate pairs for second 8 elements (cofactors) */
//     tmp.m[0] = src.m[2]*src.m[7];
//     tmp.m[1] = src.m[3]*src.m[6];
//     tmp.m[2] = src.m[1]*src.m[7];
//     tmp.m[3] = src.m[3]*src.m[5];
//     tmp.m[4] = src.m[1]*src.m[6];
//     tmp.m[5] = src.m[2]*src.m[5];
//     //Streaming SIMD Extensions - Inverse of 4x4 Matrix 8
//     tmp.m[6] = src.m[0]*src.m[7];
//     tmp.m[7] = src.m[3]*src.m[4];
//     tmp.m[8] = src.m[0]*src.m[6];
//     tmp.m[9] = src.m[2]*src.m[4];
//     tmp.m[10] = src.m[0]*src.m[5];
//     tmp.m[11] = src.m[1]*src.m[4];
//     /* calculate second 8 elements (cofactors) */
//     dst.m[8] = (tmp.m[0]*src.m[13] + tmp.m[3]*src.m[14] + tmp.m[4]*src.m[15]) - (tmp.m[1]*src.m[13] + tmp.m[2]*src.m[14] + tmp.m[5]*src.m[15]);
//     dst.m[9] = (tmp.m[1]*src.m[12] + tmp.m[6]*src.m[14] + tmp.m[9]*src.m[15]) - (tmp.m[0]*src.m[12] + tmp.m[7]*src.m[14] + tmp.m[8]*src.m[15]);
//     dst.m[10] = (tmp.m[2]*src.m[12] + tmp.m[7]*src.m[13] + tmp.m[10]*src.m[15]) - (tmp.m[3]*src.m[12] + tmp.m[6]*src.m[13] + tmp.m[11]*src.m[15]);
//     dst.m[11] = (tmp.m[5]*src.m[12] + tmp.m[8]*src.m[13] + tmp.m[11]*src.m[14]) - (tmp.m[4]*src.m[12] + tmp.m[9]*src.m[13] + tmp.m[10]*src.m[14]);
//     dst.m[12] = (tmp.m[2]*src.m[10] + tmp.m[5]*src.m[11] + tmp.m[1]*src.m[9]) - (tmp.m[4]*src.m[11] + tmp.m[0]*src.m[9] + tmp.m[3]*src.m[10]);
//     dst.m[13] = (tmp.m[8]*src.m[11] + tmp.m[0]*src.m[8] + tmp.m[7]*src.m[10]) - (tmp.m[6]*src.m[10] + tmp.m[9]*src.m[11] + tmp.m[1]*src.m[8]);
//     dst.m[14] = (tmp.m[6]*src.m[9] + tmp.m[11]*src.m[11] + tmp.m[3]*src.m[8]) - (tmp.m[10]*src.m[11] + tmp.m[2]*src.m[8] + tmp.m[7]*src.m[9]);
//     dst.m[15] = (tmp.m[10]*src.m[10] + tmp.m[4]*src.m[8] + tmp.m[9]*src.m[9]) - (tmp.m[8]*src.m[9] + tmp.m[11]*src.m[10] + tmp.m[5]*src.m[8]);
//     /* calculate determinant */
//     det = src.m[0]*dst.m[0] + src.m[1]*dst.m[1] + src.m[2]*dst.m[2] + src.m[3]*dst.m[3];
//     /* calculate matrix inverse */
//     if (det == 0)
//     {
//         det = 0;
//         return false;
//     }
//     else
//     {
//         det = 1 / det;
//     }

//     for (int i = 0; i < MAXNUM_ELEMENT; i++)
//         m[i] = dst.m[i] * det;

//     return true;
// }

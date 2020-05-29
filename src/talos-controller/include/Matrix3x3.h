/*
 *   Matrix.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _MATRIX_H_
#define _MATRIX_H_

#include "Vector.h"
#include "Point.h"


namespace Robot
{
class Matrix3x3
{
public:
    enum
    {
        m00 = 0,
        m01,
        m02,
        m10,
        m11,
        m12,
        m20,
        m21,
        m22,
        MAXNUM_ELEMENT
    };

private:

protected:

public:
    double m[MAXNUM_ELEMENT]; // Element

    Matrix3x3();
    Matrix3x3(const Matrix3x3 &mat);
    ~Matrix3x3();

    void Identity();
    bool Inverse();
    void Scale(Vector3D scale);
    void Rotate(double angle, Vector3D axis);
    void Translate(Vector3D offset);
    Point3D Transform(Point3D point);
    Vector3D Transform(Vector3D vector);
    void SetTransform(Point3D point, Vector3D angle);
    Matrix3x3 Transpose();
    void SetRx(double Roll);
    void SetRy(double Pitch);
    void SetRz(double Yaw);

    Matrix3x3 & operator = (const Matrix3x3 &mat);
    Matrix3x3 & operator *= (const Matrix3x3 &mat);
    Matrix3x3 operator * (const Matrix3x3 &mat);
};
}

#endif

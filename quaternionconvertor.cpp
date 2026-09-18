#include "quaternionconvertor.h"

#include <cmath>

QuaternionConvertor::QuaternionConvertor(QObject* parent)
    : QObject(parent)
{
}

QUATERNION QuaternionConvertor::ConvertorToQuatAcc(const GY_ICMData& data)
{
    const double axMg = data.axMg;
    const double ayMg = data.ayMg;
    const double azMg = data.azMg;

    const double aNorm = std::sqrt(axMg * axMg + ayMg * ayMg + azMg * azMg);

    if (aNorm < 1e-9) {
        return {1.0, 0.0, 0.0, 0.0};
    }

    const double ax = axMg / aNorm;
    const double ay = ayMg / aNorm;
    const double az = azMg / aNorm;

    const double refX = 0.0;
    const double refY = 0.0;
    const double refZ = 1.0;

    const double dot = refX * ax + refY * ay + refZ * az;

    const double crossX = ay * refZ - az * refY;
    const double crossY = az * refX - ax * refZ;
    const double crossZ = ax * refY - ay * refX;

    const double w = 1.0 + dot;
    const double x = crossX;
    const double y = crossY;
    const double z = crossZ;

    const double norm = std::sqrt(w * w + x * x + y * y + z * z);

    if (norm < 1e-9) {
        return {0.0, 1.0, 0.0, 0.0};
    }

    return {
        w / norm,
        x / norm,
        y / norm,
        z / norm
    };
}

QUATERNION QuaternionConvertor::ConvertorToQuatGyr(const GY_ICMData& data, double dtSec)
{
    if (dtSec <= 0.0) {
        return {1.0, 0.0, 0.0, 0.0};
    }

    constexpr double PI = 3.14159265358979323846;
    constexpr double DEG_TO_RAD = PI / 180.0;

    const double wx = data.gxDps * DEG_TO_RAD;
    const double wy = data.gyDps * DEG_TO_RAD;
    const double wz = data.gzDps * DEG_TO_RAD;

    const double omegaNorm = std::sqrt(wx * wx + wy * wy + wz * wz);

    if (omegaNorm < 1e-9) {
        return {1.0, 0.0, 0.0, 0.0};
    }

    const double ux = wx / omegaNorm;
    const double uy = wy / omegaNorm;
    const double uz = wz / omegaNorm;

    const double theta = omegaNorm * dtSec;
    const double halfTheta = theta / 2.0;

    const double s = std::sin(halfTheta);

    return {
        std::cos(halfTheta),
        ux * s,
        uy * s,
        uz * s
    };
}

QUATERNION QuaternionConvertor::normalize(const QUATERNION& q)
{
    const double norm = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);

    if (norm < 1e-12) {
        return {1.0, 0.0, 0.0, 0.0};
    }

    return {
        q.w / norm,
        q.x / norm,
        q.y / norm,
        q.z / norm
    };
}

QUATERNION QuaternionConvertor::multiply(const QUATERNION& a, const QUATERNION& b)
{
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

QUATERNION QuaternionConvertor::inverse(const QUATERNION& q)
{
    const double norm2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;

    if (norm2 < 1e-12) {
        return {1.0, 0.0, 0.0, 0.0};
    }

    return {
        q.w / norm2,
        -q.x / norm2,
        -q.y / norm2,
        -q.z / norm2
    };
}

QUATERNION QuaternionConvertor::fromSmallAngle(double dx, double dy, double dz)
{
    const QUATERNION q {
        1.0,
        dx / 2.0,
        dy / 2.0,
        dz / 2.0
    };

    return normalize(q);
}

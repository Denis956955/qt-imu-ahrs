#ifndef QUATERNIONCONVERTOR_H
#define QUATERNIONCONVERTOR_H

#include <QObject>

#include "GY_ICMData.h"
#include "Quaternion.h"

class QuaternionConvertor : public QObject
{
    Q_OBJECT

public:
    explicit QuaternionConvertor(QObject* parent = nullptr);

    QUATERNION ConvertorToQuatAcc(const GY_ICMData& data);
    QUATERNION ConvertorToQuatGyr(const GY_ICMData& data, double dtSec);

    QUATERNION normalize(const QUATERNION& q);
    QUATERNION multiply(const QUATERNION& a, const QUATERNION& b);
    QUATERNION inverse(const QUATERNION& q);

    QUATERNION fromSmallAngle(double dx, double dy, double dz);
};

#endif // QUATERNIONCONVERTOR_H

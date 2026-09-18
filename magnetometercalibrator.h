#ifndef MAGNETOMETERCALIBRATOR_H
#define MAGNETOMETERCALIBRATOR_H
#include <QObject>
#include "GY_ICMData.h"
#include <cmath>
#include <Eigen/Dense>
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
class MagnetometerCalibrator: public QObject{
    Q_OBJECT

public:
    explicit MagnetometerCalibrator(QObject* parent = nullptr);
    void MinMaxMagFinder(const GY_ICMData& data);
    void MagCal();
    GY_ICMData GiveCalData(GY_ICMData data);
private:
    double MaxmxUt;
    double MaxmyUt;
    double MaxmzUt;
    double MinmxUt;
    double MinmyUt;
    double MinmzUt;
    int sampleCount_ = 0;
    bool initialization = false;
    double bx = 0.0;
    double by = 0.0;
    double bz = 0.0;

    double rx = 1.0;
    double ry = 1.0;
    double rz = 1.0;

    double ravg = 1.0;

    double sx = 1.0;
    double sy = 1.0;
    double sz = 1.0;
};

#endif // MAGNETOMETERCALIBRATOR_H

#ifndef KALMANFILTERQUATERNION_H
#define KALMANFILTERQUATERNION_H

#include <QObject>
#include <Eigen/Dense>
#include "GY_ICMData.h"
#include "quaternionconvertor.h"
#include "AccDataForCalibartion.h"
#include <cmath>
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat9 = Eigen::Matrix<double, 9, 9>;
using Vec9 = Eigen::Matrix<double, 9, 1>;
using Mat3x9 = Eigen::Matrix<double, 3, 9>;
using Mat9x3 = Eigen::Matrix<double, 9, 3>;
using Mat3x3 = Eigen::Matrix<double, 3, 3>;
struct DeltaState
{
    double dThetaX = 0.0;
    double dThetaY = 0.0;
    double dThetaZ = 0.0;

    double dBgX = 0.0;
    double dBgY = 0.0;
    double dBgZ = 0.0;

    double dBaX = 0.0;
    double dBaY = 0.0;
    double dBaZ = 0.0;
};


class KalmanFilterQuaternion: public QObject{
    Q_OBJECT
public:
    explicit KalmanFilterQuaternion(QObject* parent = nullptr);
    void filtration(GY_ICMData data);
    Mat9 buildF(const Vec3& omega, double dt);
    void getGyrDisp(const GY_ICMData& data);
    void firstGyrrDisp();
    void accelBiasCalc(int iteration, AccDataForCalibartion data);
    void magTiltComp(QUATERNION q, GY_ICMData data);
    void magTiltComp(
        QUATERNION& q,
        GY_ICMData data,
        double magneticDeclinationDeg);
signals:
    void PitchRollReady(const double filteredPitchDeg_,const double filteredRollDeg_);
    void YawReady(const double YawForKalman);
private:

    QUATERNION filtered;
    QUATERNION rawAcc;
    QUATERNION dGyr;
    QUATERNION q;
    QUATERNION qpred;
     QUATERNION dq;
    double filteredRollDeg_ = 0.0;
    double filteredPitchDeg_ = 0.0;
    double filteredYawDeg_ = 0.0;
    double bias = 0;
    qint64 lastMs_ = 0;
    bool hasData_ = false;
    Eigen::Vector2d rollX_{0.0, 0.0};
    Eigen::Matrix2d rollP_ = Eigen::Matrix2d::Identity();

    Eigen::Vector2d pitchX_{0.0, 0.0};
    Eigen::Matrix2d pitchP_ = Eigen::Matrix2d::Identity();
    const double PI = 3.14159265358979323846;
    const double RAD_TO_DEG = 180.0 / PI;
    const double DEG_TO_RAD = PI / 180.0;
    double qAngle_ = 0.001;
    double qBias_ = 0.003;
    double rMeasure_ = 0.5;
    DeltaState dx;
    GY_ICMData dataModifide;
    Mat9 F = Mat9::Zero();
    Vec9 DXpred = Vec9::Zero();
    Vec9 DX = Vec9::Zero();
    Mat9 FT = Mat9::Zero();
    Mat9 Q  = Mat9::Zero();
    Mat9 PPred = Mat9::Zero();
    const double sigmaG  = 0.02;
    const double sigmaBg = 0.001;
    const double sigmaBa = 0.01;
    const double sigmaAx = 0.20;
    const double sigmaAy = 0.20;
    const double sigmaAz = 0.20;

    Mat9 P_ = Mat9::Zero();
    Mat3x3 Rq;
    Mat3x3 Rqmag;
    Vec3 gworld;
    Vec3 gpred;
    Vec3 gmag;
    Vec3 apred;
    Vec3 bpred;
    Vec3 ameas;
    Vec3 ya;
    Vec3 Ha;
    Mat3x3 Ra;
    Mat3x3 Sa;
    Mat9x3 Ka;
    Vec9 dxa;
    void updateEulerAnglesFromQuaternion();
    QuaternionConvertor* Converotor_ = nullptr;
    int numberOfDisplacments = 0;
    double gyroBiasXDps_ = 0;
    double gyroBiasYDps_ = 0;
    double gyroBiasZDps_ = 0;
    double gyroBiasSumX_ = 0;
    double gyroBiasSumY_ = 0;
    double gyroBiasSumZ_ = 0;
    double gyroBiasXDpsRad_ = 0;
    double gyroBiasYDpsRad_ = 0;
    double gyroBiasZDpsRad_ = 0;
    double gyroBiasSumX2_ = 0;
    double gyroBiasSumY2_ = 0;
    double gyroBiasSumZ2_ = 0;
    double sigmaBgX = 0;
    double sigmaBgY = 0;
    double sigmaBgZ = 0;
    AccDataForCalibartion horizontalPlus;
    AccDataForCalibartion horizontalMinus;
    AccDataForCalibartion noseDown;
    AccDataForCalibartion noseUp;
    AccDataForCalibartion rightSide;
    AccDataForCalibartion leftSide;
    double AccBiasZ = 0;
    double AccBiasX = 0;
    double AccBiasY = 0;
    double sigmaSqareAccBiasX = 0;
    double sigmaSqareAccBiasY = 0;
    double sigmaSqareAccBiasZ = 0;
    double sigmaGx_ = 0.0;
    double sigmaGy_ = 0.0;
    double sigmaGz_ = 0.0;
    double yawForKalman = 0.0;
    double magneticDeclinationDeg = 12.0;


};

#endif // KALMANFILTERQUATERNION_H

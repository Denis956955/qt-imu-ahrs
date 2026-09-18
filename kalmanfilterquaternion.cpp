#include "kalmanfilterquaternion.h"
#include <QLabel>
KalmanFilterQuaternion::KalmanFilterQuaternion(QObject* parent)
    : QObject(parent)
{
}
void KalmanFilterQuaternion::filtration(GY_ICMData data){
    int check{0};
    check += 1;
    if (!Converotor_) {
        Converotor_ = new QuaternionConvertor(this);
    }
    if (!hasData_) {
        GY_ICMData accCorrected = data;
        accCorrected.axMg = data.axMg - AccBiasX;
        accCorrected.ayMg = data.ayMg - AccBiasY;
        accCorrected.azMg = data.azMg - AccBiasZ;

        q = Converotor_->ConvertorToQuatAcc(accCorrected);
        q = Converotor_->normalize(q);

        magTiltComp(
            q,
            data,
            magneticDeclinationDeg
            );
        lastMs_ = data.ms;
        hasData_ = true;
        dx.dThetaX = 0;
        dx.dThetaY = 0.0;
        dx.dThetaZ = 0.0;
        dx.dBaX = 0.0;
        dx.dBaY = 0.0;
        dx.dBaZ = 0.0;
        P_ = Mat9::Zero();

        const double initRollSigma  = 0.05; // rad из даташита
        const double initPitchSigma = 0.05; // rad

        P_(0,0) = initRollSigma  * initRollSigma*2; //умножение на 2 тут с запасом
        P_(1,1) = initPitchSigma * initPitchSigma*2;
        const double initYawSigma = 15.0 * DEG_TO_RAD;
        P_(2,2) = initYawSigma * initYawSigma;

        const double initBgSigma = 0.02;

        P_(3,3) = sigmaBgX * sigmaBgX; //Это руками посчитали внизу в функции инициализации
        P_(4,4) = sigmaBgY * sigmaBgY;
        P_(5,5) = sigmaBgZ * sigmaBgZ;

        const double sigmaBaFloorG = 0.02;
        const double sigmaBaFloor2G = sigmaBaFloorG * sigmaBaFloorG;

        P_(6,6) = std::max(sigmaSqareAccBiasX / 1000000.0, sigmaBaFloor2G);
        P_(7,7) = std::max(sigmaSqareAccBiasY / 1000000.0, sigmaBaFloor2G);
        P_(8,8) = std::max(sigmaSqareAccBiasZ / 1000000.0, sigmaBaFloor2G);
        gworld << 0.0,
            0.0,
            1.0;
        dx.dBgX = 0;
        dx.dBgY = 0;
        dx.dBgZ = 0;

        return;
    }
    check += 1;
    const double dt = (data.ms - lastMs_) / 1000.0;
    if (dt <= 0.0 || dt > 1.0) {
        lastMs_ = data.ms;
        return;
    }


    Vec3 omega(
        data.gxDps * DEG_TO_RAD - gyroBiasXDpsRad_,
        data.gyDps * DEG_TO_RAD - gyroBiasYDpsRad_,
        data.gzDps * DEG_TO_RAD - gyroBiasZDpsRad_
        );

    F = buildF(omega, dt);
    FT = F.transpose();

    const double qThetaX = sigmaGx_ * sigmaGx_ * dt * dt;
    const double qThetaY = sigmaGy_ * sigmaGy_ * dt * dt;
    const double qThetaZ = sigmaGz_ * sigmaGz_ * dt * dt;

    const double sigmaBgWalk = 1e-5;
    const double qBg =
        sigmaBgWalk * sigmaBgWalk * dt;

    const double sigmaBaWalk = 1e-5;
    const double qBa =
        sigmaBaWalk * sigmaBaWalk * dt;

    Q << qThetaX, 0.0,     0.0,     0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
        0.0,     qThetaY, 0.0,     0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
        0.0,     0.0,     qThetaZ, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0,

        0.0,     0.0,     0.0,     qBg, 0.0, 0.0,  0.0, 0.0, 0.0,
        0.0,     0.0,     0.0,     0.0, qBg, 0.0,  0.0, 0.0, 0.0,
        0.0,     0.0,     0.0,     0.0, 0.0, qBg,  0.0, 0.0, 0.0,

        0.0,     0.0,     0.0,     0.0, 0.0, 0.0,  qBa, 0.0, 0.0,
        0.0,     0.0,     0.0,     0.0, 0.0, 0.0,  0.0, qBa, 0.0,
        0.0,     0.0,     0.0,     0.0, 0.0, 0.0,  0.0, 0.0, qBa;

    PPred = F * P_ * FT + Q;

    GY_ICMData gyroCorrected = data;

    gyroCorrected.gxDps = omega.x() * RAD_TO_DEG;
    gyroCorrected.gyDps = omega.y() * RAD_TO_DEG;
    gyroCorrected.gzDps = omega.z() * RAD_TO_DEG;

    dGyr = Converotor_->ConvertorToQuatGyr(gyroCorrected, dt);

    qpred = Converotor_->multiply(q, dGyr);
    qpred = Converotor_->normalize(qpred);

    const double w = qpred.w;
    const double x = qpred.x;
    const double y = qpred.y;
    const double z = qpred.z;

    Rq << 1.0 - 2.0 * (y*y + z*z),  2.0 * (x*y - w*z),        2.0 * (x*z + w*y),
        2.0 * (x*y + w*z),        1.0 - 2.0 * (x*x + z*z),  2.0 * (y*z - w*x),
        2.0 * (x*z - w*y),        2.0 * (y*z + w*x),        1.0 - 2.0 * (x*x + y*y);
    gpred = Rq.transpose() * gworld;
    Vec3 ba;
    ba << AccBiasX / 1000.0,
        AccBiasY / 1000.0,
        AccBiasZ / 1000.0;

    apred = gpred + ba;
    Vec3 ameasRaw;
    ameasRaw << data.axMg / 1000.0,
        data.ayMg / 1000.0,
        data.azMg / 1000.0;

    double accNorm = (ameasRaw - ba).norm();

    if (accNorm < 1e-6) {
        lastMs_ = data.ms;
        return;
    }


    double sigmaA = 0.05;

    if (std::abs(accNorm - 1.0) > 0.10) {
        sigmaA = 0.30;
    }

    if (std::abs(accNorm - 1.0) > 0.25) {
        sigmaA = 1.00;
    }

    Ra = Mat3::Identity() * sigmaA * sigmaA;

    Vec3 yaAcc = ameasRaw - apred;

    const double declinationRad =
        magneticDeclinationDeg * DEG_TO_RAD;

    Vec3 magWorld;
    magWorld <<
        std::sin(declinationRad),
        std::cos(declinationRad),
        0.0;

    Vec3 magPred =
        Rq.transpose() * magWorld;

    magPred.normalize();

    Vec3 magBody;
    magBody <<
        data.mxUt,
        -data.myUt,
        -data.mzUt;
    Vec3 gMeas =
        ameasRaw - ba;

    const double gMeasNorm =
        gMeas.norm();

    if (gMeasNorm < 1e-9) {
        lastMs_ = data.ms;
        return;
    }

    gMeas /= gMeasNorm;
    Mat3 Ph =
        Mat3::Identity()
        - gMeas * gMeas.transpose();

    Vec3 magMeas =
        Ph * magBody;

    if (magMeas.norm() < 5.0) {
        lastMs_ = data.ms;
        return;
    }

    magMeas.normalize();

    Vec3 yaMag =
        magMeas - magPred;

    Eigen::Matrix<double, 6, 1> ya6;

    ya6 <<
        yaAcc.x(),
        yaAcc.y(),
        yaAcc.z(),
        yaMag.x(),
        yaMag.y(),
        yaMag.z();

    const double gx = gpred.x();
    const double gy = gpred.y();
    const double gz = gpred.z();

    const double mx = magPred.x();
    const double my = magPred.y();
    const double mz = magPred.z();

    Eigen::Matrix<double, 6, 9> H6;

    H6 <<
        0.0, -gz,  gy,   0.0, 0.0, 0.0,   1.0, 0.0, 0.0,
        gz,   0.0, -gx,   0.0, 0.0, 0.0,   0.0, 1.0, 0.0,
        -gy,   gx,   0.0,  0.0, 0.0, 0.0,   0.0, 0.0, 1.0,

        0.0, -mz,  my,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,
        mz,   0.0, -mx,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,
        -my,   mx,   0.0,  0.0, 0.0, 0.0,   0.0, 0.0, 0.0;

    const double sigmaA2 =
        sigmaA * sigmaA;

    const double sigmaM =
        0.1;

    const double sigmaM2 =
        sigmaM * sigmaM;

    Eigen::Matrix<double, 6, 6> R6;

    R6 <<
        sigmaA2, 0.0,      0.0,      0.0,     0.0,     0.0,
        0.0,      sigmaA2, 0.0,      0.0,     0.0,     0.0,
        0.0,      0.0,      sigmaA2, 0.0,     0.0,     0.0,

        0.0,      0.0,      0.0,      sigmaM2, 0.0,     0.0,
        0.0,      0.0,      0.0,      0.0,     sigmaM2, 0.0,
        0.0,      0.0,      0.0,      0.0,     0.0,     sigmaM2;

    Eigen::Matrix<double, 6, 6> S6 =
        H6 * PPred * H6.transpose() + R6;

    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(S6);

    Eigen::Matrix<double, 9, 6> K6 =
        ldlt.solve(
                (PPred * H6.transpose()).transpose()
                ).transpose();

    dxa =
        K6 * ya6;

    Mat9 I9 =
        Mat9::Identity();

    Mat9 prom =
        I9 - K6 * H6;

    P_ =
        prom * PPred * prom.transpose()
        + K6 * R6 * K6.transpose();

    P_ =
        0.5 * (P_ + P_.transpose());

    Vec3 dTheta = dxa.segment<3>(0);
    Vec3 dBg    = dxa.segment<3>(3);
    Vec3 dBa = dxa.segment<3>(6);

    const double angle = dTheta.norm();

    if (angle < 1e-12) {
        dq.w = 1.0;
        dq.x = 0.0;
        dq.y = 0.0;
        dq.z = 0.0;
    }
    else {
        const double halfAngle = angle / 2.0;
        const double k = std::sin(halfAngle) / angle;

        dq.w = std::cos(halfAngle);
        dq.x = dTheta.x() * k;
        dq.y = dTheta.y() * k;
        dq.z = dTheta.z() * k;
    }

    dq = Converotor_->normalize(dq);

    q = Converotor_->multiply(qpred, dq);
    q = Converotor_->normalize(q);
    gyroBiasXDpsRad_ += dBg.x();
    gyroBiasYDpsRad_ += dBg.y();
    gyroBiasZDpsRad_ += dBg.z();
    AccBiasX += dBa.x()*1000.0;
    AccBiasY += dBa.y()*1000.0;
    AccBiasZ += dBa.z()*1000.0;
    Mat3 dThetaSkew;

    dThetaSkew <<
        0.0,       -dTheta.z(),  dTheta.y(),
        dTheta.z(), 0.0,        -dTheta.x(),
        -dTheta.y(), dTheta.x(),  0.0;

    Mat9 Greset = Mat9::Identity();

    Greset.block<3,3>(0,0) =
        Mat3::Identity() - 0.5 * dThetaSkew;

    P_ =
        Greset * P_ * Greset.transpose();

    P_ =
        0.5 * (P_ + P_.transpose());

    dx.dThetaX = 0.0;
    dx.dThetaY = 0.0;
    dx.dThetaZ = 0.0;

    dx.dBgX = 0.0;
    dx.dBgY = 0.0;
    dx.dBgZ = 0.0;

    dx.dBaX = 0.0;
    dx.dBaY = 0.0;
    dx.dBaZ = 0.0;

    updateEulerAnglesFromQuaternion();

    check += 1;
    lastMs_ = data.ms;

    static int debugCounter = 0;
    debugCounter++;

    if (debugCounter % 20 == 0) {
        qDebug() << "================ KALMAN DEBUG ================";

        qDebug() << "dt:" << dt;

        qDebug() << "raw gyro dps:"
                 << data.gxDps
                 << data.gyDps
                 << data.gzDps;

        qDebug() << "gyro bias rad/s:"
                 << gyroBiasXDpsRad_
                 << gyroBiasYDpsRad_
                 << gyroBiasZDpsRad_;

        qDebug() << "gyro bias deg/s:"
                 << gyroBiasXDpsRad_ * RAD_TO_DEG
                 << gyroBiasYDpsRad_ * RAD_TO_DEG
                 << gyroBiasZDpsRad_ * RAD_TO_DEG;

        qDebug() << "omega corrected rad/s:"
                 << omega.x()
                 << omega.y()
                 << omega.z();

        qDebug() << "omega corrected deg/s:"
                 << omega.x() * RAD_TO_DEG
                 << omega.y() * RAD_TO_DEG
                 << omega.z() * RAD_TO_DEG;

        qDebug() << "accel bias mg:"
                 << AccBiasX
                 << AccBiasY
                 << AccBiasZ;

        qDebug() << "accel bias used g:"
                 << ba.x()
                 << ba.y()
                 << ba.z();

        qDebug() << "ameasRaw g:"
                 << ameasRaw.x()
                 << ameasRaw.y()
                 << ameasRaw.z()
                 << "raw norm:" << ameasRaw.norm();

        qDebug() << "ameas corrected g:"
                 << (ameasRaw - ba).x()
                 << (ameasRaw - ba).y()
                 << (ameasRaw - ba).z()
                 << "corrected norm:" << (ameasRaw - ba).norm();

        qDebug() << "accNorm used for R:"
                 << accNorm
                 << "sigmaA:" << sigmaA;

        qDebug() << "gpred:"
                 << gpred.x()
                 << gpred.y()
                 << gpred.z()
                 << "norm:" << gpred.norm();

        qDebug() << "apred = gpred + ba:"
                 << apred.x()
                 << apred.y()
                 << apred.z()
                 << "norm:" << apred.norm();


        qDebug() << "dTheta raw rad:"
                 << dxa(0,0)
                 << dxa(1,0)
                 << dxa(2,0)
                 << "norm:" << dxa.segment<3>(0).norm();

        qDebug() << "dTheta raw deg:"
                 << dxa(0,0) * RAD_TO_DEG
                 << dxa(1,0) * RAD_TO_DEG
                 << dxa(2,0) * RAD_TO_DEG
                 << "norm deg:" << dxa.segment<3>(0).norm() * RAD_TO_DEG;

        qDebug() << "dTheta applied deg:"
                 << dTheta.x() * RAD_TO_DEG
                 << dTheta.y() * RAD_TO_DEG
                 << dTheta.z() * RAD_TO_DEG
                 << "norm deg:" << dTheta.norm() * RAD_TO_DEG;

        qDebug() << "dBg rad/s:"
                 << dBg.x()
                 << dBg.y()
                 << dBg.z();

        qDebug() << "dBg deg/s:"
                 << dBg.x() * RAD_TO_DEG
                 << dBg.y() * RAD_TO_DEG
                 << dBg.z() * RAD_TO_DEG;

        qDebug() << "dBa g:"
                 << dBa.x()
                 << dBa.y()
                 << dBa.z();

        qDebug() << "dBa mg:"
                 << dBa.x() * 1000.0
                 << dBa.y() * 1000.0
                 << dBa.z() * 1000.0;

        qDebug() << "Q diag:"
                 << Q(0,0) << Q(1,1) << Q(2,2)
                 << Q(3,3) << Q(4,4) << Q(5,5)
                 << Q(6,6) << Q(7,7) << Q(8,8);

        qDebug() << "R accel diag:"
                 << Ra(0,0)
                 << Ra(1,1)
                 << Ra(2,2);

        qDebug() << "P diag:"
                 << P_(0,0) << P_(1,1) << P_(2,2)
                 << P_(3,3) << P_(4,4) << P_(5,5)
                 << P_(6,6) << P_(7,7) << P_(8,8);

        qDebug() << "q:"
                 << q.w
                 << q.x
                 << q.y
                 << q.z;

        qDebug() << "q norm:"
                 << std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);

        qDebug() << "roll pitch:"
                 << filteredRollDeg_
                 << filteredPitchDeg_;

        qDebug() << "==============================================";
    }
    check += 1;
    double rollRad  = filteredRollDeg_  * DEG_TO_RAD;
    double pitchRad = filteredPitchDeg_ * DEG_TO_RAD;


    const double qNorm = std::sqrt(
        q.w * q.w +
        q.x * q.x +
        q.y * q.y +
        q.z * q.z
        );

    if (qNorm < 1e-9) {
        qDebug() << "Ошибка: нулевая норма кватерниона";
        return;
    }


    emit PitchRollReady(filteredPitchDeg_, filteredRollDeg_);
    emit YawReady(filteredYawDeg_);
}

Mat9 KalmanFilterQuaternion::buildF(const Vec3& omega, double dt)
{
    const double wx = omega.x();
    const double wy = omega.y();
    const double wz = omega.z();

    Mat9 F;

    F << 1.0,      wz*dt,   -wy*dt,   -dt,  0.0,  0.0,  0.0, 0.0, 0.0,
        -wz*dt,    1.0,      wx*dt,    0.0, -dt,  0.0,  0.0, 0.0, 0.0,
        wy*dt,   -wx*dt,    1.0,      0.0,  0.0, -dt,  0.0, 0.0, 0.0,

        0.0,      0.0,      0.0,      1.0,  0.0,  0.0,  0.0, 0.0, 0.0,
        0.0,      0.0,      0.0,      0.0,  1.0,  0.0,  0.0, 0.0, 0.0,
        0.0,      0.0,      0.0,      0.0,  0.0,  1.0,  0.0, 0.0, 0.0,

        0.0,      0.0,      0.0,      0.0,  0.0,  0.0,  1.0, 0.0, 0.0,
        0.0,      0.0,      0.0,      0.0,  0.0,  0.0,  0.0, 1.0, 0.0,
        0.0,      0.0,      0.0,      0.0,  0.0,  0.0,  0.0, 0.0, 1.0;

    return F;
}
void KalmanFilterQuaternion::updateEulerAnglesFromQuaternion()
{
    const double qw = q.w;
    const double qx = q.x;
    const double qy = q.y;
    const double qz = q.z;

    const double sinr_cosp =
        2.0 * (qw * qx + qy * qz);

    const double cosr_cosp =
        1.0 - 2.0 * (qx * qx + qy * qy);

    const double rollRad =
        std::atan2(sinr_cosp, cosr_cosp);


    const double sinp =
        2.0 * (qw * qy - qz * qx);

    double pitchRad;

    if (std::abs(sinp) >= 1.0) {
        pitchRad =
            std::copysign(PI / 2.0, sinp);
    } else {
        pitchRad =
            std::asin(sinp);
    }


    const double forwardWorldX =
        2.0 * (qx * qy - qw * qz);

    const double forwardWorldY =
        1.0 - 2.0 * (qx * qx + qz * qz);

    const double yawRad =
        std::atan2(
            forwardWorldX,
            forwardWorldY
            );


    filteredRollDeg_ =
        rollRad * RAD_TO_DEG;

    filteredPitchDeg_ =
        pitchRad * RAD_TO_DEG;

    filteredYawDeg_ =
        yawRad * RAD_TO_DEG;

    if (filteredYawDeg_ < 0.0) {
        filteredYawDeg_ += 360.0;
    }
}


void KalmanFilterQuaternion::getGyrDisp(const GY_ICMData& data)
{
    gyroBiasSumX_  += data.gxDps;
    gyroBiasSumY_  += data.gyDps;
    gyroBiasSumZ_  += data.gzDps;

    gyroBiasSumX2_ += data.gxDps * data.gxDps;
    gyroBiasSumY2_ += data.gyDps * data.gyDps;
    gyroBiasSumZ2_ += data.gzDps * data.gzDps;

    numberOfDisplacments += 1;
}

void KalmanFilterQuaternion::firstGyrrDisp()
{
    if (numberOfDisplacments < 2) {
        qDebug() << "Ошибка: недостаточно данных для калибровки гироскопа";
        return;
    }
    const double N = static_cast<double>(numberOfDisplacments);
    const double meanX = gyroBiasSumX_ / N;
    const double meanY = gyroBiasSumY_ / N;
    const double meanZ = gyroBiasSumZ_ / N;
    double varX = (gyroBiasSumX2_ - N * meanX * meanX) / (N - 1.0);
    double varY = (gyroBiasSumY2_ - N * meanY * meanY) / (N - 1.0);
    double varZ = (gyroBiasSumZ2_ - N * meanZ * meanZ) / (N - 1.0);
    varX = std::max(0.0, varX);
    varY = std::max(0.0, varY);
    varZ = std::max(0.0, varZ);
    const double sigmaSampleX = std::sqrt(varX);
    const double sigmaSampleY = std::sqrt(varY);
    const double sigmaSampleZ = std::sqrt(varZ);
    const double sigmaMeanX = sigmaSampleX / std::sqrt(N);
    const double sigmaMeanY = sigmaSampleY / std::sqrt(N);
    const double sigmaMeanZ = sigmaSampleZ / std::sqrt(N);
    const double sigmaBgXRadFromData = sigmaMeanX * DEG_TO_RAD;
    const double sigmaBgYRadFromData = sigmaMeanY * DEG_TO_RAD;
    const double sigmaBgZRadFromData = sigmaMeanZ * DEG_TO_RAD;
    const double sigmaBgFloor = 0.001;
    sigmaBgX = std::max(sigmaBgXRadFromData, sigmaBgFloor);
    sigmaBgY = std::max(sigmaBgYRadFromData, sigmaBgFloor);
    sigmaBgZ = std::max(sigmaBgZRadFromData, sigmaBgFloor);
    gyroBiasXDps_ = meanX;
    gyroBiasYDps_ = meanY;
    gyroBiasZDps_ = meanZ;
    gyroBiasXDpsRad_ = gyroBiasXDps_ * DEG_TO_RAD;
    gyroBiasYDpsRad_ = gyroBiasYDps_ * DEG_TO_RAD;
    gyroBiasZDpsRad_ = gyroBiasZDps_ * DEG_TO_RAD;
    sigmaGx_ = sigmaSampleX * DEG_TO_RAD;
    sigmaGy_ = sigmaSampleY * DEG_TO_RAD;
    sigmaGz_ = sigmaSampleZ * DEG_TO_RAD;
    qDebug() << "Смещение гироскопа deg/s:"
             << "X:" << gyroBiasXDps_
             << "Y:" << gyroBiasYDps_
             << "Z:" << gyroBiasZDps_;

    qDebug() << "Sigma sample deg/s:"
             << "X:" << sigmaSampleX
             << "Y:" << sigmaSampleY
             << "Z:" << sigmaSampleZ;

    qDebug() << "Sigma mean deg/s:"
             << "X:" << sigmaMeanX
             << "Y:" << sigmaMeanY
             << "Z:" << sigmaMeanZ;

    qDebug() << "Sigma bg used rad/s:"
             << "X:" << sigmaBgX
             << "Y:" << sigmaBgY
             << "Z:" << sigmaBgZ;
}
void KalmanFilterQuaternion::accelBiasCalc(int iteration, AccDataForCalibartion data){
    if(iteration == 1){
        horizontalPlus.x = data.x;
        horizontalPlus.y = data.y;
        horizontalPlus.z = data.z;
    }
    if(iteration == 3){
        horizontalMinus.x = data.x;
        horizontalMinus.y = data.y;
        horizontalMinus.z = data.z;
    }
    if(iteration == 5){
        noseDown.x = data.x;
        noseDown.y = data.y;
        noseDown.z = data.z;
    }
    if(iteration == 7){
        noseUp.x = data.x;
        noseUp.y = data.y;
        noseUp.z = data.z;
    }
    if(iteration == 9){
        rightSide.x = data.x;
        rightSide.y = data.y;
        rightSide.z = data.z;
    }
    if(iteration == 11){
        leftSide.x = data.x;
        leftSide.y = data.y;
        leftSide.z = data.z;

        double bx1 = 0.5 * (horizontalPlus.x + horizontalMinus.x);
        double bx2 = 0.5 * (noseDown.x + noseUp.x);
        double bx3 = 0.5 * (rightSide.x + leftSide.x);

        double by1 = 0.5 * (horizontalPlus.y + horizontalMinus.y);
        double by2 = 0.5 * (noseDown.y + noseUp.y);
        double by3 = 0.5 * (rightSide.y + leftSide.y);

        double bz1 = 0.5 * (horizontalPlus.z + horizontalMinus.z);
        double bz2 = 0.5 * (noseDown.z + noseUp.z);
        double bz3 = 0.5 * (rightSide.z + leftSide.z);

        AccBiasX = (bx1 + bx2 + bx3) / 3.0;
        AccBiasY = (by1 + by2 + by3) / 3.0;
        AccBiasZ = (bz1 + bz2 + bz3) / 3.0;

        double dx1 = bx1 - AccBiasX;
        double dx2 = bx2 - AccBiasX;
        double dx3 = bx3 - AccBiasX;
        sigmaSqareAccBiasX = (dx1*dx1 + dx2*dx2 + dx3*dx3) / 2.0;

        double dy1 = by1 - AccBiasY;
        double dy2 = by2 - AccBiasY;
        double dy3 = by3 - AccBiasY;
        sigmaSqareAccBiasY = (dy1*dy1 + dy2*dy2 + dy3*dy3) / 2.0;

        double dz1 = bz1 - AccBiasZ;
        double dz2 = bz2 - AccBiasZ;
        double dz3 = bz3 - AccBiasZ;
        sigmaSqareAccBiasZ = (dz1*dz1 + dz2*dz2 + dz3*dz3) / 2.0;

        qDebug() << "Смещение акселерометра по x" << AccBiasX << "mg";
        qDebug() << "Смещение акселерометра по y" << AccBiasY << "mg";
        qDebug() << "Смещение акселерометра по z" << AccBiasZ << "mg";

        qDebug() << "Дисперсия смещения акселерометра x" << sigmaSqareAccBiasX << "mg^2";
        qDebug() << "Дисперсия смещения акселерометра y" << sigmaSqareAccBiasY << "mg^2";
        qDebug() << "Дисперсия смещения акселерометра z" << sigmaSqareAccBiasZ << "mg^2";
    }

};
void KalmanFilterQuaternion::magTiltComp(
    QUATERNION& q,
    GY_ICMData data,
    double magneticDeclinationDeg)
{
    auto norm360 = [](double deg) {
        while (deg < 0.0) {
            deg += 360.0;
        }

        while (deg >= 360.0) {
            deg -= 360.0;
        }

        return deg;
    };

    auto norm180 = [](double deg) {
        while (deg > 180.0) {
            deg -= 360.0;
        }

        while (deg < -180.0) {
            deg += 360.0;
        }

        return deg;
    };

    const double wmag = q.w;
    const double xmag = q.x;
    const double ymag = q.y;
    const double zmag = q.z;

    Mat3 Rqmag;

    Rqmag <<
        1.0 - 2.0 * (ymag * ymag + zmag * zmag),
        2.0 * (xmag * ymag - wmag * zmag),
        2.0 * (xmag * zmag + wmag * ymag),

        2.0 * (xmag * ymag + wmag * zmag),
        1.0 - 2.0 * (xmag * xmag + zmag * zmag),
        2.0 * (ymag * zmag - wmag * xmag),

        2.0 * (xmag * zmag - wmag * ymag),
        2.0 * (ymag * zmag + wmag * xmag),
        1.0 - 2.0 * (xmag * xmag + ymag * ymag);

    const Vec3 gworld = Vec3::UnitZ();

    Vec3 gmag =
        Rqmag.transpose() * gworld;

    const double gNorm = gmag.norm();

    if (gNorm < 1e-9) {
        return;
    }

    gmag /= gNorm;

    const Mat3 Ph =
        Mat3::Identity()
        - gmag * gmag.transpose();

    const Vec3 fb =
        Vec3::UnitY();

    Vec3 fh =
        Ph * fb;

    const double fhNorm = fh.norm();

    if (fhNorm < 1e-9) {
        return;
    }

    fh /= fhNorm;

    Vec3 rh =
        fh.cross(gmag);

    const double rhNorm = rh.norm();

    if (rhNorm < 1e-9) {
        return;
    }

    rh /= rhNorm;

    Vec3 magBody;

    magBody <<
        data.mxUt,
        -data.myUt,
        -data.mzUt;

    Vec3 magHorizontal =
        Ph * magBody;

    const double magHorizontalNorm =
        magHorizontal.norm();

    if (magHorizontalNorm < 5.0) {
        return;
    }

    magHorizontal /= magHorizontalNorm;

    const double magForward =
        fh.dot(magHorizontal);

    const double magRight =
        rh.dot(magHorizontal);

    const double magneticDirectionRad =
        std::atan2(
            magRight,
            magForward
            );

    const double magneticHeadingDeg =
        norm360(
            -magneticDirectionRad * RAD_TO_DEG
            );

    yawForKalman =
        norm360(
            magneticHeadingDeg
            + magneticDeclinationDeg
            );

    const Vec3 forwardWorld =
        Rqmag * fb;

    const double currentYawRad =
        std::atan2(
            forwardWorld.x(),
            forwardWorld.y()
            );

    const double currentYawDeg =
        norm360(
            currentYawRad * RAD_TO_DEG
            );

    const double deltaYawDeg =
        norm180(
            yawForKalman
            - currentYawDeg
            );

    const double deltaYawRad =
        deltaYawDeg * DEG_TO_RAD;

    QUATERNION qYaw;

    qYaw.w =
        std::cos(deltaYawRad / 2.0);

    qYaw.x = 0.0;
    qYaw.y = 0.0;

    qYaw.z =
        std::sin(deltaYawRad / 2.0);

    q =
        Converotor_->multiply(
            qYaw,
            q
            );

    q =
        Converotor_->normalize(q);
}

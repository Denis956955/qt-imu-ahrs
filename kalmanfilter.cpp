#include "kalmanfilter.h"
#include <QLabel>

KalmanFilter::KalmanFilter(QObject* parent)
    : QObject(parent)
{
}
void KalmanFilter::filtration(const GY_ICMData& data){
    int check{0};
    const double rollAccDeg = std::atan2(data.ayMg, data.azMg) * 180.0 / 3.14159265358979323846;
    check += 1;
    const double pitchAccDeg = std::atan2(
                                   -data.axMg,
                                   std::sqrt(data.ayMg * data.ayMg + data.azMg * data.azMg)
                                   ) * 180.0 / 3.14159265358979323846;
    check += 1;
    if (!hasData_) {
        filteredRollDeg_ = rollAccDeg;
        filteredPitchDeg_ = pitchAccDeg;

        rollX_(0) = rollAccDeg;
        rollX_(1) = 0.0;

        pitchX_(0) = pitchAccDeg;
        pitchX_(1) = 0.0;

        lastMs_ = data.ms;
        hasData_ = true;
        return;
    }
    check += 1;
    const double dt = (data.ms - lastMs_) / 1000.0;
    if (dt <= 0.0 || dt > 1.0) {
        lastMs_ = data.ms;
        return;
    }
    check += 1;
    filteredPitchDeg_ = updateOneAxis(pitchAccDeg,data.gyDps,dt,pitchX_,pitchP_);
    filteredRollDeg_ = updateOneAxis(rollAccDeg,data.gxDps,dt,rollX_,rollP_);
    lastMs_ = data.ms;
    qDebug() << "gxDps:" << data.gxDps
             << "gyDps:" << data.gyDps
             << "| Roll acc:" << rollAccDeg
             << "Roll kalman:" << filteredRollDeg_
             << "Roll bias:" << rollX_(1)
             << "| Pitch acc:" << pitchAccDeg
             << "Pitch kalman:" << filteredPitchDeg_
             << "Pitch bias:" << pitchX_(1);
    check += 1;
    if(check == 5){
        emit PitchRollReady(filteredPitchDeg_, filteredRollDeg_);
    }
}
double KalmanFilter::updateOneAxis(double accAngleDeg,
                                   double gyroRateDps,
                                   double dt,
                                   Eigen::Vector2d& x,
                                   Eigen::Matrix2d& P)
{
    Eigen::Matrix2d F;
    F << 1.0, -dt,
        0.0,  1.0;

    Eigen::Vector2d B;
    B << dt,
        0.0;

    Eigen::Matrix2d Q;
    Q << qAngle_ * dt, 0.0,
        0.0,          qBias_ * dt;

    Eigen::RowVector2d H;
    H << 1.0, 0.0;

    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();

    const double z = accAngleDeg;
    const double u = gyroRateDps;

    Eigen::Vector2d xPred = F * x + B * u;
    Eigen::Matrix2d PPred = F * P * F.transpose() + Q;

    const double y = z - (H * xPred)(0, 0); //Нолики это элемент матрицы берем
    const double S = (H * PPred * H.transpose())(0, 0) + rMeasure_;

    Eigen::Vector2d K = PPred * H.transpose() / S;

    x = xPred + K * y;
    P = (I - K * H) * PPred;

    return x(0);
}

#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#include <QObject>
#include <Eigen/Dense>
#include "GY_ICMData.h"
class KalmanFilter: public QObject{
    Q_OBJECT
public:
    explicit KalmanFilter(QObject* parent = nullptr);
    void filtration(const GY_ICMData& data);
signals:
    void PitchRollReady(const double filteredPitchDeg_,const double filteredRollDeg_);
private:
    double axMg;
    double ayMg;
    double azMg;

    double gxDps;
    double gyDps;
    double gzDps;

    double mxUt;
    double myUt;
    double mzUt;

    double rollAccDeg;
    double pitchAccDeg;
    double filteredRollDeg_ = 0.0;
    double filteredPitchDeg_ = 0.0;
    double bias = 0;
    qint64 lastMs_ = 0;
    bool hasData_ = false;
    Eigen::Vector2d rollX_{0.0, 0.0};
    Eigen::Matrix2d rollP_ = Eigen::Matrix2d::Identity();

    Eigen::Vector2d pitchX_{0.0, 0.0};
    Eigen::Matrix2d pitchP_ = Eigen::Matrix2d::Identity();

    double qAngle_ = 0.001;
    double qBias_ = 0.003;
    double rMeasure_ = 0.5;

    double updateOneAxis(double accAngleDeg,
                         double gyroRateDps,
                         double dt,
                         Eigen::Vector2d& x,
                         Eigen::Matrix2d& P);
};

#endif // KALMANFILTER_H

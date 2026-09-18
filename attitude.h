#ifndef ATTITUDE_H
#define ATTITUDE_H
#include <QObject>
#include <QWidget>
class Attitude : public QWidget
{
    Q_OBJECT

public:
    explicit Attitude(QWidget* parent = nullptr);
    void setAttitude(double pitchDegOutside, double rollDegOutside);
protected:
    void paintEvent(QPaintEvent* event) override;

private:

    double pitchDeg = 0;
    double rollDeg = 0;
    double yawDeg = 0;
    QPixmap planePixmap_;
    double planeAngleDeg_ = 0.0;
    const double backgroundHeightFactor = 2.0;
    const double pitchLimitDeg = 10.0;
    const double planeScale = 0.5;
    const double planeWidthFactor = 1.25;
    const double planeYOffsetFactor = 1.25;
};

#endif // ATTITUDE_H

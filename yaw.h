#ifndef YAW_H
#define YAW_H
#include <QObject>
#include <QWidget>
class Yaw : public QWidget
{
    Q_OBJECT
public:
    explicit Yaw(QWidget* parent = nullptr);
    void setAttitude(double yawDegOutside);
protected:
    void paintEvent(QPaintEvent* event) override;
private:
    QPixmap planePixmap_;
    QPixmap compPixmap_;
    double planeAngleDeg_ = 0.0;
    const double backgroundHeightFactor = 2.0;
    const double pitchLimitDeg = 10.0;
    const double planeScale = 0.5;
    const double planeWidthFactor = 1.25;
    const double planeYOffsetFactor = 1.25;

};

#endif // YAW_H

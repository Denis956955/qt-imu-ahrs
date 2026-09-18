#include <QPainter>
#include "attitude.h"


Attitude::Attitude(QWidget* parent)
    : QWidget(parent), planePixmap_(":/images/plane.png")
{
    setMinimumSize(300, 200);
}


void Attitude::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    const double w = width();
    const double h = height();

    double rectHeight = h / 2.0;
    double rectHeight2 = h / 2.0;

    if (pitchDeg > pitchLimitDeg) {
        rectHeight = 0.0;
        rectHeight2 = h * backgroundHeightFactor;
        planeAngleDeg_ = rollDeg;
    }
    else if (pitchDeg < -pitchLimitDeg) {
        rectHeight = h * backgroundHeightFactor;
        rectHeight2 = 0.0;
        planeAngleDeg_ = rollDeg;
    }
    else {
        rectHeight = h - (pitchDeg * (h / pitchLimitDeg));
        rectHeight2 = h + (pitchDeg * (h / pitchLimitDeg));
        planeAngleDeg_ = rollDeg;
    }

    QPainter painter(this);
    painter.setBrush(Qt::blue);
    painter.setPen(Qt::NoPen);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    const double cx = w / 2.0;
    const double cy = h / 2.0;

    const double imageW = w;
    const double imageH = h;

    const double planeW = imageW * planeScale * planeWidthFactor;
    const double planeH = imageH * planeScale;

    painter.save();

    painter.translate(cx, cy);
    painter.rotate(planeAngleDeg_);

    painter.drawRect(QRectF(-w / 2.0, -h, w, rectHeight));

    painter.setBrush(QColor(139, 69, 19));
    painter.drawRect(QRectF(-w / 2.0, h, w, -rectHeight2));

    painter.restore();

    painter.save();

    painter.translate(cx, cy);

    painter.drawPixmap(
        QRectF(-planeW / 2.0,
               (-planeH / 2.0) * planeYOffsetFactor,
               planeW,
               planeH),
        planePixmap_,
        QRectF(0.0, 0.0, planePixmap_.width(), planePixmap_.height())
        );

    painter.restore();
}

void Attitude::setAttitude(double pitchDegOutside, double rollDegOutside){
    rollDeg = -rollDegOutside;
    pitchDeg = pitchDegOutside;
    //qDebug()<<pitchDeg<< "desert rose "<< rollDeg;

    update();
};

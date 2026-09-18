#include "yaw.h"
#include <QPainter>

Yaw::Yaw(QWidget* parent)
    : QWidget(parent), planePixmap_(":/images/aircraft_heading_transparent.png"), compPixmap_(":/images/heading_scale_transparent.png")
{
    setMinimumSize(300, 200);
}
void Yaw::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);
    const double w = width();
    const double h = height();
    QPainter painter(this);
    painter.setBrush(Qt::blue);
    painter.setPen(Qt::NoPen);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    const double cx = w / 2.0;
    const double cy = h / 2.0;

    const double imageW = 1.5*h;
    const double imageH = 1.5*h;

    const double planeW = imageW * planeScale * planeWidthFactor;
    const double planeH = imageH * planeScale;

    painter.save();

    painter.translate(cx, cy);
    painter.rotate(-planeAngleDeg_);
    painter.drawPixmap(
        QRectF(-planeW / 2.0,
               (-planeH / 2.0) * planeYOffsetFactor,
               planeW,
               planeW),
        compPixmap_,
        QRectF(0.0, 0.0, compPixmap_.width(), compPixmap_.height())
        );

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
void Yaw::setAttitude(double yawDegOutside){
    planeAngleDeg_ = yawDegOutside;


    update();
};

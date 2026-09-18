#include "magnetometercalibrator.h"
#include <algorithm>
#include <QDebug>
MagnetometerCalibrator::MagnetometerCalibrator(QObject* parent)
    : QObject(parent)
{
}
void MagnetometerCalibrator::MinMaxMagFinder(const GY_ICMData& data){
    if (!initialization) {
        MaxmxUt = data.mxUt;
        MaxmyUt = data.myUt;
        MaxmzUt = data.mzUt;
        MinmxUt = data.mxUt;
        MinmyUt = data.myUt;
        MinmzUt = data.mzUt;

        initialization = true;
        sampleCount_ = 1;

        qDebug() << "[MAG CAL] Этап сбора данных начат. Первое измерение:"
                 << "mx =" << data.mxUt
                 << "my =" << data.myUt
                 << "mz =" << data.mzUt;

        return;
    }
    MinmxUt = std::min(MinmxUt, data.mxUt);
    MaxmxUt = std::max(MaxmxUt, data.mxUt);

    MinmyUt = std::min(MinmyUt, data.myUt);
    MaxmyUt = std::max(MaxmyUt, data.myUt);

    MinmzUt = std::min(MinmzUt, data.mzUt);
    MaxmzUt = std::max(MaxmzUt, data.mzUt);

    ++sampleCount_;
}
void MagnetometerCalibrator::MagCal()
{
    if (!initialization) {
        qDebug() << "[MAG CAL] Ошибка: калибровка не выполнена, нет данных магнитометра";
        return;
    }

    if (sampleCount_ < 30) {
        qDebug() << "[MAG CAL] Предупреждение: мало измерений для калибровки магнитометра:"
                 << sampleCount_
                 << "из рекомендуемых минимум 30";
        // Можно пока не return, чтобы тестировать алгоритм
        // return;
    }

    bx = (MaxmxUt + MinmxUt) / 2.0;
    by = (MaxmyUt + MinmyUt) / 2.0;
    bz = (MaxmzUt + MinmzUt) / 2.0;

    qDebug() << "[MAG CAL] Этап 1 OK: hard-iron смещения рассчитаны:"
             << "bx =" << bx
             << "by =" << by
             << "bz =" << bz;

    rx = (MaxmxUt - MinmxUt) / 2.0;
    ry = (MaxmyUt - MinmyUt) / 2.0;
    rz = (MaxmzUt - MinmzUt) / 2.0;

    qDebug() << "[MAG CAL] Этап 2 OK: радиусы магнитного эллипсоида рассчитаны:"
             << "rx =" << rx
             << "ry =" << ry
             << "rz =" << rz;

    if (rx < 1e-9 || ry < 1e-9 || rz < 1e-9) {
        sx = 1.0;
        sy = 1.0;
        sz = 1.0;

        qDebug() << "[MAG CAL] Ошибка: один из радиусов слишком мал."
                 << "Калибровка масштаба отключена, sx/sy/sz = 1";

        return;
    }

    ravg = (rx + ry + rz) / 3.0;

    sx = ravg / rx;
    sy = ravg / ry;
    sz = ravg / rz;

    qDebug() << "[MAG CAL] Этап 3 OK: scale-факторы рассчитаны:"
             << "sx =" << sx
             << "sy =" << sy
             << "sz =" << sz;

    qDebug() << "[MAG CAL] Калибровка магнитометра завершена успешно."
             << "samples =" << sampleCount_
             << "offset = (" << bx << by << bz << ")"
             << "radius = (" << rx << ry << rz << ")"
             << "scale = (" << sx << sy << sz << ")";
}
// GY_ICMData MagnetometerCalibrator::GiveCalData(GY_ICMData data)
// {
//     qDebug() << "[MAG APPLY BEFORE]"
//              << "raw:" << data.mxUt << data.myUt << data.mzUt
//              << "offset:" << bx << by << bz
//              << "scale:" << sx << sy << sz;

//     data.mxUt = (data.mxUt - bx) * sx;
//     data.myUt = (data.myUt - by) * sy;
//     data.mzUt = (data.mzUt - bz) * sz;

//     qDebug() << "[MAG APPLY AFTER]"
//              << "cal:" << data.mxUt << data.myUt << data.mzUt;

//     return data;
// }
GY_ICMData MagnetometerCalibrator::GiveCalData(GY_ICMData data)
{
    const Vec3 magRaw(
        data.mxUt,
        data.myUt,
        data.mzUt
        );

    static const Vec3 magBias(
        -87.60,
        47.81,
        13.87
        );

    static const Mat3 magCorrection = (
                                          Mat3() <<
                                              0.947,  0.023,  0.134,
                                          0.023,  1.047, -0.022,
                                          0.134, -0.022,  1.028
                                          ).finished();

    const Vec3 magCalibrated =
        magCorrection * (magRaw - magBias);

    data.mxUt = magCalibrated.x();
    data.myUt = magCalibrated.y();
    data.mzUt = magCalibrated.z();

    qDebug() << "[MAG APPLY AFTER]"
             << "cal:"
             << data.mxUt
             << data.myUt
             << data.mzUt;

    return data;
}

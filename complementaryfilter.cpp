#include "complementaryfilter.h"
#include <QLabel>

ComplementaryFilter::ComplementaryFilter(QObject* parent)
    : QObject(parent)
{
}
void ComplementaryFilter::filtration(const GY_ICMData& data)
{
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
    filteredRollDeg_ = alpha_ * (filteredRollDeg_ + data.gxDps * dt)
                       + (1.0 - alpha_) * rollAccDeg;
    check += 1;
    filteredPitchDeg_ = alpha_ * (filteredPitchDeg_ + data.gyDps * dt)
                        + (1.0 - alpha_) * pitchAccDeg;
    check += 1;
    lastMs_ = data.ms;
    check += 1;
    if(check == 7){
        emit PitchRollReady(filteredPitchDeg_, filteredRollDeg_);
    }
    qDebug() << "Roll filtered comp:" << filteredRollDeg_
             << "Pitch filtered comp:" << filteredPitchDeg_;
    //       << "| Roll acc self:" << rollAccDeg
    //       << "Pitch acc self:" << pitchAccDeg
    //     << "| Roll ESP:" << data.rollAccDeg
    //     << "Pitch ESP:" << data.pitchAccDeg;
}

#ifndef COMPLEMENTARYFILTER_H
#define COMPLEMENTARYFILTER_H
#include <QObject>
#include "GY_ICMData.h"

class ComplementaryFilter: public QObject{
    Q_OBJECT
public:
    explicit ComplementaryFilter(QObject* parent = nullptr);
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

    qint64 lastMs_ = 0;
    bool hasData_ = false;

    double alpha_ = 0.98;
};

#endif // COMPLEMENTARYFILTER_H

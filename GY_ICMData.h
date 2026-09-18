#ifndef GY_ICMDATA_H
#define GY_ICMDATA_H
#include <vector>
#include <QByteArray>
#include <QtGlobal>

struct GY_ICMData
{
    QByteArray type;
    qint64 ms;

    double axMg;
    double ayMg;
    double azMg;

    double gxDps;
    double gyDps;
    double gzDps;

    double mxUt;
    double myUt;
    double mzUt;

    double tempC;

    double rollAccDeg;
    double pitchAccDeg;
};

struct GY_ICMDataStorage
{
    std::vector<QByteArray> type;
    std::vector<qint64> ms;

    std::vector<double> axMg;
    std::vector<double> ayMg;
    std::vector<double> azMg;

    std::vector<double> gxDps;
    std::vector<double> gyDps;
    std::vector<double> gzDps;

    std::vector<double> mxUt;
    std::vector<double> myUt;
    std::vector<double> mzUt;

    std::vector<double> tempC;

    std::vector<double> rollAccDeg;
    std::vector<double> pitchAccDeg;
};
#endif // GY_ICMDATA_H

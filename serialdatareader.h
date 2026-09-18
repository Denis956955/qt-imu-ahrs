#ifndef SERIALDATAREADER_H
#define SERIALDATAREADER_H
#include <QObject>
#include <QSerialPort>
#include "GY_ICMData.h"

class SerialDataReader : public QObject
{
    Q_OBJECT

public:
    explicit SerialDataReader(QObject* parent = nullptr);
    void openSerialPort();

signals:
    void rawDataReady(const GY_ICMData& data);

private:
    QSerialPort* serialPort_ = nullptr;
    QByteArray serialBuffer_;
    void splitDataInSerialRaw();
    void outPrintRawData(const QByteArray& line);
    void splitDataInSerialProcessed();
    void outPrintProcesedData(const QByteArray& line);
    GY_ICMData GYDATA;
    GY_ICMDataStorage GYDATASORAGE;
};

#endif // SERIALDATAREADER_H

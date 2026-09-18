#include "serialdatareader.h"
#include <QLabel>

SerialDataReader::SerialDataReader(QObject* parent)
    : QObject(parent)
{
    serialPort_ = new QSerialPort(this);

    connect(serialPort_, &QSerialPort::readyRead,
            this, &SerialDataReader::splitDataInSerialProcessed);


}
void SerialDataReader::openSerialPort()
{
    serialPort_->setPortName("/dev/ttyUSB0");
    serialPort_->setBaudRate(QSerialPort::Baud115200);
    serialPort_->setDataBits(QSerialPort::Data8);
    serialPort_->setParity(QSerialPort::NoParity);
    serialPort_->setStopBits(QSerialPort::OneStop);
    serialPort_->setFlowControl(QSerialPort::NoFlowControl);

    if (!serialPort_->open(QIODevice::ReadOnly)) {
        qDebug() << "Can't open serial port:" << serialPort_->errorString();
        return;
    }

    qDebug() << "Serial port opened successfully";
}

void SerialDataReader::splitDataInSerialRaw()
{
    serialBuffer_ += serialPort_->readAll();

    int newLineIndex = serialBuffer_.indexOf('\n');

    while (newLineIndex != -1) {
        QByteArray line = serialBuffer_.left(newLineIndex);
        serialBuffer_.remove(0, newLineIndex + 1);

        line = line.trimmed();

        if (!line.isEmpty()) {
            outPrintRawData(line);
        }

        newLineIndex = serialBuffer_.indexOf('\n');
    }
}

void SerialDataReader::outPrintRawData(const QByteArray& line)
{
    qDebug() << "ESP32:" << line;
}
void SerialDataReader::splitDataInSerialProcessed(){
    serialBuffer_ += serialPort_->readAll();

    int newLineIndex = serialBuffer_.indexOf('\n');

    while (newLineIndex != -1) {
        QByteArray line = serialBuffer_.left(newLineIndex);
        serialBuffer_.remove(0, newLineIndex + 1);

        line = line.trimmed();

        if (!line.isEmpty()) {
            outPrintProcesedData(line);
        }

        newLineIndex = serialBuffer_.indexOf('\n');
    }
}
void SerialDataReader::outPrintProcesedData(const QByteArray& line)
{
    int indexOfValidation{0};
    bool ok = false;

    QByteArray linecopy = line.trimmed();

    int IndexOfComma = linecopy.indexOf(',');

    if (IndexOfComma == -1) {
        qDebug() << "Ошибка: индекс запятой не найден для type";
        return;
    }

    GYDATA.type = linecopy.left(IndexOfComma).trimmed();

    if (GYDATA.type != "RAW") {
        qDebug() << "Ошибка: неверный тип данных:" << GYDATA.type;
        return;
    }

    indexOfValidation += 1;
    linecopy.remove(0, IndexOfComma + 1);
    //qDebug() << "typeTest:" << GYDATA.type;


    IndexOfComma = linecopy.indexOf(',');

    if (IndexOfComma == -1) {
        qDebug() << "Ошибка: индекс запятой не найден для ms";
        return;
    }

    ok = false;
    GYDATA.ms = linecopy.left(IndexOfComma).trimmed().toLongLong(&ok);

    if (!ok) {
        qDebug() << "Parse error: ms";
        return;
    }

    indexOfValidation += 1;
    linecopy.remove(0, IndexOfComma + 1);
    //qDebug() << "msTest:" << GYDATA.ms;


    IndexOfComma = linecopy.indexOf(',');

    if (IndexOfComma == -1) {
        qDebug() << "Ошибка: индекс запятой не найден для axMg";
        return;
    }

    ok = false;
    GYDATA.axMg = linecopy.left(IndexOfComma).trimmed().toDouble(&ok);

    if (!ok) {
        qDebug() << "Parse error: axMg";
        return;
    }

    indexOfValidation += 1;
    linecopy.remove(0, IndexOfComma + 1);
    //qDebug() << "axMgTest:" << GYDATA.axMg;


    IndexOfComma = linecopy.indexOf(',');

    if (IndexOfComma == -1) {
        qDebug() << "Ошибка: индекс запятой не найден для ayMg";
        return;
    }

    ok = false;
    GYDATA.ayMg = linecopy.left(IndexOfComma).trimmed().toDouble(&ok);

    if (!ok) {
        qDebug() << "Parse error: ayMg";
        return;
    }

    indexOfValidation += 1;
    linecopy.remove(0, IndexOfComma + 1);
    //qDebug() << "ayMgTest:" << GYDATA.ayMg;


    IndexOfComma = linecopy.indexOf(',');

    if (IndexOfComma == -1) {
        qDebug() << "Ошибка: индекс запятой не найден для azMg";
        return;
    }

    ok = false;
    GYDATA.azMg = linecopy.left(IndexOfComma).trimmed().toDouble(&ok);

    if (!ok) {
        qDebug() << "Parse error: azMg";
        return;
    }

    indexOfValidation += 1;
    linecopy.remove(0, IndexOfComma + 1);
    //qDebug() << "azMgTest:" << GYDATA.azMg;


    IndexOfComma = linecopy.indexOf(',');

    if (IndexOfComma == -1) {
        qDebug() << "Ошибка: индекс запятой не найден для gxDps";
        return;
    }

    ok = false;
    GYDATA.gxDps = linecopy.left(IndexOfComma).trimmed().toDouble(&ok);

    if (!ok) {
        qDebug() << "Parse error: gxDps";
        return;
    }

    indexOfValidation += 1;
    linecopy.remove(0, IndexOfComma + 1);
    //qDebug() << "gxDpsTest:" << GYDATA.gxDps;


    IndexOfComma = linecopy.indexOf(',');

    if (IndexOfComma == -1) {
        qDebug() << "Ошибка: индекс запятой не найден для gyDps";
        return;
    }

    ok = false;
    GYDATA.gyDps = linecopy.left(IndexOfComma).trimmed().toDouble(&ok);

    if (!ok) {
        qDebug() << "Parse error: gyDps";
        return;
    }

    indexOfValidation += 1;
    linecopy.remove(0, IndexOfComma + 1);
    //qDebug() << "gyDpsTest:" << GYDATA.gyDps;


    IndexOfComma = linecopy.indexOf(',');

    if (IndexOfComma == -1) {
        qDebug() << "Ошибка: индекс запятой не найден для gzDps";
        return;
    }

    ok = false;
    GYDATA.gzDps = linecopy.left(IndexOfComma).trimmed().toDouble(&ok);

    if (!ok) {
        qDebug() << "Parse error: gzDps";
        return;
    }

    indexOfValidation += 1;
    linecopy.remove(0, IndexOfComma + 1);
    //qDebug() << "gzDpsTest:" << GYDATA.gzDps;


    IndexOfComma = linecopy.indexOf(',');

    if (IndexOfComma == -1) {
        qDebug() << "Ошибка: индекс запятой не найден для mxUt";
        return;
    }

    ok = false;
    GYDATA.mxUt = linecopy.left(IndexOfComma).trimmed().toDouble(&ok);

    if (!ok) {
        qDebug() << "Parse error: mxUt";
        return;
    }

    indexOfValidation += 1;
    linecopy.remove(0, IndexOfComma + 1);
    //qDebug() << "mxUtTest:" << GYDATA.mxUt;


    IndexOfComma = linecopy.indexOf(',');

    if (IndexOfComma == -1) {
        qDebug() << "Ошибка: индекс запятой не найден для myUt";
        return;
    }

    ok = false;
    GYDATA.myUt = linecopy.left(IndexOfComma).trimmed().toDouble(&ok);

    if (!ok) {
        qDebug() << "Parse error: myUt";
        return;
    }

    indexOfValidation += 1;
    linecopy.remove(0, IndexOfComma + 1);
    //qDebug() << "myUtTest:" << GYDATA.myUt;


    IndexOfComma = linecopy.indexOf(',');

    if (IndexOfComma == -1) {
        qDebug() << "Ошибка: индекс запятой не найден для mzUt";
        return;
    }

    ok = false;
    GYDATA.mzUt = linecopy.left(IndexOfComma).trimmed().toDouble(&ok);

    if (!ok) {
        qDebug() << "Parse error: mzUt";
        return;
    }

    indexOfValidation += 1;
    linecopy.remove(0, IndexOfComma + 1);
    //qDebug() << "mzUtTest:" << GYDATA.mzUt;


    IndexOfComma = linecopy.indexOf(',');

    if (IndexOfComma == -1) {
        qDebug() << "Ошибка: индекс запятой не найден для tempC";
        return;
    }

    ok = false;
    GYDATA.tempC = linecopy.left(IndexOfComma).trimmed().toDouble(&ok);

    if (!ok) {
        qDebug() << "Parse error: tempC";
        return;
    }

    indexOfValidation += 1;
    linecopy.remove(0, IndexOfComma + 1);
    //qDebug() << "tempCTest:" << GYDATA.tempC;


    IndexOfComma = linecopy.indexOf(',');

    if (IndexOfComma == -1) {
        qDebug() << "Ошибка: индекс запятой не найден для rollAccDeg";
        return;
    }

    ok = false;
    GYDATA.rollAccDeg = linecopy.left(IndexOfComma).trimmed().toDouble(&ok);

    if (!ok) {
        qDebug() << "Parse error: rollAccDeg";
        return;
    }

    indexOfValidation += 1;
    linecopy.remove(0, IndexOfComma + 1);
    //qDebug() << "rollAccDegTest:" << GYDATA.rollAccDeg;


    if (linecopy.trimmed().isEmpty()) {
        qDebug() << "Ошибка: пустое поле pitchAccDeg";
        return;
    }

    ok = false;
    GYDATA.pitchAccDeg = linecopy.trimmed().toDouble(&ok);

    if (!ok) {
        qDebug() << "Parse error: pitchAccDeg";
        return;
    }

    indexOfValidation += 1;
    //qDebug() << "pitchAccDegTest:" << GYDATA.pitchAccDeg;


    if (indexOfValidation == 14) {
        GYDATASORAGE.type.push_back(GYDATA.type);
        GYDATASORAGE.ms.push_back(GYDATA.ms);

        GYDATASORAGE.axMg.push_back(GYDATA.axMg);
        GYDATASORAGE.ayMg.push_back(GYDATA.ayMg);
        GYDATASORAGE.azMg.push_back(GYDATA.azMg);

        GYDATASORAGE.gxDps.push_back(GYDATA.gxDps);
        GYDATASORAGE.gyDps.push_back(GYDATA.gyDps);
        GYDATASORAGE.gzDps.push_back(GYDATA.gzDps);

        GYDATASORAGE.mxUt.push_back(GYDATA.mxUt);
        GYDATASORAGE.myUt.push_back(GYDATA.myUt);
        GYDATASORAGE.mzUt.push_back(GYDATA.mzUt);

        GYDATASORAGE.tempC.push_back(GYDATA.tempC);

        GYDATASORAGE.rollAccDeg.push_back(GYDATA.rollAccDeg);
        GYDATASORAGE.pitchAccDeg.push_back(GYDATA.pitchAccDeg);
        emit rawDataReady(GYDATA);
        //qDebug() << "RAW parsed and saved:"
        //         << "ms =" << GYDATA.ms
        //         << "roll =" << GYDATA.rollAccDeg
        //        << "pitch =" << GYDATA.pitchAccDeg;
    } else {
        qDebug() << "Parse error: validation count =" << indexOfValidation;
    }
}

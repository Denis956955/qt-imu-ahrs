#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>
#include "serialdatareader.h"
#include "complementaryfilter.h"
#include "attitude.h"
#include "kalmanfilterquaternion.h"
#include <QPushButton>
#include "yaw.h"
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    auto* central = new QWidget(this);

    auto* layout = new QVBoxLayout(central);
    auto* calButtonOn = new QPushButton("Начать калибровку", central);
    yaw_ = new Yaw(central);
    horizon2_ = new Attitude(central);
    layout->addWidget(calButtonOn);
    layout->addWidget(yaw_);
    layout->addWidget(horizon2_);
    magCalibrator_ = new MagnetometerCalibrator(this);
    qDebug() << "Поставьте вертитально для каллибровки акселерометра и нажимте старт";
    gyrCalibrationTimer_.start();

    setCentralWidget(central);
    serialReader_ = new SerialDataReader(this);
    filter_ = new ComplementaryFilter(this);
    connect(calButtonOn, &QPushButton::clicked,
            this, &MainWindow::onAccelButtonClickedOn);
    connect(serialReader_, &SerialDataReader::rawDataReady,
            this, &MainWindow::handleRawData);
    filter2_ = new KalmanFilterQuaternion(this);
    connect(filter2_,&KalmanFilterQuaternion::PitchRollReady,this,&MainWindow::handleKalmanPitchAndRoll);
    connect(filter2_,&KalmanFilterQuaternion::YawReady,this,&MainWindow::handleKalmanYaw);
    serialReader_->openSerialPort();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::handleRawData(const GY_ICMData& data)
{
    switch (numberOfCalAcc) {
    case 1:
        if(numberOfGy == 0){
            qDebug() << "Начало каллибровки";}
        summurizeroOfData(data);
        if(numberOfGy >= ACC_CAL_SAMPLES){
        qDebug() << "Калибровка по данной стороне завершена на 100%";
        devision();
        filter2_->accelBiasCalc(numberOfCalAcc,Sum);
        numberOfCalAcc = 2;
        erizeSum();
        qDebug() << "Переверните плату и нажмите еще раз для продолжения";}
        return;
    case 3:
        if(numberOfGy == 0){
            qDebug() << "Идет каллибровка";}
        summurizeroOfData(data);
        if(numberOfGy >= ACC_CAL_SAMPLES){
        qDebug() << "Калибровка по данной стороне завершена на 100%";
        devision();
        filter2_->accelBiasCalc(numberOfCalAcc,Sum);
        numberOfCalAcc = 4;
        erizeSum();
        qDebug() << "Поставьте плату носом вниз и нажмите еще раз для продолжения";}
        return;
    case 5:
        if(numberOfGy == 0){
            qDebug() << "Начало каллибровки";}
        summurizeroOfData(data);
        if(numberOfGy >= ACC_CAL_SAMPLES){
            qDebug() << "Калибровка по данной стороне завершена на 100%";
            devision();
            filter2_->accelBiasCalc(numberOfCalAcc,Sum);
            numberOfCalAcc = 6;
            erizeSum();
            qDebug() << "Поставьте плату носом вверх и нажмите еще раз для продолжения";}
        return;
    case 7:
        if(numberOfGy == 0){
            qDebug() << "Начало каллибровки";}
        summurizeroOfData(data);
        if(numberOfGy >= ACC_CAL_SAMPLES){
            qDebug() << "Калибровка по данной стороне завершена на 100%";
            devision();
            filter2_->accelBiasCalc(numberOfCalAcc,Sum);
            numberOfCalAcc = 8;
            erizeSum();
            qDebug() << "Поставьте плату на правый бок и нажмите еще раз для продолжения";}
        return;
    case 9:
        if(numberOfGy == 0){
            qDebug() << "Начало каллибровки";}
        summurizeroOfData(data);
        if(numberOfGy >= ACC_CAL_SAMPLES){
            qDebug() << "Калибровка по данной стороне завершена на 100%";
            devision();
            filter2_->accelBiasCalc(numberOfCalAcc,Sum);
            numberOfCalAcc = 10;
            erizeSum();
            qDebug() << "Поставьте плату на левый бок и нажмите еще раз для продолжения";}
        return;
    case 11:
        if(numberOfGy == 0){
            qDebug() << "Начало каллибровки";}
        summurizeroOfData(data);
        if(numberOfGy >= ACC_CAL_SAMPLES){
            qDebug() << "Калибровка по данной стороне завершена на 100%";
            devision();
            filter2_->accelBiasCalc(numberOfCalAcc,Sum);
            numberOfCalAcc = 12;
            erizeSum();

            accCal = true;
            gyrPauseDone_ = false;

            lastGyrPauseSecond_ = -1;
            lastMagCalibrationSecondGyr_ = -1;

            gyrCalibrationTimer_.restart();

            qDebug() << "Калибровка акселерометра завершена";
            qDebug() << "Поставьте плату неподвижно. Через 5 секунд начнется калибровка гироскопа";}
        return;

    default:
        break;
    }
    if (accCal && !gyrPauseDone_) {
        int elapsedSec = static_cast<int>(gyrCalibrationTimer_.elapsed() / 1000);

        if (elapsedSec != lastGyrPauseSecond_) {
            lastGyrPauseSecond_ = elapsedSec;

            qDebug() << "Пауза перед калибровкой гироскопа. Не двигать плату:"
                     << elapsedSec
                     << "из"
                     << GYR_PAUSE_MS / 1000
                     << "секунд";
        }

        if (gyrCalibrationTimer_.elapsed() >= GYR_PAUSE_MS) {
            gyrPauseDone_ = true;

            lastMagCalibrationSecondGyr_ = -1;
            gyrCalibrationTimer_.restart();

            qDebug() << "Началась калибровка гироскопа. Не двигать плату";
        }

        return;
    }
    if (!gyrCalibrationReady_ && accCal && gyrPauseDone_) {
        int elapsedSec1 = static_cast<int>(gyrCalibrationTimer_.elapsed() / 1000);
        filter2_->getGyrDisp(data);
        if (elapsedSec1 != lastMagCalibrationSecondGyr_) {
            lastMagCalibrationSecondGyr_ = elapsedSec1;

            qDebug() << "Не двигать плату. Идет калибровка гироскопа:"
                     << elapsedSec1
                     << "из"
                     << GYR_CAL_MS / 1000
                     << "секунд";
        }

        if (gyrCalibrationTimer_.elapsed() >= GYR_CAL_MS) {
            gyrCalibrationReady_ = true;
            filter2_->firstGyrrDisp();
            qDebug() << "Калибровка гироскопа завершена";
            magCalibrationTimer_.start();
        }

        return;
    }
    if (!magCalibrationReady_ && accCal) {

       // magCalibrator_->MinMaxMagFinder(data);

        // int elapsedSec = static_cast<int>(magCalibrationTimer_.elapsed() / 1000);

        // if (elapsedSec != lastMagCalibrationSecond_) {
        //     lastMagCalibrationSecond_ = elapsedSec;

        //     qDebug() << "Калибруйся. Идет калибровка магнитометра:"
        //              << elapsedSec
        //              << "из 30 секунд";
        // }

        // if (magCalibrationTimer_.elapsed() >= 30000) {
        //     magCalibrator_->MagCal();
        //     magCalibrationReady_ = true;

        //     qDebug() << "Калибровка магнитометра завершена";
        // }
        magCalibrationReady_ = true;
        return;
    }
    if(magCalibrationReady_ && accCal && gyrCalibrationReady_){
    GY_ICMData datagive = magCalibrator_->GiveCalData(data);

    filter_->filtration(datagive);
    filter2_->filtration(datagive);}
}


void MainWindow::handleKalmanPitchAndRoll(double pitch, double roll)
{
    horizon2_->setAttitude(pitch, roll);
}
void MainWindow::handleKalmanYaw(double Yaw)
{
    yaw_->setAttitude(Yaw);
}
void MainWindow::onAccelButtonClickedOn(){

    if (numberOfCalAcc == 0 ||
        numberOfCalAcc == 2 ||
        numberOfCalAcc == 4 ||
        numberOfCalAcc == 6 ||
        numberOfCalAcc == 8 ||
        numberOfCalAcc == 10) {

        erizeSum();
        numberOfCalAcc += 1;

        qDebug() << "Начат сбор данных для состояния:" << numberOfCalAcc;
        return;
    }

    if (numberOfCalAcc == 1 ||
        numberOfCalAcc == 3 ||
        numberOfCalAcc == 5 ||
        numberOfCalAcc == 7 ||
        numberOfCalAcc == 9 ||
        numberOfCalAcc == 11) {

        qDebug() << "Сейчас идет сбор данных, кнопку пока нажимать не надо";
        return;
    }

    if (numberOfCalAcc == 12) {
        qDebug() << "Калибровка акселерометра уже завершена";
        return;
    }
};
void MainWindow::summurizeroOfData(const GY_ICMData& data){
    Sum.x = Sum.x + data.axMg;
    Sum.y = Sum.y + data.ayMg;
    Sum.z = Sum.z + data.azMg;
    numberOfGy += 1;
    if(numberOfGy == 100){
        qDebug() << "Калибровка по данной стороне завершена на 20%";
    }
    if(numberOfGy == 200){
        qDebug() << "Калибровка по данной стороне завершена на 40%";
    }
    if(numberOfGy == 400){
        qDebug() << "Калибровка по данной стороне завершена на 80%";
    }

}
void MainWindow::devision()
{
    if (numberOfGy <= 0) {
        qDebug() << "Ошибка: devision() вызвана без накопленных данных";
        return;
    }

    Sum.x = Sum.x / numberOfGy;
    Sum.y = Sum.y / numberOfGy;
    Sum.z = Sum.z / numberOfGy;
}
void MainWindow::erizeSum(){
    Sum.x = 0;
    Sum.y = 0;
    Sum.z = 0;
    numberOfGy = 0;

}

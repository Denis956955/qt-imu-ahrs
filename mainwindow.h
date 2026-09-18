#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "GY_ICMData.h"
#include <QElapsedTimer>
#include <QElapsedTimer>
#include "magnetometercalibrator.h"
#include <QPushButton>
#include <QLabel>
#include <QElapsedTimer>
#include <array>
#include "AccDataForCalibartion.h"
#include "yaw.h"
class KalmanFilterQuaternion;
class SerialDataReader;
class ComplementaryFilter;
class Attitude;

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void summurizeroOfData(const GY_ICMData& data);
    void devision();
    void erizeSum();


private slots:
    void handleRawData(const GY_ICMData& data);
    void handleKalmanPitchAndRoll(double pitch, double roll);
    void onAccelButtonClickedOn();
    void handleKalmanYaw(double Yaw);

private:
    Ui::MainWindow *ui;

    SerialDataReader* serialReader_ = nullptr;
    ComplementaryFilter* filter_ = nullptr;
    KalmanFilterQuaternion* filter2_ = nullptr;
    Yaw* yaw_ = nullptr;
    Attitude* horizon2_ = nullptr;
    bool checkMagMatrix = 0;
    bool magCalibrationReady_ = false;
    bool gyrCalibrationReady_ = false;
    QElapsedTimer magCalibrationTimer_;
    QElapsedTimer gyrCalibrationTimer_;

    MagnetometerCalibrator* magCalibrator_ = nullptr;
    int lastMagCalibrationSecond_ = -1;
    int lastMagCalibrationSecondGyr_ = -1;
    int numberOfCalAcc = 0;
    bool accCal = false;
    AccDataForCalibartion Sum;
    int numberOfGy = 0;
    const int ACC_CAL_SAMPLES = 200;
    bool gyrPauseDone_ = false;
    static constexpr int GYR_PAUSE_MS = 5000;
    static constexpr int GYR_CAL_MS   = 5000;
    int lastGyrPauseSecond_ = -1;
};

#endif // MAINWINDOW_H

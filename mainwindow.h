#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSettings>
#include <QtDebug>
#include <QTimer>
#include <QDateTime>
#include <direct.h>

#include "juniservo.h"
#include "robotarm.h"
#include "logger.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void ReadSettings();
    void WriteSettings();

public slots:
    void btnConnectClicked();
    void btnRunClicked();
    void btnReadyClicked();

    void updateTimeout();

    void on_cbJMode_stateChanged(int arg1);
    void on_cbCMode_stateChanged(int arg1);
    void on_cbRectMode_stateChanged(int arg1);

private:
    Ui::MainWindow *ui;

    JuniServo *servo;

    bool connectState;

    QTimer *updateTimer;

    uint offset1 = 1850;
    uint offset2 = 837;
    uint offset3 = 2513;

    uint curPos1, curPos2, curPos3;

    RobotArm *robot;

    double desX, desY, desZ, curX, curY, curZ;

    bool goalReach, rectMode;
    uint indx;

    Logger *logger;
};

const double DEG2RAD = M_PI/180.0;
const double RAD2DEG = 180.0/M_PI;
const uint maxIndx = 4;
const double path_rect[maxIndx][3] = {
//    {-0.75, 250.37, -15.2963},
    {-175,  180.37, -15.2963},
    {-175,  180.37,  75.2963},
    { 175,  180.37,  75.2963},
    { 175,  180.37, -15.2963},
//    {-0.75, 250.37, -15.2963},
};

#endif // MAINWINDOW_H

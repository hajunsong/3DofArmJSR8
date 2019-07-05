#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSettings>
#include <QtDebug>
#include <QTimer>

#include "juniservo.h"
#include "robotarm.h"

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
};

const double DEG2RAD = M_PI/180.0;
const double RAD2DEG = 180.0/M_PI;

#endif // MAINWINDOW_H

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSettings>
#include <QtDebug>
#include <QTimer>
#include "juniservo.h"

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
    void setVelTor(int arg);
    void updateTimeout();

private:
    Ui::MainWindow *ui;

    JuniServo *servo;

    bool connectState;

    QTimer *updateTimer;
};

#endif // MAINWINDOW_H

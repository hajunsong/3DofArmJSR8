#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QList<QString> portItems = {"COM1","COM2","COM3","COM4","COM5"};
    QList<QString> baudItems = {"9600","14400","19200","38400","57600","115200","229800","459700","930200"};
    ui->comboPort->addItems(portItems);
    ui->comboBaud->addItems(baudItems);

    ReadSettings();
}

MainWindow::~MainWindow()
{
    WriteSettings();
    delete ui;
}

void MainWindow::ReadSettings()
{
    QSettings settings("3DofArmJSR8");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    int indexPort = settings.value("Port").toInt();
    int indexBaud = settings.value("Baud").toInt();
    QString Velocity1 = settings.value("Velocity1").toString();
    QString Torque1 = settings.value("Torque1").toString();
    QString Velocity2 = settings.value("Velocity2").toString();
    QString Torque2 = settings.value("Torque2").toString();
    QString Velocity3 = settings.value("Velocity3").toString();
    QString Torque3 = settings.value("Torque3").toString();
    ui->comboPort->setCurrentIndex(indexPort);
    ui->comboBaud->setCurrentIndex(indexBaud);
    ui->txtVel1->setText(Velocity1);
    ui->txtTor1->setText(Torque1);
    ui->txtVel2->setText(Velocity2);
    ui->txtTor2->setText(Torque2);
    ui->txtVel3->setText(Velocity3);
    ui->txtTor3->setText(Torque3);
}

void MainWindow::WriteSettings()
{
    QSettings settings("3DofArmJSR8");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("Port",ui->comboPort->currentIndex());
    settings.setValue("Baud",ui->comboBaud->currentIndex());
    settings.setValue("Velocity1", ui->txtVel1->text());
    settings.setValue("Torque1", ui->txtTor1->text());
    settings.setValue("Velocity2", ui->txtVel2->text());
    settings.setValue("Torque2", ui->txtTor2->text());
    settings.setValue("Velocity3", ui->txtVel3->text());
    settings.setValue("Torque3", ui->txtTor3->text());
}

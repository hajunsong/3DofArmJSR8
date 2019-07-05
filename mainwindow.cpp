#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QList<QString> portItems = {"COM1","COM2","COM3","COM4","COM5"};
    QList<QString> baudItems = {"9600","14400","19200","38400","57600","115200","229800","459700","930200"};
    ui->comboPort->addItems(portItems);
    ui->comboBaud->addItems(baudItems);

    servo = new JuniServo();
    connectState = false;

    connect(ui->btnConnect, SIGNAL(clicked()), this, SLOT(btnConnectClicked()));
    connect(ui->btnRun, SIGNAL(clicked()), this, SLOT(btnRunClicked()));

    ReadSettings();

    updateTimer = new QTimer(this);
    updateTimer->setInterval(10);
    connect(updateTimer, SIGNAL(timeout()), this, SLOT(updateTimeout()));
}

MainWindow::~MainWindow()
{
    WriteSettings();
    delete ui;
    delete servo;
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
    ui->txtVel_1->setText(Velocity1);
    ui->txtTor_1->setText(Torque1);
    ui->txtVel_2->setText(Velocity2);
    ui->txtTor_2->setText(Torque2);
    ui->txtVel_3->setText(Velocity3);
    ui->txtTor_3->setText(Torque3);
}

void MainWindow::WriteSettings()
{
    QSettings settings("3DofArmJSR8");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("Port",ui->comboPort->currentIndex());
    settings.setValue("Baud",ui->comboBaud->currentIndex());
    settings.setValue("Velocity1", ui->txtVel_1->text());
    settings.setValue("Torque1", ui->txtTor_1->text());
    settings.setValue("Velocity2", ui->txtVel_2->text());
    settings.setValue("Torque2", ui->txtTor_2->text());
    settings.setValue("Velocity3", ui->txtVel_3->text());
    settings.setValue("Torque3", ui->txtTor_3->text());
}

void MainWindow::btnConnectClicked()
{
    if (connectState){
        servo->disconnect();
        connectState = false;
    }
    else{
        connectState = servo->connect(ui->comboPort->currentText(), ui->comboBaud->currentText());
        if (connectState) ReadSettings();
    }
    ui->btnConnect->setText(connectState ? "Disconnect" : "Connect");
}

void MainWindow::btnRunClicked()
{
    if (connectState){
        if (ui->btnRun->text().compare("Run")){
            uint vel1 = ui->txtVel_1->text().toUInt();
            uint vel2 = ui->txtVel_2->text().toUInt();
            uint vel3 = ui->txtVel_3->text().toUInt();
            uint tor1 = ui->txtTor_1->text().toUInt();
            uint tor2 = ui->txtTor_2->text().toUInt();
            uint tor3 = ui->txtTor_3->text().toUInt();
            servo->writeGroupNewVelocity(vel1, vel2, vel3);
            servo->writeGroupNewTorque(tor1, tor2, tor3);
            ui->btnRun->setText("Stop");
            updateTimer->start();
        }
        else{
            servo->writeGroupNewVelocity(0, 0, 0);
            servo->writeGroupNewTorque(0, 0, 0);
            ui->btnRun->setText("Run");
            updateTimer->stop();
        }
    }
}

void MainWindow::setVelTor(int arg)
{
    QString name = sender()->objectName();
    QString target = name.split("_").at(0);
    uint id = name.split("_").at(1).toUInt();

    if (connectState){
        if (target.contains("vel", Qt::CaseInsensitive)){
            servo->writeNewVelocity(id, static_cast<uint>(arg));
        }
        else if (target.contains("tor", Qt::CaseInsensitive)){
            servo->writeNewTorque(id, static_cast<uint>(arg));
        }
    }
}

void MainWindow::updateTimeout()
{

}

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
    connect(ui->btnReady, SIGNAL(clicked()), this, SLOT(btnReadyClicked()));

    ReadSettings();

    updateTimer = new QTimer(this);
    updateTimer->setInterval(100);
    connect(updateTimer, SIGNAL(timeout()), this, SLOT(updateTimeout()));

    ui->spJoint1->setRange(-360, 360);
    ui->spJoint2->setRange(-360, 360);
    ui->spJoint3->setRange(-360, 360);

    ui->spJoint1Cmd->setRange(-360, 360);
    ui->spJoint2Cmd->setRange(-360, 360);
    ui->spJoint3Cmd->setRange(-360, 360);

    ui->spEndX->setRange(-999, 999);
    ui->spEndY->setRange(-999, 999);
    ui->spEndZ->setRange(-999, 999);

    ui->spEndXCmd->setRange(-999, 999);
    ui->spEndYCmd->setRange(-999, 999);
    ui->spEndZCmd->setRange(-999, 999);

    ui->btnRun->setDisabled(1);

    ui->cbJMode->setChecked(1);

    robot = new RobotArm(3,3);
}

MainWindow::~MainWindow()
{
    WriteSettings();
    delete ui;
    delete servo;
    delete robot;
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
    }
    ui->btnConnect->setText(connectState ? "Disconnect" : "Connect");
}

void MainWindow::btnRunClicked()
{
    if (ui->cbJMode->isChecked()){
        uint cmdPos1 = 0, cmdPos2 = 0, cmdPos3 = 0;

        if (ui->spJoint1Cmd->value() > 0){
            cmdPos1 = curPos1 - static_cast<uint>(ui->spJoint1Cmd->value()*DEG2ENC);
        }
        else{
            cmdPos1 = curPos1 + static_cast<uint>(abs(ui->spJoint1Cmd->value())*DEG2ENC);
        }

        if (ui->spJoint2Cmd->value() > 0){
            cmdPos2 = curPos2 - static_cast<uint>(ui->spJoint2Cmd->value()*DEG2ENC);
        }
        else{
            cmdPos2 = curPos2 + static_cast<uint>(abs(ui->spJoint2Cmd->value())*DEG2ENC);
        }

        if (ui->spJoint3Cmd->value() > 0){
            cmdPos3 = curPos3 + static_cast<uint>(ui->spJoint3Cmd->value()*DEG2ENC);
        }
        else{
            cmdPos3 = curPos3 - static_cast<uint>(abs(ui->spJoint3Cmd->value())*DEG2ENC);
        }

        servo->writeGroupNewPosition(cmdPos1, cmdPos2, cmdPos3);

        ui->spJoint1Cmd->setValue(0);
        ui->spJoint2Cmd->setValue(0);
        ui->spJoint3Cmd->setValue(0);
    }

    if (ui->cbCMode->isChecked()){
        double des_pos[3], delta_pos[3], q[3];
        delta_pos[0] = ui->spEndXCmd->value();
        delta_pos[1] = ui->spEndYCmd->value();
        delta_pos[2] = ui->spEndZCmd->value();
        des_pos[0] = (delta_pos[0] + ui->spEndX->value())*0.001;
        des_pos[1] = (delta_pos[1] + ui->spEndY->value())*0.001;
        des_pos[2] = (delta_pos[2] + ui->spEndZ->value())*0.001;

        robot->run_inverse_kinematics(des_pos, q);

        double delta_q[3];
        delta_q[0] = q[0]*RAD2DEG - ui->spJoint1->value();
        delta_q[1] = q[1]*RAD2DEG - ui->spJoint2->value();
        delta_q[2] = q[2]*RAD2DEG - ui->spJoint3->value();

        cout << "delta_q : " << delta_q[0] << ", " << delta_q[1] << ", " << delta_q[2] << endl;

        uint cmdPos1 = 0, cmdPos2 = 0, cmdPos3 = 0;
        if (delta_q[0] > 0){
            cmdPos1 = curPos1 - static_cast<uint>(delta_q[0]*DEG2ENC);
        }
        else{
            cmdPos1 = curPos1 + static_cast<uint>(abs(delta_q[0])*DEG2ENC);
        }

        if (delta_q[1] > 0){
            cmdPos2 = curPos2 - static_cast<uint>(delta_q[1]*DEG2ENC);
        }
        else{
            cmdPos2 = curPos2 + static_cast<uint>(abs(delta_q[1])*DEG2ENC);
        }

        if (delta_q[2] > 0){
            cmdPos3 = curPos3 + static_cast<uint>(delta_q[2]*DEG2ENC);
        }
        else{
            cmdPos3 = curPos3 - static_cast<uint>(abs(delta_q[2])*DEG2ENC);
        }
        cout << "cmdPos : " << cmdPos1 << ", " << cmdPos2 << ", " << cmdPos3 << endl;

        servo->writeGroupNewPosition(cmdPos1, cmdPos2, cmdPos3);

        ui->spEndXCmd->setValue(0);
        ui->spEndYCmd->setValue(0);
        ui->spEndZCmd->setValue(0);
    }
}

void MainWindow::updateTimeout()
{
    ui->rbComState->toggle();
    curPos1 = servo->readPosition(1);
    curPos2 = servo->readPosition(2);
    curPos3 = servo->readPosition(3);
    ui->spJoint1->setValue(static_cast<int>(curPos1 - offset1)*ENC2DEG*(-1));
    ui->spJoint2->setValue(static_cast<int>(curPos2 - offset2)*ENC2DEG*(-1) + 15);
    ui->spJoint3->setValue(static_cast<int>(curPos3 - offset3)*ENC2DEG + 120);

    double q[3], end[3];
    q[0] = ui->spJoint1->value()*DEG2RAD;
    q[1] = ui->spJoint2->value()*DEG2RAD;
    q[2] = ui->spJoint3->value()*DEG2RAD;

    robot->run_kinematics(q, end);
    ui->spEndX->setValue(end[0]*1000);
    ui->spEndY->setValue(end[1]*1000);
    ui->spEndZ->setValue(end[2]*1000);
}

void MainWindow::btnReadyClicked(){
    if (connectState){
        if (!ui->btnReady->text().compare("Ready")){
            ui->btnReady->setText("Stop");
            updateTimer->start();

            uint vel1 = ui->txtVel_1->text().toUInt();
            uint vel2 = ui->txtVel_2->text().toUInt();
            uint vel3 = ui->txtVel_3->text().toUInt();
            uint tor1 = ui->txtTor_1->text().toUInt();
            uint tor2 = ui->txtTor_2->text().toUInt();
            uint tor3 = ui->txtTor_3->text().toUInt();
            servo->writeGroupNewVelocity(vel1, vel2, vel3);
            servo->writeGroupNewTorque(tor1, tor2, tor3);

            servo->writeGroupNewPosition(offset1, offset2, offset3);
            ui->btnRun->setEnabled(1);
        }
        else{
            servo->writeGroupNewVelocity(0, 0, 0);
            servo->writeGroupNewTorque(0, 0, 0);
            ui->btnReady->setText("Ready");
            updateTimer->stop();
            ui->btnRun->setDisabled(1);
        }
    }
}
